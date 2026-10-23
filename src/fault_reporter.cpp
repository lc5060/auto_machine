#include "auto_machine/fault_reporter.hpp"

#include <chrono>

#include "auto_machine/auto_machine_common.hpp"
#include "auto_machine/auto_machine_mag04.hpp"
#include "auto_machine/auto_machine_mag06.hpp"
#include "auto_machine/auto_machine_t1.hpp"
#include "auto_machine/auto_machine_v6.hpp"
#include "auto_machine/auto_machine_xyzr.hpp"
#include "auto_machine/modbus_rtu.hpp"

namespace auto_machine {

FaultReporter::FaultReporter(
    rclcpp::Node* node,
    const std::unordered_map<uint32_t, ModbusPool>& modbus_pool)
    : node_(node),
      logger_(node->get_logger()),
      modbus_pool_(modbus_pool) {
  fault_poll_interval_ms_ =
      node_->declare_parameter<int>("fault_poll_interval_ms", 1000);
  fault_pub_ =
      node_->create_publisher<msg::DeviceFault>("auto_machine/fault", 10);
}

FaultReporter::~FaultReporter() { stop_poll_thread(); }

int32_t FaultReporter::enable(uint32_t dev_type, uint8_t device_index) {
  if (!find_fault_probe(dev_type)) {
    RCLCPP_WARN(logger_, "故障上报：未知设备类型 0x%08X", dev_type);
    return err_code::ERR_UNKNOWN_DEV_TYPE;
  }
  if (!find_modbus(dev_type, device_index)) {
    RCLCPP_WARN(logger_,
                "故障上报：设备索引越界 dev=0x%08X index=%u（无对应串口）",
                dev_type, device_index);
    return err_code::ERR_INVALID_PARAM;
  }

  {
    std::lock_guard<std::mutex> lock(monitor_mutex_);
    MonitorKey key;
    key.dev_type = dev_type;
    key.device_index = device_index;
    monitored_.insert(key);
    // 首次插入默认状态（initialized=false），下一轮轮询会先发一条当前状态作为基准
    if (states_.find(key) == states_.end()) {
      states_[key] = FaultState();
    }
  }

  start_poll_thread();
  RCLCPP_INFO(logger_, "故障上报已开启：dev=0x%08X index=%u", dev_type,
              device_index);
  return err_code::OK;
}

int32_t FaultReporter::disable(uint32_t dev_type, uint8_t device_index) {
  {
    std::lock_guard<std::mutex> lock(monitor_mutex_);
    MonitorKey key;
    key.dev_type = dev_type;
    key.device_index = device_index;
    monitored_.erase(key);
    states_.erase(key);
  }
  RCLCPP_INFO(logger_, "故障上报已关闭：dev=0x%08X index=%u", dev_type,
              device_index);
  return err_code::OK;
}

const std::vector<FaultReporter::FaultProbe>& FaultReporter::fault_probes() {
  static const std::vector<FaultProbe> probes = {
      {AutoMachineDef::DEV_T1, T1_MODBUS_ADDR, false, t1_reg::FAULT, false,
       false, 0},
      {AutoMachineDef::DEV_V6, V6_MODBUS_ADDR, true, v6_status::ERR, false,
       false, 0},
      {AutoMachineDef::DEV_MAG04, MAG04_MODBUS_ADDR, false,
       mag04_reg::ERROR_CODE, false, true, mag04_reg::ADMISSION_STATUS},
      {AutoMachineDef::DEV_MAG06, MAG06_MODBUS_ADDR, false,
       mag06_reg::ERROR_CODE, false, true, mag06_reg::ADMISSION_STATUS},
      {AutoMachineDef::DEV_XYZR, XYZR_MODBUS_ADDR, true,
       xyzr_status::MASTER_STATUS, true, false, 0},
  };
  return probes;
}

const FaultReporter::FaultProbe* FaultReporter::find_fault_probe(
    uint32_t dev_type) const {
  for (const auto& p : fault_probes()) {
    if (p.dev_type == dev_type) return &p;
  }
  return nullptr;
}

std::shared_ptr<ModbusRtu> FaultReporter::find_modbus(
    uint32_t dev_type, uint8_t device_index) const {
  auto it = modbus_pool_.find(dev_type);
  if (it == modbus_pool_.end() || device_index >= it->second.size()) {
    return nullptr;
  }
  return it->second[device_index];
}

void FaultReporter::start_poll_thread() {
  std::lock_guard<std::mutex> lock(thread_mutex_);
  if (running_) return;
  running_ = true;
  fault_thread_ = std::thread(&FaultReporter::fault_poll_loop, this);
}

void FaultReporter::stop_poll_thread() {
  std::lock_guard<std::mutex> lock(thread_mutex_);
  if (!running_) return;
  running_ = false;
  cv_.notify_all();
  if (fault_thread_.joinable()) {
    fault_thread_.join();
  }
}

void FaultReporter::fault_poll_loop() {
  while (running_ && rclcpp::ok()) {
    poll_once();
    std::unique_lock<std::mutex> lk(cv_mutex_);
    cv_.wait_for(lk, std::chrono::milliseconds(fault_poll_interval_ms_),
                 [this] { return !running_.load(); });
  }
}

void FaultReporter::poll_once() {
  // 拷贝一份当前监控集合，避免串口轮询期间长期持锁阻塞 enable/disable
  std::vector<MonitorKey> keys;
  {
    std::lock_guard<std::mutex> lock(monitor_mutex_);
    keys.reserve(monitored_.size());
    for (const auto& k : monitored_) keys.push_back(k);
  }

  for (const auto& key : keys) {
    const uint32_t dev = key.dev_type;
    const uint8_t idx = key.device_index;

    const FaultProbe* probe = find_fault_probe(dev);
    if (!probe) continue;
    auto modbus = find_modbus(dev, idx);
    if (!modbus) continue;

    std::vector<uint16_t> regs;
    const bool ok = probe->is_input_reg
                        ? modbus->readInputRegisters(probe->slave_addr,
                                                     probe->reg_addr, 1, regs)
                        : modbus->readHoldingRegisters(probe->slave_addr,
                                                       probe->reg_addr, 1,
                                                       regs);
    const uint16_t code = regs.empty() ? 0 : regs[0];

    // 额外读取复位/准入状态寄存器（急停等场景故障码寄存器不置位，但准入状态会回到 0）
    bool reset_unset = false;
    if (probe->check_reset_state) {
      std::vector<uint16_t> reset_regs;
      if (modbus->readHoldingRegisters(probe->slave_addr,
                                       probe->reset_state_reg, 1,
                                       reset_regs) &&
          !reset_regs.empty()) {
        reset_unset = (reset_regs[0] == 0);
      }
    }

    // 取该监控项的边沿状态
    FaultState st;
    {
      std::lock_guard<std::mutex> lock(monitor_mutex_);
      auto it = states_.find(key);
      if (it != states_.end()) {
        st = it->second;
      }
    }

    bool changed = false;

    if (!ok) {
      // 通讯故障（读不到设备/掉线）
      if (!st.initialized || !st.last_comm_fault) {
        publish_fault(dev, idx, 0, true);
        st.last_comm_fault = true;
        st.last_fault_code = 0;
        st.initialized = true;
        changed = true;
      }
    } else {
      // 汇总故障码：主故障寄存器优先；否则若启用了复位状态检查且未复位，补报故障 1
      uint16_t fault_code = 0;
      if (probe->is_xyzr) {
        fault_code = (code == xyzr_val::MASTER_FAULT) ? code : 0;
      } else if (code != 0) {
        fault_code = code;
      } else if (probe->check_reset_state && reset_unset) {
        fault_code = 1;
      }

      if (fault_code != 0) {
        if (!st.initialized || st.last_comm_fault ||
            st.last_fault_code != fault_code) {
          publish_fault(dev, idx, fault_code, false);
          st.last_comm_fault = false;
          st.last_fault_code = fault_code;
          st.initialized = true;
          changed = true;
        }
      } else {
        // 无故障（含故障恢复）
        if (!st.initialized || st.last_comm_fault ||
            st.last_fault_code != 0) {
          publish_fault(dev, idx, 0, false);
          st.last_comm_fault = false;
          st.last_fault_code = 0;
          st.initialized = true;
          changed = true;
        }
      }
    }

    if (changed) {
      std::lock_guard<std::mutex> lock(monitor_mutex_);
      states_[key] = st;
    }
  }
}

void FaultReporter::publish_fault(uint32_t dev, uint8_t device_index,
                                  uint32_t code, bool comm_fault) {
  msg::DeviceFault msg;
  msg.header.stamp = node_->now();
  msg.dev_type = dev;
  msg.device_index = device_index;
  msg.fault_code = code;
  msg.comm_fault = comm_fault;
  fault_pub_->publish(msg);
  if (comm_fault || code != 0) {
    RCLCPP_WARN(logger_,
                "故障上报: dev=0x%08X index=%u code=0x%04X comm_fault=%d", dev,
                device_index, code, comm_fault);
  } else {
    RCLCPP_INFO(logger_, "故障恢复: dev=0x%08X index=%u", dev, device_index);
  }
}

}  // namespace auto_machine
