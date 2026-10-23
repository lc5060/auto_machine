#ifndef AUTO_MACHINE_FAULT_REPORTER_HPP
#define AUTO_MACHINE_FAULT_REPORTER_HPP

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <auto_machine/msg/device_fault.hpp>

#include "auto_machine/auto_machine_common.hpp"

namespace auto_machine {

/**
 * @brief 故障主动上报模块（多设备监控版）
 *
 * 独立于命令分发逻辑：内部用一个后台线程周期轮询各被监控设备的故障寄存器，
 * 边沿触发（状态变化才发）发布到 /auto_machine/fault 主题。
 *
 * 与旧版单设备监控的区别：
 *   - 按 (dev_type, device_index) 作为监控单元，每台设备独立串口、独立边沿状态，
 *     互不串线（多机场景下两台同型机各自独立上报）。
 *   - 发布消息带 device_index 字段，上层据此区分同一类设备的不同物理机器。
 *
 * 用法：
 *   FaultReporter reporter(node, modbus_pool);
 *   reporter.enable(DEV_MAG06, 0);   // 开始监控 0 号 M6
 *   reporter.enable(DEV_MAG06, 1);   // 开始监控 1 号 M6
 *   reporter.disable(DEV_MAG06, 1);  // 停止监控 1 号 M6
 */
class FaultReporter {
public:
  FaultReporter(rclcpp::Node* node,
                const std::unordered_map<uint32_t, ModbusPool>& modbus_pool);
  ~FaultReporter();

  // 禁止拷贝
  FaultReporter(const FaultReporter&) = delete;
  FaultReporter& operator=(const FaultReporter&) = delete;

  /**
   * @brief 开启对指定设备（含设备索引）的故障监控
   * @param dev_type     设备类型（AutoMachineDef::DEV_*）
   * @param device_index 设备索引（0/1...；非多机设备恒为 0）
   * @return err_code::OK 成功；err_code::ERR_UNKNOWN_DEV_TYPE 未知设备；
   *         err_code::ERR_INVALID_PARAM 设备索引越界（无对应串口）
   */
  int32_t enable(uint32_t dev_type, uint8_t device_index);

  /**
   * @brief 关闭对指定设备（含设备索引）的故障监控
   */
  int32_t disable(uint32_t dev_type, uint8_t device_index);

private:
  /// 故障轮询的设备探测表项
  struct FaultProbe {
    uint32_t dev_type;   // 设备类型（AutoMachineDef::DEV_*）
    uint8_t slave_addr;  // Modbus 从站地址
    bool is_input_reg;   // true=0x04 读输入寄存器 / false=0x03 读保持寄存器
    uint16_t reg_addr;   // 故障/状态寄存器地址
    bool is_xyzr;        // XYZR 特殊判定（值==MASTER_FAULT 才算故障）
    bool check_reset_state;   // true=额外检查复位/准入状态寄存器，值为0时补报故障1
                              // （急停等场景故障码寄存器不置位，但准入状态会回到 0）
    uint16_t reset_state_reg; // 复位/准入状态寄存器地址（check_reset_state=true 时有效）
  };

  /// 监控单元标识：(dev_type, device_index)
  struct MonitorKey {
    uint32_t dev_type = 0;
    uint8_t device_index = 0;

    bool operator<(const MonitorKey& o) const {
      if (dev_type != o.dev_type) return dev_type < o.dev_type;
      return device_index < o.device_index;
    }
    bool operator==(const MonitorKey& o) const {
      return dev_type == o.dev_type && device_index == o.device_index;
    }
  };

  static const std::vector<FaultProbe>& fault_probes();
  const FaultProbe* find_fault_probe(uint32_t dev_type) const;
  std::shared_ptr<ModbusRtu> find_modbus(uint32_t dev_type,
                                         uint8_t device_index) const;

  void start_poll_thread();
  void stop_poll_thread();
  void fault_poll_loop();
  void poll_once();
  void publish_fault(uint32_t dev, uint8_t device_index, uint32_t code,
                     bool comm_fault);

  /// 单个监控项的边沿触发状态
  struct FaultState {
    bool last_comm_fault = false;
    uint16_t last_fault_code = 0;
    bool initialized = false;
  };

  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  /// 设备类型 -> 串口实例池（与节点共用，索引按 mag_serial_ports 顺序）
  std::unordered_map<uint32_t, ModbusPool> modbus_pool_;
  rclcpp::Publisher<msg::DeviceFault>::SharedPtr fault_pub_;

  std::thread fault_thread_;
  std::mutex thread_mutex_;
  std::mutex cv_mutex_;
  std::condition_variable cv_;
  std::atomic<bool> running_{false};

  std::mutex monitor_mutex_;               // 保护 monitored_ / states_
  std::set<MonitorKey> monitored_;         // 当前被监控的设备集合
  std::map<MonitorKey, FaultState> states_;  // 每个监控项独立的边沿状态

  int fault_poll_interval_ms_ = 1000;
};

}  // namespace auto_machine

#endif  // AUTO_MACHINE_FAULT_REPORTER_HPP
