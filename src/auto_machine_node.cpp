#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <auto_machine/srv/auto_machine.hpp>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "auto_machine/auto_machine_common.hpp"
#include "auto_machine/auto_machine_mag04.hpp"
#include "auto_machine/auto_machine_mag06.hpp"
#include "auto_machine/auto_machine_t1.hpp"
#include "auto_machine/auto_machine_v6.hpp"
#include "auto_machine/auto_machine_xyzr.hpp"
#include "auto_machine/closed_loop.hpp"
#include "auto_machine/fault_reporter.hpp"
#include "auto_machine/modbus_rtu.hpp"

using AutoMachineSrv = auto_machine::srv::AutoMachine;
using auto_machine::AutoMachineDef;  // msg 常量定义所在结构

/**
 * @brief 自动机服务节点（多机多串口版）
 *
 * 接收 AutoMachine.srv 格式的请求，根据 dev_type 分发到对应设备协议模块
 * （T1/V6/MAG04/MAG06/XYZR），各模块通过 RS485 Modbus RTU 与设备通讯。
 *
 * 多机能力（T1 / V6 / MAG04 / MAG06 / XYZR 全部支持）：
 *   - 每台自动机一条 RS485 总线：同一串口上同时挂自动机（T1/V6/MAG，
 *     从站 0x01）和 XYZR 对齐装置（从站 0x02），靠从站地址区分，互不影响。
 *   - 请求的 param_data[0] 作为设备索引（0 → 串口0，1 → 串口1 ...），
 *     所有设备类型都按该索引选择对应串口实例。
 *   - 索引策略：MAG04/MAG06 必须显式提供；T1/V6/XYZR 省略时默认 0
 *     （兼容单机旧调用）。
 *   - 每个串口一个独立 ModbusRtu 实例（各持独立总线锁），总线物理隔离、
 *     互不串线；同一索引下自动机与 XYZR 共享该实例。
 *   - 闭环状态轮询与故障主动上报也按同一设备索引走对应串口。
 * 注意：T1/V6/MAG 的从站地址都是 0x01，同一条总线只能挂其中一种自动机。
 */
class AutoMachineNode : public rclcpp::Node {
public:
  AutoMachineNode() : Node("auto_machine_node") {
    // ---------------- 串口参数 ----------------
    // 单串口（默认 /dev/ttyRS485，由 udev 软链接固定）：未配置多串口时使用
    const std::string serial_port =
        declare_parameter<std::string>("serial_port", "/dev/ttyRS485");
    // 多串口列表，索引顺序对应 param_data[0]（所有设备类型通用）：
    //   mag_serial_ports[0] -> /dev/ttyRS485_0，mag_serial_ports[1] -> /dev/ttyRS485_1 ...
    // 为空时回退为单机（仅 serial_port，索引只接受 0），保证旧部署可直接运行。
    // 参数名沿用 mag_serial_ports，兼容既有启动脚本。
    const std::vector<std::string> mag_serial_ports =
        declare_parameter<std::vector<std::string>>(
            "mag_serial_ports", std::vector<std::string>{});

    // 打开一个串口并返回实例（失败也返回实例，只是 isOpen()=false，后续通讯会报 10006）
    auto open_serial = [this](const std::string &path) {
      auto m = std::make_shared<auto_machine::ModbusRtu>();
      if (!m->open(path)) {
        RCLCPP_ERROR(get_logger(), "打开串口失败: %s （Modbus 通讯不可用）",
                     path.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "串口已打开: %s (115200 8O1 RTU)",
                    path.c_str());
      }
      return m;
    };

    // ---------------- 串口实例池 ----------------
    // 串口池：每台自动机一个独立串口，按 mag_serial_ports 索引顺序对应
    // param_data[0]；为空时回退单机，用 serial_port。
    auto_machine::ModbusPool mag_pool;
    if (mag_serial_ports.empty()) {
      mag_pool = {open_serial(serial_port)};
    } else {
      for (const auto &path : mag_serial_ports) {
        mag_pool.push_back(open_serial(path));
      }
    }

    // 串口数量：执行器线程数与「每串口闭环占用锁」都按它初始化
    port_count_ = mag_pool.empty() ? 1 : mag_pool.size();
    port_closed_loop_guard_.reserve(port_count_);
    for (size_t i = 0; i < port_count_; ++i) {
      port_closed_loop_guard_.push_back(std::make_unique<std::mutex>());
    }

    // 所有设备类型共享同一个串口池：索引 i 对应第 i 台自动机的串口。
    // 同一串口上同时挂自动机（T1/V6/MAG，从站 0x01）和对齐装置（XYZR，从站
    // 0x02），靠从站地址区分，所以自动机与对齐装置共享该串口实例。
    // 注意：T1/V6/MAG 从站地址都是 0x01，同一条总线只能挂其中一种自动机。
    modbus_pool_[AutoMachineDef::DEV_T1] = mag_pool;
    modbus_pool_[AutoMachineDef::DEV_V6] = mag_pool;
    modbus_pool_[AutoMachineDef::DEV_XYZR] = mag_pool;
    modbus_pool_[AutoMachineDef::DEV_MAG04] = mag_pool;
    modbus_pool_[AutoMachineDef::DEV_MAG06] = mag_pool;

    // 命令处理表初始化（各设备命令号 -> 处理函数，按设备索引分表）
    init_command_tables();

    // 命令服务挂到可重入回调组：配合 MultiThreadedExecutor，
    // 不同串口的请求可以由不同线程同时处理（默认互斥回调组会强制串行）。
    command_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_ = create_service<AutoMachineSrv>(
        "auto_machine/command",
        std::bind(&AutoMachineNode::handle_request, this,
                  std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), command_callback_group_);

    // ---------------- 故障主动上报 ----------------
    fault_reporter_ = std::make_unique<auto_machine::FaultReporter>(
        this, modbus_pool_);

    // ---------------- 控制命令闭环（下发后自动轮询状态直到到位/超时） ----------------
    const int auto_query_delay_ms =
        declare_parameter<int>("auto_query_delay_ms", 500);
    const int auto_query_poll_interval_ms =
        declare_parameter<int>("auto_query_poll_interval_ms", 300);
    const int auto_query_timeout_ms =
        declare_parameter<int>("auto_query_timeout_ms", 30000);
    closed_loop_ = std::make_shared<auto_machine::ClosedLoopExecutor>(
        auto_query_delay_ms, auto_query_poll_interval_ms,
        auto_query_timeout_ms);

    // ---------------- 执行器并发 ----------------
    // executor_threads：0（默认）= 按串口数自动推导，保证每个串口至少有一个
    // 线程可用（最坏情况是 N 个串口各跑一个闭环，各占一个线程）。
    const int executor_threads_param =
        declare_parameter<int>("executor_threads", 0);
    executor_threads_ =
        executor_threads_param > 0
            ? static_cast<size_t>(executor_threads_param)
            : std::max<size_t>(4, port_count_ * 2);

    RCLCPP_INFO(get_logger(),
                "auto_machine_node 启动，服务: auto_machine/command, "
                "主题: auto_machine/fault，串口数: %zu，执行器线程数: %zu",
                port_count_, executor_threads_);
  }

  /// 执行器线程数：main() 据此创建 MultiThreadedExecutor
  size_t executorThreads() const { return executor_threads_; }

private:
  using Handler = auto_machine::Handler;
  using HandlerMap = auto_machine::HandlerMap;
  /// 设备索引 -> 命令处理表（多机设备每个索引一套表）
  using DeviceHandlerMap = std::unordered_map<uint8_t, HandlerMap>;

  /**
   * @brief 判断是否 MAG 系列（多机设备）
   */
  static bool is_mag_device(uint32_t dev_type) {
    return dev_type == AutoMachineDef::DEV_MAG04 ||
           dev_type == AutoMachineDef::DEV_MAG06;
  }

  /**
   * @brief 是否按设备索引选择串口（MAG 系列 + XYZR 对齐装置）
   */
  static bool is_indexed_device(uint32_t dev_type) {
    return dev_type == AutoMachineDef::DEV_T1 ||
           dev_type == AutoMachineDef::DEV_V6 ||
           is_mag_device(dev_type) ||
           dev_type == AutoMachineDef::DEV_XYZR;
  }

  /**
   * @brief 请求入口：校验设备类型/设备索引/命令号，分发处理
   */
  void handle_request(const std::shared_ptr<AutoMachineSrv::Request> request,
                      std::shared_ptr<AutoMachineSrv::Response> response) {
    const uint32_t dev_type = request->dev_type;
    const uint32_t command = request->command;

    RCLCPP_INFO(get_logger(), "收到请求: dev_type=0x%08X, command=0x%08X, "
                              "param_data[%zu字节]",
                dev_type, command, request->param_data.size());

    // 0. 全局命令（故障上报开关）优先拦截，不属于任何具体设备
    if (command == AutoMachineDef::CMD_FAULT_CONTROL) {
      handle_fault_control_command(request, response);
      return;
    }

    // 1. 提取设备索引（所有设备类型统一：param_data[0] 为自动机/串口索引）
    //    MAG04/MAG06：必须显式提供（原有行为）；
    //    T1/V6/XYZR ：省略时默认 0（兼容单机旧调用）。
    uint8_t device_index = 0;
    if (is_indexed_device(dev_type)) {
      if (!request->param_data.empty()) {
        device_index = request->param_data[0];
      } else if (is_mag_device(dev_type)) {
        fill_error(response, auto_machine::err_code::ERR_INVALID_PARAM,
                   "MAG 设备需 param_data[0] 作为设备索引(0/1...)");
        return;
      }
    }

    // 2. 校验设备类型
    auto dev_it = handlers_.find(dev_type);
    if (dev_it == handlers_.end()) {
      fill_error(response, auto_machine::err_code::ERR_UNKNOWN_DEV_TYPE,
                 "未知设备类型: 0x" + to_hex(dev_type));
      return;
    }

    // 3. 校验设备索引（越界：索引超出该设备类型的串口池范围）
    auto idx_it = dev_it->second.find(device_index);
    if (idx_it == dev_it->second.end()) {
      fill_error(response, auto_machine::err_code::ERR_INVALID_PARAM,
                 "设备索引越界: " + std::to_string(device_index));
      return;
    }

    // 4. 校验命令号
    const auto &cmd_map = idx_it->second;
    auto cmd_it = cmd_map.find(command);
    if (cmd_it == cmd_map.end()) {
      fill_error(response, auto_machine::err_code::ERR_UNKNOWN_COMMAND,
                 "未知命令号: 0x" + to_hex(command));
      return;
    }

    // 4.5 同串口闭环互斥
    //     只有「会进入闭环等待」的控制命令才占用该串口，避免同一串口上两个闭环
    //     交替读同一个状态寄存器。抢不到立即返回 ERR_BUSY（不排队、不占线程）；
    //     查询/急停/吹气等单帧命令不占位，可在闭环等待期间插入执行。
    std::unique_lock<std::mutex> port_lock;
    if (auto_machine::ClosedLoopExecutor::find(dev_type, command) != nullptr) {
      if (device_index >= port_closed_loop_guard_.size()) {
        fill_error(response, auto_machine::err_code::ERR_INVALID_PARAM,
                   "设备索引越界: " + std::to_string(device_index));
        return;
      }
      port_lock = std::unique_lock<std::mutex>(
          *port_closed_loop_guard_[device_index], std::try_to_lock);
      if (!port_lock.owns_lock()) {
        RCLCPP_WARN(get_logger(),
                    "串口 %u 正在执行闭环命令，拒绝新的闭环命令 0x%s",
                    static_cast<unsigned>(device_index), to_hex(command).c_str());
        fill_error(response, auto_machine::err_code::ERR_BUSY,
                   "串口 " + std::to_string(device_index) +
                       " 正在执行闭环命令，请稍后重试");
        return;
      }
    }

    // 5. 执行命令处理函数（下发控制 / 读取查询）
    //    MAG 协议本身不使用 param_data，此处把完整 param_data（含索引首字节）
    //    原样传入，handler 内部会忽略。
    std::vector<uint8_t> ret_data;
    int32_t err_code = auto_machine::err_code::OK;
    try {
      err_code = cmd_it->second(request->param_data.data(),
                                request->param_data.size(), ret_data);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "处理命令异常: %s", e.what());
      fill_error(response, auto_machine::err_code::ERR_INTERNAL,
                 std::string("内部错误: ") + e.what());
      return;
    }

    // 6. 控制命令闭环：控制下发成功后，自动轮询对应状态寄存器，
    //    直到状态到达目标值（ret_data 返回最终状态）或超时（err_code=10008）。
    //    轮询使用与本次请求相同的串口实例，保证多机各自独立。
    if (err_code == auto_machine::err_code::OK &&
        auto_machine::ClosedLoopExecutor::find(dev_type, command) != nullptr) {
      err_code = closed_loop_->run(dev_type, command,
                                   modbus_pool_[dev_type][device_index],
                                   ret_data);
    }

    response->err_code = err_code;
    response->ret_data.assign(ret_data.begin(), ret_data.end());
    response->success = (err_code == auto_machine::err_code::OK);
    RCLCPP_INFO(get_logger(),
                "请求处理完成: dev_type=0x%08X index=%u success=%d, err_code=%d",
                dev_type, device_index, response->success, response->err_code);
  }

  // ---------------- 设备命令处理 ----------------

  /**
   * @brief 初始化各设备的命令处理表
   *
   * 各设备协议逻辑在 auto_machine_t1/v6/mag04/mag06/xyzr.cpp 中实现，
   * 通过 createXxxHandlers() 注册各自的命令表。
   * 所有设备类型（T1/V6/MAG04/MAG06/XYZR）对每个串口实例各生成一套命令表。
   */
  void init_command_tables() {
    // T1/V6：按串口池索引各建一套命令表
    for (size_t i = 0; i < modbus_pool_[AutoMachineDef::DEV_T1].size(); ++i) {
      handlers_[AutoMachineDef::DEV_T1][static_cast<uint8_t>(i)] =
          auto_machine::createT1Handlers(
              modbus_pool_[AutoMachineDef::DEV_T1][i]);
    }
    for (size_t i = 0; i < modbus_pool_[AutoMachineDef::DEV_V6].size(); ++i) {
      handlers_[AutoMachineDef::DEV_V6][static_cast<uint8_t>(i)] =
          auto_machine::createV6Handlers(
              modbus_pool_[AutoMachineDef::DEV_V6][i]);
    }

    // XYZR 对齐装置：与自动机共用串口池，按串口索引各建一套命令表
    for (size_t i = 0; i < modbus_pool_[AutoMachineDef::DEV_XYZR].size(); ++i) {
      handlers_[AutoMachineDef::DEV_XYZR][static_cast<uint8_t>(i)] =
          auto_machine::createXyzrHandlers(
              modbus_pool_[AutoMachineDef::DEV_XYZR][i]);
    }

    // MAG 设备：按串口池索引各建一套命令表
    for (size_t i = 0; i < modbus_pool_[AutoMachineDef::DEV_MAG04].size(); ++i) {
      handlers_[AutoMachineDef::DEV_MAG04][static_cast<uint8_t>(i)] =
          auto_machine::createMag04Handlers(
              modbus_pool_[AutoMachineDef::DEV_MAG04][i]);
    }
    for (size_t i = 0; i < modbus_pool_[AutoMachineDef::DEV_MAG06].size(); ++i) {
      handlers_[AutoMachineDef::DEV_MAG06][static_cast<uint8_t>(i)] =
          auto_machine::createMag06Handlers(
              modbus_pool_[AutoMachineDef::DEV_MAG06][i]);
    }
    // A100 设备（命令号待定义）
  }

  // ---------------- 故障上报入口 ----------------

  /**
   * @brief 故障上报控制（全局命令 CMD_FAULT_CONTROL 的处理）
   *
   * dev_type    = 要监控的设备类型（复用 AutoMachineDef::DEV_*）
   * param_data  = [enable]                （1 字节，默认 0 号机；T1/V6/XYZR 可用）
   *               [enable, device_index]  （2 字节，指定自动机；MAG 必须 2 字节）
   *
   * 具体轮询/边沿触发逻辑在 FaultReporter 中，多机按 (dev_type, 索引) 独立监控。
   */
  void handle_fault_control_command(
      const std::shared_ptr<AutoMachineSrv::Request> request,
      std::shared_ptr<AutoMachineSrv::Response> response) {
    if (request->param_data.empty()) {
      fill_error(response, auto_machine::err_code::ERR_INVALID_PARAM,
                 "故障上报参数需至少 1 字节: [enable(0/1)]");
      return;
    }
    const bool enable = request->param_data[0] != 0;

    // 所有设备都支持第二个字节指定设备索引；
    // 只有 MAG04/MAG06 强制要求 2 字节，其余 1 字节时默认 0 号（兼容旧调用）。
    uint8_t device_index = 0;
    if (is_indexed_device(request->dev_type)) {
      if (request->param_data.size() >= 2) {
        device_index = request->param_data[1];
      } else if (is_mag_device(request->dev_type)) {
        fill_error(response, auto_machine::err_code::ERR_INVALID_PARAM,
                   "MAG 故障上报参数需 2 字节: [enable(0/1), device_index]");
        return;
      }
    }

    if (enable) {
      const int32_t err =
          fault_reporter_->enable(request->dev_type, device_index);
      if (err != auto_machine::err_code::OK) {
        const std::string msg =
            (err == auto_machine::err_code::ERR_UNKNOWN_DEV_TYPE)
                ? "未知监控设备类型: 0x" + to_hex(request->dev_type)
                : "设备索引越界: " + std::to_string(device_index);
        fill_error(response, err, msg);
        return;
      }
      response->success = true;
      response->err_code = err;
    } else {
      fault_reporter_->disable(request->dev_type, device_index);
      response->success = true;
      response->err_code = auto_machine::err_code::OK;
    }
  }

  // ---------------- 工具函数 ----------------

  /**
   * @brief 构造错误响应
   */
  static void fill_error(std::shared_ptr<AutoMachineSrv::Response> response,
                         int32_t err, const std::string &msg) {
    response->success = false;
    response->err_code = err;
    response->ret_data.assign(msg.begin(), msg.end());
  }

  /**
   * @brief 整数转十六进制字符串（用于日志和错误信息）
   */
  static std::string to_hex(uint32_t value) {
    char buf[16] = {0};
    snprintf(buf, sizeof(buf), "%08X", value);
    return std::string(buf);
  }

  /// 串口实例池：设备类型 -> 串口实例列表（索引对应 mag_serial_ports 顺序）
  std::unordered_map<uint32_t, auto_machine::ModbusPool> modbus_pool_;
  /// 设备类型 -> (设备索引 -> (命令号 -> 处理函数))
  std::unordered_map<uint32_t, DeviceHandlerMap> handlers_;
  rclcpp::Service<AutoMachineSrv>::SharedPtr service_;
  /// 命令服务回调组（可重入：允许不同串口的请求并发进入 handle_request）
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  /// 串口数量（= 串口池长度）
  size_t port_count_ = 1;
  /// 执行器线程数（0 = 按串口数自动推导）
  size_t executor_threads_ = 4;
  /// 按串口索引的闭环占用锁：同一串口同时只允许一次「下发 + 闭环等待」
  std::vector<std::unique_ptr<std::mutex>> port_closed_loop_guard_;
  /// 故障主动上报模块
  std::unique_ptr<auto_machine::FaultReporter> fault_reporter_;
  /// 控制命令闭环执行器（下发后自动查询状态直到到位/超时）
  std::shared_ptr<auto_machine::ClosedLoopExecutor> closed_loop_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoMachineNode>();

  // 多线程执行器：不同串口的请求由不同线程并行处理。
  // 命令服务已挂到 Reentrant 回调组，否则多线程也只会串行执行。
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), node->executorThreads());
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
