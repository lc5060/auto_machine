#ifndef AUTO_MACHINE_XYZR_HPP
#define AUTO_MACHINE_XYZR_HPP

#include <memory>

#include "auto_machine/auto_machine_common.hpp"
#include "auto_machine/modbus_rtu.hpp"

namespace auto_machine {

/// XYZR 移料装置主控 Modbus 从站地址（原协议确认为 0x02）
constexpr uint8_t XYZR_MODBUS_ADDR = 0x02;

/**
 * @brief XYZR 保持寄存器地址定义（读写：0x03 读 / 0x06 写）
 */
namespace xyzr_reg {
constexpr uint16_t RESET = 0x0000;      // 复位 1-主控回零，其他值无效
constexpr uint16_t CLOTH_READY = 0x0001;  // 布料已放好 1-触发视觉检测，其他值无效
constexpr uint16_t X_DISTANCE = 0x0002;   // X轴第一次移料距离，单位 mm
constexpr uint16_t Y_DISTANCE = 0x0003;   // Y轴第一次移料距离，单位 mm
}  // namespace xyzr_reg

/**
 * @brief XYZR 输入寄存器地址定义（只读，功能码 0x04）
 */
namespace xyzr_status {
constexpr uint16_t RESERVED = 0x0000;         // 预留暂未用
constexpr uint16_t ADJUST_CLOTH = 0x0001;     // 移料状态（移料后检测结果）
constexpr uint16_t MASTER_STATUS = 0x0002;    // 主控状态
constexpr uint16_t FAULT_CODE = 0x0003;       // 故障代码
}  // namespace xyzr_status

/**
 * @brief XYZR 状态值定义
 */
namespace xyzr_val {
// 移料状态（寄存器 0x0001）
constexpr uint16_t ADJUST_FAIL = 0;      // 移料失败
constexpr uint16_t ADJUST_OK = 1;        // 移料成功，移到位，可以进行下一步缝纫动作

// 主控状态（寄存器 0x0002）
constexpr uint16_t MASTER_IDLE = 0;      // 待机（主控空闲，等待指令）
constexpr uint16_t MASTER_READY = 1;     // 准备就绪，可接收命令
constexpr uint16_t MASTER_RUNNING = 10;  // 执行中，正在移料
constexpr uint16_t MASTER_DONE = 11;     // 执行完成，等待确认
constexpr uint16_t MASTER_PAUSE = 20;    // 暂停
constexpr uint16_t MASTER_FAULT = 100;   // 发生故障
}  // namespace xyzr_val

/**
 * @brief XYZR 移料装置协议处理器
 */
class XyzrProtocol {
public:
  explicit XyzrProtocol(std::shared_ptr<ModbusRtu> modbus,
                        uint8_t slave_addr = XYZR_MODBUS_ADDR);

  /**
   * @brief 处理 XYZR 设备命令
   * @param command  命令号（见 AutoMachineDef.msg 中 CMD_XYZR_*）
   * @param param_data 请求参数数据
   * @param param_len  参数数据长度
   * @param ret_data   [out] 响应返回数据（查询命令返回 2 字节大端寄存器值）
   * @return 错误码，err_code::OK 表示成功
   */
  int32_t handle(uint32_t command, const uint8_t *param_data, size_t param_len,
                 std::vector<uint8_t> &ret_data);

private:
  std::shared_ptr<ModbusRtu> modbus_;
  uint8_t slave_addr_;
};

/**
 * @brief 创建 XYZR 设备的命令处理表
 * @param modbus 共享的 Modbus RTU 实例（RS485 总线）
 * @return 命令号 -> 处理函数 映射表
 */
HandlerMap createXyzrHandlers(std::shared_ptr<ModbusRtu> modbus);

}  // namespace auto_machine

#endif  // AUTO_MACHINE_XYZR_HPP
