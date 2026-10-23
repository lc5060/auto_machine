#ifndef AUTO_MACHINE_MAG06_HPP
#define AUTO_MACHINE_MAG06_HPP

#include <memory>

#include "auto_machine/auto_machine_common.hpp"
#include "auto_machine/auto_machine_mag_base.hpp"
#include "auto_machine/modbus_rtu.hpp"

namespace auto_machine {

/// MAG06 自动机 Modbus 从站地址（原协议确认为 0x01）
constexpr uint8_t MAG06_MODBUS_ADDR = 0x01;

/**
 * @brief MAG06 寄存器地址定义（原协议确认）
 */
namespace mag06_reg {
constexpr uint16_t CTRL_CMD = 0x1064;         // 主控制命令寄存器
constexpr uint16_t CLOTH_CMD = 0x1068;        // 理布控制命令寄存器
constexpr uint16_t BLOW_CMD = 0x106A;         // 吹气控制命令寄存器
constexpr uint16_t PRESS_CMD = 0x106B;        // 压板控制命令寄存器（1-下压 0-上抬）

constexpr uint16_t ERROR_CODE = 0x1065;       // 故障码查询
constexpr uint16_t ADMISSION_STATUS = 0x1066; // 准入状态查询
constexpr uint16_t XIFENG_STATUS = 0x1067;    // 吸风状态查询
constexpr uint16_t FUPING_STATUS = 0x1069;    // 理布状态查询
}  // namespace mag06_reg

/**
 * @brief MAG06 自动机协议处理器
 */
class Mag06Protocol : public MagProtocol {
public:
  explicit Mag06Protocol(std::shared_ptr<ModbusRtu> modbus,
                         uint8_t slave_addr = MAG06_MODBUS_ADDR);

  /**
   * @brief 处理 MAG06 设备命令
   * @param command  命令号（见 AutoMachineDef.msg 中 CMD_MAG06_*）
   * @param param_data 请求参数数据
   * @param param_len  参数数据长度
   * @param ret_data   [out] 响应返回数据（查询命令返回 2 字节大端寄存器值）
   * @return 错误码，err_code::OK 表示成功
   */
  int32_t handle(uint32_t command, const uint8_t *param_data, size_t param_len,
                 std::vector<uint8_t> &ret_data);
};

/**
 * @brief 创建 MAG06 设备的命令处理表
 * @param modbus 共享的 Modbus RTU 实例（RS485 总线）
 * @return 命令号 -> 处理函数 映射表
 */
HandlerMap createMag06Handlers(std::shared_ptr<ModbusRtu> modbus);

}  // namespace auto_machine

#endif  // AUTO_MACHINE_MAG06_HPP
