#ifndef AUTO_MACHINE_T1_HPP
#define AUTO_MACHINE_T1_HPP

#include <memory>

#include "auto_machine/auto_machine_common.hpp"
#include "auto_machine/modbus_rtu.hpp"

namespace auto_machine {

/// T1 自动机 Modbus 从站地址（原协议确认为 0x01）
constexpr uint8_t T1_MODBUS_ADDR = 0x01;

/**
 * @brief T1 寄存器地址定义（原协议确认）
 */
namespace t1_reg {
// 只读状态寄存器（功能码 0x03 读）
constexpr uint16_t RUN_STATUS = 0x1000;      // 运行状态 bit0-复位完成 bit1-缝纫完成
constexpr uint16_t FAULT = 0x1001;           // 故障码
constexpr uint16_t AUTO_WAIT = 0x1004;       // 自动化设备等待状态
constexpr uint16_t SUCTION_STATUS = 0x1014;  // 吸风状态
constexpr uint16_t CLAMP_IO1 = 0x1020;       // 夹具IO状态 CH1~CH16
constexpr uint16_t CLAMP_IO2 = 0x1021;       // 夹具IO状态 CH17~CH24

// 读写控制寄存器（功能码 0x06 写，写后不等待应答）
constexpr uint16_t PRESS_FRAME = 0x1500;     // 压框 1-下压 0-抬起
constexpr uint16_t WORK_CTRL = 0x1501;       // 工作 1-开始工作 0-暂停工作
constexpr uint16_t SUCTION_CTRL = 0x1502;    // 吸风 1-打开 0-关闭
constexpr uint16_t AUTO_NEXT = 0x1520;       // 自动化下一步
constexpr uint16_t RESET = 0x1530;           // 复位

// PUREG 动作表
constexpr uint16_t PUREG_BASE = 0x2000;      // 动作表起始，每条动作占2个寄存器，最多512条
constexpr uint16_t CLAMP_IDX_REG = 0x2400;   // 当前夹具段索引
}  // namespace t1_reg

/**
 * @brief T1 自动机协议处理器
 *
 * 串口参数与所有设备统一：115200 / 奇校验 / 1停止位 / 8数据位 / RTU
 */
class T1Protocol {
public:
  explicit T1Protocol(std::shared_ptr<ModbusRtu> modbus,
                      uint8_t slave_addr = T1_MODBUS_ADDR);

  /**
   * @brief 处理 T1 设备命令
   * @param command  命令号（见 AutoMachineDef.msg 中 CMD_T1_*）
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
 * @brief 创建 T1 设备的命令处理表
 * @param modbus T1 专用的 Modbus RTU 实例（8N1 无校验串口）
 * @return 命令号 -> 处理函数 映射表
 */
HandlerMap createT1Handlers(std::shared_ptr<ModbusRtu> modbus);

}  // namespace auto_machine

#endif  // AUTO_MACHINE_T1_HPP
