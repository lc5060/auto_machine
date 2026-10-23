#ifndef AUTO_MACHINE_V6_HPP
#define AUTO_MACHINE_V6_HPP

#include <memory>

#include "auto_machine/auto_machine_common.hpp"
#include "auto_machine/modbus_rtu.hpp"

namespace auto_machine {

/// V6 自动机 Modbus 从站地址（原协议确认为 0x01）
constexpr uint8_t V6_MODBUS_ADDR = 0x01;

/**
 * @brief V6 控制寄存器地址定义（保持寄存器，功能码 0x06 写，写后不等待应答）
 */
namespace v6_reg {
constexpr uint16_t TONGUE = 0x0000;   // 舌板伸出
constexpr uint16_t FOLD = 0x0001;     // 折叠开始
constexpr uint16_t GET = 0x0002;      // 取料开始
constexpr uint16_t OPTICAL = 0x0003;  // 镭射灯控制方式
constexpr uint16_t LASER1 = 0x0004;   // 镭射灯1
constexpr uint16_t LASER2 = 0x0005;   // 镭射灯2
constexpr uint16_t LASER3 = 0x0006;   // 镭射灯3
constexpr uint16_t LASER4 = 0x0007;   // 镭射灯4
constexpr uint16_t RESET = 0x0008;    // 折叠复位
constexpr uint16_t BREEZE = 0x0009;   // 鼓风机 1-开启 2-关闭
constexpr uint16_t INHALE = 0x000A;   // 吸布 1-开启 2-关闭
constexpr uint16_t FLATTEN = 0x000B;  // 抚平 1-开始 2-返回起始位置 3-复位
}  // namespace v6_reg

/**
 * @brief V6 查询寄存器地址定义（输入寄存器，功能码 0x04 读）
 */
namespace v6_status {
constexpr uint16_t ERR = 0x0000;           // 故障码
constexpr uint16_t MACHINE = 0x0001;       // 机器状态（2为缝制界面）
constexpr uint16_t FOLD_STATUS = 0x0002;   // 折叠状态 0-等待舌板伸出 10-等待折叠 26-等待取料
constexpr uint16_t BREEZE_STATE = 0x0003;  // 鼓风机状态
constexpr uint16_t INHALE_STATE = 0x0004;  // 吸布状态 0-未开启 1-开启
constexpr uint16_t ALLOW_STATUS = 0x0005;  // 准入状态
constexpr uint16_t FLATTEN_STATE = 0x0006; // 抚平状态
}  // namespace v6_status

/**
 * @brief V6 控制动作数值定义
 */
namespace v6_val {
constexpr uint16_t ON = 1;                 // 开启/触发
constexpr uint16_t OFF = 0;                // 关闭（镭射灯等）
constexpr uint16_t BREEZE_ON = 1;          // 鼓风机开启
constexpr uint16_t BREEZE_OFF = 2;         // 鼓风机关闭
constexpr uint16_t INHALE_ON = 1;          // 吸布开启
constexpr uint16_t INHALE_OFF = 2;         // 吸布关闭
constexpr uint16_t FLATTEN_START = 1;      // 开始抚平
constexpr uint16_t FLATTEN_RETURN = 2;     // 返回抚平起始位置
constexpr uint16_t FLATTEN_RESET = 3;      // 抚平复位
}  // namespace v6_val

/**
 * @brief V6 自动机协议处理器
 */
class V6Protocol {
public:
  explicit V6Protocol(std::shared_ptr<ModbusRtu> modbus,
                      uint8_t slave_addr = V6_MODBUS_ADDR);

  /**
   * @brief 处理 V6 设备命令
   * @param command  命令号（见 AutoMachineDef.msg 中 CMD_V6_*）
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
 * @brief 创建 V6 设备的命令处理表
 * @param modbus 共享的 Modbus RTU 实例（RS485 总线）
 * @return 命令号 -> 处理函数 映射表
 */
HandlerMap createV6Handlers(std::shared_ptr<ModbusRtu> modbus);

}  // namespace auto_machine

#endif  // AUTO_MACHINE_V6_HPP
