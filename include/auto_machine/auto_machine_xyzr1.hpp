#ifndef AUTO_MACHINE_XYZR1_HPP
#define AUTO_MACHINE_XYZR1_HPP

#include <memory>

#include "auto_machine/auto_machine_common.hpp"
#include "auto_machine/modbus_rtu.hpp"

namespace auto_machine {

/**
 * @brief XYZR1 对齐装置（U1 / V6 配套）Modbus 从站地址
 *
 * 协议：主控为从站，站号 2（0x02），115200 8O1，Modbus RTU。
 *
 * 注意：XYZR1 与 XYZR（钉扣配套）的从站地址都是 0x02，但寄存器布局不同，
 *       同一条 RS485 总线上只能挂其中一种，靠 dev_type 选择用哪套协议解析。
 *       一条总线同时下发两种 XYZR 命令，会读到错位的数据。
 */
constexpr uint8_t XYZR1_MODBUS_ADDR = 0x02;

/**
 * @brief XYZR1 保持寄存器地址定义（读写：0x03 读 / 0x06 写）
 */
namespace xyzr1_reg {
constexpr uint16_t RESET = 0x0000;              // 复位 1-主控回零，其他值无效
constexpr uint16_t BODY_CLOTH_READY = 0x0001;   // 大身布料已放好 1-触发视觉检测
constexpr uint16_t POCKET_CLOTH_READY = 0x0002; // 口袋布已放好 1-触发视觉检测
}  // namespace xyzr1_reg

/**
 * @brief XYZR1 输入寄存器地址定义（只读，功能码 0x04）
 */
namespace xyzr1_status {
constexpr uint16_t RESERVED = 0x0000;           // 预留暂未用
constexpr uint16_t MASTER_STATUS = 0x0001;      // 主控状态
constexpr uint16_t FAULT_CODE = 0x0002;         // 故障码
constexpr uint16_t BODY_MOVE_STATUS = 0x0003;   // 大身移料状态（移料后检测结果）
constexpr uint16_t POCKET_MOVE_STATUS = 0x0004; // 口袋移料状态（移料后检测结果）
}  // namespace xyzr1_status

/**
 * @brief XYZR1 状态值定义
 */
namespace xyzr1_val {
// 移料状态（寄存器 0x0003 / 0x0004）
constexpr uint16_t MOVE_FAIL = 0;  // 移料失败
constexpr uint16_t MOVE_OK = 1;    // 移料成功，移到位，可以进行下一步缝纫动作

// 主控状态（寄存器 0x0001）
constexpr uint16_t MASTER_IDLE = 0;     // 待机（主控空闲，等待指令）
constexpr uint16_t MASTER_READY = 1;    // 准备就绪，可接收命令
constexpr uint16_t MASTER_RUNNING = 10; // 执行中，正在移料
constexpr uint16_t MASTER_DONE = 11;    // 执行完成，等待确认
constexpr uint16_t MASTER_PAUSE = 20;   // 暂停
constexpr uint16_t MASTER_FAULT = 100;  // 发生故障
}  // namespace xyzr1_val

/**
 * @brief XYZR1 对齐装置协议处理器
 */
class Xyzr1Protocol {
public:
  explicit Xyzr1Protocol(std::shared_ptr<ModbusRtu> modbus,
                         uint8_t slave_addr = XYZR1_MODBUS_ADDR);

  /**
   * @brief 处理 XYZR1 设备命令
   * @param command    命令号（见 AutoMachineDef.msg 中 CMD_XYZR1_*）
   * @param param_data 请求参数数据（本设备不使用）
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
 * @brief 创建 XYZR1 设备的命令处理表
 * @param modbus 共享的 Modbus RTU 实例（RS485 总线）
 * @return 命令号 -> 处理函数 映射表
 */
HandlerMap createXyzr1Handlers(std::shared_ptr<ModbusRtu> modbus);

}  // namespace auto_machine

#endif  // AUTO_MACHINE_XYZR1_HPP
