#include "auto_machine/auto_machine_xyzr1.hpp"

#include <rclcpp/rclcpp.hpp>

namespace auto_machine {

Xyzr1Protocol::Xyzr1Protocol(std::shared_ptr<ModbusRtu> modbus,
                             uint8_t slave_addr)
    : modbus_(std::move(modbus)), slave_addr_(slave_addr) {}

int32_t Xyzr1Protocol::handle(uint32_t command, const uint8_t *param_data,
                              size_t param_len, std::vector<uint8_t> &ret_data) {
  (void)param_data;
  (void)param_len;
  ret_data.clear();
  std::vector<uint16_t> regs;

  switch (command) {
    // ---------------- 控制类（保持寄存器，0x06 写，带应答校验） ----------------
    case AutoMachineDef::CMD_XYZR1_RESET:
      // 复位（主控回零）
      if (!modbus_->writeSingleRegister(slave_addr_, xyzr1_reg::RESET, 0x0001)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_XYZR1_BODY_CLOTH_READY:
      // 大身布料已放好，触发视觉检测
      if (!modbus_->writeSingleRegister(slave_addr_,
                                        xyzr1_reg::BODY_CLOTH_READY, 0x0001)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_XYZR1_POCKET_CLOTH_READY:
      // 口袋布已放好，触发视觉检测
      if (!modbus_->writeSingleRegister(slave_addr_,
                                        xyzr1_reg::POCKET_CLOTH_READY, 0x0001)) {
        return err_code::ERR_COMM;
      }
      break;

    // ---------------- 查询类（输入寄存器，功能码 0x04，1 个寄存器） ----------------
    case AutoMachineDef::CMD_XYZR1_MASTER_STATUS_QUERY:
      // 主控状态查询：0 待机 / 1 就绪 / 10 执行中 / 11 完成待确认 / 20 暂停 / 100 故障
      if (!modbus_->readInputRegisters(slave_addr_, xyzr1_status::MASTER_STATUS,
                                       1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    case AutoMachineDef::CMD_XYZR1_BODY_MOVE_STATUS_QUERY:
      // 大身移料状态：0 失败 / 1 成功到位
      if (!modbus_->readInputRegisters(slave_addr_,
                                       xyzr1_status::BODY_MOVE_STATUS, 1,
                                       regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    case AutoMachineDef::CMD_XYZR1_POCKET_MOVE_STATUS_QUERY:
      // 口袋移料状态：0 失败 / 1 成功到位
      if (!modbus_->readInputRegisters(slave_addr_,
                                       xyzr1_status::POCKET_MOVE_STATUS, 1,
                                       regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    case AutoMachineDef::CMD_XYZR1_FAULT_CODE_QUERY:
      // 故障码查询（输入寄存器 0x0002）
      if (!modbus_->readInputRegisters(slave_addr_, xyzr1_status::FAULT_CODE, 1,
                                       regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    default:
      return err_code::ERR_UNKNOWN_COMMAND;
  }

  return err_code::OK;
}

HandlerMap createXyzr1Handlers(std::shared_ptr<ModbusRtu> modbus) {
  auto protocol = std::make_shared<Xyzr1Protocol>(std::move(modbus));
  HandlerMap handlers;
  const uint32_t cmds[] = {
      AutoMachineDef::CMD_XYZR1_RESET,
      AutoMachineDef::CMD_XYZR1_BODY_CLOTH_READY,
      AutoMachineDef::CMD_XYZR1_POCKET_CLOTH_READY,
      AutoMachineDef::CMD_XYZR1_MASTER_STATUS_QUERY,
      AutoMachineDef::CMD_XYZR1_BODY_MOVE_STATUS_QUERY,
      AutoMachineDef::CMD_XYZR1_POCKET_MOVE_STATUS_QUERY,
      AutoMachineDef::CMD_XYZR1_FAULT_CODE_QUERY,
  };
  for (uint32_t cmd : cmds) {
    auto p = protocol;
    handlers[cmd] = [p, cmd](const uint8_t *d, size_t n,
                             std::vector<uint8_t> &r) {
      return p->handle(cmd, d, n, r);
    };
  }
  return handlers;
}

}  // namespace auto_machine
