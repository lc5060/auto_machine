#include "auto_machine/auto_machine_xyzr.hpp"

#include <rclcpp/rclcpp.hpp>

namespace auto_machine {

XyzrProtocol::XyzrProtocol(std::shared_ptr<ModbusRtu> modbus, uint8_t slave_addr)
    : modbus_(std::move(modbus)), slave_addr_(slave_addr) {}

int32_t XyzrProtocol::handle(uint32_t command, const uint8_t *param_data,
                             size_t param_len, std::vector<uint8_t> &ret_data) {
  (void)param_data;
  (void)param_len;
  ret_data.clear();
  auto logger = rclcpp::get_logger("XyzrProtocol");
  std::vector<uint16_t> regs;

  switch (command) {
    // ---------------- 控制类（保持寄存器，0x06 写，写后不等待应答） ----------------
    case AutoMachineDef::CMD_XYZR_RESET:
      // 复位
      if (!modbus_->writeSingleRegister(slave_addr_, xyzr_reg::RESET, 0x0001)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_XYZR_START_VISON_DETECT:
      // 布料已放好，触发视觉进行检测
      if (!modbus_->writeSingleRegister(slave_addr_, xyzr_reg::CLOTH_READY, 0x0001)) {
        return err_code::ERR_COMM;
      }
      break;

    // ---------------- 查询类（输入寄存器，功能码 0x04，1 个寄存器） ----------------
    case AutoMachineDef::CMD_XYZR_ADJUST_CLOTH_STATUS_QUERY:
      // 移料状态查询
      if (!modbus_->readInputRegisters(slave_addr_, xyzr_status::ADJUST_CLOTH,
                                       1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    case AutoMachineDef::CMD_XYZR_STATUS_QUERY:
      // 主控状态查询
      if (!modbus_->readInputRegisters(slave_addr_, xyzr_status::MASTER_STATUS, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    // 注：m4m6 协议无故障码寄存器，故障用 CMD_XYZR_STATUS_QUERY 读主控状态
    //     判断（值 100 = 发生故障），原先读 0x0003 的实现已删除。

    default:
      return err_code::ERR_UNKNOWN_COMMAND;
  }

  return err_code::OK;
}

HandlerMap createXyzrHandlers(std::shared_ptr<ModbusRtu> modbus) {
  auto protocol = std::make_shared<XyzrProtocol>(std::move(modbus));
  HandlerMap handlers;
  const uint32_t cmds[] = {
      AutoMachineDef::CMD_XYZR_RESET,
      AutoMachineDef::CMD_XYZR_START_VISON_DETECT,
      AutoMachineDef::CMD_XYZR_ADJUST_CLOTH_STATUS_QUERY,
      AutoMachineDef::CMD_XYZR_STATUS_QUERY,
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
