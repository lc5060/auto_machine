#include "auto_machine/auto_machine_t1.hpp"

#include <rclcpp/rclcpp.hpp>

namespace auto_machine {

T1Protocol::T1Protocol(std::shared_ptr<ModbusRtu> modbus, uint8_t slave_addr) {
  modbus_ = std::move(modbus);
  slave_addr_ = slave_addr;
}

int32_t T1Protocol::handle(uint32_t command, const uint8_t *param_data,
                           size_t param_len, std::vector<uint8_t> &ret_data) {
  (void)param_data;
  (void)param_len;
  ret_data.clear();
  auto logger = rclcpp::get_logger("T1Protocol");
  std::vector<uint16_t> regs;

  switch (command) {
    // ---------------- 状态查询类（功能码 0x03 读保持寄存器） ----------------
    case AutoMachineDef::CMD_T1_STATUS_QUERY: {
      // 状态查询：bit0-复位完成 bit1-缝纫完成
      if (!modbus_->readHoldingRegisters(slave_addr_, t1_reg::RUN_STATUS, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      const uint16_t run = regs.empty() ? 0 : regs[0];
      RCLCPP_INFO(logger, "状态：复位:%s | 缝纫:%s",
                  (run & 0x01) ? "已复位" : "未复位",
                  (run & 0x02) ? "完成" : "未完成");
      break;
    }

    case AutoMachineDef::CMD_T1_EXCEPTION_QUERY:
      // 故障查询
      if (!modbus_->readHoldingRegisters(slave_addr_, t1_reg::FAULT, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      if (!regs.empty() && regs[0] != 0) {
        RCLCPP_WARN(logger, "故障码: 0x%04X", regs[0]);
      }
      break;

    case AutoMachineDef::CMD_T1_WAIT_STATUS_QUERY:
      // 自动化设备等待查询
      if (!modbus_->readHoldingRegisters(slave_addr_, t1_reg::AUTO_WAIT, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    case AutoMachineDef::CMD_T1_XIFENG_STATUS_QUERY:
      // 吸风状态查询
      if (!modbus_->readHoldingRegisters(slave_addr_, t1_reg::SUCTION_STATUS, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      RCLCPP_INFO(logger, "吸风状态：%s",
                  (!regs.empty() && regs[0] != 0) ? "开启" : "关闭");
      break;

    case AutoMachineDef::CMD_T1_JIAJU_IO_STATUS1_QUERY:
      // 自动化夹具IO状态查询（CH1～CH16）
      if (!modbus_->readHoldingRegisters(slave_addr_, t1_reg::CLAMP_IO1, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    case AutoMachineDef::CMD_T1_JIAJU_IO_STATUS2_QUERY:
      // 自动化夹具IO状态查询（CH17～CH24）
      if (!modbus_->readHoldingRegisters(slave_addr_, t1_reg::CLAMP_IO2, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    // ---------------- 控制类（功能码 0x06 写，写后不等待应答） ----------------
    case AutoMachineDef::CMD_T1_PRESS_FRAME_DOWN:
      // 压框下压
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, t1_reg::PRESS_FRAME, 1)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_T1_PRESS_FRAME_UP:
      // 压框抬起
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, t1_reg::PRESS_FRAME, 0)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_T1_START_WORK:
      // 开始工作
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, t1_reg::WORK_CTRL, 1)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_T1_PAUSE_WORK:
      // 暂停工作
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, t1_reg::WORK_CTRL, 0)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_T1_START_XIFENG:
      // 打开吸风
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, t1_reg::SUCTION_CTRL, 1)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_T1_STOP_XIFENG:
      // 关闭吸风
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, t1_reg::SUCTION_CTRL, 0)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_T1_NEXT_STEP:
      // 自动化设备下一步
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, t1_reg::AUTO_NEXT, 1)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_T1_RESET:
      // 复位
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, t1_reg::RESET, 1)) {
        return err_code::ERR_COMM;
      }
      break;

    default:
      return err_code::ERR_UNKNOWN_COMMAND;
  }

  return err_code::OK;
}

HandlerMap createT1Handlers(std::shared_ptr<ModbusRtu> modbus) {
  // T1 从站地址 0x01
  auto protocol = std::make_shared<T1Protocol>(std::move(modbus), T1_MODBUS_ADDR);
  HandlerMap handlers;
  const uint32_t cmds[] = {
      AutoMachineDef::CMD_T1_STATUS_QUERY,
      AutoMachineDef::CMD_T1_EXCEPTION_QUERY,
      AutoMachineDef::CMD_T1_WAIT_STATUS_QUERY,
      AutoMachineDef::CMD_T1_XIFENG_STATUS_QUERY,
      AutoMachineDef::CMD_T1_JIAJU_IO_STATUS1_QUERY,
      AutoMachineDef::CMD_T1_JIAJU_IO_STATUS2_QUERY,
      AutoMachineDef::CMD_T1_PRESS_FRAME_DOWN,
      AutoMachineDef::CMD_T1_PRESS_FRAME_UP,
      AutoMachineDef::CMD_T1_START_WORK,
      AutoMachineDef::CMD_T1_PAUSE_WORK,
      AutoMachineDef::CMD_T1_START_XIFENG,
      AutoMachineDef::CMD_T1_STOP_XIFENG,
      AutoMachineDef::CMD_T1_NEXT_STEP,
      AutoMachineDef::CMD_T1_RESET,
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
