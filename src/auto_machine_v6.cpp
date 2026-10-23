#include "auto_machine/auto_machine_v6.hpp"

#include <rclcpp/rclcpp.hpp>

namespace auto_machine {

V6Protocol::V6Protocol(std::shared_ptr<ModbusRtu> modbus, uint8_t slave_addr) {
  modbus_ = std::move(modbus);
  slave_addr_ = slave_addr;
}

int32_t V6Protocol::handle(uint32_t command, const uint8_t *param_data,
                           size_t param_len, std::vector<uint8_t> &ret_data) {
  (void)param_data;
  (void)param_len;
  ret_data.clear();
  auto logger = rclcpp::get_logger("V6Protocol");
  std::vector<uint16_t> regs;

  switch (command) {
    // ---------------- 控制类（保持寄存器，0x06 写，写后不等待应答） ----------------
    case AutoMachineDef::CMD_V6_TONGUE_PROTRUDE:
      // 舌板伸出
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::TONGUE, v6_val::ON)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_START_FOLD:
      // 开始折叠
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::FOLD, v6_val::ON)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_START_TAKE_FABRIC:
      // 开始取料
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::GET, v6_val::ON)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_SET_LASER_CONTROL_BY_ROBOT:
      // 镭射灯机器人控制
      // TODO: 原协议仅定义 OPT_ON=1，机器人控制写 1 / 屏幕控制写 0 待确认
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::OPTICAL, v6_val::ON)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_SET_LASER_CONTROL_BY_MCU:
      // 镭射灯屏幕控制
      // TODO: 待确认（与机器人控制共用 0x0003 寄存器）
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::OPTICAL, v6_val::OFF)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_OPEN_LASER1:
      // 镭射灯1开启
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::LASER1, v6_val::ON)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_CLOSE_LASER1:
      // 镭射灯1关闭
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::LASER1, v6_val::OFF)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_OPEN_LASER2:
      // 镭射灯2开启
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::LASER2, v6_val::ON)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_CLOSE_LASER2:
      // 镭射灯2关闭
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::LASER2, v6_val::OFF)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_OPEN_LASER3:
      // 镭射灯3开启
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::LASER3, v6_val::ON)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_CLOSE_LASER3:
      // 镭射灯3关闭
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::LASER3, v6_val::OFF)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_OPEN_LASER4:
      // 镭射灯4开启
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::LASER4, v6_val::ON)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_CLOSE_LASER4:
      // 镭射灯4关闭
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::LASER4, v6_val::OFF)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_SET_FOLD_RESET:
      // 折叠复位（舌板伸出之后，发送折叠复位指令，舌板会复位；
      // 折叠完成之后，发送折叠复位指令，折叠会复位）
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::RESET, v6_val::ON)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_OPEN_BLOWER:
      // 开启鼓风机
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::BREEZE, v6_val::BREEZE_ON)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_CLOSE_BLOWER:
      // 关闭鼓风机
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::BREEZE, v6_val::BREEZE_OFF)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_START_XIBU:
      // 开始吸布
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::INHALE, v6_val::INHALE_ON)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_STOP_XIBU:
      // 停止吸布
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::INHALE, v6_val::INHALE_OFF)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_START_FUPING:
      // 开始抚平
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::FLATTEN, v6_val::FLATTEN_START)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_FUPING_BACKTO_START:
      // 返回抚平起始位置
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::FLATTEN, v6_val::FLATTEN_RETURN)) {
        return err_code::ERR_COMM;
      }
      break;

    case AutoMachineDef::CMD_V6_FUPING_RESET:
      // 抚平复位
      if (!modbus_->writeSingleRegisterNoReply(slave_addr_, v6_reg::FLATTEN, v6_val::FLATTEN_RESET)) {
        return err_code::ERR_COMM;
      }
      break;

    // ---------------- 查询类（输入寄存器，功能码 0x04，1 个寄存器） ----------------
    case AutoMachineDef::CMD_V6_MACHINE_STATUS_QUERY:
      // 查询机器状态（2为缝制界面）
      if (!modbus_->readInputRegisters(slave_addr_, v6_status::MACHINE, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    case AutoMachineDef::CMD_V6_FOLD_STATUS_QUERY: {
      // 查询折叠状态 0-等待舌板伸出 10-等待折叠 26-等待取料
      if (!modbus_->readInputRegisters(slave_addr_, v6_status::FOLD_STATUS, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      if (!regs.empty()) {
        switch (regs[0]) {
          case 0: RCLCPP_INFO(logger, "折叠状态：等待舌板伸出"); break;
          case 10: RCLCPP_INFO(logger, "折叠状态：等待折叠"); break;
          case 26: RCLCPP_INFO(logger, "折叠状态：等待取料"); break;
          default: RCLCPP_INFO(logger, "折叠状态：%u", regs[0]); break;
        }
      }
      break;
    }

    case AutoMachineDef::CMD_V6_XIBU_STATUS_QUERY:
      // 吸布状态查询 0-未开启 1-开启
      if (!modbus_->readInputRegisters(slave_addr_, v6_status::INHALE_STATE, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      RCLCPP_INFO(logger, "吸布状态：%s",
                  (!regs.empty() && regs[0] != 0) ? "开启" : "未开启");
      break;

    case AutoMachineDef::CMD_V6_ADMISSION_STATUS_QUERY:
      // 准入状态查询
      if (!modbus_->readInputRegisters(slave_addr_, v6_status::ALLOW_STATUS, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    case AutoMachineDef::CMD_V6_FUPING_STATUS_QUERY:
      // 抚平状态查询
      if (!modbus_->readInputRegisters(slave_addr_, v6_status::FLATTEN_STATE, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      break;

    case AutoMachineDef::CMD_V6_ERROR_CODE_QUERY: {
      // 故障码查询：0-无故障 非0-有故障
      if (!modbus_->readInputRegisters(slave_addr_, v6_status::ERR, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      if (!regs.empty() && regs[0] != 0) {
        RCLCPP_WARN(logger, "故障码：有故障 (0x%04X)", regs[0]);
      }
      break;
    }

    case AutoMachineDef::CMD_V6_BLOWER_STATUS_QUERY:
      // 鼓风机状态查询 0-未开启 1-已开启
      if (!modbus_->readInputRegisters(slave_addr_, v6_status::BREEZE_STATE, 1, regs)) {
        return err_code::ERR_COMM;
      }
      ret_data = regsToBytes(regs);
      RCLCPP_INFO(logger, "鼓风机状态：%s",
                  (!regs.empty() && regs[0] != 0) ? "已开启" : "未开启");
      break;

    default:
      return err_code::ERR_UNKNOWN_COMMAND;
  }

  return err_code::OK;
}

HandlerMap createV6Handlers(std::shared_ptr<ModbusRtu> modbus) {
  auto protocol = std::make_shared<V6Protocol>(std::move(modbus), V6_MODBUS_ADDR);
  HandlerMap handlers;
  const uint32_t cmds[] = {
      AutoMachineDef::CMD_V6_TONGUE_PROTRUDE,
      AutoMachineDef::CMD_V6_START_FOLD,
      AutoMachineDef::CMD_V6_START_TAKE_FABRIC,
      AutoMachineDef::CMD_V6_SET_LASER_CONTROL_BY_ROBOT,
      AutoMachineDef::CMD_V6_SET_LASER_CONTROL_BY_MCU,
      AutoMachineDef::CMD_V6_OPEN_LASER1,
      AutoMachineDef::CMD_V6_CLOSE_LASER1,
      AutoMachineDef::CMD_V6_OPEN_LASER2,
      AutoMachineDef::CMD_V6_CLOSE_LASER2,
      AutoMachineDef::CMD_V6_OPEN_LASER3,
      AutoMachineDef::CMD_V6_CLOSE_LASER3,
      AutoMachineDef::CMD_V6_OPEN_LASER4,
      AutoMachineDef::CMD_V6_CLOSE_LASER4,
      AutoMachineDef::CMD_V6_SET_FOLD_RESET,
      AutoMachineDef::CMD_V6_OPEN_BLOWER,
      AutoMachineDef::CMD_V6_CLOSE_BLOWER,
      AutoMachineDef::CMD_V6_START_XIBU,
      AutoMachineDef::CMD_V6_STOP_XIBU,
      AutoMachineDef::CMD_V6_START_FUPING,
      AutoMachineDef::CMD_V6_FUPING_BACKTO_START,
      AutoMachineDef::CMD_V6_FUPING_RESET,
      AutoMachineDef::CMD_V6_MACHINE_STATUS_QUERY,
      AutoMachineDef::CMD_V6_FOLD_STATUS_QUERY,
      AutoMachineDef::CMD_V6_XIBU_STATUS_QUERY,
      AutoMachineDef::CMD_V6_ADMISSION_STATUS_QUERY,
      AutoMachineDef::CMD_V6_FUPING_STATUS_QUERY,
      AutoMachineDef::CMD_V6_ERROR_CODE_QUERY,
      AutoMachineDef::CMD_V6_BLOWER_STATUS_QUERY,
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
