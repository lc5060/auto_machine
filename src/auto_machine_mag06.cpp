#include "auto_machine/auto_machine_mag06.hpp"

#include <rclcpp/rclcpp.hpp>

namespace auto_machine {

Mag06Protocol::Mag06Protocol(std::shared_ptr<ModbusRtu> modbus,
                             uint8_t slave_addr)
    : MagProtocol(std::move(modbus), slave_addr,
                  // 寄存器地址表（原协议确认）
                  {mag06_reg::CTRL_CMD, mag06_reg::CLOTH_CMD, mag06_reg::BLOW_CMD,
                   mag06_reg::PRESS_CMD,
                   mag06_reg::ERROR_CODE, mag06_reg::ADMISSION_STATUS,
                   mag06_reg::XIFENG_STATUS, mag06_reg::FUPING_STATUS}) {}

int32_t Mag06Protocol::handle(uint32_t command, const uint8_t *param_data,
                              size_t param_len, std::vector<uint8_t> &ret_data) {
  switch (command) {
    // ---------------- 主控制命令寄存器 0x1064（写后不等待应答） ----------------
    case AutoMachineDef::CMD_MAG06_MANUAL_CONTROL:
      // 人工控制
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::CTRL_CMD, 0x0000);

    case AutoMachineDef::CMD_MAG06_START_XIFENG:
      // 开始吸风
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::CTRL_CMD, 0x0001);

    case AutoMachineDef::CMD_MAG06_STOP_XIFENG:
      // 关闭吸风
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::CTRL_CMD, 0x0005);

    case AutoMachineDef::CMD_MAG06_START_TAILOR:
      // 开始缝制
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::CTRL_CMD, 0x0002);

    case AutoMachineDef::CMD_MAG06_EMERGENCY_STOP:
      // 急停
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::CTRL_CMD, 0x0003);

    case AutoMachineDef::CMD_MAG06_RESET:
      // 复位
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::CTRL_CMD, 0x0004);

    // ---------------- 理布控制命令寄存器 0x1068（写后不等待应答） ----------------
    case AutoMachineDef::CMD_MAG06_START_FUPING:
      // 开始理布
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::CLOTH_CMD, 0x0001);

    case AutoMachineDef::CMD_MAG06_STOP_FUPING:
      // 关闭理布
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::CLOTH_CMD, 0x0002);

    case AutoMachineDef::CMD_MAG06_STOP_CYLINDER:
      // 关闭气缸
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::CLOTH_CMD, 0x0003);

    // ---------------- 吹气控制命令寄存器 0x106A（写后不等待应答） ----------------
    case AutoMachineDef::CMD_MAG06_START_BLOWER:
      // 开始吹气
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::BLOW_CMD, 0x0001);

    case AutoMachineDef::CMD_MAG06_STOP_BLOWER:
      // 停止吹气
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::BLOW_CMD, 0x0000);

    // ---------------- 压板控制命令寄存器 0x106B（写后不等待应答） ----------------
    case AutoMachineDef::CMD_MAG06_PRESS_PLATE_DOWN:
      // 压板下压
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::PRESS_CMD, 0x0001);

    case AutoMachineDef::CMD_MAG06_PRESS_PLATE_UP:
      // 压板抬起
      return handleWrite(command, param_data, param_len, ret_data,
                         mag06_reg::PRESS_CMD, 0x0000);

    // ---------------- 查询命令（功能码 0x03，1 个寄存器） ----------------
    case AutoMachineDef::CMD_MAG06_ERROR_CODE_QUERY: {
      // 查询故障码：0-无故障
      int32_t err = handleQuery(command, param_data, param_len, ret_data,
                                mag06_reg::ERROR_CODE);
      if (err != err_code::OK) {
        return err;
      }
      logErrorCode(regFromBytes(ret_data));
      return err_code::OK;
    }

    case AutoMachineDef::CMD_MAG06_ADMISSION_STATUS_QUERY: {
      // 查询准入状态：0-机器未复位不可执行 1-机器已完成复位可执行
      //               2-机器未完成当前件缝制 3-机器已完成当前件缝制
      int32_t err = handleQuery(command, param_data, param_len, ret_data,
                                mag06_reg::ADMISSION_STATUS);
      if (err != err_code::OK) {
        return err;
      }
      logAdmission(regFromBytes(ret_data));
      return err_code::OK;
    }

    case AutoMachineDef::CMD_MAG06_XIFENG_STATUS_QUERY: {
      // 查询吸风状态：0-关闭 1-开启
      int32_t err = handleQuery(command, param_data, param_len, ret_data,
                                mag06_reg::XIFENG_STATUS);
      if (err != err_code::OK) {
        return err;
      }
      logXifeng(regFromBytes(ret_data));
      return err_code::OK;
    }

    case AutoMachineDef::CMD_MAG06_FUPING_STATUS_QUERY: {
      // 查询理布状态：0-气缸未到位 1-气缸到位 2-气缸复位
      int32_t err = handleQuery(command, param_data, param_len, ret_data,
                                mag06_reg::FUPING_STATUS);
      if (err != err_code::OK) {
        return err;
      }
      logFuping(regFromBytes(ret_data));
      return err_code::OK;
    }

    default:
      return err_code::ERR_UNKNOWN_COMMAND;
  }
}

HandlerMap createMag06Handlers(std::shared_ptr<ModbusRtu> modbus) {
  auto protocol = std::make_shared<Mag06Protocol>(std::move(modbus));
  HandlerMap handlers;
  const uint32_t cmds[] = {
      AutoMachineDef::CMD_MAG06_START_XIFENG,
      AutoMachineDef::CMD_MAG06_STOP_XIFENG,
      AutoMachineDef::CMD_MAG06_START_TAILOR,
      AutoMachineDef::CMD_MAG06_EMERGENCY_STOP,
      AutoMachineDef::CMD_MAG06_MANUAL_CONTROL,
      AutoMachineDef::CMD_MAG06_RESET,
      AutoMachineDef::CMD_MAG06_START_FUPING,
      AutoMachineDef::CMD_MAG06_STOP_FUPING,
      AutoMachineDef::CMD_MAG06_STOP_CYLINDER,
      AutoMachineDef::CMD_MAG06_START_BLOWER,
      AutoMachineDef::CMD_MAG06_STOP_BLOWER,
      AutoMachineDef::CMD_MAG06_PRESS_PLATE_DOWN,
      AutoMachineDef::CMD_MAG06_PRESS_PLATE_UP,
      AutoMachineDef::CMD_MAG06_XIFENG_STATUS_QUERY,
      AutoMachineDef::CMD_MAG06_ERROR_CODE_QUERY,
      AutoMachineDef::CMD_MAG06_ADMISSION_STATUS_QUERY,
      AutoMachineDef::CMD_MAG06_FUPING_STATUS_QUERY,
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
