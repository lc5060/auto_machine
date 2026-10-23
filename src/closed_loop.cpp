#include "auto_machine/closed_loop.hpp"

#include <chrono>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "auto_machine/auto_machine_mag04.hpp"
#include "auto_machine/auto_machine_mag06.hpp"
#include "auto_machine/auto_machine_t1.hpp"
#include "auto_machine/auto_machine_v6.hpp"
#include "auto_machine/auto_machine_xyzr.hpp"

namespace auto_machine {

namespace {

// ============================================================================
// 控制命令 -> 状态查询 闭环绑定表
// 只有“有对应状态寄存器、且协议定义了完成值”的控制命令才登记在此表；
// 未登记的控制命令（压框、镭射灯、吹气、急停、人工、暂停/下一步、V6取料/抚平、
// MAG04/06 开始/关闭理布、XYZR下发距离等）协议本身无反馈位，仍为下发即返回，不做闭环。
// ============================================================================
const std::vector<ClosedLoopBinding> kBindings = {
    // ---------------- T1（从站 0x01，0x03 读保持寄存器） ----------------
    // 复位完成：运行状态 bit0=1
    {AutoMachineDef::DEV_T1, AutoMachineDef::CMD_T1_RESET,
     T1_MODBUS_ADDR, false, t1_reg::RUN_STATUS, MatchType::BIT_SET, 0x01},
    // 缝纫完成：运行状态 bit1=1
    {AutoMachineDef::DEV_T1, AutoMachineDef::CMD_T1_START_WORK,
     T1_MODBUS_ADDR, false, t1_reg::RUN_STATUS, MatchType::BIT_SET, 0x02},
    // 打开吸风：吸风状态非 0
    {AutoMachineDef::DEV_T1, AutoMachineDef::CMD_T1_START_XIFENG,
     T1_MODBUS_ADDR, false, t1_reg::SUCTION_STATUS, MatchType::NE_ZERO, 0},
    // 关闭吸风：吸风状态为 0
    {AutoMachineDef::DEV_T1, AutoMachineDef::CMD_T1_STOP_XIFENG,
     T1_MODBUS_ADDR, false, t1_reg::SUCTION_STATUS, MatchType::EQ, 0},

    // ---------------- V6（从站 0x01，0x04 读输入寄存器） ----------------
    // 舌板伸出到位：折叠状态=10（等待折叠）
    {AutoMachineDef::DEV_V6, AutoMachineDef::CMD_V6_TONGUE_PROTRUDE,
     V6_MODBUS_ADDR, true, v6_status::FOLD_STATUS, MatchType::EQ, 10},
    // 折叠完成：折叠状态=26（等待取料）
    {AutoMachineDef::DEV_V6, AutoMachineDef::CMD_V6_START_FOLD,
     V6_MODBUS_ADDR, true, v6_status::FOLD_STATUS, MatchType::EQ, 26},
    // 折叠复位完成：折叠状态回到 0（等待舌板伸出）【目标值按状态机推断，需真机确认】
    {AutoMachineDef::DEV_V6, AutoMachineDef::CMD_V6_SET_FOLD_RESET,
     V6_MODBUS_ADDR, true, v6_status::FOLD_STATUS, MatchType::EQ, 0},
    // 开始吸布：吸布状态=1
    {AutoMachineDef::DEV_V6, AutoMachineDef::CMD_V6_START_XIBU,
     V6_MODBUS_ADDR, true, v6_status::INHALE_STATE, MatchType::EQ, 1},
    // 停止吸布：吸布状态=0
    {AutoMachineDef::DEV_V6, AutoMachineDef::CMD_V6_STOP_XIBU,
     V6_MODBUS_ADDR, true, v6_status::INHALE_STATE, MatchType::EQ, 0},
    // 开鼓风机：鼓风状态=1
    {AutoMachineDef::DEV_V6, AutoMachineDef::CMD_V6_OPEN_BLOWER,
     V6_MODBUS_ADDR, true, v6_status::BREEZE_STATE, MatchType::EQ, 1},
    // 关鼓风机：鼓风状态=0
    {AutoMachineDef::DEV_V6, AutoMachineDef::CMD_V6_CLOSE_BLOWER,
     V6_MODBUS_ADDR, true, v6_status::BREEZE_STATE, MatchType::EQ, 0},

    // ---------------- MAG04（从站 0x01，0x03 读保持寄存器） ----------------
    // 复位完成：准入状态=1
    {AutoMachineDef::DEV_MAG04, AutoMachineDef::CMD_MAG04_RESET,
     MAG04_MODBUS_ADDR, false, mag04_reg::ADMISSION_STATUS, MatchType::EQ, 1},
    // 本件缝制完成：准入状态=3
    {AutoMachineDef::DEV_MAG04, AutoMachineDef::CMD_MAG04_START_TAILOR,
     MAG04_MODBUS_ADDR, false, mag04_reg::ADMISSION_STATUS, MatchType::EQ, 3},
    // 开吸风：吸风状态非 0
    {AutoMachineDef::DEV_MAG04, AutoMachineDef::CMD_MAG04_START_XIFENG,
     MAG04_MODBUS_ADDR, false, mag04_reg::XIFENG_STATUS, MatchType::NE_ZERO, 0},
    // 关吸风：吸风状态为 0
    {AutoMachineDef::DEV_MAG04, AutoMachineDef::CMD_MAG04_STOP_XIFENG,
     MAG04_MODBUS_ADDR, false, mag04_reg::XIFENG_STATUS, MatchType::EQ, 0},
    // 注：抚平机构没有到位检测装置，开始理布 / 关闭理布 / 关闭气缸 全部保持开环
    //     （控制帧下发即返回，不轮询 FUPING_STATUS；否则寄存器恒为 0-气缸未到位，
    //      必然轮询到 30s 超时并返回 10008）

    // ---------------- MAG06（从站 0x01，0x03 读保持寄存器，基址 0x10xx） ----
    {AutoMachineDef::DEV_MAG06, AutoMachineDef::CMD_MAG06_RESET,
     MAG06_MODBUS_ADDR, false, mag06_reg::ADMISSION_STATUS, MatchType::EQ, 1},
    {AutoMachineDef::DEV_MAG06, AutoMachineDef::CMD_MAG06_START_TAILOR,
     MAG06_MODBUS_ADDR, false, mag06_reg::ADMISSION_STATUS, MatchType::EQ, 3},
    {AutoMachineDef::DEV_MAG06, AutoMachineDef::CMD_MAG06_START_XIFENG,
     MAG06_MODBUS_ADDR, false, mag06_reg::XIFENG_STATUS, MatchType::NE_ZERO, 0},
    {AutoMachineDef::DEV_MAG06, AutoMachineDef::CMD_MAG06_STOP_XIFENG,
     MAG06_MODBUS_ADDR, false, mag06_reg::XIFENG_STATUS, MatchType::EQ, 0},
    // 注：M6 同理——抚平机构无到位检测装置，关闭气缸保持开环（仅下发）

    // ---------------- XYZR（从站 0x02，0x04 读输入寄存器） ----------------
    // 复位（回零）完成：主控状态=1（准备就绪）
    {AutoMachineDef::DEV_XYZR, AutoMachineDef::CMD_XYZR_RESET,
     XYZR_MODBUS_ADDR, true, xyzr_status::MASTER_STATUS, MatchType::EQ,
     xyzr_val::MASTER_READY},
    // 触发视觉检测（两阶段）：
    //   阶段1 主控状态=11（执行完成，等待确认）
    //   阶段2 移料结果=1（移料成功），最终 ret_data 返回移料结果
    {AutoMachineDef::DEV_XYZR, AutoMachineDef::CMD_XYZR_START_VISON_DETECT,
     XYZR_MODBUS_ADDR, true, xyzr_status::MASTER_STATUS, MatchType::EQ,
     xyzr_val::MASTER_DONE,
     true, xyzr_status::ADJUST_CLOTH, true, MatchType::EQ,
     xyzr_val::ADJUST_OK},
};

}  // namespace

ClosedLoopExecutor::ClosedLoopExecutor(int delay_ms, int poll_interval_ms,
                                       int timeout_ms)
    : delay_ms_(delay_ms),
      poll_interval_ms_(poll_interval_ms),
      timeout_ms_(timeout_ms) {}

const ClosedLoopBinding *ClosedLoopExecutor::find(uint32_t dev_type,
                                                  uint32_t control_cmd) {
  for (const auto &b : kBindings) {
    if (b.dev_type == dev_type && b.control_cmd == control_cmd) {
      return &b;
    }
  }
  return nullptr;
}

bool ClosedLoopExecutor::matches(uint16_t value, MatchType type,
                                 uint16_t target) {
  switch (type) {
    case MatchType::EQ:
      return value == target;
    case MatchType::NE_ZERO:
      return value != 0;
    case MatchType::BIT_SET:
      return (value & target) == target;
    default:
      return false;
  }
}

bool ClosedLoopExecutor::readStatus(std::shared_ptr<ModbusRtu> modbus,
                                    uint8_t slave_addr, bool is_input,
                                    uint16_t reg, uint16_t &val) const {
  std::vector<uint16_t> regs;
  bool ok = is_input
                ? modbus->readInputRegisters(slave_addr, reg, 1, regs)
                : modbus->readHoldingRegisters(slave_addr, reg, 1, regs);
  if (ok && !regs.empty()) {
    val = regs[0];
    return true;
  }
  return false;
}

int32_t ClosedLoopExecutor::run(uint32_t dev_type, uint32_t control_cmd,
                                std::shared_ptr<ModbusRtu> modbus,
                                std::vector<uint8_t> &ret_data) const {
  const ClosedLoopBinding *b = find(dev_type, control_cmd);
  if (b == nullptr) {
    return err_code::OK;  // 无绑定的命令不做闭环
  }

  ret_data.clear();
  const auto logger = rclcpp::get_logger("closed_loop");

  // 控制后先等待一段时间，再开始查询（给设备启动动作的时间）
  if (delay_ms_ > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms_));
  }

  const auto start = std::chrono::steady_clock::now();
  bool in_phase2 = false;
  bool have_last = false;
  uint16_t last_value = 0;

  while (true) {
    // 选择当前阶段要查询的寄存器与判据
    const bool use_phase2 = in_phase2 && b->has_phase2;
    const uint16_t reg = use_phase2 ? b->phase2_reg : b->status_reg;
    const bool is_input = use_phase2 ? b->phase2_is_input : b->is_input_reg;
    const MatchType match = use_phase2 ? b->phase2_match : b->match;
    const uint16_t target = use_phase2 ? b->phase2_target : b->target;

    uint16_t value = 0;
    if (!readStatus(modbus, b->slave_addr, is_input, reg, value)) {
      RCLCPP_WARN(logger,
                  "闭环查询通讯失败 dev=0x%08X cmd=0x%08X reg=0x%04X",
                  dev_type, control_cmd, reg);
      if (have_last) {
        ret_data = regsToBytes(std::vector<uint16_t>{last_value});
      }
      return err_code::ERR_COMM;
    }
    // 打印首次读到的值、以及之后每次发生变化的值，便于在真机上直接核对
    // 该状态寄存器的真实反馈（排查地址/功能码/到位枚举值是否与协议一致）
    if (!have_last || value != last_value) {
      const char *match_desc =
          match == MatchType::EQ ? "等于"
          : match == MatchType::NE_ZERO ? "非0"
                                        : "位掩码";
      RCLCPP_INFO(logger,
                  "等待到位 reg=0x%04X(%s) 当前值=%u，判定要求[%s %u]",
                  reg, is_input ? "输入0x04" : "保持0x03", value, match_desc,
                  target);
    }
    last_value = value;
    have_last = true;

    if (matches(value, match, target)) {
      if (!use_phase2 && b->has_phase2) {
        // 阶段1完成，进入阶段2
        RCLCPP_INFO(logger, "阶段1到位 reg=0x%04X 值=%d，进入阶段2确认", reg,
                    value);
        in_phase2 = true;
        continue;
      }
      // 全部到位
      ret_data = regsToBytes(std::vector<uint16_t>{value});
      RCLCPP_INFO(logger, "动作到位 dev=0x%08X cmd=0x%08X 最终状态=%d",
                  dev_type, control_cmd, value);
      return err_code::OK;
    }

    // 未到位，判断是否超时
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - start)
                             .count();
    if (elapsed >= timeout_ms_) {
      ret_data = regsToBytes(std::vector<uint16_t>{last_value});
      RCLCPP_WARN(logger,
                  "动作超时未到位 dev=0x%08X cmd=0x%08X reg=0x%04X(%s) "
                  "最后读到值=%u，目标值=%u（%lldms）。"
                  "查询通讯正常但状态未达标；若真机确已到位，请核对该状态寄存器的"
                  "地址 / 功能码(0x03保持或0x04输入) / 到位枚举值",
                  dev_type, control_cmd, reg,
                  is_input ? "输入0x04" : "保持0x03", last_value, target,
                  static_cast<long long>(elapsed));
      return err_code::ERR_ACTION_TIMEOUT;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(poll_interval_ms_));
  }

  // 理论不可达（循环内已覆盖所有返回路径），兜底避免无返回值警告
  return err_code::ERR_ACTION_TIMEOUT;
}

}  // namespace auto_machine
