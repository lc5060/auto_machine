#ifndef AUTO_MACHINE_MAG_BASE_HPP
#define AUTO_MACHINE_MAG_BASE_HPP

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "auto_machine/auto_machine_common.hpp"
#include "auto_machine/modbus_rtu.hpp"

namespace auto_machine {

/**
 * @brief MAG 系列寄存器地址表
 *
 * 各型号按各自《对接协议文档》填写实际地址
 * （MAG06 已按原协议确认；MAG04 暂为占位，待协议确认）
 */
struct MagRegisterMap {
  uint16_t ctrl_cmd;          // 主控制命令寄存器（人工/吸风/关闭吸风/缝制/急停/复位）
  uint16_t cloth_cmd;         // 理布控制命令寄存器（开始理布/关闭理布/关闭气缸）
  uint16_t blow_cmd;          // 吹气控制命令寄存器（开始吹气/停止吹气）
  uint16_t press_cmd;         // 压板控制命令寄存器（1-下压 0-上抬）
  uint16_t error_code;        // 故障码查询寄存器
  uint16_t admission_status;  // 准入状态查询寄存器
  uint16_t xifeng_status;     // 吸风状态查询寄存器
  uint16_t fuping_status;     // 理布状态查询寄存器
};

/**
 * @brief MAG 系列（MAG04/MAG06）公共协议基类
 *
 * MAG04 与 MAG06 命令集完全相同（仅命令号常量和寄存器地址不同），
 * 控制指令：功能码 0x06 写命令寄存器，下发后不等待应答；
 * 查询指令：功能码 0x03 读保持寄存器，1 个寄存器。
 */
class MagProtocol {
public:
  MagProtocol(std::shared_ptr<ModbusRtu> modbus, uint8_t slave_addr,
              const MagRegisterMap &regs)
      : modbus_(std::move(modbus)), slave_addr_(slave_addr), regs_(regs) {}

  virtual ~MagProtocol() = default;

  /**
   * @brief 控制类命令：写命令寄存器（下发后不等待从站应答）
   * @param reg_addr 命令寄存器地址
   * @param value    命令值
   */
  int32_t handleWrite(uint32_t /*command*/, const uint8_t *param_data,
                      size_t param_len, std::vector<uint8_t> &ret_data,
                      uint16_t reg_addr, uint16_t value) {
    (void)param_data;
    (void)param_len;
    ret_data.clear();
    if (!modbus_->writeSingleRegisterNoReply(slave_addr_, reg_addr, value)) {
      return err_code::ERR_COMM;
    }
    return err_code::OK;
  }

  /**
   * @brief 查询类命令：读保持寄存器（功能码 0x03，1 个寄存器）
   * @param reg_addr 查询寄存器地址
   * @note 查询结果（2 字节大端）写入 ret_data，具体状态含义由各型号解析
   */
  int32_t handleQuery(uint32_t /*command*/, const uint8_t *param_data,
                      size_t param_len, std::vector<uint8_t> &ret_data,
                      uint16_t reg_addr) {
    (void)param_data;
    (void)param_len;
    ret_data.clear();

    std::vector<uint16_t> regs;
    if (!modbus_->readHoldingRegisters(slave_addr_, reg_addr, 1, regs)) {
      return err_code::ERR_COMM;
    }
    ret_data = regsToBytes(regs);
    return err_code::OK;
  }

protected:
  // ---------------- 查询结果解码日志（MAG04/MAG06 状态语义相同，共用） ----------------

  void logErrorCode(uint16_t val) const {
    auto logger = rclcpp::get_logger("MagProtocol");
    if (val == 0) {
      RCLCPP_INFO(logger, "故障码：无故障");
    } else {
      RCLCPP_WARN(logger, "故障码：有故障 (0x%04X)", val);
    }
  }

  void logAdmission(uint16_t val) const {
    auto logger = rclcpp::get_logger("MagProtocol");
    switch (val) {
      case 0: RCLCPP_INFO(logger, "准入状态：机器未复位不可执行"); break;
      case 1: RCLCPP_INFO(logger, "准入状态：机器已完成复位可执行"); break;
      case 2: RCLCPP_INFO(logger, "准入状态：机器未完成当前件缝制"); break;
      case 3: RCLCPP_INFO(logger, "准入状态：机器已完成当前件缝制"); break;
      default: RCLCPP_WARN(logger, "准入状态：未知值 %u", val); break;
    }
  }

  void logXifeng(uint16_t val) const {
    auto logger = rclcpp::get_logger("MagProtocol");
    RCLCPP_INFO(logger, "吸风状态：%s", val == 0 ? "关闭" : "开启");
  }

  void logFuping(uint16_t val) const {
    auto logger = rclcpp::get_logger("MagProtocol");
    switch (val) {
      case 0: RCLCPP_INFO(logger, "理布状态：气缸未到位"); break;
      case 1: RCLCPP_INFO(logger, "理布状态：气缸到位"); break;
      case 2: RCLCPP_INFO(logger, "理布状态：气缸复位"); break;
      default: RCLCPP_WARN(logger, "理布状态：未知值 %u", val); break;
    }
  }

  std::shared_ptr<ModbusRtu> modbus_;
  uint8_t slave_addr_;
  MagRegisterMap regs_;
};

}  // namespace auto_machine

#endif  // AUTO_MACHINE_MAG_BASE_HPP
