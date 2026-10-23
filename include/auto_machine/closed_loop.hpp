#ifndef AUTO_MACHINE_CLOSED_LOOP_HPP
#define AUTO_MACHINE_CLOSED_LOOP_HPP

#include <cstdint>
#include <memory>
#include <vector>

#include "auto_machine/auto_machine_common.hpp"
#include "auto_machine/modbus_rtu.hpp"

namespace auto_machine {

/**
 * @brief 闭环状态匹配方式
 */
enum class MatchType {
  EQ,       // 寄存器值 == 目标值
  NE_ZERO,  // 寄存器值 != 0（用于“打开/到位”等开关量）
  BIT_SET,  // (寄存器值 & 目标掩码) == 目标掩码（用于运行状态字的某个 bit）
};

/**
 * @brief 一条“控制命令 -> 状态查询”的闭环绑定
 *
 * 节点下发控制命令成功后，若该命令在绑定表中，则自动轮询 status_reg，
 * 直到状态满足 match/target（或超时）。
 * 个别动作（XYZR 触发视觉检测）需要两阶段判定：先等主控状态到 DONE，
 * 再读移料结果，此时使用 phase2_* 字段。
 */
struct ClosedLoopBinding {
  uint32_t dev_type;       // 设备类型 AutoMachineDef::DEV_*
  uint32_t control_cmd;    // 控制命令号 AutoMachineDef::CMD_*
  uint8_t slave_addr;      // Modbus 从站地址
  bool is_input_reg;       // true=0x04 读输入寄存器；false=0x03 读保持寄存器
  uint16_t status_reg;     // 要轮询的状态寄存器地址
  MatchType match;         // 状态匹配方式
  uint16_t target;         // 目标值 / 位掩码

  // 第二阶段（默认无；仅需要串联两次判定的命令使用）
  bool has_phase2 = false;
  uint16_t phase2_reg = 0;
  bool phase2_is_input = true;
  MatchType phase2_match = MatchType::EQ;
  uint16_t phase2_target = 0;
};

/**
 * @brief 控制命令闭环执行器
 *
 * 下发控制命令后：先等待 delay_ms，再按 poll_interval_ms 周期查询状态寄存器，
 * 状态到达目标值立即返回；超过 timeout_ms 仍未到位返回 ERR_ACTION_TIMEOUT。
 * 每次读寄存器都独立占用/释放总线锁，轮询等待期间不持锁，不影响故障上报线程。
 */
class ClosedLoopExecutor {
public:
  ClosedLoopExecutor(int delay_ms, int poll_interval_ms, int timeout_ms);

  /**
   * @brief 执行闭环等待
   * @param dev_type     设备类型
   * @param control_cmd  已成功下发的控制命令号
   * @param modbus       本次命令所用的串口实例（多机时按设备索引传入对应实例）
   * @param ret_data     [out] 到位/超时时返回最终状态值（2 字节大端）；通讯失败可能为空
   * @return OK 到位；ERR_COMM 轮询通讯失败；ERR_ACTION_TIMEOUT 超时未到位
   */
  int32_t run(uint32_t dev_type, uint32_t control_cmd,
              std::shared_ptr<ModbusRtu> modbus,
              std::vector<uint8_t> &ret_data) const;

  /**
   * @brief 查找某条控制命令的闭环绑定；无绑定返回 nullptr（该命令不做闭环）
   */
  static const ClosedLoopBinding *find(uint32_t dev_type, uint32_t control_cmd);

private:
  /// 读一个状态寄存器，成功写入 val 并返回 true
  bool readStatus(std::shared_ptr<ModbusRtu> modbus, uint8_t slave_addr,
                  bool is_input, uint16_t reg, uint16_t &val) const;

  static bool matches(uint16_t value, MatchType type, uint16_t target);

  int delay_ms_;          // 控制后首次查询前的等待（毫秒）
  int poll_interval_ms_;  // 轮询间隔（毫秒）
  int timeout_ms_;        // 轮询总超时（毫秒）
};

}  // namespace auto_machine

#endif  // AUTO_MACHINE_CLOSED_LOOP_HPP
