#ifndef AUTO_MACHINE_COMMON_HPP
#define AUTO_MACHINE_COMMON_HPP

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include <auto_machine/msg/auto_machine_def.hpp>

namespace auto_machine {

class ModbusRtu;

/// msg 常量定义所在结构（AutoMachineDef.msg 中的设备类型/命令号常量）
using AutoMachineDef = auto_machine::msg::AutoMachineDef;

/**
 * @brief 串口实例池
 *
 * 同一设备类型下的多个独立 ModbusRtu 串口实例（多机场景按设备索引取用）。
 * 非多机设备类型的池中通常只有 1 个元素（设备索引恒为 0）。
 */
using ModbusPool = std::vector<std::shared_ptr<ModbusRtu>>;

/**
 * @brief 自动机服务端错误码
 */
namespace err_code {
constexpr int32_t OK = 0;                        // 成功
constexpr int32_t ERR_UNKNOWN_DEV_TYPE = 10001;  // 未知设备类型
constexpr int32_t ERR_UNKNOWN_COMMAND = 10002;   // 未知命令号
constexpr int32_t ERR_INVALID_PARAM = 10003;     // 参数数据不合法
constexpr int32_t ERR_NOT_IMPLEMENTED = 10004;   // 功能未实现
constexpr int32_t ERR_INTERNAL = 10005;          // 内部错误
constexpr int32_t ERR_COMM = 10006;              // Modbus 通讯失败
constexpr int32_t ERR_DEV_EXCEPTION = 10007;     // 设备返回 Modbus 异常码
constexpr int32_t ERR_ACTION_TIMEOUT = 10008;    // 闭环控制：动作下发后，状态在超时时间内未达到目标值
constexpr int32_t ERR_BUSY = 10009;              // 该串口已有闭环命令在执行，请稍后重试（不排队）
}  // namespace err_code

/**
 * @brief 命令处理函数类型
 *
 * @param param_data 请求参数数据（AutoMachine.srv 的 param_data）
 * @param param_len  参数数据长度
 * @param ret_data   [out] 响应返回数据（AutoMachine.srv 的 ret_data）
 * @return 错误码，err_code::OK 表示成功
 */
using Handler = std::function<int32_t(const uint8_t *param_data,
                                      size_t param_len,
                                      std::vector<uint8_t> &ret_data)>;

/**
 * @brief 命令号 -> 处理函数 映射表
 */
using HandlerMap = std::unordered_map<uint32_t, Handler>;

/**
 * @brief 把字节流按大端解析为 uint16 数组（Modbus 寄存器字节序）
 */
inline std::vector<uint16_t> bytesToRegs(const uint8_t *data, size_t len) {
  std::vector<uint16_t> regs;
  regs.reserve((len + 1) / 2);
  for (size_t i = 0; i + 1 < len; i += 2) {
    regs.push_back(static_cast<uint16_t>((data[i] << 8) | data[i + 1]));
  }
  return regs;
}

/**
 * @brief 把 uint16 数组按大端序列化为字节流（Modbus 寄存器字节序）
 */
inline std::vector<uint8_t> regsToBytes(const std::vector<uint16_t> &regs) {
  std::vector<uint8_t> bytes;
  bytes.reserve(regs.size() * 2);
  for (uint16_t r : regs) {
    bytes.push_back(static_cast<uint8_t>(r >> 8));
    bytes.push_back(static_cast<uint8_t>(r & 0xFF));
  }
  return bytes;
}

/**
 * @brief 把 2 字节大端字节流解析为一个寄存器值（regsToBytes 的逆变换）
 */
inline uint16_t regFromBytes(const std::vector<uint8_t> &bytes) {
  if (bytes.size() < 2) {
    return 0;
  }
  return static_cast<uint16_t>((bytes[0] << 8) | bytes[1]);
}

}  // namespace auto_machine

#endif  // AUTO_MACHINE_COMMON_HPP
