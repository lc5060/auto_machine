#ifndef AUTO_MACHINE_MODBUS_RTU_HPP
#define AUTO_MACHINE_MODBUS_RTU_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace auto_machine {

/**
 * @brief RS485 Modbus RTU 通讯类
 *
 * 所有设备的串口参数统一为：
 *   波特率：115200
 *   校验：  奇校验（Odd）
 *   停止位：1
 *   数据位：8
 *   协议：  MODBUS RTU
 *
 * 所有设备的命令收发都通过本类的单一实例进行（RS485 半双工总线，
 * 内部使用互斥锁保证同一时刻只有一个事务占用总线）。
 */
class ModbusRtu {
public:
  ModbusRtu() = default;
  ~ModbusRtu();

  // 禁止拷贝（持有串口资源）
  ModbusRtu(const ModbusRtu &) = delete;
  ModbusRtu &operator=(const ModbusRtu &) = delete;

  /**
   * @brief 打开串口并按 115200 8O1（奇校验）配置
   * @param device_path 串口设备路径，如 /dev/ttyUSB0、/dev/ttyRS485
   * @return true 成功
   */
  bool open(const std::string &device_path);

  /**
   * @brief 关闭串口
   */
  void close();

  /**
   * @brief 串口是否已打开
   */
  bool isOpen() const { return fd_ >= 0; }

  // ---------------- Modbus 功能码封装 ----------------

  /**
   * @brief 读保持寄存器（功能码 0x03）
   * @param slave_addr 从站地址
   * @param reg_addr   起始寄存器地址
   * @param qty        读取数量（1~125）
   * @param regs       [out] 读到的寄存器值（高字节在前）
   * @return true 成功
   */
  bool readHoldingRegisters(uint8_t slave_addr, uint16_t reg_addr,
                            uint16_t qty, std::vector<uint16_t> &regs);

  /**
   * @brief 读输入寄存器（功能码 0x04）
   */
  bool readInputRegisters(uint8_t slave_addr, uint16_t reg_addr,
                          uint16_t qty, std::vector<uint16_t> &regs);

  /**
   * @brief 写单个寄存器（功能码 0x06）
   */
  bool writeSingleRegister(uint8_t slave_addr, uint16_t reg_addr,
                           uint16_t value);

  /**
   * @brief 写单个寄存器（功能码 0x06），下发后不等待从站应答
   *
   * 与 MAG06 原协议一致：控制指令只下发不读应答。
   */
  bool writeSingleRegisterNoReply(uint8_t slave_addr, uint16_t reg_addr,
                                  uint16_t value);

  /**
   * @brief 写多个寄存器（功能码 0x10）
   */
  bool writeMultipleRegisters(uint8_t slave_addr, uint16_t reg_addr,
                              const std::vector<uint16_t> &values);

  /**
   * @brief 写单个线圈（功能码 0x05）
   */
  bool writeSingleCoil(uint8_t slave_addr, uint16_t coil_addr, bool on);

  /**
   * @brief 读线圈（功能码 0x01）
   */
  bool readCoils(uint8_t slave_addr, uint16_t coil_addr,
                 uint16_t qty, std::vector<bool> &coils);

  /**
   * @brief Modbus RTU CRC16 校验
   */
  static uint16_t crc16(const uint8_t *data, size_t len);

private:
  /**
   * @brief 发送一个 Modbus 事务并接收响应（含 CRC 校验、异常码检查）
   *
   * @param slave_addr  从站地址
   * @param pdu_req     请求 PDU（功能码 + 数据，不含从站地址和 CRC）
   * @param rsp_data_len 期望的响应 PDU 中数据部分的字节数（功能码之后，
   *                     含字节计数字段；异常响应会自动按 1 字节处理）
   * @param pdu_rsp     [out] 响应 PDU（功能码 + 数据，不含从站地址和 CRC）
   * @return true 成功
   */
  bool transaction(uint8_t slave_addr, const std::vector<uint8_t> &pdu_req,
                   size_t rsp_data_len, std::vector<uint8_t> &pdu_rsp);

  bool readExact(uint8_t *buf, size_t len, int timeout_ms);

  /// 事务超时时间（毫秒）
  static constexpr int kTimeoutMs = 500;

  int fd_ = -1;
  std::string device_path_;
  /// 总线互斥锁：RS485 半双工，同一时刻只允许一个事务
  std::mutex bus_mutex_;
};

}  // namespace auto_machine

#endif  // AUTO_MACHINE_MODBUS_RTU_HPP
