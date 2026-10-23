#include "auto_machine/modbus_rtu.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace auto_machine {

ModbusRtu::~ModbusRtu() { close(); }

bool ModbusRtu::open(const std::string &device_path) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  close();

  // 检查设备是否存在，如果不存在则等待
  const int max_wait_seconds = 60;  // 最大等待时间
  const int check_interval_ms = 1000; // 检查间隔
  int total_wait_ms = 0;

  while (total_wait_ms < max_wait_seconds * 1000) {
    struct stat st;
    if (stat(device_path.c_str(), &st) == 0) {
      // 设备存在，尝试打开
      break;
    }

    // 设备不存在，等待一段时间再检查
    if (total_wait_ms == 0) {
      // 首次检测到设备不存在，记录日志
      printf("设备 %s 不存在，开始等待...", device_path.c_str());
    }

    usleep(check_interval_ms * 1000);  // 转换为微秒
    total_wait_ms += check_interval_ms;
  }

  // 检查等待超时
  if (total_wait_ms >= max_wait_seconds * 1000) {
    printf("等待设备 %s 超时（%d秒），放弃打开",
                  device_path.c_str(), max_wait_seconds);
    return false;
  }

  // 设备存在，尝试打开
  // O_NONBLOCK 配合 select() 实现超时控制
  fd_ = ::open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    printf("打开设备 %s 失败: %s",
                  device_path.c_str(), strerror(errno));
    return false;
  }
  device_path_ = device_path;

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    close();
    return false;
  }

  // 波特率 115200
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);

  cfmakeraw(&tio);  // 原始模式

  // 数据位 8
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;

  // 奇校验（Odd）：使能校验 + PARODD
  tio.c_cflag |= PARENB;
  tio.c_cflag |= PARODD;

  // 停止位 1
  tio.c_cflag &= ~CSTOPB;

  // 无硬件流控，使能接收
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cflag |= (CREAD | CLOCAL);

  // 原始输入输出，无回显
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tio.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
  tio.c_oflag &= ~OPOST;

  // 非阻塞读：VMIN=0 / VTIME=0，超时由 select() 控制
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

void ModbusRtu::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  device_path_.clear();
}

uint16_t ModbusRtu::crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool ModbusRtu::transaction(uint8_t slave_addr,
                            const std::vector<uint8_t> &pdu_req,
                            size_t rsp_data_len,
                            std::vector<uint8_t> &pdu_rsp) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (fd_ < 0) {
    return false;
  }

  // 组装 ADU：从站地址 + PDU + CRC16（低字节在前）
  std::vector<uint8_t> adu;
  adu.reserve(pdu_req.size() + 3);
  adu.push_back(slave_addr);
  adu.insert(adu.end(), pdu_req.begin(), pdu_req.end());
  const uint16_t crc = crc16(adu.data(), adu.size());
  adu.push_back(static_cast<uint8_t>(crc & 0xFF));
  adu.push_back(static_cast<uint8_t>(crc >> 8));

  tcflush(fd_, TCIOFLUSH);
  if (::write(fd_, adu.data(), adu.size()) != static_cast<ssize_t>(adu.size())) {
    return false;
  }

  // 先读从站地址 + 功能码
  uint8_t head[2] = {0, 0};
  if (!readExact(head, 2, kTimeoutMs)) {
    return false;
  }
  if (head[0] != slave_addr) {
    return false;
  }

  // 异常响应：功能码 | 0x80，后跟 1 字节异常码
  if (head[1] & 0x80) {
    uint8_t tail[3] = {0, 0, 0};
    if (!readExact(tail, 3, kTimeoutMs)) {
      return false;
    }
    const uint8_t frame[5] = {head[0], head[1], tail[0], tail[1], tail[2]};
    if (crc16(frame, 3) != static_cast<uint16_t>(tail[1] | (tail[2] << 8))) {
      return false;
    }
    pdu_rsp.assign(head, head + 2);
    pdu_rsp.push_back(tail[0]);
    return false;  // 调用方通过 pdu_rsp 末尾取异常码
  }

  // 正常响应：读数据部分 + CRC
  std::vector<uint8_t> tail(rsp_data_len + 2, 0);
  if (!readExact(tail.data(), tail.size(), kTimeoutMs)) {
    return false;
  }

  // CRC 校验（从站地址 + 功能码 + 数据）
  std::vector<uint8_t> frame;
  frame.reserve(2 + tail.size());
  frame.push_back(head[0]);
  frame.push_back(head[1]);
  frame.insert(frame.end(), tail.begin(), tail.end() - 2);
  const uint16_t rsp_crc =
      static_cast<uint16_t>(tail[tail.size() - 2] | (tail[tail.size() - 1] << 8));
  if (crc16(frame.data(), frame.size()) != rsp_crc) {
    return false;
  }

  pdu_rsp.assign(head, head + 2);
  pdu_rsp.insert(pdu_rsp.end(), tail.begin(), tail.end() - 2);
  return true;
}

bool ModbusRtu::readExact(uint8_t *buf, size_t len, int timeout_ms) {
  size_t got = 0;
  while (got < len) {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd_, &read_set);

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    const int ret = select(fd_ + 1, &read_set, nullptr, nullptr, &tv);
    if (ret <= 0) {
      return false;  // 超时或出错
    }

    const ssize_t n = ::read(fd_, buf + got, len - got);
    if (n <= 0) {
      return false;
    }
    got += static_cast<size_t>(n);
  }
  return true;
}

bool ModbusRtu::readHoldingRegisters(uint8_t slave_addr, uint16_t reg_addr,
                                     uint16_t qty,
                                     std::vector<uint16_t> &regs) {
  regs.clear();
  if (qty < 1 || qty > 125) {
    return false;
  }

  // 请求 PDU：功能码 0x03 + 起始地址(2) + 数量(2)
  std::vector<uint8_t> req = {0x03,
                              static_cast<uint8_t>(reg_addr >> 8),
                              static_cast<uint8_t>(reg_addr & 0xFF),
                              static_cast<uint8_t>(qty >> 8),
                              static_cast<uint8_t>(qty & 0xFF)};

  // 响应数据部分：字节计数(1) + 2*qty
  std::vector<uint8_t> rsp;
  if (!transaction(slave_addr, req, 1 + qty * 2, rsp) || rsp.size() < 3) {
    return false;
  }
  if (rsp[2] != qty * 2) {
    return false;
  }

  regs.reserve(qty);
  for (uint16_t i = 0; i < qty; ++i) {
    regs.push_back(static_cast<uint16_t>((rsp[3 + i * 2] << 8) | rsp[4 + i * 2]));
  }
  return true;
}

bool ModbusRtu::readInputRegisters(uint8_t slave_addr, uint16_t reg_addr,
                                   uint16_t qty,
                                   std::vector<uint16_t> &regs) {
  regs.clear();
  if (qty < 1 || qty > 125) {
    return false;
  }

  // 请求 PDU：功能码 0x04 + 起始地址(2) + 数量(2)
  std::vector<uint8_t> req = {0x04,
                              static_cast<uint8_t>(reg_addr >> 8),
                              static_cast<uint8_t>(reg_addr & 0xFF),
                              static_cast<uint8_t>(qty >> 8),
                              static_cast<uint8_t>(qty & 0xFF)};

  std::vector<uint8_t> rsp;
  if (!transaction(slave_addr, req, 1 + qty * 2, rsp) || rsp.size() < 3) {
    return false;
  }
  if (rsp[2] != qty * 2) {
    return false;
  }

  regs.reserve(qty);
  for (uint16_t i = 0; i < qty; ++i) {
    regs.push_back(static_cast<uint16_t>((rsp[3 + i * 2] << 8) | rsp[4 + i * 2]));
  }
  return true;
}

bool ModbusRtu::writeSingleRegisterNoReply(uint8_t slave_addr,
                                           uint16_t reg_addr, uint16_t value) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (fd_ < 0) {
    return false;
  }

  // ADU：从站地址 + 功能码0x06 + 寄存器地址(2) + 值(2) + CRC16（低字节在前）
  std::vector<uint8_t> adu = {slave_addr, 0x06,
                              static_cast<uint8_t>(reg_addr >> 8),
                              static_cast<uint8_t>(reg_addr & 0xFF),
                              static_cast<uint8_t>(value >> 8),
                              static_cast<uint8_t>(value & 0xFF)};
  const uint16_t crc = crc16(adu.data(), adu.size());
  adu.push_back(static_cast<uint8_t>(crc & 0xFF));
  adu.push_back(static_cast<uint8_t>(crc >> 8));

  // 发送前只清【输入】方向残留(上一帧回影/垃圾)，绝不清发送方向
  tcflush(fd_, TCIFLUSH);
  ssize_t written = ::write(fd_, adu.data(), adu.size());
  if (written == static_cast<ssize_t>(adu.size())) {
    // 等待内核把帧真正送出(只等待、不丢弃)。
    // 禁止在发送后 tcflush(TCIOFLUSH)：那会连发送队列一起清，在 pty 虚拟串口(及部分真机
    // 时序)下会把刚写出的控制帧整帧冲掉，从站收不到命令 -> 闭环一直轮询到超时(10008)。
    // 总线上的残留回影，由下一次 transaction() 发送前的 tcflush 统一清理。
    tcdrain(fd_);
    return true;
  }
  return false;
}

bool ModbusRtu::writeSingleRegister(uint8_t slave_addr, uint16_t reg_addr,
                                    uint16_t value) {
  // 请求 PDU：功能码 0x06 + 寄存器地址(2) + 值(2)
  std::vector<uint8_t> req = {0x06,
                              static_cast<uint8_t>(reg_addr >> 8),
                              static_cast<uint8_t>(reg_addr & 0xFF),
                              static_cast<uint8_t>(value >> 8),
                              static_cast<uint8_t>(value & 0xFF)};

  // 响应为请求回显：从站地址(1) + 功能码(1) + 寄存器地址(2) + 值(2) = 6 字节
  std::vector<uint8_t> rsp;
  bool success = transaction(slave_addr, req, 4, rsp) && rsp.size() == 6 &&
                 rsp[1] == 0x06;

  // 写操作完成后清除接收缓冲区，避免RS485回影影响后续数据接收
  if (success) {
    tcflush(fd_, TCIOFLUSH);
  }

  return success;
}

bool ModbusRtu::writeMultipleRegisters(uint8_t slave_addr, uint16_t reg_addr,
                                       const std::vector<uint16_t> &values) {
  if (values.empty() || values.size() > 123) {
    return false;
  }

  // 请求 PDU：功能码 0x10 + 起始地址(2) + 数量(2) + 字节数(1) + 数据
  std::vector<uint8_t> req;
  req.reserve(6 + values.size() * 2);
  req.push_back(0x10);
  req.push_back(static_cast<uint8_t>(reg_addr >> 8));
  req.push_back(static_cast<uint8_t>(reg_addr & 0xFF));
  req.push_back(static_cast<uint8_t>(values.size() >> 8));
  req.push_back(static_cast<uint8_t>(values.size() & 0xFF));
  req.push_back(static_cast<uint8_t>(values.size() * 2));
  for (uint16_t v : values) {
    req.push_back(static_cast<uint8_t>(v >> 8));
    req.push_back(static_cast<uint8_t>(v & 0xFF));
  }

  // 响应：从站地址(1) + 功能码(1) + 起始地址(2) + 数量(2) = 6 字节
  std::vector<uint8_t> rsp;
  bool success = transaction(slave_addr, req, 4, rsp) && rsp.size() == 6 &&
                 rsp[1] == 0x10;

  // 写操作完成后清除接收缓冲区，避免RS485回影影响后续数据接收
  if (success) {
    tcflush(fd_, TCIOFLUSH);
  }

  return success;
}

bool ModbusRtu::writeSingleCoil(uint8_t slave_addr, uint16_t coil_addr,
                                bool on) {
  // 请求 PDU：功能码 0x05 + 线圈地址(2) + 值(2，FF00=ON / 0000=OFF)
  const uint16_t val = on ? 0xFF00 : 0x0000;
  std::vector<uint8_t> req = {0x05,
                              static_cast<uint8_t>(coil_addr >> 8),
                              static_cast<uint8_t>(coil_addr & 0xFF),
                              static_cast<uint8_t>(val >> 8),
                              static_cast<uint8_t>(val & 0xFF)};

  std::vector<uint8_t> rsp;
  bool success = transaction(slave_addr, req, 4, rsp) && rsp.size() == 6 &&
                 rsp[1] == 0x05;

  // 写操作完成后清除接收缓冲区，避免RS485回影影响后续数据接收
  if (success) {
    tcflush(fd_, TCIOFLUSH);
  }

  return success;
}

bool ModbusRtu::readCoils(uint8_t slave_addr, uint16_t coil_addr, uint16_t qty,
                          std::vector<bool> &coils) {
  coils.clear();
  if (qty < 1 || qty > 2000) {
    return false;
  }

  // 请求 PDU：功能码 0x01 + 起始地址(2) + 数量(2)
  std::vector<uint8_t> req = {0x01,
                              static_cast<uint8_t>(coil_addr >> 8),
                              static_cast<uint8_t>(coil_addr & 0xFF),
                              static_cast<uint8_t>(qty >> 8),
                              static_cast<uint8_t>(qty & 0xFF)};

  // 响应数据部分：字节计数(1) + ceil(qty/8)
  const size_t byte_cnt = (qty + 7) / 8;
  std::vector<uint8_t> rsp;
  if (!transaction(slave_addr, req, 1 + byte_cnt, rsp) || rsp.size() < 3) {
    return false;
  }

  coils.resize(qty, false);
  for (uint16_t i = 0; i < qty; ++i) {
    coils[i] = (rsp[3 + i / 8] >> (i % 8)) & 0x01;
  }
  return true;
}

}  // namespace auto_machine
