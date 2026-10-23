#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
假 Modbus RTU 从站 —— 用于无硬件模拟测试 auto_machine 节点。

用法（Linux 上）：
    python3 fake_modbus_slave.py

脚本会创建一对虚拟串口，并打印 NODE_PORT=/dev/pts/N，
把该路径作为 auto_machine 节点的 serial_port 参数即可。

支持功能码：
    0x03 读保持寄存器 —— 应答（含正确 CRC）
    0x04 读输入寄存器   —— 应答（V6 / XYZR 查询用）
    0x05 写单个线圈     —— 应答回显
    0x06 写单个寄存器   —— 记录并应答回显（MAG/V6/T1 的 NoReply 回显会被节点 flush 掉）
    0x10 写多个寄存器   —— 记录并应答

每个收到的帧都会打印原始十六进制（含 CRC 校验结果），便于逐字节核对指令帧。
"""

import os
import pty
import sys

# 寄存器地址 -> 名称（方便日志里看懂写的是哪个寄存器）
REG_NAMES = {
    # MAG04 (0x0064 ~ 0x006B)
    0x0064: 'D100 主控制', 0x0065: 'D101 故障码', 0x0066: 'D102 准入/复位',
    0x0067: 'D103 吸风状态', 0x0068: 'D104 摆料气缸/理布', 0x0069: 'D105 摆料气缸状态',
    0x006A: 'D106 吹气', 0x006B: 'D107 压板',
    # MAG06 (0x1064 ~ 0x106B)
    0x1064: 'D100 主控制', 0x1065: 'D101 故障码', 0x1066: 'D102 准入/复位',
    0x1067: 'D103 吸风状态', 0x1068: 'D104 摆料气缸/理布', 0x1069: 'D105 摆料气缸状态',
    0x106A: 'D106 吹气', 0x106B: 'D107 压板',
    # XYZR (从站地址 0x02，保持寄存器 0x0000~0x0003 / 输入寄存器 0x0000~0x0002)
    # 注：保持寄存器与输入寄存器地址空间独立，此处统一标注，写时看保持含义、读时看输入含义
    0x0000: 'XYZR 复位(保持) / 预留(输入)',
    0x0001: 'XYZR 布料已放好(保持) / 移料状态(输入)',
    0x0002: 'XYZR X轴移料距离(保持) / 主控状态(输入)',
    0x0003: 'XYZR Y轴移料距离(保持) / 故障代码(输入)',
}


def crc16(data):
    """Modbus RTU CRC16（多项式 0xA001，初始 0xFFFF）"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def read_exact(fd, n):
    buf = b''
    while len(buf) < n:
        d = os.read(fd, n - len(buf))
        if not d:
            break
        buf += d
    return buf


class FakeSlave:
    def __init__(self, fd):
        self.fd = fd
        self.regs = {a: 0 for a in REG_NAMES}
        # 预设几个查询状态，方便看到非零返回值
        self.regs[0x0066] = 1   # 准入状态 = 已复位
        self.regs[0x0067] = 1   # 吸风状态 = 开启
        self.regs[0x1066] = 1
        self.regs[0x1067] = 1
        # XYZR 预设状态（从站 0x02）
        self.regs[0x0001] = 1   # 移料状态 = 移料成功
        self.regs[0x0002] = 1   # 主控状态 = 准备就绪

    def _name(self, addr):
        return REG_NAMES.get(addr, '0x%04X' % addr)

    def run(self):
        while True:
            h = read_exact(self.fd, 2)   # 从站地址 + 功能码
            if len(h) < 2:
                return
            slave, func = h[0], h[1]

            if func in (0x01, 0x02, 0x03, 0x04, 0x05, 0x06):
                body = read_exact(self.fd, 6)   # 剩余 6 字节（含 CRC）
                if len(body) < 6:
                    return
                self.handle(slave, func, body)
            elif func == 0x10:
                hdr = read_exact(self.fd, 5)    # 地址(2)+数量(2)+字节数(1)
                if len(hdr) < 5:
                    return
                bc = hdr[4]
                data = read_exact(self.fd, bc + 2)   # 数据 + CRC
                if len(data) < bc + 2:
                    return
                self.handle(slave, func, hdr + data)
            else:
                print('[!] 未知功能码 0x%02X，忽略' % func)

    def handle(self, slave, func, body):
        data = body[:-2]
        crc_recv = body[-2] | (body[-1] << 8)
        crc_ok = (crc16(bytes([slave, func]) + data) == crc_recv)

        # 始终打印原始帧（从站地址 + 功能码 + 数据 + CRC），便于逐字节核对指令帧
        full = bytes([slave, func]) + body
        hex_str = ' '.join('%02X' % b for b in full)
        print('  [RX %s] %s  (从站=%d 功能码=0x%02X)'
              % ('OK ' if crc_ok else 'BAD', hex_str, slave, func))
        if not crc_ok:
            print('  [!] CRC 校验失败，丢弃帧')
            return

        if func == 0x06:
            addr = (data[0] << 8) | data[1]
            val = (data[2] << 8) | data[3]
            self.regs[addr] = val
            print('      <- 写单寄存器  %s = %d (0x%04X)'
                  % (self._name(addr), val, val))
            # 标准 Modbus 0x06 应答回显。XYZR 控制命令用 writeSingleRegister
            # （带应答），需要此回显；MAG/V6/T1 的 NoReply 回显会被节点 tcflush 丢弃。
            self._reply(slave, bytes([0x06, data[0], data[1], data[2], data[3]]))

        elif func in (0x03, 0x04):
            kind = '读保持寄存器' if func == 0x03 else '读输入寄存器'
            addr = (data[0] << 8) | data[1]
            qty = (data[2] << 8) | data[3]
            print('      -> %s  %s x%d' % (kind, self._name(addr), qty))
            out = bytearray()
            for i in range(qty):
                v = self.regs.get(addr + i, 0)
                out += bytes([v >> 8, v & 0xFF])
            pdu = bytes([func, len(out)]) + bytes(out)
            self._reply(slave, pdu)

        elif func == 0x10:
            addr = (data[0] << 8) | data[1]
            qty = (data[2] << 8) | data[3]
            print('      <- 写多寄存器  %s x%d' % (self._name(addr), qty))
            pdu = bytes([0x10, data[0], data[1], data[2], data[3]])
            self._reply(slave, pdu)

        elif func == 0x05:
            addr = (data[0] << 8) | data[1]
            val = (data[2] << 8) | data[3]
            print('      <- 写单线圈  %s = %s'
                  % (self._name(addr), 'ON' if val == 0xFF00 else 'OFF'))
            self._reply(slave, bytes([0x05, data[0], data[1], data[2], data[3]]))

        else:
            print('      [!] 未处理的功能码 0x%02X' % func)

    def _reply(self, slave, pdu):
        adu = bytes([slave]) + pdu
        c = crc16(adu)
        os.write(self.fd, adu + bytes([c & 0xFF, c >> 8]))


def main():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    print('===== 假 Modbus RTU 从站已启动 =====')
    print('NODE_PORT=%s' % slave_name)
    print('请把上面的路径作为 auto_machine 节点的 serial_port 参数')
    print('等待请求...\n')
    sys.stdout.flush()
    FakeSlave(master_fd).run()


if __name__ == '__main__':
    main()
