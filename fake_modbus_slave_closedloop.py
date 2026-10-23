#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 假 Modbus RTU 从站（闭环测试专用版）  fake_modbus_slave_closedloop.py
================================================================================
用途：
  在没有真实硬件的 PC 上，配合 auto_machine 节点测试【控制命令闭环】。

与旧脚本 fake_modbus_slave.py 的两点关键区别：
  1) 寄存器按【从站地址】和【地址空间】分开建模：
       - 保持寄存器空间：功能码 0x03 读 / 0x06、0x10 写
       - 输入寄存器空间：功能码 0x04 读
     V6、XYZR 的“控制寄存器”和“状态寄存器”地址相同（都从 0x0000 开始），
     但属于两个独立空间，旧脚本用一张表会互相覆盖，本脚本不会。
  2) 收到闭环控制命令后，用后台定时器延迟把对应的【状态寄存器】翻到“完成值”，
     模拟真实设备动作，从而让节点的闭环等待走到 success=true / err_code=0。
     （旧脚本收到控制后状态永远不变，闭环只能等到超时返回 10008。）

运行（无需 ROS 环境，纯 Python3）：
  python3 fake_modbus_slave_closedloop.py
启动后会打印：
  NODE_PORT=/dev/pts/N
把这个 /dev/pts/N 作为 auto_machine 节点的 serial_port 参数即可。

说明：
  - 从站地址：T1/V6/MAG04/MAG06 = 0x01，XYZR = 0x02（与真机一致）。
  - 状态翻转有 0.8~1.8 秒的模拟动作延时，节点会先查到“未到位”，轮询几次后
    查到“到位”，这正是闭环要验证的过程。
  - Ctrl+C 退出。
================================================================================
"""

import os
import pty
import sys
import threading

# ------------------------------------------------------------------ 寄存器地址
# T1（slave=1，控制/状态都在保持寄存器空间 0x03/0x06）
T1_RUN_STATUS      = 0x1000
T1_SUCTION_STATUS  = 0x1014
T1_WORK_CTRL       = 0x1501
T1_SUCTION_CTRL    = 0x1502
T1_RESET           = 0x1530

# V6（slave=1，控制写保持寄存器，状态读【输入寄存器】0x04）
V6_TONGUE     = 0x0000   # 舌板伸出（保持）
V6_FOLD       = 0x0001   # 开始折叠（保持）
V6_FOLD_RESET = 0x0008   # 折叠复位（保持）
V6_BREEZE     = 0x0009   # 鼓风机（保持）
V6_INHALE     = 0x000A   # 吸布（保持）
V6_FOLD_STATUS  = 0x0002  # 折叠状态（输入）
V6_BREEZE_STATE = 0x0003  # 鼓风状态（输入）
V6_INHALE_STATE = 0x0004  # 吸布状态（输入）

# MAG04（slave=1，保持寄存器空间）
M4_CTRL = 0x0064   # 主控制命令
M4_CLOTH = 0x0068  # 理布控制命令
M4_ADMISSION = 0x0066
M4_XIFENG    = 0x0067
M4_FUPING    = 0x0069

# MAG06（slave=1，地址整体 +0x1000）
M6_CTRL = 0x1064
M6_CLOTH = 0x1068
M6_ADMISSION = 0x1066
M6_XIFENG    = 0x1067
M6_FUPING    = 0x1069

# XYZR（slave=2，控制写保持寄存器，状态读【输入寄存器】0x04）
XY_RESET       = 0x0000  # 保持：复位
XY_CLOTH_READY = 0x0001  # 保持：布料就位/触发检测
XY_ADJUST_CLOTH = 0x0001  # 输入：移料结果
XY_MASTER_STATUS = 0x0002  # 输入：主控状态
XY_FAULT_CODE   = 0x0003  # 输入：故障代码

# 动作步骤：(延时秒, 空间'h'保持/'i'输入, 状态寄存器, 操作'set'赋值/'or'置位, 值)
# key = (从站地址, 被写的控制寄存器, 写入值)
ACTIONS = {
    # ---------------- T1 ----------------
    (1, T1_RESET, 1):        [(1.0, 'h', T1_RUN_STATUS, 'or', 0x01)],
    (1, T1_WORK_CTRL, 1):    [(1.3, 'h', T1_RUN_STATUS, 'or', 0x02)],
    (1, T1_SUCTION_CTRL, 1): [(0.8, 'h', T1_SUCTION_STATUS, 'set', 1)],
    (1, T1_SUCTION_CTRL, 0): [(0.8, 'h', T1_SUCTION_STATUS, 'set', 0)],
    # ---------------- V6 ----------------
    (1, V6_TONGUE, 1):       [(0.8, 'i', V6_FOLD_STATUS, 'set', 10)],
    (1, V6_FOLD, 1):         [(1.2, 'i', V6_FOLD_STATUS, 'set', 26)],
    (1, V6_FOLD_RESET, 1):   [(0.8, 'i', V6_FOLD_STATUS, 'set', 0)],
    (1, V6_BREEZE, 1):       [(0.8, 'i', V6_BREEZE_STATE, 'set', 1)],
    (1, V6_BREEZE, 2):       [(0.8, 'i', V6_BREEZE_STATE, 'set', 0)],
    (1, V6_INHALE, 1):       [(0.8, 'i', V6_INHALE_STATE, 'set', 1)],
    (1, V6_INHALE, 2):       [(0.8, 'i', V6_INHALE_STATE, 'set', 0)],
    # ---------------- MAG04 ----------------
    (1, M4_CTRL, 4):         [(1.2, 'h', M4_ADMISSION, 'set', 1)],   # 复位->已复位
    (1, M4_CTRL, 2):         [(1.8, 'h', M4_ADMISSION, 'set', 3)],   # 缝制->完成
    (1, M4_CTRL, 1):         [(0.8, 'h', M4_XIFENG, 'set', 1)],      # 开吸风
    (1, M4_CTRL, 5):         [(0.8, 'h', M4_XIFENG, 'set', 0)],      # 关吸风
    (1, M4_CLOTH, 1):        [(0.8, 'h', M4_FUPING, 'set', 1)],      # 开始理布
    (1, M4_CLOTH, 2):        [(0.8, 'h', M4_FUPING, 'set', 2)],      # 关闭理布
    (1, M4_CLOTH, 3):        [(0.8, 'h', M4_FUPING, 'set', 2)],      # 关闭气缸
    # ---------------- MAG06 ----------------
    (1, M6_CTRL, 4):         [(1.2, 'h', M6_ADMISSION, 'set', 1)],
    (1, M6_CTRL, 2):         [(1.8, 'h', M6_ADMISSION, 'set', 3)],
    (1, M6_CTRL, 1):         [(0.8, 'h', M6_XIFENG, 'set', 1)],
    (1, M6_CTRL, 5):         [(0.8, 'h', M6_XIFENG, 'set', 0)],
    (1, M6_CLOTH, 1):        [(0.8, 'h', M6_FUPING, 'set', 1)],
    (1, M6_CLOTH, 2):        [(0.8, 'h', M6_FUPING, 'set', 2)],
    (1, M6_CLOTH, 3):        [(0.8, 'h', M6_FUPING, 'set', 2)],
    # ---------------- XYZR（两阶段） ----------------
    # 复位：主控先 10(执行中)，再 1(就绪)
    (2, XY_RESET, 1):
        [(0.1, 'i', XY_MASTER_STATUS, 'set', 10),
         (1.5, 'i', XY_MASTER_STATUS, 'set', 1)],
    # 触发检测：主控先 10，再 11(完成)，同时移料结果=1(成功)
    (2, XY_CLOTH_READY, 1):
        [(0.1, 'i', XY_MASTER_STATUS, 'set', 10),
         (1.5, 'i', XY_MASTER_STATUS, 'set', 11),
         (1.5, 'i', XY_ADJUST_CLOTH, 'set', 1)],
}


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


class FakeSlave:
    def __init__(self, fd):
        self.fd = fd
        self.lock = threading.Lock()
        # hold[slave][addr] 保持寄存器； inp[slave][addr] 输入寄存器
        self.hold = {1: {}, 2: {}}
        self.inp = {1: {}, 2: {}}
        # XYZR 主控初始给 0(待机)，复位闭环才能看到 0->10->1 的过程
        self.inp[2][XY_MASTER_STATUS] = 0
        self.inp[2][XY_ADJUST_CLOTH] = 0
        self.inp[2][XY_FAULT_CODE] = 0

    def _table(self, space):
        return self.hold if space == 'h' else self.inp

    def get(self, space, slave, addr):
        with self.lock:
            return self._table(space).setdefault(slave, {}).get(addr, 0)

    def set(self, space, slave, addr, op, val, tag=''):
        with self.lock:
            d = self._table(space).setdefault(slave, {})
            if op == 'or':
                d[addr] = (d.get(addr, 0) | val) & 0xFFFF
            else:
                d[addr] = val
            now = d[addr]
        sp = '保持' if space == 'h' else '输入'
        print('  [动作模拟] 从站%d %s寄存器 0x%04X -> %u (0x%04X) %s'
              % (slave, sp, addr, now, now, tag), flush=True)

    def schedule(self, slave, steps):
        for delay, space, reg, op, val in steps:
            t = threading.Timer(delay, self.set,
                                args=(space, slave, reg, op, val))
            t.daemon = True
            t.start()

    def read_exact(self, n):
        buf = b''
        while len(buf) < n:
            chunk = os.read(self.fd, n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def reply(self, slave, body):
        frame = bytes([slave]) + body
        crc = crc16(frame)
        os.write(self.fd, frame + bytes([crc & 0xFF, crc >> 8]))

    def handle(self, frame):
        slave = frame[0]
        func = frame[1]
        body = frame[1:-2]  # 去掉从站地址和 CRC，保留功能码+数据

        if func == 0x03 or func == 0x04:  # 读保持/输入寄存器
            addr = (body[1] << 8) | body[2]
            qty = (body[3] << 8) | body[4]
            space = 'h' if func == 0x03 else 'i'
            vals = [self.get(space, slave, addr + i) & 0xFFFF for i in range(qty)]
            data = b''.join(bytes([v >> 8, v & 0xFF]) for v in vals)
            self.reply(slave, bytes([func, qty * 2]) + data)
            sp = '保持' if func == 0x03 else '输入'
            print('[RX OK ] %s 从站=%d 读%s寄存器 0x%04X x%d => %s'
                  % (frame[:-2].hex(' ').upper(), slave, sp, addr, qty,
                     ' '.join('%04X' % v for v in vals)), flush=True)

        elif func == 0x06:  # 写单个保持寄存器
            addr = (body[1] << 8) | body[2]
            val = (body[3] << 8) | body[4]
            with self.lock:
                self.hold.setdefault(slave, {})[addr] = val
            self.reply(slave, body)  # 原样回显（XYZR 控制依赖此应答）
            print('[RX OK ] %s 从站=%d 写保持寄存器 0x%04X = %u'
                  % (frame[:-2].hex(' ').upper(), slave, addr, val), flush=True)
            steps = ACTIONS.get((slave, addr, val))
            if steps:
                self.schedule(slave, steps)

        elif func == 0x10:  # 写多个保持寄存器（本工程基本不用，支持即可）
            addr = (body[1] << 8) | body[2]
            qty = (body[3] << 8) | body[4]
            with self.lock:
                d = self.hold.setdefault(slave, {})
                for i in range(qty):
                    d[addr + i] = (body[6 + i * 2] << 8) | body[7 + i * 2]
            self.reply(slave, bytes([0x10, body[1], body[2], body[3], body[4]]))
            print('[RX OK ] %s 从站=%d 写多个保持寄存器 0x%04X x%d'
                  % (frame[:-2].hex(' ').upper(), slave, addr, qty), flush=True)

        elif func == 0x05:  # 写单个线圈（本工程未用，回显即可）
            self.reply(slave, body)
            print('[RX OK ] %s 从站=%d 写线圈'
                  % frame[:-2].hex(' ').upper(), flush=True)

        else:
            print('[未支持] 功能码 0x%02X，帧: %s'
                  % (func, frame.hex(' ').upper()), flush=True)

    def run(self):
        while True:
            head = self.read_exact(1)  # 第 1 字节：从站地址
            if not head:
                continue
            slave = head[0]
            fb = self.read_exact(1)    # 第 2 字节：功能码
            if not fb:
                continue
            func = fb[0]

            if func in (0x03, 0x04):
                rest = self.read_exact(6)  # addr2+qty2+crc2
            elif func == 0x06 or func == 0x05:
                rest = self.read_exact(6)  # addr2+val2+crc2
            elif func == 0x10:
                fixed = self.read_exact(5)  # addr2+qty2+bytecnt1
                if not fixed:
                    continue
                data_len = fixed[4]
                data = self.read_exact(data_len + 2)  # data + crc2
                if not data:
                    continue
                rest = fixed + data
            else:
                # 未知功能码，丢弃到超时，简单处理
                print('[未支持] 功能码 0x%02X' % func, flush=True)
                continue

            if rest is None:
                continue
            self.handle(bytes([slave, func]) + rest)


def main():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    print('===== 假 Modbus RTU 从站（闭环测试版）已启动 =====')
    print('NODE_PORT=%s' % slave_name)
    print('请把上面的路径作为 auto_machine 节点的 serial_port 参数')
    print('本从站会在收到控制命令后自动把状态翻到“完成值”，闭环应返回 success=true')
    print('等待请求...（Ctrl+C 退出）\n', flush=True)
    FakeSlave(master_fd).run()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n已退出')
        sys.exit(0)
