#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XYZR 移料装置专用测试客户端。

覆盖 XYZR 全部 7 条命令，逐条调用 /auto_machine/command 服务，
并打印期望的 Modbus 帧（含 CRC），方便与 fake_modbus_slave.py 侧
打印的 [RX OK] 原始帧逐字节核对。

多机说明：
  param_data[0] 为设备索引（0/1...），对应 mag_serial_ports 的串口顺序；
  省略时默认 0 号（兼容单机）。

用法（需先 source ROS2 环境 + 本包 install/setup.bash）：

    # 跑全部 7 条用例（0 号机）
    python3 test_xyzr_client.py

    # 单条调用：python3 test_xyzr_client.py <command十六进制> [参数字节...]
    python3 test_xyzr_client.py 0x01010001
    python3 test_xyzr_client.py 0x01010001 01     # 1 号机的对齐装置复位

XYZ R 协议要点：
  - 从站地址：0x02
  - 控制命令：功能码 0x06 写保持寄存器（带应答，与其他设备的 NoReply 不同）
  - 查询命令：功能码 0x04 读输入寄存器
  - 串口参数：115200 8O1 RTU
"""

import sys
import rclpy
from rclpy.node import Node

from auto_machine.srv import AutoMachine

# XYZR 设备类型
DEV_XYZR = 0x10000001

# Modbus CRC16（多项式 0xA001，初始 0xFFFF）
def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

# 组装完整 Modbus RTU 帧并返回十六进制字符串
def build_frame(slave, func, reg_addr, value_or_qty, is_write=True):
    """is_write=True: 0x06 写单寄存器；is_write=False: 0x04 读输入寄存器"""
    if is_write:
        pdu = bytes([func, (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
                     (value_or_qty >> 8) & 0xFF, value_or_qty & 0xFF])
    else:
        pdu = bytes([func, (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
                     (value_or_qty >> 8) & 0xFF, value_or_qty & 0xFF])
    adu = bytes([slave]) + pdu
    c = crc16(adu)
    full = adu + bytes([c & 0xFF, c >> 8])
    return ' '.join('%02X' % b for b in full)


# (用例名称, command, param_data, 期望帧说明, 寄存器地址, 写值/读数量, 是写还是读)
TEST_CASES = [
    (
        "XYZR 复位",
        0x01010001,
        [0],
        "写保持寄存器 0x0000 = 0x0001（主控回零）",
        0x0000, 0x0001, True,
    ),
    (
        "XYZR 触发视觉检测（布料已放好）",
        0x01010002,
        [0],
        "写保持寄存器 0x0001 = 0x0001（触发视觉检测）",
        0x0001, 0x0001, True,
    ),
    (
        "XYZR 移料状态查询",
        0x01010003,
        [0],
        "读输入寄存器 0x0001 x1（0=移料失败，1=移料成功）",
        0x0001, 0x0001, False,
    ),
    (
        "XYZR 主控状态查询",
        0x01010004,
        [0],
        "读输入寄存器 0x0002 x1（0=待机,1=就绪,10=执行中,11=完成,20=暂停,100=故障）",
        0x0002, 0x0001, False,
    ),
    (
        "XYZR 故障代码查询",
        0x01010007,
        [0],
        "读输入寄存器 0x0003 x1（故障代码）",
        0x0003, 0x0001, False,
    ),
    (
        "XYZR X轴第一次移料距离",
        0x01010005,
        [0],
        "写保持寄存器 0x0002 = 0x0001（注意：当前代码写死1mm，未使用param_data）",
        0x0002, 0x0001, True,
    ),
    (
        "XYZR Y轴第一次移料距离",
        0x01010006,
        [0],
        "写保持寄存器 0x0003 = 0x0001（注意：当前代码写死1mm，未使用param_data）",
        0x0003, 0x0001, True,
    ),
]


def hex_bytes(data):
    return ' '.join('%02X' % b for b in data) if data else '(空)'


class XyzrTestClient(Node):
    def __init__(self):
        super().__init__('xyzr_test_client')
        self.cli = self.create_client(AutoMachine, 'auto_machine/command')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('等待服务 auto_machine/command ...')

    def call(self, name, command, param):
        req = AutoMachine.Request()
        req.dev_type = DEV_XYZR
        req.command = command
        req.param_data = list(param)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error('[%s] 调用超时（无响应）' % name)
            return
        resp = future.result()
        print('[%s]' % name)
        print('  请求: dev_type=0x%08X command=0x%08X param=%s'
              % (DEV_XYZR, command, hex_bytes(param)))
        print('  响应: success=%s err_code=%d ret_data=%s'
              % (resp.success, resp.err_code, hex_bytes(resp.ret_data)))


def main():
    rclpy.init()
    client = XyzrTestClient()

    print('=' * 70)
    print('XYZR 移料装置测试 —— 从站地址 0x02，控制=0x06(带应答)，查询=0x04')
    print('=' * 70)
    print()

    if len(sys.argv) >= 2:
        # 单条调用模式
        command = int(sys.argv[1], 16)
        param = [int(x, 16) for x in sys.argv[2:]]
        # 查找用例信息用于打印期望帧
        case_info = None
        for c in TEST_CASES:
            if c[1] == command:
                case_info = c
                break
        if case_info:
            name, _, _, desc, reg, val, is_write = case_info
            frame = build_frame(0x02, 0x06 if is_write else 0x04, reg, val, is_write)
            print('期望帧: %s' % frame)
            print('说明: %s' % desc)
            print()
            client.call(name, command, param)
        else:
            client.call('manual', command, param)
    else:
        # 跑全部用例
        for idx, (name, command, param, desc, reg, val, is_write) in enumerate(TEST_CASES, 1):
            frame = build_frame(0x02, 0x06 if is_write else 0x04, reg, val, is_write)
            print('--- [%d/%d] %s ---' % (idx, len(TEST_CASES), name))
            print('  期望帧: %s' % frame)
            print('  说明: %s' % desc)
            client.call(name, command, param)
            print()

    print('=' * 70)
    print('测试完成。请切到 fake_modbus_slave.py 终端核对 [RX OK] 帧是否与上表一致。')
    print('控制类(0x06): ret_data 应为空；查询类(0x04): ret_data 应有 2 字节。')
    print('=' * 70)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
