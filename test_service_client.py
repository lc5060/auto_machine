#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
auto_machine 服务调用测试客户端。

用法（需先 source ROS2 环境 + 本包 install/setup.bash）：

    # 跑内置的用例表（覆盖 T1/V6/MAG04/MAG06/XYZR，默认 0 号机）
    python3 test_service_client.py

    # 单条调用：python3 test_service_client.py <dev_type十六进制> <command十六进制> [参数字节...]
    python3 test_service_client.py 0x00000003 0x00030001 00   # MAG04 开始吸风，0 号机
    python3 test_service_client.py 0x00000003 0x00030001 01   # 1 号机

param_data[0] = 自动机索引（0/1...），对应 mag_serial_ports 的串口顺序；
T1/V6/XYZR 省略时默认 0 号，MAG04/MAG06 必须显式提供。

每个用例都会打印请求与响应（success / err_code / ret_data 十六进制）。
真正的指令帧对不对，看 fake_modbus_slave.py 那侧打印的 [RX OK] 原始十六进制帧。
"""

import sys
import rclpy
from rclpy.node import Node

from auto_machine.srv import AutoMachine

# ---------------- 设备类型 / 命令号常量（见 msg/AutoMachineDef.msg） ----------------
DEV_T1 = 0x00000001
DEV_V6 = 0x00000002
DEV_MAG04 = 0x00000003
DEV_MAG06 = 0x00000004
DEV_XYZR = 0x10000001

# (名称, dev_type, command, param_data 字节列表)
TEST_CASES = [
    ("T1 开始工作",       DEV_T1,    0x00010009, [0]),
    ("T1 关闭吸风",       DEV_T1,    0x00010010, [0]),
    ("T1 状态查询",       DEV_T1,    0x00010001, [0]),
    ("V6 舌板伸出",       DEV_V6,    0x00020001, [0]),
    ("V6 机器状态查询",   DEV_V6,    0x00020016, [0]),
    ("MAG04 开始吸风",    DEV_MAG04, 0x00030001, [0]),
    ("MAG04 急停",        DEV_MAG04, 0x00030003, [0]),
    ("MAG04 查询吸风状态", DEV_MAG04, 0x0003000B, [0]),
    ("MAG04 查询准入状态", DEV_MAG04, 0x0003000D, [0]),
    ("MAG06 开始吸风",    DEV_MAG06, 0x00040001, [0]),
    ("XYZR 复位",         DEV_XYZR,  0x01010001, [0]),
    ("XYZR 主控状态查询", DEV_XYZR,  0x01010004, [0]),
]


def hex_bytes(data):
    return ' '.join('%02X' % b for b in data) if data else '(空)'


class TestClient(Node):
    def __init__(self):
        super().__init__('auto_machine_test_client')
        self.cli = self.create_client(AutoMachine, 'auto_machine/command')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('等待服务 auto_machine/command ...')

    def call(self, name, dev_type, command, param):
        req = AutoMachine.Request()
        req.dev_type = dev_type
        req.command = command
        req.param_data = list(param)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error('[%s] 调用超时（无响应）' % name)
            return
        resp = future.result()
        print('[%s] dev=0x%08X cmd=0x%08X param=%s -> success=%s err_code=%d ret_data=%s'
              % (name, dev_type, command, hex_bytes(param),
                 resp.success, resp.err_code, hex_bytes(resp.ret_data)))


def main():
    rclpy.init()
    client = TestClient()

    if len(sys.argv) >= 3:
        # 单条调用模式
        dev_type = int(sys.argv[1], 16)
        command = int(sys.argv[2], 16)
        param = [int(x, 16) for x in sys.argv[3:]]
        client.call('manual', dev_type, command, param)
    else:
        # 内置用例表
        for name, dev, cmd, param in TEST_CASES:
            client.call(name, dev, cmd, param)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
