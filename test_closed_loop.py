#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 auto_machine 闭环功能测试客户端  test_closed_loop.py
================================================================================
用法：
  1) 批量自测（推荐，依次跑全部 23 条闭环命令并自动判定到位）：
       python3 test_closed_loop.py
  2) 单条测试：
       python3 test_closed_loop.py <设备类型hex> <命令号hex> [参数字节hex...]
     例：python3 test_closed_loop.py 0x02 0x00020001 00       # V6 舌板伸出（0 号机）
         python3 test_closed_loop.py 0x10000001 0x01010002 00  # XYZR 触发视觉检测（0 号机）

参数说明：
  param_data[0] = 自动机索引（0/1...），对应 mag_serial_ports 的串口顺序；
  T1/V6/XYZR 省略时默认 0 号，MAG04/MAG06 必须显式提供。
  内置用例统一用 [0]（0 号机）；测 1 号机时把用例里的 [0] 改成 [1]。

前提（三个终端）：
  终端1：python3 fake_modbus_slave_closedloop.py   （闭环专用从站，会模拟动作到位）
  终端2：ros2 run auto_machine auto_machine_node --ros-args -p serial_port:=/dev/pts/N
  终端3：本脚本（运行前先 source install/setup.bash）

预期：
  配合【闭环专用从站】：23 条全部 PASS（success=true, err_code=0, ret_data 到位值）。
  若用旧 fake_modbus_slave.py：闭环命令会在约 30 秒后返回 err_code=10008（超时）。

判定说明：
  success=true 且 err_code=0 且 ret_data 达到目标值 => PASS（动作确实到位）
  err_code=10006 => 通讯失败（检查从站/串口）
  err_code=10008 => 动作超时未到位（状态一直没翻到目标值）
================================================================================
"""

import sys
import time

import rclpy
from rclpy.node import Node
from auto_machine.srv import AutoMachine

# 设备类型
DEV_T1 = 1
DEV_V6 = 2
DEV_MAG04 = 3
DEV_MAG06 = 4
DEV_XYZR = 0x10000001

ERR_TEXT = {
    0: '成功（动作到位）',
    10001: '未知设备类型',
    10002: '未知命令',
    10003: '参数错误',
    10004: '功能未实现',
    10005: '内部错误',
    10006: '通讯失败（串口/应答异常）',
    10007: '设备异常',
    10008: '动作超时未到位（闭环等待超时）',
}

# 判定方式：'eq' 等于目标值；'nonzero' 非0；'bit' 指定 bit 置1
# (名称, dev_type, command, 参数, 判定, 目标值)
CASES = [
    # ---------------- T1 ----------------
    ('T1 复位(等回零完成)',      DEV_T1, 0x0001000E, [0], 'bit', 0x01),
    ('T1 开始工作(等缝纫完成)',  DEV_T1, 0x00010009, [0], 'bit', 0x02),
    ('T1 开吸风(等吸风开)',      DEV_T1, 0x0001000C, [0], 'nonzero', 0),
    ('T1 关吸风(等吸风关)',      DEV_T1, 0x00010010, [0], 'eq', 0),
    # ---------------- V6（顺序：舌板->折叠->复位） ----------------
    ('V6 舌板伸出(等伸出到位)',  DEV_V6, 0x00020001, [0], 'eq', 10),
    ('V6 开始折叠(等折叠完成)',  DEV_V6, 0x00020002, [0], 'eq', 26),
    ('V6 折叠复位(等复位完成)',  DEV_V6, 0x0002000E, [0], 'eq', 0),
    ('V6 开鼓风机(等鼓风开)',    DEV_V6, 0x0002000F, [0], 'eq', 1),
    ('V6 关鼓风机(等鼓风关)',    DEV_V6, 0x00020010, [0], 'eq', 0),
    ('V6 开始吸布(等吸布开)',    DEV_V6, 0x00020011, [0], 'eq', 1),
    ('V6 停止吸布(等吸布关)',    DEV_V6, 0x00020012, [0], 'eq', 0),
    # ---------------- MAG04 ----------------
    ('MAG04 复位(等复位完成)',   DEV_MAG04, 0x00030005, [0], 'eq', 1),
    ('MAG04 开始缝制(等缝制完成)', DEV_MAG04, 0x00030002, [0], 'eq', 3),
    ('MAG04 开吸风(等吸风开)',   DEV_MAG04, 0x00030001, [0], 'nonzero', 0),
    ('MAG04 关吸风(等吸风关)',   DEV_MAG04, 0x0003000F, [0], 'eq', 0),
    ('MAG04 关闭气缸(等复位)',   DEV_MAG04, 0x00030008, [0], 'eq', 2),
    # ---------------- MAG06 ----------------
    ('MAG06 复位(等复位完成)',   DEV_MAG06, 0x00040005, [0], 'eq', 1),
    ('MAG06 开始缝制(等缝制完成)', DEV_MAG06, 0x00040002, [0], 'eq', 3),
    ('MAG06 开吸风(等吸风开)',   DEV_MAG06, 0x00040001, [0], 'nonzero', 0),
    ('MAG06 关吸风(等吸风关)',   DEV_MAG06, 0x0004000F, [0], 'eq', 0),
    ('MAG06 关闭气缸(等复位)',   DEV_MAG06, 0x00040008, [0], 'eq', 2),
    # ---------------- XYZR ----------------
    ('XYZR 复位(主控回到就绪=1)', DEV_XYZR, 0x01010001, [0], 'eq', 1),
    ('XYZR 触发视觉检测(移料成功=1)', DEV_XYZR, 0x01010002, [0], 'eq', 1),
]


def ret_value(ret_data):
    """把 2 字节大端 ret_data 还原成寄存器数值；不足 2 字节返回 None。"""
    if len(ret_data) >= 2:
        return (ret_data[0] << 8) | ret_data[1]
    return None


def judge(kind, target, val):
    if val is None:
        return False
    if kind == 'eq':
        return val == target
    if kind == 'nonzero':
        return val != 0
    if kind == 'bit':
        return (val & target) == target
    return False


class TestClient(Node):
    def __init__(self):
        super().__init__('test_closed_loop_client')
        self.cli = self.create_client(AutoMachine, 'auto_machine/command')
        timeout_sec = 40.0  # 必须大于节点闭环超时(默认30s)
        print('等待服务 auto_machine/command ...', flush=True)
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('服务不可用：请确认 auto_machine_node 已启动')
        self.timeout_sec = timeout_sec

    def call(self, dev_type, command, params):
        req = AutoMachine.Request()
        req.dev_type = dev_type
        req.command = command
        req.param_data = list(params)
        future = self.cli.call_async(req)
        t0 = time.time()
        while time.time() - t0 < self.timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                break
        elapsed = time.time() - t0
        if not future.done():
            return None, elapsed
        return future.result(), elapsed


def print_result(name, dev, cmd, resp, elapsed, expect=None):
    print('------------------------------------------------')
    print('用例：%s' % name)
    print('请求：dev_type=0x%X command=0x%08X' % (dev, cmd))
    print('耗时：%.2f 秒' % elapsed)
    if resp is None:
        print('结果：✗ 客户端等待超时（>%.0fs），节点可能仍在闭环等待' % 40.0)
        return False
    val = ret_value(resp.ret_data)
    print('响应：success=%s err_code=%d（%s）'
          % (resp.success, resp.err_code, ERR_TEXT.get(resp.err_code, '未知')))
    print('ret_data：%s  => 数值 %s'
          % (' '.join('%02X' % b for b in resp.ret_data) or '(空)',
             '无' if val is None else str(val)))
    if expect is not None:
        kind, target = expect
        ok = resp.success and resp.err_code == 0 and judge(kind, target, val)
        print('判定：%s' % ('✓ PASS 到位' if ok else '✗ FAIL 未达到目标值'))
        return ok
    print('（单条模式，不做自动判定）')
    return resp.success


def main():
    rclpy.init()
    node = TestClient()

    # 单条模式
    if len(sys.argv) >= 3:
        dev = int(sys.argv[1], 0)
        cmd = int(sys.argv[2], 0)
        params = [int(x, 0) & 0xFF for x in sys.argv[3:]]
        resp, elapsed = node.call(dev, cmd, params)
        print_result('单条命令', dev, cmd, resp, elapsed, expect=None)
        node.destroy_node()
        rclpy.shutdown()
        return

    # 批量模式
    print('\n========== 闭环批量自测，共 %d 条 ==========\n' % len(CASES))
    passed = 0
    failed = 0
    for i, (name, dev, cmd, params, kind, target) in enumerate(CASES, 1):
        print('\n[%d/%d]' % (i, len(CASES)))
        resp, elapsed = node.call(dev, cmd, params)
        ok = print_result(name, dev, cmd, resp, elapsed, expect=(kind, target))
        if ok:
            passed += 1
        else:
            failed += 1
        time.sleep(0.3)

    print('\n==================================================')
    print('汇总：PASS %d / %d，FAIL %d' % (passed, len(CASES), failed))
    if failed == 0:
        print('全部闭环命令均在超时前查到“动作到位”，闭环功能正常。')
    else:
        print('存在未到位用例：')
        print('  - 若为 10008：确认终端1启动的是 fake_modbus_slave_closedloop.py')
        print('  - 若为 10006：检查从站是否在运行、serial_port 是否为其 NODE_PORT')
    print('==================================================')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
