#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
auto_machine 多串口并发验证客户端  test_parallel.py
================================================================================
验证节点改造后的三条并发行为：

  [A] 跨串口并行   ：串口0 跑闭环（等缝制完成）期间，串口1 的查询应在毫秒级返回，
                     而不是等串口0 的闭环结束。
  [B] 同串口忙则拒 ：串口0 闭环未结束时，再向串口0 下发闭环命令，应立即返回
                     err_code=10009（ERR_BUSY），不排队、不阻塞。
  [C] 单帧可插入   ：串口0 闭环期间，向串口0 发状态查询，应正常返回（不是 10009），
                     保证 UI 周期查询和急停不被闭环挡住。

前置条件（在本机没有真机时用两个假从站）：
  1) 起两个假从站，各占一个虚拟串口，记下各自打印的 NODE_PORT：
       python3 fake_modbus_slave_closedloop.py
       python3 fake_modbus_slave_closedloop.py
  2) 起节点，把两个 pty 通过 mag_serial_ports 传进去：
       ros2 run auto_machine auto_machine_node --ros-args \\
         -p mag_serial_ports:="['/dev/pts/3','/dev/pts/4']" \\
         -p auto_query_delay_ms:=200 \\
         -p auto_query_poll_interval_ms:=100
  3) 运行本脚本：
       python3 test_parallel.py              # 三个场景全跑
       python3 test_parallel.py --scenario a # 只跑某一个

说明：
  - 假从站收到「开始缝制」后 0.8~1.8 秒才把准入状态翻到 3，所以闭环有一定时长，
    足够观察串口1 是否被阻塞。
  - 从站地址：MAG06(M6) = 0x01，XYZR = 0x02，由节点按设备类型自动选择。
  - param_data[0] = 串口索引（0/1），与 mag_serial_ports 顺序一致。
================================================================================
"""

import argparse
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from auto_machine.srv import AutoMachine

# ---------------- 设备类型 / 命令号（见 msg/AutoMachineDef.msg） ----------------
DEV_MAG04 = 0x00000003
DEV_MAG06 = 0x00000004

CMD_MAG04_XIFENG_STATUS_QUERY = 0x0003000B   # MAG04 查询吸风状态（单帧，不开闭环）
CMD_MAG06_START_TAILOR = 0x00040002          # MAG06 开始缝制（闭环，等准入=3）
CMD_MAG06_RESET = 0x00040005                 # 复位（闭环，等准入=1）
CMD_MAG06_XIFENG_STATUS_QUERY = 0x0004000B   # MAG06 查询吸风状态（单帧）

ERR_OK = 0
ERR_BUSY = 10009

CALL_TIMEOUT_SEC = 40.0
LOOP_TRIGGER_DELAY_SEC = 0.3   # 发完闭环命令后等多久再发第二个请求


class ParallelClient(Node):
    """一个节点 + 后台多线程执行器，允许同时挂多个未完成的请求。"""

    def __init__(self):
        super().__init__('auto_machine_parallel_test')
        self.cli = self.create_client(AutoMachine, 'auto_machine/command')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('等待服务 auto_machine/command ...')
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()

    def shutdown(self):
        self._executor.shutdown()
        self.destroy_node()

    def call(self, tag, dev_type, command, port):
        """异步发一个请求并等待结果，返回 (response, 耗时毫秒)。"""
        req = AutoMachine.Request()
        req.dev_type = dev_type
        req.command = command
        req.param_data = [port]

        t0 = time.monotonic()
        future = self.cli.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())

        if not done.wait(timeout=CALL_TIMEOUT_SEC):
            print('  [%-16s] port=%d 超时无响应（>%.0fs）'
                  % (tag, port, CALL_TIMEOUT_SEC))
            return None, (time.monotonic() - t0) * 1000.0

        elapsed_ms = (time.monotonic() - t0) * 1000.0
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001 - 测试脚本，异常只打印
            print('  [%-16s] port=%d 调用异常: %s' % (tag, port, exc))
            return None, elapsed_ms

        ret = ' '.join('%02X' % b for b in resp.ret_data) or '-'
        print('  [%-16s] port=%d cmd=0x%08X -> success=%-5s err=%-6d '
              'ret=%-8s 用时=%8.1f ms'
              % (tag, port, command, resp.success, resp.err_code, ret,
                 elapsed_ms))
        return resp, elapsed_ms


def _prepare_port0(client):
    """把 M6 拉回「已复位(1)」，并等假从站残留的定时器全部触发完。

    假从站收到控制命令后用定时器延迟翻状态，多个定时器会互相覆盖；
    每个场景开始前先等 2 秒再复位一次，才能保证后面的闭环时长可预期。
    """
    time.sleep(2.0)
    resp, _ = client.call('预置 复位', DEV_MAG06, CMD_MAG06_RESET, 0)
    return resp is not None and resp.err_code == ERR_OK


def _start_closed_loop(client, holder):
    """后台线程里发一条闭环控制命令（会阻塞到动作到位）。"""
    holder['result'] = client.call(
        '串口0 开始缝制', DEV_MAG06, CMD_MAG06_START_TAILOR, 0)


def scenario_a(client):
    print('\n[A] 跨串口并行：串口0 等缝制完成期间，串口1 查询')
    if not _prepare_port0(client):
        print('  => FAIL（前置复位失败，无法保证闭环时长）')
        return False
    holder = {}
    worker = threading.Thread(target=_start_closed_loop, args=(client, holder))
    worker.start()
    time.sleep(LOOP_TRIGGER_DELAY_SEC)

    resp_b, ms_b = client.call(
        '串口1 查吸风', DEV_MAG04, CMD_MAG04_XIFENG_STATUS_QUERY, 1)

    worker.join()
    resp_a, ms_a = holder.get('result', (None, 0.0))

    print('  串口0 闭环耗时 %.1f ms，串口1 查询耗时 %.1f ms'
          % (ms_a, ms_b))
    ok = (resp_a is not None and resp_a.err_code == ERR_OK
          and resp_b is not None and ms_b < 500.0 and ms_a > 500.0)
    print('  => ' + ('PASS' if ok else
                     'FAIL（串口1 被串口0 的闭环阻塞了）'))
    return ok


def scenario_b(client):
    print('\n[B] 同串口忙则拒：串口0 闭环未结束时，再发串口0 闭环命令')
    if not _prepare_port0(client):
        print('  => FAIL（前置复位失败，无法保证闭环时长）')
        return False
    holder = {}
    worker = threading.Thread(target=_start_closed_loop, args=(client, holder))
    worker.start()
    time.sleep(LOOP_TRIGGER_DELAY_SEC)

    resp, ms = client.call('串口0 复位(忙)', DEV_MAG06, CMD_MAG06_RESET, 0)

    worker.join()
    ok = (resp is not None and resp.err_code == ERR_BUSY and ms < 500.0)
    print('  => ' + ('PASS' if ok else
                     'FAIL（期望 err_code=%d 且 <500ms，实际 err_code=%s, %.1fms）'
                     % (ERR_BUSY,
                        resp.err_code if resp is not None else 'None', ms)))
    return ok


def scenario_c(client):
    print('\n[C] 单帧可插入：串口0 闭环期间，向串口0 发状态查询')
    if not _prepare_port0(client):
        print('  => FAIL（前置复位失败，无法保证闭环时长）')
        return False
    holder = {}
    worker = threading.Thread(target=_start_closed_loop, args=(client, holder))
    worker.start()
    time.sleep(LOOP_TRIGGER_DELAY_SEC)

    resp, ms = client.call(
        '串口0 查吸风', DEV_MAG06, CMD_MAG06_XIFENG_STATUS_QUERY, 0)

    worker.join()
    resp_a, ms_a = holder.get('result', (None, 0.0))
    print('  串口0 闭环耗时 %.1f ms，查询耗时 %.1f ms' % (ms_a, ms))
    ok = (resp is not None and resp.err_code != ERR_BUSY and ms < 500.0
          and ms_a > 500.0)
    print('  => ' + ('PASS' if ok else
                     'FAIL（单帧查询被闭环占位拒绝，或闭环并未真正进行中）'))
    return ok


SCENARIOS = {'a': scenario_a, 'b': scenario_b, 'c': scenario_c}


def main():
    parser = argparse.ArgumentParser(description='auto_machine 多串口并发验证')
    parser.add_argument('--scenario', choices=['a', 'b', 'c', 'all'],
                        default='all', help='要跑的场景，默认全部')
    args = parser.parse_args()

    rclpy.init()
    client = ParallelClient()
    results = {}
    try:
        names = list(SCENARIOS) if args.scenario == 'all' else [args.scenario]
        for name in names:
            results[name] = SCENARIOS[name](client)
            time.sleep(0.5)   # 给上一条闭环命令收尾
    finally:
        client.shutdown()
        rclpy.shutdown()

    print('\n================ 结果汇总 ================')
    for name in sorted(results):
        print('  场景 [%s]: %s' % (name.upper(),
                                  'PASS' if results[name] else 'FAIL'))
    all_ok = all(results.values())
    print('  总体: %s' % ('全部通过' if all_ok else '存在失败项'))
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
