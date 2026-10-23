#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""被动监听节点发出的 Modbus RTU 指令帧（只读、不回复、不校验）。

用途：确认「调用 auto_machine/command 服务后，节点是否向串口发出正确的指令帧」。
本脚本不模拟任何设备，只是把节点写到串口上的字节原样打印成十六进制。

用法（Linux）：
    python3 capture_serial.py

会打印 NODE_PORT=/dev/pts/N，把该路径作为节点的 serial_port 参数即可。
之后节点每发一帧，这里就打印一帧十六进制（一帧 8 字节）。
"""

import os
import pty
import sys
import time


def main():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    print('===== 串口帧捕获（被动监听，不回复） =====')
    print('NODE_PORT=%s' % slave_name)
    print('把上面的路径作为节点的 serial_port 参数，然后调用服务。')
    print('节点每发一帧，这里就打印一帧十六进制。\n')
    sys.stdout.flush()

    while True:
        try:
            data = os.read(master_fd, 1024)
        except OSError:
            break
        if not data:
            break
        ts = time.strftime('%H:%M:%S')
        print('[%s] %s' % (ts, ' '.join('%02X' % b for b in data)))
        sys.stdout.flush()


if __name__ == '__main__':
    main()
