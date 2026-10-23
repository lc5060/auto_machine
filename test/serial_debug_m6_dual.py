#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
MAG06（M6 钉扣机）+ 钉扣对齐装置 双串口调试界面（tkinter GUI）
================================================================================
用途：
  在没有 ROS 节点参与的情况下，同时连接两台 MAG06 钉扣机（两条独立 RS485
  串口）。每台自动机的同一串口上还并接了对齐装置（485 总线共用、从站地址
  不同），因此每个面板内可同时调试 M6 钉扣机与钉扣对齐装置。

通讯参数（与 auto_machine 节点一致）：
  115200 8 数据位 奇校验(O) 1 停止位，Modbus RTU。
  · M6 钉扣机：从站 0x01，基址 0x1064
  · 对齐装置 ：从站 0x02，0x06 写控制 / 0x04 读输入寄存器

M6 寄存器（基址 0x1064，偏移与 MAG04 相同）：
  0x1064 主控制命令    0x1065 故障码      0x1066 准入/复位状态
  0x1067 吸风状态      0x1068 理布控制    0x1069 理布状态
  0x106A 吹气控制      0x106B 压板控制

对齐装置寄存器：
  0x0000 RESET（0x06 写 1）
  0x0001 大身布料已放好（0x06 写 1；界面为闭环按钮，自动轮询到移料成功）
  0x0001 移料状态（0x04 读：0 失败 / 1 成功）
  0x0002 主控状态（0x04 读：0/1/10/11/20/100）
  0x0003 故障代码（0x04 读）

多机说明：
  界面左右并排「串口 1（0 号机）」「串口 2（1 号机）」两个独立面板，各自
  独立打开串口、独立收发、独立日志，可同时调试两台设备；对应 ROS 服务请求
  里的 param_data[0] = 0 / 1。

运行（需 pyserial）：
  pip3 install pyserial    # 若未安装
  python3 serial_debug_m6_dual.py
================================================================================
"""

import struct
import time

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, scrolledtext

# ====================== 通讯参数 ======================
SLAVE_ADDR = 0x01
BAUD_RATE = 115200
PARITY = serial.PARITY_ODD
BYTESIZE = serial.EIGHTBITS
STOPBITS = serial.STOPBITS_ONE

# ====================== MAG06 寄存器基址 ======================
MAG06_BASE = 0x1064

# 相对基址的偏移
OFF_CTRL = 0x00       # 主控制命令寄存器
OFF_ERROR = 0x01      # 故障码
OFF_ADMISSION = 0x02  # 准入/复位状态
OFF_XIFENG = 0x03     # 吸风状态
OFF_CLOTH = 0x04      # 理布控制命令
OFF_FUPING = 0x05     # 理布状态
OFF_BLOW = 0x06       # 吹气控制命令
OFF_PRESS = 0x07      # 压板控制命令

# 主控制寄存器（0x1064）命令值
CTRL_MANUAL = 0x0000       # 人工控制
CTRL_XIFENG_ON = 0x0001    # 开始吸风
CTRL_TAILOR = 0x0002       # 开始缝制
CTRL_ESTOP = 0x0003        # 急停
CTRL_RESET = 0x0004        # 复位
CTRL_XIFENG_OFF = 0x0005   # 关闭吸风

# ====================== 钉扣对齐装置（从站 0x02） ======================
ALIGN_SLAVE_ADDR = 0x02

# 0x06 写控制寄存器
ALIGN_REG_RESET = 0x0000             # 复位（主控回零）
ALIGN_REG_BODY_CLOTH_READY = 0x0001  # 大身布料已放好

# 0x04 读输入寄存器
ALIGN_REG_MOVE_STATUS = 0x0001       # 移料状态（移料后检测结果）
ALIGN_REG_MASTER_STATUS = 0x0002     # 主控状态查询
ALIGN_REG_FAULT_CODE = 0x0003        # 故障代码

# 对齐装置闭环轮询参数（与 auto_machine 节点默认一致）
ALIGN_LOOP_DELAY_MS = 500
ALIGN_LOOP_POLL_INTERVAL_MS = 300
ALIGN_LOOP_TIMEOUT_MS = 30000


def calc_crc(data):
    """Modbus RTU CRC16（多项式 0xA001，初始 0xFFFF）"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


def translate_error(code):
    return "无故障" if code == 0 else "有故障(0x{:04X})".format(code)


def translate_admission(code):
    return {
        0: "未复位不可执行",
        1: "已复位可执行",
        2: "未完成当前件缝制",
        3: "已完成当前件缝制",
    }.get(code, "未知({})".format(code))


def translate_xifeng(code):
    return "关闭" if code == 0 else "开启({})".format(code)


def translate_fuping(code):
    return {
        0: "气缸未到位",
        1: "气缸到位",
        2: "气缸复位",
    }.get(code, "未知({})".format(code))


def translate_move_status(code):
    return {
        None: "读取失败",
        0: "移料失败",
        1: "移料成功，移到位，可以进行下一步缝纫动作",
    }.get(code, "未知({})".format(code))


def translate_master_status(code):
    return {
        None: "读取失败",
        0: "待机（主控空闲，等待指令）",
        1: "准备就绪，可接收命令",
        10: "执行中，正在移料",
        11: "执行完成，等待确认",
        20: "暂停",
        100: "发生故障",
    }.get(code, "未知({})".format(code))


class Mag6Channel(ttk.Frame):
    """单台 MAG06 的调试面板：独立串口 + 控制/查询 + 日志。"""

    def __init__(self, master, title, preferred_port=None):
        super().__init__(master, padding=6)
        self.title = title
        self.ser = None
        self._preferred_port = preferred_port
        # 对齐装置闭环轮询状态（每个通道独立）
        self._align_loop_running = False
        self._align_loop_start = 0.0
        self._align_loop_phase = 1
        self._build_ui()
        self.refresh_ports()

    # ---------- UI ----------
    def _build_ui(self):
        # 通道标题
        ttk.Label(self, text=self.title, font=("TkDefaultFont", 11, "bold")).pack(
            anchor=tk.W, padx=4, pady=(2, 2))

        # 串口配置
        port_frame = ttk.LabelFrame(self, text="串口配置（115200 8O1 RTU，从站 0x01）")
        port_frame.pack(padx=4, pady=4, fill=tk.X)

        ttk.Label(port_frame, text="串口：").grid(row=0, column=0, padx=5, pady=5)
        self.port_combo = ttk.Combobox(port_frame, width=20)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Button(port_frame, text="刷新", command=self.refresh_ports).grid(
            row=0, column=2, padx=4, pady=5)
        self.open_btn = ttk.Button(port_frame, text="打开串口", command=self.open_serial)
        self.open_btn.grid(row=0, column=3, padx=6, pady=5)
        self.close_btn = ttk.Button(port_frame, text="关闭串口", command=self.close_serial,
                                    state=tk.DISABLED)
        self.close_btn.grid(row=0, column=4, padx=6, pady=5)

        self.state_label = ttk.Label(port_frame, text="未打开", foreground="red")
        self.state_label.grid(row=1, column=0, columnspan=5, padx=5, pady=(0, 5),
                              sticky=tk.W)

        # 控制命令
        ctrl_frame = ttk.LabelFrame(self, text="控制命令（功能码 0x06 写寄存器）")
        ctrl_frame.pack(padx=4, pady=4, fill=tk.X)

        ttk.Button(ctrl_frame, text="开始吸风", command=self.cmd_xifeng_on, width=14).grid(
            row=0, column=0, padx=5, pady=5)
        ttk.Button(ctrl_frame, text="关闭吸风", command=self.cmd_xifeng_off, width=14).grid(
            row=0, column=1, padx=5, pady=5)
        ttk.Button(ctrl_frame, text="开始缝制", command=self.cmd_tailor, width=14).grid(
            row=0, column=2, padx=5, pady=5)
        ttk.Button(ctrl_frame, text="急停", command=self.cmd_estop, width=14).grid(
            row=0, column=3, padx=5, pady=5)

        ttk.Button(ctrl_frame, text="复位", command=self.cmd_reset, width=14).grid(
            row=1, column=0, padx=5, pady=5)
        ttk.Button(ctrl_frame, text="人工控制", command=self.cmd_manual, width=14).grid(
            row=1, column=1, padx=5, pady=5)
        ttk.Button(ctrl_frame, text="开始理布", command=self.cmd_fuping_on, width=14).grid(
            row=1, column=2, padx=5, pady=5)
        ttk.Button(ctrl_frame, text="关闭理布", command=self.cmd_fuping_off, width=14).grid(
            row=1, column=3, padx=5, pady=5)

        ttk.Button(ctrl_frame, text="关闭气缸", command=self.cmd_cylinder_off, width=14).grid(
            row=2, column=0, padx=5, pady=5)
        ttk.Button(ctrl_frame, text="开始吹气", command=self.cmd_blow_on, width=14).grid(
            row=2, column=1, padx=5, pady=5)
        ttk.Button(ctrl_frame, text="停止吹气", command=self.cmd_blow_off, width=14).grid(
            row=2, column=2, padx=5, pady=5)
        ttk.Button(ctrl_frame, text="压板下压", command=self.cmd_press_down, width=14).grid(
            row=2, column=3, padx=5, pady=5)

        ttk.Button(ctrl_frame, text="压板抬起", command=self.cmd_press_up, width=14).grid(
            row=3, column=0, padx=5, pady=5)

        # 状态查询
        query_frame = ttk.LabelFrame(self, text="状态查询（功能码 0x03 读保持寄存器）")
        query_frame.pack(padx=4, pady=4, fill=tk.X)

        ttk.Button(query_frame, text="查询故障码", command=self.query_error, width=16).grid(
            row=0, column=0, padx=6, pady=6)
        ttk.Button(query_frame, text="查询准入状态", command=self.query_admission, width=16).grid(
            row=0, column=1, padx=6, pady=6)
        ttk.Button(query_frame, text="查询吸风状态", command=self.query_xifeng, width=16).grid(
            row=0, column=2, padx=6, pady=6)
        ttk.Button(query_frame, text="查询理布状态", command=self.query_fuping, width=16).grid(
            row=0, column=3, padx=6, pady=6)

        # 对齐装置控制（与 M6 共用同一串口，从站 0x02）
        align_ctrl_frame = ttk.LabelFrame(
            self, text="对齐装置控制（从站 0x02，功能码 0x06 写寄存器）")
        align_ctrl_frame.pack(padx=4, pady=4, fill=tk.X)

        ttk.Button(align_ctrl_frame, text="大身布料已放好（闭环）",
                   command=self.send_align_body_cloth_loop, width=22).grid(
            row=0, column=0, padx=5, pady=5)
        ttk.Button(align_ctrl_frame, text="对齐 RESET",
                   command=self.send_align_reset, width=16).grid(
            row=0, column=1, padx=5, pady=5)

        # 对齐装置状态查询
        align_query_frame = ttk.LabelFrame(
            self, text="对齐装置状态（从站 0x02，功能码 0x04 读输入寄存器）")
        align_query_frame.pack(padx=4, pady=4, fill=tk.X)

        ttk.Button(align_query_frame, text="查询移料状态",
                   command=self.query_align_move_status, width=16).grid(
            row=0, column=0, padx=6, pady=6)
        ttk.Button(align_query_frame, text="查询主控状态",
                   command=self.query_align_master_status, width=16).grid(
            row=0, column=1, padx=6, pady=6)
        ttk.Button(align_query_frame, text="查询故障代码",
                   command=self.query_align_fault_code, width=16).grid(
            row=0, column=2, padx=6, pady=6)

        # 通信日志
        log_frame = ttk.LabelFrame(self, text="通信日志")
        log_frame.pack(padx=4, pady=4, fill=tk.BOTH, expand=True)
        self.log_text = scrolledtext.ScrolledText(log_frame, width=60, height=16)
        self.log_text.pack(padx=5, pady=5, fill=tk.BOTH, expand=True)

    # ---------- 工具 ----------
    def base(self):
        return MAG06_BASE

    def log(self, msg):
        self.log_text.insert(tk.END, "[{}] {}\n".format(time.strftime('%H:%M:%S'), msg))
        self.log_text.see(tk.END)

    @staticmethod
    def hex_str(data):
        return ' '.join('{:02X}'.format(b) for b in data)

    # ---------- 串口 ----------
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        current = self.port_combo.get()
        self.port_combo["values"] = ports
        if not ports:
            self.log("未发现可用串口")
            return
        if self._preferred_port and self._preferred_port in ports:
            self.port_combo.set(self._preferred_port)
        elif current in ports:
            self.port_combo.set(current)
        elif not current:
            self.port_combo.current(0)
        self.log("已发现串口：{}".format(', '.join(ports)))

    def open_serial(self):
        if self.ser and self.ser.is_open:
            self.log("串口已打开：{}".format(self.ser.port))
            return
        port = self.port_combo.get().strip()
        if not port:
            self.log("请先选择串口")
            return
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=BAUD_RATE,
                parity=PARITY,
                stopbits=STOPBITS,
                bytesize=BYTESIZE,
                timeout=0.5,
            )
            self.log("串口已打开：{} 115200 8数据位 奇校验 1停止位".format(port))
            self.open_btn.config(state=tk.DISABLED)
            self.close_btn.config(state=tk.NORMAL)
            self.state_label.config(text="已打开 {}".format(port), foreground="green")
        except Exception as e:
            self.ser = None
            self.log("打开串口失败：{}".format(str(e)))
            self.state_label.config(text="打开失败", foreground="red")

    def close_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.log("串口已关闭")
        self.ser = None
        # 关闭串口时中止对齐装置闭环轮询，避免继续空转
        self._align_loop_running = False
        self.open_btn.config(state=tk.NORMAL)
        self.close_btn.config(state=tk.DISABLED)
        self.state_label.config(text="未打开", foreground="red")

    def is_open(self):
        return bool(self.ser and self.ser.is_open)

    # ---------- Modbus 收发 ----------
    # 0x06 写指令：发送后丢弃从站回显（本工程控制命令不等待应答）
    def send_no_reply(self, data):
        if not self.is_open():
            self.log("请先打开串口")
            return
        crc = calc_crc(data)
        frame = data + struct.pack('<H', crc)
        self.log("发送：{}".format(self.hex_str(frame)))
        self.ser.write(frame)
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    # 0x03 读指令：发送并读取应答
    def send_with_reply(self, data):
        if not self.is_open():
            self.log("请先打开串口")
            return None
        crc = calc_crc(data)
        frame = data + struct.pack('<H', crc)
        self.log("发送：{}".format(self.hex_str(frame)))
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        time.sleep(0.05)
        resp = self.ser.read(100)
        if resp:
            self.log("接收：{}".format(self.hex_str(resp)))
            return resp
        self.log("未收到从站回复")
        return None

    # 通用单寄存器写（0x06）
    def write_register(self, slave_addr, reg_addr, value):
        data = bytes([slave_addr, 0x06,
                      (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
                      (value >> 8) & 0xFF, value & 0xFF])
        self.send_no_reply(data)

    # 通用单寄存器读（默认 0x03 保持寄存器；对齐装置用 0x04 输入寄存器）
    def read_register(self, slave_addr, reg_addr, func=0x03):
        data = bytes([slave_addr, func,
                      (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
                      0x00, 0x01])
        resp = self.send_with_reply(data)
        if resp and len(resp) >= 7 and resp[1] == func:
            return (resp[3] << 8) | resp[4]
        return None

    # ---- MAG06（从站 0x01，基址 0x1064，0x03 读） ----
    def write_reg(self, offset, value):
        self.write_register(SLAVE_ADDR, self.base() + offset, value)

    def read_reg(self, offset):
        return self.read_register(SLAVE_ADDR, self.base() + offset, 0x03)

    # ---------- 控制命令 ----------
    def cmd_xifeng_on(self):
        self.write_reg(OFF_CTRL, CTRL_XIFENG_ON)

    def cmd_xifeng_off(self):
        self.write_reg(OFF_CTRL, CTRL_XIFENG_OFF)

    def cmd_tailor(self):
        self.write_reg(OFF_CTRL, CTRL_TAILOR)

    def cmd_estop(self):
        self.write_reg(OFF_CTRL, CTRL_ESTOP)

    def cmd_reset(self):
        self.write_reg(OFF_CTRL, CTRL_RESET)

    def cmd_manual(self):
        self.write_reg(OFF_CTRL, CTRL_MANUAL)

    def cmd_fuping_on(self):
        self.write_reg(OFF_CLOTH, 0x0001)

    def cmd_fuping_off(self):
        self.write_reg(OFF_CLOTH, 0x0002)

    def cmd_cylinder_off(self):
        self.write_reg(OFF_CLOTH, 0x0003)

    def cmd_blow_on(self):
        self.write_reg(OFF_BLOW, 0x0001)

    def cmd_blow_off(self):
        self.write_reg(OFF_BLOW, 0x0000)

    def cmd_press_down(self):
        self.write_reg(OFF_PRESS, 0x0001)

    def cmd_press_up(self):
        self.write_reg(OFF_PRESS, 0x0000)

    # ---------- 状态查询 ----------
    def query_error(self):
        v = self.read_reg(OFF_ERROR)
        self.log("故障码：{}".format(translate_error(v) if v is not None else "读取失败"))

    def query_admission(self):
        v = self.read_reg(OFF_ADMISSION)
        self.log("准入状态：{}".format(
            translate_admission(v) if v is not None else "读取失败"))

    def query_xifeng(self):
        v = self.read_reg(OFF_XIFENG)
        self.log("吸风状态：{}".format(
            translate_xifeng(v) if v is not None else "读取失败"))

    def query_fuping(self):
        v = self.read_reg(OFF_FUPING)
        self.log("理布状态：{}".format(
            translate_fuping(v) if v is not None else "读取失败"))

    # ---------- 对齐装置（从站 0x02，与 M6 共用同一串口） ----------
    def send_align_reset(self):
        self.write_register(ALIGN_SLAVE_ADDR, ALIGN_REG_RESET, 1)

    def query_align_move_status(self):
        v = self.read_register(ALIGN_SLAVE_ADDR, ALIGN_REG_MOVE_STATUS, 0x04)
        self.log("对齐-移料状态：{}".format(translate_move_status(v)))

    def query_align_master_status(self):
        v = self.read_register(ALIGN_SLAVE_ADDR, ALIGN_REG_MASTER_STATUS, 0x04)
        self.log("对齐-主控状态：{}".format(translate_master_status(v)))

    def query_align_fault_code(self):
        v = self.read_register(ALIGN_SLAVE_ADDR, ALIGN_REG_FAULT_CODE, 0x04)
        if v is None:
            self.log("对齐-故障代码：读取失败")
        else:
            self.log("对齐-故障代码：{} (0x{:04X})".format(v, v))

    # 大身布料已放好（闭环）：
    #   1) 写 0x0001=1 触发视觉检测/移料
    #   2) 轮询主控状态(0x0002) 到 11（执行完成，等待确认）
    #   3) 再确认移料状态(0x0001) 到 1（移料成功，移到位）
    def send_align_body_cloth_loop(self):
        if not self.is_open():
            self.log("请先打开串口")
            return
        if self._align_loop_running:
            self.log("对齐装置闭环查询进行中，请等待本次完成")
            return

        self.write_register(ALIGN_SLAVE_ADDR, ALIGN_REG_BODY_CLOTH_READY, 1)
        self.log("对齐-大身布料已放好：已下发，{} ms 后开始轮询（间隔 {} ms，超时 {} s）".format(
            ALIGN_LOOP_DELAY_MS, ALIGN_LOOP_POLL_INTERVAL_MS,
            ALIGN_LOOP_TIMEOUT_MS // 1000))

        self._align_loop_running = True
        self._align_loop_start = time.time()
        self._align_loop_phase = 1
        self.after(ALIGN_LOOP_DELAY_MS, self._align_loop_poll)

    def _align_loop_poll(self):
        if not self._align_loop_running:
            return

        elapsed_ms = (time.time() - self._align_loop_start) * 1000
        if elapsed_ms >= ALIGN_LOOP_TIMEOUT_MS:
            self._align_loop_running = False
            self.log("对齐-大身布料已放好：闭环超时（{} s 内未得到布料完成反馈）".format(
                ALIGN_LOOP_TIMEOUT_MS // 1000))
            return

        if self._align_loop_phase == 1:
            # 阶段1：主控状态 -> 11（执行完成，等待确认）
            v = self.read_register(ALIGN_SLAVE_ADDR, ALIGN_REG_MASTER_STATUS, 0x04)
            if v == 11:
                self.log("对齐-主控状态：执行完成，等待确认，进入阶段2确认移料结果")
                self._align_loop_phase = 2
            elif v is None:
                self.log("对齐-主控状态读取失败，{} ms 后重试".format(
                    ALIGN_LOOP_POLL_INTERVAL_MS))
            else:
                self.log("对齐-主控状态：{}，继续轮询...".format(translate_master_status(v)))
        else:
            # 阶段2：移料状态 -> 1（移料成功，移到位）
            v = self.read_register(ALIGN_SLAVE_ADDR, ALIGN_REG_MOVE_STATUS, 0x04)
            if v == 1:
                self._align_loop_running = False
                self.log("对齐-大身布料已放好：完成（移料成功，移到位，可以进行下一步缝纫动作）")
                return
            elif v is None:
                self.log("对齐-移料状态读取失败，{} ms 后重试".format(
                    ALIGN_LOOP_POLL_INTERVAL_MS))
            else:
                self.log("对齐-移料状态：{}，继续轮询...".format(translate_move_status(v)))

        self.after(ALIGN_LOOP_POLL_INTERVAL_MS, self._align_loop_poll)


class Mag6DualDebugApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MAG06（M6 钉扣机）+ 钉扣对齐装置 双串口调试")
        self.root.geometry("1500x980")
        self.root.minsize(1200, 800)

        # 顶部批量操作
        toolbar = ttk.Frame(root)
        toolbar.pack(fill=tk.X, padx=10, pady=(10, 0))
        ttk.Button(toolbar, text="同时打开两个串口", command=self.open_both).pack(
            side=tk.LEFT, padx=4)
        ttk.Button(toolbar, text="同时关闭两个串口", command=self.close_both).pack(
            side=tk.LEFT, padx=4)
        ttk.Label(toolbar,
                  text="提示：左右面板各选一个串口；M6 与对齐装置共用该串口（从站 0x01/0x02）",
                  foreground="gray").pack(side=tk.LEFT, padx=12)

        # 两个独立通道左右并排，中间分隔条可拖动
        paned = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.ch1 = Mag6Channel(paned, "串口 1（0 号机）",
                              preferred_port="/dev/ttyRS485_0")
        self.ch2 = Mag6Channel(paned, "串口 2（1 号机）",
                              preferred_port="/dev/ttyRS485_1")
        paned.add(self.ch1, weight=1)
        paned.add(self.ch2, weight=1)

    def open_both(self):
        self.ch1.open_serial()
        self.ch2.open_serial()

    def close_both(self):
        self.ch1.close_serial()
        self.ch2.close_serial()


if __name__ == "__main__":
    window = tk.Tk()
    app = Mag6DualDebugApp(window)
    window.mainloop()
