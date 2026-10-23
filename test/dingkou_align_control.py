# -*- coding: utf-8 -*-
"""
================================================================================
钉扣对齐装置 MODBUS RTU 串口调试界面（tkinter GUI）
================================================================================
用途：
  在没有 ROS 节点参与的情况下，直接用串口调试钉扣对齐装置，
  手动下发控制命令、查询状态、核对 Modbus RTU 原始帧。

通讯参数：
  115200 8 数据位 奇校验(O) 1 停止位，Modbus RTU，从站地址 0x02。

写控制寄存器（功能码 0x06，写后丢弃从站回显）：
  0x0000 RESET            1-复位（主控回零）
  0x0001 大身布料已放好    1-大身布已放到位，主控可触发视觉检测
                          （界面已合成闭环按钮：下发后自动轮询到移料成功）

闭环判定（与 auto_machine 节点的 XYZR 闭环绑定一致）：
  阶段1 主控状态(0x0002) == 11（执行完成，等待确认）
  阶段2 移料状态(0x0001) == 1（移料成功，移到位）

读输入寄存器（功能码 0x04，只读）：
  0x0000 预留暂未用
  0x0001 移料状态（移料后检测结果）
         0-移料失败
         1-移料成功，移到位，可以进行下一步缝纫动作
  0x0002 主控状态查询
         0-待机（主控空闲，等待指令）
         1-准备就绪，可接收命令
         10-执行中，正在移料
         11-执行完成，等待确认
         20-暂停
         100-发生故障
  0x0003 故障代码

运行（需 pyserial）：
  pip3 install pyserial    # 若未安装
  python3 dingkou_align_control.py
================================================================================
"""
import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import struct
import time

# ====================== 通讯参数 ======================
SLAVE_ADDR = 0x02          # 本机从站站号为 2
BAUD_RATE = 115200
PARITY = serial.PARITY_ODD
BYTESIZE = serial.EIGHTBITS
STOPBITS = serial.STOPBITS_ONE

# 0x06 写控制寄存器地址（仅保留大身布料与 RESET）
REG_RESET               = 0x0000    # 复位（主控回零）
REG_BODY_CLOTH_READY    = 0x0001    # 大身布料已放好

# 0x04 读输入寄存器地址
REG_MOVE_STATUS         = 0x0001    # 移料状态（移料后检测结果）
REG_MASTER_STATUS       = 0x0002    # 主控状态查询
REG_FAULT_CODE          = 0x0003    # 故障代码

# 闭环轮询参数（与 auto_machine 节点默认一致）
CLOSED_LOOP_DELAY_MS = 500           # 下发后首次查询前的等待
CLOSED_LOOP_POLL_INTERVAL_MS = 300   # 轮询间隔
CLOSED_LOOP_TIMEOUT_MS = 30000       # 轮询总超时


# ====================== CRC16 ======================
def calc_crc(data):
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


# ====================== 状态翻译函数 ======================
# 移料状态：地址1
# 0-移料失败 1-移料成功，移到位，可以进行下一步缝纫动作
def translate_move_status(code):
    if code is None:
        return "读取失败"
    table = {
        0: "移料失败",
        1: "移料成功，移到位，可以进行下一步缝纫动作",
    }
    return table.get(code, "未知({})".format(code))


# 主控状态：地址2
def translate_master_status(code):
    if code is None:
        return "读取失败"
    table = {
        0: "待机（主控空闲，等待指令）",
        1: "准备就绪，可接收命令",
        10: "执行中，正在移料",
        11: "执行完成，等待确认",
        20: "暂停",
        100: "发生故障",
    }
    return table.get(code, "未知({})".format(code))


# ====================== 主界面 ======================
class DingkouAlignControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("钉扣对齐装置 Modbus RTU 串口调试")
        self.root.geometry("860x600")

        self.ser = None

        # 闭环轮询状态
        self._loop_running = False
        self._loop_start = 0.0
        self._loop_phase = 1

        # ---------- 串口配置区域 ----------
        port_frame = ttk.LabelFrame(root, text="串口配置")
        port_frame.pack(padx=10, pady=5, fill=tk.X)

        ttk.Label(port_frame, text="串口：").grid(row=0, column=0, padx=5, pady=5)
        self.port_combo = ttk.Combobox(port_frame, width=12)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)

        self.refresh_btn = ttk.Button(port_frame, text="刷新串口", command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=2, padx=5, pady=5)

        ttk.Label(port_frame, text="波特率：").grid(row=0, column=3, padx=5, pady=5)
        self.baud_label = ttk.Label(port_frame, text="115200 奇校验 从站号2", foreground="blue")
        self.baud_label.grid(row=0, column=4, padx=5, pady=5)

        self.open_btn = ttk.Button(port_frame, text="打开串口", command=self.open_serial)
        self.open_btn.grid(row=0, column=5, padx=8, pady=5)
        self.close_btn = ttk.Button(port_frame, text="关闭串口", command=self.close_serial,
                                    state=tk.DISABLED)
        self.close_btn.grid(row=0, column=6, padx=8, pady=5)

        # ---------- 控制按钮区域（0x06 写） ----------
        btn_frame = ttk.LabelFrame(root, text="控制按钮（功能码 0x06 写寄存器）")
        btn_frame.pack(padx=10, pady=8, fill=tk.X)

        ttk.Button(btn_frame, text="大身布料已放好（闭环）",
                   command=self.send_body_cloth_ready_loop,
                   width=20).grid(row=0, column=0, padx=8, pady=6)
        ttk.Button(btn_frame, text="RESET", command=self.send_reset,
                   width=18).grid(row=0, column=1, padx=8, pady=6)

        # ---------- 状态查询区域（0x04 读） ----------
        query_frame = ttk.LabelFrame(root, text="状态查询（功能码 0x04 读输入寄存器）")
        query_frame.pack(padx=10, pady=5, fill=tk.X)

        ttk.Button(query_frame, text="查询移料状态", command=self.query_move_status,
                   width=18).grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(query_frame, text="查询主控状态", command=self.query_master_status,
                   width=18).grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(query_frame, text="查询故障代码", command=self.query_fault_code,
                   width=18).grid(row=0, column=2, padx=6, pady=6)

        # ---------- 通信日志区域 ----------
        log_frame = ttk.LabelFrame(root, text="通信日志")
        log_frame.pack(padx=10, pady=5, fill=tk.BOTH, expand=True)
        self.log_text = scrolledtext.ScrolledText(log_frame, width=98, height=16)
        self.log_text.pack(padx=5, pady=5, fill=tk.BOTH, expand=True)

        self.refresh_ports()

    # ---------- 串口 ----------
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)
            self.log("已发现串口：{}".format(', '.join(ports)))

    def open_serial(self):
        port = self.port_combo.get()
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=BAUD_RATE,
                parity=PARITY,
                stopbits=STOPBITS,
                bytesize=BYTESIZE,
                timeout=0.5
            )
            self.log("串口已打开：{} 115200 8数据位 奇校验 1停止位 从站号2".format(port))
            self.open_btn.config(state=tk.DISABLED)
            self.close_btn.config(state=tk.NORMAL)
        except Exception as e:
            self.log("打开串口失败：{}".format(str(e)))

    def close_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.log("串口已关闭")
        self.open_btn.config(state=tk.NORMAL)
        self.close_btn.config(state=tk.DISABLED)

    def log(self, msg):
        self.log_text.insert(tk.END, "[{}] {}\n".format(time.strftime('%H:%M:%S'), msg))
        self.log_text.see(tk.END)

    # ---------- Modbus 收发 ----------
    # 06写指令：发送后丢弃从站应答，避免应答堆积污染后续读指令
    def send_modbus_no_reply(self, data):
        if not self.ser or not self.ser.is_open:
            self.log("请先打开串口")
            return
        crc = calc_crc(data)
        data_crc = data + struct.pack('<H', crc)
        hex_str = ' '.join('{:02X}'.format(b) for b in data_crc)
        self.log("发送：{}".format(hex_str))
        self.ser.write(data_crc)
        # 06功能码从站会原样回显作为应答，等待并丢弃，防止缓冲区堆积
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    # 04读指令：发送并读取应答
    def send_modbus_with_reply(self, data):
        if not self.ser or not self.ser.is_open:
            self.log("请先打开串口")
            return None
        crc = calc_crc(data)
        data_crc = data + struct.pack('<H', crc)
        hex_str = ' '.join('{:02X}'.format(b) for b in data_crc)
        self.log("发送：{}".format(hex_str))
        self.ser.reset_input_buffer()
        self.ser.write(data_crc)
        time.sleep(0.05)
        resp = self.ser.read(100)
        if resp:
            resp_hex = ' '.join('{:02X}'.format(b) for b in resp)
            self.log("接收：{}".format(resp_hex))
            return resp
        else:
            self.log("未收到从站回复")
            return None

    # 通用写寄存器(0x06)
    def write_reg(self, reg, value):
        data = bytes([SLAVE_ADDR, 0x06,
                      (reg >> 8) & 0xFF, reg & 0xFF,
                      (value >> 8) & 0xFF, value & 0xFF])
        self.send_modbus_no_reply(data)

    # 通用读输入寄存器(0x04)，返回单寄存器值或 None
    def read_reg(self, reg):
        data = bytes([SLAVE_ADDR, 0x04,
                      (reg >> 8) & 0xFF, reg & 0xFF,
                      0x00, 0x01])
        resp = self.send_modbus_with_reply(data)
        if resp and len(resp) >= 7 and resp[1] == 0x04:
            return (resp[3] << 8) | resp[4]
        return None

    # ===== 06写指令：动作 =====
    # 大身布料已放好（闭环）：
    #   1) 写 0x0001=1 触发视觉检测/移料
    #   2) 轮询主控状态(0x0002) 到 11（执行完成，等待确认）
    #   3) 再确认移料状态(0x0001) 到 1（移料成功，移到位）
    def send_body_cloth_ready_loop(self):
        if not self.ser or not self.ser.is_open:
            self.log("请先打开串口")
            return
        if self._loop_running:
            self.log("闭环查询进行中，请等待本次完成")
            return

        # 1) 下发控制命令
        self.write_reg(REG_BODY_CLOTH_READY, 1)
        self.log("大身布料已放好：已下发，{} ms 后开始轮询（间隔 {} ms，超时 {} s）".format(
            CLOSED_LOOP_DELAY_MS, CLOSED_LOOP_POLL_INTERVAL_MS,
            CLOSED_LOOP_TIMEOUT_MS // 1000))

        # 2) 启动闭环（用 after 定时轮询，避免阻塞界面）
        self._loop_running = True
        self._loop_start = time.time()
        self._loop_phase = 1
        self.root.after(CLOSED_LOOP_DELAY_MS, self._loop_poll)

    def _loop_poll(self):
        if not self._loop_running:
            return

        elapsed_ms = (time.time() - self._loop_start) * 1000
        if elapsed_ms >= CLOSED_LOOP_TIMEOUT_MS:
            self._loop_running = False
            self.log("大身布料已放好：闭环超时（{} s 内未得到布料完成反馈）".format(
                CLOSED_LOOP_TIMEOUT_MS // 1000))
            return

        if self._loop_phase == 1:
            # 阶段1：主控状态 -> 11（执行完成，等待确认）
            v = self.read_reg(REG_MASTER_STATUS)
            if v == 11:
                self.log("主控状态：执行完成，等待确认，进入阶段2确认移料结果")
                self._loop_phase = 2
            elif v is None:
                self.log("主控状态读取失败，{} ms 后重试".format(CLOSED_LOOP_POLL_INTERVAL_MS))
            else:
                self.log("主控状态：{}，继续轮询...".format(translate_master_status(v)))
        else:
            # 阶段2：移料状态 -> 1（移料成功，移到位）
            v = self.read_reg(REG_MOVE_STATUS)
            if v == 1:
                self._loop_running = False
                self.log("大身布料已放好：完成（移料成功，移到位，可以进行下一步缝纫动作）")
                return
            elif v is None:
                self.log("移料状态读取失败，{} ms 后重试".format(CLOSED_LOOP_POLL_INTERVAL_MS))
            else:
                self.log("移料状态：{}，继续轮询...".format(translate_move_status(v)))

        self.root.after(CLOSED_LOOP_POLL_INTERVAL_MS, self._loop_poll)

    # 复位  寄存器0x0000=1（主控回零）
    def send_reset(self):
        self.write_reg(REG_RESET, 1)

    # ===== 04读指令：状态查询 =====
    # 移料状态  寄存器0x0001
    def query_move_status(self):
        v = self.read_reg(REG_MOVE_STATUS)
        self.log("移料状态：{}".format(translate_move_status(v)))

    # 主控状态  寄存器0x0002
    def query_master_status(self):
        v = self.read_reg(REG_MASTER_STATUS)
        self.log("主控状态：{}".format(translate_master_status(v)))

    # 故障代码  寄存器0x0003
    def query_fault_code(self):
        v = self.read_reg(REG_FAULT_CODE)
        if v is None:
            self.log("故障代码：读取失败")
        else:
            self.log("故障代码：{} (0x{:04X})".format(v, v))


if __name__ == "__main__":
    window = tk.Tk()
    app = DingkouAlignControlApp(window)
    window.mainloop()
