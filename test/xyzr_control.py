# -*- coding: utf-8 -*-
"""
XYZR 机构 MODBUS RTU 机器控制器
从站ID=02，功能码 06写/04读。
485配置：115200 8数据位 奇校验 1停止位
写控制寄存器(0x06)：
  0x0001 大身布料已放好     1-大身布已放到位，主控可以触发视觉进行检测
  0x0002 口袋布已放好       1-口袋布已放到位，主控可以触发视觉进行检测（其他值无效）
读输入寄存器(0x04)：
  0x0000 预留暂未用
  0x0001 主控状态查询      0-待机（主控空闲，等待指令）
                          1-准备就绪，可接收命令
                          10-执行中，正在移料
                          11-执行完成，等待确认
                          20-暂停
                          100-发生故障
  0x0003 大身移料状态      0-移料失败
        （移料后检测结果）  1-移料成功，移到位，可以进行下一步缝纫动作
  0x0004 口袋移料状态      0-移料失败
        （移料后检测结果）  1-移料成功，移到位，可以进行下一步缝纫动作

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

# 0x06 写控制寄存器地址
REG_BODY_CLOTH_RESET   = 0x0000
REG_BODY_CLOTH_READY   = 0x0001    # 大身布料已放好
REG_POCKET_CLOTH_READY = 0x0002    # 口袋布已放好

# 0x04 读输入寄存器地址
REG_MASTER_STATUS      = 0x0001    # 主控状态查询
REG_BODY_MOVE_STATUS   = 0x0003    # 大身移料状态
REG_POCKET_MOVE_STATUS = 0x0004    # 口袋移料状态


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
# 主控状态：地址1
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

# 移料状态：地址3/地址4，两者取值相同
# 0-移料失败 1-移料成功，移到位，可以进行下一步缝纫动作
def translate_move_status(code):
    if code is None:
        return "读取失败"
    if code == 0:
        return "移料失败"
    elif code == 1:
        return "移料成功，移到位，可以进行下一步缝纫动作"
    return "未知({})".format(code)


# ====================== 主界面 ======================
class XYZRControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("XYZR Modbus RTU 机器控制器")
        self.root.geometry("860x620")

        self.ser = None

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
        self.close_btn = ttk.Button(port_frame, text="关闭串口", command=self.close_serial, state=tk.DISABLED)
        self.close_btn.grid(row=0, column=6, padx=8, pady=5)

        # ---------- 控制按钮区域 ----------
        btn_frame = ttk.LabelFrame(root, text="控制按钮")
        btn_frame.pack(padx=10, pady=8, fill=tk.X)

        ttk.Button(btn_frame, text="大身布料已放好", command=self.send_body_cloth_ready, width=16).grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(btn_frame, text="口袋布已放好", command=self.send_pocket_cloth_ready, width=16).grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(btn_frame, text="RESET", command=self.send_body_cloth_reset, width=16).grid(row=0, column=2, padx=6, pady=6)

        # ---------- 状态查询区域 ----------
        query_frame = ttk.LabelFrame(root, text="状态查询")
        query_frame.pack(padx=10, pady=5, fill=tk.X)

        ttk.Button(query_frame, text="查询主控状态", command=self.query_master_status, width=16).grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(query_frame, text="查询大身移料状态", command=self.query_body_move_status, width=16).grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(query_frame, text="查询口袋移料状态", command=self.query_pocket_move_status, width=16).grid(row=0, column=2, padx=6, pady=6)

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

    # 通用读寄存器(0x04)，返回单寄存器值或 None
    def read_reg(self, reg):
        data = bytes([SLAVE_ADDR, 0x04,
                      (reg >> 8) & 0xFF, reg & 0xFF,
                      0x00, 0x01])
        resp = self.send_modbus_with_reply(data)
        if resp and len(resp) >= 7 and resp[1] == 0x04:
            return (resp[3] << 8) | resp[4]
        return None

    # ===== 06写指令：动作 =====
    # 大身布料已放好  寄存器0x0001=1（主控可触发视觉检测）
    def send_body_cloth_ready(self):
        self.write_reg(REG_BODY_CLOTH_READY, 1)
        
    def send_body_cloth_reset(self):
        self.write_reg(REG_BODY_CLOTH_RESET, 1)

    # 口袋布已放好  寄存器0x0002=1（主控可触发视觉检测）
    def send_pocket_cloth_ready(self):
        self.write_reg(REG_POCKET_CLOTH_READY, 1)

    # ===== 04读指令：状态查询 =====
    # 主控状态  寄存器0x0001
    def query_master_status(self):
        v = self.read_reg(REG_MASTER_STATUS)
        self.log("主控状态：{}".format(translate_master_status(v)))

    # 大身移料状态  寄存器0x0003
    def query_body_move_status(self):
        v = self.read_reg(REG_BODY_MOVE_STATUS)
        self.log("大身移料状态：{}".format(translate_move_status(v)))

    # 口袋移料状态  寄存器0x0004
    def query_pocket_move_status(self):
        v = self.read_reg(REG_POCKET_MOVE_STATUS)
        self.log("口袋移料状态：{}".format(translate_move_status(v)))


if __name__ == "__main__":
    window = tk.Tk()
    app = XYZRControlApp(window)
    window.mainloop()
