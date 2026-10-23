# -*- coding: utf-8 -*-
"""
V6 MODBUS RTU 机器控制器
从站ID=01，功能码 06写/04读。
485配置：115200 8数据位 奇校验 1停止位
写控制寄存器(0x06)：
  0x0000 舌板伸出   1-伸出
  0x0001 折叠开始   1-开始
  0x0002 取料开始   1-开始
  0x0003 光电控制寄存器
  0x0004~0x0007 光电开关1~4
  0x0008 折叠复位   1-复位
  0x0009 鼓风机开关 1-开启 2-关闭
  0x000a 吸布开关   1-开启 2-关闭
  0x000b 抚平控制   1-开始 2-返回 3-复位
读状态寄存器(0x04)：
  0x0000 故障码       0-无故障 1-有故障
  0x0001 机器状态     2-缝制界面
  0x0002 折叠状态     0-等待舌板伸出 10-等待折叠 26-等待取料
  0x0003 鼓风机状态   0-未开启 1-已开启
  0x0004 吸布状态     0-未开启 1-已开启
  0x0005 指令准入状态 bit0舌板 bit1折叠 bit2取料 bit3复位 bit4吸布
  0x0006 抚平状态     0-未到位 1-完成 2-复位完成
"""
import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import struct
import time

# ====================== 通讯参数 ======================
SLAVE_ADDR = 0x01
BAUD_RATE = 115200
PARITY = serial.PARITY_ODD
BYTESIZE = serial.EIGHTBITS
STOPBITS = serial.STOPBITS_ONE

# 0x06 写控制寄存器地址
REG_TONGUE   = 0x0000    # 舌板伸出
REG_FOLD     = 0x0001    # 折叠开始
REG_GET      = 0x0002    # 取料开始
REG_OPTICAL  = 0x0003    # 光电控制寄存器
REG_LASER1   = 0x0004    # 光电开关1
REG_LASER2   = 0x0005    # 光电开关2
REG_LASER3   = 0x0006    # 光电开关3
REG_LASER4   = 0x0007    # 光电开关4
REG_RESET    = 0x0008    # 折叠复位
REG_BREEZE   = 0x0009    # 鼓风机开关
REG_INHALE   = 0x000a    # 吸布开关
REG_FLATTEN  = 0x000b    # 抚平控制

# 0x04 读状态寄存器地址
REG_ERR           = 0x0000
REG_MACHINE       = 0x0001
REG_FOLD_STATUS    = 0x0002
REG_BREEZE_STATE  = 0x0003    # 鼓风机状态
REG_INHALE_STATE  = 0x0004    # 吸布状态
REG_ALLOW_STATUS  = 0x0005    # 指令准入状态
REG_FLATTEN_STATE = 0x0006    # 抚平状态

# 开关值
OPT_ON  = 1
OPT_OFF = 0
BREEZE_ON  = 1
BREEZE_OFF = 2
INHALE_ON  = 1
INHALE_OFF = 2
FLATTEN_START  = 1   # 开始抚平
FLATTEN_RETURN = 2   # 返回抚平起始位置
FLATTEN_RESET  = 3   # 抚平复位


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
def translate_error(code):
    if code == 0:
        return "无故障"
    elif code == 1:
        return "有故障"
    else:
        return "未知({})".format(code)

def translate_machine(code):
    if code == 2:
        return "缝制界面"
    else:
        return "未知({})".format(code)

def translate_fold(code):
    if code == 0:
        return "等待舌板伸出"
    elif code == 10:
        return "等待折叠"
    elif code == 26:
        return "等待取料"
    else:
        return "未知({})".format(code)

# 鼓风机状态：地址3 0-未开启 1-开启
def translate_breeze_state(code):
    if code == 0:
        return "未开启"
    elif code == 1:
        return "已开启"
    return "未知({})".format(code)

# 吸布状态：地址4 0-未开启 1-开启
def translate_inhale_state(code):
    if code == 0:
        return "未开启"
    elif code == 1:
        return "已开启"
    return "未知({})".format(code)

# 指令准入状态解析
def translate_allow_status(code):
    if code is None:
        return "读取失败"
    res = []
    res.append("舌板:" + ("可执行" if (code >> 0) & 1 else "禁止"))
    res.append("折叠:" + ("可执行" if (code >> 1) & 1 else "禁止"))
    res.append("取料:" + ("可执行" if (code >> 2) & 1 else "禁止"))
    res.append("复位:" + ("可执行" if (code >> 3) & 1 else "禁止"))
    res.append("吸布:" + ("可执行" if (code >> 4) & 1 else "禁止"))
    return " ".join(res)

# 抚平状态：地址6 0-抚平进行中 1-抚平到位 2-抚平复位
def translate_flatten_state(code):
    if code is None:
        return "读取失败"
    if code == 0:
        return "抚平进行中"
    elif code == 1:
        return "抚平到位"
    elif code == 2:
        return "抚平复位"
    return "未知({})".format(code)


# ====================== 主界面 ======================
class V6ControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("V6 Modbus RTU 机器控制器")
        self.root.geometry("840x780")

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
        self.baud_label = ttk.Label(port_frame, text="115200 奇校验", foreground="blue")
        self.baud_label.grid(row=0, column=4, padx=5, pady=5)

        self.open_btn = ttk.Button(port_frame, text="打开串口", command=self.open_serial)
        self.open_btn.grid(row=0, column=5, padx=8, pady=5)
        self.close_btn = ttk.Button(port_frame, text="关闭串口", command=self.close_serial, state=tk.DISABLED)
        self.close_btn.grid(row=0, column=6, padx=8, pady=5)

        # ---------- 控制按钮区域 ----------
        btn_frame = ttk.LabelFrame(root, text="控制按钮")
        btn_frame.pack(padx=10, pady=8, fill=tk.X)

        # 第一行：舌板/折叠/取料/复位
        ttk.Button(btn_frame, text="舌板伸出", command=self.send_tongue, width=16).grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(btn_frame, text="折叠开始", command=self.send_fold, width=16).grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(btn_frame, text="取料开始", command=self.send_get, width=16).grid(row=0, column=2, padx=6, pady=6)
        ttk.Button(btn_frame, text="复位", command=self.send_reset, width=16).grid(row=0, column=3, padx=6, pady=6)

        # 第二行：鼓风机/吸布
        ttk.Button(btn_frame, text="鼓风机开启", command=self.send_breeze_on, width=16).grid(row=1, column=0, padx=6, pady=6)
        ttk.Button(btn_frame, text="鼓风机关闭", command=self.send_breeze_off, width=16).grid(row=1, column=1, padx=6, pady=6)
        ttk.Button(btn_frame, text="吸布开启", command=self.send_inhale_on, width=16).grid(row=1, column=2, padx=6, pady=6)
        ttk.Button(btn_frame, text="吸布关闭", command=self.send_inhale_off, width=16).grid(row=1, column=3, padx=6, pady=6)

        # 第三行：抚平
        ttk.Button(btn_frame, text="抚平启动", command=self.send_flatten_start, width=16).grid(row=2, column=0, padx=6, pady=6)
        ttk.Button(btn_frame, text="抚平返回", command=self.send_flatten_return, width=16).grid(row=2, column=1, padx=6, pady=6)
        ttk.Button(btn_frame, text="抚平关闭", command=self.send_flatten_reset, width=16).grid(row=2, column=2, padx=6, pady=6)

        # ---------- 状态查询区域 ----------
        query_frame = ttk.LabelFrame(root, text="状态查询")
        query_frame.pack(padx=10, pady=5, fill=tk.X)

        ttk.Button(query_frame, text="查询故障码", command=self.query_err, width=16).grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(query_frame, text="查询机器状态", command=self.query_machine, width=16).grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(query_frame, text="查询折叠状态", command=self.query_fold, width=16).grid(row=0, column=2, padx=6, pady=6)
        ttk.Button(query_frame, text="查询鼓风机状态", command=self.query_breeze_state, width=16).grid(row=0, column=3, padx=6, pady=6)

        ttk.Button(query_frame, text="查询吸布状态", command=self.query_inhale_state, width=16).grid(row=1, column=0, padx=6, pady=6)
        ttk.Button(query_frame, text="查询指令准入", command=self.query_allow_status, width=16).grid(row=1, column=1, padx=6, pady=6)
        ttk.Button(query_frame, text="查询抚平状态", command=self.query_flatten_state, width=16).grid(row=1, column=2, padx=6, pady=6)

        # ---------- 通信日志区域 ----------
        log_frame = ttk.LabelFrame(root, text="通信日志")
        log_frame.pack(padx=10, pady=5, fill=tk.BOTH, expand=True)
        self.log_text = scrolledtext.ScrolledText(log_frame, width=98, height=18)
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
            self.log("串口已打开：{} 115200 8数据位 奇校验 1停止位".format(port))
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
    # 舌板伸出  寄存器0x0000=1
    def send_tongue(self):
        self.write_reg(REG_TONGUE, 1)

    # 折叠开始  寄存器0x0001=1
    def send_fold(self):
        self.write_reg(REG_FOLD, 1)

    # 取料开始  寄存器0x0002=1
    def send_get(self):
        self.write_reg(REG_GET, 1)

    # 折叠复位  寄存器0x0008=1
    def send_reset(self):
        self.write_reg(REG_RESET, OPT_ON)

    # 鼓风机开启  寄存器0x0009=1
    def send_breeze_on(self):
        self.write_reg(REG_BREEZE, BREEZE_ON)

    # 鼓风机关闭  寄存器0x0009=2
    def send_breeze_off(self):
        self.write_reg(REG_BREEZE, BREEZE_OFF)

    # 吸布开启  寄存器0x000a=1
    def send_inhale_on(self):
        self.write_reg(REG_INHALE, INHALE_ON)

    # 吸布关闭  寄存器0x000a=2
    def send_inhale_off(self):
        self.write_reg(REG_INHALE, INHALE_OFF)

    # 抚平开始  寄存器0x000b=1
    def send_flatten_start(self):
        self.write_reg(REG_FLATTEN, FLATTEN_START)

    # 抚平返回  寄存器0x000b=2
    def send_flatten_return(self):
        self.write_reg(REG_FLATTEN, FLATTEN_RETURN)

    # 抚平复位  寄存器0x000b=3
    def send_flatten_reset(self):
        self.write_reg(REG_FLATTEN, FLATTEN_RESET)

    # ===== 04读指令：状态查询 =====
    # 故障码  寄存器0x0000
    def query_err(self):
        v = self.read_reg(REG_ERR)
        self.log("故障码：{}".format(translate_error(v) if v is not None else "读取失败"))

    # 机器状态  寄存器0x0001
    def query_machine(self):
        v = self.read_reg(REG_MACHINE)
        self.log("机器状态：{}".format(translate_machine(v) if v is not None else "读取失败"))

    # 折叠状态  寄存器0x0002
    def query_fold(self):
        v = self.read_reg(REG_FOLD_STATUS)
        self.log("折叠状态：{}".format(translate_fold(v) if v is not None else "读取失败"))

    # 鼓风机状态  寄存器0x0003
    def query_breeze_state(self):
        v = self.read_reg(REG_BREEZE_STATE)
        self.log("鼓风机状态：{}".format(translate_breeze_state(v) if v is not None else "读取失败"))

    # 吸布状态  寄存器0x0004
    def query_inhale_state(self):
        v = self.read_reg(REG_INHALE_STATE)
        self.log("吸布状态：{}".format(translate_inhale_state(v) if v is not None else "读取失败"))

    # 指令准入状态  寄存器0x0005
    def query_allow_status(self):
        v = self.read_reg(REG_ALLOW_STATUS)
        self.log("指令准入状态：{}".format(translate_allow_status(v)))

    # 抚平状态  寄存器0x0006
    def query_flatten_state(self):
        v = self.read_reg(REG_FLATTEN_STATE)
        self.log("抚平状态：{}".format(translate_flatten_state(v) if v is not None else "读取失败"))


if __name__ == "__main__":
    window = tk.Tk()
    app = V6ControlApp(window)
    window.mainloop()
