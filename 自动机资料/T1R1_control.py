# -*- coding: utf-8 -*-
"""
T1-R1 MODBUS RTU 机器控制器
从站ID=01，功能码 03读/06写。
485配置：115200 8数据位 无校验 1停止位
寄存器：
  RO  0x1000 T1运行状态   bit0:0未复位/1已复位  bit1:0缝纫未完成/1缝纫完成
  RO  0x1001 T1故障查询   0无故障, 非零有故障(带故障码)
  RO  0x1004 自动化设备等待查询   0-未在等待下一步 1-在等待下一步
  RO  0x1014 吸风状态查询   0-未吸风 1-正在吸风
  RO  0x1020 夹具IO状态(1-16ch) bit0=ch1, 1=上电
  RO  0x1021 夹具IO状态(17-24ch)  bit0=ch17, 1=上电
  RW  0x1500 压框           0-抬起 1-下压
  RW  0x1501 工作           1-开始工作 0-暂停
  RW  0x1502 吸风           1-打开 0-关闭
  RW  0x1520 自动化设备下一步  1-下一步
  RW  0x1530 复位             1-复位
"""
import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import struct
import time

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

class T1R1ControlApp:
    # PUREG 寄存器映射（详见 PUREG_modbus_format.md）
    PUREG_BASE = 0x2000       # PUREG[0..1023] 起始寄存器，每条动作占2个u16
    CLAMP_IDX_REG = 0x2400    # sv.clamp_idx：当前执行到第几条 IO 动作(0~511)

    # type 动作码含义（由 clampControl() 状态机推导）
    TYPE_NAMES = {
        1:  "输出(io_num通道置vol_level，再延时delay)",
        11: "纯延时(delay ms)",
        2:  "等待前进输入触发(前段传感器)",
        6:  "等待R1下一步(写0x1520)",
        3:  "结束(本段完成, clamp_idx归0)",
        4:  "结束变体(带输入等待窗口)",
        5:  "重启(踏板重新触发)",
        21: "等待输入状态(通用)",
    }

    def __init__(self, root):
        self.root = root
        self.root.title("T1-R1 Modbus RTU 机器控制器")
        self.root.geometry("820x760")

        self.ser = None

        # 串口配置区域
        port_frame = ttk.LabelFrame(root, text="串口配置")
        port_frame.pack(padx=10, pady=5, fill=tk.X)

        ttk.Label(port_frame, text="串口：").grid(row=0, column=0, padx=5, pady=5)
        self.port_combo = ttk.Combobox(port_frame, width=12)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)

        self.refresh_btn = ttk.Button(port_frame, text="刷新串口", command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=2, padx=5, pady=5)

        ttk.Label(port_frame, text="波特率：").grid(row=0, column=3, padx=5, pady=5)
        self.baud_label = ttk.Label(port_frame, text="115200", foreground="blue")
        self.baud_label.grid(row=0, column=4, padx=5, pady=5)

        self.open_btn = ttk.Button(port_frame, text="打开串口", command=self.open_serial)
        self.open_btn.grid(row=0, column=5, padx=8, pady=5)
        self.close_btn = ttk.Button(port_frame, text="关闭串口", command=self.close_serial, state=tk.DISABLED)
        self.close_btn.grid(row=0, column=6, padx=8, pady=5)

        # 控制按钮区域
        btn_frame = ttk.LabelFrame(root, text="控制按钮")
        btn_frame.pack(padx=10, pady=8, fill=tk.X)

        # 第一行：压框/工作
        ttk.Button(btn_frame, text="压框下压", command=self.send_press_down, width=16).grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(btn_frame, text="压框抬起", command=self.send_press_up, width=16).grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(btn_frame, text="开始工作", command=self.send_work_start, width=16).grid(row=0, column=2, padx=6, pady=6)
        ttk.Button(btn_frame, text="暂停工作", command=self.send_work_pause, width=16).grid(row=0, column=3, padx=6, pady=6)

        # 第二行：吸风/自动化下一步
        ttk.Button(btn_frame, text="打开吸风", command=self.send_suction_open, width=16).grid(row=1, column=0, padx=6, pady=6)
        ttk.Button(btn_frame, text="关闭吸风", command=self.send_suction_close, width=16).grid(row=1, column=1, padx=6, pady=6)
        ttk.Button(btn_frame, text="自动化下一步", command=self.send_auto_next, width=16).grid(row=1, column=2, padx=6, pady=6)
        ttk.Button(btn_frame, text="复位", command=self.send_reset, width=16).grid(row=1, column=3, padx=6, pady=6)

        # 状态查询区域
        query_frame = ttk.LabelFrame(root, text="状态查询")
        query_frame.pack(padx=10, pady=5, fill=tk.X)

        ttk.Button(query_frame, text="查询复位状态", command=self.query_reset_status, width=20).grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(query_frame, text="查询缝纫状态", command=self.query_sewing_status, width=20).grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(query_frame, text="查询T1故障码", command=self.query_t1_fault, width=20).grid(row=0, column=2, padx=6, pady=6)
        ttk.Button(query_frame, text="查询吸风状态", command=self.query_suction, width=20).grid(row=0, column=3, padx=6, pady=6)

        ttk.Button(query_frame, text="查询自动化等待", command=self.query_auto_wait, width=20).grid(row=1, column=0, padx=6, pady=6)
        ttk.Button(query_frame, text="查询夹具IO状态(1-16ch)", command=self.query_m4_output, width=20).grid(row=1, column=1, padx=6, pady=6)
        ttk.Button(query_frame, text="查询夹具IO状态(17-24ch)", command=self.query_m4_output_high, width=20).grid(row=1, column=2, padx=6, pady=6)

        # PUREG 动作查询区域
        pureg_frame = ttk.LabelFrame(root, text="PUREG 动作查询")
        pureg_frame.pack(padx=10, pady=5, fill=tk.X)

        ttk.Button(pureg_frame, text="查询当前动作位置K", command=self.query_current_clamp_idx, width=18).grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(pureg_frame, text="查询当前执行动作", command=self.query_current_action, width=18).grid(row=0, column=1, padx=6, pady=6)

        ttk.Label(pureg_frame, text="动作索引 k：").grid(row=0, column=2, padx=6, pady=6)
        self.k_entry = ttk.Entry(pureg_frame, width=8)
        self.k_entry.grid(row=0, column=3, padx=2, pady=6)
        ttk.Button(pureg_frame, text="查询第k条动作", command=self.query_action_by_index, width=16).grid(row=0, column=4, padx=6, pady=6)

        # 通信日志区域
        log_frame = ttk.LabelFrame(root, text="通信日志")
        log_frame.pack(padx=10, pady=5, fill=tk.BOTH, expand=True)
        self.log_text = scrolledtext.ScrolledText(log_frame, width=95, height=16)
        self.log_text.pack(padx=5, pady=5, fill=tk.BOTH, expand=True)

        self.refresh_ports()

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
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.5
            )
            self.log("串口已打开：{} 115200 8数据位 无校验 1停止位".format(port))
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

    # 03读指令：发送并读取应答
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

    # ===== 06写指令：动作 =====
    # 压框下压  寄存器0x1500=1
    def send_press_down(self):
        data = bytes([0x01, 0x06, 0x15, 0x00, 0x00, 0x01])
        self.send_modbus_no_reply(data)

    # 压框抬起  寄存器0x1500=0
    def send_press_up(self):
        data = bytes([0x01, 0x06, 0x15, 0x00, 0x00, 0x00])
        self.send_modbus_no_reply(data)

    # 开始工作  寄存器0x1501=1
    def send_work_start(self):
        data = bytes([0x01, 0x06, 0x15, 0x01, 0x00, 0x01])
        self.send_modbus_no_reply(data)

    # 暂停工作  寄存器0x1501=0
    def send_work_pause(self):
        data = bytes([0x01, 0x06, 0x15, 0x01, 0x00, 0x00])
        self.send_modbus_no_reply(data)

    # 打开吸风  寄存器0x1502=1
    def send_suction_open(self):
        data = bytes([0x01, 0x06, 0x15, 0x02, 0x00, 0x01])
        self.send_modbus_no_reply(data)

    # 关闭吸风  寄存器0x1502=0
    def send_suction_close(self):
        data = bytes([0x01, 0x06, 0x15, 0x02, 0x00, 0x00])
        self.send_modbus_no_reply(data)

    # 自动化下一步  寄存器0x1520=1
    def send_auto_next(self):
        data = bytes([0x01, 0x06, 0x15, 0x20, 0x00, 0x01])
        self.send_modbus_no_reply(data)

    # 复位  寄存器0x1530=1
    def send_reset(self):
        data = bytes([0x01, 0x06, 0x15, 0x30, 0x00, 0x01])
        self.send_modbus_no_reply(data)

    # ===== 03读指令：状态查询 =====
    # 查询复位状态  寄存器0x1000 bit0
    def query_reset_status(self):
        data = bytes([0x01, 0x03, 0x10, 0x00, 0x00, 0x01])
        resp = self.send_modbus_with_reply(data)
        if resp and len(resp) >= 7:
            val = (resp[3] << 8) | resp[4]
            if val & 0x01:
                self.log("复位状态：已复位，可执行")
            else:
                self.log("复位状态：未复位，不可执行")

    # 查询缝纫状态  寄存器0x1000 bit1
    def query_sewing_status(self):
        data = bytes([0x01, 0x03, 0x10, 0x00, 0x00, 0x01])
        resp = self.send_modbus_with_reply(data)
        if resp and len(resp) >= 7:
            val = (resp[3] << 8) | resp[4]
            if val & 0x02:
                self.log("缝纫状态：缝纫已完成")
            else:
                self.log("缝纫状态：缝纫未完成")

    # 查询T1故障码  寄存器0x1001
    def query_t1_fault(self):
        data = bytes([0x01, 0x03, 0x10, 0x01, 0x00, 0x01])
        resp = self.send_modbus_with_reply(data)
        if resp and len(resp) >= 7:
            val = (resp[3] << 8) | resp[4]
            if val == 0:
                self.log("T1故障码：无故障")
            else:
                self.log("T1故障码：有故障(0x{:04X})".format(val))

    # 查询吸风状态  寄存器0x1014
    def query_suction(self):
        data = bytes([0x01, 0x03, 0x10, 0x14, 0x00, 0x01])
        resp = self.send_modbus_with_reply(data)
        if resp and len(resp) >= 7:
            val = (resp[3] << 8) | resp[4]
            if val == 0:
                self.log("吸风状态：未吸风")
            elif val == 1:
                self.log("吸风状态：正在吸风")

    # 查询自动化设备等待  寄存器0x1004
    def query_auto_wait(self):
        data = bytes([0x01, 0x03, 0x10, 0x04, 0x00, 0x01])
        resp = self.send_modbus_with_reply(data)
        if resp and len(resp) >= 7:
            val = (resp[3] << 8) | resp[4]
            if val == 0:
                self.log("自动化设备等待：未在等待下一步指令")
            elif val == 1:
                self.log("自动化设备等待：在等待下一步指令")
            else:
                self.log("自动化设备等待：值=0x{:04X}".format(val))

    # 查询M4输出状态 寄存器0x1020  bit0=ch1, 1=活动
    def query_m4_output(self):
        data = bytes([0x01, 0x03, 0x10, 0x20, 0x00, 0x01])
        resp = self.send_modbus_with_reply(data)
        if resp and len(resp) >= 7:
            val = (resp[3] << 8) | resp[4]
            active = ["ch{}".format(i + 1) for i in range(16) if val & (1 << i)]
            if active:
                self.log("夹具IO状态(1-16ch)：0x{:04X}，上电：{}".format(val, ", ".join(active)))
            else:
                self.log("夹具IO状态(1-16ch)：0x{:04X}，无活动通道".format(val))

    # 查询M4输出状态  寄存器0x1021  bit0=ch17, 1=活动(高字节恒为0)
    def query_m4_output_high(self):
        data = bytes([0x01, 0x03, 0x10, 0x21, 0x00, 0x01])
        resp = self.send_modbus_with_reply(data)
        if resp and len(resp) >= 7:
            val = (resp[3] << 8) | resp[4]
            low = val & 0x00FF
            active = ["ch{}".format(i + 17) for i in range(8) if low & (1 << i)]
            if active:
                self.log("夹具IO状态(17-24ch)：0x{:04X}，上电：{}".format(val, ", ".join(active)))
            else:
                self.log("夹具IO状态(17-24ch)：0x{:04X}，无活动通道".format(val))

    # ===== 03读指令：PUREG 动作查询 =====
    # 通用：读单个寄存器，返回 u16（失败返回 None）
    def read_register(self, addr):
        data = bytes([0x01, 0x03, (addr >> 8) & 0xFF, addr & 0xFF, 0x00, 0x01])
        resp = self.send_modbus_with_reply(data)
        if resp and len(resp) >= 5:
            return (resp[3] << 8) | resp[4]
        return None

    # 通用：读连续 count 个寄存器，返回 list[u16]（失败返回 None）
    def read_registers(self, addr, count):
        data = bytes([0x01, 0x03, (addr >> 8) & 0xFF, addr & 0xFF,
                      (count >> 8) & 0xFF, count & 0xFF])
        resp = self.send_modbus_with_reply(data)
        if resp and len(resp) >= 3:
            byte_count = resp[2]
            vals = []
            for i in range(byte_count // 2):
                off = 3 + i * 2
                if off + 1 < len(resp):
                    vals.append((resp[off] << 8) | resp[off + 1])
            return vals
        return None

    # 按文档第5节解码第 k 条动作(V0/V1)
    def decode_action(self, k, v0, v1):
        type_code = v0 & 0x0F
        vol_level = (v0 >> 4) & 0x0F
        io_num = (v0 >> 8) & 0xFF
        delay_ms = v1
        type_desc = self.TYPE_NAMES.get(type_code, "未知动作码")
        return (
            "第 {} 条动作解码：\n"
            "  V0=0x{:04X}  V1=0x{:04X}\n"
            "  type      = {} ({})\n"
            "  vol_level = {}\n"
            "  io_num    = {}\n"
            "  delay     = {} ms".format(k, v0, v1, type_code, type_desc, vol_level, io_num, delay_ms)
        )

    # 查询当前动作位置 K  寄存器0x2400
    def query_current_clamp_idx(self):
        k = self.read_register(self.CLAMP_IDX_REG)
        if k is None:
            self.log("查询当前动作位置失败")
            return
        self.log("当前动作位置 K = {} (clamp_idx)".format(k))

    # 查询第 k 条动作属性并解码  寄存器0x2000+2k, 0x2000+2k+1
    def query_action_by_index(self):
        txt = self.k_entry.get().strip()
        try:
            k = int(txt)
        except ValueError:
            self.log("请输入有效的动作索引 k")
            return
        if not (0 <= k <= 511):
            self.log("k 范围应为 0~511")
            return
        vals = self.read_registers(self.PUREG_BASE + 2 * k, 2)
        if not vals or len(vals) < 2:
            self.log("读取第 {} 条动作失败".format(k))
            return
        self.log(self.decode_action(k, vals[0], vals[1]))

    # 查询当前正在执行的动作(读K后解码第K条)
    def query_current_action(self):
        k = self.read_register(self.CLAMP_IDX_REG)
        if k is None:
            self.log("查询当前动作位置失败")
            return
        self.log("当前动作位置 K = {}".format(k))
        vals = self.read_registers(self.PUREG_BASE + 2 * k, 2)
        if not vals or len(vals) < 2:
            self.log("读取当前动作(第 {} 条)失败".format(k))
            return
        self.log(self.decode_action(k, vals[0], vals[1]))

if __name__ == "__main__":
    window = tk.Tk()
    app = T1R1ControlApp(window)
    window.mainloop()
