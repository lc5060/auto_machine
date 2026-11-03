import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import os
import pty
import select
import tty
import serial
import serial.tools.list_ports
import threading
import time
import queue

SLAVE_ID = 0x02          # 主控从站地址
REG_RESET = 0x0000       # 复位主控
REG_FABRIC_READY = 0x0001  # 大身布料已放好
REG_POCKET_READY = 0x0002  # 口袋布已放好
IN_STATUS = 0x0001       # 主控状态（输入寄存器，0x04）


def crc16(data):
    """Modbus CRC16（多项式 0xA001，初值 0xFFFF）。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(pdu):
    """在 PDU 后追加 CRC（低字节在前）。"""
    crc = crc16(pdu)
    return bytes(pdu) + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def build_write_single(address, value):
    """功能码 0x06：写单个保持寄存器。"""
    return build_frame(bytes((
        SLAVE_ID, 0x06,
        (address >> 8) & 0xFF, address & 0xFF,
        (value >> 8) & 0xFF, value & 0xFF,
    )))


def build_read_holding(start, count):
    """功能码 0x03：读保持寄存器。"""
    return build_frame(bytes((
        SLAVE_ID, 0x03,
        (start >> 8) & 0xFF, start & 0xFF,
        (count >> 8) & 0xFF, count & 0xFF,
    )))


def build_read_input(start, count):
    """功能码 0x04：读输入寄存器（主控状态区）。"""
    return build_frame(bytes((
        SLAVE_ID, 0x04,
        (start >> 8) & 0xFF, start & 0xFF,
        (count >> 8) & 0xFF, count & 0xFF,
    )))


STATUS_TEXT = {
    0: "待机",
    1: "准备就绪",
    10: "执行中",
    11: "执行完成待确认",
    20: "暂停",
    100: "故障",
}

# 故障码由主控定义，协议文档未给出码表；拿到主控码表后在这里补充
FAULT_TEXT = {
    0: "无故障",
}


def fault_text(code):
    return FAULT_TEXT.get(code, "协议未定义")


def hex_str(data):
    return " ".join(f"{b:02X}" for b in data)


class RosBridgeClient:
    """通过 ROS 服务/话题操作 /serial_bridge，串口由 ROS 节点独占。

    这样页面与其它 ROS 节点（导航、视觉等）可以同时使用同一路串口。
    需要先 source ROS 环境并启动 serial_bridge。
    """

    def __init__(self, log_queue):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import Bool, String, UInt16MultiArray
        from std_srvs.srv import Trigger

        self.log_queue = log_queue
        self._rclpy = rclpy
        self._Trigger = Trigger
        self._UInt16 = UInt16MultiArray

        rclpy.init(args=None)
        self.node = rclpy.create_node("serial_assistant_ui")

        self.pub_write = self.node.create_publisher(
            UInt16MultiArray, "/serial_bridge/write_register", 10
        )
        self.pub_read = self.node.create_publisher(
            UInt16MultiArray, "/serial_bridge/read_registers", 10
        )
        self.pub_read_holding = self.node.create_publisher(
            UInt16MultiArray, "/serial_bridge/read_holding", 10
        )
        self.cli_reset = self.node.create_client(Trigger, "/serial_bridge/reset")
        self.cli_fabric = self.node.create_client(Trigger, "/serial_bridge/fabric_ready")
        self.cli_pocket = self.node.create_client(Trigger, "/serial_bridge/pocket_ready")
        self.cli_status = self.node.create_client(Trigger, "/serial_bridge/read_status")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, "/serial_bridge/rx_hex", self._on_hex, 10)
        self.node.create_subscription(
            UInt16MultiArray, "/serial_bridge/registers", self._on_regs, 10
        )
        self.node.create_subscription(Bool, "/serial_bridge/connected", self._on_connected, latched)

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------ 发送
    def publish_write(self, address, value):
        msg = self._UInt16()
        msg.data = [address & 0xFFFF, value & 0xFFFF]
        self.pub_write.publish(msg)

    def publish_read(self, start, count):
        """0x04 输入寄存器（主控状态）。"""
        msg = self._UInt16()
        msg.data = [start & 0xFFFF, count & 0xFFFF]
        self.pub_read.publish(msg)

    def publish_read_holding(self, start, count):
        """0x03 保持寄存器（命令区）。"""
        msg = self._UInt16()
        msg.data = [start & 0xFFFF, count & 0xFFFF]
        self.pub_read_holding.publish(msg)

    def call(self, client, label):
        if not client.service_is_ready():
            self.log_queue.put(f"【ROS】{client.srv_name} 不可用，请先启动 serial_bridge\n")
            return False
        future = client.call_async(self._Trigger.Request())
        future.add_done_callback(lambda fut: self._done(fut, label))
        return True

    def _done(self, future, label):
        try:
            result = future.result()
            text = result.message if result is not None else "无返回"
            self.log_queue.put(f"【ROS】{label}: {text}\n")
        except Exception as exc:
            self.log_queue.put(f"【ROS】{label} 调用失败: {exc}\n")

    # ------------------------------------------------------------ 接收
    def _on_hex(self, msg):
        self.log_queue.put(f"【接收←】{msg.data}\n")

    def _on_regs(self, msg):
        data = list(msg.data)
        if not data:
            return
        values = " ".join(f"0x{v:04X}" for v in data[1:])
        self.log_queue.put(f"【应答←】0x{data[0]:04X}: {values}\n")

    def _on_connected(self, msg):
        self.log_queue.put(f"【ROS】串口{('已连接' if msg.data else '未连接')}\n")

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=0.5)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self._rclpy.try_shutdown()
        except Exception:
            pass


class SerialAssistant:
    def __init__(self, root):
        self.root = root
        self.root.title("串口调试助手（真实串口 / 虚拟串口）")
        self.root.geometry("700x520")

        self.ser = None
        self.is_open = False
        self.receive_thread = None
        self.auto_send_flag = False
        self.auto_send_thread = None
        self.rx_buffer = bytearray()   # 应答解析缓存
        self.cmd_history = []          # 命令历史
        self.cmd_index = 0
        self.pending_read = None       # 最近一次读请求 (起始地址, 个数)
        self.serial_write_lock = threading.Lock()
        self.status_snapshot = None    # 轮询期间最近一次状态
        self.polling = False           # 正在后台轮询状态（抑制刷屏）
        self.rx_queue = queue.Queue()  # 接收线程 -> 主线程 的数据通道
        self.log_queue = queue.Queue()  # 后台线程 -> 界面的提示文字

        self.virtual_running = False   # 内置虚拟从站状态
        self.virtual_master = None
        self.virtual_slave_fd = None
        self.virtual_path = ""
        self.virtual_regs = {}
        self.virtual_inputs = {}       # 虚拟从站输入寄存器（0x04）
        self.ros = None                # ROS 模式客户端

        top_frame = ttk.Frame(root, padding=4)
        top_frame.pack(fill="x")

        row1 = ttk.Frame(top_frame)
        row1.pack(fill="x")
        ttk.Label(row1, text="串口:").pack(side="left", padx=2)
        # 可手动输入路径，也可点“创建虚拟串口”自动生成
        self.port_combo = ttk.Combobox(row1, width=28, state="normal")
        self.port_combo.pack(side="left", padx=2)
        self.refresh_btn = ttk.Button(row1, text="刷新", command=self.refresh_ports, width=5)
        self.refresh_btn.pack(side="left", padx=2)
        self.virtual_btn = ttk.Button(
            row1, text="创建虚拟串口", command=self.toggle_virtual_port, width=12
        )
        self.virtual_btn.pack(side="left", padx=2)
        self.open_btn = ttk.Button(row1, text="打开串口", command=self.toggle_serial, width=10)
        self.open_btn.pack(side="left", padx=8)
        self.ros_mode_var = tk.BooleanVar()
        self.ros_check = ttk.Checkbutton(
            row1, text="ROS 模式", variable=self.ros_mode_var, command=self.on_ros_mode_changed
        )
        self.ros_check.pack(side="left", padx=4)

        row2 = ttk.Frame(top_frame)
        row2.pack(fill="x", pady=(4, 0))
        ttk.Label(row2, text="波特率:").pack(side="left", padx=2)
        self.baud_combo = ttk.Combobox(row2, width=8, values=[
            "9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"
        ], state="readonly")
        self.baud_combo.current(4)
        self.baud_combo.pack(side="left", padx=2)

        ttk.Label(row2, text="数据位:").pack(side="left", padx=2)
        self.databits_combo = ttk.Combobox(row2, width=4, values=["5", "6", "7", "8"], state="readonly")
        self.databits_combo.current(3)
        self.databits_combo.pack(side="left", padx=2)

        ttk.Label(row2, text="停止位:").pack(side="left", padx=2)
        self.stopbits_combo = ttk.Combobox(row2, width=4, values=["1", "1.5", "2"], state="readonly")
        self.stopbits_combo.current(0)
        self.stopbits_combo.pack(side="left", padx=2)

        ttk.Label(row2, text="校验:").pack(side="left", padx=2)
        self.parity_combo = ttk.Combobox(row2, width=4, values=["无", "奇", "偶"], state="readonly")
        self.parity_combo.current(1)  # 现场主控为 8O1（奇校验）
        self.parity_combo.pack(side="left", padx=2)
        ttk.Label(row2, text="虚拟串口不受波特率影响", foreground="#777").pack(side="left", padx=10)

        quick_frame = ttk.LabelFrame(root, text="快捷指令", padding=4)
        quick_frame.pack(fill="x", padx=4)
        ttk.Button(quick_frame, text="复位主控", command=self.send_reset, width=9).pack(side="left", padx=4, pady=2)
        ttk.Button(quick_frame, text="大身布料", command=self.send_fabric_ready, width=9).pack(side="left", padx=4, pady=2)
        ttk.Button(quick_frame, text="口袋布", command=self.send_pocket_ready, width=9).pack(side="left", padx=4, pady=2)
        ttk.Button(quick_frame, text="读状态", command=self.send_status, width=8).pack(side="left", padx=4, pady=2)
        ttk.Button(quick_frame, text="命令帮助", command=self.show_help, width=8).pack(side="right", padx=4, pady=2)

        cmd_frame = ttk.LabelFrame(root, text="命令控制台（回车发送，输入 help 查看命令）", padding=4)
        cmd_frame.pack(fill="x", padx=4, pady=2)
        self.cmd_entry = ttk.Entry(cmd_frame, font=("Consolas", 9))
        self.cmd_entry.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.cmd_entry.bind("<Return>", lambda event: self.send_command())
        self.cmd_entry.bind("<Up>", lambda event: self._history_prev())
        self.cmd_entry.bind("<Down>", lambda event: self._history_next())
        ttk.Button(cmd_frame, text="发送命令", width=8, command=self.send_command).pack(side="left")

        recv_frame = ttk.LabelFrame(root, text="接收区", padding=4)
        recv_frame.pack(fill="both", expand=True, padx=4, pady=2)

        self.recv_text = scrolledtext.ScrolledText(recv_frame, wrap="word", font=("Consolas", 9))
        self.recv_text.pack(fill="both", expand=True)

        recv_opt_frame = ttk.Frame(recv_frame)
        recv_opt_frame.pack(fill="x", pady=2)
        self.hex_recv_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(recv_opt_frame, text="HEX 显示", variable=self.hex_recv_var).pack(side="left")
        self.parse_reply_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(recv_opt_frame, text="解析应答", variable=self.parse_reply_var).pack(side="left", padx=8)
        ttk.Button(recv_opt_frame, text="清空接收", command=self.clear_recv, width=6).pack(side="right")

        send_frame = ttk.LabelFrame(root, text="发送区", padding=4)
        send_frame.pack(fill="x", padx=4, pady=2)

        self.send_text = scrolledtext.ScrolledText(send_frame, height=3, wrap="word", font=("Consolas", 9))
        self.send_text.pack(fill="x")

        send_opt_frame = ttk.Frame(send_frame)
        send_opt_frame.pack(fill="x", pady=2)

        self.hex_send_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(send_opt_frame, text="HEX 发送", variable=self.hex_send_var).pack(side="left", padx=2)

        ttk.Label(send_opt_frame, text="定时(ms):").pack(side="left", padx=2)
        self.auto_interval = ttk.Entry(send_opt_frame, width=6)
        self.auto_interval.insert(0, "1000")
        self.auto_interval.pack(side="left", padx=1)

        self.auto_send_var = tk.BooleanVar()
        self.auto_send_check = ttk.Checkbutton(send_opt_frame, text="自动发送", variable=self.auto_send_var, command=self.toggle_auto_send)
        self.auto_send_check.pack(side="left", padx=2)

        ttk.Button(send_opt_frame, text="发送", command=self.send_data, width=5).pack(side="right", padx=2)
        ttk.Button(send_opt_frame, text="清空发送", command=self.clear_send, width=6).pack(side="right", padx=2)

        self.status_var = tk.StringVar()
        self.status_var.set("就绪 | 串口未打开")
        status_bar = ttk.Label(root, textvariable=self.status_var, anchor="w", padding=2)
        status_bar.pack(fill="x", side="bottom")

        self.refresh_ports()
        self.root.after(30, self._poll_rx)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _poll_rx(self):
        """主线程定时消费接收队列（Tkinter 只能在主线程操作控件）。"""
        try:
            while True:
                self.show_receive(self.rx_queue.get_nowait())
        except queue.Empty:
            pass
        try:
            while True:
                self._log(self.log_queue.get_nowait())
        except queue.Empty:
            pass
        try:
            self.root.after(30, self._poll_rx)
        except tk.TclError:
            pass

    def _log(self, text):
        self.recv_text.insert("end", text)
        self.recv_text.see("end")

    def send_reset(self):
        if self.ros is not None:
            if self.ros.call(self.ros.cli_reset, "复位主控"):
                self._log("【发送→】复位主控   (ROS 服务)\n")
            return
        if self._send_frame(build_write_single(REG_RESET, 1), "复位主控"):
            self._start_watch("复位主控", lambda v: v[0] in (0, 1), 15.0)

    def send_fabric_ready(self):
        if self.ros is not None:
            if self.ros.call(self.ros.cli_fabric, "大身布料已放好"):
                self._log("【发送→】大身布料已放好   (ROS 服务)\n")
            return
        if self._send_frame(build_write_single(REG_FABRIC_READY, 1), "大身布料已放好"):
            self._start_watch("大身布料", lambda v: v[2] == 1 or v[0] == 11, 30.0)

    def send_pocket_ready(self):
        if self.ros is not None:
            if self.ros.call(self.ros.cli_pocket, "口袋布已放好"):
                self._log("【发送→】口袋布已放好   (ROS 服务)\n")
            return
        if self._send_frame(build_write_single(REG_POCKET_READY, 1), "口袋布已放好"):
            self._start_watch("口袋布", lambda v: v[3] == 1 or v[0] == 11, 30.0)

    def send_status(self):
        if self.ros is not None:
            if self.ros.call(self.ros.cli_status, "读主控状态"):
                self._log("【发送→】读主控状态   (ROS 服务)\n")
            return
        self.pending_read = (IN_STATUS, 4)
        self._send_frame(build_read_input(IN_STATUS, 4), "读主控状态 0x0001 x4")

    # ------------------------------------------------------------ 命令控制台
    def send_command(self):
        text = self.cmd_entry.get().strip()
        if not text:
            return
        self.cmd_entry.delete(0, "end")
        self.cmd_history.append(text)
        self.cmd_index = len(self.cmd_history)

        self.recv_text.insert("end", f"\n> {text}\n")
        self.recv_text.see("end")

        if text.lower() in ("help", "帮助", "?"):
            self.show_help()
            return

        try:
            frame, label = self._parse_command(text)
        except ValueError as exc:
            messagebox.showerror("命令错误", str(exc))
            return

        if frame is not None:
            if self.ros is not None:
                self._send_via_ros(frame, label)
            elif self._send_frame(frame, label):
                self._watch_for_frame(frame)

    def _parse_command(self, text):
        """把一行命令解析成 (帧, 说明)；仅提示类命令返回 (None, '')。"""
        parts = text.replace(",", " ").split()
        name = parts[0].lower()
        args = parts[1:]

        if name in ("reset", "复位", "复位主控"):
            return build_write_single(REG_RESET, 1), "复位主控"

        if name in ("fabric", "body", "ready", "大身", "大身布料", "布料", "布料已放好", "放料"):
            return build_write_single(REG_FABRIC_READY, 1), "大身布料已放好"

        if name in ("pocket", "口袋", "口袋布", "口袋布已放好"):
            return build_write_single(REG_POCKET_READY, 1), "口袋布已放好"

        if name in ("status", "状态"):
            self.pending_read = (IN_STATUS, 4)
            return build_read_input(IN_STATUS, 4), "读主控状态 0x0001 x4"

        if name in ("write", "w", "写"):
            if len(args) < 2:
                raise ValueError("用法: write <地址> <值>\n例如: write 0x0005 100")
            address = self._to_int(args[0])
            value = self._to_int(args[1])
            return build_write_single(address, value), f"写寄存器 0x{address:04X} = 0x{value:04X}"

        if name in ("read", "r", "读"):
            if len(args) < 1:
                raise ValueError("用法: read <起始地址> [个数]\n例如: read 1 4")
            start = self._to_int(args[0])
            count = self._to_int(args[1]) if len(args) > 1 else 1
            if count <= 0 or count > 125:
                raise ValueError("读取个数需在 1~125 之间")
            self.pending_read = (start, count)
            return build_read_input(start, count), f"读输入寄存器 0x{start:04X} x{count}"

        if name in ("readh", "读保持"):
            if len(args) < 1:
                raise ValueError("用法: readh <起始地址> [个数]\n例如: readh 0 2")
            start = self._to_int(args[0])
            count = self._to_int(args[1]) if len(args) > 1 else 1
            self.pending_read = None
            return build_read_holding(start, count), f"读保持寄存器 0x{start:04X} x{count}"

        if name in ("hex", "raw"):
            if not args:
                raise ValueError("用法: hex <十六进制字节>\n例如: hex 02 06 00 00 00 01 48 39")
            try:
                data = bytes.fromhex("".join(args))
            except ValueError:
                raise ValueError("十六进制格式不正确，例如: hex 02 06 00 00 00 01 48 39")
            return data, "原始帧"

        if name in ("slave", "地址"):
            raise ValueError(f"从站地址固定为 0x{SLAVE_ID:02X}（可在文件开头 SLAVE_ID 修改）")

        raise ValueError(f"未知命令: {parts[0]}，输入 help 查看支持的命令")

    @staticmethod
    def _to_int(text):
        try:
            value = int(text, 0)
        except ValueError:
            raise ValueError(f"无法识别的数值: {text}（支持 100 或 0x64）")
        if value < 0 or value > 0xFFFF:
            raise ValueError(f"数值超出 0~0xFFFF: {text}")
        return value

    def _history_prev(self):
        if not self.cmd_history:
            return
        self.cmd_index = max(0, self.cmd_index - 1)
        self.cmd_entry.delete(0, "end")
        self.cmd_entry.insert(0, self.cmd_history[self.cmd_index])

    def _history_next(self):
        if not self.cmd_history:
            return
        self.cmd_index = min(len(self.cmd_history), self.cmd_index + 1)
        self.cmd_entry.delete(0, "end")
        if self.cmd_index < len(self.cmd_history):
            self.cmd_entry.insert(0, self.cmd_history[self.cmd_index])

    def show_help(self):
        lines = [
            "",
            "可用命令（回车发送）:",
            "  reset / 复位            复位主控(0x0000=1)     02 06 00 00 00 01 48 39",
            "  fabric / 大身布料       大身布料已放好(0x0001=1) 02 06 00 01 00 01 19 F9",
            "  pocket / 口袋           口袋布已放好(0x0002=1)  02 06 00 02 00 01 E9 F9",
            "  status / 状态           读主控状态 0x0001~0x0004",
            "  read <地址> [个数]      读输入寄存器(0x04)     例: read 1 4",
            "  readh <地址> [个数]     读保持寄存器(0x03)     例: readh 0 2",
            "  write <地址> <值>       写单个寄存器(0x06)     例: write 0x0005 100",
            "  hex <字节>              原样发送               例: hex 02 06 00 00 00 01 48 39",
            "  help / 帮助             显示本帮助",
            "",
            "协议（主控为从站 0x02，115200 8O1）:",
            "  保持寄存器(0x06 写): 0x0000=复位 0x0001=大身布料 0x0002=口袋布，写 1 有效",
            "  输入寄存器(0x04 读): 0x0001=主控状态 0x0002=故障码 0x0003=大身移料 0x0004=口袋移料",
            "  主控状态: 0 待机 / 1 准备就绪 / 10 执行中 / 11 完成待确认 / 20 暂停 / 100 故障",
            "  移料状态: 0 未成功 / 1 成功到位",
            "",
            f"从站地址: 0x{SLAVE_ID:02X}，地址/数值支持十进制或 0x 前缀，↑/↓ 可翻命令历史",
            "虚拟调试: 点“创建虚拟串口”自动生成，再点“打开串口”即可自测",
            "真实调试: 刷新后选 /dev/ttyUSB0，波特率 115200、校验【奇】",
            "ROS 并行: 勾选“ROS 模式”并连接后，串口由 serial_bridge 独占，其它 ROS 节点可同时使用",
            "ROS 模式: reset/fabric/pocket/status 走服务；read/readh/write 走话题；原始 hex 需用直连模式",
            "",
        ]
        self.recv_text.insert("end", "\n".join(lines))
        self.recv_text.see("end")

    def _send_frame(self, frame, label=""):
        if not self.is_open or not self.ser or not self.ser.is_open:
            messagebox.showwarning("提示", "请先打开串口")
            return False
        try:
            with self.serial_write_lock:
                self.ser.write(frame)
        except Exception as exc:
            messagebox.showerror("发送错误", str(exc))
            return False

        self.recv_text.insert("end", f"【发送→】{label}   {hex_str(frame)}\n")
        self.recv_text.see("end")
        return True

    # ------------------------------------------------------------ 闭环检测
    def _watch_for_frame(self, frame):
        """0x06 写的是复位/布料时，自动开启完成检测。"""
        if frame[1] != 0x06 or len(frame) != 8:
            return
        address = (frame[2] << 8) | frame[3]
        if address == REG_RESET:
            self._start_watch("复位主控", lambda v: v[0] in (0, 1), 15.0)
        elif address == REG_FABRIC_READY:
            self._start_watch("大身布料", lambda v: v[2] == 1 or v[0] == 11, 30.0)
        elif address == REG_POCKET_READY:
            self._start_watch("口袋布", lambda v: v[3] == 1 or v[0] == 11, 30.0)

    def _start_watch(self, name, predicate, timeout):
        if not self.is_open:
            return
        threading.Thread(
            target=self._watch_completion, args=(name, predicate, timeout), daemon=True
        ).start()

    def _watch_completion(self, name, predicate, timeout):
        """下发后轮询 0x04 状态，把“已完成/超时”结果打到接收区。"""
        self.log_queue.put(f"【等待】{name}：轮询主控状态...\n")
        start = time.time()
        busy_seen = False
        last = None
        self.polling = True
        try:
            while self.is_open and time.time() - start < timeout:
                self.status_snapshot = None
                self._send_status_query()
                time.sleep(0.2)
                values = self.status_snapshot
                if not values:
                    continue

                last = values
                status = values[0]
                if status == 100:
                    self.log_queue.put(
                        f"【完成检测】{name}: 主控报故障（故障码={values[1]}）\n"
                    )
                    return
                if status in (10, 11, 20):
                    busy_seen = True

                settled = busy_seen or (time.time() - start) >= 1.0
                if settled and predicate(values):
                    self.log_queue.put(
                        f"【完成检测】{name}: 已完成  {self._status_line(values)}\n"
                    )
                    return

            if last:
                self.log_queue.put(
                    f"【完成检测】{name}: {timeout:.0f}s 内未完成  最后 {self._status_line(last)}\n"
                )
            else:
                self.log_queue.put(f"【完成检测】{name}: {timeout:.0f}s 内无状态应答\n")
        finally:
            self.polling = False
            self.status_snapshot = None

    def _send_status_query(self):
        if not self.is_open or self.ser is None:
            return
        try:
            with self.serial_write_lock:
                self.ser.write(build_read_input(IN_STATUS, 4))
        except Exception:
            pass

    @staticmethod
    def _status_line(values):
        status, fault, body, pocket = (list(values) + [0, 0, 0, 0])[:4]
        return (
            f"状态={status}({STATUS_TEXT.get(status, '未知')}) "
            f"故障码={fault}({fault_text(fault)}) "
            f"大身移料={'成功' if body == 1 else '未成功'}({body}) "
            f"口袋移料={'成功' if pocket == 1 else '未成功'}({pocket})"
        )

    # ------------------------------------------------------------ ROS 模式
    def on_ros_mode_changed(self):
        if self.ros is not None:
            self._disconnect_ros()
        if self.is_open:
            self.toggle_serial()
        if self.ros_mode_var.get():
            self.open_btn.config(text="连接 ROS")
            self.status_var.set("ROS 模式 | 点击“连接 ROS”")
            self._log("\n【ROS】已切到 ROS 模式：串口由 serial_bridge 独占，页面通过话题/服务操作\n")
        else:
            self.open_btn.config(text="打开串口")
            self.status_var.set("就绪 | 串口未打开")

    def toggle_ros(self):
        if self.ros is not None:
            self._disconnect_ros()
            return
        try:
            self.ros = RosBridgeClient(self.log_queue)
        except Exception as exc:
            self.ros = None
            messagebox.showerror(
                "ROS 连接失败",
                f"{exc}\n\n请确认:\n"
                "1) 终端已 source /opt/ros/jazzy/setup.bash\n"
                "2) 已启动 ros2 run align_serial serial_bridge 或 virtual_test.launch.py",
            )
            return
        self.open_btn.config(text="断开 ROS")
        self.status_var.set("已连接 ROS | /serial_bridge")
        self._log("【ROS】已连接 /serial_bridge，串口由 ROS 节点独占，可与其它 ROS 节点并行\n")

    def _disconnect_ros(self):
        if self.ros is not None:
            self.ros.close()
            self.ros = None
            self._log("【ROS】已断开\n")
        if self.ros_mode_var.get():
            self.open_btn.config(text="连接 ROS")
            self.status_var.set("ROS 模式 | 未连接")
        else:
            self.open_btn.config(text="打开串口")
            self.status_var.set("就绪 | 串口未打开")

    def _send_via_ros(self, frame, label):
        """把命令帧翻译成 ROS 话题消息（服务/话题由 serial_bridge 执行）。"""
        func = frame[1]
        if func == 0x06 and len(frame) == 8:
            address = (frame[2] << 8) | frame[3]
            value = (frame[4] << 8) | frame[5]
            self.ros.publish_write(address, value)
        elif func == 0x04 and len(frame) == 8:
            start = (frame[2] << 8) | frame[3]
            count = (frame[4] << 8) | frame[5]
            self.ros.publish_read(start, count)
        elif func == 0x03 and len(frame) == 8:
            start = (frame[2] << 8) | frame[3]
            count = (frame[4] << 8) | frame[5]
            self.ros.publish_read_holding(start, count)
        else:
            self._log("【ROS】该命令在 ROS 模式下不支持，请切回串口直连模式\n")
            return
        self._log(f"【发送→】{label}   {hex_str(frame)}   (ROS)\n")

    # ------------------------------------------------------------ 内置虚拟从站
    def toggle_virtual_port(self):
        """一键创建/关闭内置虚拟从站（pty），用于无硬件联调。"""
        if not self.virtual_running and self.ros_mode_var.get():
            messagebox.showinfo(
                "提示",
                "ROS 模式下串口由 serial_bridge 持有。\n"
                "请用 ros2 launch align_serial virtual_test.launch.py 同时启动虚拟从站和桥接，"
                "本页面再连接 ROS。",
            )
            return
        if self.virtual_running:
            self._stop_virtual_port()
            self.virtual_btn.config(text="创建虚拟串口")
            self.port_combo.set("")
            self.status_var.set("就绪 | 虚拟串口已关闭")
            self._log("【虚拟从站】已关闭\n")
            return

        if self.is_open:
            self.toggle_serial()  # 先断开已打开的真实串口

        try:
            master_fd, slave_fd = pty.openpty()
            tty.setraw(master_fd)
            tty.setraw(slave_fd)
        except Exception as exc:
            messagebox.showerror("错误", f"创建虚拟串口失败: {exc}")
            return

        self.virtual_master = master_fd
        self.virtual_slave_fd = slave_fd
        self.virtual_path = os.ttyname(slave_fd)
        self.virtual_regs = {}
        # 输入寄存器（0x04 只读）：0x0001 状态初始为 1（准备就绪）
        self.virtual_inputs = {IN_STATUS: 1, 0x0002: 0, 0x0003: 0, 0x0004: 0}
        self.virtual_running = True

        self.port_combo.set(self.virtual_path)
        self.virtual_btn.config(text="关闭虚拟串口")
        self.status_var.set(f"虚拟从站已就绪 | {self.virtual_path}")
        self._log(
            f"\n【虚拟从站】已创建: {self.virtual_path}（从站地址 0x{SLAVE_ID:02X}）"
            f"，点击“打开串口”即可对接\n"
        )
        threading.Thread(target=self._virtual_slave_loop, daemon=True).start()

    def _stop_virtual_port(self):
        self.virtual_running = False
        if self.is_open and self.ser is not None and getattr(self.ser, "port", "") == self.virtual_path:
            self.toggle_serial()
        time.sleep(0.15)  # 等接收线程退出
        for fd in (self.virtual_master, self.virtual_slave_fd):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
        self.virtual_master = None
        self.virtual_slave_fd = None
        self.virtual_path = ""

    def _virtual_slave_loop(self):
        buffer = bytearray()
        while self.virtual_running:
            try:
                ready, _, _ = select.select([self.virtual_master], [], [], 0.1)
            except (OSError, ValueError, TypeError):
                break
            if not ready:
                continue
            try:
                data = os.read(self.virtual_master, 512)
            except OSError:
                break
            if not data:
                continue

            buffer.extend(data)
            while True:
                frame = self._pop_request_frame(buffer)
                if frame is None:
                    break
                try:
                    self._handle_virtual_request(frame)
                except Exception as exc:
                    self.log_queue.put(f"【虚拟从站】处理请求失败: {exc}\n")

    @staticmethod
    def _pop_request_frame(buffer):
        """按主站请求格式切帧（0x03/0x06 固定 8 字节，0x10 由字节数决定）。"""
        while len(buffer) >= 8:
            func = buffer[1]
            if func in (0x03, 0x04, 0x06):
                size = 8
            elif func == 0x10:
                if len(buffer) < 7:
                    return None
                size = 9 + buffer[6]
            else:
                del buffer[0]
                continue

            if len(buffer) < size:
                return None
            frame = bytes(buffer[:size])
            if crc16(frame[:-2]) != (frame[-2] | (frame[-1] << 8)):
                del buffer[0]
                continue
            del buffer[:size]
            return frame
        return None

    def _handle_virtual_request(self, frame):
        if frame[0] != SLAVE_ID:
            return
        func = frame[1]

        if func == 0x06:
            address = (frame[2] << 8) | frame[3]
            value = (frame[4] << 8) | frame[5]
            self.virtual_regs[address] = value
            self._virtual_simulate(address, value)
            self.log_queue.put(
                f"【虚拟从站】写 0x{address:04X} = 0x{value:04X}{self._virtual_event(address)}\n"
            )
            self._virtual_reply(frame[:6])

        elif func == 0x04:
            start = (frame[2] << 8) | frame[3]
            count = (frame[4] << 8) | frame[5]
            if count == 0 or count > 125:
                self._virtual_reply(bytes((SLAVE_ID, 0x84, 0x03)))
                return
            payload = b"".join(
                int(self.virtual_inputs.get(start + i, 0)).to_bytes(2, "big")
                for i in range(count)
            )
            self._virtual_reply(bytes((SLAVE_ID, 0x04, len(payload))) + payload)
            if not self.polling:  # 闭环轮询时不刷屏
                self.log_queue.put(f"【虚拟从站】读输入寄存器 0x{start:04X} x{count}\n")

        elif func == 0x03:
            start = (frame[2] << 8) | frame[3]
            count = (frame[4] << 8) | frame[5]
            if count == 0 or count > 125:
                self._virtual_reply(bytes((SLAVE_ID, 0x83, 0x03)))
                return
            payload = b"".join(
                int(self.virtual_regs.get(start + i, 0)).to_bytes(2, "big") for i in range(count)
            )
            self._virtual_reply(bytes((SLAVE_ID, 0x03, len(payload))) + payload)
            self.log_queue.put(f"【虚拟从站】读 0x{start:04X} x{count}\n")

        elif func == 0x10:
            start = (frame[2] << 8) | frame[3]
            count = (frame[4] << 8) | frame[5]
            for i in range(count):
                self.virtual_regs[start + i] = (frame[7 + 2 * i] << 8) | frame[8 + 2 * i]
            self.log_queue.put(f"【虚拟从站】批量写 0x{start:04X} x{count}\n")
            self._virtual_reply(frame[:6])

        else:
            self._virtual_reply(bytes((SLAVE_ID, func | 0x80, 0x01)))

    def _virtual_simulate(self, address, value):
        """模拟主控：先进入 10 执行中，0.6 s 后置完成状态。"""
        if not value:
            return
        self.virtual_inputs[IN_STATUS] = 10
        timer = threading.Timer(0.6, self._virtual_finish, args=(address,))
        timer.daemon = True
        timer.start()

    def _virtual_finish(self, address):
        if address == REG_RESET:
            self.virtual_inputs[0x0003] = 0
            self.virtual_inputs[0x0004] = 0
        elif address == REG_FABRIC_READY:
            self.virtual_inputs[0x0003] = 1
        elif address == REG_POCKET_READY:
            self.virtual_inputs[0x0004] = 1
        self.virtual_inputs[IN_STATUS] = 1

    @staticmethod
    def _virtual_event(address):
        if address == REG_RESET:
            return "  >>> 主控回零"
        if address == REG_FABRIC_READY:
            return "  >>> 大身布料已放好，可触发视觉"
        if address == REG_POCKET_READY:
            return "  >>> 口袋布已放好，可触发视觉"
        return ""

    def _virtual_reply(self, pdu):
        if not self.virtual_running or self.virtual_master is None:
            return
        try:
            os.write(self.virtual_master, build_frame(pdu))
        except OSError:
            pass

    def on_close(self):
        try:
            if self.is_open:
                self.toggle_serial()
            if self.ros is not None:
                self.ros.close()
                self.ros = None
            self._stop_virtual_port()
        finally:
            self.root.destroy()

    def refresh_ports(self):
        current = self.port_combo.get().strip()
        devices = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = devices
        if current:
            self.port_combo.set(current)  # 保留手输的路径（虚拟串口不在列表中）
        elif devices:
            self.port_combo.current(0)

    def toggle_serial(self):
        if self.ros_mode_var.get():
            self.toggle_ros()
            return
        if not self.is_open:
            try:
                port = self.port_combo.get().strip()
                if not port:
                    messagebox.showwarning(
                        "提示", "请选择串口，或手动输入路径\n如 /dev/ttyUSB0 或 /tmp/align_virtual_slave"
                    )
                    return
                baud = int(self.baud_combo.get())
                databits = int(self.databits_combo.get())
                stopbits = float(self.stopbits_combo.get())
                parity_map = {"无":"N", "奇":"O", "偶":"E"}
                parity = parity_map[self.parity_combo.get()]

                self.ser = serial.Serial(
                    port=port,
                    baudrate=baud,
                    bytesize=databits,
                    stopbits=stopbits,
                    parity=parity,
                    timeout=0.1
                )
                self.is_open = True
                self.rx_buffer.clear()
                self.open_btn.config(text="关闭串口")
                self.status_var.set(f"已连接 | {port} @ {baud}bps")

                self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
                self.receive_thread.start()

            except Exception as e:
                messagebox.showerror("错误", f"打开串口失败：{str(e)}\n请检查串口权限和设备")
        else:
            self.auto_send_flag = False
            self.is_open = False
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.open_btn.config(text="打开串口")
            self.status_var.set("就绪 | 串口已关闭")

    def receive_loop(self):
        """只做串口读取，把数据丢进队列，控件更新交给主线程。"""
        while self.is_open and self.ser is not None and self.ser.is_open:
            try:
                n = self.ser.in_waiting
                if n > 0:
                    self.rx_queue.put(self.ser.read(n))
                else:
                    time.sleep(0.01)
            except Exception:
                break

    def show_receive(self, data):
        if self.polling:
            # 后台轮询状态时不刷屏，只更新快照
            self._feed_reply_parser(data)
            return
        if self.hex_recv_var.get():
            text = " ".join([f"{b:02X}" for b in data]) + " "
        else:
            try:
                text = data.decode("utf-8", errors="replace")
            except:
                text = data.hex()
        self.recv_text.insert("end", text)
        self.recv_text.see("end")

        if self.parse_reply_var.get():
            self._feed_reply_parser(data)

    def _feed_reply_parser(self, data):
        """把收到的字节流按应答帧切开，翻译成可读文字。"""
        self.rx_buffer.extend(data)
        while len(self.rx_buffer) >= 5:
            func = self.rx_buffer[1]
            if func & 0x80:
                size = 5
            elif func in (0x03, 0x04):
                size = 5 + self.rx_buffer[2]
            elif func in (0x06, 0x10):
                size = 8
            else:
                del self.rx_buffer[0]
                continue

            if len(self.rx_buffer) < size:
                return

            frame = bytes(self.rx_buffer[:size])
            if crc16(frame[:-2]) != (frame[-2] | (frame[-1] << 8)):
                del self.rx_buffer[0]
                continue
            del self.rx_buffer[:size]
            self._show_parsed(frame)

    def _show_parsed(self, frame):
        func = frame[1]
        if func & 0x80:
            text = f"【应答←】从站异常 0x{frame[2]:02X}"
        elif func == 0x06:
            address = (frame[2] << 8) | frame[3]
            value = (frame[4] << 8) | frame[5]
            text = f"【应答←】写成功 0x{address:04X} = 0x{value:04X}"
        elif func in (0x03, 0x04):
            count = frame[2] // 2
            values = [
                (frame[3 + 2 * i] << 8) | frame[4 + 2 * i] for i in range(count)
            ]
            if func == 0x04:
                self.status_snapshot = values
                if self.polling:
                    return
            text = "【应答←】读回 " + " ".join(f"0x{v:04X}" for v in values)
            if func == 0x04 and values:
                text += f"  状态={values[0]}({STATUS_TEXT.get(values[0], '未知')})"
        else:
            return
        self.recv_text.insert("end", text + "\n")
        self.recv_text.see("end")

    def send_data(self):
        if self.ros is not None:
            messagebox.showinfo("提示", "ROS 模式下请用命令控制台；发送区原始数据仅支持串口直连")
            return
        if not self.is_open or not self.ser.is_open:
            messagebox.showwarning("提示", "请先打开串口")
            return

        send_str = self.send_text.get("1.0", "end-1c")
        if not send_str:
            return

        try:
            if self.hex_send_var.get():
                hex_str = send_str.replace(" ", "").replace("\n", "")
                data = bytes.fromhex(hex_str)
            else:
                data = send_str.encode("utf-8")
            with self.serial_write_lock:
                self.ser.write(data)
        except Exception as e:
            messagebox.showerror("发送错误", str(e))

    def toggle_auto_send(self):
        if self.auto_send_var.get() and self.ros is not None:
            messagebox.showinfo("提示", "ROS 模式下请用命令控制台；定时发送仅支持串口直连")
            self.auto_send_var.set(False)
            return
        if self.auto_send_var.get():
            if not self.is_open:
                messagebox.showwarning("提示", "请先打开串口")
                self.auto_send_var.set(False)
                return
            try:
                interval = int(self.auto_interval.get()) / 1000
            except:
                messagebox.showerror("错误", "定时时间请输入数字")
                self.auto_send_var.set(False)
                return
            self.auto_send_flag = True
            self._auto_send_tick(interval)
        else:
            self.auto_send_flag = False

    def _auto_send_tick(self, interval):
        """定时发送也用主线程 after 驱动，避免跨线程操作控件。"""
        if not self.auto_send_flag or not self.is_open:
            return
        self.send_data()
        self.root.after(max(10, int(interval * 1000)), lambda: self._auto_send_tick(interval))

    def clear_recv(self):
        self.recv_text.delete("1.0", "end")

    def clear_send(self):
        self.send_text.delete("1.0", "end")

if __name__ == "__main__":
    root = tk.Tk()
    app = SerialAssistant(root)
    root.mainloop()
