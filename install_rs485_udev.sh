#!/usr/bin/env bash
# 一键安装 udev 规则，把 USB 转 485 固定为 /dev/ttyRS485（单机）以及
# /dev/ttyRS485_0、/dev/ttyRS485_1 ...（多机，按 USB 物理端口区分）。
#
# 装一次永久生效：之后无论插哪个 USB 口、重启多少次，系统都会自动创建
# 这些固定软链接，不用再手动跑任何东西。
#
# 多机场景说明：
#   一个机器人同时挂多台同类自动机（如两台 MAG06 钉扣机）时，每台走一条
#   独立 RS485 串口。因为同款 USB-485 芯片 VID:PID 相同，必须按 USB 物理端口
#   （KERNELS 值）区分，分别生成 ttyRS485_0 / ttyRS485_1。
#   安装前请先按 udev/99-usb-rs485.rules 里的注释，用
#     udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
#   查到每台设备所在的物理端口，再把规则里的 KERNELS 值改成实际路径。
#
# 用法（在仓库根目录）：
#     sudo bash install_rs485_udev.sh
#
# 等价于手动执行：
#     sudo cp udev/99-usb-rs485.rules /etc/udev/rules.d/
#     sudo udevadm control --reload-rules
#     sudo udevadm trigger

set -e

# 脚本所在目录（install_rs485_udev.sh 与 udev/ 同级）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULE_SRC="$SCRIPT_DIR/udev/99-usb-rs485.rules"
RULE_DST="/etc/udev/rules.d/99-usb-rs485.rules"

if [ "$(id -u)" -ne 0 ]; then
    echo "需要 root 权限，请用：sudo bash install_rs485_udev.sh"
    exit 1
fi

if [ ! -f "$RULE_SRC" ]; then
    echo "找不到规则文件: $RULE_SRC"
    exit 1
fi

cp "$RULE_SRC" "$RULE_DST"
udevadm control --reload-rules
udevadm trigger

echo "udev 规则已安装：$RULE_DST"
echo "之后 USB-485 接入时，系统会自动创建："
echo "  /dev/ttyRS485   （单机通用名）"
echo "  /dev/ttyRS485_0 / /dev/ttyRS485_1 ...（多机，按物理端口）"
echo "注意：多机软链接的 KERNELS 值需按现场 USB 物理端口改好规则后再安装。"
if [ -L /dev/ttyRS485 ]; then
    echo "当前单机软链接已生效: $(ls -l /dev/ttyRS485)"
else
    echo "当前设备未接入；接入后会自动出现 /dev/ttyRS485。"
fi
