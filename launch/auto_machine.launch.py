#!/usr/bin/env python3
"""一键启动：auto_machine_node 节点 + /rosout 日志 bag 录制。

用法：
    ros2 launch auto_machine auto_machine.launch.py

说明：
  - 节点串口参数已固化（udev 规则生成的固定软链接名，换口/重启不变）：
      mag_serial_ports[0] -> /dev/ttyRS4850 （0 号机）
      mag_serial_ports[1] -> /dev/ttyRS4851 （1 号机）
  - 每次启动自动用 ros2 bag record 录制 /rosout（节点全部日志），
    保存到 ~/bags/rosout_<启动时间戳>/，Ctrl+C 退出时自动落盘。
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# bag 保存根目录：~/bags/<时间戳子目录>
# 时间戳目录可避免 "bag 目录已存在" 报错，且天然按次归档历史日志
BAG_ROOT = os.path.expanduser("~/bags")
BAG_DIR = os.path.join(
    BAG_ROOT, "rosout_" + datetime.now().strftime("%Y%m%d_%H%M%S")
)


def generate_launch_description():
    # 确保 bag 根目录存在（ros2 bag record 不会自动创建父目录）
    os.makedirs(BAG_ROOT, exist_ok=True)

    return LaunchDescription([
        Node(
            package="auto_machine",
            executable="auto_machine_node",
            name="auto_machine_node",
            output="both",  # 终端实时显示 + 发到 /rosout 供 bag 录制
            parameters=[{
                # 双串口模式：索引顺序对应 param_data[0]
                "mag_serial_ports": [
                    "/dev/ttyRS4850",
                    "/dev/ttyRS4851",
                ],
            }],
        ),
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", BAG_DIR, "--topics", "/rosout"],
            output="screen",
        ),
    ])
