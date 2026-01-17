# dm_arm 遥操作（Teleoperation）脚本
#
# 前提：dm_arm 已通过插件机制注册（--robot.type=dm_follower 等可用）
# 使用前确保：
# 1. 已激活包含 LeRobot 和 lerobot_robot_multi_robots 包的 Python 环境
# 2. Leader 臂和 Follower 臂已正确连接并上电
# 3. Leader 臂端口默认 /dev/ttyUSB0，Follower 臂端口默认 /dev/ttyACM0（可通过参数覆盖）
# 4. 若使用 --display_data，请确保摄像头已连接并索引正确（可通过 get_uvc_cam_idx 脚本检查）
#
# 使用方法示例：
# python teleop_dk1.py                     # 无界面纯遥操作（仅关节映射，无摄像头）
# python teleop_dk1.py --display_data      # 启动 rerun.io GUI + 自动显示摄像头
# python teleop_dk1.py --follower_port /dev/ttyACM1 --leader_port /dev/ttyUSB1 --freq 100
#
# 支持的参数：
# --follower_port <port> Follower 臂串口路径（默认 /dev/ttyACM0）
# --leader_port <port> Leader 臂串口路径（默认 /dev/ttyUSB0）
# --freq <float> 无界面模式下控制循环频率（Hz，默认 200.0）
# --display_data 若指定，则启动 lerobot-teleoperate GUI 模式（自动启用摄像头显示，不运行脚本主循环）

# 固定摄像头配置（仅在 --display_data 模式下生效）
# 根据实际硬件修改以下配置（例如摄像头索引、分辨率、FPS 等）
CAMERAS_CONFIG = {
    "context": {
        "type": "opencv",
        "index_or_path": 2,
        "width": 1280,
        "height": 720,
        "fps": 30,
    },
}

import argparse
import time
import subprocess
import json
from lerobot_robot_multi_robots.dm_arm import DMFollower, DMLeader
from lerobot_robot_multi_robots.dm_arm import DMFollowerConfig, DMLeaderConfig

CAMERAS_JSON = json.dumps(CAMERAS_CONFIG)


def parse_args():
    ap = argparse.ArgumentParser(description="DK1 teleoperation")
    ap.add_argument("--follower_port", default="/dev/ttyACM0")
    ap.add_argument("--leader_port", default="/dev/ttyUSB0")
    ap.add_argument("--freq", type=float, default=200.0)
    ap.add_argument(
        "--display_data",
        action="store_true",
        help="若开启，则启动 lerobot-teleoperate GUI 并自动显示摄像头（rerun.io）",
    )
    return ap.parse_args()


def main():
    args = parse_args()

    if args.display_data:
        cmd = [
            "lerobot-teleoperate",
            "--robot.type=dm_follower",
            f"--robot.port={args.follower_port}",
            "--robot.joint_velocity_scaling=1.0",
            "--teleop.type=dm_leader",
            f"--teleop.port={args.leader_port}",
            "--display_data=true",
            f"--robot.cameras={CAMERAS_JSON}",  # 固定启用摄像头配置
        ]

        try:
            subprocess.run(cmd, check=True)
        except KeyboardInterrupt:
            print("\nStopping teleop GUI...")
        finally:
            # 确保在退出时安全断开连接
            leader = DMLeader(DMLeaderConfig(port=args.leader_port))
            leader.connect()
            follower = DMFollower(
                DMFollowerConfig(
                    port=args.follower_port, disable_torque_on_disconnect=True
                )
            )
            follower.connect()
            leader.disconnect()
            follower.disconnect()
        return

    # 无界面纯遥操作模式（仅关节动作映射，无摄像头、无 GUI）
    leader = DMLeader(DMLeaderConfig(port=args.leader_port))
    leader.connect()
    follower = DMFollower(
        DMFollowerConfig(
            port=args.follower_port,
            joint_velocity_scaling=1.0,
            disable_torque_on_disconnect=True,
        )
    )
    follower.connect()

    try:
        period = 1.0 / args.freq
        print("Starting pure teleoperation (no GUI, no cameras)...")
        while True:
            action = leader.get_action()
            follower.send_action(action)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping pure teleoperation...")
    finally:
        leader.disconnect()
        follower.disconnect()


if __name__ == "__main__":
    main()
