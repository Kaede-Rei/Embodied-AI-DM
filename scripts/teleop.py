# DK1 遥操作（Teleoperation）脚本
#
# 前提：DK1 已通过插件机制注册（--robot.type=dk1_follower 等可用）
# 使用前确保：
# 1. 已激活包含 LeRobot 和 trlc_dk1 包的 Python 环境
# 2. Leader 臂和 Follower 臂已正确连接并上电
# 3. Leader 臂端口默认 /dev/ttyUSB0，Follower 臂端口默认 /dev/ttyACM0（可通过参数覆盖）
#
# 使用方法示例：
# python teleop_dk1.py
# python teleop_dk1.py --display_data
# python teleop_dk1.py --follower_port /dev/ttyACM1 --leader_port /dev/ttyUSB1 --freq 100
#
# 支持的参数：
# --follower_port <port>                Follower 臂串口路径（默认 /dev/ttyACM0）
# --leader_port <port>                  Leader 臂串口路径（默认 /dev/ttyUSB0）
# --freq <float>                        无界面模式下控制循环频率（Hz，默认 200.0）
# --display_data                        若指定，则启动 lerobot-teleoperate GUI 模式（不运行脚本主循环）

import argparse, time, subprocess
from trlc_dk1.follower import DK1Follower, DK1FollowerConfig
from trlc_dk1.leader import DK1Leader, DK1LeaderConfig

def parse_args():
    ap = argparse.ArgumentParser(description="DK1 teleoperation")
    ap.add_argument("--follower_port", default="/dev/ttyACM0")
    ap.add_argument("--leader_port", default="/dev/ttyUSB0")
    ap.add_argument("--freq", type=float, default=200.0)
    ap.add_argument("--display_data", action="store_true",
                    help="若开启，则调用 lerobot-teleoperate GUI（不运行本脚本循环）")
    return ap.parse_args()

def main():
    args = parse_args()

    if args.display_data:
        cmd = [
            "lerobot-teleoperate",
            f"--robot.type=dk1_follower",
            f"--robot.port={args.follower_port}",
            f"--robot.joint_velocity_scaling=1.0",
            f"--teleop.type=dk1_leader",
            f"--teleop.port={args.leader_port}",
            "--display_data=true",
        ]
        print("Launching GUI:", " ".join(cmd))
        try:
            subprocess.run(cmd, check=True)
        except KeyboardInterrupt:
            print("\nStopping teleop...")
        finally:
            leader = DK1Leader(DK1LeaderConfig(port=args.leader_port))
            leader.connect()
            follower = DK1Follower(DK1FollowerConfig(
                port=args.follower_port,
                disable_torque_on_disconnect=True
            ))
            follower.connect()
            leader.disconnect()
            follower.disconnect()
        return

    # 无界面脚本联动
    leader = DK1Leader(DK1LeaderConfig(port=args.leader_port))  
    leader.connect()

    follower = DK1Follower(DK1FollowerConfig(
        port=args.follower_port,
        joint_velocity_scaling=1.0,
        disable_torque_on_disconnect=True
    ))
    follower.connect()

    try:
        period = 1.0 / args.freq
        while True:
            action = leader.get_action()
            follower.send_action(action)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping teleop...")
    finally:
        leader.disconnect()
        follower.disconnect()

if __name__ == "__main__":
    main()
