#!/usr/bin/env python
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
            f"--robot.joint_velocity_scaling=0.2",
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
