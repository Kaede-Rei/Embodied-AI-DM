"""
DK1 Follower 机械臂复位脚本
功能：
- 通过时间插值 + 单步限幅，缓慢平滑地将所有关节角归零
- 避免一步跳目标导致的猛拉或塌一下
"""

import argparse
import time
import sys
import numpy as np
from lerobot_robot_multi_robots.dm_arm import DMFollower
from lerobot_robot_multi_robots.config_dm_arm import DMFollowerConfig

def parse_args():
    parser = argparse.ArgumentParser(description="DK1 Follower 平滑复位脚本")
    parser.add_argument(
        "--follower_port",
        default="/dev/ttyACM0",
        help="Follower 臂串口（默认 /dev/ttyACM0）"
    )
    parser.add_argument(
        "--joint_velocity_scaling",
        type=float,
        default=0.2,
        help="关节运动速度缩放（0-1，默认 0.2）"
    )
    parser.add_argument(
        "--control_freq",
        type=float,
        default=200.0,
        help="控制频率 Hz（默认 200）"
    )
    parser.add_argument(
        "--smooth_time",
        type=float,
        default=2.0,
        help="期望归零时间（秒，默认 2s）"
    )
    parser.add_argument(
        "--max_step",
        type=float,
        default=0.05,
        help="单次关节最大步进（rad，默认 0.05 约 2.86 度）"
    )
    parser.add_argument(
        "--open_gripper",
        action="store_true",
        help="复位时打开夹爪"
    )
    parser.add_argument(
        "--close_gripper",
        action="store_true",
        help="复位时关闭夹爪"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="最大复位超时时间（秒，默认 30）"
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.02,
        help="位置容差（rad，默认 0.02 约 1.15 度）"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 60)
    print("DK1 Follower 机械臂平滑复位脚本")
    print("=" * 60)
    print(f"串口: {args.follower_port}")
    print(f"速度缩放: {args.joint_velocity_scaling}")
    print(f"控制频率: {args.control_freq} Hz")
    print(f"平滑时间: {args.smooth_time} s")
    print(f"单步限幅: {args.max_step:.4f} rad")
    print(f"位置容差: {args.tolerance:.4f} rad")

    print("\n正在连接机械臂...")
    try:
        follower = DMFollower(DMFollowerConfig(
            port=args.follower_port,
            joint_velocity_scaling=args.joint_velocity_scaling,
            disable_torque_on_disconnect=True
        ))
        follower.connect()
        print("机械臂已连接")
    except Exception as e:
        print(f"连接失败: {e}")
        sys.exit(1)

    try:
        current_obs = follower.get_observation()

        joints = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
        ]

        print("\n当前关节位置:")
        for j in joints:
            q = current_obs[f"{j}.pos"]
            print(f"  {j}: {q:+.3f} rad ({q * 180 / np.pi:+.1f} 度)")

        start_positions = {
            j: current_obs[f"{j}.pos"] for j in joints
        }
        target_positions = {j: 0.0 for j in joints}

        if args.open_gripper:
            gripper_target = 0.0
            print("\n夹爪目标：打开")
        elif args.close_gripper:
            gripper_target = 1.0
            print("\n夹爪目标：关闭")
        else:
            gripper_target = current_obs["gripper.pos"]
            print("\n夹爪目标：保持当前")

        print("\n开始平滑归零（Ctrl+C 可中断）")

        control_period = 1.0 / args.control_freq
        start_time = time.time()
        last_print = start_time

        while True:
            loop_t0 = time.time()
            elapsed = loop_t0 - start_time

            current_obs = follower.get_observation()

            alpha = min(elapsed / args.smooth_time, 1.0)

            action = {}
            max_error = 0.0
            all_reached = True

            for j in joints:
                q_start = start_positions[j]
                q_target = target_positions[j]

                q_ref = (1 - alpha) * q_start + alpha * q_target

                q_now = current_obs[f"{j}.pos"]
                dq = q_ref - q_now
                if abs(dq) > args.max_step:
                    q_cmd = q_now + np.sign(dq) * args.max_step
                else:
                    q_cmd = q_ref

                action[f"{j}.pos"] = q_cmd

                err = abs(q_now - q_target)
                max_error = max(max_error, err)
                if err > args.tolerance:
                    all_reached = False

            action["gripper.pos"] = gripper_target
            follower.send_action(action)

            if loop_t0 - last_print >= 1.0:
                print(f"  [{elapsed:5.1f}s] 最大误差: {max_error:.4f} rad")
                last_print = loop_t0

            if all_reached:
                print(f"\n归零完成，用时 {elapsed:.2f} 秒")
                break

            if elapsed > args.timeout:
                print("\n复位超时，提前结束")
                break

            sleep_t = control_period - (time.time() - loop_t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

        print("\n最终关节位置:")
        final_obs = follower.get_observation()
        for j in joints:
            q = final_obs[f"{j}.pos"]
            err = abs(q)
            status = "正常" if err <= args.tolerance else "超出容差"
            print(f"  {j}: {q:+.3f} rad ({status})")

        print("\n正在安全失能...")
        time.sleep(0.3)

    except KeyboardInterrupt:
        print("\n用户中断，正在安全失能...")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        follower.disconnect()
        print("电机已失能，连接已断开")
        print("\n复位流程结束")


if __name__ == "__main__":
    main()
