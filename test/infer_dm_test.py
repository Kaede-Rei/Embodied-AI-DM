#!/usr/bin/env python
"""
测试模型重载耗时 - 模拟真实场景：
1. 机械臂/相机连接并保持
2. 模型推理运行一段时间
3. 卸载模型
4. 重新加载模型
5. 继续推理
"""

import argparse
import time
import gc
from pathlib import Path

import numpy as np
import torch

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot_robot_multi_robots.dm_arm import DMFollower
from lerobot_robot_multi_robots.config_dm_arm import DMFollowerConfig

# 相机配置
CAMERAS_CONFIG = {
    "end": OpenCVCameraConfig(
        index_or_path="/dev/com-1.2-video",
        width=640,
        height=480,
        fps=30,
    ),
    "eye": OpenCVCameraConfig(
        index_or_path=2,
        width=1280,
        height=720,
        fps=30,
    ),
}

DEFAULT_MODEL_PATH = "./outputs/leaf_v0_model/checkpoints/last/pretrained_model"


def preprocess_observation(obs_dict: dict, device: torch.device) -> dict:
    """预处理观测"""
    processed = {}
    state_keys = [
        "joint_1.pos", "joint_2.pos", "joint_3.pos",
        "joint_4.pos", "joint_5.pos", "joint_6.pos", "gripper.pos",
    ]
    state = np.array([obs_dict[key] for key in state_keys], dtype=np.float32)
    processed["observation.state"] = torch.from_numpy(state).unsqueeze(0).to(device)
    
    for cam_name in ["eye", "end"]:
        if cam_name in obs_dict:
            img = obs_dict[cam_name]
            img = np.transpose(img, (2, 0, 1)).astype(np.float32) / 255.0
            processed[f"observation.images.{cam_name}"] = (
                torch.from_numpy(img).unsqueeze(0).to(device)
            )
    return processed


def postprocess_action(action: torch.Tensor) -> dict:
    """后处理动作"""
    action_np = action.squeeze(0).cpu().numpy()
    return {
        "joint_1.pos": float(action_np[0]),
        "joint_2.pos": float(action_np[1]),
        "joint_3.pos": float(action_np[2]),
        "joint_4.pos": float(action_np[3]),
        "joint_5.pos": float(action_np[4]),
        "joint_6.pos": float(action_np[5]),
        "gripper.pos": float(action_np[6]),
    }


def run_inference(policy, robot, device, duration_sec: float, freq: float = 30.0):
    """运行推理一段时间"""
    print(f"   开始推理 {duration_sec}s (频率 {freq} Hz)...")
    period = 1.0 / freq
    start_time = time.perf_counter()
    steps = 0
    
    while time.perf_counter() - start_time < duration_sec:
        loop_start = time.perf_counter()
        
        raw_obs = robot.get_observation()
        obs = preprocess_observation(raw_obs, device)
        
        with torch.inference_mode():
            action = policy.select_action(obs)
        
        action_dict = postprocess_action(action)
        robot.send_action(action_dict)
        
        steps += 1
        
        # 控制频率
        elapsed = time.perf_counter() - loop_start
        if elapsed < period:
            time.sleep(period - elapsed)
    
    actual_time = time.perf_counter() - start_time
    actual_freq = steps / actual_time
    print(f"   ✓ 推理完成: {steps} 步, 实际频率 {actual_freq:.1f} Hz")


def load_model(model_path: str, device: torch.device):
    """加载模型"""
    t0 = time.perf_counter()
    
    load_start = time.perf_counter()
    policy = ACTPolicy.from_pretrained(model_path)
    load_time = time.perf_counter() - load_start
    
    to_start = time.perf_counter()
    policy.to(device)
    to_time = time.perf_counter() - to_start
    
    policy.eval()
    
    total_time = time.perf_counter() - t0
    
    return policy, {
        "from_pretrained": load_time,
        "to_device": to_time,
        "total": total_time,
    }


def unload_model(policy):
    """卸载模型"""
    t0 = time.perf_counter()
    
    policy.cpu()
    del policy
    gc.collect()
    
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.synchronize()
    
    unload_time = time.perf_counter() - t0
    return unload_time


def main():
    parser = argparse.ArgumentParser(description="测试模型重载耗时（真实场景）")
    parser.add_argument(
        "--model_path", type=str, default=DEFAULT_MODEL_PATH, help="模型路径"
    )
    parser.add_argument(
        "--port", type=str, default="/dev/ttyACM0", help="机械臂串口"
    )
    parser.add_argument(
        "--device", type=str, default="cuda", help="推理设备"
    )
    parser.add_argument(
        "--iterations", type=int, default=3, help="重载测试次数"
    )
    parser.add_argument(
        "--run_duration", type=float, default=5.0, help="每次推理运行时长（秒）"
    )
    parser.add_argument(
        "--freq", type=float, default=30.0, help="推理频率 Hz"
    )
    args = parser.parse_args()

    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    
    print("=" * 70)
    print("模型重载延迟测试 - 真实场景模拟")
    print("=" * 70)
    print(f"设备: {device}")
    print(f"模型: {args.model_path}")
    print(f"测试次数: {args.iterations}")
    print(f"每轮推理时长: {args.run_duration}s @ {args.freq}Hz")
    print("=" * 70)

    # ===================== 1. 初始化硬件（一次性，保持连接）=====================
    print("\n[步骤 1] 初始化机械臂和相机...")
    init_start = time.perf_counter()
    
    robot_config = DMFollowerConfig(
        port=args.port,
        cameras=CAMERAS_CONFIG,
        joint_velocity_scaling=0.1,
        disable_torque_on_disconnect=True,
    )
    robot = DMFollower(robot_config)
    robot.connect()
    
    init_time = time.perf_counter() - init_start
    print(f"✓ 硬件初始化完成: {init_time:.2f}s")

    # ===================== 2. 重载循环测试 =====================
    results = []
    
    for i in range(args.iterations):
        print(f"\n{'='*70}")
        print(f"第 {i+1}/{args.iterations} 轮测试")
        print(f"{'='*70}")
        
        # (1) 加载模型
        print(f"\n[A] 加载模型...")
        policy, load_stats = load_model(args.model_path, device)
        print(f"    - from_pretrained: {load_stats['from_pretrained']:.3f}s")
        print(f"    - to(device):      {load_stats['to_device']:.3f}s")
        print(f"    - 总计:            {load_stats['total']:.3f}s")
        
        # (2) 运行推理
        print(f"\n[B] 运行推理...")
        run_inference(policy, robot, device, args.run_duration, args.freq)
        
        # (3) 卸载模型
        print(f"\n[C] 卸载模型...")
        unload_time = unload_model(policy)
        print(f"    - 卸载耗时: {unload_time:.3f}s")
        
        load_stats["unload"] = unload_time
        results.append(load_stats)
        
        print(f"\n>>> 本轮重载周期: {load_stats['total'] + unload_time:.3f}s")
        
        # 等待一下
        if i < args.iterations - 1:
            print("\n等待 1 秒后开始下一轮...")
            time.sleep(1.0)
    
    # ===================== 3. 统计结果 =====================
    print(f"\n{'='*70}")
    print("测试结果汇总")
    print(f"{'='*70}")
    
    load_times = [r["total"] for r in results]
    unload_times = [r["unload"] for r in results]
    reload_cycles = [r["total"] + r["unload"] for r in results]
    
    print(f"\n【模型加载】")
    print(f"  平均:   {np.mean(load_times):.3f}s")
    print(f"  最快:   {np.min(load_times):.3f}s")
    print(f"  最慢:   {np.max(load_times):.3f}s")
    print(f"  标准差: {np.std(load_times):.3f}s")
    
    print(f"\n【模型卸载】")
    print(f"  平均:   {np.mean(unload_times):.3f}s")
    print(f"  最快:   {np.min(unload_times):.3f}s")
    print(f"  最慢:   {np.max(unload_times):.3f}s")
    
    print(f"\n{'='*70}")
    print(f"★ 完整重载周期 (卸载 + 重新加载):")
    print(f"    平均: {np.mean(reload_cycles):.3f}s")
    print(f"    最快: {np.min(reload_cycles):.3f}s")
    print(f"    最慢: {np.max(reload_cycles):.3f}s")
    print(f"{'='*70}")
    
    print(f"\n💡 ROS 节点设计参考:")
    print(f"   - 用户请求推理时，预期等待约 {np.mean(reload_cycles):.1f}s")
    print(f"   - 如果需要低延迟响应，建议保持模型常驻内存")
    print(f"{'='*70}")

    # 断开连接
    print("\n断开机械臂连接...")
    robot.disconnect()
    print("测试完成！")


if __name__ == "__main__":
    main()