#!/usr/bin/env python
"""
DM 机械臂推理脚本 - 使用训练好的模型进行持续推理
从 ./outputs/leaf_v0_model/checkpoints/last/pretrained_model 加载模型
"""

import argparse
import time
from pathlib import Path
from contextlib import nullcontext

import numpy as np
import torch

# LeRobot imports
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.processor import PolicyProcessorPipeline
from lerobot.cameras.opencv import OpenCVCameraConfig

# Local imports
from lerobot_robot_multi_robots.dm_arm import DMFollower
from lerobot_robot_multi_robots.config_dm_arm import DMFollowerConfig

# ===================== 配置 =====================

# 相机配置 - 使用 OpenCVCameraConfig 对象
CAMERAS_CONFIG = {
    "end": OpenCVCameraConfig(
        index_or_path="/dev/com-1.2-video",
        width=640,
        height=480,
        fps=30,
    ),
    "eye": OpenCVCameraConfig(
        index_or_path=4,
        width=1280,
        height=720,
        fps=30,
    ),
}

# 默认模型路径
DEFAULT_MODEL_PATH = "./outputs/leaf_v0_model/checkpoints/last/pretrained_model"


def parse_args():
    parser = argparse.ArgumentParser(description="DM 机械臂推理脚本")
    parser.add_argument(
        "--model_path",
        type=str,
        default=DEFAULT_MODEL_PATH,
        help="预训练模型路径",
    )
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyACM0",
        help="机械臂串口",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda",
        help="推理设备 (cuda/cpu)",
    )
    parser.add_argument(
        "--freq",
        type=float,
        default=30.0,
        help="推理频率 Hz",
    )
    parser.add_argument(
        "--use_amp",
        action="store_true",
        help="使用混合精度推理",
    )
    parser.add_argument(
        "--joint_velocity_scaling",
        type=float,
        default=0.2,
        help="关节速度缩放 (0-1)",
    )
    return parser.parse_args()


def preprocess_observation(obs_dict: dict, device: torch.device) -> dict:
    """
    将机械臂观测转换为模型输入格式
    
    Args:
        obs_dict: 来自机械臂的原始观测
        device: 目标设备
    
    Returns:
        处理后的观测字典
    """
    processed = {}
    
    # 处理关节状态 (7个关节: joint_1-6 + gripper)
    state_keys = [
        "joint_1.pos", "joint_2.pos", "joint_3.pos",
        "joint_4.pos", "joint_5.pos", "joint_6.pos",
        "gripper.pos"
    ]
    state = np.array([obs_dict[key] for key in state_keys], dtype=np.float32)
    # 添加 batch 维度: (7,) -> (1, 7)
    processed["observation.state"] = torch.from_numpy(state).unsqueeze(0).to(device)
    
    # 处理图像 (HWC uint8 -> CHW float32, normalized)
    for cam_name in ["eye", "end"]:
        if cam_name in obs_dict:
            img = obs_dict[cam_name]  # (H, W, C) uint8
            # 转换为 (C, H, W) float32
            img = np.transpose(img, (2, 0, 1)).astype(np.float32) / 255.0
            # 添加 batch 维度: (C, H, W) -> (1, C, H, W)
            processed[f"observation.images.{cam_name}"] = torch.from_numpy(img).unsqueeze(0).to(device)
    
    return processed


def postprocess_action(action: torch.Tensor) -> dict:
    """
    将模型输出转换为机械臂动作格式
    
    Args:
        action: 模型输出的动作张量 (batch, action_dim)
    
    Returns:
        机械臂动作字典
    """
    # 移除 batch 维度并转为 numpy
    action_np = action.squeeze(0).cpu().numpy()
    
    action_dict = {
        "joint_1.pos": float(action_np[0]),
        "joint_2.pos": float(action_np[1]),
        "joint_3.pos": float(action_np[2]),
        "joint_4.pos": float(action_np[3]),
        "joint_5.pos": float(action_np[4]),
        "joint_6.pos": float(action_np[5]),
        "gripper.pos": float(action_np[6]),
    }
    
    return action_dict


def main():
    args = parse_args()
    
    # 设置设备
    device = torch.device(args.device if torch.cuda.is_available() or args.device == "cpu" else "cpu")
    print(f"使用设备: {device}")
    
    # 设置高性能模式
    torch.backends.cudnn.benchmark = True
    torch.backends.cuda.matmul.allow_tf32 = True
    
    # ===================== 加载模型 =====================
    print(f"加载模型: {args.model_path}")
    model_path = Path(args.model_path)
    
    if not model_path.exists():
        raise FileNotFoundError(f"模型路径不存在: {model_path}")
    
    # 使用 ACTPolicy.from_pretrained 加载模型
    policy = ACTPolicy.from_pretrained(str(model_path))
    policy.to(device)
    policy.eval()
    print(f"模型加载成功: {type(policy).__name__}")
    print(f"模型配置: chunk_size={policy.config.chunk_size}, n_action_steps={policy.config.n_action_steps}")
    
    # ===================== 初始化机械臂 =====================
    print("初始化机械臂...")
    robot_config = DMFollowerConfig(
        port=args.port,
        cameras=CAMERAS_CONFIG,
        joint_velocity_scaling=args.joint_velocity_scaling,
        disable_torque_on_disconnect=True,
    )
    robot = DMFollower(robot_config)
    
    try:
        robot.connect()
        print("机械臂连接成功")
        
        # ===================== 推理循环 =====================
        period = 1.0 / args.freq
        print(f"开始推理循环，频率: {args.freq} Hz")
        print("按 Ctrl+C 停止...")
        
        step = 0
        amp_context = torch.autocast(device_type=device.type) if args.use_amp else nullcontext()
        
        while True:
            loop_start = time.perf_counter()
            
            # 1. 读取观测
            obs_start = time.perf_counter()
            raw_obs = robot.get_observation()
            obs_time = (time.perf_counter() - obs_start) * 1000
            
            # 2. 预处理观测
            proc_start = time.perf_counter()
            obs = preprocess_observation(raw_obs, device)
            proc_time = (time.perf_counter() - proc_start) * 1000
            
            # 3. 模型推理
            infer_start = time.perf_counter()
            with torch.inference_mode(), amp_context:
                action = policy.select_action(obs)
            infer_time = (time.perf_counter() - infer_start) * 1000
            
            # 4. 后处理动作
            action_dict = postprocess_action(action)
            
            # 5. 发送动作
            send_start = time.perf_counter()
            robot.send_action(action_dict)
            send_time = (time.perf_counter() - send_start) * 1000
            
            # 统计
            total_time = (time.perf_counter() - loop_start) * 1000
            
            if step % 30 == 0:  # 每秒打印一次
                print(
                    f"[Step {step:5d}] "
                    f"obs: {obs_time:5.1f}ms | "
                    f"proc: {proc_time:5.1f}ms | "
                    f"infer: {infer_time:5.1f}ms | "
                    f"send: {send_time:5.1f}ms | "
                    f"total: {total_time:5.1f}ms | "
                    f"gripper: {action_dict['gripper.pos']:.3f}"
                )
            
            # 控制循环频率
            elapsed = time.perf_counter() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)
            
            step += 1
            
    except KeyboardInterrupt:
        print("\n停止推理...")
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("断开机械臂连接...")
        try:
            robot.disconnect()
        except:
            pass
        print("完成")


if __name__ == "__main__":
    main()
