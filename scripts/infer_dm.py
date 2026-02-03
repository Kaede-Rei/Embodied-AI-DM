#!/usr/bin/env python

"""
LeRobot 纯推理脚本 (Infinite Inference Loop)

功能：
1. 加载指定路径的模型。
2. 创建环境（基于模型配置）。
3. 无限循环运行推理 (Observation -> Policy -> Action -> Env)，直到 Ctrl+C。
4. 实时 Render 画面。

默认路径：
- 模型: ./outputs/leaf_v0_model/checkpoints/last/pretrained_model
- 数据集根目录: ./dataset/
"""

import json
import logging
import time
import signal
import sys
from pathlib import Path
from contextlib import nullcontext

import gymnasium as gym
import numpy as np
import torch
from termcolor import colored

# LeRobot 内部组件引用
from lerobot.envs.factory import make_env, make_env_pre_post_processors
from lerobot.envs.utils import preprocess_observation
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.utils.utils import get_safe_torch_device, init_logging, set_seed
from lerobot.utils.constants import ACTION

# ================= 配置区域 =================
# 你要求的模型路径
PRETRAINED_MODEL_PATH = Path("./outputs/leaf_v0_model/checkpoints/last/pretrained_model")
# 你要求的数据集路径 (如果环境或策略需要读取本地 dataset info)
DATASET_ROOT = Path("./dataset/")
# 设备 (自动检测 cuda/mps/cpu)
DEVICE = get_safe_torch_device("cuda", log=True)
# 是否使用混合精度 (通常设为 False 即可，除非模型极大)
USE_AMP = False
# 渲染频率限制 (FPS)，设为 None 则全速运行
MAX_FPS = 30
# ===========================================

def signal_handler(sig, frame):
    print(colored("\n[INFO] 检测到 Ctrl+C，正在停止推理...", "yellow"))
    sys.exit(0)

def main():
    init_logging()
    signal.signal(signal.SIGINT, signal_handler)

    if not PRETRAINED_MODEL_PATH.exists():
        print(colored(f"[Error] 模型路径不存在: {PRETRAINED_MODEL_PATH}", "red"))
        return

    # 1. 加载 Config
    # LeRobot 模型文件夹下通常有 config.json，我们需要它来构建正确的环境和策略
    config_path = PRETRAINED_MODEL_PATH / "config.json"
    with open(config_path, "r") as f:
        cfg_dict = json.load(f)
    
    # 简单的 Namespace 对象模拟 Hydra 配置，方便后续函数调用
    from argparse import Namespace
    
    # 提取关键配置
    # 注意：这里假设 config.json 结构符合 LeRobot 标准
    # 如果是旧版本模型，可能需要根据实际 json 结构微调
    policy_cfg = Namespace(**cfg_dict["policy"])
    env_cfg = Namespace(**cfg_dict["env"])
    
    # 修正 pretrained_path 为本地绝对路径/相对路径
    policy_cfg.pretrained_path = str(PRETRAINED_MODEL_PATH)
    
    # 设置随机种子
    set_seed(1000)

    print(colored(f"加载模型: {PRETRAINED_MODEL_PATH}", "green"))
    print(colored(f"使用设备: {DEVICE}", "green"))

    # 2. 创建环境
    # 将 dataset root 注入环境变量或配置中，以防环境需要读取 dataset info
    # 注意：大多数 LeRobot 推理不需要 raw dataset，只需要 config 里的 stats。
    # 但如果环境是基于文件的（如 aloha_sim 某些版本），可能需要。
    logging.info("Making environment...")
    env = make_env(
        env_cfg,
        n_envs=1, # 推理时我们通常只看一个环境
        dataset_root=str(DATASET_ROOT) # 传递数据集路径
    )

    # 3. 创建策略 (Policy)
    logging.info("Making policy...")
    policy = make_policy(
        cfg=policy_cfg,
        env_cfg=env_cfg,
        dataset_root=str(DATASET_ROOT)
    )
    policy.eval()
    policy.to(DEVICE)

    # 4. 创建处理器 (Pre/Post Processors)
    # 这一步至关重要，处理图像归一化、Action 反归一化等
    preprocessor_overrides = {
        "device_processor": {"device": str(DEVICE)},
    }
    
    # 通用处理器
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy_cfg,
        pretrained_path=str(PRETRAINED_MODEL_PATH),
        preprocessor_overrides=preprocessor_overrides,
    )
    
    # 环境特定处理器 (比如 Libero 需要特定处理)
    env_preprocessor, env_postprocessor = make_env_pre_post_processors(
        env_cfg=env_cfg, 
        policy_cfg=policy_cfg
    )

    # ================= 推理主循环 =================
    print(colored("\n开始无限推理循环... (按 Ctrl+C 停止)", "cyan", attrs=["bold"]))
    
    observation, info = env.reset(seed=None)
    
    try:
        while True:
            loop_start_time = time.time()

            # A. 预处理 Observation
            # 1. 转换为 Tensor 并调整维度 (numpy -> torch)
            observation = preprocess_observation(observation)
            # 2. 环境特定的预处理
            observation = env_preprocessor(observation)
            # 3. 策略通用的预处理 (如归一化)
            observation = preprocessor(observation)

            # B. 策略推理
            with torch.inference_mode(), torch.autocast(device_type=DEVICE.type) if USE_AMP else nullcontext():
                action = policy.select_action(observation)

            # C. 后处理 Action
            # 1. 策略通用的后处理 (如反归一化)
            action = postprocessor(action)
            # 2. 包装成字典以便通过 env_postprocessor
            action_transition = {ACTION: action}
            action_transition = env_postprocessor(action_transition)
            action = action_transition[ACTION]

            # D. 执行 Action
            # 转换为 Numpy (Batch size 1 -> Squeeze)
            action_numpy = action.to("cpu").numpy()
            
            # Env step
            observation, reward, terminated, truncated, info = env.step(action_numpy)
            
            # E. 渲染画面
            # vector env 需要调用 envs[0] 或者直接 render (取决于 gymnasium 版本)
            # LeRobot 的 make_env 返回的是 SyncVectorEnv
            if hasattr(env, "envs"):
                env.envs[0].render()
            else:
                env.render()

            # F. 自动重置逻辑
            # VectorEnv 通常会自动 reset，但为了逻辑清晰，我们检查 done
            # 注意: gym.vector.VectorEnv 会在 done 时自动 reset 并将 info["final_observation"] 返回
            # 所以这里其实不需要手动 reset，除非你想在 done 时停顿
            done = terminated | truncated
            if np.any(done):
                 print(colored("Episode Finished. Auto-resetting...", "blue"))
                 # 在 VectorEnv 中，Reset 是自动发生的，observation已经是新的了
                 pass

            # G. 控制帧率 (可选，方便肉眼观察)
            if MAX_FPS:
                dt = time.time() - loop_start_time
                target_dt = 1.0 / MAX_FPS
                if dt < target_dt:
                    time.sleep(target_dt - dt)

    except KeyboardInterrupt:
        pass
    finally:
        env.close()
        print("环境已关闭。")

if __name__ == "__main__":
    main()