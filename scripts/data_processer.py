#!/usr/bin/env python3
"""
LeRobot parquet 数据处理脚本，支持跳变点检测、平滑、线性插值、可视化对比等功能

前提：将 <repo_id>/data/chunk-xxx/file-xxx.parquet 下载到本地 data/ 目录，最后会输出处理后的文件 file-xxx_new.parquet 放到 data/ 下

使用方法示例：
python scripts/data_processer.py
python scripts/data_processer.py --file file-001.parquet --loop 2
python scripts/data_processer.py --target action
python scripts/data_processer.py --jump_threshold 0.02 --smooth_window 31 --poly_order 3

支持的参数：
--file: 要处理的 parquet 文件名（位于 data/ 目录下）
--loop: 重复处理次数（用于观察过度平滑，建议最多两次）
--target: 处理字段（从臂 observation.state 或 主臂 action）
--jump_threshold: 跳变检测阈值（rad）
--smooth_window: Savitzky-Golay 窗口长度(奇数)，通常取控制频率的 0.5~2.0 倍 
--poly_order: Savitzky-Golay 多项式阶数，通常取 2 或 3
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter


# ============================================================
# 默认参数（可被命令行覆盖）
# ============================================================

DEFAULT_TARGET = "observation.state"
DEFAULT_JUMP_THRESHOLD = 0.1
DEFAULT_SMOOTH_WINDOW = 25
DEFAULT_POLY_ORDER = 3
DEFAULT_LOOP = 1


# ============================================================
# 单关节处理逻辑
# ============================================================

def process_joint_once(raw_data, jump_threshold, smooth_window, poly_order):
    """
    对单个关节的一维时间序列执行一次处理：
    - 跳变检测
    - 线性插值
    - Savitzky-Golay 平滑
    """
    diff = np.abs(np.diff(raw_data, prepend=raw_data[0]))
    mask_jump = diff > jump_threshold

    series = pd.Series(raw_data)
    series[mask_jump] = np.nan
    interp_data = series.interpolate(
        method="linear",
        limit_direction="both"
    ).values

    if len(interp_data) > smooth_window:
        interp_data = savgol_filter(
            interp_data,
            smooth_window,
            poly_order
        )

    return interp_data, mask_jump


def process_joint_loop(raw_data, loop, **kwargs):
    """
    对单关节数据执行多次处理（loop 次）
    """
    data = raw_data.copy()
    all_masks = []

    for _ in range(loop):
        data, mask = process_joint_once(data, **kwargs)
        all_masks.append(mask)

    # 仅返回第一次的跳变点用于可视化
    return data, all_masks[0]


# ============================================================
# Episode 级处理
# ============================================================

def plot_episode(df, ep_id, target, loop, args):
    """
    可视化单个 episode：
    左：原始数据
    右：loop 次处理后的结果
    """
    df_ep = df[df["episode_index"] == ep_id]
    if df_ep.empty:
        print(f"Episode {ep_id} 不存在")
        return

    data_matrix = np.array(df_ep[target].tolist())
    num_frames, num_joints = data_matrix.shape

    fig, axes = plt.subplots(
        num_joints, 2,
        figsize=(16, 2.5 * num_joints),
        sharex=True
    )
    plt.subplots_adjust(hspace=0.35)

    total_jumps = 0

    for j in range(num_joints):
        raw = data_matrix[:, j]
        processed, jumps = process_joint_loop(
            raw,
            loop=loop,
            jump_threshold=args.jump_threshold,
            smooth_window=args.smooth_window,
            poly_order=args.poly_order,
        )
        total_jumps += jumps.sum()

        y_min, y_max = np.nanmin(raw), np.nanmax(raw)
        if y_min == y_max:
            y_min -= 0.1
            y_max += 0.1
        margin = (y_max - y_min) * 0.1

        # 原始
        axes[j, 0].plot(raw, color="gray", alpha=0.6)
        if jumps.any():
            axes[j, 0].scatter(
                np.where(jumps)[0],
                raw[jumps],
                color="red",
                s=12
            )
        axes[j, 0].set_ylim(y_min - margin, y_max + margin)
        axes[j, 0].set_ylabel(f"Joint {j}")
        axes[j, 0].grid(alpha=0.2)

        # 处理后
        axes[j, 1].plot(processed, color="blue", linewidth=1.2)
        axes[j, 1].set_ylim(y_min - margin, y_max + margin)
        axes[j, 1].grid(alpha=0.2)

        if j == 0:
            axes[j, 0].set_title(f"Original (Episode {ep_id})")
            axes[j, 1].set_title(f"Processed x{loop}")

        if j == num_joints - 1:
            axes[j, 0].set_xlabel("Frame")
            axes[j, 1].set_xlabel("Frame")

    print(f"\nEpisode {ep_id} 跳变点总数: {total_jumps}")
    plt.show()


def process_all_episodes(df, target, loop, args):
    """
    对整个 parquet 中的所有 episode 进行处理
    """
    df_out = df.copy()
    all_data = df_out[target].tolist()

    start_idx = 0
    ep_ids = df_out["episode_index"].unique()

    print("开始处理所有 episodes...")

    for i, ep_id in enumerate(ep_ids, 1):
        ep_mask = df_out["episode_index"] == ep_id
        ep_size = ep_mask.sum()

        ep_data = np.array(all_data[start_idx:start_idx + ep_size])
        processed = np.zeros_like(ep_data)

        for j in range(ep_data.shape[1]):
            processed[:, j], _ = process_joint_loop(
                ep_data[:, j],
                loop=loop,
                jump_threshold=args.jump_threshold,
                smooth_window=args.smooth_window,
                poly_order=args.poly_order,
            )

        for k in range(ep_size):
            all_data[start_idx + k] = processed[k].tolist()

        start_idx += ep_size

        if i % 10 == 0 or i == len(ep_ids):
            print(f"处理进度: {i}/{len(ep_ids)}")

    df_out[target] = all_data
    print("全部 episodes 处理完成")
    return df_out


# ============================================================
# 主入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="LeRobot parquet joint cleaner"
    )

    parser.add_argument(
        "--file",
        type=str,
        default="file-000.parquet",
        help="要处理的 parquet 文件名（位于 data/ 目录下）"
    )
    parser.add_argument(
        "--target",
        type=str,
        default=DEFAULT_TARGET,
        help="处理字段（observation.state 或 action）"
    )
    parser.add_argument(
        "--jump_threshold",
        type=float,
        default=DEFAULT_JUMP_THRESHOLD,
        help="跳变检测阈值（rad）"
    )
    parser.add_argument(
        "--smooth_window",
        type=int,
        default=DEFAULT_SMOOTH_WINDOW,
        help="Savitzky-Golay 窗口长度（奇数）"
    )
    parser.add_argument(
        "--poly_order",
        type=int,
        default=DEFAULT_POLY_ORDER,
        help="Savitzky-Golay 多项式阶数"
    )
    parser.add_argument(
        "--loop",
        type=int,
        default=DEFAULT_LOOP,
        help="重复处理次数（用于观察过度平滑）"
    )

    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    data_file = script_dir.parent / "data" / args.file
    output_file = data_file.with_stem(data_file.stem + "_new")

    if not data_file.exists():
        print(f"找不到文件: {data_file}")
        return

    df = pd.read_parquet(data_file)
    ep_ids = sorted(df["episode_index"].unique())

    print(f"数据加载成功，共 {len(ep_ids)} 个 episodes")
    current_ep = ep_ids[0]

    while True:
        print(f"\n当前 Episode: {current_ep}")
        print("[数字] 切换 episode | [save] 保存 | [q] 退出")

        plot_episode(df, current_ep, args.target, args.loop, args)

        cmd = input("请输入指令: ").strip().lower()

        if cmd == "q":
            break
        elif cmd == "save":
            df_new = process_all_episodes(
                df, args.target, args.loop, args
            )
            df_new.to_parquet(output_file, index=False)
            print(f"已保存至: {output_file}")
            break
        else:
            try:
                ep = int(cmd)
                if ep in ep_ids:
                    current_ep = ep
                else:
                    print("Episode 不存在")
            except ValueError:
                print("无效输入")


if __name__ == "__main__":
    main()
