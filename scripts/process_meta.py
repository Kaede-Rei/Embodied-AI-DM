"""
LeRobot dataset/meta/stats.json 数据处理脚本，用于统计信息的重新计算与更新

前提：将所有 <repo_id>/data/chunk-xxx/file-xxx.parquet 下载到本地 dataset/data/ 目录，经过处理后得到 file-xxx_new.parquet，最后处理新数据会输出更新后的 stats.json 到 dataset/meta/ 下

使用方法：
终端输入 cd /media/$USER/AgroTech/home/LeRobot-Workspace/custom-hw-sim/ 确保进入工作区后再启动脚本
python scripts/process_meta.py
"""

import json
import numpy as np
import pandas as pd
from pathlib import Path

DATA_DIR = Path("dataset/data")
META_STATS = Path("dataset/meta/stats.json")

TARGET_KEYS = ["observation.state", "action"]

PARQUET_SUFFIX = "_new.parquet"


def load_all_processed_parquets():
    dfs = []
    print(f"- 从目录 {DATA_DIR} 加载所有以 '{PARQUET_SUFFIX}' 结尾的 parquet 文件：")
    for p in sorted(DATA_DIR.glob(f"**/*{PARQUET_SUFFIX}")):
        print(f"-   {p}")
        dfs.append(pd.read_parquet(p))

    if not dfs:
        raise RuntimeError(
            f"- 在目录 {DATA_DIR} 中未找到以 '{PARQUET_SUFFIX}' 结尾的 parquet 文件"
        )

    return pd.concat(dfs, ignore_index=True)


def compute_stats_matrix(matrix: np.ndarray):
    """
    matrix: 形状 (N, D)
    """
    return {
        "min": matrix.min(axis=0).tolist(),
        "max": matrix.max(axis=0).tolist(),
        "mean": matrix.mean(axis=0).tolist(),
        "std": matrix.std(axis=0).tolist(),
        "count": [matrix.shape[0]],
        "q01": np.quantile(matrix, 0.01, axis=0).tolist(),
        "q10": np.quantile(matrix, 0.10, axis=0).tolist(),
        "q50": np.quantile(matrix, 0.50, axis=0).tolist(),
        "q90": np.quantile(matrix, 0.90, axis=0).tolist(),
        "q99": np.quantile(matrix, 0.99, axis=0).tolist(),
    }


def main():
    print("=== 重新构建 meta/stats.json ===")

    # 读取所有处理后的 parquet 文件
    df_all = load_all_processed_parquets()
    print(f"- 总行数: {len(df_all)}")

    # 读取原始 stats.json
    with open(META_STATS, "r") as f:
        stats = json.load(f)

    # 逐字段更新统计信息
    for key in TARGET_KEYS:
        if key not in df_all.columns:
            raise KeyError(f"- 在 parquet 文件中未找到列 '{key}'")

        values = df_all[key].values

        # 维度一致性检查
        first_dim = len(values[0])
        for i, v in enumerate(values):
            if len(v) != first_dim:
                raise ValueError(
                    f"- {key} 在第 {i} 行维度不匹配: " f"{len(v)} != {first_dim}"
                )

        data = np.stack(values, axis=0)
        print(f"- {key}: 形状 = {data.shape}")

        stats[key] = compute_stats_matrix(data)

    # 写回 stats.json
    with open(META_STATS, "w") as f:
        json.dump(stats, f, indent=2)

    print("=== stats.json 更新成功 ===")


if __name__ == "__main__":
    main()
