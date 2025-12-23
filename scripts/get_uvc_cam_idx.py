#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USB 摄像头检测与能力扫描工具（OpenCV + v4l2）

功能：
1. 扫描指定 OpenCV 摄像头索引范围内的可用摄像头
2. 查询每个可用摄像头真实支持的分辨率及对应帧率（FPS）
3. 可选实时预览摄像头画面

工作原理：
- 使用 OpenCV (cv2) 探测可用摄像头索引
- 使用 v4l2-ctl 查询摄像头在内核层面支持的分辨率和帧率
- 预览阶段仅用于显示，不用于能力探测

系统依赖（必须）：
- Linux
- v4l-utils（提供 v4l2-ctl）
  Ubuntu / Debian:
      sudo apt install v4l-utils

Python 依赖：
- opencv-python
- argparse（标准库）

使用示例：
1. 扫描默认索引范围（0~10）：
   python scripts/get_uvc_cam_idx.py

2. 指定扫描索引范围：
   python scripts/get_uvc_cam_idx.py --start 0 --end 4

3. 扫描并预览所有可用摄像头：
   python scripts/get_uvc_cam_idx.py --preview

4. 指定范围并预览：
   python scripts/get_uvc_cam_idx.py --start 2 --end 6 --preview

参数说明：
--start    起始 OpenCV 摄像头索引（默认 0）
--end      结束 OpenCV 摄像头索引（默认 10）
--preview  启用实时预览模式（按 q 退出）
"""

import cv2
import argparse
import subprocess
import re
import shutil
import sys


def check_v4l2_ctl():
    """检查 v4l2-ctl 是否存在"""
    if shutil.which("v4l2-ctl") is None:
        print(
            "\n[ERROR] 未检测到 v4l2-ctl 工具。\n"
            "该脚本需要 v4l2-ctl 来查询摄像头分辨率和帧率。\n\n"
            "请安装：\n"
            "  sudo apt install v4l-utils\n"
        )
        sys.exit(1)


def test_camera(index):
    """测试指定 OpenCV 摄像头索引是否可用"""
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return False
    ret, _ = cap.read()
    cap.release()
    return ret


def list_supported_formats(video_dev):
    """
    使用 v4l2-ctl 查询摄像头支持的分辨率和 FPS

    返回格式：
    {
        (width, height): [fps1, fps2, ...]
    }
    """
    cmd = ["v4l2-ctl", "--device", video_dev, "--list-formats-ext"]
    proc = subprocess.run(cmd, capture_output=True, text=True)

    if proc.returncode != 0:
        return {}

    res = {}
    current_size = None

    size_pattern = re.compile(r"Size:\s+Discrete\s+(\d+)x(\d+)")
    fps_pattern = re.compile(r"\((\d+\.\d+)\s+fps\)")

    for line in proc.stdout.splitlines():
        size_match = size_pattern.search(line)
        if size_match:
            w, h = map(int, size_match.groups())
            current_size = (w, h)
            res.setdefault(current_size, [])
            continue

        fps_match = fps_pattern.search(line)
        if fps_match and current_size:
            fps = int(float(fps_match.group(1)))
            res[current_size].append(fps)

    return res


def preview_camera(index):
    """实时预览指定摄像头"""
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"[WARN] 无法预览摄像头 {index}")
        return

    print(f"[INFO] 正在预览摄像头 {index}（按 q 退出）")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow(f"Camera {index}", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="USB 摄像头能力扫描工具（OpenCV + v4l2）")
    parser.add_argument("--start", type=int, default=0, help="起始 OpenCV 摄像头索引（默认 0）")
    parser.add_argument("--end", type=int, default=10, help="结束 OpenCV 摄像头索引（默认 10）")
    parser.add_argument("--preview", action="store_true", help="启用实时预览模式")

    args = parser.parse_args()

    check_v4l2_ctl()

    print(f"扫描 OpenCV 摄像头索引 {args.start} 到 {args.end}")

    available = []

    for idx in range(args.start, args.end + 1):
        print(f"测试索引 {idx}...", end=" ")
        if test_camera(idx):
            print("可用")
            available.append(idx)
        else:
            print("不可用")

    if not available:
        print("未发现可用摄像头")
        return

    print("\n========================================")
    for idx in available:
        video_dev = f"/dev/video{idx}"
        print(f"- 摄像头索引 {idx} ({video_dev})")

        caps = list_supported_formats(video_dev)
        if not caps:
            print("  无法获取分辨率信息")
            continue

        for (w, h), fps_list in caps.items():
            fps_str = ", ".join(map(str, sorted(set(fps_list))))
            print(f"-    {w}x{h} @ [{fps_str}] FPS")

        if args.preview:
            preview_camera(idx)
    print("========================================")


if __name__ == "__main__":
    main()
