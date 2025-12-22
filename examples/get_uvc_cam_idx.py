#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头检测与预览工具
功能：扫描指定索引范围内的USB摄像头，检测哪些可用，并可选择预览所有检测到的摄像头

使用方法：
1. 基本扫描（索引0到10）: python get_uvc_cam_idx.py
2. 指定扫描范围: python get_uvc_cam_idx.py --start 0 --end 5
3. 扫描并预览: python get_uvc_cam_idx.py --preview
4. 指定范围并预览: python get_uvc_cam_idx.py --start 0 --end 3 --preview

参数说明：
--start: 起始摄像头索引（默认0）
--end:   结束摄像头索引（默认10）
--preview: 启用预览模式
依赖库：OpenCV (cv2)
"""

import cv2
import argparse

def test_camera(index):
    """测试指定索引的摄像头是否可用"""
    cap = cv2.VideoCapture(index)
    if cap.isOpened():
        # 尝试读取一帧来确认摄像头真正可用
        ret, frame = cap.read()
        cap.release()
        if ret and frame is not None:
            return True
    return False

def scan_cameras(start_idx=0, end_idx=10):
    """扫描指定范围内的摄像头"""
    available_cameras = []
    
    print(f"正在扫描摄像头索引 {start_idx} 到 {end_idx}...")
    
    for i in range(start_idx, end_idx + 1):
        print(f"正在测试摄像头索引 {i}...", end=' ')
        if test_camera(i):
            available_cameras.append(i)
            print("可用")
        else:
            print("不可用")
    
    return available_cameras

def main():
    # 创建命令行参数解析器
    parser = argparse.ArgumentParser(description='检测可用的USB摄像头')
    parser.add_argument('--start', type=int, default=0, help='开始检测的摄像头索引 (默认: 0)')
    parser.add_argument('--end', type=int, default=10, help='结束检测的摄像头索引 (默认: 10)')
    parser.add_argument('--preview', action='store_true', help='预览所有可用摄像头')
    
    args = parser.parse_args()
    
    # 扫描摄像头
    available_cams = scan_cameras(args.start, args.end)
    
    # 显示结果
    print("\n===============================")
    if available_cams:
        print(f"- 找到 {len(available_cams)} 个可用的摄像头:")
        for cam_idx in available_cams:
            print(f"-    索引 {cam_idx}: 可用")
        print("===============================")
    else:
        print(f"在索引 {args.start} 到 {args.end} 范围内未找到可用的摄像头")
        print("请尝试调整 --start 和 --end 参数，或检查摄像头连接")
    
    # 如果启用预览模式，预览所有可用摄像头
    if args.preview and available_cams:
        print("\n正在启动摄像头预览 (按 'q' 键退出预览)...")
        
        # 为每个可用摄像头创建VideoCapture对象
        caps = []
        for cam_idx in available_cams:
            cap = cv2.VideoCapture(cam_idx)
            if cap.isOpened():
                caps.append((cam_idx, cap))
            else:
                print(f"警告: 无法打开摄像头索引 {cam_idx} 进行预览")
        
        if not caps:
            print("没有摄像头可以预览")
            return
        
        print(f"正在预览 {len(caps)} 个摄像头...")
        
        while True:
            frames = []
            titles = []
            
            # 从每个摄像头读取一帧
            for cam_idx, cap in caps:
                ret, frame = cap.read()
                if ret:
                    # 调整帧大小以便同时显示多个摄像头
                    frame = cv2.resize(frame, (640, 480))
                    frames.append(frame)
                    titles.append(f"Camera {cam_idx}")
            
            if not frames:
                break
            
            # 如果只有一个摄像头，直接显示
            if len(frames) == 1:
                cv2.imshow(titles[0], frames[0])
            else:
                # 多个摄像头时，将它们组合在一起显示
                # 每行显示2个摄像头
                rows = []
                for i in range(0, len(frames), 2):
                    row_frames = frames[i:i+2]
                    # 如果这一行只有一个摄像头，创建一个空白图像填充
                    if len(row_frames) == 1:
                        blank = cv2.resize(row_frames[0], (640, 480))
                        blank[:] = 0
                        row_frames.append(blank)
                    row = cv2.hconcat(row_frames)
                    rows.append(row)
                
                combined = cv2.vconcat(rows)
                cv2.imshow('Multiple Cameras Preview', combined)
            
            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # 释放所有摄像头
        for _, cap in caps:
            cap.release()
        
        cv2.destroyAllWindows()
    elif args.preview and not available_cams:
        print("没有可用的摄像头进行预览")

if __name__ == "__main__":
    main()