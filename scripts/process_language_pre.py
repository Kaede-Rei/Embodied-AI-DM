"""
LeRobot Task Language 完整处理脚本 (支持 observation.task)

功能：
1. 交互式编辑 meta/tasks.parquet 中的 task 语言描述
2. 自动将 task 字段注入到所有 data/file-*.parquet 数据分片中
3. 支持注入到 observation.task（VLA 模型需要）
4. 完整的验证和备份机制

工作流程：
Step 1: 读取并编辑 meta/tasks.parquet
Step 2: 将 task_index → language 映射注入到数据分片
Step 3: 验证注入结果

使用方法：
    cd /your/workspace/
    python scripts/process_language.py

适用场景：
- 单任务 / 多任务 LeRobot 数据集
- 为 BC / VLA / Language-conditioned Policy 注入或完善 task 语义
- 自动处理元数据和实际数据的同步
- 支持 SmolVLA 等需要 observation.task 的模型

注意事项：
- 所有修改操作都会自动备份原文件
- 不会新增或伪造 task_index
- 不会修改 episode / frame / action / observation 其他字段
- 同时注入顶层 task 和 observation.task 以保证兼容性
"""

import pandas as pd
from pathlib import Path
from tqdm import tqdm


def get_dataset_paths():
    """自动定位数据集路径"""
    script_dir = Path(__file__).resolve().parent
    dataset_dir = script_dir.parent / "dataset"
    
    tasks_path = dataset_dir / "meta" / "tasks.parquet"
    data_dir = dataset_dir / "data"
    
    return dataset_dir, tasks_path, data_dir


def is_valid_language(text: str) -> bool:
    """
    判断一个 task 的 Language 描述是否"有意义"
    
    判定规则：
    - 非 None
    - 非空字符串
    - 非默认占位文本（如 "Task description."）
    """
    if text is None:
        return False
    text = str(text).strip()
    if text == "":
        return False
    if text.lower() in ["task description.", "none", "null", "nan"]:
        return False
    return True


def prompt_multiline_input() -> str:
    """
    终端多行输入工具函数
    
    使用说明：
    - 支持输入多行文本
    - 输入完成后：输入空行并按 Ctrl+D (Linux/Mac) 或 Ctrl+Z (Windows) 保存
    """
    print("\n请输入 Language 描述（支持多行，结束请输入空行并按 Ctrl+D）：")
    lines = []
    try:
        while True:
            line = input()
            lines.append(line)
    except EOFError:
        pass
    return "\n".join(lines).strip()


def edit_tasks_metadata(tasks_path: Path) -> tuple[pd.DataFrame, str]:
    """
    Step 1: 交互式编辑 meta/tasks.parquet
    
    返回：
        (编辑后的 DataFrame, language 列名)
    """
    print("\n" + "=" * 80)
    print(" Step 1: 编辑 Task 元数据 (meta/tasks.parquet)")
    print("=" * 80)
    
    if not tasks_path.exists():
        raise FileNotFoundError(f"未找到 {tasks_path}")
    
    df = pd.read_parquet(tasks_path)
    
    # 统一 tasks.parquet 的列结构
    if "task_index" not in df.columns:
        raise ValueError("tasks.parquet 中缺少 task_index 列，数据结构异常")
    
    # 自动识别 Language 列
    language_col = None
    for c in ["task", "task_description", "language"]:
        if c in df.columns:
            language_col = c
            break
    
    # 若不存在 language 列，则创建标准 task 列
    if language_col is None:
        language_col = "task"
        df[language_col] = ""
    
    # 按 task_index 排序，保证处理顺序稳定
    df = df.sort_values("task_index").reset_index(drop=True)
    
    print(f"\n当前数据集包含 {len(df)} 个 task")
    print(f"Language 列名: {language_col}\n")
    
    modified = False
    
    # 逐 task_index 处理
    for idx, row in df.iterrows():
        task_index = row["task_index"]
        current_lang = row.get(language_col, "")
        
        print("\n" + "-" * 80)
        print(f"Task Index: {task_index}")
        
        if is_valid_language(current_lang):
            print("检测到已存在的 Language 描述：")
            print("-" * 40)
            print(current_lang)
            print("-" * 40)
            choice = input("\n是否需要修改该 Language？ [y/N] ").strip().lower()
            if choice != "y":
                print("✓ 保持原有 Language，不做修改")
                continue
        else:
            print("⚠️  尚未注入有效的 Language 描述")
        
        # Language 输入确认循环
        while True:
            new_lang = prompt_multiline_input()
            
            print("\n你输入的 Language 内容如下：")
            print("-" * 40)
            print(new_lang)
            print("-" * 40)
            
            action = (
                input("\n确认操作？ [y=保存 / r=重新输入 / s=跳过当前 task] ")
                .strip()
                .lower()
            )
            
            if action == "y":
                df.at[idx, language_col] = new_lang
                modified = True
                print("✓ Language 已保存")
                break
            elif action == "r":
                print("↻ 重新输入 Language")
                continue
            elif action == "s":
                print("⊗ 跳过当前 task，不做修改")
                break
            else:
                print("❌ 无效输入，请重新选择")
    
    # 写回 parquet
    if modified:
        backup_path = tasks_path.with_suffix(".parquet.bak")
        if not backup_path.exists():
            tasks_path.rename(backup_path)
            print(f"\n✓ 原文件已备份为: {backup_path.name}")
        else:
            print(f"\n⚠️  备份文件已存在，覆盖写入: {tasks_path.name}")
        
        df.to_parquet(tasks_path)
        print(f"✓ {tasks_path.name} 已更新")
    else:
        print("\n未检测到修改内容，meta/tasks.parquet 未发生变更")
    
    return df, language_col


def inject_tasks_to_data(df_tasks: pd.DataFrame, language_col: str, data_dir: Path):
    """
    Step 2: 将 task 字段注入到所有数据分片文件
    
    注意：同时注入 task 和 observation.task 两个字段以保证兼容性
    
    参数：
        df_tasks: tasks.parquet 的 DataFrame
        language_col: language 列的名称
        data_dir: data/ 目录路径
    """
    print("\n" + "=" * 80)
    print(" Step 2: 注入 Task 字段到数据分片 (data/file-*.parquet)")
    print("=" * 80)
    
    # 构建 task_index → language 映射
    task_map = dict(zip(df_tasks["task_index"], df_tasks[language_col]))
    
    print(f"\n构建的 Task 映射表：")
    for idx, desc in task_map.items():
        preview = desc[:60] + "..." if len(desc) > 60 else desc
        print(f"  Task {idx}: {preview}")
    
    # 查找所有数据分片文件
    parquet_files = sorted(data_dir.glob("file-*.parquet"))
    
    if not parquet_files:
        print(f"\n⚠️  在 {data_dir} 中未找到任何 file-*.parquet 文件")
        print("可能原因：")
        print("  1. 数据集尚未生成数据分片")
        print("  2. 数据文件命名不符合 file-*.parquet 格式")
        return
    
    print(f"\n找到 {len(parquet_files)} 个数据分片文件")
    
    # 询问是否继续
    print("\n" + "⚠️ " * 20)
    print("注意：本操作将修改所有 data/file-*.parquet 文件")
    print("将注入以下字段：")
    print("  - task (顶层，兼容传统模型)")
    print("  - observation.task (VLA 模型需要)")
    print("原文件将自动备份为 .parquet.bak（仅首次备份）")
    print("⚠️ " * 20)
    
    confirm = input("\n是否继续注入 task 字段到数据分片？ [y/N] ").strip().lower()
    if confirm != "y":
        print("⊗ 跳过数据注入步骤")
        return
    
    print("\n开始处理...\n")
    
    success_count = 0
    skip_count = 0
    error_count = 0
    
    for parquet_file in tqdm(parquet_files, desc="注入 Task 字段"):
        try:
            # 读取数据（优先从备份读取）
            backup_path = parquet_file.with_suffix(".parquet.bak")
            if backup_path.exists():
                df = pd.read_parquet(backup_path)
            else:
                df = pd.read_parquet(parquet_file)
                # 首次处理时创建备份
                parquet_file.rename(backup_path)
            
            # 检查是否有 task_index 列
            if "task_index" not in df.columns:
                tqdm.write(f"⚠️  {parquet_file.name}: 缺少 task_index 列，跳过")
                skip_count += 1
                # 恢复文件（如果刚才备份了）
                if not parquet_file.exists():
                    backup_path.rename(parquet_file)
                continue
            
            # 根据 task_index 映射 task 字段
            task_values = df["task_index"].map(task_map)
            
            # 注入到两个位置
            df["task"] = task_values  # 顶层 task（兼容性）
            df["observation.task"] = task_values  # observation.task（VLA 模型需要）
            
            # 检查是否有未映射的 task_index
            unmapped = task_values.isnull().sum()
            if unmapped > 0:
                unique_unmapped = df[df["task_index"].map(task_map).isnull()]["task_index"].unique()
                tqdm.write(
                    f"⚠️  {parquet_file.name}: {unmapped}/{len(df)} 条数据无法映射 "
                    f"(task_index: {list(unique_unmapped)})"
                )
            
            # 写回文件
            df.to_parquet(parquet_file)
            success_count += 1
            
        except Exception as e:
            tqdm.write(f"❌ {parquet_file.name}: 处理失败 - {e}")
            error_count += 1
    
    print("\n" + "=" * 80)
    print("数据注入完成统计：")
    print(f"  ✓ 成功: {success_count} 个文件")
    print(f"  ⊗ 跳过: {skip_count} 个文件")
    print(f"  ❌ 失败: {error_count} 个文件")
    print("=" * 80)


def verify_injection(data_dir: Path):
    """
    Step 3: 验证注入结果
    """
    print("\n" + "=" * 80)
    print(" Step 3: 验证注入结果")
    print("=" * 80)
    
    parquet_files = sorted(data_dir.glob("file-*.parquet"))
    
    if not parquet_files:
        print("\n未找到数据分片文件，跳过验证")
        return
    
    # 检查第一个文件作为示例
    sample_file = parquet_files[0]
    print(f"\n检查示例文件: {sample_file.name}")
    print("-" * 80)
    
    try:
        df = pd.read_parquet(sample_file)
        
        # 检查两个 task 字段
        has_task = "task" in df.columns
        has_obs_task = "observation.task" in df.columns
        
        print(f"\n字段检查:")
        print(f"  {'✓' if has_task else '❌'} task (顶层)")
        print(f"  {'✓' if has_obs_task else '❌'} observation.task")
        
        if not has_task and not has_obs_task:
            print("\n❌ 两个 task 字段都不存在！")
            return
        
        print(f"\n数据集总行数: {len(df)}")
        
        # 检查顶层 task
        if has_task:
            null_count = df['task'].isnull().sum()
            print(f"\ntask 空值数量: {null_count}/{len(df)}")
        
        # 检查 observation.task
        if has_obs_task:
            null_count = df['observation.task'].isnull().sum()
            print(f"observation.task 空值数量: {null_count}/{len(df)}")
        
        # 显示示例内容（优先 observation.task）
        display_col = "observation.task" if has_obs_task else "task"
        print(f"\n前 5 条数据的 {display_col} 内容：")
        print("-" * 80)
        
        preview = df[["task_index", display_col]].head()
        for _, row in preview.iterrows():
            task_text = str(row[display_col])[:60] + "..." if len(str(row[display_col])) > 60 else str(row[display_col])
            print(f"  Task {row['task_index']}: {task_text}")
        
        print("\n" + "=" * 80)
        if has_obs_task:
            print("✓ 验证通过！数据集已成功注入 observation.task 字段")
        else:
            print("⚠️  仅注入了顶层 task，VLA 模型可能需要 observation.task")
        print("=" * 80)
        
    except Exception as e:
        print(f"❌ 验证失败: {e}")


def main():
    """主流程"""
    print("=" * 80)
    print(" LeRobot Task Language 完整处理工具 (支持 VLA)")
    print("=" * 80)
    print()
    print("功能：")
    print("  1. 交互式编辑 meta/tasks.parquet")
    print("  2. 自动注入 task 字段到 data/file-*.parquet")
    print("  3. 同时注入顶层 task 和 observation.task（VLA 模型支持）")
    print("  4. 验证注入结果")
    print()
    
    # 定位数据集路径
    dataset_dir, tasks_path, data_dir = get_dataset_paths()
    
    print(f"数据集路径: {dataset_dir}")
    print(f"任务元数据: {tasks_path}")
    print(f"数据分片目录: {data_dir}")
    
    if not tasks_path.exists():
        print(f"\n❌ 未找到 {tasks_path}")
        print("请确保当前工作目录正确，且数据集结构完整")
        return
    
    try:
        # Step 1: 编辑 meta/tasks.parquet
        df_tasks, language_col = edit_tasks_metadata(tasks_path)
        
        # Step 2: 注入到数据分片
        if data_dir.exists():
            inject_tasks_to_data(df_tasks, language_col, data_dir)
            
            # Step 3: 验证
            verify_injection(data_dir)
        else:
            print(f"\n⚠️  数据分片目录不存在: {data_dir}")
            print("仅完成了 meta/tasks.parquet 的编辑")
        
        print("\n" + "=" * 80)
        print("处理完成！")
        print("\n后续步骤：")
        print("  1. 检查验证结果是否符合预期")
        print("  2. 更新 info.json 添加 observation.task 配置")
        print("  3. 重新运行训练脚本")
        print("  4. 如有问题，可从 .parquet.bak 备份文件恢复")
        print("=" * 80)
        
    except Exception as e:
        print(f"\n❌ 处理过程中出现错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()