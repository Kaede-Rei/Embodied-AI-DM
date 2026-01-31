"""
LeRobot meta/tasks.parquet 交互式语言注入与检查脚本

功能：
- 读取 dataset/meta/tasks.parquet
- 按 task_index 顺序逐个检查 task 的 Language 描述
- 判断当前 task 是否已经注入“有效语言”
- 若未注入或用户选择修改，则支持终端交互式输入（支持多行）
- 输入完成后进行回显确认，可选择：
  - 保存
  - 重新输入
  - 跳过当前 task
- 若存在多个 task_index，将依次完成上述流程
- 仅在确认修改后才写回 parquet，并自动备份原文件

适用场景：
- 单任务 / 多任务 LeRobot 数据集
- 为 BC / VLA / Language-conditioned Policy 注入或完善 task 语义
- 不修改 data/*.parquet，仅作用于 meta/tasks.parquet

使用方法：
终端输入 cd /media/$USER/AgroTech/home/LeRobot-Workspace/custom-hw-sim/ 确保进入工作区后再启动脚本
python scripts/process_language.py

注意事项：
- 不会新增或伪造 task_index
- 不会修改 episode / frame / action / observation 数据
- 仅维护 task_index → language 的语义映射
"""

import pandas as pd
from pathlib import Path

DATASET_DIR = Path("dataset")
TASKS_PATH = DATASET_DIR / "meta" / "tasks.parquet"


def is_valid_language(text: str) -> bool:
    """
    判断一个 task 的 Language 描述是否“有意义”

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
    - 输入完成后：
        多输入一行空行并按 Ctrl+D 保存
        按 Ctrl+Z 取消
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


def main():
    if not TASKS_PATH.exists():
        raise FileNotFoundError(f"未找到 {TASKS_PATH}")

    df = pd.read_parquet(TASKS_PATH)

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

    print("=" * 60)
    print(" LeRobot Task Language 交互式注入工具")
    print("=" * 60)

    modified = False

    # 逐 task_index 处理
    for idx, row in df.iterrows():
        task_index = row["task_index"]
        current_lang = row.get(language_col, "")

        print("\n" + "-" * 60)
        print(f"当前 Task index: {task_index}")

        if is_valid_language(current_lang):
            print("✔ 检测到已存在的 Language 描述：\n")
            print(current_lang)
            choice = input("\n是否需要修改该 Language？ [y/N] ").strip().lower()
            if choice != "y":
                print("保持原有 Language，不做修改")
                continue
        else:
            print("✘ 尚未注入有效的 Language 描述")

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
                print("✔ Language 已保存")
                break
            elif action == "r":
                print("↺ 重新输入 Language")
                continue
            elif action == "s":
                print("↷ 跳过当前 task，不做修改")
                break
            else:
                print("无效输入，请重新选择")

    # 写回 parquet
    if modified:
        backup_path = TASKS_PATH.with_suffix(".parquet.bak")
        TASKS_PATH.rename(backup_path)
        df.to_parquet(TASKS_PATH)
        print("\n✔ tasks.parquet 已更新")
        print(f"✔ 原文件已备份为: {backup_path}")
    else:
        print("\n未检测到修改内容，tasks.parquet 未发生变更")

    print("\n处理完成。")


if __name__ == "__main__":
    main()
