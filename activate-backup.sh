#!/bin/bash
# activate.sh
# 便携式 micromamba 激活脚本（备份）
# 如果误删 home/activate.sh 请将此文件复制到 home/ 目录下并重命名为 activate.sh
# 使用方式：
#     . ./activate.sh
#     . ./activate.sh lerobot
#     source ./activate.sh lerobot

# 找到脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# micromamba 主目录
MAMBA_ROOT="${SCRIPT_DIR}/micromamba"

# micromamba 可执行文件
MAMBA_BIN="${MAMBA_ROOT}/bin/micromamba"
if [ ! -x "$MAMBA_BIN" ]; then
    echo "错误：未找到 micromamba 可执行文件："
    echo "      $MAMBA_BIN"
    echo "请确认已将 micromamba 解压到 micromamba/ 目录"
    return 1 2>/dev/null || exit 1
fi

# 目标环境（默认 base）
ENV_NAME="${1:-base}"
echo "目标环境: $ENV_NAME"

# 便携缓存目录
PORTABLE_CACHE="${SCRIPT_DIR}/.cache"
PORTABLE_PKGS="${PORTABLE_CACHE}/.pkgs"
PORTABLE_TMP="${PORTABLE_CACHE}/.tmp"
PORTABLE_PIP="${PORTABLE_CACHE}/.pip"
mkdir -p "$PORTABLE_CACHE" "$PORTABLE_PKGS" "$PORTABLE_TMP" "$PORTABLE_PIP"

# micromamba 专用环境变量
export MAMBA_ROOT_PREFIX="$MAMBA_ROOT"
export MAMBA_EXE="$MAMBA_BIN"

# 让 envs / pkgs / pip / tmp 全都放移动硬盘
export MAMBA_ENVS_DIRS="${MAMBA_ROOT}/envs"
export MAMBA_PKGS_DIRS="$PORTABLE_PKGS"
export PIP_CACHE_DIR="$PORTABLE_PIP"
export TMPDIR="$PORTABLE_TMP"
export TEMP="$PORTABLE_TMP"
export TMP="$PORTABLE_TMP"
export PIP_USER=no
export PYTHONNOUSERSITE=1

# 清理系统里已有 conda/mamba 影响
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
unset CONDA_SHLVL
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -E "conda|mamba" | tr '\n' ':')
export PATH="${MAMBA_ROOT}/bin:${PATH}"

# 初始化 shell hook
eval "$($MAMBA_BIN shell hook -s bash)"

# 创建环境（如果不存在）
if ! micromamba env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    if [ "$ENV_NAME" != "base" ]; then
        echo "环境 $ENV_NAME 不存在，是否创建？(y/n)"
        read -r ans
        if [ "$ans" = "y" ]; then
            micromamba create -y -n "$ENV_NAME" python=3.10
        else
            return 1
        fi
    fi
fi

# 激活
micromamba activate "$ENV_NAME"

if [ $? -eq 0 ]; then
    echo "--------------------------------------------------"
    echo "- 成功激活: $MAMBA_DEFAULT_ENV"
    echo "- micromamba: $(which micromamba)"
    echo "- python:     $(which python)"
    echo "- envs:       $MAMBA_ENVS_DIRS"
    echo "- pkgs:       $MAMBA_PKGS_DIRS"
    echo "- pip cache:  $PIP_CACHE_DIR"
    echo "- tmp:        $TMPDIR"
    echo "--------------------------------------------------"
else
    echo "环境激活失败"
    micromamba env list
fi

