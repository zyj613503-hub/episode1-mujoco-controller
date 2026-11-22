#!/bin/bash
# 双击运行：给这个文件夹建一个独立的 Python 环境（.venv），并装好运行需要的库。
# 只需要做一次；装好后以后直接双击「2-启动上位机.command」。
cd "$(dirname "$0")" || exit 1
ROOT="$PWD"
cd "$ROOT" || exit 1

echo "=========== episode1 + MuJoCo 上位机 · 安装依赖 ==========="
echo "文件夹: $ROOT"
echo

# 1) 找一个能用的 Python（3.9 ~ 3.13；在 3.12 上实测过）
PYBIN=""
for cand in python3.12 python3.13 python3.11 python3.10 python3; do
  if command -v "$cand" >/dev/null 2>&1; then
    v=$("$cand" -c 'import sys;print("%d.%d"%sys.version_info[:2])' 2>/dev/null)
    case "$v" in
      3.9|3.10|3.11|3.12|3.13) PYBIN="$(command -v "$cand")"; echo "使用 Python $v → $PYBIN"; break ;;
    esac
  fi
done
if [ -z "$PYBIN" ]; then
  echo "❌ 没找到合适的 Python（需要 3.9–3.13）。"
  echo "   请先装 Python 3.12：https://www.python.org/downloads/ （或 brew install python@3.12）"
  read -r -p "按回车关闭…"
  exit 1
fi

# 2) 建虚拟环境
if [ -x "$ROOT/.venv/bin/python" ]; then
  echo "已存在 .venv，跳过创建"
else
  echo "创建 .venv …"
  "$PYBIN" -m venv "$ROOT/.venv" || { echo "❌ 创建 .venv 失败"; read -r -p "按回车关闭…"; exit 1; }
fi
VPY="$ROOT/.venv/bin/python"

# 3) 装依赖
echo
echo "安装依赖（mujoco / mink / numpy，约 100 MB，需要联网）…"
"$VPY" -m pip install -U pip >/dev/null 2>&1
"$VPY" -m pip install -r "$ROOT/requirements.txt" || {
  echo "❌ 依赖安装失败（多半是网络问题），可以重跑一次这个脚本。"
  read -r -p "按回车关闭…"
  exit 1
}

# 4) 自检
echo
"$VPY" - <<'EOF'
import mujoco, numpy, mink, mink.exceptions
print("mujoco", mujoco.__version__, "| numpy", numpy.__version__, "| mink OK")
EOF
if [ $? -ne 0 ]; then
  echo "❌ 自检失败"
  read -r -p "按回车关闭…"
  exit 1
fi
echo
echo "✅ 安装完成。现在可以双击「2-启动上位机.command」了。"
read -r -p "按回车关闭…"
