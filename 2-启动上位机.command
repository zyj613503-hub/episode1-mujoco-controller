#!/bin/bash
# 双击运行：启动上位机服务，并打开浏览器里的「基础控制」页。
# 页面上点「启动服务器」会自动弹出 MuJoCo 仿真窗口（独立进程），上位机通过网络控制它。
cd "$(dirname "$0")" || exit 1
ROOT="$PWD"
PY="$ROOT/.venv/bin/python"
PORT="${UPPER_PORT:-8770}"   # 想换端口就改这里（例如 8780），同时记住浏览器地址里的端口

if [ ! -x "$PY" ]; then
  echo "❌ 还没安装依赖。请先双击「1-安装依赖.command」。"
  read -r -p "按回车关闭…"
  exit 1
fi

if curl -s --max-time 1 "http://127.0.0.1:$PORT/api/health" >/dev/null 2>&1; then
  echo "服务已经在运行：http://127.0.0.1:$PORT/v2/base"
else
  echo "启动上位机服务：http://127.0.0.1:$PORT/v2/base"
  PORT="$PORT" "$PY" "$ROOT/运行器/server/app.py" &
  SRV=$!
  for i in $(seq 1 60); do
    sleep 0.5
    if curl -s --max-time 1 "http://127.0.0.1:$PORT/api/health" >/dev/null 2>&1; then break; fi
  done
fi

open "http://127.0.0.1:$PORT/v2/base"
echo
echo "浏览器已打开。关掉这个终端窗口 = 关闭上位机服务。"
wait ${SRV:-0} 2>/dev/null || true
