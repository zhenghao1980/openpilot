#!/usr/bin/env bash
# WSL 联调四件套：mapd / maprenderd / 合成数据 / UI
# 用法：bash tools/sim/run_ui_sim.sh   （每个组件一个 tmux 窗格，关掉即停）
set -e
cd "$(git rev-parse --show-toplevel 2>/dev/null || echo ~)"
tmux new-session -d -s mapsim -n stack
tmux send-keys -t mapsim:stack 'echo "== 1. mapd =="; ./selfdrive/mapd/mapd 2>&1 | tee /tmp/mapd.log' C-m
tmux split-window -h -t mapsim:stack
tmux send-keys -t mapsim:stack 'echo "== 2. maprenderd =="; sleep 1; ./selfdrive/maprender/build/maprenderd /data/mapd_render/style.json /data/mapd_render/china.mbtiles 2>&1 | tee /tmp/maprenderd.log' C-m
tmux split-window -v -t mapsim:stack
tmux send-keys -t mapsim:stack 'echo "== 3. sim gps =="; sleep 2; python tools/sim/sim_gps.py 2>&1 | tee /tmp/sim.log' C-m
tmux split-window -v -t mapsim:stack
tmux send-keys -t mapsim:stack 'echo "== 4. UI =="; sleep 3; python selfdrive/ui/ui.py 2>&1 | tee /tmp/ui.log' C-m
tmux select-layout -t mapsim:stack tiled
echo "tmux session 'mapsim' 已启动：tmux attach -t mapsim"
