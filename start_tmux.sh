#!/bin/bash
# 使用方法: ./start_tmux.sh
# 进入已有会话: tmux attach -t wbt_dev
# 退出tmux: Ctrl+b 然后按 d
tmux new-session -s wbt_dev "bash -c 'claude --dangerously-skip-permissions; exec bash'" \; \
  split-window -h "codex" \; \
  set -g mouse on \; \
  attach
