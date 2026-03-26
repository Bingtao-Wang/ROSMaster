#!/bin/bash
# 使用方法: ./start_tmux.sh
# 进入已有会话: tmux attach -t wbt_dev
# 退出tmux: Ctrl+b 然后按 d

SESSION_NAME="wbt_dev"

# 已存在会话时直接附着，避免重复启动
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  tmux attach -t "$SESSION_NAME"
  exit 0
fi

# 创建双窗格会话：左侧 Claude，右侧 Codex
# 说明：
# - 关闭 mouse：避免当前终端/聊天分屏 UI 中出现选区、copy-mode、误粘贴问题
# - 关闭 set-clipboard：避免 tmux 与外部剪贴板联动，减少把 pane 内容误当作粘贴源的风险
# - 保留最小配置，优先稳定输入体验

tmux new-session -d -s "$SESSION_NAME" "bash -lc 'claude --dangerously-skip-permissions; exec bash'"
tmux split-window -h -t "$SESSION_NAME":0 "bash -lc 'codex; exec bash'"
tmux set-option -t "$SESSION_NAME" -g mouse on
tmux set-option -t "$SESSION_NAME" -g set-clipboard off

tmux display-message -t "$SESSION_NAME" "Session $SESSION_NAME started (mouse=on, set-clipboard=off)"
tmux attach -t "$SESSION_NAME"
