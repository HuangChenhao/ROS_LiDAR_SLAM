#!/bin/bash
# ROSMaster R2 远程命令执行器
# 用法: ./robot.sh "命令" > output.txt
# 所有脚本通过此文件连接小车，免密码

SSH_KEY="$HOME/.ssh/rosmaster_codex_nopass"
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=10"
HOST="pi@192.168.0.110"

if [ ! -f "$SSH_KEY" ]; then
    echo "错误: SSH 密钥未设置，请先运行 setup_ssh.sh" >&2
    exit 1
fi

ssh -i "$SSH_KEY" $SSH_OPTS "$HOST" bash << REMOTE
$@
REMOTE
