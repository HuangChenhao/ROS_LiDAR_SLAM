#!/bin/bash
# 一次性设置脚本：配置 Mac 到小车的免密 SSH
# 运行后以后所有连接都不需要密码

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
KEY_FILE="$SCRIPT_DIR/.ssh/rosmaster_codex_nopass"
PUB_KEY_FILE="$SCRIPT_DIR/.ssh/rosmaster_codex_nopass.pub"
HOST="192.168.0.110"
USER="pi"

echo "=== 设置 ROSMaster R2 免密 SSH ==="

# 1. 设置密钥权限
chmod 600 "$KEY_FILE"
echo "[OK] 密钥权限已设置"

# 2. 复制到 ~/.ssh/
mkdir -p ~/.ssh
cp "$KEY_FILE" ~/.ssh/rosmaster_codex_nopass
cp "$PUB_KEY_FILE" ~/.ssh/rosmaster_codex_nopass.pub
chmod 600 ~/.ssh/rosmaster_codex_nopass
echo "[OK] 密钥已复制到 ~/.ssh/"

# 3. 添加 SSH config（如果还没有的话）
if ! grep -q "Host rosmaster" ~/.ssh/config 2>/dev/null; then
    cat >> ~/.ssh/config << 'EOF'

Host rosmaster
    HostName 192.168.0.110
    User pi
    Port 22
    IdentityFile ~/.ssh/rosmaster_codex_nopass
    IdentitiesOnly yes
    StrictHostKeyChecking no
    UserKnownHostsFile /dev/null
    LogLevel ERROR
EOF
    echo "[OK] SSH config 已添加"
else
    echo "[跳过] SSH config 已存在"
fi

# 4. 将公钥添加到小车（需要输入一次密码：yahboom）
echo ""
echo "现在需要把公钥添加到小车上（最后一次输入密码）..."
echo "密码是: yahboom"
echo ""
ssh-copy-id -i "$PUB_KEY_FILE" -o StrictHostKeyChecking=no "${USER}@${HOST}"

# 5. 测试免密连接
echo ""
echo "测试免密连接..."
if ssh -i ~/.ssh/rosmaster_codex_nopass -o StrictHostKeyChecking=no -o ConnectTimeout=5 "${USER}@${HOST}" "echo '免密连接成功！'" 2>/dev/null; then
    echo ""
    echo "=== 设置完成！==="
    echo "以后可以直接: ssh rosmaster"
    echo "或者脚本自动连接，无需密码。"
else
    echo ""
    echo "[错误] 免密连接测试失败，请检查密码是否正确。"
fi
