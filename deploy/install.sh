#!/bin/bash
# 安装 dart_standalone 为 systemd user 服务（开机自启）
set -e

PROJ_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SERVICE_FILE="$PROJ_DIR/deploy/dart.service"

# 检查 ffmpeg（录像合并依赖）
if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "[WARN] 未检测到 ffmpeg，录像分帧无法合并为 mp4。请安装："
    echo "       sudo apt install ffmpeg"
fi

# 替换服务文件中的占位路径为实际路径
sed -i "s|/home/jade/Desktop/dart_standalone|$PROJ_DIR|g" "$SERVICE_FILE"

mkdir -p ~/.config/systemd/user/
cp "$SERVICE_FILE" ~/.config/systemd/user/dart.service
systemctl --user daemon-reload

echo "============================================"
echo "  dart_standalone 服务已安装"
echo "============================================"
echo ""
echo "启用并立即启动："
echo "  systemctl --user enable --now dart"
echo ""
echo "【重要】使服务在未登录/开机时自动启动："
echo "  loginctl enable-linger \$USER"
echo ""
echo "常用命令："
echo "  systemctl --user status dart     # 查看状态"
echo "  systemctl --user stop dart       # 停止"
echo "  systemctl --user start dart      # 启动"
echo "  journalctl --user -u dart -f     # 查看日志"
echo "============================================"
