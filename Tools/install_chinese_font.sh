#!/bin/bash

# 中文字体安装脚本
# 用于解决matplotlib中文显示为方框的问题

set -e

echo "=========================================="
echo "  中文字体安装脚本"
echo "=========================================="
echo ""

# 检测系统类型
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$ID
else
    echo "错误: 无法检测操作系统类型"
    exit 1
fi

echo "检测到操作系统: $OS"
echo ""

# 根据系统类型安装字体
case $OS in
    ubuntu|debian)
        echo "安装文泉驿字体..."
        sudo apt-get update
        sudo apt-get install -y fonts-wqy-microhei fonts-wqy-zenhei fonts-noto-cjk
        ;;
    centos|rhel|fedora)
        echo "安装文泉驿字体..."
        sudo yum install -y wqy-microhei-fonts wqy-zenhei-fonts google-noto-sans-cjk-fonts
        ;;
    arch|manjaro)
        echo "安装文泉驿字体..."
        sudo pacman -S --noconfirm wqy-microhei wqy-zenhei noto-fonts-cjk
        ;;
    *)
        echo "警告: 未识别的系统 ($OS)"
        echo "请手动安装以下字体之一:"
        echo "  - WenQuanYi Micro Hei"
        echo "  - WenQuanYi Zen Hei"
        echo "  - Noto Sans CJK"
        exit 1
        ;;
esac

echo ""
echo "更新字体缓存..."
sudo fc-cache -fv

echo ""
echo "清除matplotlib缓存..."
rm -rf ~/.cache/matplotlib 2>/dev/null || true
rm -rf ~/.matplotlib 2>/dev/null || true

echo ""
echo "=========================================="
echo "  字体安装完成！"
echo "=========================================="
echo ""
echo "已安装的中文字体:"
fc-list :lang=zh | head -5
echo ""
echo "请重新运行 fast.py 测试中文显示"
echo ""
