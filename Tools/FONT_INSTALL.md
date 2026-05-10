# 中文字体安装指南

## 问题说明

如果运行 `fast.py` 时中文显示为方框 `□□□`，说明系统缺少中文字体或matplotlib无法找到中文字体。

## 解决方案

### Linux系统（推荐）

#### 方法1: 安装文泉驿字体（最简单）

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install fonts-wqy-microhei fonts-wqy-zenhei

# CentOS/RHEL
sudo yum install wqy-microhei-fonts wqy-zenhei-fonts

# Arch Linux
sudo pacman -S wqy-microhei wqy-zenhei
```

#### 方法2: 安装Noto字体

```bash
# Ubuntu/Debian
sudo apt-get install fonts-noto-cjk fonts-noto-cjk-extra

# CentOS/RHEL
sudo yum install google-noto-sans-cjk-fonts

# Arch Linux
sudo pacman -S noto-fonts-cjk
```

#### 方法3: 手动安装字体

```bash
# 1. 下载字体文件（例如：SimHei.ttf）
# 2. 复制到字体目录
sudo mkdir -p /usr/share/fonts/truetype/chinese
sudo cp SimHei.ttf /usr/share/fonts/truetype/chinese/

# 3. 更新字体缓存
sudo fc-cache -fv

# 4. 清除matplotlib缓存
rm -rf ~/.cache/matplotlib
```

### Windows系统

Windows系统通常已经包含中文字体（如SimHei、Microsoft YaHei），如果仍然显示方框：

```powershell
# 清除matplotlib缓存
del %USERPROFILE%\.matplotlib\*
```

### macOS系统

macOS系统通常已经包含中文字体（如PingFang SC），如果仍然显示方框：

```bash
# 清除matplotlib缓存
rm -rf ~/.matplotlib
```

## 验证安装

### 1. 检查系统字体

```bash
# Linux
fc-list :lang=zh

# 输出应该包含中文字体，例如：
# /usr/share/fonts/truetype/wqy/wqy-microhei.ttc: WenQuanYi Micro Hei
```

### 2. 检查matplotlib可用字体

```python
import matplotlib.font_manager as fm

# 列出所有字体
fonts = [f.name for f in fm.fontManager.ttflist]
chinese_fonts = [f for f in fonts if 'WenQuanYi' in f or 'SimHei' in f or 'Noto' in f]
print("可用的中文字体:")
for font in chinese_fonts:
    print(f"  - {font}")
```

### 3. 测试中文显示

```python
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 设置字体
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei']
plt.rcParams['axes.unicode_minus'] = False

# 绘制测试图
plt.figure(figsize=(8, 6))
plt.plot([1, 2, 3], [1, 4, 9])
plt.title('中文测试 - 最速曲线')
plt.xlabel('水平距离 (米)')
plt.ylabel('垂直距离 (米)')
plt.show()
```

## 常见问题

### Q1: 安装字体后仍然显示方框？

**A**: 需要清除matplotlib缓存

```bash
# Linux/macOS
rm -rf ~/.cache/matplotlib
rm -rf ~/.matplotlib

# Windows
del %USERPROFILE%\.matplotlib\*
```

### Q2: 如何查看当前使用的字体？

**A**: 运行 `fast.py` 时会显示：

```
使用中文字体: WenQuanYi Micro Hei
```

### Q3: 警告"未找到中文字体"怎么办？

**A**: 按照上述方法安装字体，然后清除缓存

### Q4: 能否使用其他中文字体？

**A**: 可以！修改 `fast.py` 中的字体列表：

```python
# 在 setup_chinese_font() 函数中
fonts = ['你喜欢的字体', 'WenQuanYi Micro Hei', ...]
```

## 推荐字体

### Linux

| 字体名称 | 包名 | 特点 |
|---------|------|------|
| 文泉驿微米黑 | fonts-wqy-microhei | 轻量、清晰 |
| 文泉驿正黑 | fonts-wqy-zenhei | 完整、美观 |
| Noto Sans CJK | fonts-noto-cjk | Google开发、全面 |
| Droid Sans Fallback | fonts-droid-fallback | Android默认 |

### Windows

| 字体名称 | 说明 |
|---------|------|
| SimHei | 黑体，系统自带 |
| Microsoft YaHei | 微软雅黑，现代美观 |
| SimSun | 宋体，传统字体 |
| KaiTi | 楷体，书法风格 |

### macOS

| 字体名称 | 说明 |
|---------|------|
| PingFang SC | 苹方，系统默认 |
| Heiti SC | 黑体，清晰易读 |
| STHeiti | 华文黑体 |
| Arial Unicode MS | 支持多语言 |

## 快速修复脚本

### Linux一键安装

创建文件 `install_chinese_font.sh`:

```bash
#!/bin/bash

echo "安装中文字体..."

# 检测系统类型
if [ -f /etc/debian_version ]; then
    # Debian/Ubuntu
    sudo apt-get update
    sudo apt-get install -y fonts-wqy-microhei fonts-wqy-zenhei
elif [ -f /etc/redhat-release ]; then
    # CentOS/RHEL
    sudo yum install -y wqy-microhei-fonts wqy-zenhei-fonts
elif [ -f /etc/arch-release ]; then
    # Arch Linux
    sudo pacman -S --noconfirm wqy-microhei wqy-zenhei
else
    echo "未识别的系统，请手动安装字体"
    exit 1
fi

# 更新字体缓存
sudo fc-cache -fv

# 清除matplotlib缓存
rm -rf ~/.cache/matplotlib
rm -rf ~/.matplotlib

echo "字体安装完成！"
echo "请重新运行 fast.py"
```

运行：
```bash
chmod +x install_chinese_font.sh
./install_chinese_font.sh
```

## 无需中文的替代方案

如果无法安装中文字体，可以修改代码使用英文：

```python
# 修改标题和标签为英文
ax.set_xlabel('Horizontal Distance (m)', fontsize=12)
ax.set_ylabel('Vertical Distance (m)', fontsize=12)
ax.set_title('Brachistochrone Curve', fontsize=14, fontweight='bold')
```

或者创建一个英文版本的脚本 `fast_en.py`。

## 技术支持

如果以上方法都无法解决问题，请提供以下信息：

1. 操作系统和版本
2. Python版本
3. matplotlib版本
4. `fc-list :lang=zh` 的输出（Linux）
5. 错误信息截图

```bash
# 查看版本信息
python3 --version
python3 -c "import matplotlib; print(matplotlib.__version__)"
```

## 参考资料

- [matplotlib中文显示问题](https://matplotlib.org/stable/tutorials/text/text_props.html)
- [Linux字体配置](https://wiki.archlinux.org/title/Fonts)
- [文泉驿字体项目](http://wenq.org/)
