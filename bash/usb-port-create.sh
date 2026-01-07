#!/bin/bash
# USB 端口符号链接创建脚本
# 该脚本为每个物理 USB 端口创建稳定的符号链接，以便在连接多个设备（如摄像头、串口设备）时，可以通过固定路径访问对应设备，避免设备路径变化带来的问题
# 
# 使用方法：
# 1. 以 root 权限运行此脚本：sudo ./bash/usb-port-create.sh
# 2. 脚本会在 /etc/udev/rules.d/90-usb-by-port.rules 创建 udev 规则文件
# 3. 重新加载 udev 规则并触发设备重新识别
# 4. 终端输入 ls -l /dev/usbport-* 查看当前创建的符号链接及其对应的实际设备路径
#    示例输出：
#    /dev/usbport-1.4-tty -> ttyACM0
#    /dev/usbport-2.1-tty -> ttyUSB0
#    /dev/usbport-3.2-video -> video2
# 5. 在 LeRobot 配置中使用这些固定路径，例如：
#    FOLLOWER_PORT="/dev/usbport-1.4-tty"
#    LEADER_PORT="/dev/usbport-2.1-tty"
#    CAMERAS_CONFIG='{"context": {"type": "opencv", "index_or_path": "/dev/usbport-3.2-video", ...}}'
#
# 注意事项：该脚本在新设备只需启用一次用于创建 .rules 文件，之后每次连接设备时 udev 会自动创建对应符号链接，只需区分 tty 和 video

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

RULE_FILE="/etc/udev/rules.d/90-usb-by-port.rules"

safe_exit() {
    exit 0
}

# 创建规则文件
cat > "$RULE_FILE" << 'EOF'
# Auto-generated USB by physical port rules
# 每个端口根据设备类型创建对应的符号链接

# TTY 设备规则 (ttyACM*, ttyUSB*)
SUBSYSTEM=="tty", ENV{ID_PATH}=="*usb-*", \
    PROGRAM="/bin/bash -c 'echo $env{ID_PATH} | sed -E \"s/.*usb-[0-9]+:([0-9.]+).*/\1/\"'", \
    SYMLINK+="usbport-%c-tty"

# Video4Linux 设备规则 (主摄像头设备)
SUBSYSTEM=="video4linux", ENV{ID_PATH}=="*usb-*", ATTR{index}=="0", \
    PROGRAM="/bin/bash -c 'echo $env{ID_PATH} | sed -E \"s/.*usb-[0-9]+:([0-9.]+).*/\1/\"'", \
    SYMLINK+="usbport-%c-video"
EOF

echo "正在重新加载 udev 规则..."
sudo udevadm control --reload
sudo udevadm trigger

echo ""
echo "=========================================="
echo "- 设备符号链接创建成功！"
echo "- "
echo "- 查看当前设备链接："
echo "-   ls -l /dev/usbport-*"
echo "- "
echo "- 现在检查每个端口对应哪个设备："

# 显示当前连接的设备
for dev in /dev/usbport-*; do
    [ -e "$dev" ] && echo "-   $dev -> $(readlink $dev)"
done

echo "- "
echo "- 使用固定路径更新 LeRobot 配置，示例："
echo '-   FOLLOWER_PORT="/dev/usbport-1.4-tty"'
echo '-   LEADER_PORT="/dev/usbport-2.1-tty"'
echo '-   CAMERAS_CONFIG='"'"'{"context": {"type": "opencv", "index_or_path": "/dev/usbport-3.2-video", ...}}'"'"
echo "=========================================="

safe_exit