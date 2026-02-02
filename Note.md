# 开发笔记——Waveshare UGV with Jetson Orin Nano Super
## 1. Jetson Orin Nano Super开发板基础配置
刚拿到这个开发板，我们首先需要一些配置，才能在上面编写程序。
## 1.0 刷系统
一般而言，厂商会在出场时已经帮我们烧录好系统，可以直接使用。目前Jetson支持的系统最新是Ubuntu22.04。
## 1.1 连接到电脑
刚开始时，Jetson没有配网，无法SSH到它做更多操作。但是，Jetson上的Type-C口可以作为终端串口。将其连接至电脑后，使用mobaxterm等串口工具连接到对应的COM口，波特率115200，即可进入Jetson的终端。
## 1.2 配置网络
使用nmcli命令配置网络连接：
```bash
nmcli general status # 查看网络状态
nmcli device wifi list # 列出可用的WiFi网络
nmcli device wifi connect "SSID名称" password "密码" # 连接到指定WiFi网络
hostname -I # 查看当前IP地址
```
获得了IP后，就可以通过SSH连接到Jetson了。
## 1.3 更改密码
初始密码通常是默认的，为了安全起见，建议更改密码：
```bash
passwd # 按提示输入当前密码和新密码
sudo usermod -l 新用户名 旧用户名 # 更改用户名
```
## 1.4 基本工具下载
在SSH中新开中端，安装常用工具：
```bash
sudo apt update # 更新软件包列表
sudo apt upgrade # 升级已安装的软件包
# 安装git工具，文本编辑器和终端复用器
sudo apt install \ 
    git \ 
    vim \ 
    tmux
# 安装tailscale内网穿刺工具。这样Jetson就会有固定IP，而且可以跨局域网连接
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
# 按装ros2，docker等开发工具
wget http://fishros.com/install -O fishros && . fishros # 使用fishros脚本一键安装
```
## 2. UGV开发
基础资料参见[https://www.waveshare.net/wiki/UGV_Rover_Jetson_Orin_ROS2](url)