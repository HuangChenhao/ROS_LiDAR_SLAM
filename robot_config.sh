# 小车连接配置 — 换网络时只改这个文件
# IP 查找方法: 小车接屏幕看 ifconfig, 或路由器/热点管理页看设备列表,
#              或 Mac 上: arp -a | grep -i 'b8:27\|dc:a6\|d8:3a\|2c:cf' (树莓派网卡前缀)
# 学校热点 (IPv6-only): 用小车的 IPv6 链路本地地址 (ndp -an 查, MAC 2c:cf:67:a9:d1:53)
ROBOT_IP="192.168.0.110"
# 家里路由器时改回: ROBOT_IP="192.168.0.110"
ROBOT_USER="pi"
