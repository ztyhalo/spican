#!/bin/bash

password='root'

export ARCH=arm
export CROSS_COMPILE=/opt/M6708/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-
source /opt/M6708/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi
cd /home/hn/study/kernel-imx
make CONFIG_CAN_MCP251X=m -C /home/hn/study/kernel-imx M=/home/hn/study/kernel-imx/drivers/net/can modules


/usr/bin/expect <<-EOF

spawn scp /home/hn/study/kernel-imx/drivers/net/can/mcp251x.ko root@192.168.168.56:/lib/modules/3.0.35-2666-gbdde708-g0351673-dirty/kernel/drivers/net/can/
set timeout 300 
expect "root@192.168.168.56's password:"
set timeout 300 
send "$password\r"
set timeout 300 
send "exit\r"
expect eof
EOF
