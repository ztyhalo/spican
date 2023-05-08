#!/bin/bash

export ARCH=arm
export CROSS_COMPILE=/opt/M6708/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-
source /opt/M6708/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi

password='root'
cd /home/hn/study/kernel-imx
make imx6dl-sabresd.dtb

/usr/bin/expect <<-EOF

spawn scp /home/hn/study/kernel-imx/arch/arm/boot/dts/imx6dl-sabresd.dtb root@192.168.1.121:/media/mmcblk2p2/mktool/images/
set timeout 300 
expect "root@192.168.1.121's password:"
set timeout 300 
send "$password\r"
set timeout 300 
send "exit\r"

expect eof
EOF
