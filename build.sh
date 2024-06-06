export ARCH=arm
export CROSS_COMPILE=/opt/poky/1.7/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-

#make distclean
#make imx_v7_defconfig
make hndz_defconfig
#make menuconfig
make zImage -j12
make imx6dl-sabresd.dtb

scp arch/arm/boot/zImage root@169.254.1.83:/home/root
scp arch/arm/boot/dts/imx6dl-sabresd.dtb root@169.254.1.83:/home/root

exit 0

#make distclean
#make imx_v7_defconfig
make menuconfig

make zImage -j8 && cp arch/arm/boot/zImage var/tftpboot/zImage -f
make imx6dl-sabresd.dtb && cp arch/arm/boot/dts/imx6dl-sabresd.dtb var/tftpboot/ -f
make imx6q-sabresd.dtb  && cp arch/arm/boot/dts/imx6q-sabresd.dtb var/tftpboot/ -f
make modules && cp drivers/net/wireless/brcm80211/brcmfmac/brcmfmac.ko var/tftpboot -f && cp drivers/net/wireless/brcm80211/brcmutil/brcmutil.ko var/tftpboot/ -f

