/*
 * Copyright (C) 2011-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include <linux/unaligned/access_ok.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

#define IMX291_CHIP_ID_H	(0xA0)
#define IMX291_CHIP_ID_L	(0xB2)

#define OV5640_VOLTAGE_ANALOG               2800000
#define OV5640_VOLTAGE_DIGITAL_CORE         1500000
#define OV5640_VOLTAGE_DIGITAL_IO           1800000


#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV5640_XCLK_MIN 6000000
#define OV5640_XCLK_MAX 24000000

#define OV5640_CHIP_ID_HIGH_BYTE	0x300A
#define OV5640_CHIP_ID_LOW_BYTE		0x300B

#define imx307_REG_VALUE_08BIT		1
#define imx307_REG_VALUE_16BIT		2

#define imx307_REG_MODE_SELECT		0x0100
#define imx307_MODE_STANDBY		    0x00
#define imx307_MODE_STREAMING		0x01

/* Chip ID */
#define imx307_REG_CHIP_ID		0x0000
#define imx307_CHIP_ID			0x0219

enum sonyimx_mode {
	imx_mode_1080p_1920_1080 = 1,
	imx_mode_3280_2464 =2,
	imx_mode_1640_1232 = 4,
	imx_mode_1640_922 = 5,
	imx_mode_1280_720 = 6,
	imx_mode_640_480 = 7,
};


enum imx307_frame_rate {
	imx_15_fps,
	imx_30_fps
};

static int imx307_framerates[] = {
	[imx_15_fps] = 15,
	[imx_30_fps] = 30,
};

/* image size under 1280 * 960 are SUBSAMPLING
 * image size upper 1280 * 960 are SCALING
 */
enum ov5640_downsize_mode {
	SUBSAMPLING,
	SCALING,
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};
struct imx307_reg {
	u16 address;
	u8 val;
};

struct imx307_mode_info {
	enum sonyimx_mode mode;
	enum ov5640_downsize_mode dn_mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data imx307_data;
static int pwn_gpio, rst_gpio;


static const struct imx307_reg mode_1920_1080_regs[] = {
	{0x0100, 0x00},
	{0x30eb, 0x05},
	{0x30eb, 0x0c},
	{0x300a, 0xff},
	{0x300b, 0xff},
	{0x30eb, 0x05},
	{0x30eb, 0x09},
	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012a, 0x18},
	{0x012b, 0x00},
	{0x0162, 0x0d},
	{0x0163, 0x78},
	{0x0164, 0x02},
	{0x0165, 0xa8},
	{0x0166, 0x0a},
	{0x0167, 0x27},
	{0x0168, 0x02},
	{0x0169, 0xb4},
	{0x016a, 0x06},
	{0x016b, 0xeb},
	{0x016c, 0x07},
	{0x016d, 0x80},
	{0x016e, 0x04},
	{0x016f, 0x38},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x00},
	{0x0175, 0x00},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x030b, 0x01},
	{0x030c, 0x00},
	{0x030d, 0x72},
	{0x0624, 0x07},
	{0x0625, 0x80},
	{0x0626, 0x04},
	{0x0627, 0x38},
	{0x455e, 0x00},
	{0x471e, 0x4b},
	{0x4767, 0x0f},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47b4, 0x14},
	{0x4713, 0x30},
	{0x478b, 0x10},
	{0x478f, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0e},
	{0x479b, 0x0e},
	{0x0162, 0x0d},
	{0x0163, 0x78},
};


static struct imx307_mode_info imx307_mode_info_data = {
		imx_mode_1080p_1920_1080, SUBSAMPLING, 640,  480,
		mode_1920_1080_regs,
		ARRAY_SIZE(mode_1920_1080_regs)};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;

static int imx307_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int imx307_remove(struct i2c_client *client);

static s32 imx307_read_reg(u16 reg, u8 *val);
static s32 imx307_write_reg(u16 reg, u8 val);

static const struct i2c_device_id imx307_id[] = {
	{"imx307_mipi", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, imx307_id);

static struct i2c_driver imx307_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "imx307_mipi",
		  },
	.probe  = imx307_probe,
	.remove = imx307_remove,
	.id_table = imx307_id,
};

static void imx307_standby(s32 enable)
{
	if (enable)
		gpio_set_value(pwn_gpio, 1);
	else
		gpio_set_value(pwn_gpio, 0);

	msleep(2);
}

static void imx307_reset(void)
{
	/* camera reset */
	gpio_set_value(rst_gpio, 1);

	/* camera power dowmn */
	gpio_set_value(pwn_gpio, 1);
	msleep(5);

	gpio_set_value(pwn_gpio, 0);
	msleep(5);

	gpio_set_value(rst_gpio, 0);
	msleep(1);

	gpio_set_value(rst_gpio, 1);
	msleep(5);

	gpio_set_value(pwn_gpio, 1);
}

static int imx307_power_on(struct device *dev)
{
	int ret = 0;

	io_regulator = devm_regulator_get(dev, "DOVDD");
	if (!IS_ERR(io_regulator)) {
		regulator_set_voltage(io_regulator,
				      OV5640_VOLTAGE_DIGITAL_IO,
				      OV5640_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(io_regulator);
		if (ret) {
			pr_err("%s:io set voltage error\n", __func__);
			return ret;
		} else {
			dev_dbg(dev,
				"%s:io set voltage ok\n", __func__);
		}
	} else {
		pr_err("%s: cannot get io voltage error\n", __func__);
		io_regulator = NULL;
	}

	core_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(core_regulator)) {
		regulator_set_voltage(core_regulator,
				      OV5640_VOLTAGE_DIGITAL_CORE,
				      OV5640_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(core_regulator);
		if (ret) {
			pr_err("%s:core set voltage error\n", __func__);
			return ret;
		} else {
			dev_dbg(dev,
				"%s:core set voltage ok\n", __func__);
		}
	} else {
		core_regulator = NULL;
		pr_err("%s: cannot get core voltage error\n", __func__);
	}

	analog_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(analog_regulator)) {
		regulator_set_voltage(analog_regulator,
				      OV5640_VOLTAGE_ANALOG,
				      OV5640_VOLTAGE_ANALOG);
		ret = regulator_enable(analog_regulator);
		if (ret) {
			pr_err("%s:analog set voltage error\n",
				__func__);
			return ret;
		} else {
			dev_dbg(dev,
				"%s:analog set voltage ok\n", __func__);
		}
	} else {
		analog_regulator = NULL;
		pr_err("%s: cannot get analog voltage error\n", __func__);
	}

	return ret;
}

static s32 imx307_write_reg(u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(imx307_data.i2c_client, au8Buf, 3) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}

static s32 imx307_read_reg(u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(imx307_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(imx307_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}


/* Read registers up to 2 at a time */
static int nimx307_read_reg(u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = imx307_data.i2c_client;
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 2 at a time */
static int nimx307_write_reg(u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = imx307_data.i2c_client;
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx307_write_regs(const struct imx307_reg *regs, u32 len)
{
	struct i2c_client *client = imx307_data.i2c_client;
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = nimx307_write_reg(regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}


int imx291_read( uint16_t reg,
		unsigned char *value)
{
	struct i2c_client *client = imx307_data.i2c_client;
	uint8_t buf[2] = {(reg>>8)&0xff, reg&0xff};
	struct i2c_msg msg[2] = {
		[0] = {
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= buf,
		},
		[1] = {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= value,
		}
	};
	int ret;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret > 0)
		ret = 0;

	return ret;
}

int imx291_write( uint16_t reg,
		 unsigned char value)
{
	struct i2c_client *client = imx307_data.i2c_client;
	uint8_t buf[3] = {(reg>>8)&0xff, reg&0xff, value};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 3,
		.buf	= buf,
	};
	int ret;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int prev_sysclk, prev_HTS;
static int AE_low, AE_high, AE_Target = 52;

void imx307_stream_on(void)
{
	imx307_write_reg(0x4202, 0x00);
}

void imx307_stream_off(void)
{
	imx307_write_reg(0x4202, 0x0f);
}


//int imx307_get_sysclk(void)
//{
//	 /* calculate sysclk */
//	int xvclk = imx307_data.mclk / 10000;
//	int temp1, temp2;
//	int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv;
//	int Bit_div2x = 1, sclk_rdiv, sysclk;
//	u8 temp;
//
//	int sclk_rdiv_map[] = {1, 2, 4, 8};
//
//	temp1 = ov5640_read_reg(0x3034, &temp);
//	temp2 = temp1 & 0x0f;
//	if (temp2 == 8 || temp2 == 10)
//		Bit_div2x = temp2 / 2;
//
//	temp1 = ov5640_read_reg(0x3035, &temp);
//	SysDiv = temp1>>4;
//	if (SysDiv == 0)
//		SysDiv = 16;
//
//	temp1 = ov5640_read_reg(0x3036, &temp);
//	Multiplier = temp1;
//
//	temp1 = ov5640_read_reg(0x3037, &temp);
//	PreDiv = temp1 & 0x0f;
//	Pll_rdiv = ((temp1 >> 4) & 0x01) + 1;
//
//	temp1 = ov5640_read_reg(0x3108, &temp);
//	temp2 = temp1 & 0x03;
//	sclk_rdiv = sclk_rdiv_map[temp2];
//
//	VCO = xvclk * Multiplier / PreDiv;
//
//	sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;
//
//	return sysclk;
//}
//
//void OV5640_set_night_mode(void)
//{
//	 /* read HTS from register settings */
//	u8 mode;
//
//	ov5640_read_reg(0x3a00, &mode);
//	mode &= 0xfb;
//	ov5640_write_reg(0x3a00, mode);
//}
//
//int OV5640_get_HTS(void)
//{
//	 /* read HTS from register settings */
//	int HTS;
//	u8 temp;
//
//	HTS = ov5640_read_reg(0x380c, &temp);
//	HTS = (HTS<<8) + ov5640_read_reg(0x380d, &temp);
//
//	return HTS;
//}
//
//int OV5640_get_VTS(void)
//{
//	 /* read VTS from register settings */
//	int VTS;
//	u8 temp;
//
//	/* total vertical size[15:8] high byte */
//	VTS = ov5640_read_reg(0x380e, &temp);
//
//	VTS = (VTS<<8) + ov5640_read_reg(0x380f, &temp);
//
//	return VTS;
//}
//
//int OV5640_set_VTS(int VTS)
//{
//	 /* write VTS to registers */
//	 int temp;
//
//	 temp = VTS & 0xff;
//	 ov5640_write_reg(0x380f, temp);
//
//	 temp = VTS>>8;
//	 ov5640_write_reg(0x380e, temp);
//
//	 return 0;
//}
//
//int OV5640_get_shutter(void)
//{
//	 /* read shutter, in number of line period */
//	int shutter;
//	u8 temp;
//
//	shutter = (ov5640_read_reg(0x03500, &temp) & 0x0f);
//	shutter = (shutter<<8) + ov5640_read_reg(0x3501, &temp);
//	shutter = (shutter<<4) + (ov5640_read_reg(0x3502, &temp)>>4);
//
//	 return shutter;
//}
//
//int OV5640_set_shutter(int shutter)
//{
//	 /* write shutter, in number of line period */
//	 int temp;
//
//	 shutter = shutter & 0xffff;
//
//	 temp = shutter & 0x0f;
//	 temp = temp<<4;
//	 ov5640_write_reg(0x3502, temp);
//
//	 temp = shutter & 0xfff;
//	 temp = temp>>4;
//	 ov5640_write_reg(0x3501, temp);
//
//	 temp = shutter>>12;
//	 ov5640_write_reg(0x3500, temp);
//
//	 return 0;
//}
//
//int OV5640_get_gain16(void)
//{
//	 /* read gain, 16 = 1x */
//	int gain16;
//	u8 temp;
//
//	gain16 = ov5640_read_reg(0x350a, &temp) & 0x03;
//	gain16 = (gain16<<8) + ov5640_read_reg(0x350b, &temp);
//
//	return gain16;
//}
//
//int OV5640_set_gain16(int gain16)
//{
//	/* write gain, 16 = 1x */
//	u8 temp;
//	gain16 = gain16 & 0x3ff;
//
//	temp = gain16 & 0xff;
//	ov5640_write_reg(0x350b, temp);
//
//	temp = gain16>>8;
//	ov5640_write_reg(0x350a, temp);
//
//	return 0;
//}
//
//int OV5640_get_light_freq(void)
//{
//	/* get banding filter value */
//	int temp, temp1, light_freq = 0;
//	u8 tmp;
//
//	temp = ov5640_read_reg(0x3c01, &tmp);
//
//	if (temp & 0x80) {
//		/* manual */
//		temp1 = ov5640_read_reg(0x3c00, &tmp);
//		if (temp1 & 0x04) {
//			/* 50Hz */
//			light_freq = 50;
//		} else {
//			/* 60Hz */
//			light_freq = 60;
//		}
//	} else {
//		/* auto */
//		temp1 = ov5640_read_reg(0x3c0c, &tmp);
//		if (temp1 & 0x01) {
//			/* 50Hz */
//			light_freq = 50;
//		} else {
//			/* 60Hz */
//		}
//	}
//	return light_freq;
//}
//
//void OV5640_set_bandingfilter(void)
//{
//	int prev_VTS;
//	int band_step60, max_band60, band_step50, max_band50;
//
//	/* read preview PCLK */
//	prev_sysclk = OV5640_get_sysclk();
//	/* read preview HTS */
//	prev_HTS = OV5640_get_HTS();
//
//	/* read preview VTS */
//	prev_VTS = OV5640_get_VTS();
//
//	/* calculate banding filter */
//	/* 60Hz */
//	band_step60 = prev_sysclk * 100/prev_HTS * 100/120;
//	ov5640_write_reg(0x3a0a, (band_step60 >> 8));
//	ov5640_write_reg(0x3a0b, (band_step60 & 0xff));
//
//	max_band60 = (int)((prev_VTS-4)/band_step60);
//	ov5640_write_reg(0x3a0d, max_band60);
//
//	/* 50Hz */
//	band_step50 = prev_sysclk * 100/prev_HTS;
//	ov5640_write_reg(0x3a08, (band_step50 >> 8));
//	ov5640_write_reg(0x3a09, (band_step50 & 0xff));
//
//	max_band50 = (int)((prev_VTS-4)/band_step50);
//	ov5640_write_reg(0x3a0e, max_band50);
//}
//
//int OV5640_set_AE_target(int target)
//{
//	/* stable in high */
//	int fast_high, fast_low;
//	AE_low = target * 23 / 25;	/* 0.92 */
//	AE_high = target * 27 / 25;	/* 1.08 */
//
//	fast_high = AE_high<<1;
//	if (fast_high > 255)
//		fast_high = 255;
//
//	fast_low = AE_low >> 1;
//
//	ov5640_write_reg(0x3a0f, AE_high);
//	ov5640_write_reg(0x3a10, AE_low);
//	ov5640_write_reg(0x3a1b, AE_high);
//	ov5640_write_reg(0x3a1e, AE_low);
//	ov5640_write_reg(0x3a11, fast_high);
//	ov5640_write_reg(0x3a1f, fast_low);
//
//	return 0;
//}
//
//void OV5640_turn_on_AE_AG(int enable)
//{
//	u8 ae_ag_ctrl;
//
//	ov5640_read_reg(0x3503, &ae_ag_ctrl);
//	if (enable) {
//		/* turn on auto AE/AG */
//		ae_ag_ctrl = ae_ag_ctrl & ~(0x03);
//	} else {
//		/* turn off AE/AG */
//		ae_ag_ctrl = ae_ag_ctrl | 0x03;
//	}
//	ov5640_write_reg(0x3503, ae_ag_ctrl);
//}
//
//bool binning_on(void)
//{
//	u8 temp;
//	ov5640_read_reg(0x3821, &temp);
//	temp &= 0xfe;
//	if (temp)
//		return true;
//	else
//		return false;
//}
//
//static void ov5640_set_virtual_channel(int channel)
//{
//	u8 channel_id;
//
//	ov5640_read_reg(0x4814, &channel_id);
//	channel_id &= ~(3 << 6);
//	ov5640_write_reg(0x4814, channel_id | (channel << 6));
//}
//
///* download ov5640 settings to sensor through i2c */
//static int ov5640_download_firmware(struct reg_value *pModeSetting, s32 ArySize)
//{
//	register u32 Delay_ms = 0;
//	register u16 RegAddr = 0;
//	register u8 Mask = 0;
//	register u8 Val = 0;
//	u8 RegVal = 0;
//	int i, retval = 0;
//
//	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
//		Delay_ms = pModeSetting->u32Delay_ms;
//		RegAddr = pModeSetting->u16RegAddr;
//		Val = pModeSetting->u8Val;
//		Mask = pModeSetting->u8Mask;
//
//		if (Mask) {
//			retval = ov5640_read_reg(RegAddr, &RegVal);
//			if (retval < 0)
//				goto err;
//
//			RegVal &= ~(u8)Mask;
//			Val &= Mask;
//			Val |= RegVal;
//		}
//
//		retval = ov5640_write_reg(RegAddr, Val);
//		if (retval < 0)
//			goto err;
//
//		if (Delay_ms)
//			msleep(Delay_ms);
//	}
//err:
//	return retval;
//}
//
///* sensor changes between scaling and subsampling
// * go through exposure calcualtion
// */
//static int ov5640_change_mode_exposure_calc(enum ov5640_frame_rate frame_rate,
//				enum ov5640_mode mode)
//{
//	struct reg_value *pModeSetting = NULL;
//	s32 ArySize = 0;
//	u8 average;
//	int prev_shutter, prev_gain16;
//	int cap_shutter, cap_gain16;
//	int cap_sysclk, cap_HTS, cap_VTS;
//	int light_freq, cap_bandfilt, cap_maxband;
//	long cap_gain16_shutter;
//	int retval = 0;
//
//	/* check if the input mode and frame rate is valid */
//	pModeSetting =
//		ov5640_mode_info_data[frame_rate][mode].init_data_ptr;
//	ArySize =
//		ov5640_mode_info_data[frame_rate][mode].init_data_size;
//
//	ov5640_data.pix.width =
//		ov5640_mode_info_data[frame_rate][mode].width;
//	ov5640_data.pix.height =
//		ov5640_mode_info_data[frame_rate][mode].height;
//
//	if (ov5640_data.pix.width == 0 || ov5640_data.pix.height == 0 ||
//		pModeSetting == NULL || ArySize == 0)
//		return -EINVAL;
//
//	/* auto focus */
//	/* OV5640_auto_focus();//if no af function, just skip it */
//
//	/* turn off AE/AG */
//	OV5640_turn_on_AE_AG(0);
//
//	/* read preview shutter */
//	prev_shutter = OV5640_get_shutter();
//	if ((binning_on()) && (mode != ov5640_mode_720P_1280_720)
//			&& (mode != ov5640_mode_1080P_1920_1080))
//		prev_shutter *= 2;
//
//	/* read preview gain */
//	prev_gain16 = OV5640_get_gain16();
//
//	/* get average */
//	ov5640_read_reg(0x56a1, &average);
//
//	/* turn off night mode for capture */
//	OV5640_set_night_mode();
//
//	/* turn off overlay */
//	/* ov5640_write_reg(0x3022, 0x06);//if no af function, just skip it */
//
//	OV5640_stream_off();
//
//	/* Write capture setting */
//	retval = ov5640_download_firmware(pModeSetting, ArySize);
//	if (retval < 0)
//		goto err;
//
//	/* read capture VTS */
//	cap_VTS = OV5640_get_VTS();
//	cap_HTS = OV5640_get_HTS();
//	cap_sysclk = OV5640_get_sysclk();
//
//	/* calculate capture banding filter */
//	light_freq = OV5640_get_light_freq();
//	if (light_freq == 60) {
//		/* 60Hz */
//		cap_bandfilt = cap_sysclk * 100 / cap_HTS * 100 / 120;
//	} else {
//		/* 50Hz */
//		cap_bandfilt = cap_sysclk * 100 / cap_HTS;
//	}
//	cap_maxband = (int)((cap_VTS - 4)/cap_bandfilt);
//
//	/* calculate capture shutter/gain16 */
//	if (average > AE_low && average < AE_high) {
//		/* in stable range */
//		cap_gain16_shutter =
//		  prev_gain16 * prev_shutter * cap_sysclk/prev_sysclk
//		  * prev_HTS/cap_HTS * AE_Target / average;
//	} else {
//		cap_gain16_shutter =
//		  prev_gain16 * prev_shutter * cap_sysclk/prev_sysclk
//		  * prev_HTS/cap_HTS;
//	}
//
//	/* gain to shutter */
//	if (cap_gain16_shutter < (cap_bandfilt * 16)) {
//		/* shutter < 1/100 */
//		cap_shutter = cap_gain16_shutter/16;
//		if (cap_shutter < 1)
//			cap_shutter = 1;
//
//		cap_gain16 = cap_gain16_shutter/cap_shutter;
//		if (cap_gain16 < 16)
//			cap_gain16 = 16;
//	} else {
//		if (cap_gain16_shutter >
//				(cap_bandfilt * cap_maxband * 16)) {
//			/* exposure reach max */
//			cap_shutter = cap_bandfilt * cap_maxband;
//			cap_gain16 = cap_gain16_shutter / cap_shutter;
//		} else {
//			/* 1/100 < (cap_shutter = n/100) =< max */
//			cap_shutter =
//			  ((int) (cap_gain16_shutter/16 / cap_bandfilt))
//			  *cap_bandfilt;
//			cap_gain16 = cap_gain16_shutter / cap_shutter;
//		}
//	}
//
//	/* write capture gain */
//	OV5640_set_gain16(cap_gain16);
//
//	/* write capture shutter */
//	if (cap_shutter > (cap_VTS - 4)) {
//		cap_VTS = cap_shutter + 4;
//		OV5640_set_VTS(cap_VTS);
//	}
//	OV5640_set_shutter(cap_shutter);
//
//	OV5640_stream_on();
//
//err:
//	return retval;
//}
//
///* if sensor changes inside scaling or subsampling
// * change mode directly
// * */
//static int ov5640_change_mode_direct(enum ov5640_frame_rate frame_rate,
//				enum ov5640_mode mode)
//{
//	struct reg_value *pModeSetting = NULL;
//	s32 ArySize = 0;
//	int retval = 0;
//
//	/* check if the input mode and frame rate is valid */
//	pModeSetting =
//		ov5640_mode_info_data[frame_rate][mode].init_data_ptr;
//	ArySize =
//		ov5640_mode_info_data[frame_rate][mode].init_data_size;
//
//	ov5640_data.pix.width =
//		ov5640_mode_info_data[frame_rate][mode].width;
//	ov5640_data.pix.height =
//		ov5640_mode_info_data[frame_rate][mode].height;
//
//	if (ov5640_data.pix.width == 0 || ov5640_data.pix.height == 0 ||
//		pModeSetting == NULL || ArySize == 0)
//		return -EINVAL;
//
//	/* turn off AE/AG */
//	OV5640_turn_on_AE_AG(0);
//
//	OV5640_stream_off();
//
//	/* Write capture setting */
//	retval = ov5640_download_firmware(pModeSetting, ArySize);
//	if (retval < 0)
//		goto err;
//
//	OV5640_stream_on();
//
//	OV5640_turn_on_AE_AG(1);
//
//err:
//	return retval;
//}
//
//static int ov5640_init_mode(enum ov5640_frame_rate frame_rate,
//			    enum ov5640_mode mode, enum ov5640_mode orig_mode)
//{
//	struct reg_value *pModeSetting = NULL;
//	s32 ArySize = 0;
//	int retval = 0;
//	void *mipi_csi2_info;
//	u32 mipi_reg, msec_wait4stable = 0;
//	enum ov5640_downsize_mode dn_mode, orig_dn_mode;
//
//	if ((mode > ov5640_mode_MAX || mode < ov5640_mode_MIN)
//		&& (mode != ov5640_mode_INIT)) {
//		pr_err("Wrong ov5640 mode detected!\n");
//		return -1;
//	}
//
//	mipi_csi2_info = mipi_csi2_get_info();
//
//	/* initial mipi dphy */
//	if (!mipi_csi2_info) {
//		printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n",
//		       __func__, __FILE__);
//		return -1;
//	}
//
//	if (!mipi_csi2_get_status(mipi_csi2_info))
//		mipi_csi2_enable(mipi_csi2_info);
//
//	if (!mipi_csi2_get_status(mipi_csi2_info)) {
//		pr_err("Can not enable mipi csi2 driver!\n");
//		return -1;
//	}
//
//	mipi_csi2_set_lanes(mipi_csi2_info);
//
//	/*Only reset MIPI CSI2 HW at sensor initialize*/
//	if (mode == ov5640_mode_INIT)
//		mipi_csi2_reset(mipi_csi2_info);
//
//	if (ov5640_data.pix.pixelformat == V4L2_PIX_FMT_UYVY)
//		mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);
//	else if (ov5640_data.pix.pixelformat == V4L2_PIX_FMT_RGB565)
//		mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_RGB565);
//	else
//		pr_err("currently this sensor format can not be supported!\n");
//
//	dn_mode = ov5640_mode_info_data[frame_rate][mode].dn_mode;
//	orig_dn_mode = ov5640_mode_info_data[frame_rate][orig_mode].dn_mode;
//	if (mode == ov5640_mode_INIT) {
//		pModeSetting = ov5640_init_setting_30fps_VGA;
//		ArySize = ARRAY_SIZE(ov5640_init_setting_30fps_VGA);
//
//		ov5640_data.pix.width = 640;
//		ov5640_data.pix.height = 480;
//		retval = ov5640_download_firmware(pModeSetting, ArySize);
//		if (retval < 0)
//			goto err;
//
//		pModeSetting = ov5640_setting_30fps_VGA_640_480;
//		ArySize = ARRAY_SIZE(ov5640_setting_30fps_VGA_640_480);
//		retval = ov5640_download_firmware(pModeSetting, ArySize);
//	} else if ((dn_mode == SUBSAMPLING && orig_dn_mode == SCALING) ||
//			(dn_mode == SCALING && orig_dn_mode == SUBSAMPLING)) {
//		/* change between subsampling and scaling
//		 * go through exposure calucation */
//		retval = ov5640_change_mode_exposure_calc(frame_rate, mode);
//	} else {
//		/* change inside subsampling or scaling
//		 * download firmware directly */
//		retval = ov5640_change_mode_direct(frame_rate, mode);
//	}
//
//	if (retval < 0)
//		goto err;
//
//	OV5640_set_AE_target(AE_Target);
//	OV5640_get_light_freq();
//	OV5640_set_bandingfilter();
//	ov5640_set_virtual_channel(ov5640_data.csi);
//
//	/* add delay to wait for sensor stable */
//	if (mode == ov5640_mode_QSXGA_2592_1944) {
//		/* dump the first two frames: 1/7.5*2
//		 * the frame rate of QSXGA is 7.5fps */
//		msec_wait4stable = 267;
//	} else if (frame_rate == ov5640_15_fps) {
//		/* dump the first nine frames: 1/15*9 */
//		msec_wait4stable = 600;
//	} else if (frame_rate == ov5640_30_fps) {
//		/* dump the first nine frames: 1/30*9 */
//		msec_wait4stable = 300;
//	}
//	msleep(msec_wait4stable);
//
//	if (mipi_csi2_info) {
//		unsigned int i;
//
//		i = 0;
//
//		/* wait for mipi sensor ready */
//		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
//		while ((mipi_reg == 0x200) && (i < 10)) {
//			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
//			i++;
//			msleep(10);
//		}
//
//		if (i >= 10) {
//			pr_err("mipi csi2 can not receive sensor clk!\n");
//			return -1;
//		}
//
//		i = 0;
//
//		/* wait for mipi stable */
//		mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
//		while ((mipi_reg != 0x0) && (i < 10)) {
//			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
//			i++;
//			msleep(10);
//		}
//
//		if (i >= 10) {
//			pr_err("mipi csi2 can not reveive data correctly!\n");
//			return -1;
//		}
//	}
//err:
//	return retval;
//}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = imx307_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", imx307_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = OV5640_XCLK_MIN;
	p->u.bt656.clock_max = OV5640_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
//static int ioctl_s_power(struct v4l2_int_device *s, int on)
//{
//	struct sensor_data *sensor = s->priv;
//
//	if (on && !sensor->on) {
//		if (io_regulator)
//			if (regulator_enable(io_regulator) != 0)
//				return -EIO;
//		if (core_regulator)
//			if (regulator_enable(core_regulator) != 0)
//				return -EIO;
//		if (gpo_regulator)
//			if (regulator_enable(gpo_regulator) != 0)
//				return -EIO;
//		if (analog_regulator)
//			if (regulator_enable(analog_regulator) != 0)
//				return -EIO;
//		/* Make sure power on */
//		ov5640_standby(0);
//	} else if (!on && sensor->on) {
//		if (analog_regulator)
//			regulator_disable(analog_regulator);
//		if (core_regulator)
//			regulator_disable(core_regulator);
//		if (io_regulator)
//			regulator_disable(io_regulator);
//		if (gpo_regulator)
//			regulator_disable(gpo_regulator);
//
//		ov5640_standby(1);
//	}
//
//	sensor->on = on;
//
//	return 0;
//}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
//static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
//{
//	struct sensor_data *sensor = s->priv;
//	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
//	u32 tgt_fps;	/* target frames per secound */
//	enum ov5640_frame_rate frame_rate;
//	enum ov5640_mode orig_mode;
//	int ret = 0;
//
//	/* Make sure power on */
//	ov5640_standby(0);
//
//	switch (a->type) {
//	/* This is the only case currently handled. */
//	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
//		/* Check that the new frame rate is allowed. */
//		if ((timeperframe->numerator == 0) ||
//		    (timeperframe->denominator == 0)) {
//			timeperframe->denominator = DEFAULT_FPS;
//			timeperframe->numerator = 1;
//		}
//
//		tgt_fps = timeperframe->denominator /
//			  timeperframe->numerator;
//
//		if (tgt_fps > MAX_FPS) {
//			timeperframe->denominator = MAX_FPS;
//			timeperframe->numerator = 1;
//		} else if (tgt_fps < MIN_FPS) {
//			timeperframe->denominator = MIN_FPS;
//			timeperframe->numerator = 1;
//		}
//
//		/* Actual frame rate we use */
//		tgt_fps = timeperframe->denominator /
//			  timeperframe->numerator;
//
//		if (tgt_fps == 15)
//			frame_rate = ov5640_15_fps;
//		else if (tgt_fps == 30)
//			frame_rate = ov5640_30_fps;
//		else {
//			pr_err(" The camera frame rate is not supported!\n");
//			return -EINVAL;
//		}
//
//		orig_mode = sensor->streamcap.capturemode;
//		ret = ov5640_init_mode(frame_rate,
//				(u32)a->parm.capture.capturemode, orig_mode);
//		if (ret < 0)
//			return ret;
//
//		sensor->streamcap.timeperframe = *timeperframe;
//		sensor->streamcap.capturemode =
//				(u32)a->parm.capture.capturemode;
//
//		break;
//
//	/* These are all the possible cases. */
//	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
//	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
//	case V4L2_BUF_TYPE_VBI_CAPTURE:
//	case V4L2_BUF_TYPE_VBI_OUTPUT:
//	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
//	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
//		pr_debug("   type is not " \
//			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
//			a->type);
//		ret = -EINVAL;
//		break;
//
//	default:
//		pr_debug("   type is unknown - %d\n", a->type);
//		ret = -EINVAL;
//		break;
//	}
//
//	return ret;
//}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = imx307_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = imx307_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = imx307_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = imx307_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = imx307_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = imx307_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = imx307_data.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;

	pr_debug("In ov5640:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
//static int ioctl_enum_framesizes(struct v4l2_int_device *s,
//				 struct v4l2_frmsizeenum *fsize)
//{
//	if (fsize->index > ov5640_mode_MAX)
//		return -EINVAL;
//
//	fsize->pixel_format = ov5640_data.pix.pixelformat;
//	fsize->discrete.width =
//			max(ov5640_mode_info_data[0][fsize->index].width,
//			    ov5640_mode_info_data[1][fsize->index].width);
//	fsize->discrete.height =
//			max(ov5640_mode_info_data[0][fsize->index].height,
//			    ov5640_mode_info_data[1][fsize->index].height);
//	return 0;
//}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
//static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
//					 struct v4l2_frmivalenum *fival)
//{
//	int i, j, count = 0;
//
//	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
//	fival->discrete.numerator = 1;
//
//	for (i = 0; i < ARRAY_SIZE(ov5640_mode_info_data); i++)
//		for (j = 0; j < (ov5640_mode_MAX + 1); j++)
//			if (fival->pixel_format == ov5640_data.pix.pixelformat
//			 && fival->width == ov5640_mode_info_data[i][j].width
//			 && fival->height == ov5640_mode_info_data[i][j].height
//			 && ov5640_mode_info_data[i][j].init_data_ptr != NULL
//			 && fival->index == count++) {
//				fival->discrete.denominator =
//						ov5640_framerates[i];
//				return 0;
//			}
//
//	return -EINVAL;
//}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
//static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
//{
//	((struct v4l2_dbg_chip_ident *)id)->match.type =
//					V4L2_CHIP_MATCH_I2C_DRIVER;
//	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
//		"ov5640_mipi_camera");
//
//	return 0;
//}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
//static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
//			      struct v4l2_fmtdesc *fmt)
//{
//	if (fmt->index > ov5640_mode_MAX)
//		return -EINVAL;
//
//	fmt->pixelformat = ov5640_data.pix.pixelformat;
//
//	return 0;
//}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
//static int ioctl_dev_init(struct v4l2_int_device *s)
//{
//	struct sensor_data *sensor = s->priv;
//	u32 tgt_xclk;	/* target xclk */
//	u32 tgt_fps;	/* target frames per secound */
//	int ret;
//	enum ov5640_frame_rate frame_rate;
//	void *mipi_csi2_info;
//
//	ov5640_data.on = true;
//
//	/* mclk */
//	tgt_xclk = ov5640_data.mclk;
//	tgt_xclk = min(tgt_xclk, (u32)OV5640_XCLK_MAX);
//	tgt_xclk = max(tgt_xclk, (u32)OV5640_XCLK_MIN);
//	ov5640_data.mclk = tgt_xclk;
//
//	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
//
//	/* Default camera frame rate is set in probe */
//	tgt_fps = sensor->streamcap.timeperframe.denominator /
//		  sensor->streamcap.timeperframe.numerator;
//
//	if (tgt_fps == 15)
//		frame_rate = ov5640_15_fps;
//	else if (tgt_fps == 30)
//		frame_rate = ov5640_30_fps;
//	else
//		return -EINVAL; /* Only support 15fps or 30fps now. */
//
//	mipi_csi2_info = mipi_csi2_get_info();
//
//	/* enable mipi csi2 */
//	if (mipi_csi2_info)
//		mipi_csi2_enable(mipi_csi2_info);
//	else {
//		printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n",
//		       __func__, __FILE__);
//		return -EPERM;
//	}
//
//	ret = ov5640_init_mode(frame_rate, ov5640_mode_INIT, ov5640_mode_INIT);
//
//	return ret;
//}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
//static int ioctl_dev_exit(struct v4l2_int_device *s)
//{
//	void *mipi_csi2_info;
//
//	mipi_csi2_info = mipi_csi2_get_info();
//
//	/* disable mipi csi2 */
//	if (mipi_csi2_info)
//		if (mipi_csi2_get_status(mipi_csi2_info))
//			mipi_csi2_disable(mipi_csi2_info);
//
//	return 0;
//}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
//static struct v4l2_int_ioctl_desc ov5640_ioctl_desc[] = {
//	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *) ioctl_dev_init},
//	{vidioc_int_dev_exit_num, ioctl_dev_exit},
//	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *) ioctl_s_power},
//	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *) ioctl_g_ifparm},
///*	{vidioc_int_g_needs_reset_num,
//				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
///*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
//	{vidioc_int_init_num, (v4l2_int_ioctl_func *) ioctl_init},
//	{vidioc_int_enum_fmt_cap_num,
//				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
///*	{vidioc_int_try_fmt_cap_num,
//				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
//	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
///*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_s_fmt_cap}, */
//	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
//	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
///*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
//	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
//	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
//	{vidioc_int_enum_framesizes_num,
//				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
//	{vidioc_int_enum_frameintervals_num,
//			(v4l2_int_ioctl_func *) ioctl_enum_frameintervals},
//	{vidioc_int_g_chip_ident_num,
//				(v4l2_int_ioctl_func *) ioctl_g_chip_ident},
//};
//
//static struct v4l2_int_slave ov5640_slave = {
//	.ioctls = ov5640_ioctl_desc,
//	.num_ioctls = ARRAY_SIZE(ov5640_ioctl_desc),
//};
//
//static struct v4l2_int_device ov5640_int_device = {
//	.module = THIS_MODULE,
//	.name = "ov5640",
//	.type = v4l2_int_type_slave,
//	.u = {
//		.slave = &ov5640_slave,
//	},
//};

/* Verify chip ID */
static int imx307_identify_module(void)
{
	int ret;
	u32 val;
	u8  zval = 0;

	printk("zty imx307 id 0x%x!\n", imx307_data.i2c_client->addr);
	ret = imx291_read(0x300A, &zval);
//	if (ret) {
		dev_err(&imx307_data.i2c_client->dev, "failed to read chip id %x\n",
				imx307_CHIP_ID);
//		return ret;
//	}
	printk("zty read imx307 chip id 0x%x!\n", zval);
//	ret = nimx307_read_reg(IMX291_CHIP_ID_H,
//			      imx307_REG_VALUE_16BIT, &val);
//	if (ret) {
//		dev_err(&imx307_data.i2c_client->dev, "failed to read chip id %x\n",
//				imx307_CHIP_ID);
//		return ret;
//	}
//	printk("zty read imx307 chip id 0x%x!\n", val);
//
//	if (val != imx307_CHIP_ID) {
//		dev_err(&imx307_data.i2c_client->dev, "chip id mismatch: %x!=%x\n",
//			imx307_CHIP_ID, val);
//		return -EIO;
//	}

	return 0;
}

/*!
 * ov5640 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int imx307_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high, chip_id_low;

	printk("zty imx307 probe start!\n");
	/* request power down pin */
//	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
//	if (!gpio_is_valid(pwn_gpio)) {
//		dev_warn(dev, "no sensor pwdn pin available");
//		return -EINVAL;
//	}
//	retval = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,
//					"im307_mipi_pwdn");
//	if (retval < 0)
//		return retval;
//
//	/* request reset pin */
//	rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
//	if (!gpio_is_valid(rst_gpio)) {
//		dev_warn(dev, "no sensor reset pin available");
//		return -EINVAL;
//	}
//	retval = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH,
//					"imx307_mipi_reset");
//	if (retval < 0)
//		return retval;
//
//	/* Set initial values for the sensor struct. */
//	memset(&imx307_data, 0, sizeof(imx307_data));
//	imx307_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
//	if (IS_ERR(imx307_data.sensor_clk)) {
//		/* assuming clock enabled by default */
//		imx307_data.sensor_clk = NULL;
//		printk("zty imx307 mclk missing!\n");
//		dev_err(dev, "clock-frequency missing or invalid\n");
//		return PTR_ERR(imx307_data.sensor_clk);
//	}
//
//	retval = of_property_read_u32(dev->of_node, "mclk",
//					&(imx307_data.mclk));
//	if (retval) {
//		dev_err(dev, "mclk missing or invalid\n");
//		return retval;
//	}
//
//	retval = of_property_read_u32(dev->of_node, "mclk_source",
//					(u32 *) &(imx307_data.mclk_source));
//	if (retval) {
//		dev_err(dev, "mclk_source missing or invalid\n");
//		return retval;
//	}
//
//	retval = of_property_read_u32(dev->of_node, "csi_id",
//					&(imx307_data.csi));
//	if (retval) {
//		dev_err(dev, "csi id missing or invalid\n");
//		return retval;
//	}
//
//	clk_prepare_enable(imx307_data.sensor_clk);
//
//	imx307_data.io_init = imx307_reset;
	imx307_data.i2c_client = client;
//	imx307_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
//	imx307_data.pix.width = 640;
//	imx307_data.pix.height = 480;
//	imx307_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
//					   V4L2_CAP_TIMEPERFRAME;
//	imx307_data.streamcap.capturemode = 0;
//	imx307_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
//	imx307_data.streamcap.timeperframe.numerator = 1;

//	ov5640_power_on(dev);
//
//	ov5640_reset();
//
//	ov5640_standby(0);

	if(imx307_identify_module() != 0)
		return -ENODEV;

//	ov5640_standby(1);

//	ov5640_int_device.priv = &imx307_data;
//	retval = v4l2_int_device_register(&ov5640_int_device);

//	clk_disable_unprepare(imx307_data.sensor_clk);

	pr_info("camera ov5640_mipi is found\n");
	return retval;
}

/*!
 * ov5640 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int imx307_remove(struct i2c_client *client)
{
//	v4l2_int_device_unregister(&imx307_int_device);

	if (gpo_regulator)
		regulator_disable(gpo_regulator);

	if (analog_regulator)
		regulator_disable(analog_regulator);

	if (core_regulator)
		regulator_disable(core_regulator);

	if (io_regulator)
		regulator_disable(io_regulator);

	return 0;
}

/*!
 * imx307 init function
 * Called by insmod ov5640_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int imx307_init(void)
{
	u8 err;

	err = i2c_add_driver(&imx307_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * OV5640 cleanup function
 * Called on rmmod ov5640_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit imx307_clean(void)
{
	i2c_del_driver(&imx307_i2c_driver);
}

module_init(imx307_init);
module_exit(imx307_clean);

MODULE_AUTHOR("HNDZ, Inc.");
MODULE_DESCRIPTION("IMX307 MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
