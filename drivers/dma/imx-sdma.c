/*
 * drivers/dma/imx-sdma.c
 *
 * This file contains a driver for the Freescale Smart DMA engine
 *
 * Copyright 2010 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 *
 * Based on code from Freescale:
 *
 * Copyright 2004-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/genalloc.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>

#include <asm/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/platform_data/dma-imx-sdma.h>
#include <linux/platform_data/dma-imx.h>

#include "dmaengine.h"

/* SDMA registers */
#define SDMA_H_C0PTR		0x000
#define SDMA_H_INTR		0x004
#define SDMA_H_STATSTOP		0x008
#define SDMA_H_START		0x00c
#define SDMA_H_EVTOVR		0x010
#define SDMA_H_DSPOVR		0x014
#define SDMA_H_HOSTOVR		0x018
#define SDMA_H_EVTPEND		0x01c
#define SDMA_H_DSPENBL		0x020
#define SDMA_H_RESET		0x024
#define SDMA_H_EVTERR		0x028
#define SDMA_H_INTRMSK		0x02c
#define SDMA_H_PSW		0x030
#define SDMA_H_EVTERRDBG	0x034
#define SDMA_H_CONFIG		0x038
#define SDMA_ONCE_ENB		0x040
#define SDMA_ONCE_DATA		0x044
#define SDMA_ONCE_INSTR		0x048
#define SDMA_ONCE_STAT		0x04c
#define SDMA_ONCE_CMD		0x050
#define SDMA_EVT_MIRROR		0x054
#define SDMA_ILLINSTADDR	0x058
#define SDMA_CHN0ADDR		0x05c
#define SDMA_ONCE_RTB		0x060
#define SDMA_XTRIG_CONF1	0x070
#define SDMA_XTRIG_CONF2	0x074
#define SDMA_CHNENBL0_IMX35	0x200
#define SDMA_CHNENBL0_IMX31	0x080
#define SDMA_CHNPRI_0		0x100

/*
 * Buffer descriptor status values.
 */
#define BD_DONE  0x01
#define BD_WRAP  0x02
#define BD_CONT  0x04
#define BD_INTR  0x08
#define BD_RROR  0x10
#define BD_LAST  0x20
#define BD_EXTD  0x80

/*
 * Data Node descriptor status values.
 */
#define DND_END_OF_FRAME  0x80
#define DND_END_OF_XFER   0x40
#define DND_DONE          0x20
#define DND_UNUSED        0x01

/*
 * IPCV2 descriptor status values.
 */
#define BD_IPCV2_END_OF_FRAME  0x40

#define IPCV2_MAX_NODES        50
/*
 * Error bit set in the CCB status field by the SDMA,
 * in setbd routine, in case of a transfer error
 */
#define DATA_ERROR  0x10000000

/*
 * Buffer descriptor commands.
 */
#define C0_ADDR             0x01
#define C0_LOAD             0x02
#define C0_DUMP             0x03
#define C0_SETCTX           0x07
#define C0_GETCTX           0x03
#define C0_SETDM            0x01
#define C0_SETPM            0x04
#define C0_GETDM            0x02
#define C0_GETPM            0x08
/*
 * Change endianness indicator in the BD command field
 */
#define CHANGE_ENDIANNESS   0x80

/*
 * Mode/Count of data node descriptors - IPCv2
 */
struct sdma_mode_count {
	u32 count   : 16; /* size of the buffer pointed by this BD */
	u32 status  :  8; /* E,R,I,C,W,D status bits stored here */
	u32 command :  8; /* command mostlky used for channel 0 */
};

/*
 * Buffer descriptor
 */
struct sdma_buffer_descriptor {
	struct sdma_mode_count  mode;
	u32 buffer_addr;	/* address of the buffer described */
	u32 ext_buffer_addr;	/* extended buffer address */
} __attribute__ ((packed));

/**
 * struct sdma_channel_control - Channel control Block
 *
 * @current_bd_ptr	current buffer descriptor processed
 * @base_bd_ptr		first element of buffer descriptor array
 * @unused		padding. The SDMA engine expects an array of 128 byte
 *			control blocks
 */
struct sdma_channel_control {
	u32 current_bd_ptr;
	u32 base_bd_ptr;
	u32 unused[2];
} __attribute__ ((packed));

/**
 * struct sdma_state_registers - SDMA context for a channel
 *
 * @pc:		program counter
 * @t:		test bit: status of arithmetic & test instruction
 * @rpc:	return program counter
 * @sf:		source fault while loading data
 * @spc:	loop start program counter
 * @df:		destination fault while storing data
 * @epc:	loop end program counter
 * @lm:		loop mode
 */
struct sdma_state_registers {
	u32 pc     :14;
	u32 unused1: 1;
	u32 t      : 1;
	u32 rpc    :14;
	u32 unused0: 1;
	u32 sf     : 1;
	u32 spc    :14;
	u32 unused2: 1;
	u32 df     : 1;
	u32 epc    :14;
	u32 lm     : 2;
} __attribute__ ((packed));

/**
 * struct sdma_context_data - sdma context specific to a channel
 *
 * @channel_state:	channel state bits
 * @gReg:		general registers
 * @mda:		burst dma destination address register
 * @msa:		burst dma source address register
 * @ms:			burst dma status register
 * @md:			burst dma data register
 * @pda:		peripheral dma destination address register
 * @psa:		peripheral dma source address register
 * @ps:			peripheral dma status register
 * @pd:			peripheral dma data register
 * @ca:			CRC polynomial register
 * @cs:			CRC accumulator register
 * @dda:		dedicated core destination address register
 * @dsa:		dedicated core source address register
 * @ds:			dedicated core status register
 * @dd:			dedicated core data register
 */
struct sdma_context_data {
	struct sdma_state_registers  channel_state;
	u32  gReg[8];
	u32  mda;
	u32  msa;
	u32  ms;
	u32  md;
	u32  pda;
	u32  psa;
	u32  ps;
	u32  pd;
	u32  ca;
	u32  cs;
	u32  dda;
	u32  dsa;
	u32  ds;
	u32  dd;
	u32  scratch0;
	u32  scratch1;
	u32  scratch2;
	u32  scratch3;
	u32  scratch4;
	u32  scratch5;
	u32  scratch6;
	u32  scratch7;
} __attribute__ ((packed));

#define NUM_BD (int)(PAGE_SIZE / sizeof(struct sdma_buffer_descriptor))
#define SDMA_BD_MAX_CNT	0xfffc /* align with 4 bytes */

struct sdma_engine;

/**
 * struct sdma_channel - housekeeping for a SDMA channel
 *
 * @sdma		pointer to the SDMA engine for this channel
 * @channel		the channel number, matches dmaengine chan_id + 1
 * @direction		transfer type. Needed for setting SDMA script
 * @peripheral_type	Peripheral type. Needed for setting SDMA script
 * @event_id0		aka dma request line
 * @event_id1		for channels that use 2 events
 * @word_size		peripheral access size
 * @buf_tail		ID of the buffer that was processed
 * @num_bd		max NUM_BD. number of descriptors currently handling
 * @bd_iram		flag indicating the memory location of buffer descriptor
 */
struct sdma_channel {
	struct sdma_engine		*sdma;
	unsigned int			channel;
	enum dma_transfer_direction		direction;
	enum sdma_peripheral_type	peripheral_type;
	unsigned int			event_id0;
	unsigned int			event_id1;
	enum dma_slave_buswidth		word_size;
	unsigned int			buf_tail;
	unsigned int			num_bd;
	unsigned int			period_len;
	struct sdma_buffer_descriptor	*bd;
	dma_addr_t			bd_phys;
	bool				bd_iram;
	unsigned int			pc_from_device, pc_to_device;
	unsigned int			device_to_device;
	unsigned int                    pc_to_pc;
	unsigned long			flags;
	dma_addr_t			per_address, per_address2;
	unsigned long			event_mask[2];
	unsigned long			watermark_level;
	u32				shp_addr, per_addr;
	struct dma_chan			chan;
	spinlock_t			lock;
	struct dma_async_tx_descriptor	desc;
	enum dma_status			status;
	unsigned int			chn_count;
	unsigned int			chn_real_count;
	struct tasklet_struct		tasklet;
	struct imx_dma_data		data;
};

#define IMX_DMA_SG_LOOP		BIT(0)

#define MAX_DMA_CHANNELS 32
#define MXC_SDMA_DEFAULT_PRIORITY 1
#define MXC_SDMA_MIN_PRIORITY 1
#define MXC_SDMA_MAX_PRIORITY 7
/*
 * 0x78(SDMA_XTRIG_CONF2+4)~0x100(SDMA_CHNPRI_O) registers are reserved and
 * can't be accessed. Skip these register touch in suspend/resume. Also below
 * two macros are only used on i.mx6sx.
 */
#define MXC_SDMA_RESERVED_REG (SDMA_CHNPRI_0 - SDMA_XTRIG_CONF2 - 4)
#define MXC_SDMA_SAVED_REG_NUM (((SDMA_CHNENBL0_IMX35 + 4 * 48) - \
				MXC_SDMA_RESERVED_REG) / 4)

#define SDMA_FIRMWARE_MAGIC 0x414d4453

/**
 * struct sdma_firmware_header - Layout of the firmware image
 *
 * @magic		"SDMA"
 * @version_major	increased whenever layout of struct sdma_script_start_addrs
 *			changes.
 * @version_minor	firmware minor version (for binary compatible changes)
 * @script_addrs_start	offset of struct sdma_script_start_addrs in this image
 * @num_script_addrs	Number of script addresses in this image
 * @ram_code_start	offset of SDMA ram image in this firmware image
 * @ram_code_size	size of SDMA ram image
 * @script_addrs	Stores the start address of the SDMA scripts
 *			(in SDMA memory space)
 */
struct sdma_firmware_header {
	u32	magic;
	u32	version_major;
	u32	version_minor;
	u32	script_addrs_start;
	u32	num_script_addrs;
	u32	ram_code_start;
	u32	ram_code_size;
};

struct sdma_driver_data {
	int chnenbl0;
	int num_events;
	struct sdma_script_start_addrs	*script_addrs;
};

struct sdma_engine {
	struct device			*dev;
	struct device_dma_parameters	dma_parms;
	struct sdma_channel		channel[MAX_DMA_CHANNELS];
	struct sdma_channel_control	*channel_control;
	u32				save_regs[MXC_SDMA_SAVED_REG_NUM];
	const char			*fw_name;
	void __iomem			*regs;
	struct sdma_context_data	*context;
	dma_addr_t			context_phys;
	struct dma_device		dma_device;
	struct clk			*clk_ipg;
	struct clk			*clk_ahb;
	spinlock_t			channel_0_lock;
	u32				script_number;
	struct sdma_script_start_addrs	*script_addrs;
	const struct sdma_driver_data	*drvdata;
	struct gen_pool 		*iram_pool;
};

static struct sdma_driver_data sdma_imx31 = {
	.chnenbl0 = SDMA_CHNENBL0_IMX31,
	.num_events = 32,
};

static struct sdma_script_start_addrs sdma_script_imx25 = {
	.ap_2_ap_addr = 729,
	.uart_2_mcu_addr = 904,
	.per_2_app_addr = 1255,
	.mcu_2_app_addr = 834,
	.uartsh_2_mcu_addr = 1120,
	.per_2_shp_addr = 1329,
	.mcu_2_shp_addr = 1048,
	.ata_2_mcu_addr = 1560,
	.mcu_2_ata_addr = 1479,
	.app_2_per_addr = 1189,
	.app_2_mcu_addr = 770,
	.shp_2_per_addr = 1407,
	.shp_2_mcu_addr = 979,
};

static struct sdma_driver_data sdma_imx25 = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx25,
};

static struct sdma_driver_data sdma_imx35 = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
};

static struct sdma_script_start_addrs sdma_script_imx51 = {
	.ap_2_ap_addr = 642,
	.uart_2_mcu_addr = 817,
	.mcu_2_app_addr = 747,
	.mcu_2_shp_addr = 961,
	.ata_2_mcu_addr = 1473,
	.mcu_2_ata_addr = 1392,
	.app_2_per_addr = 1033,
	.app_2_mcu_addr = 683,
	.shp_2_per_addr = 1251,
	.shp_2_mcu_addr = 892,
};

static struct sdma_driver_data sdma_imx51 = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx51,
};

static struct sdma_script_start_addrs sdma_script_imx53 = {
	.ap_2_ap_addr = 642,
	.app_2_mcu_addr = 683,
	.mcu_2_app_addr = 747,
	.uart_2_mcu_addr = 817,
	.shp_2_mcu_addr = 891,
	.mcu_2_shp_addr = 960,
	.uartsh_2_mcu_addr = 1032,
	.spdif_2_mcu_addr = 1100,
	.mcu_2_spdif_addr = 1134,
	.firi_2_mcu_addr = 1193,
	.mcu_2_firi_addr = 1290,
};

static struct sdma_driver_data sdma_imx53 = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx53,
};

static struct sdma_script_start_addrs sdma_script_imx6q = {
	.ap_2_ap_addr = 642,
	.uart_2_mcu_addr = 817,
	.mcu_2_app_addr = 747,
	.uartsh_2_mcu_addr = 1032,
	.mcu_2_shp_addr = 960,
	.app_2_mcu_addr = 683,
	.shp_2_mcu_addr = 891,
	.spdif_2_mcu_addr = 1100,
	.mcu_2_spdif_addr = 1134,
};

static struct sdma_driver_data sdma_imx6q = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx6q,
};

static struct sdma_script_start_addrs sdma_script_imx6sx = {
	.ap_2_ap_addr = 642,
	.uart_2_mcu_addr = 817,
	.mcu_2_app_addr = 747,
	.uartsh_2_mcu_addr = 1032,
	.mcu_2_shp_addr = 960,
	.app_2_mcu_addr = 683,
	.shp_2_mcu_addr = 891,
	.spdif_2_mcu_addr = 1100,
	.mcu_2_spdif_addr = 1134,
};

static struct sdma_driver_data sdma_imx6sx = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx6sx,
};

static struct platform_device_id sdma_devtypes[] = {
	{
		.name = "imx25-sdma",
		.driver_data = (unsigned long)&sdma_imx25,
	}, {
		.name = "imx31-sdma",
		.driver_data = (unsigned long)&sdma_imx31,
	}, {
		.name = "imx35-sdma",
		.driver_data = (unsigned long)&sdma_imx35,
	}, {
		.name = "imx51-sdma",
		.driver_data = (unsigned long)&sdma_imx51,
	}, {
		.name = "imx53-sdma",
		.driver_data = (unsigned long)&sdma_imx53,
	}, {
		.name = "imx6q-sdma",
		.driver_data = (unsigned long)&sdma_imx6q,
	}, {
		.name = "imx6sx-sdma",
		.driver_data = (unsigned long)&sdma_imx6sx,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, sdma_devtypes);

static const struct of_device_id sdma_dt_ids[] = {
	{ .compatible = "fsl,imx6sx-sdma", .data = &sdma_imx6sx, },
	{ .compatible = "fsl,imx6q-sdma", .data = &sdma_imx6q, },
	{ .compatible = "fsl,imx53-sdma", .data = &sdma_imx53, },
	{ .compatible = "fsl,imx51-sdma", .data = &sdma_imx51, },
	{ .compatible = "fsl,imx35-sdma", .data = &sdma_imx35, },
	{ .compatible = "fsl,imx31-sdma", .data = &sdma_imx31, },
	{ .compatible = "fsl,imx25-sdma", .data = &sdma_imx25, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sdma_dt_ids);

#define SDMA_H_CONFIG_DSPDMA	BIT(12) /* indicates if the DSPDMA is used */
#define SDMA_H_CONFIG_RTD_PINS	BIT(11) /* indicates if Real-Time Debug pins are enabled */
#define SDMA_H_CONFIG_ACR	BIT(4)  /* indicates if AHB freq /core freq = 2 or 1 */
#define SDMA_H_CONFIG_CSM	(3)       /* indicates which context switch mode is selected*/

static inline u32 chnenbl_ofs(struct sdma_engine *sdma, unsigned int event)
{
	u32 chnenbl0 = sdma->drvdata->chnenbl0;
	return chnenbl0 + event * 4;
}

static int sdma_config_ownership(struct sdma_channel *sdmac,
		bool event_override, bool mcu_override, bool dsp_override)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	unsigned long evt, mcu, dsp;

	if (event_override && mcu_override && dsp_override)
		return -EINVAL;

	evt = readl_relaxed(sdma->regs + SDMA_H_EVTOVR);
	mcu = readl_relaxed(sdma->regs + SDMA_H_HOSTOVR);
	dsp = readl_relaxed(sdma->regs + SDMA_H_DSPOVR);

	if (dsp_override)
		__clear_bit(channel, &dsp);
	else
		__set_bit(channel, &dsp);

	if (event_override)
		__clear_bit(channel, &evt);
	else
		__set_bit(channel, &evt);

	if (mcu_override)
		__clear_bit(channel, &mcu);
	else
		__set_bit(channel, &mcu);

	writel_relaxed(evt, sdma->regs + SDMA_H_EVTOVR);
	writel_relaxed(mcu, sdma->regs + SDMA_H_HOSTOVR);
	writel_relaxed(dsp, sdma->regs + SDMA_H_DSPOVR);

	return 0;
}

static void sdma_enable_channel(struct sdma_engine *sdma, int channel)
{
	writel(BIT(channel), sdma->regs + SDMA_H_START);
}

/*
 * sdma_run_channel0 - run a channel and wait till it's done
 */
static int sdma_run_channel0(struct sdma_engine *sdma)
{
	int ret;
	unsigned long timeout = 500;

	sdma_enable_channel(sdma, 0);

	while (!(ret = readl_relaxed(sdma->regs + SDMA_H_INTR) & 1)) {
		if (timeout-- <= 0)
			break;
		udelay(1);
	}

	if (ret) {
		/* Clear the interrupt status */
		writel_relaxed(ret, sdma->regs + SDMA_H_INTR);
	} else {
		dev_err(sdma->dev, "Timeout waiting for CH0 ready\n");
	}

	return ret ? 0 : -ETIMEDOUT;
}

static int sdma_load_script(struct sdma_engine *sdma, void *buf, int size,
		u32 address)
{
	struct sdma_buffer_descriptor *bd0 = sdma->channel[0].bd;
	void *buf_virt;
	dma_addr_t buf_phys;
	int ret;
	unsigned long flags;
	bool use_iram = true;

	buf_virt = gen_pool_dma_alloc(sdma->iram_pool, size, &buf_phys);
	if (!buf_virt) {
		use_iram = false;
		buf_virt = dma_alloc_coherent(NULL, size, &buf_phys, GFP_KERNEL);
		if (!buf_virt)
			return -ENOMEM;
	}

	spin_lock_irqsave(&sdma->channel_0_lock, flags);

	bd0->mode.command = C0_SETPM;
	bd0->mode.status = BD_DONE | BD_INTR | BD_WRAP | BD_EXTD;
	bd0->mode.count = size / 2;
	bd0->buffer_addr = buf_phys;
	bd0->ext_buffer_addr = address;

	memcpy(buf_virt, buf, size);

	ret = sdma_run_channel0(sdma);

	spin_unlock_irqrestore(&sdma->channel_0_lock, flags);

	if (use_iram)
		gen_pool_free(sdma->iram_pool, (unsigned long)buf_virt, size);
	else
		dma_free_coherent(NULL, size, buf_virt, buf_phys);

	return ret;
}

static void sdma_event_enable(struct sdma_channel *sdmac, unsigned int event)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	unsigned long val;
	u32 chnenbl = chnenbl_ofs(sdma, event);

	val = readl_relaxed(sdma->regs + chnenbl);
	__set_bit(channel, &val);
	writel_relaxed(val, sdma->regs + chnenbl);
}

static void sdma_event_disable(struct sdma_channel *sdmac, unsigned int event)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	u32 chnenbl = chnenbl_ofs(sdma, event);
	unsigned long val;

	val = readl_relaxed(sdma->regs + chnenbl);
	__clear_bit(channel, &val);
	writel_relaxed(val, sdma->regs + chnenbl);
}

static void sdma_handle_channel_loop(struct sdma_channel *sdmac)
{
	if (sdmac->desc.callback)
	{
		sdmac->desc.callback(sdmac->desc.callback_param);
		// printk("hndz callback %pF channel %d event_id0 %d!\n", sdmac->desc.callback, sdmac->channel, sdmac->event_id0);
	}
}

static void sdma_update_channel_loop(struct sdma_channel *sdmac)
{
	struct sdma_buffer_descriptor *bd;

	/*
	 * loop mode. Iterate over descriptors, re-setup them and
	 * call callback function.
	 */
	while (1) {
		bd = &sdmac->bd[sdmac->buf_tail];

		if (bd->mode.status & BD_DONE)
			break;

		if (bd->mode.status & BD_RROR)
			sdmac->status = DMA_ERROR;

		bd->mode.status |= BD_DONE;
		sdmac->buf_tail++;
		sdmac->buf_tail %= sdmac->num_bd;
		if (sdmac->peripheral_type == IMX_DMATYPE_UART) {
			/* restore mode.count after counter readed */
			sdmac->chn_real_count = bd->mode.count;
			bd->mode.count = sdmac->chn_count;
		}
	}
}

static void mxc_sdma_handle_channel_normal(struct sdma_channel *sdmac)
{
	struct sdma_buffer_descriptor *bd;
	int i, error = 0;

	sdmac->chn_real_count = 0;
	/*
	 * non loop mode. Iterate over all descriptors, collect
	 * errors and call callback function
	 */
	for (i = 0; i < sdmac->num_bd; i++) {
		bd = &sdmac->bd[i];

		 if (bd->mode.status & (BD_DONE | BD_RROR))
			error = -EIO;
		 sdmac->chn_real_count += bd->mode.count;
	}

	if (error)
		sdmac->status = DMA_ERROR;
	else
		sdmac->status = DMA_COMPLETE;

	dma_cookie_complete(&sdmac->desc);
	if (sdmac->desc.callback)
	{
		sdmac->desc.callback(sdmac->desc.callback_param);
		// if(sdmac->event_id0 == 5)
		// 	printk("hndz callback %pF channel %d event_id0 %d!\n", sdmac->desc.callback, sdmac->channel, sdmac->event_id0);
	}
}

static void sdma_tasklet(unsigned long data)
{
	struct sdma_channel *sdmac = (struct sdma_channel *) data;
	unsigned long flags;

	spin_lock_irqsave(&sdmac->lock, flags);
	if (sdmac->status != DMA_IN_PROGRESS && !(sdmac->flags & IMX_DMA_SG_LOOP)) {
		spin_unlock_irqrestore(&sdmac->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&sdmac->lock, flags);

	if (sdmac->flags & IMX_DMA_SG_LOOP)
		sdma_handle_channel_loop(sdmac);
	else
		mxc_sdma_handle_channel_normal(sdmac);
}

// extern void spi_imx_irq_rx_callback(void *cookie);
// extern void spi_imx_irq_tx_callback(void *cookie);
static irqreturn_t sdma_int_handler(int irq, void *dev_id)
{
	struct sdma_engine *sdma = dev_id;
	unsigned long stat, flags;

	stat = readl_relaxed(sdma->regs + SDMA_H_INTR);
	/* not interested in channel 0 interrupts */
	stat &= ~1;
	writel_relaxed(stat, sdma->regs + SDMA_H_INTR);

	while (stat) {
		int channel = fls(stat) - 1;
		struct sdma_channel *sdmac = &sdma->channel[channel];

		if ((sdmac->flags & IMX_DMA_SG_LOOP) &&
			(sdmac->peripheral_type != IMX_DMATYPE_HDMI))
			sdma_update_channel_loop(sdmac);

		spin_lock_irqsave(&sdmac->lock, flags);
		if (sdmac->status == DMA_IN_PROGRESS || (sdmac->flags & IMX_DMA_SG_LOOP))
		{
			// if(sdmac->event_id0 == 5)
			// {
			// 	// sdmac->status = DMA_COMPLETE;
			// 	spi_imx_irq_rx_callback(sdmac->desc.callback_param);
			// }
			// else if(sdmac->event_id0 == 6)
			// {
			// 	// sdmac->status = DMA_COMPLETE;
			// 	spi_imx_irq_tx_callback(sdmac->desc.callback_param);
			// }
			tasklet_schedule(&sdmac->tasklet);
		}
		spin_unlock_irqrestore(&sdmac->lock, flags);

		__clear_bit(channel, &stat);
	}

	return IRQ_HANDLED;
}

/*
 * sets the pc of SDMA script according to the peripheral type
 */
static void sdma_get_pc(struct sdma_channel *sdmac,
		enum sdma_peripheral_type peripheral_type)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int per_2_emi = 0, emi_2_per = 0;
	/*
	 * These are needed once we start to support transfers between
	 * two peripherals or memory-to-memory transfers
	 */
	int per_2_per = 0, emi_2_emi = 0;

	sdmac->pc_from_device = 0;
	sdmac->pc_to_device = 0;
	sdmac->device_to_device = 0;
	sdmac->pc_to_pc = 0;

	switch (peripheral_type) {
	case IMX_DMATYPE_MEMORY:
		emi_2_emi = sdma->script_addrs->ap_2_ap_addr;
		break;
	case IMX_DMATYPE_DSP:
		emi_2_per = sdma->script_addrs->bp_2_ap_addr;
		per_2_emi = sdma->script_addrs->ap_2_bp_addr;
		break;
	case IMX_DMATYPE_FIRI:
		per_2_emi = sdma->script_addrs->firi_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_firi_addr;
		break;
	case IMX_DMATYPE_UART:
		per_2_emi = sdma->script_addrs->uart_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_app_addr;
		break;
	case IMX_DMATYPE_UART_SP:
		per_2_emi = sdma->script_addrs->uartsh_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_shp_addr;
		break;
	case IMX_DMATYPE_ATA:
		per_2_emi = sdma->script_addrs->ata_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_ata_addr;
		break;
	case IMX_DMATYPE_CSPI:
		per_2_emi = sdma->script_addrs->app_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_ecspi_addr;
		break;
	case IMX_DMATYPE_EXT:
	case IMX_DMATYPE_SSI:
	case IMX_DMATYPE_SAI:
		per_2_emi = sdma->script_addrs->app_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_app_addr;
		break;
	case IMX_DMATYPE_SSI_DUAL:
		per_2_emi = sdma->script_addrs->ssish_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_ssish_addr;
		break;
	case IMX_DMATYPE_SSI_SP:
	case IMX_DMATYPE_MMC:
	case IMX_DMATYPE_SDHC:
	case IMX_DMATYPE_CSPI_SP:
	case IMX_DMATYPE_ESAI:
	case IMX_DMATYPE_MSHC_SP:
		per_2_emi = sdma->script_addrs->shp_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_shp_addr;
		break;
	case IMX_DMATYPE_ASRC:
		per_2_emi = sdma->script_addrs->asrc_2_mcu_addr;
		emi_2_per = sdma->script_addrs->asrc_2_mcu_addr;
		per_2_per = sdma->script_addrs->per_2_per_addr;
		break;
	case IMX_DMATYPE_ASRC_SP:
		per_2_emi = sdma->script_addrs->shp_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_shp_addr;
		per_2_per = sdma->script_addrs->per_2_per_addr;
		break;
	case IMX_DMATYPE_MSHC:
		per_2_emi = sdma->script_addrs->mshc_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_mshc_addr;
		break;
	case IMX_DMATYPE_CCM:
		per_2_emi = sdma->script_addrs->dptc_dvfs_addr;
		break;
	case IMX_DMATYPE_SPDIF:
		per_2_emi = sdma->script_addrs->spdif_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_spdif_addr;
		break;
	case IMX_DMATYPE_IPU_MEMORY:
		emi_2_per = sdma->script_addrs->ext_mem_2_ipu_addr;
		break;
	case IMX_DMATYPE_HDMI:
		emi_2_per = sdma->script_addrs->hdmi_dma_addr;
		break;
	default:
		break;
	}

	sdmac->pc_from_device = per_2_emi;
	sdmac->pc_to_device = emi_2_per;
	sdmac->device_to_device = per_2_per;
	sdmac->pc_to_pc = emi_2_emi;
}

static int sdma_load_context(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	int load_address;
	struct sdma_context_data *context = sdma->context;
	struct sdma_buffer_descriptor *bd0 = sdma->channel[0].bd;
	int ret;
	unsigned long flags;

	if (sdmac->direction == DMA_DEV_TO_MEM)
		load_address = sdmac->pc_from_device;
	else if (sdmac->direction == DMA_DEV_TO_DEV)
		load_address = sdmac->device_to_device;
	else if (sdmac->direction == DMA_MEM_TO_MEM)
		load_address = sdmac->pc_to_pc;
	else
		load_address = sdmac->pc_to_device;

	if (load_address < 0)
		return load_address;

	dev_dbg(sdma->dev, "load_address = %d\n", load_address);
	dev_dbg(sdma->dev, "wml = 0x%08x\n", (u32)sdmac->watermark_level);
	dev_dbg(sdma->dev, "shp_addr = 0x%08x\n", sdmac->shp_addr);
	dev_dbg(sdma->dev, "per_addr = 0x%08x\n", sdmac->per_addr);
	dev_dbg(sdma->dev, "event_mask0 = 0x%08x\n", (u32)sdmac->event_mask[0]);
	dev_dbg(sdma->dev, "event_mask1 = 0x%08x\n", (u32)sdmac->event_mask[1]);

	spin_lock_irqsave(&sdma->channel_0_lock, flags);

	memset(context, 0, sizeof(*context));
	context->channel_state.pc = load_address;

	/* Send by context the event mask,base address for peripheral
	 * and watermark level
	 */
	if (sdmac->peripheral_type == IMX_DMATYPE_HDMI) {
		context->gReg[4] = sdmac->per_addr;
		context->gReg[6] = sdmac->shp_addr;
	} else {
		context->gReg[0] = sdmac->event_mask[1];
		context->gReg[1] = sdmac->event_mask[0];
		context->gReg[2] = sdmac->per_addr;
		context->gReg[6] = sdmac->shp_addr;
		context->gReg[7] = sdmac->watermark_level;
	}

	bd0->mode.command = C0_SETDM;
	bd0->mode.status = BD_DONE | BD_INTR | BD_WRAP | BD_EXTD;
	bd0->mode.count = sizeof(*context) / 4;
	bd0->buffer_addr = sdma->context_phys;
	bd0->ext_buffer_addr = 2048 + (sizeof(*context) / 4) * channel;
	ret = sdma_run_channel0(sdma);

	spin_unlock_irqrestore(&sdma->channel_0_lock, flags);

	return ret;
}

static void sdma_disable_channel(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	unsigned long flags;

	spin_lock_irqsave(&sdmac->lock, flags);
	sdmac->status = DMA_ERROR;
	spin_unlock_irqrestore(&sdmac->lock, flags);

	writel_relaxed(BIT(channel), sdma->regs + SDMA_H_STATSTOP);
}

static void sdma_set_watermarklevel_for_p2p(struct sdma_channel *sdmac)
{
	int lwml = sdmac->watermark_level & 0xff;
	int hwml = (sdmac->watermark_level >> 16) & 0xff;

	if (sdmac->event_id0 > 31) {
		sdmac->event_mask[0] |= 0;
		__set_bit(28, &sdmac->watermark_level);
		sdmac->event_mask[1] |=
				BIT(sdmac->event_id0 % 32);
	} else {
		sdmac->event_mask[0] |= 0;
		sdmac->event_mask[1] |=
				BIT(sdmac->event_id0 % 32);
	}
	if (sdmac->event_id1 > 31) {
		sdmac->event_mask[1] |= 0;
		__set_bit(29, &sdmac->watermark_level);
		sdmac->event_mask[0] |=
			BIT(sdmac->event_id1 % 32);
	} else {
		sdmac->event_mask[1] |= 0;
		sdmac->event_mask[0] |=
			BIT(sdmac->event_id1 % 32);
	}

	/*
	 * If LWML(src_maxburst) > HWML(dst_maxburst), we need
	 * swap LWML and HWML of INFO(A.3.2.5.1), also need swap
	 * r0(event_mask[1]) and r1(event_mask[0]).
	 */
	if (lwml > hwml) {
		sdmac->watermark_level &= ~0xff00ff;
		sdmac->watermark_level |= hwml;
		sdmac->watermark_level |= lwml << 16;
		swap(sdmac->event_mask[0], sdmac->event_mask[1]);
	}
	/* BIT 11:
	 * 1 : Source on SPBA
	 * 0 : Source on AIPS
	 */
	__set_bit(11, &sdmac->watermark_level);
	/* BIT 12:
	 * 1 : Destination on SPBA
	 * 0 : Destination on AIPS
	 */
	__set_bit(12, &sdmac->watermark_level);
	__set_bit(31, &sdmac->watermark_level);
	/* BIT 31:
	 * 1 : Amount of samples to be transferred is
	 * unknown and script will keep on transferring
	 * samples as long as both events are detected
	 * and script must be manually stopped by the
	 * application.
	 * 0 : The amount of samples to be is equal to
	 * the count field of mode word
	 *
	 */
	__set_bit(25, &sdmac->watermark_level);
	__clear_bit(24, &sdmac->watermark_level);
}

static int sdma_config_channel(struct sdma_channel *sdmac)
{
	int ret;

	sdma_disable_channel(sdmac);

	sdmac->event_mask[0] = 0;
	sdmac->event_mask[1] = 0;
	sdmac->shp_addr = 0;
	sdmac->per_addr = 0;

	if (sdmac->event_id0) {
		if (sdmac->event_id0 >= sdmac->sdma->drvdata->num_events)
			return -EINVAL;
		sdma_event_enable(sdmac, sdmac->event_id0);
	}

	if (sdmac->event_id1) {
		if (sdmac->event_id1 >= sdmac->sdma->drvdata->num_events)
			return -EINVAL;
		sdma_event_enable(sdmac, sdmac->event_id1);
	}

	switch (sdmac->peripheral_type) {
	case IMX_DMATYPE_DSP:
		sdma_config_ownership(sdmac, false, true, true);
		break;
	case IMX_DMATYPE_MEMORY:
		sdma_config_ownership(sdmac, false, true, false);
		break;
	default:
		sdma_config_ownership(sdmac, true, true, false);
		break;
	}

	sdma_get_pc(sdmac, sdmac->peripheral_type);

	if ((sdmac->peripheral_type != IMX_DMATYPE_MEMORY) &&
			(sdmac->peripheral_type != IMX_DMATYPE_DSP)) {
		/* Handle multiple event channels differently */
		if (sdmac->event_id1) {
			if (sdmac->peripheral_type == IMX_DMATYPE_ASRC_SP ||
			    sdmac->peripheral_type == IMX_DMATYPE_ASRC)
				sdma_set_watermarklevel_for_p2p(sdmac);
		} else
			__set_bit(sdmac->event_id0, sdmac->event_mask);

		/* Watermark Level */
		sdmac->watermark_level |= sdmac->watermark_level;
		/* Address */
		if (sdmac->direction == DMA_DEV_TO_DEV ||
			 (sdmac->peripheral_type == IMX_DMATYPE_HDMI)) {
			sdmac->shp_addr = sdmac->per_address2;
			sdmac->per_addr = sdmac->per_address;
		} else {
			sdmac->shp_addr = sdmac->per_address;
		}
	} else {
		sdmac->watermark_level = 0; /* FIXME: M3_BASE_ADDRESS */
	}

	ret = sdma_load_context(sdmac);

	return ret;
}

static int sdma_set_channel_priority(struct sdma_channel *sdmac,
		unsigned int priority)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;

	if (priority < MXC_SDMA_MIN_PRIORITY
	    || priority > MXC_SDMA_MAX_PRIORITY) {
		return -EINVAL;
	}

	writel_relaxed(priority, sdma->regs + SDMA_CHNPRI_0 + 4 * channel);

	return 0;
}

static int sdma_request_channel(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	int ret = -EBUSY;

	sdmac->bd_iram = true;
	sdmac->bd = gen_pool_dma_alloc(sdma->iram_pool, PAGE_SIZE, &sdmac->bd_phys);
	if (!sdmac->bd) {
		sdmac->bd_iram = false;
		sdmac->bd = dma_alloc_coherent(NULL, PAGE_SIZE, &sdmac->bd_phys, GFP_KERNEL);
		if (!sdmac->bd) {
			ret = -ENOMEM;
			goto out;
		}
	}

	memset(sdmac->bd, 0, PAGE_SIZE);

	sdma->channel_control[channel].base_bd_ptr = sdmac->bd_phys;
	sdma->channel_control[channel].current_bd_ptr = sdmac->bd_phys;

	sdma_set_channel_priority(sdmac, MXC_SDMA_DEFAULT_PRIORITY);
	return 0;
out:

	return ret;
}

static struct sdma_channel *to_sdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct sdma_channel, chan);
}

static dma_cookie_t sdma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	unsigned long flags;
	struct sdma_channel *sdmac = to_sdma_chan(tx->chan);
	dma_cookie_t cookie;

	spin_lock_irqsave(&sdmac->lock, flags);

	cookie = dma_cookie_assign(tx);

	spin_unlock_irqrestore(&sdmac->lock, flags);

	return cookie;
}

static int sdma_alloc_chan_resources(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct imx_dma_data *data = chan->private;
	int prio, ret;

	if (!data)
		return -EINVAL;

	switch (data->priority) {
	case DMA_PRIO_HIGH:
		prio = 3;
		break;
	case DMA_PRIO_MEDIUM:
		prio = 2;
		break;
	case DMA_PRIO_LOW:
	default:
		prio = 1;
		break;
	}

	sdmac->peripheral_type = data->peripheral_type;
	sdmac->event_id0 = data->dma_request;
	sdmac->event_id1 = data->dma_request2;

	clk_enable(sdmac->sdma->clk_ipg);
	clk_enable(sdmac->sdma->clk_ahb);

	ret = sdma_request_channel(sdmac);
	if (ret)
		return ret;

	ret = sdma_set_channel_priority(sdmac, prio);
	if (ret)
		return ret;

	dma_async_tx_descriptor_init(&sdmac->desc, chan);
	sdmac->desc.tx_submit = sdma_tx_submit;
	/* txd.flags will be overwritten in prep funcs */
	sdmac->desc.flags = DMA_CTRL_ACK;

	return 0;
}

static void sdma_free_chan_resources(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;

	sdma_disable_channel(sdmac);

	if (sdmac->event_id0)
		sdma_event_disable(sdmac, sdmac->event_id0);
	if (sdmac->event_id1)
		sdma_event_disable(sdmac, sdmac->event_id1);

	sdmac->event_id0 = 0;
	sdmac->event_id1 = 0;

	sdma_set_channel_priority(sdmac, 0);

	if (sdmac->bd_iram)
		gen_pool_free(sdma->iram_pool, (unsigned long)sdmac->bd, PAGE_SIZE);
	else
		dma_free_coherent(NULL, PAGE_SIZE, sdmac->bd, sdmac->bd_phys);

	clk_disable(sdma->clk_ipg);
	clk_disable(sdma->clk_ahb);
}

static int sdma_transfer_init(struct sdma_channel *sdmac,
			      enum dma_transfer_direction direction)
{
	int ret = 0;

	sdmac->status = DMA_IN_PROGRESS;
	sdmac->buf_tail = 0;
	sdmac->flags = 0;
	sdmac->direction = direction;

	ret = sdma_load_context(sdmac);
	if (ret)
		return ret;

	sdmac->chn_count = 0;

	return ret;
}

static int check_bd_buswidth(struct sdma_buffer_descriptor *bd,
			     struct sdma_channel *sdmac, int count,
			     dma_addr_t dma_dst, dma_addr_t dma_src)
{
	int ret = 0;

	switch (sdmac->word_size) {
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		bd->mode.command = 0;
		if ((count | dma_dst | dma_src) & 3)
			ret = -EINVAL;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		bd->mode.command = 2;
		if ((count | dma_dst | dma_src) & 1)
			ret = -EINVAL;
		break;
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		 bd->mode.command = 1;
		 break;
	default:
		 return -EINVAL;
	}

	return ret;
}

static struct dma_async_tx_descriptor *sdma_prep_memcpy(
		struct dma_chan *chan, dma_addr_t dma_dst,
		dma_addr_t dma_src, size_t len, unsigned long flags)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	size_t count;
	int i = 0, param;
	struct sdma_buffer_descriptor *bd;

	if (!chan || !len || sdmac->status == DMA_IN_PROGRESS)
	{
		if(!chan)
			printk("hndz chan null!\n");
		else
			printk("hndz len %d sdmac->status %d!\n", len, sdmac->status);
		return NULL;
	}

	if (len >= NUM_BD * SDMA_BD_MAX_CNT) {
		dev_err(sdma->dev, "channel%d: maximum bytes exceeded:%zu > %d\n",
			channel, len, NUM_BD * SDMA_BD_MAX_CNT);
		goto err_out;
	}

	dev_dbg(sdma->dev, "memcpy: %pad->%pad, len=%zu, channel=%d.\n",
		&dma_src, &dma_dst, len, channel);

	if (sdma_transfer_init(sdmac, DMA_MEM_TO_MEM))
	{
		printk("hndz sdma_transfer_init error!\n");
		goto err_out;
	}

	do {
		count = min_t(size_t, len, SDMA_BD_MAX_CNT);
		bd = &sdmac->bd[i];
		bd->buffer_addr = dma_src;
		bd->ext_buffer_addr = dma_dst;
		bd->mode.count = count;

		if (check_bd_buswidth(bd, sdmac, count, dma_dst, dma_src))
		{
			printk("hndz check_bd_buswidth error!\n");
			goto err_out;
		}
			

		dma_src += count;
		dma_dst += count;
		len -= count;
		i++;

		param = BD_DONE | BD_EXTD | BD_CONT;
		/* last bd */
		if (!len) {
			param |= BD_INTR;
			param |= BD_LAST;
			param &= ~BD_CONT;
		}

		dev_dbg(sdma->dev, "entry %d: count: %d dma: 0x%u %s%s\n",
				i, count, bd->buffer_addr,
				param & BD_WRAP ? "wrap" : "",
				param & BD_INTR ? " intr" : "");

		bd->mode.status = param;
		sdmac->chn_count += count;
	} while (len);

	sdmac->num_bd = i;
	sdma->channel_control[channel].current_bd_ptr = sdmac->bd_phys;

	return &sdmac->desc;
err_out:
	sdmac->status = DMA_ERROR;
	return NULL;
}

/*
 * Please ensure dst_nents no smaller than src_nents , also every sg_len of
 * dst_sg node no smaller than src_sg. To simply things, please use the same
 * size of dst_sg as src_sg.
 */
static struct dma_async_tx_descriptor *sdma_prep_sg(
		struct dma_chan *chan,
		struct scatterlist *dst_sg, unsigned int dst_nents,
		struct scatterlist *src_sg, unsigned int src_nents,
		enum dma_transfer_direction direction)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;
	int ret, i, count;
	int channel = sdmac->channel;
	struct scatterlist *sg_src = src_sg, *sg_dst = dst_sg;

	if (sdmac->status == DMA_IN_PROGRESS)
	{
		printk("hndz sdmac status is DMA_IN_PROGRESS!\n");
		return NULL;
	}

	dev_dbg(sdma->dev, "setting up %d entries for channel %d.\n",
			src_nents, channel);

	if (src_nents > NUM_BD) {
		dev_err(sdma->dev, "SDMA channel %d: maximum number of sg exceeded: %d > %d\n",
				channel, src_nents, NUM_BD);
		ret = -EINVAL;
		goto err_out;
	}

	if (sdma_transfer_init(sdmac, direction))
	{
		printk("hndz sdmac transfer init error!\n");
		goto err_out;
	}

	for_each_sg(src_sg, sg_src, src_nents, i) {
		struct sdma_buffer_descriptor *bd = &sdmac->bd[i];
		int param;

		bd->buffer_addr = sg_src->dma_address;

		if (direction == DMA_MEM_TO_MEM) {
			BUG_ON(!sg_dst);
			bd->ext_buffer_addr = sg_dst->dma_address;
		}

		count = sg_dma_len(sg_src);

		if (count > SDMA_BD_MAX_CNT) {
			dev_err(sdma->dev, "SDMA channel %d: maximum bytes for sg entry exceeded: %d > %d\n",
					channel, count, SDMA_BD_MAX_CNT);
			ret = -EINVAL;
			goto err_out;
		}

		bd->mode.count = count;
		sdmac->chn_count += count;

		if (direction == DMA_MEM_TO_MEM)
			ret = check_bd_buswidth(bd, sdmac, count,
						sg_dst->dma_address,
						sg_src->dma_address);
		else
			ret = check_bd_buswidth(bd, sdmac, count, 0,
						sg_src->dma_address);
		if (ret)
		{
			printk("hndz check bd error!\n");
			goto err_out;
		}

		param = BD_DONE | BD_EXTD | BD_CONT;

		if (i + 1 == src_nents) {
			param |= BD_INTR;
			param |= BD_LAST;
			param &= ~BD_CONT;
		}

		dev_dbg(sdma->dev, "entry %d: count: %d dma: 0x%pad %s%s\n",
				i, count, &sg_src->dma_address,
				param & BD_WRAP ? "wrap" : "",
				param & BD_INTR ? " intr" : "");

		bd->mode.status = param;
		if (direction == DMA_MEM_TO_MEM)
			sg_dst = sg_next(sg_dst);
	}

	sdmac->num_bd = src_nents;
	sdma->channel_control[channel].current_bd_ptr = sdmac->bd_phys;

	return &sdmac->desc;
err_out:
	sdmac->status = DMA_ERROR;
	return NULL;
}

static struct dma_async_tx_descriptor *sdma_prep_memcpy_sg(
		struct dma_chan *chan,
		struct scatterlist *dst_sg, unsigned int dst_nents,
		struct scatterlist *src_sg, unsigned int src_nents,
		unsigned long flags)
{
	return sdma_prep_sg(chan, dst_sg, dst_nents, src_sg, src_nents,
			   DMA_MEM_TO_MEM);
}

static struct dma_async_tx_descriptor *sdma_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	return sdma_prep_sg(chan, NULL, 0, sgl, sg_len, direction);
}

static struct dma_async_tx_descriptor *sdma_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t dma_addr, size_t buf_len,
		size_t period_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	int ret, i = 0, buf = 0;
	int num_periods;

	dev_dbg(sdma->dev, "%s channel: %d\n", __func__, channel);

	if (sdmac->status == DMA_IN_PROGRESS)
		return NULL;

	sdmac->status = DMA_IN_PROGRESS;

	sdmac->buf_tail = 0;
	sdmac->period_len = period_len;

	sdmac->flags |= IMX_DMA_SG_LOOP;
	sdmac->direction = direction;
	ret = sdma_load_context(sdmac);
	if (ret)
		goto err_out;

	if (sdmac->peripheral_type == IMX_DMATYPE_HDMI)
		return &sdmac->desc;
	else
		num_periods = buf_len / period_len;

	if (num_periods > NUM_BD) {
		dev_err(sdma->dev, "SDMA channel %d: maximum number of sg exceeded: %d > %d\n",
				channel, num_periods, NUM_BD);
		goto err_out;
	}

	if (period_len > SDMA_BD_MAX_CNT) {
		dev_err(sdma->dev, "SDMA channel %d: maximum period size exceeded: %zu > %d\n",
				channel, period_len, SDMA_BD_MAX_CNT);
		goto err_out;
	}

	if (sdmac->peripheral_type == IMX_DMATYPE_UART)
		sdmac->chn_count = period_len;

	while (buf < buf_len) {
		struct sdma_buffer_descriptor *bd = &sdmac->bd[i];
		int param;

		bd->buffer_addr = dma_addr;

		bd->mode.count = period_len;

		if (sdmac->word_size > DMA_SLAVE_BUSWIDTH_4_BYTES)
			goto err_out;
		if (sdmac->word_size == DMA_SLAVE_BUSWIDTH_4_BYTES)
			bd->mode.command = 0;
		else
			bd->mode.command = sdmac->word_size;

		param = BD_DONE | BD_EXTD | BD_CONT | BD_INTR;
		if (i + 1 == num_periods)
			param |= BD_WRAP;

		dev_dbg(sdma->dev, "entry %d: count: %d dma: %pad %s%s\n",
				i, period_len, &dma_addr,
				param & BD_WRAP ? "wrap" : "",
				param & BD_INTR ? " intr" : "");

		bd->mode.status = param;

		dma_addr += period_len;
		buf += period_len;

		i++;
	}

	sdmac->num_bd = num_periods;
	sdma->channel_control[channel].current_bd_ptr = sdmac->bd_phys;

	return &sdmac->desc;
err_out:
	sdmac->status = DMA_ERROR;
	return NULL;
}

static int sdma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
		unsigned long arg)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct dma_slave_config *dmaengine_cfg = (void *)arg;

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		sdma_disable_channel(sdmac);
		return 0;
	case DMA_SLAVE_CONFIG:
		if (dmaengine_cfg->direction == DMA_DEV_TO_MEM) {
			sdmac->per_address = dmaengine_cfg->src_addr;
			sdmac->watermark_level = dmaengine_cfg->src_maxburst *
						dmaengine_cfg->src_addr_width;
			sdmac->word_size = dmaengine_cfg->src_addr_width;
		} else if (dmaengine_cfg->direction == DMA_DEV_TO_DEV) {
			sdmac->per_address = dmaengine_cfg->src_addr;
			sdmac->per_address2 = dmaengine_cfg->dst_addr;
			sdmac->watermark_level =
				dmaengine_cfg->src_maxburst & 0xff;
			sdmac->watermark_level |=
				(dmaengine_cfg->dst_maxburst & 0xff) << 16;
			sdmac->word_size = dmaengine_cfg->dst_addr_width;
		} else if (dmaengine_cfg->direction == DMA_MEM_TO_DEV) {
			sdmac->per_address = dmaengine_cfg->dst_addr;
			sdmac->watermark_level = dmaengine_cfg->dst_maxburst *
						dmaengine_cfg->dst_addr_width;
			sdmac->word_size = dmaengine_cfg->dst_addr_width;
		} else if (sdmac->peripheral_type == IMX_DMATYPE_HDMI) {
			sdmac->per_address = dmaengine_cfg->src_addr;
			sdmac->per_address2 = dmaengine_cfg->dst_addr;
			sdmac->watermark_level = 0;
		} else if (dmaengine_cfg->direction == DMA_MEM_TO_MEM) {
			sdmac->word_size = dmaengine_cfg->dst_addr_width;
		}

		sdmac->direction = dmaengine_cfg->direction;
		return sdma_config_channel(sdmac);
	default:
		return -ENOSYS;
	}

	return -EINVAL;
}

static enum dma_status sdma_tx_status(struct dma_chan *chan,
				      dma_cookie_t cookie,
				      struct dma_tx_state *txstate)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	u32 residue;

	/*
	 * For uart rx data may not receive fully, use old chn_real_count to
	 * know the real rx count.
	 */
	if ((sdmac->flags & IMX_DMA_SG_LOOP) &&
		(sdmac->peripheral_type != IMX_DMATYPE_UART))
		residue = (sdmac->num_bd - sdmac->buf_tail) * sdmac->period_len;
	else
		residue = sdmac->chn_count - sdmac->chn_real_count;

	dma_set_tx_state(txstate, chan->completed_cookie, chan->cookie,
			 residue);

	return sdmac->status;
}

static void sdma_issue_pending(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;

	if (sdmac->status == DMA_IN_PROGRESS)
		sdma_enable_channel(sdma, sdmac->channel);
}

#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V1	34
#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V2	38
#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V3	41

static void sdma_add_scripts(struct sdma_engine *sdma,
		const struct sdma_script_start_addrs *addr)
{
	s32 *addr_arr = (u32 *)addr;
	s32 *saddr_arr = (u32 *)sdma->script_addrs;
	int i;

	/* use the default firmware in ROM if missing external firmware */
	if (!sdma->script_number)
		sdma->script_number = SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V1;

	for (i = 0; i < sdma->script_number; i++)
		if (addr_arr[i] > 0)
			saddr_arr[i] = addr_arr[i];
}

static void sdma_load_firmware(const struct firmware *fw, void *context)
{
	struct sdma_engine *sdma = context;
	const struct sdma_firmware_header *header;
	const struct sdma_script_start_addrs *addr;
	unsigned short *ram_code;

	if (!fw) {
		dev_err(sdma->dev, "firmware not found\n");
		return;
	}

	if (fw->size < sizeof(*header))
		goto err_firmware;

	header = (struct sdma_firmware_header *)fw->data;

	if (header->magic != SDMA_FIRMWARE_MAGIC)
		goto err_firmware;
	if (header->ram_code_start + header->ram_code_size > fw->size)
		goto err_firmware;
	switch (header->version_major) {
		case 1:
			sdma->script_number = SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V1;
			break;
		case 2:
			sdma->script_number = SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V2;
			break;
		case 3:
			sdma->script_number = SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V3;
			break;
		default:
			dev_err(sdma->dev, "unknown firmware version\n");
			goto err_firmware;
	}

	addr = (void *)header + header->script_addrs_start;
	ram_code = (void *)header + header->ram_code_start;

	clk_enable(sdma->clk_ipg);
	clk_enable(sdma->clk_ahb);
	/* download the RAM image for SDMA */
	sdma_load_script(sdma, ram_code,
			header->ram_code_size,
			addr->ram_code_start_addr);
	clk_disable(sdma->clk_ipg);
	clk_disable(sdma->clk_ahb);

	sdma_add_scripts(sdma, addr);

	dev_info(sdma->dev, "loaded firmware %d.%d\n",
			header->version_major,
			header->version_minor);

err_firmware:
	release_firmware(fw);
}

#define EVENT_REMAP_CELLS 3

static int __init sdma_event_remap(struct sdma_engine *sdma)
{
	struct device_node *np = sdma->dev->of_node;
	struct device_node *gpr_np = of_parse_phandle(np, "gpr", 0);
	struct property *event_remap;
	struct regmap *gpr;
	char propname[] = "fsl,sdma-event-remap";
	u32 reg, val, shift, num_map, i;
	int ret = 0;

	if (IS_ERR(np) || IS_ERR(gpr_np))
		goto out;

	event_remap = of_find_property(np, propname, NULL);
	num_map = event_remap ? (event_remap->length / sizeof(u32)) : 0;
	if (!num_map) {
		dev_warn(sdma->dev, "no event needs to be remapped\n");
		goto out;
	} else if (num_map % EVENT_REMAP_CELLS) {
		dev_err(sdma->dev, "the property %s must modulo %d\n",
				propname, EVENT_REMAP_CELLS);
		ret = -EINVAL;
		goto out;
	}

	gpr = syscon_node_to_regmap(gpr_np);
	if (IS_ERR(gpr)) {
		dev_err(sdma->dev, "failed to get gpr regmap\n");
		ret = PTR_ERR(gpr);
		goto out;
	}

	for (i = 0; i < num_map; i += EVENT_REMAP_CELLS) {
		ret = of_property_read_u32_index(np, propname, i, &reg);
		if (ret) {
			dev_err(sdma->dev, "failed to read property %s index %d\n",
					propname, i);
			goto out;
		}

		ret = of_property_read_u32_index(np, propname, i + 1, &shift);
		if (ret) {
			dev_err(sdma->dev, "failed to read property %s index %d\n",
					propname, i + 1);
			goto out;
		}

		ret = of_property_read_u32_index(np, propname, i + 2, &val);
		if (ret) {
			dev_err(sdma->dev, "failed to read property %s index %d\n",
					propname, i + 2);
			goto out;
		}

		regmap_update_bits(gpr, reg, BIT(shift), val << shift);
	}

out:
	if (!IS_ERR(gpr_np))
		of_node_put(gpr_np);

	return ret;
}

static int sdma_get_firmware(struct sdma_engine *sdma,
		const char *fw_name)
{
	int ret;

	ret = request_firmware_nowait(THIS_MODULE,
			FW_ACTION_HOTPLUG, fw_name, sdma->dev,
			GFP_KERNEL, sdma, sdma_load_firmware);

	return ret;
}

static int __init sdma_init(struct sdma_engine *sdma)
{
	int i, ret, ccbsize;
	dma_addr_t ccb_phys;

	clk_enable(sdma->clk_ipg);
	clk_enable(sdma->clk_ahb);

	/* Be sure SDMA has not started yet */
	writel_relaxed(0, sdma->regs + SDMA_H_C0PTR);

	ccbsize = MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control)
		+ sizeof(struct sdma_context_data);

	sdma->channel_control = gen_pool_dma_alloc(sdma->iram_pool, ccbsize, &ccb_phys);
	if (!sdma->channel_control) {
		sdma->channel_control = dma_alloc_coherent(NULL, ccbsize,
						&ccb_phys, GFP_KERNEL);
		if (!sdma->channel_control) {
			ret = -ENOMEM;
			goto err_dma_alloc;
		}
	}

	sdma->context = (void *)sdma->channel_control +
		MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control);
	sdma->context_phys = ccb_phys +
		MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control);

	/* Zero-out the CCB structures array just allocated */
	memset(sdma->channel_control, 0,
			MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control));

	/* disable all channels */
	for (i = 0; i < sdma->drvdata->num_events; i++)
		writel_relaxed(0, sdma->regs + chnenbl_ofs(sdma, i));

	/* All channels have priority 0 */
	for (i = 0; i < MAX_DMA_CHANNELS; i++)
		writel_relaxed(0, sdma->regs + SDMA_CHNPRI_0 + i * 4);

	ret = sdma_request_channel(&sdma->channel[0]);
	if (ret)
		goto err_dma_alloc;

	sdma_config_ownership(&sdma->channel[0], false, true, false);

	/* Set Command Channel (Channel Zero) */
	writel_relaxed(0x4050, sdma->regs + SDMA_CHN0ADDR);

	/* Set bits of CONFIG register but with static context switching */
	/* FIXME: Check whether to set ACR bit depending on clock ratios */
	writel_relaxed(0, sdma->regs + SDMA_H_CONFIG);

	writel_relaxed(ccb_phys, sdma->regs + SDMA_H_C0PTR);

	/* Set bits of CONFIG register with given context switching mode */
	writel_relaxed(SDMA_H_CONFIG_CSM, sdma->regs + SDMA_H_CONFIG);

	/* Initializes channel's priorities */
	sdma_set_channel_priority(&sdma->channel[0], 7);

	clk_disable(sdma->clk_ipg);
	clk_disable(sdma->clk_ahb);

	return 0;

err_dma_alloc:
	clk_disable(sdma->clk_ipg);
	clk_disable(sdma->clk_ahb);
	dev_err(sdma->dev, "initialisation failed with %d\n", ret);
	return ret;
}

static bool sdma_filter_fn(struct dma_chan *chan, void *fn_param)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct imx_dma_data *data = fn_param;

	if (!imx_dma_is_general_purpose(chan))
		return false;

	sdmac->data = *data;
	chan->private = &sdmac->data;

	return true;
}

static struct dma_chan *sdma_xlate(struct of_phandle_args *dma_spec,
				   struct of_dma *ofdma)
{
	struct sdma_engine *sdma = ofdma->of_dma_data;
	dma_cap_mask_t mask = sdma->dma_device.cap_mask;
	struct imx_dma_data data;

	if (dma_spec->args_count != 3)
		return NULL;

	data.dma_request = dma_spec->args[0];
	data.peripheral_type = dma_spec->args[1];
	data.priority = dma_spec->args[2];


	/*
	 * init dma_request2 to zero, which is not used by the dts.
	 * For P2P, dma_request2 is init from dma_request_channel(),
	 * chan->private will point to the imx_dma_data, and in
	 * device_alloc_chan_resources(), imx_dma_data.dma_request2 will
	 * be set to sdmac->event_id1.
	 */
	data.dma_request2 = 0;

	return dma_request_channel(mask, sdma_filter_fn, &data);
}

static int __init sdma_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(sdma_dt_ids, &pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	const char *fw_name;
	int ret;
	int irq;
	struct resource *iores;
	struct sdma_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;
	struct sdma_engine *sdma;
	s32 *saddr_arr;
	const struct sdma_driver_data *drvdata = NULL;

	if (of_id)
		drvdata = of_id->data;
	else if (pdev->id_entry)
		drvdata = (void *)pdev->id_entry->driver_data;

	if (!drvdata) {
		dev_err(&pdev->dev, "unable to find driver data\n");
		return -EINVAL;
	}

	ret = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	sdma = kzalloc(sizeof(*sdma), GFP_KERNEL);
	if (!sdma)
		return -ENOMEM;

	spin_lock_init(&sdma->channel_0_lock);

	sdma->dev = &pdev->dev;
	sdma->drvdata = drvdata;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!iores || irq < 0) {
		ret = -EINVAL;
		goto err_irq;
	}

	if (!request_mem_region(iores->start, resource_size(iores), pdev->name)) {
		ret = -EBUSY;
		goto err_request_region;
	}

	sdma->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(sdma->clk_ipg)) {
		ret = PTR_ERR(sdma->clk_ipg);
		goto err_clk;
	}

	sdma->clk_ahb = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(sdma->clk_ahb)) {
		ret = PTR_ERR(sdma->clk_ahb);
		goto err_clk;
	}

	clk_prepare(sdma->clk_ipg);
	clk_prepare(sdma->clk_ahb);

	sdma->regs = ioremap(iores->start, resource_size(iores));
	if (!sdma->regs) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	ret = request_irq(irq, sdma_int_handler, 0, "sdma", sdma);
	if (ret)
		goto err_request_irq;

	sdma->script_addrs = kzalloc(sizeof(*sdma->script_addrs), GFP_KERNEL);
	if (!sdma->script_addrs) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	/* initially no scripts available */
	saddr_arr = (s32 *)sdma->script_addrs;
	for (i = 0; i < SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V1; i++)
		saddr_arr[i] = -EINVAL;

	dma_cap_set(DMA_SLAVE, sdma->dma_device.cap_mask);
	dma_cap_set(DMA_CYCLIC, sdma->dma_device.cap_mask);
	dma_cap_set(DMA_MEMCPY, sdma->dma_device.cap_mask);

	INIT_LIST_HEAD(&sdma->dma_device.channels);
	/* Initialize channel parameters */
	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		struct sdma_channel *sdmac = &sdma->channel[i];

		sdmac->sdma = sdma;
		spin_lock_init(&sdmac->lock);

		sdmac->chan.device = &sdma->dma_device;
		dma_cookie_init(&sdmac->chan);
		sdmac->channel = i;

		tasklet_init(&sdmac->tasklet, sdma_tasklet,
			     (unsigned long) sdmac);
		/*
		 * Add the channel to the DMAC list. Do not add channel 0 though
		 * because we need it internally in the SDMA driver. This also means
		 * that channel 0 in dmaengine counting matches sdma channel 1.
		 */
		if (i)
			list_add_tail(&sdmac->chan.device_node,
					&sdma->dma_device.channels);
	}

	if (np)
		sdma->iram_pool = of_get_named_gen_pool(np, "iram", 0);
	if (!sdma->iram_pool)
		dev_warn(&pdev->dev, "no iram assigned, using external mem\n");

	ret = sdma_init(sdma);
	if (ret)
		goto err_init;

	ret = sdma_event_remap(sdma);
	if (ret)
		goto err_init;

	if (sdma->drvdata->script_addrs)
		sdma_add_scripts(sdma, sdma->drvdata->script_addrs);
	if (pdata && pdata->script_addrs)
		sdma_add_scripts(sdma, pdata->script_addrs);

	if (pdata) {
		ret = sdma_get_firmware(sdma, pdata->fw_name);
		if (ret)
			dev_warn(&pdev->dev, "failed to get firmware from platform data\n");
	} else {
		/*
		 * Because that device tree does not encode ROM script address,
		 * the RAM script in firmware is mandatory for device tree
		 * probe, otherwise it fails.
		 */
		ret = of_property_read_string(np, "fsl,sdma-ram-script-name",
					      &fw_name);
		if (ret)
			dev_warn(&pdev->dev, "failed to get firmware name\n");
		else {
			ret = sdma_get_firmware(sdma, fw_name);
			if (ret)
				dev_warn(&pdev->dev, "failed to get firmware from device tree\n");
		}
	}
	sdma->fw_name = fw_name;

	sdma->dma_device.dev = &pdev->dev;

	sdma->dma_device.device_alloc_chan_resources = sdma_alloc_chan_resources;
	sdma->dma_device.device_free_chan_resources = sdma_free_chan_resources;
	sdma->dma_device.device_tx_status = sdma_tx_status;
	sdma->dma_device.device_prep_slave_sg = sdma_prep_slave_sg;
	sdma->dma_device.device_prep_dma_cyclic = sdma_prep_dma_cyclic;
	sdma->dma_device.device_prep_dma_memcpy = sdma_prep_memcpy;
	sdma->dma_device.device_prep_dma_sg = sdma_prep_memcpy_sg;
	sdma->dma_device.device_control = sdma_control;
	sdma->dma_device.device_issue_pending = sdma_issue_pending;
	sdma->dma_device.dev->dma_parms = &sdma->dma_parms;
	dma_set_max_seg_size(sdma->dma_device.dev, 65535);

	ret = dma_async_device_register(&sdma->dma_device);
	if (ret) {
		dev_err(&pdev->dev, "unable to register\n");
		goto err_init;
	}

	if (np) {
		ret = of_dma_controller_register(np, sdma_xlate, sdma);
		if (ret) {
			dev_err(&pdev->dev, "failed to register controller\n");
			goto err_register;
		}
	}

	platform_set_drvdata(pdev, sdma);
	dev_info(sdma->dev, "initialized\n");

	return 0;

err_register:
	dma_async_device_unregister(&sdma->dma_device);
err_init:
	kfree(sdma->script_addrs);
err_alloc:
	free_irq(irq, sdma);
err_request_irq:
	iounmap(sdma->regs);
err_ioremap:
err_clk:
	release_mem_region(iores->start, resource_size(iores));
err_request_region:
err_irq:
	kfree(sdma);
	return ret;
}

static int sdma_remove(struct platform_device *pdev)
{
	return -EBUSY;
}

#ifdef CONFIG_PM_SLEEP
static int sdma_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdma_engine *sdma = platform_get_drvdata(pdev);
	int i;

	/* Do nothing if not i.MX6SX */
	if (sdma->drvdata != &sdma_imx6sx)
		return 0;

	clk_enable(sdma->clk_ipg);
	clk_enable(sdma->clk_ahb);
	/* save regs */
	for (i = 0; i < MXC_SDMA_SAVED_REG_NUM; i++) {
		/*
		 * 0x78(SDMA_XTRIG_CONF2+4)~0x100(SDMA_CHNPRI_O) registers are
		 * reserved and can't be touched. Skip these regs.
		 */
		if (i > SDMA_XTRIG_CONF2 / 4)
			sdma->save_regs[i] = readl_relaxed(sdma->regs +
							   MXC_SDMA_RESERVED_REG
							   + 4 * i);
		else
			sdma->save_regs[i] = readl_relaxed(sdma->regs + 4 * i);
	}

	clk_disable(sdma->clk_ipg);
	clk_disable(sdma->clk_ahb);

	return 0;
}

static int sdma_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdma_engine *sdma = platform_get_drvdata(pdev);
	int i, ret;

	/* Do nothing if not i.MX6SX */
	if (sdma->drvdata != &sdma_imx6sx)
		return 0;

	clk_enable(sdma->clk_ipg);
	clk_enable(sdma->clk_ahb);
	/* Do nothing if mega/fast mix not turned off */
	if (readl_relaxed(sdma->regs + SDMA_H_C0PTR)) {
		clk_disable(sdma->clk_ipg);
		clk_disable(sdma->clk_ahb);
		return 0;
	}
	/* restore regs and load firmware */
	for (i = 0; i < MXC_SDMA_SAVED_REG_NUM; i++) {
		/*
		 * 0x78(SDMA_XTRIG_CONF2+4)~0x100(SDMA_CHNPRI_O) registers are
		 * reserved and can't be touched. Skip these regs.
		 */
		if (i > SDMA_XTRIG_CONF2 / 4)
			writel_relaxed(sdma->save_regs[i], sdma->regs +
				       MXC_SDMA_RESERVED_REG + 4 * i);
		else
			writel_relaxed(sdma->save_regs[i] , sdma->regs + 4 * i);
	}

	/* prepare priority for channel0 to start */
	sdma_set_channel_priority(&sdma->channel[0], MXC_SDMA_DEFAULT_PRIORITY);
	clk_disable(sdma->clk_ipg);
	clk_disable(sdma->clk_ahb);

	ret = sdma_get_firmware(sdma, sdma->fw_name);
	if (ret) {
		dev_warn(&pdev->dev, "failed to get firware\n");
		return ret;
	}

	return 0;
}
#endif

static const struct dev_pm_ops sdma_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sdma_suspend, sdma_resume)
};

static struct platform_driver sdma_driver = {
	.driver		= {
		.name	= "imx-sdma",
		.of_match_table = sdma_dt_ids,
		.pm = &sdma_pm_ops,
	},
	.id_table	= sdma_devtypes,
	.remove		= sdma_remove,
};

static int __init sdma_module_init(void)
{
	return platform_driver_probe(&sdma_driver, sdma_probe);
}
module_init(sdma_module_init);

MODULE_AUTHOR("Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("i.MX SDMA driver");
MODULE_LICENSE("GPL");
