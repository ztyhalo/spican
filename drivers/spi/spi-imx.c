/*
 * Copyright 2004-2007, 2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 Juergen Beisert
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation
 * 51 Franklin Street, Fifth Floor
 * Boston, MA  02110-1301, USA.
 */

#include "spi-imx.h"

static int gSpiStage = 0;

static inline int is_imx27_cspi(struct spi_imx_data *d)
{
	return d->devtype_data->devtype == IMX27_CSPI;
}

static inline int is_imx35_cspi(struct spi_imx_data *d)
{
	return d->devtype_data->devtype == IMX35_CSPI;
}

static inline unsigned spi_imx_get_fifosize(struct spi_imx_data *d)
{
	return (d->devtype_data->devtype == IMX51_ECSPI) ? 64 : 8;
}

#define MXC_SPI_BUF_RX(type)						\
static void spi_imx_buf_rx_##type(struct spi_imx_data *spi_imx)		\
{									\
	unsigned int val = readl(spi_imx->base + MXC_CSPIRXDATA);	\
									\
	if (spi_imx->rx_buf) {						\
		*(type *)spi_imx->rx_buf = val;				\
		spi_imx->rx_buf += sizeof(type);			\
	}								\
}

#define MXC_SPI_BUF_TX(type)						\
static void spi_imx_buf_tx_##type(struct spi_imx_data *spi_imx)		\
{									\
	type val = 0;							\
									\
	if (spi_imx->tx_buf) {						\
		val = *(type *)spi_imx->tx_buf;				\
		spi_imx->tx_buf += sizeof(type);			\
	}								\
									\
	spi_imx->count -= sizeof(type);					\
									\
	writel(val, spi_imx->base + MXC_CSPITXDATA);			\
}

MXC_SPI_BUF_RX(u8)
MXC_SPI_BUF_TX(u8)
MXC_SPI_BUF_RX(u16)
MXC_SPI_BUF_TX(u16)
MXC_SPI_BUF_RX(u32)
MXC_SPI_BUF_TX(u32)

/* First entry is reserved, second entry is valid only if SDHC_SPIEN is set
 * (which is currently not the case in this driver)
 */
static int mxc_clkdivs[] = {0, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192,
	256, 384, 512, 768, 1024};

/* MX21, MX27 */
static unsigned int spi_imx_clkdiv_1(unsigned int fin,
		unsigned int fspi, unsigned int max)
{
	int i;

	for (i = 2; i < max; i++)
		if (fspi * mxc_clkdivs[i] >= fin)
			return i;

	return max;
}

/* MX1, MX31, MX35, MX51 CSPI */
static unsigned int spi_imx_clkdiv_2(unsigned int fin,
		unsigned int fspi)
{
	int i, div = 4;

	for (i = 0; i < 7; i++) {
		if (fspi * div >= fin)
			return i;
		div <<= 1;
	}

	return 7;
}

static bool spi_imx_can_dma(struct spi_master *master, struct spi_device *spi,
			 struct spi_transfer *transfer)
{
	// dump_stack();
	// printk("zty spi imx can dma %d !\n", transfer->len);
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);

	// if (spi_imx->dma_is_inited &&
	// 	(transfer->len > spi_imx_get_fifosize(spi_imx)) &&
	// 	(transfer->len > spi_imx_get_fifosize(spi_imx)))
	if (spi_imx->dma_is_inited )
		return true;
	return false;
}

#define MX51_ECSPI_CTRL		0x08
#define MX51_ECSPI_CTRL_ENABLE		(1 <<  0)
#define MX51_ECSPI_CTRL_XCH		(1 <<  2)
#define MX51_ECSPI_CTRL_SMC		(1 << 3)
#define MX51_ECSPI_CTRL_MODE_MASK	(0xf << 4)
#define MX51_ECSPI_CTRL_POSTDIV_OFFSET	8
#define MX51_ECSPI_CTRL_PREDIV_OFFSET	12
#define MX51_ECSPI_CTRL_CS(cs)		((cs) << 18)
#define MX51_ECSPI_CTRL_BL_OFFSET	20

#define MX51_ECSPI_CONFIG	0x0c
#define MX51_ECSPI_CONFIG_SCLKPHA(cs)	(1 << ((cs) +  0))
#define MX51_ECSPI_CONFIG_SCLKPOL(cs)	(1 << ((cs) +  4))
#define MX51_ECSPI_CONFIG_SBBCTRL(cs)	(1 << ((cs) +  8))
#define MX51_ECSPI_CONFIG_SSBPOL(cs)	(1 << ((cs) + 12))
#define MX51_ECSPI_CONFIG_SCLKCTL(cs)	(1 << ((cs) + 20))

#define MX51_ECSPI_INT		0x10
#define MX51_ECSPI_INT_TEEN		(1 <<  0)
#define MX51_ECSPI_INT_RREN		(1 <<  3)

#define MX51_ECSPI_DMA      0x14
#define MX51_ECSPI_DMA_TX_WML_OFFSET	0
#define MX51_ECSPI_DMA_TX_WML_MASK	0x3F
#define MX51_ECSPI_DMA_RX_WML_OFFSET	16
#define MX51_ECSPI_DMA_RX_WML_MASK	(0x3F << 16)
#define MX51_ECSPI_DMA_RXT_WML_OFFSET	24
#define MX51_ECSPI_DMA_RXT_WML_MASK	(0x3F << 24)

#define MX51_ECSPI_DMA_TEDEN_OFFSET	7
#define MX51_ECSPI_DMA_RXDEN_OFFSET	23
#define MX51_ECSPI_DMA_RXTDEN_OFFSET	31

#define MX51_ECSPI_STAT		0x18
#define MX51_ECSPI_STAT_RR		(1 <<  3)

/* MX51 eCSPI */
static unsigned int mx51_ecspi_clkdiv(unsigned int fin, unsigned int fspi,
				      unsigned int *fres)
{
	/*
	 * there are two 4-bit dividers, the pre-divider divides by
	 * $pre, the post-divider by 2^$post
	 */
	unsigned int pre, post;

	if (unlikely(fspi > fin))
		return 0;

	post = fls(fin) - fls(fspi);
	if (fin > fspi << post)
		post++;

	/* now we have: (fin <= fspi << post) with post being minimal */

	post = max(4U, post) - 4;
	if (unlikely(post > 0xf)) {
		pr_err("%s: cannot set clock freq: %u (base freq: %u)\n",
				__func__, fspi, fin);
		return 0xff;
	}

	pre = DIV_ROUND_UP(fin, fspi << post) - 1;

	pr_debug("%s: fin: %u, fspi: %u, post: %u, pre: %u\n",
			__func__, fin, fspi, post, pre);

	/* Resulting frequency for the SCLK line. */
	*fres = (fin / (pre + 1)) >> post;

	return (pre << MX51_ECSPI_CTRL_PREDIV_OFFSET) |
		(post << MX51_ECSPI_CTRL_POSTDIV_OFFSET);
}

static void __maybe_unused mx51_ecspi_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned val = 0;

	if (enable & MXC_INT_TE)
		val |= MX51_ECSPI_INT_TEEN;

	if (enable & MXC_INT_RR)
		val |= MX51_ECSPI_INT_RREN;

	writel(val, spi_imx->base + MX51_ECSPI_INT);
}

static void __maybe_unused mx51_ecspi_trigger(struct spi_imx_data *spi_imx)
{
	u32 reg = readl(spi_imx->base + MX51_ECSPI_CTRL);

	if (!spi_imx->usedma)
		reg |= MX51_ECSPI_CTRL_XCH;
	else if (!spi_imx->dma_finished)
		reg |= MX51_ECSPI_CTRL_SMC;
	else
		reg &= ~MX51_ECSPI_CTRL_SMC;
	writel(reg, spi_imx->base + MX51_ECSPI_CTRL);
}

static int __maybe_unused mx51_ecspi_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	u32 ctrl = MX51_ECSPI_CTRL_ENABLE, cfg = 0, dma = 0;
	u32 tx_wml_cfg, rx_wml_cfg, rxt_wml_cfg;
	u32 clk = config->speed_hz, delay;

	/*
	 * The hardware seems to have a race condition when changing modes. The
	 * current assumption is that the selection of the channel arrives
	 * earlier in the hardware than the mode bits when they are written at
	 * the same time.
	 * So set master mode for all channels as we do not support slave mode.
	 */
	ctrl |= MX51_ECSPI_CTRL_MODE_MASK;

	/* set clock speed */
	ctrl |= mx51_ecspi_clkdiv(spi_imx->spi_clk, config->speed_hz, &clk);

	/* set chip select to use */
	ctrl |= MX51_ECSPI_CTRL_CS(config->cs);

	ctrl |= (config->bpw - 1) << MX51_ECSPI_CTRL_BL_OFFSET;

	cfg |= MX51_ECSPI_CONFIG_SBBCTRL(config->cs);

	if (config->mode & SPI_CPHA)
		cfg |= MX51_ECSPI_CONFIG_SCLKPHA(config->cs);

	if (config->mode & SPI_CPOL) {
		cfg |= MX51_ECSPI_CONFIG_SCLKPOL(config->cs);
		cfg |= MX51_ECSPI_CONFIG_SCLKCTL(config->cs);
	}
	if (config->mode & SPI_CS_HIGH)
		cfg |= MX51_ECSPI_CONFIG_SSBPOL(config->cs);

	writel(ctrl, spi_imx->base + MX51_ECSPI_CTRL);
	writel(cfg, spi_imx->base + MX51_ECSPI_CONFIG);

	/*
	 * Wait until the changes in the configuration register CONFIGREG
	 * propagate into the hardware. It takes exactly one tick of the
	 * SCLK clock, but we will wait two SCLK clock just to be sure. The
	 * effect of the delay it takes for the hardware to apply changes
	 * is noticable if the SCLK clock run very slow. In such a case, if
	 * the polarity of SCLK should be inverted, the GPIO ChipSelect might
	 * be asserted before the SCLK polarity changes, which would disrupt
	 * the SPI communication as the device on the other end would consider
	 * the change of SCLK polarity as a clock tick already.
	 */
	delay = (2 * 1000000) / clk;
	if (likely(delay < 10))	/* SCLK is faster than 100 kHz */
		udelay(delay);
	else			/* SCLK is _very_ slow */
		usleep_range(delay, delay + 10);

	/*
	 * Configure the DMA register: setup the watermark
	 * and enable DMA request.
	 */
	if (spi_imx->dma_is_inited) {
		dma = readl(spi_imx->base + MX51_ECSPI_DMA);

		spi_imx->tx_wml = spi_imx_get_fifosize(spi_imx) / 4;
		spi_imx->rx_wml = spi_imx_get_fifosize(spi_imx) / 2;
		spi_imx->rxt_wml = spi_imx_get_fifosize(spi_imx) / 2;

		spi_imx->tx_wml = 1;
		spi_imx->rx_wml = 1;
		spi_imx->rxt_wml = 1;

		rx_wml_cfg = spi_imx->rx_wml << MX51_ECSPI_DMA_RX_WML_OFFSET;
		tx_wml_cfg = spi_imx->tx_wml << MX51_ECSPI_DMA_TX_WML_OFFSET;
		rxt_wml_cfg = spi_imx->rxt_wml << MX51_ECSPI_DMA_RXT_WML_OFFSET;
		dma = (dma & ~MX51_ECSPI_DMA_TX_WML_MASK
			   & ~MX51_ECSPI_DMA_RX_WML_MASK
			   & ~MX51_ECSPI_DMA_RXT_WML_MASK)
			   | rx_wml_cfg | tx_wml_cfg | rxt_wml_cfg
			   |(1 << MX51_ECSPI_DMA_TEDEN_OFFSET)
			   |(1 << MX51_ECSPI_DMA_RXDEN_OFFSET)
			   |(1 << MX51_ECSPI_DMA_RXTDEN_OFFSET);

		writel(dma, spi_imx->base + MX51_ECSPI_DMA);
	}

	return 0;
}

static int __maybe_unused mx51_ecspi_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MX51_ECSPI_STAT) & MX51_ECSPI_STAT_RR;
}

static void __maybe_unused mx51_ecspi_reset(struct spi_imx_data *spi_imx)
{
	/* drain receive buffer */
	while (mx51_ecspi_rx_available(spi_imx))
		readl(spi_imx->base + MXC_CSPIRXDATA);
}

#define MX31_INTREG_TEEN	(1 << 0)
#define MX31_INTREG_RREN	(1 << 3)

#define MX31_CSPICTRL_ENABLE	(1 << 0)
#define MX31_CSPICTRL_MASTER	(1 << 1)
#define MX31_CSPICTRL_XCH	(1 << 2)
#define MX31_CSPICTRL_POL	(1 << 4)
#define MX31_CSPICTRL_PHA	(1 << 5)
#define MX31_CSPICTRL_SSCTL	(1 << 6)
#define MX31_CSPICTRL_SSPOL	(1 << 7)
#define MX31_CSPICTRL_BC_SHIFT	8
#define MX35_CSPICTRL_BL_SHIFT	20
#define MX31_CSPICTRL_CS_SHIFT	24
#define MX35_CSPICTRL_CS_SHIFT	12
#define MX31_CSPICTRL_DR_SHIFT	16

#define MX31_CSPISTATUS		0x14
#define MX31_STATUS_RR		(1 << 3)

/* These functions also work for the i.MX35, but be aware that
 * the i.MX35 has a slightly different register layout for bits
 * we do not use here.
 */
static void __maybe_unused mx31_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX31_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX31_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx31_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX31_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused mx31_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX31_CSPICTRL_ENABLE | MX31_CSPICTRL_MASTER;
	int cs = spi_imx->chipselect[config->cs];

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, config->speed_hz) <<
		MX31_CSPICTRL_DR_SHIFT;

	if (is_imx35_cspi(spi_imx)) {
		reg |= (config->bpw - 1) << MX35_CSPICTRL_BL_SHIFT;
		reg |= MX31_CSPICTRL_SSCTL;
	} else {
		reg |= (config->bpw - 1) << MX31_CSPICTRL_BC_SHIFT;
	}

	if (config->mode & SPI_CPHA)
		reg |= MX31_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX31_CSPICTRL_POL;
	if (config->mode & SPI_CS_HIGH)
		reg |= MX31_CSPICTRL_SSPOL;
	if (cs < 0)
		reg |= (cs + 32) <<
			(is_imx35_cspi(spi_imx) ? MX35_CSPICTRL_CS_SHIFT :
						  MX31_CSPICTRL_CS_SHIFT);

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx31_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MX31_CSPISTATUS) & MX31_STATUS_RR;
}

static void __maybe_unused mx31_reset(struct spi_imx_data *spi_imx)
{
	/* drain receive buffer */
	while (readl(spi_imx->base + MX31_CSPISTATUS) & MX31_STATUS_RR)
		readl(spi_imx->base + MXC_CSPIRXDATA);
}

#define MX21_INTREG_RR		(1 << 4)
#define MX21_INTREG_TEEN	(1 << 9)
#define MX21_INTREG_RREN	(1 << 13)

#define MX21_CSPICTRL_POL	(1 << 5)
#define MX21_CSPICTRL_PHA	(1 << 6)
#define MX21_CSPICTRL_SSPOL	(1 << 8)
#define MX21_CSPICTRL_XCH	(1 << 9)
#define MX21_CSPICTRL_ENABLE	(1 << 10)
#define MX21_CSPICTRL_MASTER	(1 << 11)
#define MX21_CSPICTRL_DR_SHIFT	14
#define MX21_CSPICTRL_CS_SHIFT	19

static void __maybe_unused mx21_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX21_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX21_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx21_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX21_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused mx21_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX21_CSPICTRL_ENABLE | MX21_CSPICTRL_MASTER;
	int cs = spi_imx->chipselect[config->cs];
	unsigned int max = is_imx27_cspi(spi_imx) ? 16 : 18;

	reg |= spi_imx_clkdiv_1(spi_imx->spi_clk, config->speed_hz, max) <<
		MX21_CSPICTRL_DR_SHIFT;
	reg |= config->bpw - 1;

	if (config->mode & SPI_CPHA)
		reg |= MX21_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX21_CSPICTRL_POL;
	if (config->mode & SPI_CS_HIGH)
		reg |= MX21_CSPICTRL_SSPOL;
	if (cs < 0)
		reg |= (cs + 32) << MX21_CSPICTRL_CS_SHIFT;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx21_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MXC_CSPIINT) & MX21_INTREG_RR;
}

static void __maybe_unused mx21_reset(struct spi_imx_data *spi_imx)
{
	writel(1, spi_imx->base + MXC_RESET);
}

#define MX1_INTREG_RR		(1 << 3)
#define MX1_INTREG_TEEN		(1 << 8)
#define MX1_INTREG_RREN		(1 << 11)

#define MX1_CSPICTRL_POL	(1 << 4)
#define MX1_CSPICTRL_PHA	(1 << 5)
#define MX1_CSPICTRL_XCH	(1 << 8)
#define MX1_CSPICTRL_ENABLE	(1 << 9)
#define MX1_CSPICTRL_MASTER	(1 << 10)
#define MX1_CSPICTRL_DR_SHIFT	13

static void __maybe_unused mx1_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX1_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX1_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx1_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX1_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused mx1_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX1_CSPICTRL_ENABLE | MX1_CSPICTRL_MASTER;

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, config->speed_hz) <<
		MX1_CSPICTRL_DR_SHIFT;
	reg |= config->bpw - 1;

	if (config->mode & SPI_CPHA)
		reg |= MX1_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX1_CSPICTRL_POL;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx1_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MXC_CSPIINT) & MX1_INTREG_RR;
}

static void __maybe_unused mx1_reset(struct spi_imx_data *spi_imx)
{
	writel(1, spi_imx->base + MXC_RESET);
}

static struct spi_imx_devtype_data imx1_cspi_devtype_data = {
	.intctrl = mx1_intctrl,
	.config = mx1_config,
	.trigger = mx1_trigger,
	.rx_available = mx1_rx_available,
	.reset = mx1_reset,
	.devtype = IMX1_CSPI,
};

static struct spi_imx_devtype_data imx21_cspi_devtype_data = {
	.intctrl = mx21_intctrl,
	.config = mx21_config,
	.trigger = mx21_trigger,
	.rx_available = mx21_rx_available,
	.reset = mx21_reset,
	.devtype = IMX21_CSPI,
};

static struct spi_imx_devtype_data imx27_cspi_devtype_data = {
	/* i.mx27 cspi shares the functions with i.mx21 one */
	.intctrl = mx21_intctrl,
	.config = mx21_config,
	.trigger = mx21_trigger,
	.rx_available = mx21_rx_available,
	.reset = mx21_reset,
	.devtype = IMX27_CSPI,
};

static struct spi_imx_devtype_data imx31_cspi_devtype_data = {
	.intctrl = mx31_intctrl,
	.config = mx31_config,
	.trigger = mx31_trigger,
	.rx_available = mx31_rx_available,
	.reset = mx31_reset,
	.devtype = IMX31_CSPI,
};

static struct spi_imx_devtype_data imx35_cspi_devtype_data = {
	/* i.mx35 and later cspi shares the functions with i.mx31 one */
	.intctrl = mx31_intctrl,
	.config = mx31_config,
	.trigger = mx31_trigger,
	.rx_available = mx31_rx_available,
	.reset = mx31_reset,
	.devtype = IMX35_CSPI,
};

static struct spi_imx_devtype_data imx51_ecspi_devtype_data = {
	.intctrl = mx51_ecspi_intctrl,
	.config = mx51_ecspi_config,
	.trigger = mx51_ecspi_trigger,
	.rx_available = mx51_ecspi_rx_available,
	.reset = mx51_ecspi_reset,
	.devtype = IMX51_ECSPI,
};

static struct platform_device_id spi_imx_devtype[] = {
	{
		.name = "imx1-cspi",
		.driver_data = (kernel_ulong_t) &imx1_cspi_devtype_data,
	}, {
		.name = "imx21-cspi",
		.driver_data = (kernel_ulong_t) &imx21_cspi_devtype_data,
	}, {
		.name = "imx27-cspi",
		.driver_data = (kernel_ulong_t) &imx27_cspi_devtype_data,
	}, {
		.name = "imx31-cspi",
		.driver_data = (kernel_ulong_t) &imx31_cspi_devtype_data,
	}, {
		.name = "imx35-cspi",
		.driver_data = (kernel_ulong_t) &imx35_cspi_devtype_data,
	}, {
		.name = "imx51-ecspi",
		.driver_data = (kernel_ulong_t) &imx51_ecspi_devtype_data,
	}, {
		/* sentinel */
	}
};

static const struct of_device_id spi_imx_dt_ids[] = {
	{ .compatible = "fsl,imx1-cspi", .data = &imx1_cspi_devtype_data, },
	{ .compatible = "fsl,imx21-cspi", .data = &imx21_cspi_devtype_data, },
	{ .compatible = "fsl,imx27-cspi", .data = &imx27_cspi_devtype_data, },
	{ .compatible = "fsl,imx31-cspi", .data = &imx31_cspi_devtype_data, },
	{ .compatible = "fsl,imx35-cspi", .data = &imx35_cspi_devtype_data, },
	{ .compatible = "fsl,imx51-ecspi", .data = &imx51_ecspi_devtype_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spi_imx_dt_ids);

static void spi_imx_chipselect(struct spi_device *spi, int is_active)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	int gpio = spi_imx->chipselect[spi->chip_select];
	int active = is_active != BITBANG_CS_INACTIVE;
	int dev_is_lowactive = !(spi->mode & SPI_CS_HIGH);

	if (!gpio_is_valid(gpio))
		return;

	gpio_set_value(gpio, dev_is_lowactive ^ active);
}

static void spi_imx_push(struct spi_imx_data *spi_imx)
{
	while (spi_imx->txfifo < spi_imx_get_fifosize(spi_imx)) {
		if (!spi_imx->count)
			break;
		spi_imx->tx(spi_imx);
		spi_imx->txfifo++;
	}

	spi_imx->devtype_data->trigger(spi_imx);
}

static irqreturn_t spi_imx_isr(int irq, void *dev_id)
{
	struct spi_imx_data *spi_imx = dev_id;

	while (spi_imx->devtype_data->rx_available(spi_imx)) {
		spi_imx->rx(spi_imx);
		spi_imx->txfifo--;
	}

	if (spi_imx->count) {
		spi_imx_push(spi_imx);
		return IRQ_HANDLED;
	}

	if (spi_imx->txfifo) {
		/* No data left to push, but still waiting for rx data,
		 * enable receive data available interrupt.
		 */
		spi_imx->devtype_data->intctrl(
				spi_imx, MXC_INT_RR);
		return IRQ_HANDLED;
	}

	spi_imx->devtype_data->intctrl(spi_imx, 0);
	complete(&spi_imx->xfer_done);

	return IRQ_HANDLED;
}

static int spi_imx_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	struct spi_imx_config config;

	config.bpw = t ? t->bits_per_word : spi->bits_per_word;
	config.speed_hz  = t ? t->speed_hz : spi->max_speed_hz;
	config.mode = spi->mode;
	config.cs = spi->chip_select;

	// printk("hndz spi_imx_setupxfer!\n");

	if (!config.speed_hz)
		config.speed_hz = spi->max_speed_hz;
	if (!config.bpw)
		config.bpw = spi->bits_per_word;

	/* Initialize the functions for transfer */
	if (config.bpw <= 8) {
		spi_imx->rx = spi_imx_buf_rx_u8;
		spi_imx->tx = spi_imx_buf_tx_u8;
	} else if (config.bpw <= 16) {
		spi_imx->rx = spi_imx_buf_rx_u16;
		spi_imx->tx = spi_imx_buf_tx_u16;
	} else {
		spi_imx->rx = spi_imx_buf_rx_u32;
		spi_imx->tx = spi_imx_buf_tx_u32;
	}

	spi_imx->devtype_data->config(spi_imx, &config);

	return 0;
}

static void spi_imx_sdma_exit(struct spi_imx_data *spi_imx)
{
	struct spi_master *master = spi_imx->bitbang.master;

	if (master->dma_rx) {
		dma_release_channel(master->dma_rx);
		master->dma_rx = NULL;
	}

	if (master->dma_tx) {
		dma_release_channel(master->dma_tx);
		master->dma_tx = NULL;
	}

	spi_imx->dma_is_inited = 0;
}

static int spi_imx_sdma_init(struct device *dev, struct spi_imx_data *spi_imx,
			     struct spi_master *master,
			     const struct resource *res)
{
	struct dma_slave_config slave_config = {};
	int ret;
//	printk("zty spi sdma init!\n");
	/* Prepare for TX DMA: */
	master->dma_tx = dma_request_slave_channel(dev, "tx");
	if (!master->dma_tx) {
		dev_err(dev, "cannot get the TX DMA channel!\n");
		ret = -EINVAL;
		goto err;
	}

	slave_config.direction = DMA_MEM_TO_DEV;
	slave_config.dst_addr = res->start + MXC_CSPITXDATA;
	slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	slave_config.dst_maxburst = spi_imx_get_fifosize(spi_imx) / 4;
	slave_config.dst_maxburst = 1;
	ret = dmaengine_slave_config(master->dma_tx, &slave_config);
	if (ret) {
		dev_err(dev, "error in TX dma configuration.\n");
		goto err;
	}

	/* Prepare for RX : */
	master->dma_rx = dma_request_slave_channel(dev, "rx");
	if (!master->dma_rx) {
		dev_dbg(dev, "cannot get the DMA channel.\n");
		ret = -EINVAL;
		goto err;
	}

	slave_config.direction = DMA_DEV_TO_MEM;
	slave_config.src_addr = res->start + MXC_CSPIRXDATA;
	slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	slave_config.src_maxburst = spi_imx_get_fifosize(spi_imx) / 2;
	slave_config.src_maxburst = 1;
	ret = dmaengine_slave_config(master->dma_rx, &slave_config);
	if (ret) {
		dev_err(dev, "error in RX dma configuration.\n");
		goto err;
	}

	init_completion(&spi_imx->dma_rx_completion);
	init_completion(&spi_imx->dma_tx_completion);
	master->can_dma = spi_imx_can_dma;
	master->max_dma_len = MAX_SDMA_BD_BYTES;
	spi_imx->bitbang.master->flags = SPI_MASTER_MUST_RX |
					 SPI_MASTER_MUST_TX;
	spi_imx->dma_is_inited = 1;
	//printk("zty spi sdma end!\n");

	return 0;
err:
	spi_imx_sdma_exit(spi_imx);
	return ret;
}

static void spi_imx_dma_rx_callback(void *cookie)
{
	struct spi_imx_data *spi_imx = (struct spi_imx_data *)cookie;

	complete(&spi_imx->dma_rx_completion);

}

static void spi_imx_dma_tx_callback(void *cookie)
{
	struct spi_imx_data *spi_imx = (struct spi_imx_data *)cookie;

	complete(&spi_imx->dma_tx_completion);
}

static void spi_imx_irq_dma_rx_callback(void *cookie)
{
	struct spi_device * spi = (struct spi_device *) cookie;

	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);

	complete(&spi_imx->dma_rx_completion);
}




static void spi_imx_irq_dma_tx_callback(void *cookie)
{

	struct spi_device * spi = (struct spi_device *) cookie;

	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);

	complete(&spi_imx->dma_tx_completion);
}

static int spi_imx_dma_transfer(struct spi_imx_data *spi_imx,
				struct spi_transfer *transfer)
{
	struct dma_async_tx_descriptor *desc_tx = NULL, *desc_rx = NULL;
	int ret;
	int left = 0;
	struct spi_master *master = spi_imx->bitbang.master;
	struct sg_table *tx = &transfer->tx_sg, *rx = &transfer->rx_sg;

	if (tx) {
		desc_tx = dmaengine_prep_slave_sg(master->dma_tx,
					tx->sgl, tx->nents, DMA_TO_DEVICE,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc_tx)
			goto no_dma;

		desc_tx->callback = spi_imx_dma_tx_callback;
		desc_tx->callback_param = (void *)spi_imx;
		dmaengine_submit(desc_tx);
	}

	if (rx) {
		struct scatterlist *sgl_last = &rx->sgl[rx->nents - 1];
		unsigned int	orig_length = sgl_last->length;
		int	wml_mask = ~(spi_imx->rx_wml - 1);
		/*
		 * Adjust the transfer lenth of the last scattlist if there are
		 * some tail data, use PIO read to get the tail data since DMA
		 * sometimes miss the last tail interrupt.
		 */
		left = transfer->len % spi_imx->rx_wml;
		if (left)
			sgl_last->length = orig_length & wml_mask;

		desc_rx = dmaengine_prep_slave_sg(master->dma_rx,
					rx->sgl, rx->nents, DMA_FROM_DEVICE,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc_rx)
			goto no_dma;

		desc_rx->callback = spi_imx_dma_rx_callback;
		desc_rx->callback_param = (void *)spi_imx;
		dmaengine_submit(desc_rx);
	}

	reinit_completion(&spi_imx->dma_rx_completion);
	reinit_completion(&spi_imx->dma_tx_completion);

	/* Trigger the cspi module. */
	spi_imx->dma_finished = 0;

	spi_imx->devtype_data->trigger(spi_imx);

	dma_async_issue_pending(master->dma_tx);
	dma_async_issue_pending(master->dma_rx);
	/* Wait SDMA to finish the data transfer.*/
	ret = wait_for_completion_timeout(&spi_imx->dma_tx_completion,
					  IMX_DMA_TIMEOUT(transfer->len));
	if (!ret) {
		pr_warn("%s %s: I/O Error in DMA TX:%x\n",
			dev_driver_string(&master->dev),
			dev_name(&master->dev), transfer->len);
		dmaengine_terminate_all(master->dma_tx);
	} else {
		ret = wait_for_completion_timeout(&spi_imx->dma_rx_completion,
				IMX_DMA_TIMEOUT(transfer->len));
		if (!ret) {
			pr_warn("%s %s: I/O Error in DMA RX:%x\n",
				dev_driver_string(&master->dev),
				dev_name(&master->dev), transfer->len);
			spi_imx->devtype_data->reset(spi_imx);
			dmaengine_terminate_all(master->dma_rx);
		} else if (left) {
			/* read the tail data by PIO */
			void *tmpbuf = transfer->rx_buf + transfer->len - left;

			while (readl(spi_imx->base + MX51_ECSPI_STAT) & 0x8) {
				*(char *)tmpbuf =
					readl(spi_imx->base + MXC_CSPIRXDATA);
				tmpbuf++;
			}
		}
	}

	spi_imx->dma_finished = 1;
	spi_imx->devtype_data->trigger(spi_imx);

	if (!ret)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		ret = transfer->len;

	return ret;

no_dma:
	pr_warn_once("%s %s: DMA not available, falling back to PIO\n",
		     dev_driver_string(&master->dev),
		     dev_name(&master->dev));
	return -EAGAIN;
}

static int spi_imx_pio_transfer(struct spi_device *spi,
				struct spi_transfer *transfer)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);

	spi_imx->tx_buf = transfer->tx_buf;
	spi_imx->rx_buf = transfer->rx_buf;
	spi_imx->count = transfer->len;
	spi_imx->txfifo = 0;

	reinit_completion(&spi_imx->xfer_done);

	spi_imx_push(spi_imx);

	spi_imx->devtype_data->intctrl(spi_imx, MXC_INT_TE);

	wait_for_completion(&spi_imx->xfer_done);

	return transfer->len;
}

static int spi_imx_transfer(struct spi_device *spi,
				struct spi_transfer *transfer)
{
	int ret;
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);

	if (spi_imx->bitbang.master->can_dma &&
	    spi_imx_can_dma(spi_imx->bitbang.master, spi, transfer)) {
		spi_imx->usedma = true;
		ret = spi_imx_dma_transfer(spi_imx, transfer);
		if (ret != -EAGAIN)
			return ret;
	}
	spi_imx->usedma = false;

	return spi_imx_pio_transfer(spi, transfer);
	return -EAGAIN;
}

static int spi_imx_setup(struct spi_device *spi)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	int gpio = spi_imx->chipselect[spi->chip_select];

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n", __func__,
		 spi->mode, spi->bits_per_word, spi->max_speed_hz);

	if (gpio_is_valid(gpio))
		gpio_direction_output(gpio, spi->mode & SPI_CS_HIGH ? 0 : 1);

	spi_imx_chipselect(spi, BITBANG_CS_INACTIVE);

	return 0;
}

static void spi_imx_cleanup(struct spi_device *spi)
{
}

static int
spi_imx_prepare_message(struct spi_master *master, struct spi_message *msg)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);
	int ret;

	ret = clk_prepare_enable(spi_imx->clk_per);
	if (ret)
		return ret;

	ret = clk_prepare_enable(spi_imx->clk_ipg);
	if (ret) {
		clk_disable_unprepare(spi_imx->clk_per);
		return ret;
	}

	return 0;
}

static int hndz_spi_map_buf(struct spi_master *master, struct device *dev,
		       struct sg_table *sgt, void *buf, size_t len,
		       enum dma_data_direction dir)
{
	const bool vmalloced_buf = is_vmalloc_addr(buf);
	const int desc_len = vmalloced_buf ? PAGE_SIZE : master->max_dma_len;
	const int sgs = DIV_ROUND_UP(len, desc_len);
	struct page *vm_page;
	void *sg_buf;
	size_t min;
	int i, ret;

	ret = sg_alloc_table(sgt, sgs, GFP_KERNEL);
	if (ret != 0)
		return ret;

	for (i = 0; i < sgs; i++) {
		min = min_t(size_t, len, desc_len);

		if (vmalloced_buf) {
			vm_page = vmalloc_to_page(buf);
			if (!vm_page) {
				sg_free_table(sgt);
				return -ENOMEM;
			}
			sg_set_page(&sgt->sgl[i], vm_page,
				    min, offset_in_page(buf));
		} else {
			sg_buf = buf;
			sg_set_buf(&sgt->sgl[i], sg_buf, min);
		}


		buf += min;
		len -= min;
	}

	ret = dma_map_sg(dev, sgt->sgl, sgt->nents, dir);
	if (!ret)
		ret = -ENOMEM;
	if (ret < 0) {
		sg_free_table(sgt);
		return ret;
	}

	sgt->nents = ret;

	return 0;
}

static void hndz_spi_unmap_buf(struct spi_master *master, struct device *dev,
			  struct sg_table *sgt, enum dma_data_direction dir)
{
	if (sgt->orig_nents) {
		dma_unmap_sg(dev, sgt->sgl, sgt->orig_nents, dir);
		sg_free_table(sgt);
	}
}

static int hndz_spi_unmap_msg(struct spi_master *master, struct spi_message *msg)
{
	struct spi_transfer *xfer;
	struct device *tx_dev, *rx_dev;

	if (!master->cur_msg_mapped || !master->can_dma)
		return 0;

	tx_dev = master->dma_tx->device->dev;
	rx_dev = master->dma_rx->device->dev;

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (!master->can_dma(master, msg->spi, xfer))
			continue;

		hndz_spi_unmap_buf(master, rx_dev, &xfer->rx_sg, DMA_FROM_DEVICE);
		hndz_spi_unmap_buf(master, tx_dev, &xfer->tx_sg, DMA_TO_DEVICE);
	}

	return 0;
}

static int hndz_spi_map_msg(struct spi_master *master, struct spi_message *msg)
{
	struct device *tx_dev, *rx_dev;
	struct spi_transfer *xfer;
	int ret;

	if (!master->can_dma)
		return 0;

	tx_dev = master->dma_tx->device->dev;
	rx_dev = master->dma_rx->device->dev;

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (!master->can_dma(master, msg->spi, xfer))
			continue;

		if (xfer->tx_buf != NULL) {
			// printk("hndz xfer tx buf map!\n");
			ret = hndz_spi_map_buf(master, tx_dev, &xfer->tx_sg,
					  (void *)xfer->tx_buf, xfer->len,
					  DMA_TO_DEVICE);
			if (ret != 0)
				return ret;
		}

		if (xfer->rx_buf != NULL) {
			// printk("hndz xfer rx buf map!\n");
			ret = hndz_spi_map_buf(master, rx_dev, &xfer->rx_sg,
					  xfer->rx_buf, xfer->len,
					  DMA_FROM_DEVICE);
			if (ret != 0) {
				printk("hndz xfer rx buf map DMA_TO_DEVICE!\n");
				hndz_spi_unmap_buf(master, tx_dev, &xfer->tx_sg,
					      DMA_TO_DEVICE);
				return ret;
			}
		}
	}

	master->cur_msg_mapped = true;

	return 0;
}




int spi_imx_irq_dma_transfer(struct spi_device * spi, struct spi_transfer *transfer, dma_async_tx_callback rxcallback, dma_async_tx_callback txcallback)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	struct dma_async_tx_descriptor *desc_tx = NULL, *desc_rx = NULL;
	int ret = 0;
	int left = 0;
	struct spi_master *master = spi_imx->bitbang.master;
	struct sg_table *tx = &transfer->tx_sg, *rx = &transfer->rx_sg;

	// pr_info_once("hndz irq first spi tx len %d rx len %d!\n", tx->nents, rx->nents);
	if (tx) {
		desc_tx = dmaengine_prep_slave_sg(master->dma_tx,
					tx->sgl, tx->nents, DMA_TO_DEVICE,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc_tx)
		{
			printk("hndz desc tx error len %d!\n", transfer->len);
			goto irq_no_dma;
		}

		if(txcallback != NULL)
		{
			desc_tx->callback = txcallback;
			desc_tx->callback_param = (void *)spi;
		}
		else
		{
			desc_tx->callback = spi_imx_irq_dma_tx_callback;
			desc_tx->callback_param = (void *)spi;
		}
		
		dmaengine_submit(desc_tx);
	}
	if (rx) {
		struct scatterlist *sgl_last = &rx->sgl[rx->nents - 1];
		unsigned int	orig_length = sgl_last->length;
		int	wml_mask = ~(spi_imx->rx_wml - 1);
		/*
		 * Adjust the transfer lenth of the last scattlist if there are
		 * some tail data, use PIO read to get the tail data since DMA
		 * sometimes miss the last tail interrupt.
		 */
		left = transfer->len % spi_imx->rx_wml;
		if (left)
			sgl_last->length = orig_length & wml_mask;

		desc_rx = dmaengine_prep_slave_sg(master->dma_rx,
					rx->sgl, rx->nents, DMA_FROM_DEVICE,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc_rx)
		{
			printk("hndz desc rx error!\n");
			goto irq_no_dma;
		}
		if(rxcallback != NULL)
		{
			desc_rx->callback = rxcallback;
			desc_rx->callback_param = (void *)spi;
		}
		else
		{
			desc_rx->callback = spi_imx_irq_dma_rx_callback;
			desc_rx->callback_param = (void *)spi;
		}
		
		dmaengine_submit(desc_rx);
	}

	reinit_completion(&spi_imx->dma_rx_completion);
	reinit_completion(&spi_imx->dma_tx_completion);

	/* Trigger the cspi module. */
	spi_imx->dma_finished = 0;

	spi_imx->devtype_data->trigger(spi_imx);

	dma_async_issue_pending(master->dma_tx);
	dma_async_issue_pending(master->dma_rx);
	

	return ret;

irq_no_dma:
	pr_warn_once("%s %s: irq DMA not available, falling back to PIO\n",
		     dev_driver_string(&master->dev),
		     dev_name(&master->dev));
	return -EAGAIN;

}

int spi_imx_dma_wait(struct spi_device * spi, struct spi_transfer *transfer)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	int ret;
	struct spi_master *master = spi_imx->bitbang.master;


	ret = wait_for_completion_timeout(&spi_imx->dma_tx_completion,
					  IMX_DMA_TIMEOUT(transfer->len));
	if (!ret) {
		pr_warn("%s %s: I/O Error in DMA TX:%x state %d\n",
			dev_driver_string(&master->dev),
			dev_name(&master->dev), transfer->len, gSpiStage);
		dmaengine_terminate_all(master->dma_tx);
	} else {
		ret = wait_for_completion_timeout(&spi_imx->dma_rx_completion,
				IMX_DMA_TIMEOUT(transfer->len));
		if (!ret) {
			pr_warn("%s %s: I/O Error in DMA RX:%x\n",
				dev_driver_string(&master->dev),
				dev_name(&master->dev), transfer->len);
			spi_imx->devtype_data->reset(spi_imx);
			dmaengine_terminate_all(master->dma_rx);
		}
		
	}

	spi_imx->dma_finished = 1;
	spi_imx->devtype_data->trigger(spi_imx);

	if (!ret)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		ret = transfer->len;

	ndelay(100);
	spi_imx_chipselect(spi, 0);
	ndelay(100);

	return ret;
}

int spi_imx_sdma_rt_stage1(struct spi_device *spi,struct spi_message *msg, struct spi_transfer *t, dma_async_tx_callback rxcallback, 
							dma_async_tx_callback txcallback)
{
	struct spi_master *master = spi->master;
	int ret = 0;
	
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	msg->spi = spi;

	ret = spi_imx_prepare_message(spi->master, msg);
	if (ret) {
		dev_err(&master->dev,
			"failed to prepare message: %d\n", ret);
		return ret;
	}
	master->cur_msg_prepared = true;
	ret = hndz_spi_map_msg(master, msg);
	if (ret) {
		printk("hndz spi map msg error!\n");
		msg->status = ret;
		return ret;
	}

	spi_imx_setupxfer(spi, t);
	spi_imx_chipselect(spi, 1);
	ndelay(100);

	// ret = spi_imx_transfer(spi, t);
	spi_imx->usedma = true;
	gSpiStage = 1;
	ret = spi_imx_irq_dma_transfer(spi, t, rxcallback, txcallback);

	return ret;
}

int spi_imx_sdma_rt_stage2(struct spi_device *spi,struct spi_message *msg, struct spi_transfer *t)
{
	int ret = 0;
	ret = spi_imx_dma_wait(spi, t);

	if(ret != t->len)
	{
		printk("hndz ret %d len %d!\n", ret, t->len);
		ret = -EREMOTEIO;
	}
	else
		ret = 0;

	hndz_spi_unmap_msg(spi->master, msg);

	// ndelay(100);
	// spi_imx_chipselect(spi, 0);
	// ndelay(100);

	return ret;

}

int spi_imx_sdma_rt(struct spi_device *spi,struct spi_message *msg, struct spi_transfer *t)
{
	// struct spi_master *master = spi->master;
	// int ret = 0;
	
	// struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);

	// msg->spi = spi;

	// ret = spi_imx_prepare_message(spi->master, msg);
	// if (ret) {
	// 	dev_err(&master->dev,
	// 		"failed to prepare message: %d\n", ret);
	// 	return ret;
	// }
	// master->cur_msg_prepared = true;
	// ret = hndz_spi_map_msg(master, msg);
	// if (ret) {
	// 	printk("hndz spi map msg error!\n");
	// 	msg->status = ret;
	// 	return ret;
	// }

	// spi_imx_setupxfer(spi, t);
	// spi_imx_chipselect(spi, 1);
	// ndelay(100);

	// ret = spi_imx_transfer(spi, t);
	// // spi_imx->usedma = true;
	// // spi_imx_irq_dma_transfer(spi, t);
	// // ret = spi_imx_dma_wait(spi, t);

	// if(ret != t->len)
	// {
	// 	ret = -EREMOTEIO;
	// }
	// else
	// 	ret = 0;

	// ndelay(100);
	// spi_imx_chipselect(spi, 0);
	// ndelay(100);

	int ret;
	ret = spi_imx_sdma_rt_stage1(spi, msg, t, NULL, NULL);
	ret = spi_imx_sdma_rt_stage2(spi, msg, t);

	return ret;

}

static int
spi_imx_unprepare_message(struct spi_master *master, struct spi_message *msg)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);

	clk_disable_unprepare(spi_imx->clk_ipg);
	clk_disable_unprepare(spi_imx->clk_per);
	return 0;
}

static int spi_imx_rt_reg(struct spi_device *spi,struct spi_transfer *transfer)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	uint32_t i,count;
	char* rxbuf=(char*)transfer->rx_buf;
	char* txbuf=(char*)transfer->tx_buf;

	// writel(0, spi_imx->base + MX51_ECSPI_DMA);
	spi_imx->usedma = false;
	for ( i = 0; i < transfer->len; i++)
	{
		count = 0;
		while((readl(spi_imx->base + MX51_ECSPI_STAT) & (1 << 0)) == 0){
			count++;
			if (count == 0x000000F8){
				printk("www spi_imx_rt_reg tx timeout\n");
				break;
			}
		}
		writel(txbuf[i], spi_imx->base + MXC_CSPITXDATA);
		
		spi_imx->devtype_data->trigger(spi_imx);

		count = 0;
		while((readl(spi_imx->base + MX51_ECSPI_STAT) & (1 << 3)) == 0){
			count++;
			if (count == 0x000000F8){
				printk("www spi_imx_rt_reg rx timeout\n");
				break;
			}
		}
		rxbuf[i] = readl(spi_imx->base + MXC_CSPIRXDATA);

	}
	return 0;
}

int spi_imx_rt(struct spi_device *spi,struct spi_message *msg,struct spi_transfer *t)
{

	spi_imx_setupxfer(spi,t);
	spi_imx_chipselect(spi,1);
	ndelay(100);
	

	spi_imx_rt_reg(spi,t);

	ndelay(100);
	spi_imx_chipselect(spi,0);
	ndelay(100);

	return 0;
}

static int spi_imx_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
			of_match_device(spi_imx_dt_ids, &pdev->dev);
	struct spi_imx_master *mxc_platform_info =
			dev_get_platdata(&pdev->dev);
	struct spi_master *master;
	struct spi_imx_data *spi_imx;
	struct resource *res;
	int i, ret, num_cs;

	if (!np && !mxc_platform_info) {
		dev_err(&pdev->dev, "can't get the platform data\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "fsl,spi-num-chipselects", &num_cs);
	if (ret < 0) {
		if (mxc_platform_info)
			num_cs = mxc_platform_info->num_chipselect;
		else
			return ret;
	}

	master = spi_alloc_master(&pdev->dev,
			sizeof(struct spi_imx_data) + sizeof(int) * num_cs);
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 32);
	master->bus_num = pdev->id;
	master->num_chipselect = num_cs;

	spi_imx = spi_master_get_devdata(master);
	spi_imx->bitbang.master = master;

	for (i = 0; i < master->num_chipselect; i++) {
		int cs_gpio = of_get_named_gpio(np, "cs-gpios", i);
		if (!gpio_is_valid(cs_gpio) && mxc_platform_info)
			cs_gpio = mxc_platform_info->chipselect[i];

		spi_imx->chipselect[i] = cs_gpio;
		if (!gpio_is_valid(cs_gpio))
			continue;

		ret = devm_gpio_request(&pdev->dev, spi_imx->chipselect[i],
					DRIVER_NAME);
		if (ret) {
			dev_err(&pdev->dev, "can't get cs gpios\n");
			goto out_master_put;
		}
	}

	spi_imx->bitbang.chipselect = spi_imx_chipselect;
	spi_imx->bitbang.setup_transfer = spi_imx_setupxfer;
	spi_imx->bitbang.txrx_bufs = spi_imx_transfer;
	spi_imx->bitbang.master->setup = spi_imx_setup;
	spi_imx->bitbang.master->cleanup = spi_imx_cleanup;
	spi_imx->bitbang.master->prepare_message = spi_imx_prepare_message;
	spi_imx->bitbang.master->unprepare_message = spi_imx_unprepare_message;
	spi_imx->bitbang.master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	init_completion(&spi_imx->xfer_done);

	spi_imx->devtype_data = of_id ? of_id->data :
		(struct spi_imx_devtype_data *) pdev->id_entry->driver_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spi_imx->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spi_imx->base)) {
		ret = PTR_ERR(spi_imx->base);
		goto out_master_put;
	}

	spi_imx->irq = platform_get_irq(pdev, 0);
	if (spi_imx->irq < 0) {
		ret = spi_imx->irq;
		goto out_master_put;
	}

	ret = devm_request_irq(&pdev->dev, spi_imx->irq, spi_imx_isr, 0,
			       dev_name(&pdev->dev), spi_imx);
	if (ret) {
		dev_err(&pdev->dev, "can't get irq%d: %d\n", spi_imx->irq, ret);
		goto out_master_put;
	}

	spi_imx->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(spi_imx->clk_ipg)) {
		ret = PTR_ERR(spi_imx->clk_ipg);
		goto out_master_put;
	}

	spi_imx->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(spi_imx->clk_per)) {
		ret = PTR_ERR(spi_imx->clk_per);
		goto out_master_put;
	}

	ret = clk_prepare_enable(spi_imx->clk_per);
	if (ret)
		goto out_master_put;

	ret = clk_prepare_enable(spi_imx->clk_ipg);
	if (ret)
		goto out_put_per;

	spi_imx->spi_clk = clk_get_rate(spi_imx->clk_per);
	/*
	 * Only validated on i.mx6 now, can remove the constrain if validated on
	 * other chips.
	 */
	if (spi_imx->devtype_data == &imx51_ecspi_devtype_data
	    && spi_imx_sdma_init(&pdev->dev, spi_imx, master, res))
		dev_err(&pdev->dev, "dma setup error,use pio instead\n");

	spi_imx->devtype_data->reset(spi_imx);

	spi_imx->devtype_data->intctrl(spi_imx, 0);

	master->dev.of_node = pdev->dev.of_node;
	ret = spi_bitbang_start(&spi_imx->bitbang);
	if (ret) {
		dev_err(&pdev->dev, "bitbang start failed with %d\n", ret);
		goto out_clk_put;
	}

	dev_info(&pdev->dev, "probed\n");

	// clk_disable_unprepare(spi_imx->clk_ipg);
	// clk_disable_unprepare(spi_imx->clk_per);
	return ret;

out_clk_put:
	clk_disable_unprepare(spi_imx->clk_ipg);
out_put_per:
	clk_disable_unprepare(spi_imx->clk_per);
out_master_put:
	spi_master_put(master);

	return ret;
}

static int spi_imx_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);

	spi_bitbang_stop(&spi_imx->bitbang);

	writel(0, spi_imx->base + MXC_CSPICTRL);
	clk_unprepare(spi_imx->clk_ipg);
	clk_unprepare(spi_imx->clk_per);
	spi_imx_sdma_exit(spi_imx);
	spi_master_put(master);

	return 0;
}

static struct platform_driver spi_imx_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = spi_imx_dt_ids,
		   },
	.id_table = spi_imx_devtype,
	.probe = spi_imx_probe,
	.remove = spi_imx_remove,
};
module_platform_driver(spi_imx_driver);

MODULE_DESCRIPTION("SPI Master Controller driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
