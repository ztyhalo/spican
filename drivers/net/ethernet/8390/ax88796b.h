/* Generic AX88796B register definitions. */
/* This file is part of AX88796B drivers, and is distributed
   under the same license.*/

#ifndef _ax88796_h
#define _ax88796_h

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/if_vlan.h>
#include <linux/param.h>
#include <linux/ethtool.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>

#include <linux/platform_device.h>

#define TX_PAGES		12

#define TX_QUEUE_CNT		100

#define AX88796B_IO_EXTENT	0x8000000

#define NESM_START_PG		0x40	/* First page of TX buffer */
#define NESM_RX_START_PG	(NESM_START_PG + TX_PAGES)	/* First page of RX buffer */

#define NESM_STOP_PG		0x80	/* Last page +1 of RX ring */

#define ETHER_ADDR_LEN		6

#define AX88796B_BASE		0x08000000

/* The 796b specific per-packet-header format. */
struct ax_pkt_hdr {
  unsigned char status; /* status */
  unsigned char next;   /* pointer to next packet. */
  unsigned short count; /* header + packet length in bytes */
};

enum watchdog_state {
	chk_link = 0,			/* Routine A */
	chk_cable_exist,		/* Called by A */
	chk_cable_exist_again,		/* Routine B */
	phy_power_up,			/* Called by B */
	chk_cable_status,		/* Routine C */
	wait_autoneg,
};

/* Most of these entries should be in 'struct net_device' (or most of the
   things in there should be here!) */
/* You have one of these per-board */
struct ax_device {
	void			*membase;
	struct net_device	*ndev;
	unsigned char		bus_width;
	unsigned char		mcfilter[8];
	unsigned char		media;
	unsigned char		tx_curr_page, tx_prev_ctepr, tx_full;
	unsigned char		tx_start_page, tx_stop_page,rx_start_page, stop_page;
	unsigned char		current_page;	/* Read pointer in buffer  */
	spinlock_t		page_lock;		/* Page register locks */
	struct timer_list	watchdog;
	struct net_device_stats stat;		/* The new statistics table. */
	unsigned irqlock:1;
	unsigned dmaing:1;

	struct mii_if_info	mii;
	enum watchdog_state	state;
	int			tick_times;
	u16			phy_advertise;
	u16			phy_bmcr;

	u8			wol;
	struct napi_struct 	napi;
	u8			intr;
	struct sk_buff 		*tx_queue;
#	define WAKEUP_MP	0x01
#	define WAKEUP_LSCWE	0x02
};

#define AX88796_WATCHDOG_PERIOD_50MS		(HZ / 20)
#define AX88796_WATCHDOG_PERIOD_500MS		(HZ / 2)
#define AX88796_WATCHDOG_PERIOD_3S		(HZ * 3)

/* Some generic ethernet register configurations. */

#define E8390_RXCONFIG		0x0	/* EN0_RXCR: broadcasts, no multicast,errors */
#define E8390_RXOFF		0x20	/* EN0_RXCR: Accept no packets */
#define E8390_TXCONFIG		0x80	/* EN0_TXCR: Normal transmit mode */
#define E8390_TXOFF		0x02	/* EN0_TXCR: Transmitter off */

/*  Register accessed at EN_CMD, the 8390 base addr.  */
#define E8390_STOP		0x01   /* Stop and reset the chip */
#define E8390_START		0x02   /* Start the chip, clear reset */
#define E8390_TRANS		0x04   /* Transmit a frame */
#define E8390_RREAD		0x08   /* Remote read */
#define E8390_RWRITE		0x10   /* Remote write  */
#define E8390_NODMA		0x20   /* Remote DMA */
#define E8390_PAGE0		0x00   /* Select page chip registers */
#define E8390_PAGE1		0x40   /* using the two high-order bits */
#define E8390_PAGE2		0x80   /* Page 2 is invalid. */
#define E8390_PAGE3		0xc0   /* Page 3 for AX88796B */
#define E8390_PAGE_MASK		0x3F   /* Mask of PAGES */

#define EI_SHIFT(x)		((x) << 1)

#define E8390_CMD		EI_SHIFT(0x00)  /* The command register (for all pages) */
/* Page 0 register offsets. */
#define EN0_CLDALO		EI_SHIFT(0x01)	/* Low byte of current local dma addr  RD */
#define EN0_STARTPG		EI_SHIFT(0x01)	/* Starting page of ring bfr WR */
#define EN0_CLDAHI		EI_SHIFT(0x02)	/* High byte of current local dma addr  RD */
#define EN0_STOPPG		EI_SHIFT(0x02)	/* Ending page +1 of ring bfr WR */
#define EN0_BOUNDARY		EI_SHIFT(0x03)	/* Boundary page of ring bfr RD WR */
#define EN0_TSR			EI_SHIFT(0x04)	/* Transmit status reg RD */
#define EN0_TPSR		EI_SHIFT(0x04)	/* Transmit starting page WR */
#define EN0_NCR			EI_SHIFT(0x05)	/* Number of collision reg RD */
#define EN0_TCNTLO		EI_SHIFT(0x05)	/* Low  byte of tx byte count WR */
#define EN0_FIFO		EI_SHIFT(0x06)	/* FIFO RD */
#define EN0_TCNTHI		EI_SHIFT(0x06)	/* High byte of tx byte count WR */
#define EN0_CURPAG		EI_SHIFT(0x06)	/* RX current page */
#define EN0_ISR			EI_SHIFT(0x07)	/* Interrupt status reg RD WR */
#define EN0_CRDALO		EI_SHIFT(0x08)	/* low byte of current remote dma address RD */
#define EN0_RSARLO		EI_SHIFT(0x08)	/* Remote start address reg 0 */
#define EN0_CRDAHI		EI_SHIFT(0x09)	/* high byte, current remote dma address RD */
#define EN0_RSARHI		EI_SHIFT(0x09)	/* Remote start address reg 1 */
#define EN0_RCNTLO		EI_SHIFT(0x0a)	/* Remote byte count reg WR */
#define EN0_RCNTHI		EI_SHIFT(0x0b)	/* Remote byte count reg WR */
#define EN0_RSR			EI_SHIFT(0x0c)	/* rx status reg RD */
#define EN0_RXCR		EI_SHIFT(0x0c)	/* RX configuration reg WR */
#define EN0_TXCR		EI_SHIFT(0x0d)	/* TX configuration reg WR */
#define EN0_COUNTER0		EI_SHIFT(0x0d)	/* Rcv alignment error counter RD */
#define EN0_DCFG		EI_SHIFT(0x0e)	/* Data configuration reg WR */
#define EN0_COUNTER1		EI_SHIFT(0x0e)	/* Rcv CRC error counter RD */
#define EN0_IMR			EI_SHIFT(0x0f)	/* Interrupt mask reg WR */
#define EN0_COUNTER2		EI_SHIFT(0x0f)	/* Rcv missed frame error counter RD */
#define EN0_DATAPORT		EI_SHIFT(0x10)
#define EN0_PHYID		EI_SHIFT(0x10)
#define AX88796_MII_EEPROM	EI_SHIFT(0x14)
#define EN0_BTCR		EI_SHIFT(0x15)	/* Buffer Type Configure Register */
#define EN0_SR			EI_SHIFT(0X17)	/* AX88796B Status Register */
#define EN0_FLOW		EI_SHIFT(0x1a)	/* AX88796B Flow control register */
#define EN0_MCR			EI_SHIFT(0X1b)  /* Mac configure register */
#define EN0_CTEPR		EI_SHIFT(0x1c)	/* Current TX End Page */
#define EN0_VID0		EI_SHIFT(0x1c)	/* VLAN ID 0 */
#define EN0_VID1		EI_SHIFT(0x1d)	/* VLAN ID 1 */
#define EN0_RESET		EI_SHIFT(0X1f)	/* Issue a read to reset, a write to clear. */

#define EN0_DATA_ADDR		0x0800
#define EN0_DATA_ADDR_T		0x0040


#define ENVLAN_ENABLE		0x08

/* Bits in EN0_ISR - Interrupt status register */
#define ENISR_RX		0x01	/* Receiver, no error */
#define ENISR_TX		0x02	/* Transmitter, no error */
#define ENISR_RX_ERR		0x04	/* Receiver, with error */
#define ENISR_TX_ERR		0x08	/* Transmitter, with error */
#define ENISR_OVER		0x10	/* Receiver overwrote the ring */
#define ENISR_COUNTERS		0x20	/* Counters need emptying */
#define ENISR_RDC		0x40	/* remote dma complete */
#define ENISR_RESET		0x80	/* Reset completed */
#define ENISR_ALL		(ENISR_RX | ENISR_TX | ENISR_RX_ERR | ENISR_TX_ERR | ENISR_OVER | ENISR_COUNTERS)/* Interrupts we will enable */

/* Bits in EN0_CTEPR */
#define ENCTEPR_TXCQF		0x80	/* Transmit command queue full */

/* Bits in EN0_DCFG - Data config register */
#define ENDCFG_WTS	0x01	/* word transfer mode selection */
#define ENDCFG_BOS	0x02	/* byte order selection */

#define ENFLOW_ENABLE	0xc7		/* Flow Control Control Register */
#define ENTQC_ENABLE    0x20		/* Enable TXQ */

#define EN3_TBR         EI_SHIFT(0x0d)	/* Transmit Buffer Ring Control Register */
#define ENTBR_ENABLE    0x01			/* Enable Transmit Buffer Ring */

/* Page 1 register offsets. */
#define EN1_PHYS            EI_SHIFT(0x01)  /* This board's physical enet addr RD WR */
#define EN1_PHYS_SHIFT(i)   EI_SHIFT(i+1)   /* Get and set mac address */
#define EN1_CURPAG          EI_SHIFT(0x07)  /* Current memory page RD WR */
#define EN1_MULT            EI_SHIFT(0x08)  /* Multicast filter mask array (8 bytes) RD WR */
#define EN1_MULT_SHIFT(i)   EI_SHIFT(8+i)   /* Get and set multicast filter */

/* Bits in received packet status byte and EN0_RSR*/
#define ENRSR_RXOK      0x01	/* Received a good packet */
#define ENRSR_CRC       0x02	/* CRC error */
#define ENRSR_FAE       0x04	/* frame alignment error */
#define ENRSR_FO        0x08	/* FIFO overrun */
#define ENRSR_MPA       0x10	/* missed pkt */
#define ENRSR_PHY       0x20	/* physical/multicast address */
#define ENRSR_DIS       0x40	/* receiver disable. set in monitor mode */
#define ENRSR_DEF       0x80	/* deferring */

/* Transmitted packet status, EN0_TSR. */
#define ENTSR_PTX       0x01   /* Packet transmitted without error */
#define ENTSR_ND        0x02   /* The transmit wasn't deferred. */
#define ENTSR_COL       0x04   /* The transmit collided at least once. */
#define ENTSR_ABT       0x08   /* The transmit collided 16 times, and was deferred. */
#define ENTSR_CRS       0x10   /* The carrier sense was lost. */
#define ENTSR_FU        0x20   /* A "FIFO underrun" occurred during transmit. */
#define ENTSR_CDH       0x40   /* The collision detect "heartbeat" signal was lost. */
#define ENTSR_OWC       0x80   /* There was an out-of-window collision. */

/* Bits in buffer type configure register */
#define ENBTCR_PME_INT_EN	0x40	/* PME interrupt enable */
#define ENBTCR_INT_ACT_HIGH	0x10
#define ENBTCR_PME_PULL		0x02
#define ENBTCR_PME_ACT_HIGH	0x01

/* Bits in device status register, EN0_SR */
#define ENSR_DMA_DONE		0x40	/* Remote DMA completed */
#define ENSR_DMA_READY		0x20	/* Remote DMA ready */
#define ENSR_DEV_READY		0x10	/* Device ready */
#define ENSR_SPEED_100		0x04	/* PHY link at 100 Mb/s */
#define ENSR_DUPLEX_DULL	0x02	/* PHY link at full duplex */
#define ENSR_LINK		0x01	/* PHY link up */

/* Power Management register offsets. */
#define EN3_BM0         EI_SHIFT(0x01)
#define EN3_BM1         EI_SHIFT(0x02)
#define EN3_BM2         EI_SHIFT(0x03)
#define EN3_BM3         EI_SH2IFT(0x04)
#define EN3_BM10CRC     EI_SHIFT(0x05)
#define EN3_BM32CRC     EI_SHIFT(0x06)
#define EN3_BMOFST      EI_SHIFT(0x07)
#define EN3_LSTBYT      EI_SHIFT(0x08)
#define EN3_BMCD        EI_SHIFT(0x09)
#define EN3_WUCS        EI_SHIFT(0x0a)
#define EN3_PMR         EI_SHIFT(0x0b)

/* Bits in Wake up Control */
#define ENWUCS_MPEN	0x01
#define ENWUCS_WUEN	0x02
#define ENWUCS_LSCWE	0x04
#define ENWUCS_MPR	0x10
#define ENWUCS_WUFR	0x20
#define ENWUCS_LSC	0x40

/* Bits in PM Control */
#define ENPMR_D1	0x01
#define ENPMR_D2	0x02

#define MEDIA_AUTO      0
#define MEDIA_100FULL   1
#define MEDIA_100HALF   2
#define MEDIA_10FULL    3
#define MEDIA_10HALF    4

/* Debug Message Display Level Definition */
#define DRIVER_MSG      0x0001
#define INIT_MSG        0x0002
#define TX_MSG          0x0004
#define RX_MSG          0x0008
#define INT_MSG         0x0010
#define ERROR_MSG       0x0020
#define WARNING_MSG     0x0040
#define DEBUG_MSG       0x0080
#define OTHERS_MSG      0x0100
#define ALL_MSG         0x01FF
#define NO_MSG          0x0000
#define DEFAULT_MSG     (DRIVER_MSG | ERROR_MSG) 
#define DEBUG_FLAGS     DEFAULT_MSG

#endif /* _8390_h */
