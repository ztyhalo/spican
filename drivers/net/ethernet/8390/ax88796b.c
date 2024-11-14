/*
 * ============================================================================
 * ax88796b.c: Linux2.6.x Ethernet device driver for ASIX AX88796B chip.
 *
 * This program is free software; you can distrine_block_inputbute it and/or
 * modify it under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * This program is based on
 *
 *    ne.c:    A general non-shared-memory NS8390 ethernet driver for linux
 *             Written 1992-94 by Donald Becker.
 *
 *    8390. c: A general NS8390 ethernet driver core for linux.
 *             Written 1992-94 by Donald Becker.
 *
 * Modify: For Linux3.14.28,  yuanwenlin@zlg.cn,  (2017.11.2)
 * ============================================================================
 */
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <linux/atmdev.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/libata.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
//#include <mach/mx6.h>
#include <linux/of_gpio.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/inetdevice.h>
#include <linux/device.h>
#include "ax88796b.h"

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/platform_data/dma-imx.h>

/* NAMING CONSTANT DECLARATIONS */
#define DRV_NAME	"AX88796B"
#define ADP_NAME	"ASIX AX88796B Ethernet Adapter"
#define DRV_VERSION	"2.2.0"
#define PFX		DRV_NAME ": "

#define	PRINTK(flag, args...) if (flag & DEBUG_FLAGS) printk(args)

/*
 * #define DEBUG_PRI(args...)	printk("%s, %d, (%s, %s)+++\n", __func__, __LINE__, __TIME__, __DATE__);\
		printk(args);
*/
#define DEBUG_PRI(args...)
#define MEM_CS0		"mem-cs0"
#define REG_EIM		"reg-eim"

#define CONFIG_AX88796B_USE_MEMCPY		0//1 ck
//#define CONFIG_AX88796B_8BIT_WIDE		1 //0
#define CONFIG_AX88796B_8BIT_WIDE		0 //ck 
#define CONFIG_AX88796B_EEPROM_READ_WRITE	0
#define ADDR_SHIFT16(add)	((unsigned int)add << 16) //ck

/* LOCAL VARIABLES DECLARATIONS */
static char version[] =
KERN_INFO ADP_NAME ":v" DRV_VERSION "\n";

//  #define AX88796_SDMA_MODE 1

struct imx_weim_devtype {
	unsigned int	cs_index;
	unsigned int	cs_regs_count;
	unsigned int	cs_stride;
};



static unsigned int media = 0;
static int mem = 0;
static int irq = 0;
static int weight = 0;

// extern unsigned long hndz_read_current_timer(void);

// static unsigned long gtime1, gtime2, gtime3;

static u8 gctepr=0, gfree_pages=0, gneed_pages =0;

static struct boardcast_info sbcast_info;

#define AX88796_WIGH  8


module_param (mem, int, 0);
module_param (irq, int, 0);
module_param (media, int, 0);
module_param (weight, int, 0); 

MODULE_PARM_DESC (mem, "MEMORY base address(es),required");
MODULE_PARM_DESC (irq, "IRQ number(s)");
MODULE_PARM_DESC (media,
	"Media Mode(0=auto, 1=100full, 2=100half, 3=10full, 4=10half)");
MODULE_PARM_DESC (weight, "NAPI weight");

MODULE_DESCRIPTION ("ASIX AX88796B Fast Ethernet driver");
MODULE_LICENSE ("GPL");

#define AX88796_RESET			IMX_GPIO_NR(6, 6)

static ssize_t show_boardcast_mark(struct device *dev , struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", sbcast_info.bcast_mark);
}

static ssize_t set_boardcast_mark(struct device *dev , struct device_attribute * attr, const char * buf, size_t len)
{
	int value = 0;
	value  = simple_strtoul(buf, NULL, 0);
	if(value)
	{
		if(sbcast_info.bcast_max == 0)
			sbcast_info.bcast_max = 2048;
		if(sbcast_info.bcast_time <=50 )
		{
			sbcast_info.bcast_time = 5000;
		}

	}
	sbcast_info.bcast_mark = value;
	return len;
}
static struct device_attribute device_attr_bcast_mark =
	__ATTR(boardcast_mark,  S_IWUSR|S_IRUSR, show_boardcast_mark,  set_boardcast_mark);

static ssize_t show_boardcast_max(struct device *dev , struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", sbcast_info.bcast_max);
}

static ssize_t set_boardcast_max(struct device *dev , struct device_attribute * attr, const  char * buf, size_t len)
{
	int value = 0;
	value  = simple_strtoul(buf, NULL, 0);
	sbcast_info.bcast_max = value;
	return len;
}

static struct device_attribute device_attr_bcast_max =
	__ATTR(boardcast_max,  S_IWUSR|S_IRUSR, show_boardcast_max,  set_boardcast_max);

static ssize_t show_boardcast_time(struct device *dev , struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", sbcast_info.bcast_time);
}

static ssize_t set_boardcast_time(struct device *dev , struct device_attribute * attr, const char * buf, size_t len)
{
	int value = 0;
	value  = simple_strtoul(buf, NULL, 0);
	sbcast_info.bcast_time = value;
	return len;
}

static struct device_attribute device_attr_bcast_time =
	__ATTR(boardcast_time,  S_IWUSR|S_IRUSR, show_boardcast_time,  set_boardcast_time);

// static DEVICE_ATTR(boardcast_mark, S_IWUSR|S_IRUSR, show_boardcast_mark, set_boardcast_mark);
// static DEVICE_ATTR(boardcast_max, S_IWUSR|S_IRUSR, show_boardcast_max, set_boardcast_max);
// static DEVICE_ATTR(boardcast_time, S_IWUSR|S_IRUSR, show_boardcast_time, set_boardcast_time);
static inline void boardcast_inifo_init(void)
{
	sbcast_info.bcast_mark = 1;
	sbcast_info.bcast_max = 2048;
	sbcast_info.bcast_time = 5000;
}

static int ax_get_link (struct ax_device *ax_local);
static void 
ax_trigger_send (struct net_device *ndev, unsigned int length, int start_page);

#if (CONFIG_AX88796B_8BIT_WIDE == 1)
static inline u16 READ_FIFO (void *membase)
{
	return (readb (membase) | (((u16)readb (membase)) << 8));
}

static inline void WRITE_FIFO (void *membase, u16 data)
{
	writeb ((u8)data , membase);
	writeb ((u8)(data >> 8) , membase);
}
#else
static inline u16 READ_FIFO (void *membase)
{
	 return readw (membase);
	// return *((u16*)(membase));
}
static inline u16 zREAD_FIFO (void *membase)
{
	return *(const volatile u16 __force *) (membase);
}

static inline void WRITE_FIFO (void *membase, u16 data)
{
	writew (data, membase);
}
static inline void zWRITE_FIFO (void *membase, u16 data)
{
// *((u16*)(ax_base + ADDR_SHIFT16(EN0_DATAPORT)))=*((u16 *)(buf + i));
	*(volatile u16 __force *)(membase) = data;
}

#endif

static inline struct ax_device *ax_get_priv (struct net_device *ndev)
{
	return (struct ax_device *) netdev_priv (ndev);
}
static int __init etherm_addr(char *addr)
{
	unsigned int serial;

	if (system_serial_low == 0 && system_serial_high == 0)
	{
		printk("hndz serial error!\n");
		return -ENODEV;
	}

	serial = system_serial_low;

	printk("hndz serial 0x%x!\n", serial);

	addr[0] = 2;
	addr[1] = 0x12;
	addr[2] = (serial >> 24);
	addr[3] = serial >> 16;
	addr[4] = serial >> 8;
	addr[5] = serial;
	return 0;
}
/*  
 *  ======================================================================
 *   MII interface support
 *  ======================================================================
 */
#define MDIO_SHIFT_CLK		0x01
#define MDIO_DATA_WRITE0	0x00
#define MDIO_DATA_WRITE1	0x08
#define MDIO_DATA_READ		0x04
#define MDIO_MASK		0x0f
#define MDIO_ENB_IN		0x02



#define EIM_CS0_PHY_START_ADDR   0x08000000

static struct dma_chan *dma_m2m_rxchan = NULL;
static struct dma_chan *dma_m2m_txchan = NULL;
// static struct completion dma_m2m_tx_ok;				//DMA传输完成等待量
// static struct completion dma_m2m_rx_ok;				//DMA传输完成等待量
static unsigned char * gEIMrxbuf = NULL;
static unsigned char * gEIMtxbuf = NULL;
static unsigned int    eim_rx_length;
static unsigned int    eim_tx_length;

static struct ax_pkt_hdr grx_frame;

static dma_addr_t gdma_dst = 0;							//EIM DMA接收数据的目的地址	接收数据的源地址肯定是EIM的硬件映射地址
static dma_addr_t gdma_src = 0;							//EIM DMA发送数据的源地址		发送数据的目的地址肯定是EIM的硬件映射地址

static int gAxRxTx_state = 0;
static u8 CurrImr;
static int gAxStop = 0;

static void ax88796b_get_hdr(struct net_device *ndev, struct ax_pkt_hdr *hdr, int ring_page);
static void ax_tx_intr (struct net_device *ndev);
static void ax_tx_err (struct net_device *ndev);
static void ax_block_output (struct net_device *ndev, int count,  const unsigned char *buf, const int start_page);
static  int eim_sdma_tx_start(struct net_device *ndev, int count, const unsigned char *buf, const int start_page);

static bool dma_m2m_filter(struct dma_chan *chan, void *param)
{
	if (!imx_dma_is_general_purpose(chan))
		return false;
	chan->private = param;
	return true;
}


static inline void eim_sdma_rx_init_config(void)
{
	struct dma_slave_config slave_config = {};
	// memset(&eim_dma_m2m_config, 0x00, sizeof(struct dma_slave_config));

	// gdma_dst = dma_map_single(NULL, gEIMrxbuf, 2048, DMA_DEV_TO_MEM); //map 映射

	slave_config.direction = DMA_DEV_TO_MEM;
	slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	slave_config.src_addr = (EIM_CS0_PHY_START_ADDR + ADDR_SHIFT16(EN0_DATAPORT));
	slave_config.dst_addr = gdma_dst;

	dmaengine_slave_config(dma_m2m_rxchan, &slave_config);

}

static inline void eim_sdma_tx_init_config(void)
{
	struct dma_slave_config slave_config = {};

	// gdma_src = dma_map_single(NULL, gEIMtxbuf, 2048, 	DMA_MEM_TO_DEV); //map 映射

	slave_config.direction = DMA_MEM_TO_DEV;
	slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	slave_config.src_addr = gdma_src;
	slave_config.dst_addr = (EIM_CS0_PHY_START_ADDR + ADDR_SHIFT16(EN0_DATAPORT));

	dmaengine_slave_config(dma_m2m_txchan, &slave_config);

}

static int req_sdma_channel(void)
{
	dma_cap_mask_t dma_m2m_mask;
	struct imx_dma_data m2m_dma_data = {0};

	dma_cap_zero(dma_m2m_mask);
	dma_cap_set(DMA_SLAVE, dma_m2m_mask);
	m2m_dma_data.peripheral_type = IMX_DMATYPE_MEMORY;
	m2m_dma_data.priority = DMA_PRIO_HIGH;

	dma_m2m_rxchan = dma_request_channel(dma_m2m_mask, dma_m2m_filter, &m2m_dma_data);
	if (!dma_m2m_rxchan) {
		printk("Error opening the SDMA memory to memory channel\n");
		return -EINVAL;
	}
	printk("hndz req sdma mem channel is %d!\n", m2m_dma_data.dma_request);

	dma_m2m_txchan = dma_request_channel(dma_m2m_mask, dma_m2m_filter, &m2m_dma_data);
	if (!dma_m2m_txchan) {
		printk("dma_m2m_txchan Error opening the SDMA memory to memory channel\n");
		return -EINVAL;
	}
	else
	{
		printk("hndz req dma_m2m_txchan ok!\n");
	}

	eim_sdma_rx_init_config();
	eim_sdma_tx_init_config();
	return 0;
}

static int eim_sdma_init(struct net_device *ndev)
{
	gEIMrxbuf = dma_alloc_coherent(NULL, 2048, &gdma_dst, GFP_KERNEL);
	// gEIMrxbuf = kzalloc(2048, GFP_DMA);
	if(!gEIMrxbuf)
	{
		printk("hndz eim rx buf alloc error!!!\n");
		return -1;
	}
	gEIMtxbuf = dma_alloc_coherent(NULL, 2048, &gdma_src, GFP_KERNEL);
	// gEIMtxbuf = kzalloc(2048, GFP_DMA);
	if(!gEIMtxbuf)
	{
		printk("hndz eim tx buf alloc error!!!\n");
		return -1;
	}
	return req_sdma_channel();
}

static int ax88796b_send_data_process (struct sk_buff *skb, struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	int send_length;

	u8 ctepr=0, free_pages=0, need_pages;

	send_length = skb->len;

	writeb (E8390_PAGE0 | E8390_NODMA, ax_base + ADDR_SHIFT16(E8390_CMD));

	writeb (0x00, ax_base + ADDR_SHIFT16(EN0_IMR));
	ax_local->irqlock = 1;

	need_pages = ((send_length - 1) >> 8) + 1;

	ctepr = readb (ax_base + ADDR_SHIFT16(EN0_CTEPR));

	if (ctepr & ENCTEPR_TXCQF) {
		free_pages = 0;
	} else if (ctepr == 0) {

		/*
		 * If someone issues STOP bit of command register at run time,
		 * the TX machine will stop the current transmission and the
		 * CTEPR register will be reset to zero.
		 * This is just precautionary measures.
		 */
		if (ax_local->tx_prev_ctepr != 0) {
			ax_local->tx_curr_page = ax_local->tx_start_page;
			ax_local->tx_prev_ctepr = 0;
		}

		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page;
	}
	else if (ctepr < ax_local->tx_curr_page - 1) {
		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page + 
					ctepr - ax_local->tx_start_page + 1;
	}
	else if (ctepr > ax_local->tx_curr_page - 1) {
		free_pages = ctepr + 1 - ax_local->tx_curr_page;
	}
	else if (ctepr == ax_local->tx_curr_page - 1) {
		if (ax_local->tx_full)
			free_pages = 0;
		else
			free_pages = TX_PAGES;
	}


	if (free_pages < need_pages) {
		printk("hndz process free_pages < need_pages\n");
		netif_stop_queue (ndev);
		ax_local->tx_full = 1;
		ax_local->irqlock = 0;
		ax_local->tx_skb = NULL;	
		writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));
		return NETDEV_TX_BUSY;
	}
	ax_block_output (ndev, send_length, skb->data, ax_local->tx_curr_page);
	ax_trigger_send (ndev, send_length, ax_local->tx_curr_page);
	if (free_pages == need_pages) {
		netif_stop_queue (ndev);
		ax_local->tx_full = 1;
	}
	ax_local->tx_prev_ctepr = ctepr;
	ax_local->tx_curr_page = ((ax_local->tx_curr_page + need_pages) <
		ax_local->tx_stop_page) ? 
		(ax_local->tx_curr_page + need_pages) : 
		(need_pages - (ax_local->tx_stop_page - ax_local->tx_curr_page)
		 + ax_local->tx_start_page);

	ax_local->irqlock = 0;	


	gAxRxTx_state = 2;
	CurrImr = ENISR_TX;
	writeb (ENISR_TX, ax_base + ADDR_SHIFT16(EN0_IMR));
	

	// dev_kfree_skb (skb);

	// ndev->trans_start = jiffies;
	// ax_local->stat.tx_bytes += send_length;
	return 0;
}

static int ax88796b_dma_send_data_process (struct sk_buff *skb, struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	int send_length;

	u8 ctepr=0, free_pages=0, need_pages;

	send_length = skb->len;

	writeb (E8390_PAGE0 | E8390_NODMA, ax_base + ADDR_SHIFT16(E8390_CMD));

	writeb (0x00, ax_base + ADDR_SHIFT16(EN0_IMR));
	ax_local->irqlock = 1;

	need_pages = ((send_length - 1) >> 8) + 1;

	ctepr = readb (ax_base + ADDR_SHIFT16(EN0_CTEPR));

	if (ctepr & ENCTEPR_TXCQF) {
		free_pages = 0;
	} else if (ctepr == 0) {

		/*
		 * If someone issues STOP bit of command register at run time,
		 * the TX machine will stop the current transmission and the
		 * CTEPR register will be reset to zero.
		 * This is just precautionary measures.
		 */
		if (ax_local->tx_prev_ctepr != 0) {
			ax_local->tx_curr_page = ax_local->tx_start_page;
			ax_local->tx_prev_ctepr = 0;
		}

		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page;
	}
	else if (ctepr < ax_local->tx_curr_page - 1) {
		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page + 
					ctepr - ax_local->tx_start_page + 1;
	}
	else if (ctepr > ax_local->tx_curr_page - 1) {
		free_pages = ctepr + 1 - ax_local->tx_curr_page;
	}
	else if (ctepr == ax_local->tx_curr_page - 1) {
		if (ax_local->tx_full)
			free_pages = 0;
		else
			free_pages = TX_PAGES;
	}

	if (free_pages < need_pages) {
		printk("hndz  dma process free_pages < need_pages\n");
		netif_stop_queue (ndev);
		ax_local->tx_full = 1;
		ax_local->irqlock = 0;	
		writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));
		return NETDEV_TX_BUSY;
	}

	eim_sdma_tx_start (ndev, send_length, skb->data, ax_local->tx_curr_page);
	gctepr= ctepr;
	gfree_pages = free_pages;
	gneed_pages = need_pages;
	return 0;
}

static int ax88796b_sdma_rx_poll(struct net_device *ndev, struct ax_device *ax_local);

static int gRxNumcount = 0;
//DMA回调函数
static void dma_m2m_rx_callback(void *data)
{
	int i = 0;
	unsigned long flags;
	unsigned char rxing_page, this_frame, next_frame;
	struct net_device *ndev = (struct net_device *)data;
	u8 cmd;
	struct ax_device *ax_local = ax_get_priv (ndev);

	int pkt_len = eim_rx_length;

	struct sk_buff *skb;
	struct sk_buff *txskb;
	int status;

	void *ax_base = ax_local->membase;

	if(ndev == NULL)
	{
		printk("zty callback data error!\n");
		return ;
	}

	if(gAxStop == 1)
	{
		gAxRxTx_state = 0;
		return ;
	}

	spin_lock_irqsave (&ax_local->page_lock, flags);

	skb = dev_alloc_skb (pkt_len + 2);
	if (skb == NULL) {
		printk(" Couldn't allocate a sk_buff"
			" of size %d.\n", pkt_len);
		ax_local->stat.rx_dropped++;

		spin_unlock_irqrestore (&ax_local->page_lock, flags);
		
		return;
	}

	/* IP headers on 16 byte boundaries */
	skb_reserve (skb, 2);
	skb->dev = ndev;
	skb_put (skb, pkt_len);
	memcpy(skb->data, gEIMrxbuf, eim_rx_length);	
	// dma_unmap_single(NULL, dma_dst, 2048, DMA_DEV_TO_MEM);
	skb->protocol = eth_type_trans (skb,ndev);

	skb_queue_tail(&ax_local->skb_queue, skb);
	// status = netif_rx_ni(skb);
	// if (status != NET_RX_SUCCESS)
	// {
	// 	printk("hndz ax recv error status 0x%x!\n", status);
	// }

	ndev->last_rx = jiffies;
	ax_local->stat.rx_packets++;
	ax_local->stat.rx_bytes += pkt_len;
	if (grx_frame.status & ENRSR_PHY)
	{
		ax_local->stat.multicast++;
		ax_local->boardcast_num++;
	}


	gRxNumcount++;
 	
   
	writeb (ENISR_RDC, ax_base + ADDR_SHIFT16(EN0_ISR));
	ax_local->dmaing = 0;

	next_frame = grx_frame.next;

	ax_local->current_page = next_frame;
	writeb (next_frame-1, ax_base + ADDR_SHIFT16(EN0_BOUNDARY));

	if(gRxNumcount < 6)
	{
		rxing_page = readb (ax_base + ADDR_SHIFT16(EN0_CURPAG));

		this_frame = readb (ax_base + ADDR_SHIFT16(EN0_BOUNDARY)) + 1;
		if (this_frame >= ax_local->stop_page)
			this_frame = ax_local->rx_start_page;
		
		if (this_frame == rxing_page) {	/* Read all the frames? */
			gAxRxTx_state = 0;
			ax_local->irqlock 	 = 0;
			gRxNumcount = 0;

			cmd = readb (ax_base + ADDR_SHIFT16(E8390_CMD));
			writeb ((cmd & E8390_PAGE_MASK) , ax_base + ADDR_SHIFT16(E8390_CMD));

			CurrImr = ENISR_ALL;
			writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));
			
		}
		else
		{
			 if(ax88796b_sdma_rx_poll(ndev, ax_local) != 1)
			 {

				gAxRxTx_state = 0;
				ax_local->irqlock 	 = 0;
				gRxNumcount = 0;

				cmd = readb (ax_base + ADDR_SHIFT16(E8390_CMD));
				writeb ((cmd & E8390_PAGE_MASK) , ax_base + ADDR_SHIFT16(E8390_CMD));

				writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));
				printk("hndz call back rx error!\n");
				gRxNumcount = 0;
			 }
			 else
			 {
				i = 1;
			 }
		}
	}

	if(i == 0)
	{
		napi_schedule(&ax_local->napi);
		if(ax_local->tx_skb != NULL)
		{
			if(skb->len > 64)
				status = ax88796b_dma_send_data_process(ax_local->tx_skb, ndev);
			else
			{
			 	status = ax88796b_send_data_process(ax_local->tx_skb, ndev);
				if(status != 0)
					printk("hndz call send ret is 0x%x!\n", status);
				txskb = ax_local->tx_skb;
				ax_local->tx_skb = NULL;
				spin_unlock_irqrestore (&ax_local->page_lock, flags);
				dev_kfree_skb (txskb);

				ndev->trans_start = jiffies;
				ax_local->stat.tx_bytes += skb->len ;
				return ;
			}
			// status = ax88796b_dma_send_data_process(ax_local->tx_skb, ndev);
			
			if(status != 0)
				printk("hndz call send ret is 0x%x!\n", status);
		}
	}

	spin_unlock_irqrestore (&ax_local->page_lock, flags);
	
	
	return ;
}

static void dma_m2m_tx_callback(void *data)
{
	// int i = 0;
	unsigned long flags;
	struct sk_buff  * skb;
	struct net_device *ndev = (struct net_device *)data;

	
	struct ax_device *ax_local = ax_get_priv (ndev);

	void *ax_base = ax_local->membase;

	// if(ndev == NULL)
	// {
	// 	printk("zty callback data error!\n");
	// 	return ;
	// }

	// dma_unmap_single(NULL, gdma_src, 2048, DMA_MEM_TO_DEV);
	if(gAxStop == 1)
	{
		gAxRxTx_state = 0;
		return ;
	}
	
	spin_lock_irqsave (&ax_local->page_lock, flags);
	skb = ax_local->tx_skb;
	writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));
	// cmd = readb (ax_base + ADDR_SHIFT16(E8390_CMD));
	// printk("hndz cmd is 0x%x!\n", cmd);

	// while ((readb(ax_base + ADDR_SHIFT16(EN0_ISR)) & 0x40) == 0)
	// {
	// 	i++;
	// 	if(i >= 0x00000f00)
	// 	{
	// 		printk("hndz tx dma time over!\n");
	// 		// ax_local->dmaing = 0;
	// 		// return -5;
	// 		break;
	// 	}
	// }

	// if ((readb(ax_base + ADDR_SHIFT16(EN0_ISR)) & 0x40) == 0) {

	// 		printk ("hndz timeout waiting for Tx RDC.\n");
	// }
		// else
		// 	printk("zty write ok!\n");
	
	// writeb (ENISR_RDC, ax_base + ADDR_SHIFT16(EN0_ISR));

	writeb (ENISR_RDC, ax_base + ADDR_SHIFT16(EN0_ISR));	/* Ack intr. */

	ax_local->dmaing = 0;

	ax_trigger_send (ndev, eim_tx_length, ax_local->tx_curr_page);

	if (gfree_pages == gneed_pages) {
		netif_stop_queue (ndev);
		ax_local->tx_full = 1;
	}
	ax_local->tx_prev_ctepr = gctepr;
	ax_local->tx_curr_page = ((ax_local->tx_curr_page + gneed_pages) <
		ax_local->tx_stop_page) ? 
		(ax_local->tx_curr_page + gneed_pages) : 
		(gneed_pages - (ax_local->tx_stop_page - ax_local->tx_curr_page)
			+ ax_local->tx_start_page);

	ax_local->irqlock = 0;	

	ax_local->tx_skb = NULL;

	gAxRxTx_state = 2;
	CurrImr = ENISR_TX;
	writeb (ENISR_TX, ax_base + ADDR_SHIFT16(EN0_IMR));
	
	

	spin_unlock_irqrestore (&ax_local->page_lock, flags);
	dev_kfree_skb (skb);


	

	ndev->trans_start = jiffies;
	ax_local->stat.tx_bytes += eim_tx_length;
	
	
	return ;
}



static  int eim_sdma_tx_start(struct net_device *ndev, int count, const unsigned char *buf, const int start_page)
{

	// int ret = 0;
	// int ring_offset;
	// uint32_t i = 0;
	struct dma_async_tx_descriptor *dma_m2m_desc;
	unsigned int size;

	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;

	// gtime1 = hndz_read_current_timer();
	memcpy(gEIMtxbuf, buf, count);

	eim_tx_length = count;
	
	 size = (eim_tx_length +4) & 0x7FE;
	// size = count;

	
	// if(size > 2048)
	// {
	// 	printk("hndz net rec %d data too long!\n", size);
	// 	ret = -1;
	// 	return ret;
	// }

	// eim_sdma_tx_init_config();

		/* This shouldn't happen. If it does, it's the last thing you'll see */
	if (ax_local->dmaing)
	{
		printk ("hndz DMAing conflict in ne_block_output."
			"[DMAstat:%d][irqlock:%d]\n",
			ax_local->dmaing, ax_local->irqlock);
		
		return -2;
	}

	ax_local->dmaing |= 0x01;

	// printk("hndz write %d!\n", size);



		/* Now the normal output. */
	writeb (size & 0xff, ax_base + ADDR_SHIFT16(EN0_RCNTLO));
	writeb (size >> 8,   ax_base + ADDR_SHIFT16(EN0_RCNTHI));
	writeb (0x00, ax_base + ADDR_SHIFT16(EN0_RSARLO));
	writeb (start_page, ax_base + ADDR_SHIFT16(EN0_RSARHI));

	writeb (E8390_RWRITE, ax_base + ADDR_SHIFT16(E8390_CMD));

	dma_m2m_desc = dma_m2m_txchan->device->device_prep_dma_memcpy(dma_m2m_txchan,
	(EIM_CS0_PHY_START_ADDR + ADDR_SHIFT16(EN0_DATAPORT)),  gdma_src, size, DMA_MEM_TO_DEV);													
	if (!dma_m2m_desc)
	{
		ax_local->dmaing = 0;
		printk("tx prep error!!\n");
		return -4;
	}


	
	//设置dma传输完成后的回调函数	
	dma_m2m_desc->callback = dma_m2m_tx_callback;
	dma_m2m_desc->callback_param = (void *)ndev;
	// printk("zty eim submit start 0x%lx!\n", jiffies);


	// printk("hndz eno sr 0x%x!\n", readb (ax_base + ADDR_SHIFT16(EN0_SR)));

	gAxRxTx_state = 2;
	ax_local->irqlock  = 1;

	dmaengine_submit(dma_m2m_desc);
	dma_async_issue_pending(dma_m2m_txchan);
	// gtime2 = hndz_read_current_timer();
	// printk("hndz tx 1 %lu!\n", gtime2 - gtime1);

		/* Now the normal output. */


	return 0;	
}

static  int eim_sdma_rx_start(unsigned int size, unsigned short current_offset, struct net_device * ndev)
{

	int ret = 0;
	int ring_offset;
	uint32_t i = 0;
	struct dma_async_tx_descriptor *dma_m2m_desc;
	unsigned int count;
	// struct net_device *ndev = (struct net_device *)data;

	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;

	eim_rx_length = size;
	ring_offset = current_offset;	
	count = (eim_rx_length + 4) & 0x7FE;

	
	if(size > 2048)
	{
		printk("hndz net rec %d data too long!\n", size);
		ret = -1;
		return ret;
	}


	if (ax_local->dmaing)
	{
		printk ("hndz DMAing conflict in ne_block_input "
			"[DMAstat:%d][irqlock:%d].\n",
			ax_local->dmaing, ax_local->irqlock);
		return -3;
	}

	ax_local->dmaing |= 0x01;

	writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));
	writeb (count & 0xff, ax_base + ADDR_SHIFT16(EN0_RCNTLO));
	writeb (count >> 8, ax_base + ADDR_SHIFT16(EN0_RCNTHI));
	writeb (ring_offset & 0xff, ax_base + ADDR_SHIFT16(EN0_RSARLO));
	writeb (ring_offset >> 8, ax_base + ADDR_SHIFT16(EN0_RSARHI));
	writeb (E8390_RREAD, ax_base + ADDR_SHIFT16(E8390_CMD));


	// eim_sdma_rx_init_config();

	dma_m2m_desc = dma_m2m_rxchan->device->device_prep_dma_memcpy(dma_m2m_rxchan, gdma_dst,
	(EIM_CS0_PHY_START_ADDR + ADDR_SHIFT16(EN0_DATAPORT)),  count, DMA_DEV_TO_MEM);													
	if (!dma_m2m_desc)
	{
		ax_local->dmaing = 0;
		printk("rx prep error!!\n");
		return -4;
	}
	
	//设置dma传输完成后的回调函数	
	dma_m2m_desc->callback = dma_m2m_rx_callback;
	dma_m2m_desc->callback_param = (void *)ndev;
	// printk("zty eim submit start 0x%lx!\n", jiffies);

	while ((readb (ax_base + ADDR_SHIFT16(EN0_SR)) & ENSR_DMA_READY) == 0)
	{
		i++;
		if(i >= 0x0F000000)
		{
			printk("hndz ax dma time over!\n");
			ax_local->dmaing = 0;
			return -5;
		}
	}

	gAxRxTx_state = 1;
	ax_local->irqlock  = 1;
	gRxNumcount = 0;

	dmaengine_submit(dma_m2m_desc);
	dma_async_issue_pending(dma_m2m_rxchan);
	return 0;	
}

static int ax88796b_sdma_rx_poll(struct net_device *ndev, struct ax_device *ax_local)
{
	void *ax_base = ax_local->membase;
	unsigned char rxing_page, this_frame, next_frame;
	unsigned short current_offset;
	// struct ax_pkt_hdr rx_frame;
	int ret = 0;

	if(netif_running(ndev)) 
	{

		int pkt_len, pkt_stat;

		/* Get the rx page (incoming packet pointer). */
		rxing_page = readb (ax_base + ADDR_SHIFT16(EN0_CURPAG));

		/* 
		 * Remove one frame from the ring.
		 * Boundary is always a page behind.
		 */
		this_frame = readb (ax_base + ADDR_SHIFT16(EN0_BOUNDARY)) + 1;

		if (this_frame >= ax_local->stop_page)
			this_frame = ax_local->rx_start_page;
		
		if (this_frame == rxing_page) {	/* Read all the frames? */
			return 0;			/* Done for now */
		}
		current_offset = this_frame << 8;
		ax88796b_get_hdr (ndev, &grx_frame, this_frame);


		pkt_len = grx_frame.count - sizeof (struct ax_pkt_hdr);
		pkt_stat = grx_frame.status;
		next_frame = this_frame + 1 + ((pkt_len + 4) >> 8);

		if (pkt_len < 60  ||  pkt_len > 1518) {
			PRINTK (ERROR_MSG, PFX
				" bogus pkt size: %d, status=%#2x nxpg=%#2x.\n",
				grx_frame.count, grx_frame.status, grx_frame.next);
			ax_local->stat.rx_errors++;
			ax_local->stat.rx_length_errors++;
		} else if ((pkt_stat & 0x0F) == ENRSR_RXOK) {
			if(eim_sdma_rx_start(pkt_len, current_offset + sizeof (grx_frame), ndev) == 0)
			{
				ret = 1;
				return 1;
			}
			else
			{
				printk("hndz ax rx error!\n");
			}
			
			
		} else {
			PRINTK (ERROR_MSG, PFX
				" bogus packet: status=%#2x"
				" nxpg=%#2x size=%d\n",
				grx_frame.status, grx_frame.next, grx_frame.count);
			ax_local->stat.rx_errors++;
			/* NB: The NIC counts CRC, frame and missed errors. */
			if (pkt_stat & ENRSR_FO)
				ax_local->stat.rx_fifo_errors++;
		}
		next_frame = grx_frame.next;

		ax_local->current_page = next_frame;
		writeb (next_frame-1, ax_base + ADDR_SHIFT16(EN0_BOUNDARY));

	}
	else
	{
		printk("hndz netif_running no runing!\n");
	}

	return ret;
}

// static int ax88796b_dma_start_xmit (struct sk_buff *skb, struct net_device *ndev)
// {
// 	struct ax_device *ax_local = ax_get_priv (ndev);
// 	void *ax_base = ax_local->membase;
// 	int send_length;
// 	unsigned long flags;
// 	u8 ctepr=0, free_pages=0, need_pages;

// 	PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);

// 	send_length = skb->len;

// 	spin_lock_irqsave (&ax_local->page_lock, flags);

// 	if (ax_local->tx_skb) {
// 		printk("hndz ax hard_xmit called while tx busy\n");
// 		spin_unlock_irqrestore(&ax_local->page_lock, flags);
// 		return NETDEV_TX_BUSY;
// 	}
// 	netif_stop_queue (ndev);
// 	ax_local->tx_skb = skb;
// 	if(gAxRxTx_state != 0)
// 	{
// 		spin_unlock_irqrestore(&ax_local->page_lock, flags);
// 		return NETDEV_TX_OK;
// 	}
	

// 	writeb (E8390_PAGE0 | E8390_NODMA, ax_base + ADDR_SHIFT16(E8390_CMD));

// 	writeb (0x00, ax_base + ADDR_SHIFT16(EN0_IMR));
// 	ax_local->irqlock = 1;

// 	need_pages = ((send_length - 1) >> 8) + 1;

// 	ctepr = readb (ax_base + ADDR_SHIFT16(EN0_CTEPR));

// 	if (ctepr & ENCTEPR_TXCQF) {
// 		free_pages = 0;
// 	} else if (ctepr == 0) {

// 		/*
// 		 * If someone issues STOP bit of command register at run time,
// 		 * the TX machine will stop the current transmission and the
// 		 * CTEPR register will be reset to zero.
// 		 * This is just precautionary measures.
// 		 */
// 		if (ax_local->tx_prev_ctepr != 0) {
// 			ax_local->tx_curr_page = ax_local->tx_start_page;
// 			ax_local->tx_prev_ctepr = 0;
// 		}

// 		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page;
// 	}
// 	else if (ctepr < ax_local->tx_curr_page - 1) {
// 		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page + 
// 					ctepr - ax_local->tx_start_page + 1;
// 	}
// 	else if (ctepr > ax_local->tx_curr_page - 1) {
// 		free_pages = ctepr + 1 - ax_local->tx_curr_page;
// 	}
// 	else if (ctepr == ax_local->tx_curr_page - 1) {
// 		if (ax_local->tx_full)
// 			free_pages = 0;
// 		else
// 			free_pages = TX_PAGES;
// 	}

// 	if (free_pages < need_pages) {
// 		printk("hndz free_pages < need_pages\n");
// 		netif_stop_queue (ndev);
// 		ax_local->tx_full = 1;
// 		ax_local->irqlock = 0;	
// 		writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));
// 		spin_unlock_irqrestore (&ax_local->page_lock, flags);
// 		return NETDEV_TX_BUSY;
// 	}

// 	ax_block_output (ndev, send_length, skb->data, ax_local->tx_curr_page);
// 	ax_trigger_send (ndev, send_length, ax_local->tx_curr_page);
// 	if (free_pages == need_pages) {
// 		netif_stop_queue (ndev);
// 		ax_local->tx_full = 1;
// 	}
// 	ax_local->tx_prev_ctepr = ctepr;
// 	ax_local->tx_curr_page = ((ax_local->tx_curr_page + need_pages) <
// 		ax_local->tx_stop_page) ? 
// 		(ax_local->tx_curr_page + need_pages) : 
// 		(need_pages - (ax_local->tx_stop_page - ax_local->tx_curr_page)
// 		 + ax_local->tx_start_page);

// 	ax_local->irqlock = 0;	
// 	ax_local->tx_skb = NULL;

// 	gAxRxTx_state = 2;
// 	CurrImr = ENISR_TX;
// 	writeb (ENISR_TX, ax_base + ADDR_SHIFT16(EN0_IMR));
	
// 	spin_unlock_irqrestore (&ax_local->page_lock, flags);


// 	dev_kfree_skb (skb);

// 	ndev->trans_start = jiffies;
// 	ax_local->stat.tx_bytes += send_length;

// 	return NETDEV_TX_OK;
// }


static int ax88796b_eim_dma_start_xmit (struct sk_buff *skb, struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	int send_length;
	unsigned long flags;
	u8 ctepr=0, free_pages=0, need_pages;


	if(gAxStop == 1)
		return NETDEV_TX_BUSY;
	send_length = skb->len;

	spin_lock_irqsave (&ax_local->page_lock, flags);

	if (ax_local->tx_skb) {
		printk("hndz ax hard_xmit called while tx busy\n");
		spin_unlock_irqrestore(&ax_local->page_lock, flags);
		return NETDEV_TX_BUSY;
	}
	netif_stop_queue (ndev);
	ax_local->tx_skb = skb;
	if(gAxRxTx_state != 0)
	{
		spin_unlock_irqrestore(&ax_local->page_lock, flags);
		return NETDEV_TX_OK;
	}
	

	writeb (E8390_PAGE0 | E8390_NODMA, ax_base + ADDR_SHIFT16(E8390_CMD));

	writeb (0x00, ax_base + ADDR_SHIFT16(EN0_IMR));
	ax_local->irqlock = 1;

	need_pages = ((send_length - 1) >> 8) + 1;

	ctepr = readb (ax_base + ADDR_SHIFT16(EN0_CTEPR));

	if (ctepr & ENCTEPR_TXCQF) {
		free_pages = 0;
	} else if (ctepr == 0) {

		/*
		 * If someone issues STOP bit of command register at run time,
		 * the TX machine will stop the current transmission and the
		 * CTEPR register will be reset to zero.
		 * This is just precautionary measures.
		 */
		if (ax_local->tx_prev_ctepr != 0) {
			ax_local->tx_curr_page = ax_local->tx_start_page;
			ax_local->tx_prev_ctepr = 0;
		}

		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page;
	}
	else if (ctepr < ax_local->tx_curr_page - 1) {
		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page + 
					ctepr - ax_local->tx_start_page + 1;
	}
	else if (ctepr > ax_local->tx_curr_page - 1) {
		free_pages = ctepr + 1 - ax_local->tx_curr_page;
	}
	else if (ctepr == ax_local->tx_curr_page - 1) {
		if (ax_local->tx_full)
			free_pages = 0;
		else
			free_pages = TX_PAGES;
	}

	if (free_pages < need_pages) {
		printk("hndz free_pages < need_pages\n");
		netif_stop_queue (ndev);
		ax_local->tx_full = 1;
		ax_local->irqlock = 0;	
		writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));
		spin_unlock_irqrestore (&ax_local->page_lock, flags);
		return NETDEV_TX_BUSY;
	}

	if(send_length > 64)
	{
		eim_sdma_tx_start (ndev, send_length, skb->data, ax_local->tx_curr_page);
		gctepr= ctepr;
		gfree_pages = free_pages;
		gneed_pages = need_pages;
		spin_unlock_irqrestore (&ax_local->page_lock, flags);
	}
	else
	{
		//  gtime1 = hndz_read_current_timer();
		ax_block_output (ndev, send_length, skb->data, ax_local->tx_curr_page);
		ax_trigger_send (ndev, send_length, ax_local->tx_curr_page);
		if (free_pages == need_pages) {
			netif_stop_queue (ndev);
			ax_local->tx_full = 1;
		}
		ax_local->tx_prev_ctepr = ctepr;
		ax_local->tx_curr_page = ((ax_local->tx_curr_page + need_pages) <
			ax_local->tx_stop_page) ? 
			(ax_local->tx_curr_page + need_pages) : 
			(need_pages - (ax_local->tx_stop_page - ax_local->tx_curr_page)
			+ ax_local->tx_start_page);

		ax_local->irqlock = 0;	
		ax_local->tx_skb = NULL;

		gAxRxTx_state = 2;
		CurrImr = ENISR_TX;
		writeb (ENISR_TX, ax_base + ADDR_SHIFT16(EN0_IMR));
		spin_unlock_irqrestore (&ax_local->page_lock, flags);
		//  gtime2 = hndz_read_current_timer();
		dev_kfree_skb (skb);

		ndev->trans_start = jiffies;
		ax_local->stat.tx_bytes += send_length;
		// printk("hndz tx %lu!\n", gtime2 - gtime1);
	}
	
	




	return NETDEV_TX_OK;
}

static irqreturn_t ax_dma_interrupt (int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	int interrupts;
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	u8 cmd;
	// u8 CurrImr;

	if (ndev == NULL) 
	{
		PRINTK (ERROR_MSG,
			"net_interrupt(): irq %d for unknown device.\n", irq);
		return IRQ_RETVAL (0);
	}
	if(gAxStop == 1)
		return IRQ_RETVAL (1);

	spin_lock (&ax_local->page_lock);
	if(gAxRxTx_state == 0)
	{
		
		writeb (E8390_NODMA | E8390_PAGE2, ax_base + ADDR_SHIFT16(E8390_CMD));

		CurrImr = readb (ax_base + ADDR_SHIFT16(EN0_IMR));

		writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));
		writeb (0x00, ax_base + ADDR_SHIFT16(EN0_IMR));


		if (ax_local->irqlock) {
			printk ("Interrupt occurred when irqlock locked\n");
			spin_unlock (&ax_local->page_lock);
			return IRQ_RETVAL (0);
		}
	
		if ((interrupts = readb (ax_base + ADDR_SHIFT16(EN0_ISR))) != 0)
		{
			writeb (interrupts, ax_base + ADDR_SHIFT16(EN0_ISR)); /* Ack the interrupts */

			if (interrupts & ENISR_TX) {

				ax_tx_intr (ndev);
				CurrImr = ENISR_ALL;			
			}

			if (interrupts & (ENISR_RX | ENISR_RX_ERR | ENISR_OVER)) {
				if(ax88796b_sdma_rx_poll(ndev, ax_local) == 1)
				{
					CurrImr = 0;
				}
				else
				{
					CurrImr = ENISR_ALL;
					cmd = readb (ax_base + ADDR_SHIFT16(E8390_CMD));
					writeb ((cmd & E8390_PAGE_MASK) , ax_base + ADDR_SHIFT16(E8390_CMD));
					
				}			
				
			}

			if (interrupts & ENISR_TX_ERR) {
				PRINTK (INT_MSG, PFX " TX err int\n");
				ax_tx_err (ndev);
			}

			if (interrupts & ENISR_COUNTERS) {   
				ax_local->stat.rx_frame_errors += 
						readb (ax_base + ADDR_SHIFT16(EN0_COUNTER0));
				ax_local->stat.rx_crc_errors += 
						readb (ax_base + ADDR_SHIFT16(EN0_COUNTER1));
				ax_local->stat.rx_missed_errors += 
						readb (ax_base + ADDR_SHIFT16(EN0_COUNTER2));
				writeb (ENISR_COUNTERS, ax_base + ADDR_SHIFT16(EN0_ISR));
			}

			if (interrupts & ENISR_RDC)
				writeb (ENISR_RDC, ax_base + ADDR_SHIFT16(EN0_ISR));
		}
		writeb (CurrImr, ax_base + ADDR_SHIFT16(EN0_IMR));
	}
	else if (gAxRxTx_state == 2)
	{		
		writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));

		if ((interrupts = readb (ax_base + ADDR_SHIFT16(EN0_ISR))) != 0)
		{
			writeb (interrupts, ax_base + ADDR_SHIFT16(EN0_ISR));
			if (interrupts & ENISR_TX) 
			{
				ax_tx_intr (ndev);

				gAxRxTx_state = 0;
				
				CurrImr = 0;
				writeb (CurrImr, ax_base + ADDR_SHIFT16(EN0_IMR));
				if(ax88796b_sdma_rx_poll(ndev, ax_local) != 1)
				{
					CurrImr = ENISR_ALL;
					writeb (CurrImr, ax_base + ADDR_SHIFT16(EN0_IMR));
				}
				
			}
			else
			{
				if (!(interrupts & (ENISR_RX))){
					printk("hndz tx rx inter error 0x%x!\n", interrupts);
				}
				
			}

		}
		else
		{
			printk("hndz tx error!\n");
		}

	}
	else
	{
		;
	}

	spin_unlock (&ax_local->page_lock);

	return IRQ_RETVAL (1);
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: mdio_sync
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static void mdio_sync (struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
    int bits;
    for (bits = 0; bits < 32; bits++) {
		writeb (MDIO_DATA_WRITE1, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (MDIO_DATA_WRITE1 | MDIO_SHIFT_CLK,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
    }
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: mdio_sync
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static int mdio_read (struct net_device *ndev, int phy_id, int loc)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	u_int cmd = (0xf6<<10)|(phy_id<<5)|loc;
	int i, retval = 0;

	mdio_sync (ndev);
	for (i = 13; i >= 0; i--) {
		int dat = (cmd&(1<<i)) ? MDIO_DATA_WRITE1 : MDIO_DATA_WRITE0;
		writeb (dat, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (dat | MDIO_SHIFT_CLK, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}
	for (i = 19; i > 0; i--) {
		writeb (MDIO_ENB_IN, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		retval = (retval << 1) | ((readb (ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM))
				& MDIO_DATA_READ) != 0);
		writeb (MDIO_ENB_IN | MDIO_SHIFT_CLK,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}

	return (retval>>1) & 0xffff;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: mdio_write
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static void mdio_write (struct net_device *ndev, int phy_id, int loc, int value)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	u_int cmd = (0x05<<28)|(phy_id<<23)|(loc<<18)|(1<<17)|value;
	int i;

	mdio_sync (ndev);
	for (i = 31; i >= 0; i--) {
		int dat = (cmd&(1<<i)) ? MDIO_DATA_WRITE1 : MDIO_DATA_WRITE0;
		writeb (dat, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (dat | MDIO_SHIFT_CLK, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}
	for (i = 1; i >= 0; i--) {
		writeb (MDIO_ENB_IN, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (MDIO_ENB_IN | MDIO_SHIFT_CLK,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}

}


#if (CONFIG_AX88796B_EEPROM_READ_WRITE == 1)
/*  
 *  ======================================================================
 *   EEPROM interface support
 *  ======================================================================
 */
#define EEPROM_SHIFT_CLK			(0x80)
#define EEPROM_DATA_READ1			(0x40)
#define EEPROM_DATA_WRITE0			(0x00)
#define EEPROM_DATA_WRITE1			(0x20)
#define EEPROM_SELECT				(0x10)
#define EEPROM_DIR_IN				(0x02)

#define EEPROM_READ				(0x02)
#define EEPROM_EWEN				(0x00)
#define EEPROM_ERASE				(0x03)
#define EEPROM_WRITE				(0x01)
#define EEPROM_ERALL				(0x00)
#define EEPROM_WRAL				(0x00)
#define EEPROM_EWDS				(0x00)
#define EEPROM_93C46_OPCODE(x)			((x) << 6)
#define EEPROM_93C46_STARTBIT			(1 << 8)

/*
 * ----------------------------------------------------------------------------
 * Function Name: eeprom_write_en
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static void
eeprom_write_en (struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;

	unsigned short cmd = EEPROM_93C46_STARTBIT | 
			     EEPROM_93C46_OPCODE(EEPROM_EWEN) | 0x30;
	unsigned char tmp;
	int i;

	// issue a "SB OP Addr" command
	for (i = 8; i >= 0 ; i--) {
		tmp = (cmd & (1 << i)) == 0 ? 
				EEPROM_DATA_WRITE0 : EEPROM_DATA_WRITE1;
		writeb (tmp | EEPROM_SELECT, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (tmp | EEPROM_SELECT | EEPROM_SHIFT_CLK,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}

	for (i = 15; i >= 0; i--) {
		writeb (EEPROM_DATA_WRITE0 | EEPROM_SELECT,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (EEPROM_DATA_WRITE0 | EEPROM_SELECT | EEPROM_SHIFT_CLK,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}

	writeb (0, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: eeprom_write_dis
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static void
eeprom_write_dis (struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;

	unsigned short cmd = EEPROM_93C46_STARTBIT |
			     EEPROM_93C46_OPCODE (EEPROM_EWDS);
	unsigned char tmp;
	int i;

	// issue a "SB OP Addr" command
	for (i = 8; i >= 0 ; i--) {
		tmp = (cmd & (1 << i)) == 0 ? 
				EEPROM_DATA_WRITE0 : EEPROM_DATA_WRITE1;
		writeb (tmp | EEPROM_SELECT, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (tmp | EEPROM_SELECT | EEPROM_SHIFT_CLK,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}

	for (i = 15; i >= 0; i--) {
		writeb (EEPROM_DATA_WRITE0 | EEPROM_SELECT,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (EEPROM_DATA_WRITE0 | EEPROM_SELECT | EEPROM_SHIFT_CLK,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}

	writeb (0, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: eeprom_write
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static int
eeprom_write (struct net_device *ndev, unsigned char loc, unsigned short nValue)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;

	unsigned short cmd = EEPROM_93C46_STARTBIT |
			     EEPROM_93C46_OPCODE (EEPROM_WRITE) | loc;
	unsigned char tmp;
	int i;
	int ret = 0;

	// issue a "SB OP Addr" command
	for(i = 8; i >= 0 ; i--) {
		tmp = (cmd & (1 << i)) == 0 ? 
				EEPROM_DATA_WRITE0 : EEPROM_DATA_WRITE1;
		writeb (tmp | EEPROM_SELECT, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (tmp | EEPROM_SELECT | EEPROM_SHIFT_CLK,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}

	// writing the data
	for (i = 15; i >= 0; i--) {
		tmp = (nValue & (1 << i)) == 0 ?
				EEPROM_DATA_WRITE0 : EEPROM_DATA_WRITE1;
		writeb (tmp | EEPROM_SELECT, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (tmp | EEPROM_SELECT | EEPROM_SHIFT_CLK,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}

	//
	// check busy
	//

	// Turn, wait two clocks
	writeb (EEPROM_DIR_IN, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	writeb (EEPROM_SHIFT_CLK | EEPROM_DIR_IN, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	writeb (EEPROM_DIR_IN, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	writeb (EEPROM_SHIFT_CLK | EEPROM_DIR_IN, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));

	// waiting for busy signal
	i = 0xFFFF;
	while (--i) {
		writeb (EEPROM_SELECT | EEPROM_DIR_IN,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (EEPROM_SELECT | EEPROM_DIR_IN | EEPROM_SHIFT_CLK,
				ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		tmp = readb (ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		if ((tmp & EEPROM_DATA_READ1) == 0)
			break;
	}
	if (i <= 0) {
		PRINTK (ERROR_MSG, PFX
			"Failed on waiting for EEPROM bus busy signal\n");
		ret = -1;
	} else {
		i = 0xFFFF;
		while (--i) {
			writeb (EEPROM_SELECT | EEPROM_DIR_IN,
					ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
			writeb ((EEPROM_SELECT | EEPROM_DIR_IN | 
				 EEPROM_SHIFT_CLK),
				 ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
			tmp = readb (ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
			if (tmp & EEPROM_DATA_READ1)
				break;
		}

		if (i <= 0) {
			PRINTK (ERROR_MSG, PFX
				"Failed on waiting for EEPROM completion\n");
			ret = -1;
		}
	}

	writeb (0, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));

	return ret;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: eeprom_read
 * Purpose: 
 * ----------------------------------------------------------------------------
 */
static unsigned short
eeprom_read (struct net_device *ndev, unsigned char loc)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;

	unsigned short cmd = EEPROM_93C46_STARTBIT |
			     EEPROM_93C46_OPCODE (EEPROM_READ) | loc;
	unsigned char tmp;
	unsigned short retValue = 0;
	int i;

	// issue a "SB OP Addr" command
	for (i = 8; i >= 0 ; i--) {
		tmp = (cmd & (1 << i)) == 0 ? 
		EEPROM_DATA_WRITE0 : EEPROM_DATA_WRITE1;
		writeb (tmp | EEPROM_SELECT, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		writeb (tmp | EEPROM_SELECT | EEPROM_SHIFT_CLK,
			ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}

	// Turn
	writeb (EEPROM_DIR_IN | EEPROM_SELECT, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	writeb (EEPROM_DIR_IN | EEPROM_SELECT | 
		EEPROM_SHIFT_CLK | EEPROM_DATA_WRITE1,
		ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));

	// retriving the data
	for (i = 15; i >= 0; i--) {
		writeb (EEPROM_SELECT | EEPROM_DIR_IN,
			ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		tmp = readb (ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
		if (tmp & EEPROM_DATA_READ1)
			retValue |= (1 << i);
		writeb (EEPROM_SELECT | EEPROM_DIR_IN | EEPROM_SHIFT_CLK,
			ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));
	}

	writeb (0, ax_base + ADDR_SHIFT16(AX88796_MII_EEPROM));

	return retValue;
}
#endif /* #if (CONFIG_AX88796B_EEPROM_READ_WRITE == 1) */

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_dump_regs
 * Purpose: Dump all MAC registers
 * ----------------------------------------------------------------------------
 */
static void ax88796b_dump_regs (struct ax_device *ax_local)
{
	void __iomem *ax_base = ax_local->membase;
	u8 cr, i, j;

	cr = readb (ax_base + ADDR_SHIFT16(E8390_CMD)) & E8390_PAGE_MASK;
	PRINTK (DEBUG_MSG, "       Page 0   Page 1   Page 2   Page 3\n");
	for (i = 0; i < 0x1F; i++) {

		PRINTK (DEBUG_MSG, " 0x%02x   ", i);
		for (j = 0; j < 4; j++) {
			writeb (cr | (j << 6), ax_base + ADDR_SHIFT16(E8390_CMD));

			PRINTK (DEBUG_MSG, "0x%02x     ",
				readb (ax_base + ADDR_SHIFT16(EI_SHIFT (i))));
		}
		PRINTK (DEBUG_MSG, "\n");
	}
	PRINTK (DEBUG_MSG, "\n");

	writeb (cr | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_dump_phy_regs
 * Purpose: Dump PHY register from MR0 to MR5
 * ----------------------------------------------------------------------------
 */
static void ax88796b_dump_phy_regs (struct ax_device *ax_local)
{
	int i;

	PRINTK (DEBUG_MSG, "Dump PHY registers:\n");
	for (i = 0; i < 6; i++) {
		PRINTK (DEBUG_MSG, "  MR%d = 0x%04x\n", i,
			mdio_read (ax_local->ndev, 0x10, i));
	}
}






void get_ip_addr(struct net_device *ndev, char * address)
{
	struct in_device * ip = ndev->ip_ptr;
	struct in_ifaddr *in;
//	unsigned char address[4];
	if (ip == NULL) return;
	in = ip->ifa_list;
	if(in != NULL)
	{
		memcpy(address, &in->ifa_address, 4);
	}
	// printk("ip is %d.%d.%d.%d !\n", address[0], address[1],address[2],address[3]);

	// printk("mac is %x:%x:%x:%x:%x:%x !\n", ndev->dev_addr[0], ndev->dev_addr[1],ndev->dev_addr[2],
	//                           ndev->dev_addr[3], ndev->dev_addr[4],ndev->dev_addr[5]);

	// 	struct net_device *dev = ndev;
	// struct in_device *ip = dev->ip_ptr;
	// struct in_ifaddr *in;
	// unsigned char address[4], netmask[4];

	// if (ip == NULL) return;
	// in = ip->ifa_list;
	// while (in != NULL) {
	// 	memcpy(address, &in->ifa_address, sizeof(address));
	// 	memcpy(netmask, &in->ifa_mask, sizeof(netmask));
	// 	in = in->ifa_next;
	// }
}
static int ax88796b_send_data (struct net_device *ndev)
{
	char ginitbuf[48];
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	int send_length;
	unsigned long flags;
	int i;
	u8 ctepr=0, free_pages=0, need_pages;

	char initdata[10] = {0x08, 0x06, 0x00, 0x01, 0x08, 0x00, 0x06, 0x04,0x00, 0x02
	                    };

//	printk("zty send !\n");
	
	for(i = 0; i < 6; i++)
	{
		ginitbuf[i] = 0xff;
	}

	memcpy(ginitbuf+i, ndev->dev_addr, 6);
	memcpy(ginitbuf+12, initdata, 10);
	memcpy(ginitbuf+22, ndev->dev_addr, 6);
	get_ip_addr(ndev, ginitbuf+28);
	memcpy(ginitbuf+32, ndev->dev_addr, 6);
	get_ip_addr(ndev, ginitbuf+38);

	send_length = 42;

	spin_lock_irqsave (&ax_local->page_lock, flags);

	writeb (E8390_PAGE0 | E8390_NODMA, ax_base + ADDR_SHIFT16(E8390_CMD));

	writeb (0x00, ax_base + ADDR_SHIFT16(EN0_IMR));
	ax_local->irqlock = 1;

	need_pages = ((send_length - 1) >> 8) + 1;

	ctepr = readb (ax_base + ADDR_SHIFT16(EN0_CTEPR));

	if (ctepr & ENCTEPR_TXCQF) {
		free_pages = 0;
	} else if (ctepr == 0) {

		/*
		 * If someone issues STOP bit of command register at run time,
		 * the TX machine will stop the current transmission and the
		 * CTEPR register will be reset to zero.
		 * This is just precautionary measures.
		 */
		if (ax_local->tx_prev_ctepr != 0) {
			ax_local->tx_curr_page = ax_local->tx_start_page;
			ax_local->tx_prev_ctepr = 0;
		}

		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page;
	}
	else if (ctepr < ax_local->tx_curr_page - 1) {
		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page + 
					ctepr - ax_local->tx_start_page + 1;
	}
	else if (ctepr > ax_local->tx_curr_page - 1) {
		free_pages = ctepr + 1 - ax_local->tx_curr_page;
	}
	else if (ctepr == ax_local->tx_curr_page - 1) {
		if (ax_local->tx_full)
			free_pages = 0;
		else
			free_pages = TX_PAGES;
	}


	if (free_pages < need_pages) {
		PRINTK (DEBUG_MSG, "free_pages < need_pages\n");
		netif_stop_queue (ndev);
		ax_local->tx_full = 1;
		ax_local->irqlock = 0;	
		writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));
		spin_unlock_irqrestore (&ax_local->page_lock, flags);
		return NETDEV_TX_BUSY;
	}

	// ax_block_dma_output(ndev, send_length, ginitbuf, ax_local->tx_curr_page);
	ax_block_output (ndev, send_length, ginitbuf, ax_local->tx_curr_page);
	// printk("hndz ax_local->tx_curr_page %d!\n", ax_local->tx_curr_page);
	// eim_sdma_tx_start (ndev, send_length, ginitbuf, ax_local->tx_curr_page);
	ax_trigger_send (ndev, send_length, ax_local->tx_curr_page);
	if (free_pages == need_pages) {
		netif_stop_queue (ndev);
		ax_local->tx_full = 1;
	}
	ax_local->tx_prev_ctepr = ctepr;
	ax_local->tx_curr_page = ((ax_local->tx_curr_page + need_pages) <
		ax_local->tx_stop_page) ? 
		(ax_local->tx_curr_page + need_pages) : 
		(need_pages - (ax_local->tx_stop_page - ax_local->tx_curr_page)
		 + ax_local->tx_start_page);

	ax_local->irqlock = 0;	
	writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));

	 spin_unlock_irqrestore (&ax_local->page_lock, flags);


	ndev->trans_start = jiffies;
	//ax_local->stat.tx_bytes += send_length;

	return NETDEV_TX_OK;
}
/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_get_drvinfo
 * Purpose: Exported for Ethtool to query the driver version
 * ----------------------------------------------------------------------------
 */
static void ax88796b_get_drvinfo (struct net_device *ndev,
				 struct ethtool_drvinfo *info)
{
	/* Inherit standard device info */
	strncpy (info->driver, DRV_NAME, sizeof info->driver);
	strncpy (info->version, DRV_VERSION, sizeof info->version);
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_get_link
 * Purpose: Exported for Ethtool to query the media link status
 * ----------------------------------------------------------------------------
 */
static u32 ax88796b_get_link (struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);

	return ax_get_link (ax_local);
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_get_wol
 * Purpose: Exported for Ethtool to query wake on lan setting
 * ----------------------------------------------------------------------------
 */
static void
ax88796b_get_wol (struct net_device *ndev, struct ethtool_wolinfo *wolinfo)
{
	struct ax_device *ax_local = ax_get_priv (ndev);

	wolinfo->supported = WAKE_PHY | WAKE_MAGIC;
	wolinfo->wolopts = 0;
	
	if (ax_local->wol & WAKEUP_LSCWE)
		wolinfo->wolopts |= WAKE_PHY;
	if (ax_local->wol & WAKE_MAGIC)
		wolinfo->wolopts |= WAKE_MAGIC;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_set_wol
 * Purpose: Exported for Ethtool to set the wake on lan setting
 * ----------------------------------------------------------------------------
 */
static int
ax88796b_set_wol (struct net_device *ndev, struct ethtool_wolinfo *wolinfo)
{
	struct ax_device *ax_local = ax_get_priv (ndev);

	ax_local->wol = 0;

	if (wolinfo->wolopts & WAKE_PHY)
		ax_local->wol |= WAKEUP_LSCWE;
	if (wolinfo->wolopts & WAKE_MAGIC)
		ax_local->wol |= WAKEUP_MP;

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_get_settings
 * Purpose: Exported for Ethtool to query PHY setting
 * ----------------------------------------------------------------------------
 */
static int
ax88796b_get_settings (struct net_device *ndev, struct ethtool_cmd *cmd)
{
	struct ax_device *ax_local = ax_get_priv (ndev);

	return mii_ethtool_gset(&ax_local->mii, cmd);
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_set_settings
 * Purpose: Exported for Ethtool to set PHY setting
 * ----------------------------------------------------------------------------
 */
static int
ax88796b_set_settings (struct net_device *ndev, struct ethtool_cmd *cmd)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	int retval;

	retval = mii_ethtool_sset (&ax_local->mii, cmd);

	return retval;

}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_nway_reset
 * Purpose: Exported for Ethtool to restart PHY autonegotiation
 * ----------------------------------------------------------------------------
 */
static int ax88796b_nway_reset (struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	return mii_nway_restart(&ax_local->mii);
}

struct ethtool_ops ax88796b_ethtool_ops = {
	.get_drvinfo		= ax88796b_get_drvinfo,
	.get_link		= ax88796b_get_link,
	.get_wol		= ax88796b_get_wol,
	.set_wol		= ax88796b_set_wol,
	.get_settings		= ax88796b_get_settings,
	.set_settings		= ax88796b_set_settings,
	.nway_reset		= ax88796b_nway_reset,
};


//ssize_t ds2460_read_generic(u8 *buf, loff_t addr, unsigned len);

#define HW_OCOTP_CUST_N(n)	(0x00000400 + (n) * 0x10)
#define BF(value, field)	(((value) << BP_##field) & BM_##field)
#define HW_OCOTP_TIMING			0x00000010
#define BP_OCOTP_TIMING_STROBE_READ	16
#define BM_OCOTP_TIMING_STROBE_READ	0x003F0000
#define BP_OCOTP_TIMING_RELAX		12
#define BM_OCOTP_TIMING_RELAX		0x0000F000
#define BP_OCOTP_TIMING_STROBE_PROG	0
#define BM_OCOTP_TIMING_STROBE_PROG	0x00000FFF
#define HW_OCOTP_CTRL			0x00000000
#define BM_OCOTP_CTRL_ERROR		0x00000200
#define BM_OCOTP_CTRL_BUSY		0x00000100


// static void set_otp_timing(void* otp_base, struct clk *otp_clk)
// {
// 	unsigned long clk_rate = 0;
// 	unsigned long strobe_read, relex, strobe_prog;
// 	u32 timing = 0;

// 	clk_rate = clk_get_rate(otp_clk);

// 	/* do optimization for too many zeros */
// 	relex = clk_rate / (1000000000 / 20) - 1;
// 	strobe_prog = clk_rate / (1000000000 / 10000) + 2 * (20 + 1) - 1;
// 	strobe_read = clk_rate / (1000000000 / 40) + 2 * (20 + 1) - 1;

// 	timing = BF(relex, OCOTP_TIMING_RELAX);
// 	timing |= BF(strobe_read, OCOTP_TIMING_STROBE_READ);
// 	timing |= BF(strobe_prog, OCOTP_TIMING_STROBE_PROG);

// 	__raw_writel(timing, otp_base + HW_OCOTP_TIMING);
// }

// static int otp_wait_busy(u32 flags, void* otp_base, struct clk *otp_clk)
// {
// 	int count;
// 	u32 c;

// 	for (count = 10000; count >= 0; count--) {
// 		c = __raw_readl(otp_base + HW_OCOTP_CTRL);
// 		if (!(c & (BM_OCOTP_CTRL_BUSY | BM_OCOTP_CTRL_ERROR | flags)))
// 			break;
// 		cpu_relax();
// 	}

// 	if (count < 0)
// 		return -ETIMEDOUT;

// 	return 0;
// }

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_load_macaddr
 * Purpose: Load MAC address from EEPROM
 * ----------------------------------------------------------------------------
 */
 
ssize_t ds2460_read_generic(u8 *buf, loff_t addr, unsigned len);
static void ax88796b_load_macaddr (struct net_device *ndev, unsigned char *pMac)
{
	// struct device_node *np = NULL;
	// struct clk *otp_clk;
	// void __iomem *otp_base;
	// struct resource res;
	// int ret;
	// u32 value = 0;

	// np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	// ret=of_address_to_resource(np, 0, &res);
	// otp_base=ioremap(res.start,0x4000);
	// otp_clk = of_clk_get(np, 0);
	// if (IS_ERR(otp_clk)) {
	// 	printk( "wwwwwww of_clk_get_by_name xxxxxxx\n");
	// }

	// ret = clk_prepare_enable(otp_clk);
	// if (ret)
	// 	printk("clk_prepare_enable xxxxxxxx\n");

	// set_otp_timing(otp_base, otp_clk);
	// ret = otp_wait_busy(0, otp_base, otp_clk);
	// if (ret)
	// 	printk("otp_wait_busy xxxxxxxx\n");

	// value = __raw_readl(otp_base + HW_OCOTP_CUST_N(34));
	// pMac[2]=value>>24;
	// pMac[3]=value>>16;
	// pMac[4]=value>>8;
	// pMac[5]=value;
	// value = __raw_readl(otp_base + HW_OCOTP_CUST_N(35));
	// pMac[0]=value>>8;
	// pMac[1]=value;
	// clk_disable_unprepare(otp_clk);

#if 0
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	int i;
	struct {unsigned char value, offset; } program_seq[] =
	{
		{E8390_NODMA | E8390_PAGE0 | E8390_STOP, E8390_CMD},
		{ax_local->bus_width, EN0_DCFG},
		{0x00, EN0_RCNTLO},
		{0x00, EN0_RCNTHI},
		{0x00, EN0_IMR},
		{0xFF, EN0_ISR},
		{E8390_RXOFF, EN0_RXCR},
		{E8390_TXOFF, EN0_TXCR},
		{0x0C, EN0_RCNTLO},
		{0x00, EN0_RCNTHI},
		{0x00, EN0_RSARLO},
		{0x00, EN0_RSARHI},
		{E8390_RREAD | E8390_START, E8390_CMD},
	};

	/* Read the 12 bytes of station address PROM. */
	for (i = 0; i < sizeof (program_seq)/sizeof (program_seq[0]); i++)
		writeb (program_seq[i].value, ax_base + program_seq[i].offset);

	while (( readb (ax_base + EN0_SR) & 0x20) == 0);

	for (i = 0; i < 6; i++)
		pMac[i] = (unsigned char) READ_FIFO (ax_base + EN0_DATAPORT);

	writeb (ENISR_RDC, ax_base + EN0_ISR);	/* Ack intr. */

	/* Support for No EEPROM */
	if (!is_valid_ether_addr (pMac)) {
		PRINTK (DRIVER_MSG, "Use random MAC address\n");
		random_ether_addr(pMac);
	}
#else
	//  ds2460_read_generic(pMac, 0x90, 6);
	etherm_addr(pMac);	
	if (!is_valid_ether_addr (pMac)) {
		PRINTK (DRIVER_MSG, "MAC read from ds2460 is invalid, so use random MAC address\n");
		random_ether_addr(pMac);
	}
#endif
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_set_macaddr
 * Purpose: Set MAC addres into hardware
 * ----------------------------------------------------------------------------
 */
static void ax88796b_set_macaddr (struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	u8 i;

	/* Copy the station address into the DS8390 registers. */
	writeb (E8390_NODMA | E8390_PAGE1, ax_base + ADDR_SHIFT16(E8390_CMD));
	writeb (NESM_START_PG + TX_PAGES + 1, ax_base + ADDR_SHIFT16(EN1_CURPAG));
	for (i = 0; i < 6; i++) 
	{
		writeb (ndev->dev_addr[i], ax_base + ADDR_SHIFT16(EN1_PHYS_SHIFT (i)));
		if (readb (ax_base + ADDR_SHIFT16(EN1_PHYS_SHIFT (i)))!=ndev->dev_addr[i])
			PRINTK (ERROR_MSG, PFX
				"Hw. address read/write mismap %d\n", i);
	}
	writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_set_mac_address
 * Purpose: Exported for upper layer
 * ----------------------------------------------------------------------------
 */
static int ax88796b_set_mac_address (struct net_device *ndev, void *p)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	struct sockaddr *addr = p;
	unsigned long flags;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(ndev->dev_addr, addr->sa_data, ndev->addr_len);

	spin_lock_irqsave (&ax_local->page_lock, flags);
	ax88796b_set_macaddr (ndev);
	spin_unlock_irqrestore (&ax_local->page_lock, flags);

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_reset
 * Purpose: Reset AX88796B MAC
 * ----------------------------------------------------------------------------
 */
static void ax88796b_reset (struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	unsigned long reset_start_time = jiffies;

	readb (ax_base + ADDR_SHIFT16(EN0_RESET));

	ax_local->dmaing = 0;

	while ((readb (ax_base + ADDR_SHIFT16(EN0_SR)) & ENSR_DEV_READY) == 0) {
		if (jiffies - reset_start_time > 2*HZ/100) {
			PRINTK (ERROR_MSG, PFX
				" reset did not complete.\n");
			break;
		}
	}
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_get_stats
 * Purpose: Collect the stats.
 * ----------------------------------------------------------------------------
 */
static struct net_device_stats *ax88796b_get_stats (struct net_device *ndev)
{
 	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase; 
	unsigned long flags;

	/* If the card is stopped, just return the present stats. */
	if (!netif_running (ndev))
		return &ax_local->stat;

	spin_lock_irqsave (&ax_local->page_lock,flags);
	/* Read the counter registers, assuming we are in page 0. */
	ax_local->stat.rx_frame_errors += readb (ax_base + ADDR_SHIFT16(EN0_COUNTER0));
	ax_local->stat.rx_crc_errors   += readb (ax_base + ADDR_SHIFT16(EN0_COUNTER1));
	ax_local->stat.rx_missed_errors+= readb (ax_base + ADDR_SHIFT16(EN0_COUNTER2));
	spin_unlock_irqrestore (&ax_local->page_lock, flags);

	return &ax_local->stat;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: make_mc_bits
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static inline void make_mc_bits (u8 *bits, struct net_device *ndev)
{
	u32 crc;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	struct dev_mc_list *dmi;
	for (dmi=ndev->mc_list; dmi; dmi=dmi->next) {
		if (dmi->dmi_addrlen != ETH_ALEN) {
			PRINTK (INIT_MSG, PFX
				" invalid multicast address length given.\n");
			continue;
		}
		crc = ether_crc (ETH_ALEN, dmi->dmi_addr);
		/* 
		 * The 8390 uses the 6 most significant bits of the
		 * CRC to index the multicast table.
		 */
		bits[crc >> 29] |= (1 << ((crc >> 26) & 7));
	}
#else
	struct netdev_hw_addr *ha;
	netdev_for_each_mc_addr (ha, ndev) {
		crc = ether_crc (ETH_ALEN, ha->addr);
		bits[crc >> 29] |= (1 << ((crc >> 26) & 7));
	}
#endif
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: do_set_multicast_list
 * Purpose: Set RX mode and multicast filter
 * ----------------------------------------------------------------------------
 */
static void do_set_multicast_list (struct net_device *ndev)
{
 	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase; 
	int i;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	int mc_count = ndev->mc_count;
#else
	int mc_count = netdev_mc_count (ndev);
#endif

	if (!(ndev->flags & (IFF_PROMISC | IFF_ALLMULTI))) {
		memset (ax_local->mcfilter, 0, 8);
		if (mc_count)
			make_mc_bits (ax_local->mcfilter, ndev);
	} else {
		/* mcast set to accept-all */
		memset (ax_local->mcfilter, 0xFF, 8);
	}

	if (netif_running (ndev))
		writeb (E8390_RXCONFIG, ax_base + ADDR_SHIFT16(EN0_RXCR));

	writeb (E8390_NODMA | E8390_PAGE1, ax_base + ADDR_SHIFT16(E8390_CMD));
	for (i = 0; i < 8; i++) {
		writeb (ax_local->mcfilter[i], ax_base + ADDR_SHIFT16(EN1_MULT_SHIFT (i)));
	}
	writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));

  	if (ndev->flags&IFF_PROMISC)
	{
		writeb (E8390_RXCONFIG | 0x18, ax_base + ADDR_SHIFT16(EN0_RXCR));
		// writeb (E8390_RXCONFIG , ax_base + ADDR_SHIFT16(EN0_RXCR));
		// printk("zty promisc mode!\n");
	}
	else if (ndev->flags & IFF_ALLMULTI || mc_count)
	{
		// dump_stack();
		// printk("zty multi mode!\n");
		writeb (E8390_RXCONFIG | 0x08, ax_base + ADDR_SHIFT16(EN0_RXCR));
		// writeb (0x04, ax_base + ADDR_SHIFT16(EN0_RXCR));
	}
	else 
	{
		// printk("zty broadcast mode!\n");
		writeb (E8390_RXCONFIG, ax_base + ADDR_SHIFT16(EN0_RXCR));
	}
	//  writeb (0x00, ax_base + ADDR_SHIFT16(EN0_RXCR));
 }


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_set_multicast_list
 * Purpose: Exported for upper layer
 * ----------------------------------------------------------------------------
 */
static void ax88796b_set_multicast_list (struct net_device *ndev)
{
	unsigned long flags;
	struct ax_device *ax_local = ax_get_priv (ndev);

	PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);

	spin_lock_irqsave (&ax_local->page_lock, flags);
	do_set_multicast_list (ndev);
	spin_unlock_irqrestore (&ax_local->page_lock, flags);

	PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);
}	


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796_PHY_init
 * Purpose: Initialize PHY media mode
 * ----------------------------------------------------------------------------
 */
static int ax88796_PHY_init (struct net_device *ndev, u8 echo)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	u16 advertise;
	u16 phy14h, phy15h, phy16h;
	u8 rty_cnt;

	/* Enable AX88796B FOLW CONTROL */
	writeb (ENFLOW_ENABLE, ax_base + ADDR_SHIFT16(EN0_FLOW));

	rty_cnt = 0;
restart:
	mdio_write (ndev, 0x10, 0x14, 0x742C);
	phy14h = mdio_read (ndev, 0x10, 0x14);
	phy15h = mdio_read (ndev, 0x10, 0x15);
	phy16h = mdio_read (ndev, 0x10, 0x16);
	if ((phy14h != 0x742C) || (phy15h != 0x03c8) || (phy16h != 0x4044)) {
		mdio_write (ndev, 0x10, 0x15, 0x03c8);
		mdio_write (ndev, 0x10, 0x16, 0x4044);
		if (rty_cnt == 0) {
			PRINTK (DRIVER_MSG, PFX "Restore PHY default setting");
			rty_cnt = 1;
		} else if (++rty_cnt > 10) {
			PRINTK (ERROR_MSG, PFX
				"Failed to restore PHY default setting");
			return -EIO;
		}
		goto restart;
	}

	advertise = mdio_read (ndev, 0x10, MII_ADVERTISE) & ~ADVERTISE_ALL;

	switch (ax_local->media) {
	case MEDIA_AUTO:
		//if (echo)
		//	PRINTK (DRIVER_MSG, PFX 
		//		" The media mode is autosense.\n");
		advertise |= ADVERTISE_ALL | 0x400;
		break;

	case MEDIA_100FULL:
		if (echo)
			PRINTK (DRIVER_MSG, PFX 
				" The media mode is forced to 100full.\n");
		advertise |= ADVERTISE_100FULL | 0x400;
		break;

	case MEDIA_100HALF:
		if (echo)
			PRINTK (DRIVER_MSG, PFX 
				" The media mode is forced to 100half.\n");
		advertise |= ADVERTISE_100HALF;
		break;

	case MEDIA_10FULL:
		if (echo)
			PRINTK (DRIVER_MSG, PFX 
				" The media mode is forced to 10full.\n");
		advertise |= ADVERTISE_10FULL;
		break;

	case MEDIA_10HALF:
		if (echo)
			PRINTK (DRIVER_MSG, PFX 
				" The media mode is forced to 10half.\n");
		advertise |= ADVERTISE_10HALF;
		break;
	default:
		advertise |= ADVERTISE_ALL | 0x400;
		break;
	}

	mdio_write (ndev, 0x10, MII_ADVERTISE, advertise);
	mii_nway_restart (&ax_local->mii);

	return 0;
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_init
 * Purpose: Initialize MAC registers
 * ----------------------------------------------------------------------------
 */
static int ax88796b_init (struct net_device *ndev, int startp)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	int ret = 0;

	/* Follow National Semi's recommendations for initing the DP83902. */
	writeb (E8390_NODMA | E8390_PAGE0 | E8390_STOP, ax_base + ADDR_SHIFT16(E8390_CMD));
	writeb (ax_local->bus_width, ax_base + ADDR_SHIFT16(EN0_DCFG));/* 8-bit or 16-bit */

	/* Set AX88796B interrupt active high */
	writeb (0x20, ax_base + ADDR_SHIFT16(EN0_BTCR));

	/* Clear the remote byte count registers. */
	writeb (0x00,  ax_base + ADDR_SHIFT16(EN0_RCNTLO));
	writeb (0x00,  ax_base + ADDR_SHIFT16(EN0_RCNTHI));

	/* Set to monitor and loopback mode -- this is vital!. */
	writeb (E8390_RXOFF, ax_base + ADDR_SHIFT16(EN0_RXCR)); /* 0x20 */
	writeb (E8390_TXOFF, ax_base + ADDR_SHIFT16(EN0_TXCR)); /* 0x02 */

	/* Set the transmit page and receive ring. */
	writeb (NESM_START_PG, ax_base + ADDR_SHIFT16(EN0_TPSR));
	writeb (NESM_RX_START_PG, ax_base + ADDR_SHIFT16(EN0_STARTPG));
	writeb (NESM_RX_START_PG, ax_base + ADDR_SHIFT16(EN0_BOUNDARY));

	/* assert boundary+1 */
	ax_local->current_page = NESM_RX_START_PG + 1;
	writeb (NESM_STOP_PG, ax_base + ADDR_SHIFT16(EN0_STOPPG));

	ax_local->tx_prev_ctepr = 0;
	ax_local->tx_start_page = NESM_START_PG;
	ax_local->tx_curr_page = NESM_START_PG;
	ax_local->tx_stop_page = NESM_START_PG + TX_PAGES;

	/* Clear the pending interrupts and mask. */
	writeb (0xFF, ax_base + ADDR_SHIFT16(EN0_ISR));
	writeb (0x00,  ax_base + ADDR_SHIFT16(EN0_IMR));

	ax88796b_set_macaddr (ndev);

	if (startp) 
	{
		ret = ax88796_PHY_init (ndev, 1);
		if (ret != 0)
			return ret;

		/* Enable AX88796B TQC */
		writeb ((readb (ax_base + ADDR_SHIFT16(EN0_MCR)) | ENTQC_ENABLE),
			ax_base + ADDR_SHIFT16(EN0_MCR));
	
		/* Enable AX88796B Transmit Buffer Ring */
		writeb (E8390_NODMA | E8390_PAGE3 | E8390_STOP,
				ax_base + ADDR_SHIFT16(E8390_CMD));
		writeb (ENTBR_ENABLE, ax_base + ADDR_SHIFT16(EN3_TBR));
		writeb (E8390_NODMA | E8390_PAGE0 | E8390_STOP,
				ax_base + ADDR_SHIFT16(E8390_CMD));

		writeb (0xff,  ax_base + ADDR_SHIFT16(EN0_ISR));
		writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));
		writeb (E8390_NODMA | E8390_PAGE0 | E8390_START,
				ax_base + ADDR_SHIFT16(E8390_CMD));
		writeb (E8390_TXCONFIG, ax_base + ADDR_SHIFT16(EN0_TXCR)); /* xmit on. */

		writeb (E8390_RXCONFIG, ax_base + ADDR_SHIFT16(EN0_RXCR)); /* rx on,  */
		do_set_multicast_list (ndev);	/* (re)load the mcast table */
	}

	return ret;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_media_link
 * Purpose: 
 * ----------------------------------------------------------------------------
 */
static void ax_media_link (struct net_device *ndev)
{
	u16 phy_reg;

	phy_reg = mdio_read (ndev, 0x10, MII_BMCR);

	PRINTK (DRIVER_MSG, "%s: link up, %sMbps, %s-duplex\n",
		ndev->name, (phy_reg & BMCR_SPEED100 ? "100" : "10"),
		(phy_reg & BMCR_FULLDPLX ? "Full" : "Half"));

	netif_carrier_on (ndev);
	netif_wake_queue (ndev);
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_get_link
 * Purpose: 
 * ----------------------------------------------------------------------------
 */
static int ax_get_link (struct ax_device *ax_local)
{
	u8 nway_done = 1;
	u16 phy_reg;

	phy_reg = mdio_read (ax_local->ndev, 0x10, MII_BMCR);
	if (phy_reg & BMCR_ANENABLE)
	{
		nway_done = (mdio_read (ax_local->ndev, 0x10, MII_BMSR) &
					BMSR_ANEGCOMPLETE) ? 1 : 0;
	}

	if (nway_done)
	{
		return mii_link_ok (&ax_local->mii);
	}

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_watchdog
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static void ax88796b_watchdog (unsigned long arg)
{
	struct net_device *ndev = (struct net_device *)(arg);
 	struct ax_device *ax_local = ax_get_priv (ndev);
	unsigned long time_to_wake = AX88796_WATCHDOG_PERIOD_50MS;
	u8 link;
	u16 phy_reg;

	switch (ax_local->state) {
	case wait_autoneg:

		if (ax_get_link (ax_local)) {
			ax_media_link (ndev);
			ax_local->state = chk_link;
			ax88796b_send_data(ndev);
		} else if (ax_local->tick_times-- < 0) {
			ax_local->state = chk_cable_exist;
			ax_local->tick_times = (1800 / 50); //1.8s
		}
		break;
	case chk_link:
		link = ax_get_link (ax_local);

		if (netif_carrier_ok(ndev) != link) {

			if (link) {
				ax_media_link (ndev);
				ax_local->state = chk_link;
			} else {
				netif_stop_queue (ndev);
				netif_carrier_off (ndev);
				PRINTK (DRIVER_MSG, 
					"%s Link down.\n", ndev->name);
				ax_local->state = chk_cable_exist;
				ax_local->tick_times = (1800 / 50); //1.8s
			}
		}
		else if(sbcast_info.bcast_mark)
		{
			ax_local->time_num++;
			if(ax_local->time_num >= sbcast_info.bcast_time/50) //5s
			{
				void *ax_base = ax_local->membase; 
				ax_local->time_num = 0;
				if(sbcast_info.bcast_max && ax_local->boardcast_num > sbcast_info.bcast_max)
				{
					ax_local->storm_mark = 1;
					printk("hndz board cast so mach %d!\n", ax_local->boardcast_num);
					writeb (0x00, ax_base + ADDR_SHIFT16(EN0_RXCR));
					ax_local->boardcast_num  = 0;
					break;
				}
				ax_local->boardcast_num  = 0;
				if(ax_local->storm_mark == 1)
				{
					printk("hndz start boardcast recv!\n");
					ax_local->storm_mark = 0;

					writeb (0x04, ax_base + ADDR_SHIFT16(EN0_RXCR));
				}
				// printk("hndz board cast num %d!\n", ax_local->boardcast_num);
			}
		}
		break;
	case chk_cable_exist:
		phy_reg = mdio_read (ndev, 0x10, 0x12);
		if ((phy_reg != 0x8012) && (phy_reg != 0x8013)) {
			mdio_write (ndev, 0x10, 0x16, 0x4040);
			mii_nway_restart(&ax_local->mii);
			ax_local->state = chk_cable_status;
			time_to_wake = AX88796_WATCHDOG_PERIOD_500MS;
			ax_local->tick_times = (4 * 1000 / 500); //4s
		} else if (--ax_local->tick_times <= 0) {
			mii_nway_restart(&ax_local->mii);
			ax_local->state = chk_cable_exist_again;
			ax_local->tick_times = (6 * 1000 / 50); //6s
		}
		break;
	case chk_cable_exist_again:
		/* if cable disconnected */
		phy_reg = mdio_read (ndev, 0x10, 0x12);
		if ((phy_reg != 0x8012) && (phy_reg != 0x8013)) {
			mii_nway_restart(&ax_local->mii);
			ax_local->state = chk_cable_status;
			time_to_wake = AX88796_WATCHDOG_PERIOD_500MS;
			ax_local->tick_times = (4 * 1000 / 500); //4s
		} else if (--ax_local->tick_times <= 0) {

			ax_local->phy_advertise = mdio_read (ndev, 0x10,
							     MII_ADVERTISE);
			ax_local->phy_bmcr = mdio_read (ndev, 0x10, MII_BMCR);
			mdio_write (ndev, 0x10, MII_BMCR, BMCR_PDOWN);
			time_to_wake = AX88796_WATCHDOG_PERIOD_3S;
			ax_local->state = phy_power_up;
		}
		break;

	case chk_cable_status:

		if (ax_get_link (ax_local)) {
			ax_local->state = chk_link;
		} else if (--ax_local->tick_times <= 0) {

			mdio_write (ndev, 0x10, 0x16, 0x4040);
			mii_nway_restart(&ax_local->mii);
			ax_local->state = chk_cable_exist_again;
			ax_local->tick_times = (6 * 1000 / 50); //6s
		} else {
			time_to_wake = AX88796_WATCHDOG_PERIOD_500MS;
		}
		break;
	case phy_power_up:
		/* Restore default setting */
		mdio_write (ndev, 0x10, MII_BMCR, ax_local->phy_bmcr);
		mdio_write (ndev, 0x10, MII_ADVERTISE, ax_local->phy_advertise);
		mdio_write (ndev, 0x10, 0x16, 0x4044);
		mii_nway_restart(&ax_local->mii);

		ax_local->state = chk_cable_exist_again;
		ax_local->tick_times = (6 * 1000 / 50); //6s
		break;
	default:
		break;
	}

	mod_timer (&ax_local->watchdog, jiffies + time_to_wake);
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_get_hdr
 * Purpose: Grab the 796b specific header
 * ----------------------------------------------------------------------------
 */
static void
ax88796b_get_hdr (struct net_device *ndev, 
		  struct ax_pkt_hdr *hdr, int ring_page)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	u16 *buf = (u16 *)hdr;

	/* This shouldn't happen. If it does, it's the last thing you'll see */
	if (ax_local->dmaing)
	{
		PRINTK (ERROR_MSG, PFX
			" DMAing conflict in ax88796b_get_hdr"
			"[DMAstat:%d][irqlock:%d].\n",
			ax_local->dmaing, ax_local->irqlock);
		return;
	}

	ax_local->dmaing |= 0x01;

	writeb (E8390_NODMA | E8390_PAGE0, ax_base+ ADDR_SHIFT16(E8390_CMD));
	writeb (sizeof (struct ax_pkt_hdr), ax_base + ADDR_SHIFT16(EN0_RCNTLO));
	writeb (0, ax_base + ADDR_SHIFT16(EN0_RCNTHI));
	writeb (0, ax_base + ADDR_SHIFT16(EN0_RSARLO));		/* On page boundary */
	writeb (ring_page, ax_base + ADDR_SHIFT16(EN0_RSARHI));
	writeb (E8390_RREAD, ax_base + ADDR_SHIFT16(E8390_CMD));

	while (( readb (ax_base + ADDR_SHIFT16(EN0_SR)) & ENSR_DMA_READY) == 0);

	*buf = READ_FIFO (ax_base + ADDR_SHIFT16(EN0_DATAPORT));
	*(++buf) = READ_FIFO (ax_base + ADDR_SHIFT16(EN0_DATAPORT));

	writeb (ENISR_RDC, ax_base + ADDR_SHIFT16(EN0_ISR));	/* Ack intr. */
	ax_local->dmaing = 0;

	le16_to_cpus (&hdr->count);
}


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_block_input
 * Purpose:
 * ----------------------------------------------------------------------------
 */
#if 0
static void 
ax88796b_block_input (struct net_device *ndev, int count,
			struct sk_buff *skb, int ring_offset)
{
    struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	u16 *buf = (u16 *)skb->data;
	u16 i;

	/* This shouldn't happen. If it does, it's the last thing you'll see */
	if (ax_local->dmaing)
	{
		PRINTK (ERROR_MSG, PFX " DMAing conflict in ne_block_input "
			"[DMAstat:%d][irqlock:%d].\n",
			ax_local->dmaing, ax_local->irqlock);
		return;
	}

	ax_local->dmaing |= 0x01;

	writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));
	writeb (count & 0xff, ax_base + ADDR_SHIFT16(EN0_RCNTLO));
	writeb (count >> 8, ax_base + ADDR_SHIFT16(EN0_RCNTHI));
	writeb (ring_offset & 0xff, ax_base + ADDR_SHIFT16(EN0_RSARLO));
	writeb (ring_offset >> 8, ax_base + ADDR_SHIFT16(EN0_RSARHI));
	writeb (E8390_RREAD, ax_base + ADDR_SHIFT16(E8390_CMD));


	while ((readb (ax_base + ADDR_SHIFT16(EN0_SR)) & ENSR_DMA_READY) == 0);

#if (CONFIG_AX88796B_USE_MEMCPY == 1)
	{
		/* make the burst length be divided for 32-bit */
		i = ((count - 2) + 3) & 0x7FC;

		/* Read first 2 bytes */
		*buf = READ_FIFO (ax_base + ADDR_SHIFT16(EN0_DATAPORT));

		/* The address of ++buf should be agigned on 32-bit boundary */
		memcpy (++buf, ax_base + ADDR_SHIFT16(EN0_DATA_ADDR), i);
	}
#else
	{
		// if(debugPrint == 0)
		// {
		// 	sdma_eim_read(count +2);
		// 	tasklet_eim_rx_handle(ndev);
		// 	printk("zty tasklet_eim_rx_handle end!\n");
		// 	debugPrint = 1;
		// 	//while (sdma_read_ok == 0);

		// }
		// else
		{
			// if(debugPrint < 6)
			// {
			// 	printk("read %d start 0x%lx!\n", count, jiffies);
			// }
			for (i = 0; i < count; i += 2)
				*buf++ = READ_FIFO (ax_base + ADDR_SHIFT16(EN0_DATAPORT));

			//for (i = 0; i < count -2; i += 2) {
			//	 *buf++ = zREAD_FIFO (ax_base + ADDR_SHIFT16(EN0_DATAPORT));
			//	// *buf++ = *((u16*)(ax_base + ADDR_SHIFT16(EN0_DATAPORT)));
			//}
			//*buf = READ_FIFO (ax_base + ADDR_SHIFT16(EN0_DATAPORT));
			// if(debugPrint < 6)
			// {
			// 	printk("read end 0x%lx!\n", jiffies);
			// 	debugPrint++;
			// }
			// buf = (u16 *)skb->data;
			// for(i = 0; i < 6; i++)
			// 	printk("0x%x ", *(buf+i));
			// printk("!\n");
		}
	}
#endif

	writeb (ENISR_RDC, ax_base + ADDR_SHIFT16(EN0_ISR));	/* Ack intr. */
	// if(debugPrint == 0)
	// {
	// 	printk("zty receive size 0x%x!\n", count);
	// 	for(i = 0; i < count; i++)
	// 		printk("0x%x ", ((u8 *)skb->data)[i]);
	// 	printk("\n");
	// 	debugPrint = 1;
	// }
	ax_local->dmaing = 0;
}
#endif
/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_trigger_send
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static void 
ax_trigger_send (struct net_device *ndev, unsigned int length, int start_page)
{
 	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;

	writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));
	writeb (length & 0xff, ax_base + ADDR_SHIFT16(EN0_TCNTLO));
	writeb (length >> 8, ax_base + ADDR_SHIFT16(EN0_TCNTHI));
	writeb (start_page, ax_base + ADDR_SHIFT16(EN0_TPSR));

	writeb (E8390_NODMA | E8390_TRANS, ax_base + ADDR_SHIFT16(E8390_CMD));
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_block_output
 * Purpose:
 * ----------------------------------------------------------------------------
 */

static void 
ax_block_output (struct net_device *ndev, int count,
		 const unsigned char *buf, const int start_page)
{
    struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	unsigned long dma_start;

	/* This shouldn't happen. If it does, it's the last thing you'll see */
	if (ax_local->dmaing)
	{
		PRINTK (ERROR_MSG, PFX " DMAing conflict in ne_block_output."
			"[DMAstat:%d][irqlock:%d]\n",
			ax_local->dmaing, ax_local->irqlock);
		return;
	}

	ax_local->dmaing |= 0x01;

	/* Now the normal output. */
	writeb (count & 0xff, ax_base + ADDR_SHIFT16(EN0_RCNTLO));
	writeb (count >> 8,   ax_base + ADDR_SHIFT16(EN0_RCNTHI));
	writeb (0x00, ax_base + ADDR_SHIFT16(EN0_RSARLO));
	writeb (start_page, ax_base + ADDR_SHIFT16(EN0_RSARHI));

	writeb (E8390_RWRITE, ax_base + ADDR_SHIFT16(E8390_CMD));

#if (CONFIG_AX88796B_USE_MEMCPY == 1)
	memcpy ((ax_base + ADDR_SHIFT16(EN0_DATA_ADDR)), buf, ((count+ 3) & 0x7FC));
#else
	{
		u16 i;
		for (i = 0; i < count; i += 2)
			WRITE_FIFO (ax_base + ADDR_SHIFT16(EN0_DATAPORT), *((u16 *)(buf + i)));
		// for (i = 0; i < count -2; i += 2) {

		// 	 zWRITE_FIFO(ax_base + ADDR_SHIFT16(EN0_DATAPORT), *((u16 *)(buf + i)));
		// }
		// WRITE_FIFO (ax_base + ADDR_SHIFT16(EN0_DATAPORT), *((u16 *)(buf + i)));
	}
#endif

	dma_start = jiffies;
	while ((readb(ax_base + ADDR_SHIFT16(EN0_ISR)) & 0x40) == 0) {
		if (jiffies - dma_start > 2*HZ/100) {		/* 20ms */
			PRINTK (ERROR_MSG, PFX
				" timeout waiting for Tx RDC.\n");
			ax88796b_reset (ndev);
			ax88796b_init (ndev, 1);
			break;
		}
	}
	writeb (ENISR_RDC, ax_base + ADDR_SHIFT16(EN0_ISR));	/* Ack intr. */

	ax_local->dmaing = 0;
	return;
}



/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_start_xmit
 * Purpose: begin packet transmission
 * ----------------------------------------------------------------------------
 */
 int ax88796b_start_xmit (struct sk_buff *skb, struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	int send_length;
	unsigned long flags;
	u8 ctepr=0, free_pages=0, need_pages;

	PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);

	send_length = skb->len;
	// dump_stack();
	// mutex_lock(&eim_lock);
	// printk("zty send start!\n");
	// if(sdma_read_ok == 1)
	// {
	// 	printk("zty eim is rx!\n");
	// 	dump_stack();
	// 	return NETDEV_TX_BUSY;
	// }
	spin_lock_irqsave (&ax_local->page_lock, flags);

	writeb (E8390_PAGE0 | E8390_NODMA, ax_base + ADDR_SHIFT16(E8390_CMD));

	writeb (0x00, ax_base + ADDR_SHIFT16(EN0_IMR));
	ax_local->irqlock = 1;

	need_pages = ((send_length - 1) >> 8) + 1;

	ctepr = readb (ax_base + ADDR_SHIFT16(EN0_CTEPR));

	if (ctepr & ENCTEPR_TXCQF) {
		free_pages = 0;
	} else if (ctepr == 0) {

		/*
		 * If someone issues STOP bit of command register at run time,
		 * the TX machine will stop the current transmission and the
		 * CTEPR register will be reset to zero.
		 * This is just precautionary measures.
		 */
		if (ax_local->tx_prev_ctepr != 0) {
			ax_local->tx_curr_page = ax_local->tx_start_page;
			ax_local->tx_prev_ctepr = 0;
		}

		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page;
	}
	else if (ctepr < ax_local->tx_curr_page - 1) {
		free_pages = ax_local->tx_stop_page - ax_local->tx_curr_page + 
					ctepr - ax_local->tx_start_page + 1;
	}
	else if (ctepr > ax_local->tx_curr_page - 1) {
		free_pages = ctepr + 1 - ax_local->tx_curr_page;
	}
	else if (ctepr == ax_local->tx_curr_page - 1) {
		if (ax_local->tx_full)
			free_pages = 0;
		else
			free_pages = TX_PAGES;
	}

	PRINTK (DEBUG_MSG, PFX " tx pkt len %d, need %d pages, free pages %d\n",
		skb->len, need_pages, free_pages);

	if (free_pages < need_pages) {
		PRINTK (DEBUG_MSG, "free_pages < need_pages\n");
		//printk("zty ax88796 start send!\n");
		netif_stop_queue (ndev);
		ax_local->tx_full = 1;
		ax_local->irqlock = 0;	
		writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));
		spin_unlock_irqrestore (&ax_local->page_lock, flags);
		return NETDEV_TX_BUSY;
	}

	if (DEBUG_MSG & DEBUG_FLAGS) {
		int i;
		PRINTK (DEBUG_MSG, PFX " Dump tx pkt %d", skb->len);
		for (i = 0; i < skb->len; i++) {
			if ((i % 16) == 0)
				PRINTK (DEBUG_MSG, "\n");
			PRINTK (DEBUG_MSG, 
				"%02x ", *(skb->data + i));
		}
		PRINTK (DEBUG_MSG, "\n");
	}
	ax_block_output (ndev, send_length, skb->data, ax_local->tx_curr_page);
	ax_trigger_send (ndev, send_length, ax_local->tx_curr_page);
	if (free_pages == need_pages) {
		netif_stop_queue (ndev);
		ax_local->tx_full = 1;
	}
	ax_local->tx_prev_ctepr = ctepr;
	ax_local->tx_curr_page = ((ax_local->tx_curr_page + need_pages) <
		ax_local->tx_stop_page) ? 
		(ax_local->tx_curr_page + need_pages) : 
		(need_pages - (ax_local->tx_stop_page - ax_local->tx_curr_page)
		 + ax_local->tx_start_page);

	ax_local->irqlock = 0;	
	// printk("zty ax88796 start send ii!\n");
	writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));

	spin_unlock_irqrestore (&ax_local->page_lock, flags);
	// mutex_unlock(&eim_lock);

	dev_kfree_skb (skb);

	ndev->trans_start = jiffies;
	ax_local->stat.tx_bytes += send_length;
	// printk("zty send end!\n");
	PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);

	return NETDEV_TX_OK;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_tx_intr
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static void ax_tx_intr (struct net_device *ndev)
{
    struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	int status = readb (ax_base + ADDR_SHIFT16(EN0_TSR));

	ax_local->tx_full = 0;
	if (netif_queue_stopped (ndev))
		netif_wake_queue (ndev);

	/* Minimize Tx latency: update the statistics after we restart TXing. */
	if (status & ENTSR_COL)
		ax_local->stat.collisions++;
	if (status & ENTSR_PTX)
		ax_local->stat.tx_packets++;
	else 
	{
		ax_local->stat.tx_errors++;
		if (status & ENTSR_ABT) 
		{
			ax_local->stat.tx_aborted_errors++;
			ax_local->stat.collisions += 16;
		}
		if (status & ENTSR_CRS) 
			ax_local->stat.tx_carrier_errors++;
		if (status & ENTSR_FU) 
			ax_local->stat.tx_fifo_errors++;
		if (status & ENTSR_CDH)
			ax_local->stat.tx_heartbeat_errors++;
		if (status & ENTSR_OWC)
			ax_local->stat.tx_window_errors++;
	}
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_tx_err
 * Purpose:
 * ----------------------------------------------------------------------------
 */
static void ax_tx_err (struct net_device *ndev)
{
    struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;

	unsigned char txsr = readb (ax_base + ADDR_SHIFT16(EN0_TSR));
	unsigned char tx_was_aborted = txsr & (ENTSR_ABT+ENTSR_FU);

	if (tx_was_aborted)
		ax_tx_intr (ndev);
	else {
		ax_local->stat.tx_errors++;
		if (txsr & ENTSR_CRS) ax_local->stat.tx_carrier_errors++;
		if (txsr & ENTSR_CDH) ax_local->stat.tx_heartbeat_errors++;
		if (txsr & ENTSR_OWC) ax_local->stat.tx_window_errors++;
	}
}
#if 0
static int ax88796b_rx_poll(struct net_device *ndev, struct ax_device *ax_local,
		      int budget)
{
	void *ax_base = ax_local->membase;
	unsigned char rxing_page, this_frame, next_frame;
	unsigned short current_offset;
	struct ax_pkt_hdr rx_frame;
	int pkt_cnt = 0;
	int i;

	//PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);
	// unsigned long time1, time2;
	// time1 = hndz_read_current_timer();
	for ( i = 0 ; i < budget && netif_running(ndev); i++) {

		int pkt_len, pkt_stat;

		/* Get the rx page (incoming packet pointer). */
		rxing_page = readb (ax_base + ADDR_SHIFT16(EN0_CURPAG));

		/* 
		 * Remove one frame from the ring.
		 * Boundary is always a page behind.
		 */
		this_frame = readb (ax_base + ADDR_SHIFT16(EN0_BOUNDARY)) + 1;
		if (this_frame >= ax_local->stop_page)
			this_frame = ax_local->rx_start_page;
		
		if (this_frame == rxing_page) {	/* Read all the frames? */
			break;				/* Done for now */
		}
		current_offset = this_frame << 8;
		ax88796b_get_hdr (ndev, &rx_frame, this_frame);

		// PRINTK (DEBUG_MSG, PFX 
		// 	" Current page 0x%x, boundary page 0x%x\n",
		// 	rxing_page, this_frame);

		pkt_len = rx_frame.count - sizeof (struct ax_pkt_hdr);
		pkt_stat = rx_frame.status;
		next_frame = this_frame + 1 + ((pkt_len + 4) >> 8);

		if (pkt_len < 60  ||  pkt_len > 1518) {
			PRINTK (ERROR_MSG, PFX
				" bogus pkt size: %d, status=%#2x nxpg=%#2x.\n",
				rx_frame.count, rx_frame.status, rx_frame.next);
			ax_local->stat.rx_errors++;
			ax_local->stat.rx_length_errors++;
		} else if ((pkt_stat & 0x0F) == ENRSR_RXOK) {
			struct sk_buff *skb;
			int status;
			skb = dev_alloc_skb (pkt_len + 2);
			if (skb == NULL) {
				PRINTK (ERROR_MSG, PFX
					" Couldn't allocate a sk_buff"
					" of size %d.\n", pkt_len);
				ax_local->stat.rx_dropped++;
				break;
			}

			/* IP headers on 16 byte boundaries */
			skb_reserve (skb, 2);
			skb->dev = ndev;
			skb_put (skb, pkt_len);	/* Make room */
			ax88796b_block_input (ndev, pkt_len, skb,
					current_offset + sizeof (rx_frame));

			// if (DEBUG_MSG & DEBUG_FLAGS) {
			// 	int i;
			// 	PRINTK (DEBUG_MSG, PFX 
			// 		" Dump rx pkt %d", skb->len);
			// 	for (i = 0; i < skb->len; i++) {
			// 		if ((i % 16) == 0)
			// 			PRINTK (DEBUG_MSG, "\n");
			// 		PRINTK (DEBUG_MSG, 
			// 			"%02x ", *(skb->data + i));
			// 	}
			// 	PRINTK (DEBUG_MSG, "\n");
			// }

			skb->protocol = eth_type_trans (skb,ndev);

			status = netif_rx (skb);
			if (status != NET_RX_SUCCESS)
				PRINTK (ERROR_MSG,
					"netif_rx status %d\n", status);

			ndev->last_rx = jiffies;
			ax_local->stat.rx_packets++;
			ax_local->stat.rx_bytes += pkt_len;
			if (pkt_stat & ENRSR_PHY)
			{
				ax_local->stat.multicast++;
				ax_local->boardcast_num++;
				// printk("zty multicast!\n");
			}
			// else
			// {
			// 	// printk("zty boardcast packet!\n");
			// 	ax_local->boardcast_num++;
			// }

			pkt_cnt++;
		} else {
			PRINTK (ERROR_MSG, PFX
				" bogus packet: status=%#2x"
				" nxpg=%#2x size=%d\n",
				rx_frame.status, rx_frame.next, rx_frame.count);
			ax_local->stat.rx_errors++;
			/* NB: The NIC counts CRC, frame and missed errors. */
			if (pkt_stat & ENRSR_FO)
				ax_local->stat.rx_fifo_errors++;
		}
		next_frame = rx_frame.next;

		ax_local->current_page = next_frame;
		writeb (next_frame-1, ax_base + ADDR_SHIFT16(EN0_BOUNDARY));

	}
	// time2 = hndz_read_current_timer();
	// printk("hndz 88796 time %lu num %d!\n", time2-time1, pkt_cnt);
	//PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);

	return pkt_cnt;
}

static int ax88796b_poll(struct napi_struct *napi, int budget)
{
	struct ax_device *ax_local = container_of(napi, struct ax_device, napi);
	struct net_device *dev = ax_local->ndev;
	void *ax_base = ax_local->membase;
	int work_done;
	u8 cmd;
	unsigned long flags;

	//PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);

	spin_lock_irqsave(&ax_local->page_lock, flags);

	work_done = ax88796b_rx_poll(dev, ax_local, budget);

	
	if (work_done < budget) {

		cmd = readb (ax_base + ADDR_SHIFT16(E8390_CMD));
		writeb ((cmd & E8390_PAGE_MASK) , ax_base + ADDR_SHIFT16(E8390_CMD));
		writeb (ENISR_ALL, ax_base + ADDR_SHIFT16(EN0_IMR));
		  __napi_complete(napi);

	}

	
	spin_unlock_irqrestore(&ax_local->page_lock, flags);

	return work_done;
}
#endif
static int ax88796b_dma_poll(struct napi_struct *napi, int quota)
{
	struct ax_device *ax_local = container_of(napi, struct ax_device, napi);


	int work_done;

	struct sk_buff *skb;
	work_done = 0;
	//PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);
	while ((work_done < quota) &&
	       (skb = skb_dequeue(&ax_local->skb_queue))) {

		work_done++;
		netif_receive_skb(skb);
	}

	if (work_done < quota) {

		napi_complete(napi);

		/* Check if there was another interrupt */
		if (!skb_queue_empty(&ax_local->skb_queue))
			napi_reschedule(&ax_local->napi);
	}

	return work_done;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_interrupt
 * Purpose:
 * ----------------------------------------------------------------------------
 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
 irqreturn_t ax_interrupt (int irq, void *dev_id)
#else
static irqreturn_t ax_interrupt (int irq, void *dev_id, struct pt_regs * regs)
#endif
{
	struct net_device *ndev = dev_id;
	int interrupts;
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	u8 CurrImr;

	//PRINTK (INT_MSG, PFX " %s beginning ..........\n", __FUNCTION__);

	if (ndev == NULL) 
	{
		PRINTK (ERROR_MSG,
			"net_interrupt(): irq %d for unknown device.\n", irq);
		return IRQ_RETVAL (0);
	}

	spin_lock (&ax_local->page_lock);

	writeb (E8390_NODMA | E8390_PAGE3, ax_base + ADDR_SHIFT16(E8390_CMD));

	CurrImr = readb (ax_base + ADDR_SHIFT16(EN0_IMR));

	writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));
	writeb (0x00, ax_base + ADDR_SHIFT16(EN0_IMR));

	if (ax_local->irqlock) {
		printk ("Interrupt occurred when irqlock locked\n");
		spin_unlock (&ax_local->page_lock);
		return IRQ_RETVAL (0);
	}

	do {
		if ((interrupts = readb (ax_base + ADDR_SHIFT16(EN0_ISR))) == 0)
			break;
		// printk("zty 88796 interupt 0x%x!\n", interrupts);
		writeb (interrupts, ax_base + ADDR_SHIFT16(EN0_ISR)); /* Ack the interrupts */

		if (interrupts & ENISR_TX) {
			PRINTK (INT_MSG, PFX " TX int\n");
			ax_tx_intr (ndev);
		}

		if (interrupts & (ENISR_RX | ENISR_RX_ERR | ENISR_OVER)) {
			//PRINTK (INT_MSG, PFX " RX int\n");

			if (napi_schedule_prep(&ax_local->napi)) {
				CurrImr = ENISR_ALL & ~(ENISR_RX | ENISR_RX_ERR | ENISR_OVER);
				__napi_schedule(&ax_local->napi);
			}
		}

		if (interrupts & ENISR_TX_ERR) {
			PRINTK (INT_MSG, PFX " TX err int\n");
			ax_tx_err (ndev);
		}

		if (interrupts & ENISR_COUNTERS) {   
			ax_local->stat.rx_frame_errors += 
					readb (ax_base + ADDR_SHIFT16(EN0_COUNTER0));
			ax_local->stat.rx_crc_errors += 
					readb (ax_base + ADDR_SHIFT16(EN0_COUNTER1));
			ax_local->stat.rx_missed_errors += 
					readb (ax_base + ADDR_SHIFT16(EN0_COUNTER2));
			writeb (ENISR_COUNTERS, ax_base + ADDR_SHIFT16(EN0_ISR));
		}

		if (interrupts & ENISR_RDC)
			writeb (ENISR_RDC, ax_base + ADDR_SHIFT16(EN0_ISR));
	} while (0);

	writeb (CurrImr, ax_base + ADDR_SHIFT16(EN0_IMR));
	

	spin_unlock (&ax_local->page_lock);


	//PRINTK (INT_MSG, PFX " %s end ..........\n", __FUNCTION__);

	return IRQ_RETVAL (1);
}



/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_open
 * Purpose: Open/initialize 796b
 * ----------------------------------------------------------------------------
 */
static int ax88796b_open (struct net_device *ndev)
{
	unsigned long flags;
	struct ax_device *ax_local = ax_get_priv (ndev);
	int ret = 0;

	PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);
	gAxStop = 0;
	skb_queue_head_init(&ax_local->skb_queue);
#ifndef AX88796_SDMA_MODE
	//   ret = request_irq (ndev->irq,&ax_interrupt,IRQF_TRIGGER_FALLING, ndev->name, ndev);
	   ret = request_irq (ndev->irq, &ax_dma_interrupt,IRQF_TRIGGER_FALLING, ndev->name, ndev);
#else
	ret = request_irq (ndev->irq,&ax_sdma_interrupt,IRQF_TRIGGER_FALLING, ndev->name, ndev);
#endif
	

	if (ret) {
		PRINTK (ERROR_MSG, PFX
			" unable to get IRQ %d (errno=%d).\n",
			ndev->irq, ret);
		return -ENXIO;
	}

	PRINTK (DEBUG_MSG, PFX " Request IRQ success !!\n");

	spin_lock_irqsave (&ax_local->page_lock, flags);

	ax88796b_reset (ndev);
	ret = ax88796b_init (ndev, 1);
	if (ret != 0) {
		spin_unlock_irqrestore (&ax_local->page_lock, flags);
		free_irq (ndev->irq, ndev);
		return ret;
	}

	if (DEBUG_MSG & DEBUG_FLAGS) {
		PRINTK (DEBUG_MSG, PFX
			"Dump MAC registers after initialization:\n");
		ax88796b_dump_regs (ax_local);
		ax88796b_dump_phy_regs (ax_local);
	}

	netif_carrier_off (ndev);
	netif_start_queue (ndev);
	ax_local->irqlock = 0;
	spin_unlock_irqrestore (&ax_local->page_lock, flags);

	ax_local->state = wait_autoneg;
	ax_local->tick_times = (4 * 1000 / 40); //4S

#if 1
	init_timer (&ax_local->watchdog);
	ax_local->watchdog.function = &ax88796b_watchdog;
	ax_local->watchdog.expires = jiffies + AX88796_WATCHDOG_PERIOD_50MS;

	ax_local->watchdog.data = (unsigned long) ndev;
	add_timer (&ax_local->watchdog);
#endif

	  napi_enable(&ax_local->napi);

	PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_close
 * Purpose: 
 * ----------------------------------------------------------------------------
 */

static int ax88796b_close (struct net_device *ndev)
{
	struct ax_device *ax_local = ax_get_priv (ndev);
	unsigned long flags;
	int i = 0;
	// void *ax_base = ax_local->membase;

	gAxStop = 1;
	printk("hndz gAxRxTx_state %d curr 0x%x!\n", gAxRxTx_state, CurrImr);
	printk("hndz netif_queue_stopped state %d!\n", netif_queue_stopped(ndev));



 	PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);

	 napi_disable(&ax_local->napi);
	del_timer_sync (&ax_local->watchdog);
	disable_irq (ndev->irq);

	while(gAxRxTx_state == 1)
	{
		i++;
		msleep(6);
		if(i > 10)
			break;		
	}
	// dma_release_channel(dma_m2m_chan);
	// dma_m2m_chan = NULL;
	spin_lock_irqsave (&ax_local->page_lock, flags);

	ax88796b_init (ndev, 0);

	free_irq (ndev->irq, ndev);
	gAxRxTx_state = 0;
	CurrImr  = 0;
   	spin_unlock_irqrestore (&ax_local->page_lock, flags);

	netif_stop_queue (ndev);

	PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);
	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
const struct net_device_ops ax_netdev_ops = {
	.ndo_open		= ax88796b_open,
	.ndo_stop		= ax88796b_close,
	.ndo_start_xmit		= ax88796b_eim_dma_start_xmit,
	.ndo_get_stats		= ax88796b_get_stats,
	.ndo_set_rx_mode = ax88796b_set_multicast_list,
	.ndo_set_mac_address 	= ax88796b_set_mac_address,
};
#endif


/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_probe
 * Purpose: 
 * ----------------------------------------------------------------------------
 */
static int ax_probe (struct net_device *ndev, struct ax_device *ax_local)
{
	int i;
	int reg0;
	void *ax_base = ax_local->membase;

	PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);

	reg0 = readb (ax_base);
	if (reg0 == 0xFF) {
		printk("zty read reg0 error!\n");
		return -ENODEV;
	}

	/* Do a preliminary verification that we have a 8390. */
	{
		int regd;
		writeb (E8390_NODMA | E8390_PAGE1 | E8390_STOP,
			ax_base + ADDR_SHIFT16(E8390_CMD));
		regd = readb (ax_base + ADDR_SHIFT16(EN0_COUNTER0));
		writeb (0xff, ax_base + ADDR_SHIFT16(EN0_COUNTER0));
		writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));
		/* Clear the counter by reading. */
		readb (ax_base + ADDR_SHIFT16(EN0_COUNTER0));
		if (readb (ax_base + ADDR_SHIFT16(EN0_COUNTER0)) != 0) {
			writeb (reg0, ax_base);
			/* Restore the old values. */
			writeb (regd, ax_base + ADDR_SHIFT16(EN0_COUNTER0));
			printk("zty write reg error!\n");
			return -ENODEV;
		}
	}

	printk (version);

	if (DEBUG_MSG & DEBUG_FLAGS) {
		PRINTK (DEBUG_MSG, PFX
			"Dump MAC registers before initialization:\n");
		ax88796b_dump_regs (ax_local);
		ax88796b_dump_phy_regs (ax_local);
	}

	/* Reset card. */
	{
		unsigned long reset_start_time = jiffies;

		readb (ax_base + ADDR_SHIFT16(EN0_RESET));
		while ((readb (ax_base + ADDR_SHIFT16(EN0_SR)) & ENSR_DEV_READY) == 0) {
			if (jiffies - reset_start_time > 2*HZ/100) {
				PRINTK (ERROR_MSG,
					" not found (no reset ack).\n");
					return -ENODEV;
				break;
			}
		}
	}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
	ndev->netdev_ops = &ax_netdev_ops;
#else
	ndev->open			= &ax88796b_open;
	ndev->stop			= &ax88796b_close;
	ndev->hard_start_xmit		= &ax88796b_start_xmit;
	ndev->get_stats			= &ax88796b_get_stats;
	ndev->set_multicast_list	= &ax88796b_set_multicast_list;
	ndev->set_mac_address		= ax88796b_set_mac_address,
#endif

	ndev->ethtool_ops = &ax88796b_ethtool_ops;

	ether_setup (ndev);

	ax_local->tx_start_page = NESM_START_PG;
	ax_local->rx_start_page = NESM_RX_START_PG;
	ax_local->stop_page = NESM_STOP_PG;
	ax_local->media = media;

// 	if (weight == 0) {
// #ifndef AX88796_SDMA_MODE
// 		netif_napi_add(ndev, &ax_local->napi, ax88796b_poll, 4);
// #else
// 		netif_napi_add(ndev, &ax_local->napi, ax88796b_sdma_poll, 4);
// #endif
		
// 		//PRINTK (DRIVER_MSG, "NAPI_WEIGHT (default) = %d\n", 4);
// 	}
// 	else {
// 		netif_napi_add(ndev, &ax_local->napi, ax88796b_poll, weight);
// 		//PRINTK (DRIVER_MSG, "NAPI_WEIGHT= %d\n", weight);
// 	}
	netif_napi_add(ndev, &ax_local->napi, ax88796b_dma_poll, AX88796_WIGH);
#if (CONFIG_AX88796B_8BIT_WIDE == 1)
	ax_local->bus_width = 0;
#else
	ax_local->bus_width = 1;
#endif

	spin_lock_init (&ax_local->page_lock);

	ax88796b_load_macaddr (ndev, ndev->dev_addr);

	PRINTK (DRIVER_MSG, "%s: MAC ADDRESS ",DRV_NAME);
	for (i = 0; i < ETHER_ADDR_LEN; i++) {
		PRINTK (DRIVER_MSG, " %2.2x", ndev->dev_addr[i]);
	}

	//PRINTK (DRIVER_MSG, "\n");
	//PRINTK (DRIVER_MSG, PFX " found at 0x%x, using IRQ %d.\n",
	//	AX88796B_BASE, ndev->irq);

	/* Initialize MII structure */
	ax_local->mii.dev = ndev;
	ax_local->mii.mdio_read = mdio_read;
	ax_local->mii.mdio_write = mdio_write;
	ax_local->mii.phy_id = 0x10;
	ax_local->mii.phy_id_mask = 0x1f;
	ax_local->mii.reg_num_mask = 0x1f;

	ax88796b_init (ndev, 0);

	PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);

	return register_netdev (ndev);
}

#ifdef CONFIG_OF

static const struct imx_weim_devtype imx6_weim_devtype = {
	.cs_index	= 0,
	.cs_regs_count	= 6,
	.cs_stride	= 0x18,
};

static const struct of_device_id asix_dt_ids[] = {
	{.compatible = "fsl,imx6q-weim-asix-phy", .data = &imx6_weim_devtype,},
};

MODULE_DEVICE_TABLE(of, asix_dt_ids);

#else

#endif

static int __init eim_parse_dt(struct platform_device *pdev, void __iomem *base)
{
	int ret, i;
	u32 value[16];
	struct device_node *np = pdev->dev.of_node;	
	const struct of_device_id *of_id = of_match_device(asix_dt_ids, &pdev->dev);
	const struct imx_weim_devtype *devtype = of_id->data;

	ret = of_property_read_u32_array(np, "fsl,weim-cs-timing", value, devtype->cs_regs_count);
	if (ret)
		return ret;
	for (i = 0; i < devtype->cs_regs_count; i++) {
		DEBUG_PRI("0x%x\n", value[i]);
		writel(value[i], base + devtype->cs_index * devtype->cs_stride + i * 4);
	}

	return 0;
}

static int __init eim_init(struct platform_device *pdev, void __iomem *base)
{
	struct clk * clk;
	unsigned long rate;
	int ret;

	/* get the clock */
	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(&pdev->dev, "%s clk_prepare_enable failed!\n", __func__);
		return -EFAULT;
	}
	rate = clk_get_rate(clk);
	printk("eim clk: %ldHz\n", rate);

	eim_parse_dt(pdev, base);

	return 0;
}

void eim_cs_mem_config(void)
{
	#define IMX6Q_GPR1_ADDRS0_128MB		(0x2 << 1)
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
	{   
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX6Q_GPR1_ACT_CS3, 0);
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX6Q_GPR1_ACT_CS2, 0);
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX6Q_GPR1_ACT_CS1, 0);
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX6Q_GPR1_ACT_CS0, 1);

		regmap_update_bits(gpr, IOMUXC_GPR1, IMX6Q_GPR1_ADDRS0_MASK, IMX6Q_GPR1_ADDRS0_128MB); 
	}   
	else
			pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_drv_probe
 * Purpose: Driver get resource and probe
 * ----------------------------------------------------------------------------
 */
static int ax88796b_drv_probe(struct platform_device *pdev)
{
	struct ax_device *ax_local;
	struct resource *res = NULL;
	void *addr, *base;
	int ret, phy_reset, err;
	struct net_device *ndev;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "Device does not have associated DT data\n");
		return -EINVAL;
	}

	PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);

	/* User can overwrite AX88796B's base address */
	if (!mem){

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, MEM_CS0);
		if (!res) {
			printk("%s: get no resource !\n", DRV_NAME);
			return -ENODEV;
		}

		mem = res->start;
	}
	DEBUG_PRI("mem=%x, size=%x\n", mem, resource_size(res));
	boardcast_inifo_init();
/*	
	addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(addr)) {
		dev_err(&pdev->dev, "%s devm_ioremap_resource failed.\n", __func__);
		return PTR_ERR(addr);
	} */

	/* Get the IRQ resource from kernel */
	if(!irq) {
		irq = of_get_named_gpio(np, "phy-interrupts-gpios", 0);
		
		if (!gpio_is_valid(irq)) {
		dev_err(&pdev->dev, "%s get phy-interrupts-gpios failed.\n", __func__);
		return -EINVAL;
		}
	}
	irq = gpio_to_irq(irq);
	/* Request the regions */
	if (!request_mem_region(mem, AX88796B_IO_EXTENT, "ax88796b")) {
		PRINTK (ERROR_MSG, PFX " request_mem_region fail !");
		return -EBUSY;
	}

	addr = ioremap_nocache(mem, AX88796B_IO_EXTENT);
	if (!addr) {
		ret = -EBUSY;
		printk("zty ioremap_nocache error!\n");
		goto release_region;
	}



	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, REG_EIM);
	if (!res) {
		printk("%s: get no resource !\n", DRV_NAME);
		return -ENODEV;
	}
	DEBUG_PRI("base=%x, size=%x\n", res->start, resource_size(res));
	base = ioremap_nocache(res->start, resource_size(res));
	if (!base) {
		ret = -EBUSY;
		printk("zty ioremap_nocache2 error!\n");
		goto release_region;
	}


	//EIM Init
	if (eim_init(pdev, base)) {
		ret = -EFAULT;
		printk("zty eim_init error!\n");
		goto release_region;
	}
	eim_cs_mem_config();
	
	phy_reset = of_get_named_gpio(np, "phy-reset-gpios", 0);
	DEBUG_PRI("phy_reset=%d\n", phy_reset);
	if (!gpio_is_valid(phy_reset)) {
		dev_err(&pdev->dev, "%s get phy-reset-gpio failed.\n", __func__);
		return -EINVAL;
	}

	err = devm_gpio_request_one(&pdev->dev, phy_reset, GPIOF_OUT_INIT_LOW, "phy-reset");
	if (err) {
		dev_err(&pdev->dev, "failed to get phy-reset-gpios: %d\n", err);
		return -EBUSY;
	}
	msleep(20);
	gpio_set_value(phy_reset, 1);
	msleep(20);

	ndev = alloc_etherdev (sizeof (struct ax_device));
	if (!ndev) {
		PRINTK (ERROR_MSG, PFX " Could not allocate device.\n");
		ret = -ENOMEM;
		goto unmap_region;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);
	platform_set_drvdata(pdev, ndev);

	ndev->base_addr = mem;
	ndev->irq = irq;

	ax_local = ax_get_priv (ndev);
	ax_local->membase = addr;
 	ax_local->ndev = ndev;

	if(device_create_file(&pdev->dev, &device_attr_bcast_mark))
	{    //在mytest_device设备目录下创建一个my_device_test属性文件
            // return -1;
		printk("hndz creat file fail!\n");
    }
	if(device_create_file(&pdev->dev, &device_attr_bcast_max))
	{    //在mytest_device设备目录下创建一个my_device_test属性文件
            // return -1;
		printk("hndz creat file fail!\n");
    }
	if(device_create_file(&pdev->dev, &device_attr_bcast_time))
	{    //在mytest_device设备目录下创建一个my_device_test属性文件
            // return -1;
		printk("hndz creat file fail!\n");
    }

#ifdef AX88796_SDMA_MODE
	eim_sdma_init(ndev);
	sdma_open();
#endif 
	eim_sdma_init(ndev);
	//printk("zty ax_probe before!\n");
	ret = ax_probe (ndev, ax_local);
	if (!ret) {
		PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);
		return 0;
	}

	platform_set_drvdata (pdev, NULL);
	free_netdev (ndev);
unmap_region:
	iounmap (addr);
release_region:
	release_mem_region (mem, AX88796B_IO_EXTENT);

	PRINTK (ERROR_MSG, PFX "not found (%d).\n", ret);
	PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);
	return ret;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_suspend
 * Purpose: Device suspend handling function
 * ----------------------------------------------------------------------------
 */
static int
ax88796b_suspend(struct platform_device *p_dev, pm_message_t state)
{
	struct net_device *ndev = dev_get_drvdata(&(p_dev)->dev);
	struct ax_device *ax_local = ax_get_priv (ndev);
	void *ax_base = ax_local->membase;
	unsigned long flags;

	PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);

	if (!ndev || !netif_running (ndev)) {
		PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);
		return 0;
	}

	netif_device_detach (ndev);
	netif_stop_queue (ndev);

	spin_lock_irqsave (&ax_local->page_lock, flags);

	writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));
	writeb (0x00, ax_base + ADDR_SHIFT16(EN0_IMR));

	if (ax_local->wol) {
		u8 pme = 0;

		if (ax_local->wol & WAKEUP_LSCWE) {
			pme |= ENWUCS_LSCWE;
			PRINTK (DEBUG_MSG, PFX 
				" Enable link change wakeup\n");
		}

		if (ax_local->wol & WAKEUP_MP) {
			pme |= ENWUCS_MPEN;
			PRINTK (DEBUG_MSG, PFX 
				" Enable magic packet wakeup\n");
		}

		writeb (E8390_NODMA | E8390_PAGE3, ax_base + ADDR_SHIFT16(E8390_CMD));
		writeb (pme, ax_base + ADDR_SHIFT16(EN3_WUCS));

		/* Enable D1 power saving */
		writeb (ENPMR_D1, ax_base + ADDR_SHIFT16(EN3_PMR));

		/* Set PME output type Push-Pull */
		writeb (E8390_NODMA | E8390_PAGE1, ax_base + ADDR_SHIFT16(E8390_CMD));
		writeb (ENBTCR_PME_PULL, ax_base + ADDR_SHIFT16(EN0_BTCR));
		writeb (E8390_NODMA | E8390_PAGE0, ax_base + ADDR_SHIFT16(E8390_CMD));
	} else {
		/* Enable D2 power saving */
		writeb (E8390_NODMA | E8390_PAGE3, ax_base + ADDR_SHIFT16(E8390_CMD));
		writeb (ENPMR_D2, ax_base + ADDR_SHIFT16(EN3_PMR));
		PRINTK (DEBUG_MSG, PFX " Enable D2 power saving mode\n");
	}

	spin_unlock_irqrestore (&ax_local->page_lock, flags);

	PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);
	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_resume
 * Purpose: Device resume handling function
 * ----------------------------------------------------------------------------
 */
static int
ax88796b_resume (struct platform_device *p_dev)
{
	struct net_device *ndev = dev_get_drvdata(&(p_dev)->dev);
	struct ax_device *ax_local = netdev_priv (ndev);
	void __iomem *ax_base = ax_local->membase;
	unsigned long flags;
	int ret;
	u8 pme;

	PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);

	spin_lock_irqsave (&ax_local->page_lock, flags);

	if (ax_local->wol) {
		writeb (E8390_NODMA | E8390_PAGE3, ax_base + ADDR_SHIFT16(E8390_CMD));
		pme = readb (ax_base + ADDR_SHIFT16(EN3_WUCS));
		if (pme & ENWUCS_LSC) {
			PRINTK (DEBUG_MSG, PFX " Waked by link change\n");
		} else if (pme & ENWUCS_MPR) {
			PRINTK (DEBUG_MSG, PFX " Waked by magic packet\n");
		}
	} else {
		writeb (0xFF, ax_base + ADDR_SHIFT16(EN0_RESET));
		PRINTK (DEBUG_MSG, PFX " Host wakeup\n");
		mdelay (100);
	}

	netif_device_attach (ndev);

	ax88796b_reset (ndev);
	ret = ax88796b_init (ndev, 1);
	gAxStop = 0;
	spin_unlock_irqrestore (&ax_local->page_lock, flags);

	if (ret == 0)
		netif_start_queue (ndev);

	PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_exit_module
 * Purpose: Driver clean and exit
 * ----------------------------------------------------------------------------
 */
static int __exit ax88796b_exit_module(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata (pdev);
	struct ax_device *ax_local = netdev_priv (ndev);
	void __iomem *ax_base = ax_local->membase;

	PRINTK (DEBUG_MSG, PFX " %s beginning ..........\n", __FUNCTION__);
	printk("hndz ax88796 exit!\n");

	if (dma_m2m_rxchan) {
		dma_release_channel(dma_m2m_rxchan);
		dma_m2m_rxchan = NULL;

		dma_free_coherent(NULL, 2048, (void *)gEIMrxbuf, gdma_dst);
		gEIMrxbuf = NULL;
	}

	if (dma_m2m_txchan) {
		dma_release_channel(dma_m2m_txchan);
		dma_m2m_txchan = NULL;

		dma_free_coherent(NULL, 2048, (void *)gEIMtxbuf, gdma_src);
		gEIMtxbuf = NULL;
	}

	platform_set_drvdata (pdev, NULL);
	unregister_netdev (ndev);
	iounmap (ax_base);
	release_mem_region (mem, AX88796B_IO_EXTENT);
	free_netdev (ndev);

	PRINTK (DEBUG_MSG, PFX " %s end ..........\n", __FUNCTION__);

	return 1;
}

/*Fill platform driver information*/
static struct platform_driver ax88796b_driver = {
	.driver	= {
		.name    = "ax88796b",
		.owner	 = THIS_MODULE,
		.of_match_table	= asix_dt_ids,
	},
	.probe   = ax88796b_drv_probe,
	.remove  = ax88796b_exit_module,
	.suspend = ax88796b_suspend,
	.resume  = ax88796b_resume,
};

static int __init
ax88796b_init_mod(void)
{
	return platform_driver_register (&ax88796b_driver);
}

/*
 * ----------------------------------------------------------------------------
 * Function Name: ax88796b_exit_module
 * Purpose: Platform driver unregister 
 * ----------------------------------------------------------------------------
 */
static void __exit
ax88796b_cleanup(void)
{
	/* Unregister platform driver*/
	platform_driver_unregister (&ax88796b_driver);
}

module_init(ax88796b_init_mod);
//device_initcall_sync(ax88796b_init_mod);
//late_initcall(ax88796b_init_mod);
module_exit(ax88796b_cleanup);

