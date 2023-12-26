/*
 * drivers/net/phy/realtek.c
 *
 * Driver for Realtek PHYs
 *
 * Author: Johnson Leung <r58129@freescale.com>
 *
 * Copyright (c) 2004 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/phy.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/mdio.h>

#define PHY_ID_BCM89881	0xae025032


MODULE_DESCRIPTION("BCM PHY driver");
MODULE_AUTHOR("zhaotongyang");
MODULE_LICENSE("GPL");


static inline int zty_c45_read(struct phy_device *dev, int devad, int reg)
{
	return mdiobus_c45_read(dev->bus, dev->addr, devad, reg);
}

static inline int zty_c45_write(struct phy_device *dev, int devad, int reg, int val)
{
	return mdiobus_c45_write(dev->bus, dev->addr, devad, reg, val);
}
// static int rtlphy_config_init(struct phy_device *phydev)
// {
//      int val;

//      val = phy_read(phydev, 3);
//      netdev_info(phydev->attached_dev, "rtlphy_config_init done for phy id 0x%x.\n", val);
//      return 0;
//  }


// static int rtl821x_ack_interrupt(struct phy_device *phydev)
// {
// 	int err;

// 	err = phy_read(phydev, RTL821x_INSR);

// 	return (err < 0) ? err : 0;
// }

// static int rtl8211b_config_intr(struct phy_device *phydev)
// {
// 	int err;

// 	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
// 		err = phy_write(phydev, RTL821x_INER,
// 				RTL821x_INER_INIT);
// 	else
// 		err = phy_write(phydev, RTL821x_INER, 0);

// 	return err;
// }

// static int rtl8211e_config_intr(struct phy_device *phydev)
// {
// 	int err;

// 	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
// 		err = phy_write(phydev, RTL821x_INER,
// 				RTL8211E_INER_LINK_STATUS);
// 	else
// 		err = phy_write(phydev, RTL821x_INER, 0);

// 	return err;
// }

// static int rtl8211f_ack_interrupt(struct phy_device *phydev)
// {
// 	int err;

// 	phy_write(phydev, RTL8211F_PAGE_SELECT, 0xa43);
// 	err = phy_read(phydev, RTL8211F_INSR);
// 	/* restore to default page 0 */
// 	phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0);

// 	return (err < 0) ? err : 0;
// }

// static int rtl8211f_config_intr(struct phy_device *phydev)
// {
// 	int err;

// 	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
// 		err = phy_write(phydev, RTL821x_INER,
// 				RTL8211F_INER_LINK_STATUS);
// 	else
// 		err = phy_write(phydev, RTL821x_INER, 0);

// 	return err;
// }

// static int rtl8211f_config_init(struct phy_device *phydev)
// {
// 	int ret;
// 	u16 reg;

// 	ret = rtlphy_config_init(phydev);
// 	if (ret < 0)
// 		return ret;

// 	//1000m led green
// 	phy_write(phydev, RTL8211F_PAGE_SELECT, 0xd04);
// 	phy_write(phydev, 0x10, 0x6359);
// 	/* restore to default page 0 */
// 	phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0);



// 	phy_write(phydev, RTL8211F_PAGE_SELECT, 0xd08);
// 	reg = phy_read(phydev, 0x11);

// 	/* enable TX-delay for rgmii-id and rgmii-txid, otherwise disable it */
// 	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_ID ||
// 	    phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID)
// 		reg |= RTL8211F_TX_DELAY;
// 	else
// 		reg &= ~RTL8211F_TX_DELAY;

// 	phy_write(phydev, 0x11, reg);

// 	//tx delay enable
// 	phy_write(phydev, 0x11, 0x109);
// 	reg = phy_read(phydev,0x11);
// 	pr_err("ydx page 0xd08 register 0x11 is 0x%x\n",reg);

// 	//rx delay enable
// 	phy_write(phydev, 0x15, 0x19);
// 	reg = phy_read(phydev,0x15);
// 	pr_err("ydx page 0xd08 register 0x15 is 0x%x\n",reg);
// 	/* restore to default page 0 */
// 	phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0);



// 	return 0;
// }

static int bcm89881_config_init(struct phy_device *phydev)
{
	/* Temporarily just say we support everything */
	int val;
	int tc10_ctrl, tc10_disable, superisolate;
	u32 features = 0;
//	printk("zty gen10g config init!\n");
//	gen10g_get_features(phydev);
//	phydev->supported = SUPPORTED_10000baseT_Full;
	
	int speed = 1000;
	int polarity_ena_100 = 1;
	
	int autoneg = 0;
	int workmode= 0;
	int testoff_100m = 0;
	int AutoNegForce_MS = 0; //能设置

	phydev->advertising = SUPPORTED_10000baseT_Full;
	phydev->link = 0;
	features = (SUPPORTED_TP | SUPPORTED_MII
			| SUPPORTED_AUI | SUPPORTED_FIBRE |
			SUPPORTED_BNC);

	// val = mdiobus_c45_read(phydev->bus, phydev->addr, 1, 0x0834);

	// printk("zty read 0x0834 val 0x%x!\n", val);

	// val = mdiobus_c45_read(phydev->bus, phydev->addr, 1, 0x8b00);

	// printk("zty read 0x8b00 val 0x%x!\n", val);


	// zty_c45_write(phydev, 1, 0x0834, 0x8001);
	// zty_c45_write(phydev, 1, 0x8b00, 0x0003);
	// msleep(500);

	// val = mdiobus_c45_read(phydev->bus, phydev->addr, MDIO_MMD_AN, 0x0201);

	// printk("zty read mdio stat1 0x%x!\n", val);

	// 	val = mdiobus_c45_read(phydev->bus, phydev->addr, 1, 1);
	// printk("zty read link state 0x%x!\n", val);

	// return 0;
    //1000M disable_autoneg slave
// 	tc10_ctrl = zty_c45_read(phydev, 0x1e, 0x00f0);
// 	if(tc10_ctrl < 0)
// 	{

// 		printk("zty read tc10 ctrl error!\n");
// 		return  tc10_ctrl;
// 	}
// 	else
// 	{
// 		printk("zty devad 0x1e address 0xf0 val 0x%x!\n", tc10_ctrl);
// 		tc10_disable = tc10_ctrl & 0x8000;
// 	}

// 	superisolate = zty_c45_read(phydev, 1, 0x932a);
// 	if(superisolate < 0)
// 	{
// 		printk("zty read superisolate error!\n");
// 		return superisolate;
// 	}
// 	else
// 	{
// 		printk("zty devad 0x1 address 0x932a val 0x%x!\n", superisolate);
// 		superisolate = superisolate & 0x0020;
// 	}

// 	if(tc10_disable == 0)//tc10 able
// 	{
// 		printk("zty tc10 disbale!\n");
// 		zty_c45_write(phydev, 1, 0x8f02, 0x0001);
// 		zty_c45_write(phydev, 0x1e, 0x00f0, tc10_ctrl|0x800);
// 		zty_c45_write(phydev, 0x1e, 0x00f0, tc10_ctrl);
// 	}
// 	else if (superisolate == 0)
// 	{
// 		zty_c45_write(phydev, 1, 0x8f02, 0x0001);
// 	}

// 	printk("zty read 900b val 0x%x!\n",zty_c45_read(phydev, 0x1, 0x900b));
// 	zty_c45_write(phydev, 1, 0x900b, 0x0000);
// 	 zty_c45_write(phydev, 1, 0x0834, 0x8001);
// 	zty_c45_write(phydev, 7, 0x0200, 0x0200);

// 	if(tc10_disable == 0 || superisolate == 0)
// 	{
// 		zty_c45_write(phydev, 1, 0x8f02, 0x0000);
// 	}
// 	else
// 		zty_c45_write(phydev, 1, 0x932a, 0x0002);
// 	zty_c45_write(phydev, 1, 0x931d, 0x3410);
// 	zty_c45_write(phydev, 1, 0x931e, 0x3863);
// 	zty_c45_write(phydev, 1, 0xa027, 0x0317);
// 	zty_c45_write(phydev, 1, 0x9319, 0x2508);

// 	// val = zty_c45_read(phydev, 1, 0);
// 	// printk("zty read 1 0 val 0x%x!\n", val);
// 	// val = zty_c45_read(phydev, 3, 0);
// 	// printk("zty read 3 0 val 0x%x!\n", val);


// 	// zty_c45_write(phydev, 3, 0, val |(1 << 15));

// 	// val = zty_c45_read(phydev, 1, 0);
// 	// printk("zty read 1 0 val 0x%x!\n", val);
// 	// zty_c45_write(phydev, 1, 0, val |(1 << 15));

// 	phydev->autoneg = AUTONEG_DISABLE;
// 	autoneg = 0;
// 	phydev->speed = 100;
// 	phydev->duplex = DUPLEX_FULL;

// 	phydev->speed = 1000;
// 	features |= SUPPORTED_1000baseT_Full;

// 	phydev->supported = features;
// 	phydev->advertising = features;

// 	return 0;

// 	zty_c45_write(phydev, 1, 0, 0x8040);

// 	val = mdiobus_c45_read(phydev->bus, phydev->addr, 1, 0x8b00);

// 	printk("zty read 0x8b00 val 0x%x!\n", val);
// 	val = mdiobus_c45_read(phydev->bus, phydev->addr, 1, 0x0834);

// 	printk("zty read 0x0834 val 0x%x!\n", val);
	

//	zty_c45_write(phydev, 1, 0x0834, 0x8000);//slave 100
//	zty_c45_write(phydev, 1, 0x0834, 0x4000);//master 100
//	zty_c45_write(phydev, 1, 0x0834, 0x4001);//master 1000
// 	 zty_c45_write(phydev, 1, 0x0834, 0x8000);//slave 1000
// 	zty_c45_write(phydev, 0x07, 0x0200, 0x0200);

// 	return 0;
//	zty_c45_write(phydev, 1, 0x8b00, 0x0003);
//	zty_c45_write(phydev, 1, 0x0834, 0xc001);
//	zty_c45_write(phydev, 1, 0x0834, 0xc001);

	val = mdiobus_c45_read(phydev->bus, phydev->addr, MDIO_MMD_AN, 0x0200); //读取auto controlregister

//	printk("zty devad 7 address 0x200 val 0x%x!\n", val);

	//zty_c45_write(phydev, 7, 0x0200, val|(1 << 12));

//	val = mdiobus_c45_read(phydev->bus, phydev->addr, MDIO_MMD_AN, 0x0200); 

	//printk("zty devad 7 address 0x200 val 0x%x!\n", val);

	//zty_c45_write(phydev, 7, 0x0200, val|(1 << 9));
	// while(1)
	// {
	// 	val = mdiobus_c45_read(phydev->bus, phydev->addr, MDIO_MMD_AN, 0x0201); 
	// 	printk("zty devad 7 address 0x201 val 0x%x!\n", val);
	// 	if(val &(1 << 5))
	// 	{
	// 		val = mdiobus_c45_read(phydev->bus, phydev->addr, MDIO_MMD_AN, 0x0203); 
	// 		printk("zty auto ok 0x%x!\n",val);
	// 		break;
	// 	}
	// 	else
	// 		msleep(900);
	// }

	if(val & (1 << 12))
	{
		//printk("zty rxd3 is hight!\n");
		phydev->autoneg = AUTONEG_ENABLE;
		features |= SUPPORTED_Autoneg;
		autoneg = 1;
	}
	else
	{
		//printk("zty rxd3 is low!\n");

		phydev->autoneg = AUTONEG_DISABLE;
		autoneg = 0;
		phydev->speed = 100;
		phydev->duplex = DUPLEX_FULL;
	}
//	val = mdiobus_c45_read(phydev->bus, phydev->addr, MDIO_MMD_AN, 0x0201);

//	printk("zty read mdio stat1 0x%x!\n", val);

//	zty_c45_write(phydev, 1, 0x0834, 0x8001);

//	if(val < 0)
//		return val;
//	if (val & MDIO_AN_STAT1_ABLE)
//	{
//		printk("zty phy support autoneg!\n");
//
//	}

	val = zty_c45_read(phydev, 0x1, 0x0834);
	if(val < 0)
	{
		printk("zty read 834 error!\n");
		return val;
	}
	else
	{
//		printk("zty devad 0x1 address 0x834 val 0x%x!\n", val);

		if(val & 0x01)
		{
			speed = 1000;
		}
		else
		{
			speed = 100;
		}
	//	speed = 100;

		if(speed == 100)
		{

			phydev->speed = 100;
			features |= SUPPORTED_100baseT_Full;
			//printk("zty phy speed %d!\n",speed);

			//zty_c45_write(phydev, 0x01, 0x0834, 0x8000);
		}
		else
		{
			phydev->speed = 1000;
			features |= SUPPORTED_1000baseT_Full;
			//printk("zty phy speed %d!\n",speed);

			//zty_c45_write(phydev, 0x01, 0x0834, 0x8001);
		}

		if(val & (1 << 14))
		{
			workmode = 1;
			//printk("zty configure phy as master!\n");
		}
		// else
		// 	printk("zty configure phy as slave!\n");

		if(speed == 100 && autoneg == 0)
			testoff_100m = 0;
	}

	val = zty_c45_read(phydev, 0x1, 0xa015);
//	printk("zty read rgmii mode 0x%x!\n", val);

	tc10_ctrl = zty_c45_read(phydev, 0x1e, 0x00f0);
	if(tc10_ctrl < 0)
	{

		printk("zty read tc10 ctrl error!\n");
		return  tc10_ctrl;
	}
	else
	{
		//printk("zty devad 0x1e address 0xf0 val 0x%x!\n", tc10_ctrl);
		tc10_disable = tc10_ctrl & 0x8000;
	}

	superisolate = zty_c45_read(phydev, 1, 0x932a);
	if(superisolate < 0)
	{
		printk("zty read superisolate error!\n");
		return superisolate;
	}
	else
	{
		//printk("zty devad 0x1 address 0x932a val 0x%x!\n", superisolate);
		superisolate = superisolate & 0x0020;
	}

	if(tc10_disable == 0)//tc10 able
	{
		zty_c45_write(phydev, 1, 0x8f02, 0x0001);
		if(speed == 1000)
		{
			zty_c45_write(phydev, 0x1e, 0x00f0, tc10_ctrl|0x800);
			zty_c45_write(phydev, 0x1e, 0x00f0, tc10_ctrl);
		}

	}
	else if (superisolate == 0)
	{
		zty_c45_write(phydev, 1, 0x8f02, 0x0001);
	}

	zty_c45_write(phydev, 1, 0x900b, 0x0000);
//	if(speed = 100)
//		zty_c45_write(phydev, 0x01, 0x0834, 0x8000);
//	else
//		zty_c45_write(phydev, 0x01, 0x0834, 0x8001);
//	zty_c45_write(phydev, 0x07, 0x0200, 0x0200);
//
//	if(tc10_disable == 0 || superisolate == 0)
//	{
//		zty_c45_write(phydev, 1, 0x8f02, 0x0000);
//	}

	if(polarity_ena_100)
	{
//		zty_c45_write(phydev, 1, 0x8b02, 0x0001);
		zty_c45_write(phydev, 1, 0x8b02, 0xb265);
	}

	if(speed == 100 || autoneg)
	{
		zty_c45_write(phydev, 0x01, 0x8130, 0x798d);
		zty_c45_write(phydev, 0x01, 0x8131, 0x0688);
		zty_c45_write(phydev, 0x01, 0x8132, 0xa405);
		zty_c45_write(phydev, 0x01, 0x8133, 0x2110);
		zty_c45_write(phydev, 0x01, 0x8134, 0xbf04);
		zty_c45_write(phydev, 0x01, 0x8135, 0x1818);
		zty_c45_write(phydev, 0x01, 0x8136, 0x2181);
		zty_c45_write(phydev, 0x01, 0x8140, 0x94a9);
		zty_c45_write(phydev, 0x01, 0x8141, 0x0688);
		zty_c45_write(phydev, 0x01, 0x8142, 0xa405);
		zty_c45_write(phydev, 0x01, 0x8143, 0x2110);
		zty_c45_write(phydev, 0x01, 0x8144, 0xbf84);
		zty_c45_write(phydev, 0x01, 0x8145, 0x1818);
		zty_c45_write(phydev, 0x01, 0x8146, 0x0209);

		if(autoneg)
		{
			  zty_c45_write(phydev, 0x01, 0x8150, 0x0080);
			  zty_c45_write(phydev, 0x01, 0x8151, 0x0001);
			  zty_c45_write(phydev, 0x01, 0x8152, 0x0000);
			  zty_c45_write(phydev, 0x01, 0x8153, 0x0014);
			  zty_c45_write(phydev, 0x01, 0x8154, 0x0304);
			  zty_c45_write(phydev, 0x01, 0x8155, 0x8d3e);
			  zty_c45_write(phydev, 0x01, 0x8156, 0x25bc);
			  zty_c45_write(phydev, 0x01, 0x8160, 0x00e7);
			  zty_c45_write(phydev, 0x01, 0x8161, 0x8000);
			  zty_c45_write(phydev, 0x01, 0x8162, 0x0000);
			  zty_c45_write(phydev, 0x01, 0x8163, 0x0104);
			  zty_c45_write(phydev, 0x01, 0x8164, 0x7f04);
			  zty_c45_write(phydev, 0x01, 0x8165, 0x1a18);
			  zty_c45_write(phydev, 0x01, 0x8166, 0x002d);
			  zty_c45_write(phydev, 0x01, 0x8170, 0x00e8);
			  zty_c45_write(phydev, 0x01, 0x8171, 0xc088);
			  zty_c45_write(phydev, 0x01, 0x8172, 0x0000);
			  zty_c45_write(phydev, 0x01, 0x8173, 0x0114);
			  zty_c45_write(phydev, 0x01, 0x8174, 0x7f04);
			  zty_c45_write(phydev, 0x01, 0x8175, 0x9d1a);
			  zty_c45_write(phydev, 0x01, 0x8176, 0x04ac);
			  zty_c45_write(phydev, 0x01, 0x8180, 0x00e9);
			  zty_c45_write(phydev, 0x01, 0x8181, 0x8010);
			  zty_c45_write(phydev, 0x01, 0x8182, 0x0000);
			  zty_c45_write(phydev, 0x01, 0x8183, 0x0114);
			  zty_c45_write(phydev, 0x01, 0x8184, 0x7f04);
			  zty_c45_write(phydev, 0x01, 0x8185, 0x9d1a);
			  zty_c45_write(phydev, 0x01, 0x8186, 0x04ac);
			  zty_c45_write(phydev, 0x01, 0x8190, 0x00ea);
			  zty_c45_write(phydev, 0x01, 0x8191, 0x8008);
			  zty_c45_write(phydev, 0x01, 0x8192, 0x0000);
			  zty_c45_write(phydev, 0x01, 0x8193, 0x0114);
			  zty_c45_write(phydev, 0x01, 0x8194, 0x7f04);
			  zty_c45_write(phydev, 0x01, 0x8195, 0x1d1a);
			  zty_c45_write(phydev, 0x01, 0x8196, 0x04ac);
			  zty_c45_write(phydev, 0x01, 0x81a0, 0x00eb);
			  zty_c45_write(phydev, 0x01, 0x81a1, 0x8009);
			  zty_c45_write(phydev, 0x01, 0x81a2, 0x0000);
			  zty_c45_write(phydev, 0x01, 0x81a3, 0x0114);
			  zty_c45_write(phydev, 0x01, 0x81a4, 0x7f14);
			  zty_c45_write(phydev, 0x01, 0x81a5, 0x1d1a);
			  zty_c45_write(phydev, 0x01, 0x81a6, 0x00ad);
			  zty_c45_write(phydev, 0x01, 0x81b0, 0xeca3);
			  zty_c45_write(phydev, 0x01, 0x81b1, 0x8a8a);
			  zty_c45_write(phydev, 0x01, 0x81b2, 0xaa8c);
			  zty_c45_write(phydev, 0x01, 0x81b3, 0x0114);
			  zty_c45_write(phydev, 0x01, 0x81b4, 0x7f14);
			  zty_c45_write(phydev, 0x01, 0x81b5, 0x1818);
			  zty_c45_write(phydev, 0x01, 0x81b6, 0x002d);
			  zty_c45_write(phydev, 0x01, 0x81c0, 0x00e7);
			  zty_c45_write(phydev, 0x01, 0x81c1, 0x0001);
			  zty_c45_write(phydev, 0x01, 0x81c2, 0x0000);
			  zty_c45_write(phydev, 0x01, 0x81c3, 0x0114);
			  zty_c45_write(phydev, 0x01, 0x81c4, 0x7f14);
			  zty_c45_write(phydev, 0x01, 0x81c5, 0x1818);
			  zty_c45_write(phydev, 0x01, 0x81c6, 0x002d);
		}
		else
		{
			  zty_c45_write(phydev, 0x01, 0x81d0, 0x009d);
			  zty_c45_write(phydev, 0x01, 0x81d1, 0x8000);
			  zty_c45_write(phydev, 0x01, 0x81d2, 0x0000);
			  zty_c45_write(phydev, 0x01, 0x81d3, 0x0104);
			  zty_c45_write(phydev, 0x01, 0x81d4, 0x7f04);
			  zty_c45_write(phydev, 0x01, 0x81d5, 0x1a18);
			  zty_c45_write(phydev, 0x01, 0x81d6, 0x002d);
			  zty_c45_write(phydev, 0x01, 0x81e0, 0x7eef);
			  zty_c45_write(phydev, 0x01, 0x81e1, 0x800a);
			  zty_c45_write(phydev, 0x01, 0x81e2, 0x0007);
			  zty_c45_write(phydev, 0x01, 0x81e3, 0x0014);
			  zty_c45_write(phydev, 0x01, 0x81e4, 0x0300);
			  zty_c45_write(phydev, 0x01, 0x81e5, 0x893e);
			  zty_c45_write(phydev, 0x01, 0x81e6, 0x25bf);
			  zty_c45_write(phydev, 0x01, 0x81f0, 0x007e);
			  zty_c45_write(phydev, 0x01, 0x81f1, 0x6f95);
			  zty_c45_write(phydev, 0x01, 0x81f2, 0x0000);
			  zty_c45_write(phydev, 0x01, 0x81f3, 0x0014);
			  zty_c45_write(phydev, 0x01, 0x81f4, 0x0300);
			  zty_c45_write(phydev, 0x01, 0x81f5, 0x893e);
			  zty_c45_write(phydev, 0x01, 0x81f6, 0x25bf);
		}

		// if(workmode)
		// {
		// 	if(autoneg)
		// 	{
		// 		zty_c45_write(phydev, 0x01, 0x8032, 0xe38c);
		// 		zty_c45_write(phydev, 0x01, 0x8033, 0xe57f);
		// 	}
		// }

		if(testoff_100m && autoneg)
		{
			if(workmode)//master mode
			{
			      zty_c45_write(phydev, 0x01, 0x8032, 0xe38c);
			      zty_c45_write(phydev, 0x01, 0x8033, 0xe57f);
			}
			else
			{
			      zty_c45_write(phydev, 0x01, 0x8031, 0xe4a8);
			      zty_c45_write(phydev, 0x01, 0x8033, 0xe69c);
			}
		}
		else if(testoff_100m == 0)
		{
			if(workmode)
			{
				if(autoneg)
				{
			          zty_c45_write(phydev, 0x01, 0x8031, 0x0079);
			          zty_c45_write(phydev, 0x01, 0x8033, 0xe57f);
				}
				else
				{
					zty_c45_write(phydev, 0x01, 0x8033, 0xee7d);
				}
				zty_c45_write(phydev, 0x01, 0x8032, 0xe38c);
			}
			else
			{
				if(autoneg)
				{
			          zty_c45_write(phydev, 0x01, 0x8032, 0x0094);
			          zty_c45_write(phydev, 0x01, 0x8033, 0xe69c);
				}
				else
				{
					zty_c45_write(phydev, 0x01, 0x8033, 0xed9c);
				}
				zty_c45_write(phydev, 0x01, 0x8031, 0xe4a8);
			}

		}
		else
		{
			printk("zty phy configure error!\n");
		}
	}

	if(autoneg == 0) //auto neg off
	{
		if(workmode) //master
		{
			if(speed == 1000)
			{
				zty_c45_write(phydev, 0x01, 0x834, 0xc001);
			}
			else
				zty_c45_write(phydev, 0x01, 0x834, 0xc000);
		}
		else
		{
			if(speed == 1000)
			{
				zty_c45_write(phydev, 0x01, 0x834, 0x8001);
//				zty_c45_write(phydev, 0x07, 0x0203, 0x0080);
//				zty_c45_write(phydev, 0x07, 0x0202, 0x1001);
			}
			else
				zty_c45_write(phydev, 0x01, 0x834, 0x8000);
		}
//		val = zty_c45_read(phydev, 0x07, 0x0200);
//		printk("zty autoneg ieee 0x0200 0x%x!\n", val);
//		val = zty_c45_read(phydev, 0x07, 0x0202);
//		printk("zty autoneg ieee 0x0202 0x%x!\n", val);
//		val = zty_c45_read(phydev, 0x07, 0x0203);
//		printk("zty autoneg ieee 0x0203 0x%x!\n", val);
//
//		val = zty_c45_read(phydev, 0x01, 0x8b00);
//		printk("zty autoneg ieee 0x8b00 0x%x!\n", val);
//		zty_c45_write(phydev, 0x01, 0x8b00, 0x0003);

		zty_c45_write(phydev, 0x07, 0x0200, 0x0200);

		// val = zty_c45_read(phydev, 0x07, 0x0200);
		// printk("zty autoneg ieee 0x0200 0x%x!\n", val);

	//	if(speed == 1000)
		{		
			val = zty_c45_read(phydev, 0x01, 0xa010);
			//printk("zty  0xa010 0x%x!\n", val);
			zty_c45_write(phydev, 0x01, 0xa010, val|0x01);
			//zty_c45_write(phydev, 0x01, 0xa010, 0x000);
		}


	}
	else
	{
		if(workmode) //master
		{
			if(speed == 1000)
			{
				zty_c45_write(phydev, 0x07, 0x0203, 0x00b0);
			}
			else
				zty_c45_write(phydev, 0x07, 0x0203, 0x0030);
			if(AutoNegForce_MS)
			{
				zty_c45_write(phydev, 0x07, 0x0202, 0x1001);
			}
			else
				zty_c45_write(phydev, 0x07, 0x0202, 0x0001);
		}
		else
		{
			if(speed == 1000)
			{
				zty_c45_write(phydev, 0x07, 0x0203, 0x00a0);
			}
			else
				zty_c45_write(phydev, 0x07, 0x0203, 0x0020);
			if(AutoNegForce_MS)
			{
				zty_c45_write(phydev, 0x07, 0x0202, 0x1001);
			}
			else
				zty_c45_write(phydev, 0x07, 0x0202, 0x0001);
		}
		zty_c45_write(phydev, 0x07, 0x0200, 0x1200);
	}

	if(tc10_disable == 0 || superisolate == 0)
	{
		val = zty_c45_read(phydev, 0x01, 0x8f02);
		//printk("zty read 0x8f02 val 0x%x!\n", val);
		zty_c45_write(phydev, 0x01, 0x8f02, 0x0000);
	}
	else
	{
		//zty_c45_write(phydev, 0x01, 0x932a, 0x0002);
		printk("zty disable superisolate!\n");
	}

//	val = zty_c45_read(phydev, 0x01, 0x931D);
//	printk("zty read 0x931D val 0x%x 0x3410!\n", val);
//	val = zty_c45_read(phydev, 0x01, 0x931e);
//	printk("zty read 0x931e val 0x%x 0x3863!\n", val);
	zty_c45_write(phydev, 0x01, 0x931D, 0x3410);
	zty_c45_write(phydev, 0x01, 0x931E, 0x3863);
	zty_c45_write(phydev, 0x01, 0xA027, 0x0317);
//	val = zty_c45_read(phydev, 0x01, 0x9319);
//	printk("zty read 0x9319 val 0x%x 0x2508!\n", val);
	zty_c45_write(phydev, 0x01, 0x9319, 0x2508);
//
//	val = zty_c45_read(phydev, 0x01, 0x8130);
//	printk("zty read 8130 val 0x%x 0x798d!\n", val);
//	val = zty_c45_read(phydev, 0x1, 0x0834);
//	printk("zty phy config end 0x%x!\n", val);
	phydev->supported = features;
	phydev->advertising = features;

	// val = zty_c45_read(phydev, 0x1, 0xa015);
	// printk("zty read rgmii mode 0x%x!\n", val);
	return 0;
}

static int bcm89881_config_aneg(struct phy_device *phydev)
{
	return 0;
}

static int bcm89881_read_status(struct phy_device *phydev)
{
//	int devad, reg;
	int val;
	int adv;
	// int err;
	int lpa;
	int lpagb = 0;
	int common_adv;
	int common_adv_gb = 0;
//	u32 mmd_mask = phydev->c45_ids.devices_in_package;

//	phydev->link = 1;
//
//	/* For now just lie and say it's 10G all the time */
//	phydev->speed = SPEED_10000;
//	phydev->duplex = DUPLEX_FULL;
//
//	for (devad = 0; mmd_mask; devad++, mmd_mask = mmd_mask >> 1) {
//		if (!(mmd_mask & 1))
//			continue;
//
//		/* Read twice because link state is latched and a
//		 * read moves the current state into the register
//		 */
//		phy_read_mmd(phydev, devad, MDIO_STAT1);
//		reg = phy_read_mmd(phydev, devad, MDIO_STAT1);
//		if (reg < 0 || !(reg & MDIO_STAT1_LSTATUS))
//			phydev->link = 0;
//	}

	val = mdiobus_c45_read(phydev->bus, phydev->addr, 1, 1);
//	printk("zty read link state 0x%x!\n", val);

	if((val >= 0) && (val &MDIO_STAT1_LSTATUS))
	{
		if(phydev->link ==0)
			printk("zty find link!\n");
		phydev->link = 1;

	}
	else
	{
		printk("zty find link down!\n");
		phydev->link = 0;
	}

	if (AUTONEG_ENABLE == phydev->autoneg) {
		printk("zty find state auto neg !\n");
		if (phydev->supported & (SUPPORTED_1000baseT_Half
					| SUPPORTED_1000baseT_Full)) {
			lpagb = phy_read(phydev, MII_STAT1000);
			if (lpagb < 0)
				return lpagb;

			adv = phy_read(phydev, MII_CTRL1000);
			if (adv < 0)
				return adv;

			phydev->lp_advertising =
				mii_stat1000_to_ethtool_lpa_t(lpagb);
			common_adv_gb = lpagb & adv << 2;
		}

		lpa = phy_read(phydev, MII_LPA);
		if (lpa < 0)
			return lpa;

		phydev->lp_advertising |= mii_lpa_to_ethtool_lpa_t(lpa);

		adv = phy_read(phydev, MII_ADVERTISE);
		if (adv < 0)
			return adv;

		common_adv = lpa & adv;

		phydev->speed = SPEED_10;
		phydev->duplex = DUPLEX_HALF;
		phydev->pause = 0;
		phydev->asym_pause = 0;

		if (common_adv_gb & (LPA_1000FULL | LPA_1000HALF)) {
			phydev->speed = SPEED_1000;

			if (common_adv_gb & LPA_1000FULL)
				phydev->duplex = DUPLEX_FULL;
		} else if (common_adv & (LPA_100FULL | LPA_100HALF)) {
			phydev->speed = SPEED_100;

			if (common_adv & LPA_100FULL)
				phydev->duplex = DUPLEX_FULL;
		} else
			if (common_adv & LPA_10FULL)
				phydev->duplex = DUPLEX_FULL;

		if (phydev->duplex == DUPLEX_FULL) {
			phydev->pause = lpa & LPA_PAUSE_CAP ? 1 : 0;
			phydev->asym_pause = lpa & LPA_PAUSE_ASYM ? 1 : 0;
		}
	} else {
		int bmcr = mdiobus_c45_read(phydev->bus, phydev->addr, 1, 0);

		if (bmcr < 0)
		{
			printk("zty read 1 0 err!\n");
			return bmcr;
		}



		if((bmcr & (1 << 6)) && !(bmcr & (1 <<13)))
		{
	//		printk("zty control speed 1000!\n");
			phydev->duplex = DUPLEX_FULL;
			phydev->speed = SPEED_1000;
		}
		else if(!(bmcr & (1 << 6)) && (bmcr & (1 <<13)))
		{
	//		printk("zty control speed 100!\n");
			phydev->duplex = DUPLEX_FULL;
			phydev->speed = SPEED_100;
		}
		else
		{
			printk("zty control speed error!\n");
		}

		bmcr = mdiobus_c45_read(phydev->bus, phydev->addr, 3, 0);
		//printk("zty read 3 0 val 0x%x!\n", bmcr);


		phydev->pause = 0;
		phydev->asym_pause = 0;
	}

//	val = mdiobus_c45_read(phydev->bus, phydev->addr, 1, 0x0901);
//	printk("zty read 1000 link state 0x%x!\n", val);
//
//	if((val >= 0) && (val &0x01))
//	{
//		phydev->link = 1;
//		printk("zty find link 1000!\n");
//	}
//	else
//		phydev->link = 0;
//
//	val = mdiobus_c45_read(phydev->bus, phydev->addr, 7, 0x0201);
//	printk("zty read auto neg link state 0x%x!\n", val);
//
//	if((val >= 0) && (val &MDIO_STAT1_LSTATUS))
//	{
//		phydev->link = 1;
//		printk("zty find link 1000!\n");
//	}
//	else
//		phydev->link = 0;
//
//	val = mdiobus_c45_read(phydev->bus, phydev->addr, MDIO_MMD_AN, 0x0201);
//
//		printk("zty read mdio stat1 0x%x!\n", val);
//
//		if(val < 0)
//			return val;
//		if (val & MDIO_AN_STAT1_ABLE)
//		{
//			printk("zty phy support autoneg!\n");
//			phydev->supported = SUPPORTED_Autoneg;
//			//autoneg = 1;
//		}
//
//		val = mdiobus_c45_read(phydev->bus, phydev->addr, 0x1, 0x0834);
//
//		printk("zty read 834 val 0x%x!\n", val);
	return 0;
}

static int bcm89881_suspend(struct phy_device *phydev)
{
	return 0;
}

static int bcm89881_resume(struct phy_device *phydev)
{
	return 0;
}
static struct phy_driver bcm89881_driver = {
	.phy_id		= PHY_ID_BCM89881,
	.name		= "Broadcom BCM89881",
	.phy_id_mask	= 0xffffffff,
	.features       = 0,
//	.flags		= PHY_HAS_INTERRUPT,
	.config_init	= bcm89881_config_init,
	.config_aneg	= bcm89881_config_aneg,
	.read_status	= bcm89881_read_status,
	.ack_interrupt	= NULL,
	.config_intr	= NULL,
	.suspend	= bcm89881_suspend,
	.resume		= bcm89881_resume,
	.driver		= { .owner = THIS_MODULE },
};

static int __init bcm89881_init(void)
{
	int ret;

	ret = phy_driver_register(&bcm89881_driver);
	if (ret < 0)
		return -ENODEV;
	return 0;
}

static void __exit bcm89881_exit(void)
{
	phy_driver_unregister(&bcm89881_driver);
}

module_init(bcm89881_init);
module_exit(bcm89881_exit);

// static struct mdio_device_id __maybe_unused bcm89881_tbl[] = {
// 	{ 0x001cc912, 0x001fffff },
// 	{ 0x001cc915, 0x001fffff },
// 	{ 0x001cc916, 0x001fffff },
// 	{ }
// };

// MODULE_DEVICE_TABLE(mdio, bcm89881_tbl);
