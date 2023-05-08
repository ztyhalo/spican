/***************************
***74hc595驱动*************
****************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
//#include <mach/gpio.h>
#include "../../../arch/arm/mach-imx/hardware.h"

#define HC595_EN IMX_GPIO_NR(1,11)
#define HC595_RESET IMX_GPIO_NR(3,26)


#define HC595_SCLK IMX_GPIO_NR(3,19)
#define HC595_DATA IMX_GPIO_NR(3,17)
#define HC595_SCLR IMX_GPIO_NR(3,22)

#define TURN_OFF   0
#define TURN_ON    1
#define TURN_ALL_OFF 3
#define TURN_ALL_ON 4
#define SET_MODE    5

static int hc595_mode;
static int hc595_devid;
static struct cdev hc595_cdev; //char dev
static struct class * hc595_class;
static struct device * hc595_device;


static int hc595_open(struct inode * inode, struct file * file)
{
	printk("hc595 open!\n");
	gpio_request(HC595_RESET, "hc595_reset");
	gpio_request(HC595_SCLK, "hc595_clk");
	gpio_request(HC595_DATA, "hc595_data");
	gpio_request(HC595_SCLR, "hc595_sclr");
	gpio_request(HC595_EN, "hc595_EN");
	gpio_direction_output(HC595_RESET, 1);
	mdelay(5); 
	gpio_direction_output(HC595_RESET, 0);
	mdelay(5);
	gpio_direction_output(HC595_RESET, 1);

	gpio_direction_output(HC595_EN, 0);
	gpio_direction_output(HC595_SCLR, 0);
	gpio_direction_output(HC595_SCLK, 0);
	gpio_direction_output(HC595_DATA, 0);
	return 0;
}



static int write_to_hc595(int hc595_data)
{
	int i;
	printk("set sclr val!\n");
	gpio_set_value(HC595_SCLR, 0);
	for(i = 0; i < 8; i++)
	{
		if((hc595_data<<i) &0x80)
			gpio_set_value(HC595_DATA,1);
		else
			gpio_set_value(HC595_DATA, 0);
		gpio_set_value(HC595_SCLK, 0);
		udelay(5);
		gpio_set_value(HC595_SCLK, 1);
		udelay(5);	
	}
	gpio_set_value(HC595_SCLR, 1);
	return 0;
}

static long hc595_ioctl(struct file * file, unsigned int cmd, unsigned long arg)
{
	static int hc595_val = 0;
	if(arg < 0 || arg > 16)
	{
		printk(KERN_ALERT"HC595 enter is invalid\n");
		return -1;
	}	
	switch(cmd)
	{
		case TURN_ON:
		{
			hc595_val |= (1 << arg);
			write_to_hc595(hc595_val);
			break;
		}
		case TURN_OFF:
		{
			hc595_val &= ~(1 << arg);
			write_to_hc595(hc595_val);
			break;
		}
		
		case TURN_ALL_ON:
		{
			hc595_val  = 0xff; 
			write_to_hc595(hc595_val);
			break;
		}
		case TURN_ALL_OFF:
		{
			hc595_val = 0;
			write_to_hc595(hc595_val);
			break;
		}
		case SET_MODE:
		{
			hc595_mode = arg;
			break;
		}
		default:
		{
			printk(KERN_ALERT"cmd is invalue!\n");
			break;
		}
	}
	return 0;
}
static int hc595_release(struct inode * inode, struct file * file)
{
	return 0;
}


static struct file_operations hc595_fops = 
{
	.owner = THIS_MODULE,
	.open = hc595_open,
	.unlocked_ioctl = hc595_ioctl,
	.release = hc595_release,
};

static int hc595_init(void)
{
	int ret = alloc_chrdev_region(&hc595_devid, 0, 1, "hc595_drv");
	
	if(ret < 0)
	{
		printk(KERN_INFO"HC595 alloc chrdev regionfailed!\n");
		return -1;
	}
	
	cdev_init(&hc595_cdev, &hc595_fops);
	hc595_cdev.owner = THIS_MODULE;
	ret = cdev_add(&hc595_cdev, hc595_devid, 1);
	if(ret < 0)
	{
		printk(KERN_INFO"hc595 cdev add failed!\n");
		return -2;
	}
	hc595_class = class_create(THIS_MODULE, "hc595_class");
	
	hc595_device = device_create(hc595_class, NULL, hc595_devid, NULL, "hc595_device");
	if(hc595_device == NULL)
	{
		printk(KERN_INFO"hc595 device create failed!\n");
		return -3;
	}
	printk("HC595 INIT SUCCESS!!!!!!!!\n");
	return 0;
}


static void hc595_exit(void)
{
	cdev_del(&hc595_cdev);
	unregister_chrdev_region(hc595_devid, 1);
	device_destroy(hc595_class, hc595_devid);
	class_destroy(hc595_class);
}

module_init(hc595_init);
module_exit(hc595_exit);
MODULE_LICENSE("GPL");
