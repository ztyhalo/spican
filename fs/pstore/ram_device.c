/*
 * Copyright (C) 2012 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/memblock.h>
#include <linux/rslib.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/pstore_ram.h>
#include <asm/page.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/pstore.h>
#include <linux/time.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/compiler.h>



static struct ramoops_platform_data ramoops_data = {
        .mem_size           =  0x2000000,
        .mem_address        =  0x32000000,
        .record_size        =  0x40000,
		.console_size    	=  0x80000,
		.ftrace_size    	=  0x80000,
        //.dump_oops              = <...>,
        //.ecc                    = <...>,
};


/*
 * platform设备结构体 
 */
static struct platform_device ramoops_dev = {
	.name = "ramoops",
	//.id = -1,
	.dev = {
		.platform_data = &ramoops_data,
	},
	//.num_resources = ARRAY_SIZE(led_resources),
	//.resource = led_resources,
};
		
/*
 * @description	: 设备模块加载 
 * @param 		: 无
 * @return 		: 无
 */
static int __init ramdevice_init(void)
{
	int ret;
	printk("wwwww ramdevice_init\n");
	ret = platform_device_register(&ramoops_dev);
	if (ret) {
		printk("wwww unable to register platform device\n");
	}
	return ret;
	//return platform_device_register(&ramdevice);
}

/*
 * @description	: 设备模块注销
 * @param 		: 无
 * @return 		: 无
 */
static void __exit ramdevice_exit(void)
{
	platform_device_unregister(&ramoops_dev);
}

module_init(ramdevice_init);
module_exit(ramdevice_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("wd");

