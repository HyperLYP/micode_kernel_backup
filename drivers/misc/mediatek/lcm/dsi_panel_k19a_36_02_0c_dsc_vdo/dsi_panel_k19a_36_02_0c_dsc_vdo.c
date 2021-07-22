/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#  include <linux/string.h>
#  include <linux/kernel.h>
#endif
#include <linux/gpio.h>
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "data_hw_roundedpattern.h"
#endif

#include "lcm_drv.h"
#include "mtk_boot_common.h"
#include <linux/hqsysfs.h>
#ifdef BUILD_LK
#  include <platform/upmu_common.h>
#  include <platform/mt_gpio.h>
#  include <platform/mt_i2c.h>
#  include <platform/mt_pmic.h>
#  include <string.h>
#elif defined(BUILD_UBOOT)
#  include <asm/arch/mt_gpio.h>
#endif

#ifdef mdelay
#undef mdelay
#endif

#ifdef udelay
#undef udelay
#endif


#ifdef BUILD_LK
#  define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#  define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#  define LCM_LOGI(fmt, args...)  pr_err("[KERNEL/"LOG_TAG"]"fmt, ##args)
#  define LCM_LOGD(fmt, args...)  pr_err("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID_nt36672c 0x83
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#  include <linux/kernel.h>
#  include <linux/module.h>
#  include <linux/fs.h>
#  include <linux/slab.h>
#  include <linux/init.h>
#  include <linux/list.h>
#  include <linux/i2c.h>
#  include <linux/irq.h>
#  include <linux/uaccess.h>
#  include <linux/interrupt.h>
#  include <linux/io.h>
#  include <linux/platform_device.h>
#endif

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "lcm_i2c.h"

#define FRAME_WIDTH			(1080)
#define FRAME_HEIGHT			(2400)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH		(67716)
#define LCM_PHYSICAL_HEIGHT		(150480)
#define LCM_DENSITY			(405)

#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE		0xFFFD
#define REGFLAG_RESET_LOW		0xFFFE
#define REGFLAG_RESET_HIGH		0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifdef CONFIG_MTK_MT6382_BDG
#define DSC_ENABLE
#endif

/* i2c control start */

#define LCM_I2C_ADDR 0x3E
#define LCM_I2C_BUSNUM  1	/* for I2C channel 0 */
#define LCM_I2C_ID_NAME "I2C_LCD_BIAS"


static unsigned ENP = 490; //gpio165
static unsigned ENN = 494; //gpio169

#define GPIO_LCD_BIAS_ENP   ENP
#define GPIO_LCD_BIAS_ENN   ENN


#ifdef CONFIG_MI_ERRFLAG_ESD_CHECK_ENABLE

#endif
/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int _lcm_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int _lcm_i2c_remove(struct i2c_client *client);


/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct _lcm_i2c_dev {
	struct i2c_client *client;

};

static const struct of_device_id _lcm_i2c_of_match[] = {
	{ .compatible = "mediatek,I2C_LCD_BIAS", },
	{},
};

static const struct i2c_device_id _lcm_i2c_id[] = {
	{LCM_I2C_ID_NAME, 0},
	{}
};

static struct i2c_driver _lcm_i2c_driver = {
	.id_table = _lcm_i2c_id,
	.probe = _lcm_i2c_probe,
	.remove = _lcm_i2c_remove,
	/* .detect               = _lcm_i2c_detect, */
	.driver = {
		   .owner = THIS_MODULE,
		   .name = LCM_I2C_ID_NAME,
		   .of_match_table = _lcm_i2c_of_match,
		   },

};

/*****************************************************************************
 * Function
 *****************************************************************************/
static int _lcm_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	pr_debug("[LCM][I2C] NT: info==>name=%s addr=0x%x\n",
		client->name, client->addr);
	_lcm_i2c_client = client;
	return 0;
}


static int _lcm_i2c_remove(struct i2c_client *client)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	_lcm_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}


/*
 * module load/unload record keeping
 */
static int __init _lcm_i2c_init(void)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	i2c_add_driver(&_lcm_i2c_driver);
	pr_debug("[LCM][I2C] %s success\n", __func__);
	return 0;
}

static void __exit _lcm_i2c_exit(void)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	i2c_del_driver(&_lcm_i2c_driver);
}

module_init(_lcm_i2c_init);
module_exit(_lcm_i2c_exit);
/* i2c control end */

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};

static struct LCM_setting_table init_setting_vdo[] = {
	{0x00,1,{0x00}},
	{0xFF,3,{0x87,0x20,0x01}},
	{0x00,1,{0x80}},
	{0xFF,2,{0x87,0x20}},
	{0x00,1,{0x00}},
	{0x2A,4,{0x00,0x00,0x04,0x37}},
	{0x00,1,{0x00}},
	{0x2B,4,{0x00,0x00,0x09,0x5F}},
	{0x00,1,{0xA3}},
	{0xB3,4,{0x09,0x60,0x00,0x18}},
	{0x00,1,{0x80}},
	{0xC0,6,{0x00,0x60,0x00,0x85,0x00,0x11}},
	{0x00,1,{0x90}},
	{0xC0,6,{0x00,0x60,0x00,0x3D,0x00,0x11}},
	{0x00,1,{0xA0}},
	{0xC0,6,{0x00,0x69,0x00,0x3D,0x00,0x11}},
		{0x00,1,{0xB0}},
	{0xC0,5,{0x00,0xBA,0x00,0x3D,0x11}},
	{0x00,1,{0x60}},
	{0xC0,6,{0x00,0x9D,0x00,0x3D,0x00,0x11}},
	{0x00,1,{0x70}},
	{0xC0,12,{0x00,0xC0,0x00,0xC4,0x0D,0x02,0xB6,0x00,0x00,0x15,0x00,0xF3}},
	{0x00,1,{0xA3}},
	{0xC1,6,{0x00,0x50,0x00,0x2C,0x00,0x02}},
	{0x00,1,{0x80}},
	{0xCE,16,{0x01,0x81,0xFF,0xFF,0x00,0xB4,0x00,0xC4,0x00,0x00,0x00,0x00,0x01,0x18,0x01,0x28}},
	{0x00,1,{0x90}},
	{0xCE,15,{0x00,0xA7,0x10,0x43,0x00,0xA7,0x80,0xFF,0xFF,0x00,0x06,0x40,0x0F,0x0F,0x00}},
	{0x00,1,{0xA0}},
	{0xCE,3,{0x00,0x00,0x00}},
	{0x00,1,{0xB0}},
	{0xCE,3,{0x22,0x00,0x00}},
	{0x00,1,{0xD1}},
	{0xCE,7,{0x00,0x00,0x01,0x00,0x00,0x00,0x00}},
	{0x00,1,{0xE1}},
	{0xCE,11,{0x03,0x02,0xB6,0x02,0xB6,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,1,{0xF1}},
	{0xCE,9,{0x1F,0x1F,0x00,0x01,0x31,0x01,0x31,0x00,0x00}},
	{0x00,1,{0xB0}},
	{0xCF,4,{0x00,0x00,0x97,0x9B}},
	{0x00,1,{0xB5}},
	{0xCF,4,{0x04,0x04,0xF9,0xFD}},
	{0x00,1,{0xC0}},
	{0xCF,4,{0x09,0x09,0x5B,0x5F}},
	{0x00,1,{0xC5}},
	{0xCF,4,{0x09,0x09,0x61,0x65}},
	{0x00,1,{0xD1}},
	{0xC1,12,{0x0A,0xDF,0x0F,0x1A,0x19,0xCB,0x0A,0xDF,0x0F,0x1A,0x19,0xCB}},
	{0x00,1,{0xE1}},
	{0xC1,2,{0x0F,0x1A}},
	{0x00,1,{0xE4}},
	{0xCF,12,{0x0A,0x01,0x0A,0x00,0x0A,0x00,0x0A,0x00,0x0A,0x00,0x0A,0x00}},
	{0x00,1,{0x80}},
	{0xC1,2,{0x00,0x00}},
	{0x00,1,{0x90}},
	{0xC1,1,{0x03}},
	{0x00,1,{0xF5}},
	{0xCF,1,{0x02}},
	{0x00,1,{0xF6}},
	{0xCF,1,{0x3C}},
	{0x00,1,{0xF1}},
	{0xCF,1,{0x3C}},
	{0x00,1,{0xF0}},
	{0xC1,1,{0x00}},
	{0x00,1,{0xCC}},
	{0xC1,1,{0x18}},
	{0x00,1,{0xE0}},
	{0xC1,1,{0x00}},
	{0x00,1,{0x86}},
	{0xC0,6,{0x00,0x03,0x00,0x00,0x13,0x07}},
	{0x00,1,{0xB3}},
	{0xCE,6,{0x00,0x03,0x00,0x00,0x13,0x07}},
	{0x00,1,{0x96}},
	{0xC0,6,{0x00,0x03,0x00,0x00,0x13,0x07}},
	{0x00,1,{0x77}},
	{0xC0,2,{0x00,0x26}},
	{0x00,1,{0xD1}},
	{0xCE,7,{0x00,0x0A,0x01,0x01,0x00,0xEB,0x01}},
	{0x00,1,{0xE8}},
	{0xCE,2,{0x00,0x26}},
	{0x00,1,{0xE1}},
	{0xC1,2,{0x0E,0xDE}},
	{0x00,1,{0xA1}},
	{0xF3,1,{0x01}},
	{0x00,1,{0xA3}},
	{0xCE,6,{0x00,0x02,0x00,0x00,0x0A,0x07}},
	{0x00,1,{0xA6}},
	{0xC0,6,{0x00,0x00,0x00,0x01,0x0F,0x06}},
	{0x00,1,{0x66}},
	{0xC0,6,{0x00,0x00,0x00,0x01,0x1B,0x05}},
	{0x00,1,{0xB0}},
	{0xB3,1,{0x00}},
	{0x00,1,{0x83}},
	{0xB0,1,{0x63}},
	{0x00,1,{0x80}},
	{0xB3,1,{0x22}},
	{0x00,1,{0xF0}},
	{0xC1,1,{0x00}},
	{0x00,1,{0xF5}},
	{0xCF,1,{0x00}},
	{0x00,1,{0x80}},
	{0xC2,8,{0x85,0x02,0x48,0x0B,0x00,0x00,0x00,0x00}},
	{0x00,1,{0x90}},
	{0xC2,4,{0x00,0x00,0x00,0x00}},
	{0x00,1,{0xA0}},
	{0xC2,15,{0x81,0x04,0x00,0x02,0x91,0x80,0x01,0x00,0x02,0x91,0x82,0x02,0x00,0x02,0x91}},
	{0x00,1,{0xB0}},
	{0xC2,10,{0x83,0x03,0x00,0x02,0x91,0x00,0x00,0x00,0x00,0x00}},
	{0x00,1,{0xE0}},
	{0xC2,4,{0x33,0x33,0x00,0x00}},
	{0x00,1,{0x8C}},
	{0xC3,3,{0x01,0x00,0x00}},
	{0x00,1,{0xD0}},
	{0xC3,16,{0x35,0x0A,0x00,0x00,0x35,0x0A,0x00,0x00,0x35,0x0A,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,1,{0xE0}},
	{0xC3,16,{0x35,0x0A,0x00,0x00,0x35,0x0A,0x00,0x00,0x35,0x0A,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,1,{0x80}},
	{0xCB,16,{0x00,0x05,0x00,0x00,0x05,0x00,0x00,0x0E,0xCE,0x01,0xC5,0x00,0x00,0x00,0xC0,0x00}},
	{0x00,1,{0x90}},
	{0xCB,16,{0x00,0x00,0x00,0x00,0x30,0x00,0x00,0x30,0x00,0x0C,0x3C,0x00,0x30,0x00,0x00,0x00}},
	{0x00,1,{0xA0}},
	{0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x00}},
	{0x00,1,{0xB0}},
	{0xCB,4,{0x10,0x42,0x94,0x00}},
	{0x00,1,{0xC0}},
	{0xCB,4,{0x10,0x42,0x94,0x00}},
	{0x00,1,{0xD5}},
	{0xCB,11,{0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00}},
	{0x00,1,{0xE0}},
	{0xCB,13,{0x83,0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00,0x83,0x83,0x00,0x83}},
	{0x00,1,{0x80}},
	{0xCC,16,{0x23,0x23,0x23,0x16,0x17,0x18,0x23,0x23,0x23,0x16,0x17,0x18,0x1F,0x02,0x07,0x08}},
	{0x00,1,{0x90}},
	{0xCC,8,{0x06,0x09,0x1D,0x1E,0x23,0x23,0x21,0x20}},
	{0x00,1,{0x80}},
	{0xCD,16,{0x23,0x23,0x23,0x16,0x17,0x18,0x23,0x23,0x23,0x16,0x17,0x18,0x1F,0x02,0x07,0x08}},
	{0x00,1,{0x90}},
	{0xCD,8,{0x06,0x09,0x1D,0x1E,0x23,0x23,0x21,0x20}},
	{0x00,1,{0xA0}},
	{0xCC,16,{0x23,0x23,0x23,0x16,0x17,0x18,0x23,0x23,0x23,0x16,0x17,0x18,0x1F,0x02,0x06,0x09}},
	{0x00,1,{0xB0}},
	{0xCC,8,{0x07,0x08,0x1E,0x1D,0x23,0x23,0x21,0x20}},
	{0x00,1,{0xA0}},
	{0xCD,16,{0x23,0x23,0x23,0x16,0x17,0x18,0x23,0x23,0x23,0x16,0x17,0x18,0x1F,0x02,0x06,0x09}},
	{0x00,1,{0xB0}},
	{0xCD,8,{0x07,0x08,0x1E,0x1D,0x23,0x23,0x21,0x20}},
	{0x00,1,{0x93}},
	{0xC5,1,{0x37}},
	{0x00,1,{0x97}},
	{0xC5,1,{0x37}},
	{0x00,1,{0x9A}},
	{0xC5,1,{0x2D}},
	{0x00,1,{0x9C}},
	{0xC5,1,{0x2D}},
	{0x00,1,{0xB6}},
	{0xC5,8,{0x19,0x19,0x0A,0x0A,0x0F,0x0F,0x0A,0x0A}},
	{0x00,1,{0x88}},
	{0xC4,1,{0x08}},
	{0x00,1,{0xCA}},
	{0xC0,1,{0x90}},
	{0x00,1,{0xC0}},
	{0xC3,1,{0xC9}},
	{0x00,1,{0x88}},
	{0xC1,1,{0x8F}},

	{0x00,1,{0x9C}},
	{0xF5,1,{0x00}},
	{0x00,1,{0x9E}},
	{0xF5,1,{0x00}},
	{0x00,1,{0x86}},
	{0xF5,1,{0x4B}},
	{0x00,1,{0x96}},
	{0xF5,1,{0x0C}},
	{0x00,1,{0x00}},
	{0xD8,2,{0x23,0x23}},
	{0x00,1,{0x9B}},
	{0xC4,1,{0xFF}},
	{0x00,1,{0x94}},
	{0xE9,1,{0x00}},
	{0x00,1,{0x00}},
	{0xE1,16,{0x00,0x01,0x0C,0x13,0x32,0x1F,0x28,0x2F,0x3A,0x46,0x44,0x4C,0x52,0x58,0x49,0x5D}},
	{0x00,1,{0x10}},
	{0xE1,16,{0x66,0x6E,0x76,0xAE,0x7E,0x85,0x8D,0x96,0x2F,0xA1,0xA7,0xAE,0xB5,0x52,0xBE,0xC8}},
	{0x00,1,{0x20}},
	{0xE1,8,{0xD5,0xDD,0x9F,0xE7,0xF3,0xFB,0xFF,0x5B}},
	{0x00,1,{0x30}},
	{0xE1,16,{0x00,0x01,0x0C,0x13,0x32,0x1F,0x28,0x2F,0x3A,0x46,0x44,0x4C,0x52,0x58,0x49,0x5D}},
	{0x00,1,{0x40}},
	{0xE1,16,{0x66,0x6E,0x76,0xAE,0x7E,0x85,0x8D,0x96,0x2F,0xA1,0xA7,0xAE,0xB5,0x52,0xBE,0xC8}},
	{0x00,1,{0x50}},
	{0xE1,8,{0xD5,0xDD,0x9F,0xE7,0xF3,0xFB,0xFF,0x5B}},
	{0x00,1,{0x60}},
	{0xE1,16,{0x00,0x01,0x0C,0x13,0x32,0x1F,0x28,0x2F,0x3A,0x46,0x44,0x4C,0x52,0x58,0x49,0x5D}},
	{0x00,1,{0x70}},
	{0xE1,16,{0x66,0x6E,0x76,0xAE,0x7E,0x85,0x8D,0x96,0x2F,0xA1,0xA7,0xAE,0xB5,0x52,0xBE,0xC8}},
	{0x00,1,{0x80}},
	{0xE1,8,{0xD5,0xDD,0x9F,0xE7,0xF3,0xFB,0xFF,0x5B}},
	{0x00,1,{0x90}},
	{0xE1,16,{0x00,0x01,0x0C,0x13,0x32,0x1F,0x28,0x2F,0x3A,0x46,0x44,0x4C,0x52,0x58,0x49,0x5D}},
	{0x00,1,{0xA0}},
	{0xE1,16,{0x66,0x6E,0x76,0xAE,0x7E,0x85,0x8D,0x96,0x2F,0xA1,0xA7,0xAE,0xB5,0x52,0xBE,0xC8}},
	{0x00,1,{0xB0}},
	{0xE1,8,{0xD5,0xDD,0x9F,0xE7,0xF3,0xFB,0xFF,0x5B}},
	{0x00,1,{0xC0}},
	{0xE1,16,{0x00,0x01,0x0C,0x13,0x32,0x1F,0x28,0x2F,0x3A,0x46,0x44,0x4C,0x52,0x58,0x49,0x5D}},
	{0x00,1,{0xD0}},
	{0xE1,16,{0x66,0x6E,0x76,0xAE,0x7E,0x85,0x8D,0x96,0x2F,0xA1,0xA7,0xAE,0xB5,0x52,0xBE,0xC8}},
	{0x00,1,{0xE0}},
	{0xE1,8,{0xD5,0xDD,0x9F,0xE7,0xF3,0xFB,0xFF,0x5B}},
	{0x00,1,{0xF0}},
	{0xE1,16,{0x00,0x01,0x0C,0x13,0x32,0x1F,0x28,0x2F,0x3A,0x46,0x44,0x4C,0x52,0x58,0x49,0x5D}},
	{0x00,1,{0x00}},
	{0xE2,16,{0x66,0x6E,0x76,0xAE,0x7E,0x85,0x8D,0x96,0x2F,0xA1,0xA7,0xAE,0xB5,0x52,0xBE,0xC8}},
	{0x00,1,{0x10}},
	{0xE2,8,{0xD5,0xDD,0x9F,0xE7,0xF3,0xFB,0xFF,0x5B}},
	{0x00,1,{0x80}},
	{0xA7,1,{0x03}},
	{0x00,1,{0x82}},
	{0xA7,1,{0x22}},
	{0x00,1,{0x8D}},
	{0xA7,1,{0x02}},
	{0x00,1,{0x99}},
	{0xCF,1,{0x50}},
	{0x00,1,{0xB0}},
	{0xC5,6,{0xD0,0x4A,0x31,0xD0,0x4A,0x0C}},
	{0x00,1,{0x92}},
	{0xC5,1,{0x00}},
	{0x00,1,{0xA0}},
	{0xC5,1,{0x40}},
	{0x00,1,{0x98}},
	{0xC5,1,{0x27}},
	{0x00,1,{0x94}},
	{0xC5,1,{0x04}},
	{0x00,1,{0x80}},
	{0xA4,1,{0xC8}},
	{0x00,1,{0x8D}},
	{0xC5,1,{0x04}},
	{0x00,1,{0x8A}},
	{0xC5,1,{0x04}},
	{0x00,1,{0xF0}},
	{0xCF,2,{0x01,0x78}},
	{0x00,1,{0xA4}},
	{0xD7,1,{0x9F}},
	{0x00,1,{0xA1}},
	{0xC5,1,{0x40}},
	{0x00,1,{0x9D}},
	{0xC5,1,{0x44}},
	{0x00,1,{0xE0}},
	{0xCF,1,{0x34}},
	{0x00,1,{0xE8}},
	{0xC0,1,{0x40}},
	{0x00,1,{0x00}},
	{0xD9,1,{0x28}},
	{0x00,1,{0x06}},
	{0xD9,3,{0x28,0x28,0x28}},
	{0x00,1,{0x91}},
	{0xC4,1,{0x88}},
	{0x00,1,{0x80}},
	{0xC5,1,{0x88}},
	{0x00,1,{0xB0}},
	{0xB4,14,{0x00,0x08,0x00,0xAA,0x00,0x2B,0x00,0x07,0x0D,0xB7,0x0C,0xB7,0x1B,0xA0}},

	{0x11,0,{}},
	{REGFLAG_DELAY,120, {}},
	{0x29,0,{}},
	{0x35,1,{0x00}},
	{REGFLAG_DELAY,20, {}},

};


static struct LCM_setting_table
__maybe_unused lcm_deep_sleep_mode_in_setting[] = {
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 50, {} },
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 150, {} },
};

static struct LCM_setting_table __maybe_unused lcm_sleep_out_setting[] = {
	{0x11, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },
	{0x29, 1, {0x00} },
	{REGFLAG_DELAY, 50, {} },
};

static struct LCM_setting_table bl_level[] = {
	{0x51, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};


static void push_table(void *cmdq, struct LCM_setting_table *table,
		       unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;
		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count,
					 table[i].para_list, force_update);
			if (table[i].count > 1)
				MDELAY(1);
			break;
		}
	}
}

static void lcm_set_gpio_output(unsigned GPIO, unsigned int output)
{
	int ret;

	ret = gpio_request(GPIO, "GPIO");
	if (ret < 0) {
		pr_err("[%s]: GPIO requset fail!\n", __func__);
	}

	if (gpio_is_valid(GPIO)) {
		ret = gpio_direction_output(GPIO, output);
			if (ret < 0) {
				pr_err("[%s]: failed to set output", __func__);
			}
	}

	gpio_free(GPIO);
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
static void lcm_dfps_int(struct LCM_DSI_PARAMS *dsi)
{
	struct dfps_info *dfps_params = dsi->dfps_params;

	dsi->dfps_enable = 1;
	dsi->dfps_default_fps = 6000;/*real fps * 100, to support float*/
	dsi->dfps_def_vact_tim_fps = 9000;/*real vact timing fps * 100*/
	/* traversing array must less than DFPS_LEVELS */
	/* DPFS_LEVEL0 */
	dfps_params[0].level = DFPS_LEVEL0;
	dfps_params[0].fps = 6000;/*real fps * 100, to support float*/
	dfps_params[0].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/* if mipi clock solution */
	/* dfps_params[0].PLL_CLOCK = 574; */
	/* dfps_params[0].data_rate = xx; */
	dfps_params[0].vertical_frontporch = 1300;
	dfps_params[0].vertical_frontporch_for_low_power = 0;

	/* DPFS_LEVEL1 */
	dfps_params[1].level = DFPS_LEVEL1;
	dfps_params[1].fps = 9000;/*real fps * 100, to support float*/
	dfps_params[1].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/* if mipi clock solution */
	/* dfps_params[1].PLL_CLOCK = 380; */
	/* dfps_params[1].data_rate = xx; */
	dfps_params[1].vertical_frontporch = 54;
	dfps_params[1].vertical_frontporch_for_low_power = 0;

	dsi->dfps_num = 2;
}
#endif

static void lcm_get_params(struct LCM_PARAMS *params)
{
	// unsigned int i = 0;tting
	// unsigned int dynamic_fps_levels = 0;

	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH / 1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT / 1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density = LCM_DENSITY;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
	LCM_LOGI("%s: lcm_dsi_mode %d\n", __func__, lcm_dsi_mode);
	params->dsi.switch_mode_enable = 0;
	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 20;
	params->dsi.vertical_frontporch = 1300;
	//params->dsi.vertical_frontporch_for_low_power = 750;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 31;
	params->dsi.horizontal_frontporch = 165;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;
#ifdef CONFIG_MTK_MT6382_BDG
	params->dsi.bdg_ssc_disable = 1;
	params->dsi.dsc_params.ver = 17;
	params->dsi.dsc_params.slice_mode = 1;
	params->dsi.dsc_params.rgb_swap = 0;
	params->dsi.dsc_params.dsc_cfg = 34;
	params->dsi.dsc_params.rct_on = 1;
	params->dsi.dsc_params.bit_per_channel = 8;
	params->dsi.dsc_params.dsc_line_buf_depth = 9;
	params->dsi.dsc_params.bp_enable = 1;
	params->dsi.dsc_params.bit_per_pixel = 128;
	params->dsi.dsc_params.pic_height = 2400;
	params->dsi.dsc_params.pic_width = 1080;
	params->dsi.dsc_params.slice_height = 8;
	params->dsi.dsc_params.slice_width = 540;
	params->dsi.dsc_params.chunk_size = 540;
	params->dsi.dsc_params.xmit_delay = 170;
	params->dsi.dsc_params.dec_delay = 526;
	params->dsi.dsc_params.scale_value = 32;
	params->dsi.dsc_params.increment_interval = 43;
	params->dsi.dsc_params.decrement_interval = 7;
	params->dsi.dsc_params.line_bpg_offset = 12;
	params->dsi.dsc_params.nfl_bpg_offset = 3511;
	params->dsi.dsc_params.slice_bpg_offset = 3255;
	params->dsi.dsc_params.initial_offset = 6144;
	params->dsi.dsc_params.final_offset = 7072;
	params->dsi.dsc_params.flatness_minqp = 3;
	params->dsi.dsc_params.flatness_maxqp = 12;
	params->dsi.dsc_params.rc_model_size = 8192;
	params->dsi.dsc_params.rc_edge_factor = 6;
	params->dsi.dsc_params.rc_quant_incr_limit0 = 11;
	params->dsi.dsc_params.rc_quant_incr_limit1 = 11;
	params->dsi.dsc_params.rc_tgt_offset_hi = 3;
	params->dsi.dsc_params.rc_tgt_offset_lo = 3;
#endif
	params->dsi.dsc_enable = 0;
#ifndef CONFIG_FPGA_EARLY_PORTING
	/* this value must be in MTK suggested table */
#ifdef DSC_ENABLE
	params->dsi.bdg_dsc_enable = 1;
	params->dsi.PLL_CLOCK = 380; //with dsc
#else
	params->dsi.bdg_dsc_enable = 0;
	params->dsi.PLL_CLOCK = 550; //without dsc
#endif
	params->dsi.PLL_CK_CMD = 480;
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
	params->dsi.CLK_HS_POST = 36;
	params->dsi.clk_lp_per_line_enable = 0;
#ifdef CONFIG_MI_ERRFLAG_ESD_CHECK_ENABLE
	params->dsi.esd_check_enable = 1;
#endif

	//params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
	/*params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;*/

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 0;
	params->corner_pattern_height = ROUND_CORNER_H_TOP;
	params->corner_pattern_height_bot = ROUND_CORNER_H_BOT;
	params->corner_pattern_tp_size = sizeof(top_rc_pattern);
	params->corner_pattern_lt_addr = (void *)top_rc_pattern;
#endif

	#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/****DynFPS start****/
	lcm_dfps_int(&(params->dsi));
	/****DynFPS end****/
	#endif
}

/* turn on gate ic & control voltage to 5.5V */



static void lcm_init_power(void)
{
	LCM_LOGI("[nt36672D] %s enter\n", __func__);
	SET_RESET_PIN(0);
	MDELAY(3);
	lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, 1);
	MDELAY(3);

	lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, 1);
	MDELAY(15);

	LCM_LOGI("[nt36672D] %s exit\n", __func__);
}

static void lcm_suspend_power(void)
{
	

		lcm_set_gpio_output(GPIO_LCD_BIAS_ENN, 0);
		MDELAY(3);
		lcm_set_gpio_output(GPIO_LCD_BIAS_ENP, 0);
		MDELAY(5);
}


static void lcm_resume_power(void)
{

	LCM_LOGI("[DENNIS][%s][%d]\n", __func__, __LINE__);
	lcm_init_power();
}

static void lcm_init(void)
{

      	SET_RESET_PIN(1);
      	MDELAY(5);
      	SET_RESET_PIN(0);
      	MDELAY(2);
      	SET_RESET_PIN(1);
	MDELAY(20);

	push_table(NULL, init_setting_vdo, ARRAY_SIZE(init_setting_vdo), 1);
}

static void lcm_suspend(void)
{

	LCM_LOGI("[DENNIS][%s][%d]\n", __func__, __LINE__);
	push_table(NULL, lcm_suspend_setting,
		   ARRAY_SIZE(lcm_suspend_setting), 1);
}

static void lcm_resume(void)
{
	LCM_LOGI("[DENNIS][%s][%d]\n", __func__, __LINE__);
	lcm_init();
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int id[3] = {0x83, 0x11, 0x2B};
	unsigned int data_array[3];
	unsigned char read_buf[3];

	data_array[0] = 0x00033700; /* set max return size = 3 */
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x04, read_buf, 3); /* read lcm id */

	LCM_LOGI("ATA read = 0x%x, 0x%x, 0x%x\n",
		 read_buf[0], read_buf[1], read_buf[2]);

	if ((read_buf[0] == id[0]) &&
	    (read_buf[1] == id[1]) &&
	    (read_buf[2] == id[2]))
		ret = 1;
	else
		ret = 0;

	return ret;
#else
	return 0;
#endif
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	pr_err("%s,nt36672c backlight: level = %d\n", __func__, level);

	bl_level[0].para_list[0] = level;

	push_table(handle, bl_level, ARRAY_SIZE(bl_level), 1);
}

static void lcm_set_hw_info(void)
{
	hq_regiser_hw_info(HWID_LCM, "incell,vendor:TianMa,IC:nt36672C(novatek)");
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width,
	unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

#ifdef LCM_SET_DISPLAY_ON_DELAY
	lcm_set_display_on();
#endif

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[1];
	unsigned int array[16];

	array[0] = 0x00013700;  /* read id return 1byte */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xDA, buffer, 1);
	id = buffer[0];     /* we only need ID */

	LCM_LOGI("%s,nt36672c id = 0x%08x\n", __func__, id);

	if (id == LCM_ID_nt36672c)
		return 1;
	else
		return 0;

}

struct LCM_DRIVER dsi_panel_k19a_36_02_0c_dsc_vdo_lcm_drv = {
	.name = "dsi_panel_k19a_36_02_0c_dsc_vdo_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.compare_id = lcm_compare_id,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
	.set_hw_info = lcm_set_hw_info,
#ifdef CONFIG_MI_ERRFLAG_ESD_CHECK_ENABLE
	//.esd_recover = lcd_esd_recover,
#endif
};

