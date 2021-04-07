/*
 * Copyright (c) 2016  xiaomi Inc.
 */
#define pr_fmt(fmt)	"[Onewire] %s: " fmt, __func__

#include <linux/slab.h> /* kfree() */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/spinlock.h>

#define ow_info	pr_info
#define ow_dbg	pr_debug
#define ow_err	pr_debug
#define ow_log	pr_err

struct onewire_gpio_data {
	struct platform_device *pdev;
	struct device *dev;

	int ow_gpio;

	struct gpio_desc *ow_gpio_desc;
	struct gpio_chip *ow_gpio_chip;

	raw_spinlock_t lock;

	int version;

};

static struct class *onewire_class;
static int onewire_major;

static int onewire_gpio_detected;
static struct onewire_gpio_data *g_onewire_data;

#define ONE_WIRE_CONFIG_OUT		gpio_direction_output(g_onewire_data->ow_gpio, 1)// OUT
#define ONE_WIRE_CONFIG_IN		gpio_direction_input(g_onewire_data->ow_gpio)// IN
#define ONE_WIRE_OUT_HIGH		gpio_set_value(g_onewire_data->ow_gpio,1)// OUT: 1
#define ONE_WIRE_OUT_LOW		gpio_set_value(g_onewire_data->ow_gpio,0)// OUT: 0
#define ONE_WIRE_FORWARD_STAT	gpio_get_value(g_onewire_data->ow_gpio)//read gpio

void Delay_us(unsigned int T)
{
	udelay(T);
}
EXPORT_SYMBOL(Delay_us);

void Delay_ns(unsigned int T)
{
	ndelay(T);
}
EXPORT_SYMBOL(Delay_ns);

unsigned char ow_reset(void)
{
	unsigned char presence = 0xFF;
	unsigned long flags;

	raw_spin_lock_irqsave(&g_onewire_data->lock, flags);

	ONE_WIRE_CONFIG_OUT;
	ONE_WIRE_OUT_LOW;
	Delay_us(48);// 48
	ONE_WIRE_OUT_HIGH;
	ONE_WIRE_CONFIG_IN;
	Delay_us(10);
	presence = (unsigned char)ONE_WIRE_FORWARD_STAT; // Read
	ow_log("presence: 0x%x\n", presence);
	Delay_us(10);

	raw_spin_unlock_irqrestore(&g_onewire_data->lock, flags);
	return presence;
}
EXPORT_SYMBOL(ow_reset);

unsigned char read_bit(void)
{
	unsigned int vamm;

	ONE_WIRE_CONFIG_OUT;
	ONE_WIRE_OUT_LOW;
	Delay_us(1);
	ONE_WIRE_CONFIG_IN;
	vamm = ONE_WIRE_FORWARD_STAT; // Read
	Delay_us(15);

	return (unsigned char)vamm;
}

void write_bit(char bitval)
{
	ONE_WIRE_CONFIG_OUT;
	ONE_WIRE_OUT_LOW;
	Delay_us(2);
	if (bitval != 0)
		ONE_WIRE_OUT_HIGH;
	Delay_us(10);
	ONE_WIRE_OUT_HIGH;
	Delay_us(5);

}

unsigned char read_byte(void)
{
	unsigned char i;
	unsigned char value = 0;
	unsigned long flags;

	raw_spin_lock_irqsave(&g_onewire_data->lock, flags);
	for (i = 0; i < 8; i++) {
		if (read_bit())
			value |= 0x01 << i;// reads byte in, one byte at a time and then shifts it left
	}

	raw_spin_unlock_irqrestore(&g_onewire_data->lock, flags);
	return value;
}
EXPORT_SYMBOL(read_byte);

void write_byte(char val)
{
	unsigned char i;
	unsigned char temp;
	unsigned long flags;

	raw_spin_lock_irqsave(&g_onewire_data->lock, flags);
	ONE_WIRE_CONFIG_OUT;
	// writes byte, one bit at a time
	for (i = 0; i < 8; i++) {
		temp = val >> i ; // shifts val right 鈥榠鈥?spaces
		temp &= 0x01; // copy that bit to temp
		write_bit(temp); // write bit in temp into
	}
	raw_spin_unlock_irqrestore(&g_onewire_data->lock, flags);
}
EXPORT_SYMBOL(write_byte);

int onewire_gpio_get_status(void)
{
	return onewire_gpio_detected;
}
EXPORT_SYMBOL(onewire_gpio_get_status);

// parse dts
static int onewire_gpio_parse_dt(struct device *dev,
				struct onewire_gpio_data *pdata)
{
	int error, val;
	struct device_node *np = dev->of_node;
	// parse version
	pdata->version = 0;
	error = of_property_read_u32(np, "xiaomi,version", &val);
	if (error && (error != -EINVAL))
		ow_err("Unable to read version\n");
	else if (error != -EINVAL)
		pdata->version = val;
	// parse gpio
	pdata->ow_gpio = of_get_named_gpio(np,
					"xiaomi,ow_gpio", 0);
	if (pdata->ow_gpio < 0)
		ow_err("Unable to read onewire gpio\n");

	return 0;
}

// read data from file
static ssize_t onewire_gpio_ow_gpio_status_read(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int status;
//	struct onewire_gpio_data *onewire_data = dev_get_drvdata(dev);

	status = ONE_WIRE_FORWARD_STAT;

	return scnprintf(buf, PAGE_SIZE, "%d", status);
}

// write data to file
static ssize_t onewire_gpio_ow_gpio_store(struct device *dev,
struct device_attribute *attr,
const char *buf, size_t count)
{
	unsigned char result = 0x01;
	int buf_int;
	unsigned char i;
	unsigned char RomID[8] = {0};

	if (sscanf(buf, "%1u", &buf_int) != 1)
		return -EINVAL;

	if (buf_int == 0) {
		ONE_WIRE_OUT_LOW;
		ow_log("gpio : OUT 0");
	} else if (buf_int == 1) {
		ONE_WIRE_OUT_HIGH;
		ow_log("gpio : OUT 1");
	} else if (buf_int == 2) {
		ONE_WIRE_CONFIG_OUT;
		ow_log("gpio : OUT");
	} else if (buf_int == 3) {
		ONE_WIRE_CONFIG_IN;
		ow_log("gpio : IN");
	} else if (buf_int == 4) {
		result = ow_reset();
		if (result)
			ow_log("ow_reset: no device.result = %02x", result);
		else
			ow_log("ow_reset: device exist.result = %02x", result);
	} else if (buf_int == 5) {
		result = read_bit();
		ow_log("read_bit: %02x", result);
	} else if (buf_int == 6) {
		write_bit(0x01);
		ow_log("write_bit 0");
	} else if (buf_int == 7) {
		result = ow_reset();
		if (result)
			ow_log("ow_reset: no device.result = %02x", result);
		else
			ow_log("ow_reset: device exist.result = %02x", result);

		ow_dbg("Ready to write 0x33 to maxim IC!\n");
		write_byte(0x33);

		for (i = 0; i < 8; i++)
			RomID[i] = read_byte();

		ow_log("RomID = %02x%02x%02x%02x%02x%02x%02x%02x\n", RomID[0], RomID[1], RomID[2], RomID[3], RomID[4], RomID[5], RomID[6], RomID[7]);
	} else if (buf_int == 8) {
		ONE_WIRE_CONFIG_OUT;
		ONE_WIRE_OUT_HIGH;
		ONE_WIRE_OUT_LOW;
		ONE_WIRE_OUT_HIGH;
		ONE_WIRE_OUT_LOW;
		ONE_WIRE_OUT_HIGH;
		ONE_WIRE_OUT_LOW;
		ONE_WIRE_OUT_HIGH;
		ONE_WIRE_OUT_LOW;
		ONE_WIRE_OUT_HIGH;
		ONE_WIRE_OUT_LOW;

		Delay_us(1000);
		ONE_WIRE_OUT_HIGH;
		Delay_us(1);
		ONE_WIRE_OUT_LOW;
		Delay_us(1);
		ONE_WIRE_OUT_HIGH;
		Delay_us(1);
		ONE_WIRE_OUT_LOW;
		Delay_us(1);
		ONE_WIRE_OUT_HIGH;
		Delay_us(1);
		ONE_WIRE_OUT_LOW;
		Delay_us(1);
		ONE_WIRE_OUT_HIGH;
		Delay_us(1);
		ONE_WIRE_OUT_LOW;
		Delay_us(1);
		ONE_WIRE_OUT_HIGH;
		Delay_us(1);
		ONE_WIRE_OUT_LOW;
		Delay_us(1);

		Delay_us(1000);
		ONE_WIRE_OUT_HIGH;
		Delay_us(5);
		ONE_WIRE_OUT_LOW;
		Delay_us(5);
		ONE_WIRE_OUT_HIGH;
		Delay_us(5);
		ONE_WIRE_OUT_LOW;
		Delay_us(5);
		ONE_WIRE_OUT_HIGH;
		Delay_us(5);
		ONE_WIRE_OUT_LOW;
		Delay_us(5);
		ONE_WIRE_OUT_HIGH;
		Delay_us(5);
		ONE_WIRE_OUT_LOW;
		Delay_us(5);
		ONE_WIRE_OUT_HIGH;
		Delay_us(5);
		ONE_WIRE_OUT_LOW;
		Delay_us(5);
	}

	return count;
}

static DEVICE_ATTR(ow_gpio, S_IRUGO | S_IWUSR | S_IWGRP,
		onewire_gpio_ow_gpio_status_read,
		onewire_gpio_ow_gpio_store);


static int onewire_gpio_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct onewire_gpio_data *onewire_data;
	struct kobject *p;

	ow_log("onewire probe entry");

	if (!pdev->dev.of_node || !of_device_is_available(pdev->dev.of_node))
		return -ENODEV;
	if (pdev->dev.of_node) {
		onewire_data = devm_kzalloc(&pdev->dev,
			sizeof(struct onewire_gpio_data),
			GFP_KERNEL);
		if (!onewire_data) {
			ow_err("Failed to allocate memory\n");
			return -ENOMEM;
		}
		retval = onewire_gpio_parse_dt(&pdev->dev, onewire_data);
		if (retval) {
			retval = -EINVAL;
			goto onewire_parse_dt_err;
		}
	} else {
		onewire_data = pdev->dev.platform_data;
	}

	if (!onewire_data) {
		ow_err("No platform data found\n");
		return -EINVAL;
	}

	g_onewire_data = onewire_data;
	onewire_data->pdev = pdev;
	platform_set_drvdata(pdev, onewire_data);
	raw_spin_lock_init(&g_onewire_data->lock);
	// request onewire gpio
	if (gpio_is_valid(onewire_data->ow_gpio)) {
		retval = gpio_request(onewire_data->ow_gpio,
						"onewire gpio");
	} else {
		retval = -EINVAL;
	}
	if (retval) {
		ow_err("request onewire gpio failed, retval=%d\n",
				retval);
		goto onewire_ow_gpio_err;
	}
	// gpio output 1
	ONE_WIRE_CONFIG_OUT;
	// create device node
	onewire_data->dev = device_create(onewire_class,
		pdev->dev.parent->parent, onewire_major, onewire_data, "onewirectrl");
	if (IS_ERR(onewire_data->dev)) {
		ow_err("Failed to create interface device\n");
		goto onewire_interface_dev_create_err;
	}

	p = &onewire_data->dev->kobj;

	// create attr file
	retval = sysfs_create_file(p, &dev_attr_ow_gpio.attr);
	if (retval < 0) {
		ow_err("Failed to create sysfs attr file\n");
		goto onewire_sysfs_ow_gpio_err;
	}

	retval = sysfs_create_link(&onewire_data->dev->kobj, &pdev->dev.kobj,
								"pltdev");
	if (retval) {
		ow_err("Failed to create sysfs link\n");
		goto onewire_syfs_create_link_err;
	}
	return 0;
onewire_syfs_create_link_err:
	if (gpio_is_valid(onewire_data->ow_gpio))
		sysfs_remove_file(&pdev->dev.kobj, &dev_attr_ow_gpio.attr);
onewire_sysfs_ow_gpio_err:
	device_destroy(onewire_class, onewire_major);
onewire_interface_dev_create_err:
	if (gpio_is_valid(onewire_data->ow_gpio))
		gpio_free(onewire_data->ow_gpio);
onewire_ow_gpio_err:
onewire_parse_dt_err:
	kfree(onewire_data);
	return retval;
}

static int onewire_gpio_remove(struct platform_device *pdev)
{
	struct onewire_gpio_data *onewire_data = platform_get_drvdata(pdev);

	if (gpio_is_valid(onewire_data->ow_gpio)) {
		sysfs_remove_file(&pdev->dev.kobj, &dev_attr_ow_gpio.attr);
		gpio_free(onewire_data->ow_gpio);
	}
	kfree(onewire_data);

	return 0;
}

static long onewire_dev_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	ow_dbg("%d, cmd: 0x%x\n", __LINE__, cmd);
	return 0;
}
static int onewire_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int onewire_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations onewire_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= onewire_dev_open,
	.unlocked_ioctl = onewire_dev_ioctl,
	.release	= onewire_dev_release,
};

static const struct of_device_id onewire_gpio_dt_match[] = {
	{.compatible = "xiaomi,onewire_gpio"},
	{},
};

static struct platform_driver onewire_gpio_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "onewire_gpio",
		.of_match_table = onewire_gpio_dt_match,
	},
	.probe = onewire_gpio_probe,
	.remove = onewire_gpio_remove,
};

static int __init onewire_gpio_init(void)
{
	int retval;
	onewire_gpio_detected = false;

	ow_log("onewire gpio init entry.");

	onewire_class = class_create(THIS_MODULE, "onewire");
	if (IS_ERR(onewire_class)) {
		ow_err("coudn't create class");
		return PTR_ERR(onewire_class);
	}

	onewire_major = register_chrdev(0, "onewirectrl", &onewire_dev_fops);
	if (onewire_major < 0) {
		ow_err("failed to allocate char dev\n");
		retval = onewire_major;
		goto class_unreg;
	}

	return platform_driver_register(&onewire_gpio_driver);

class_unreg:
	class_destroy(onewire_class);
	return retval;
}

static void __exit onewire_gpio_exit(void)
{
	ow_log("onewire gpio exit entry.");
	platform_driver_unregister(&onewire_gpio_driver);

	unregister_chrdev(onewire_major, "onewirectrl");
	class_destroy(onewire_class);
}

module_init(onewire_gpio_init);
module_exit(onewire_gpio_exit);

MODULE_AUTHOR("xiaomi Inc.");
MODULE_DESCRIPTION("onewire driver");
MODULE_LICENSE("GPL");
