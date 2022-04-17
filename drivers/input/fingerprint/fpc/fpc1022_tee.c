/*
 * FPC1022 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <teei_fp.h>
#include <linux/fb.h>
#include <linux/atomic.h>
#ifndef CONFIG_SPI_MT65XX
#include "mtk_spi.h"
#include "mtk_spi_hal.h"
#endif

#ifndef CONFIG_SPI_MT65XX
#include "mtk_gpio.h"
#include "mach/gpio_const.h"
#endif

#ifdef CONFIG_HQ_SYSFS_SUPPORT
#include <linux/hqsysfs.h>
#endif
#include  <linux/regulator/consumer.h>

#define FPC1022_NAME "fpc1022_irq"

#define FPC1022_RESET_LOW_US 5000
#define FPC1022_RESET_HIGH1_US 100
#define FPC1022_RESET_HIGH2_US 5000

#define FPC_IRQ_DEV_NAME         "fpc_irq"
#define FPC_TTW_HOLD_TIME 2000
#define FP_UNLOCK_REJECTION_TIMEOUT (FPC_TTW_HOLD_TIME - 500) /*ms*/

#define     FPC102X_REG_HWID      252
#define FPC1022_CHIP 0x1800
#define FPC1022_CHIP_MASK_SENSOR_TYPE 0xff00

#define GPIO_GET(pin) __gpio_get_value(pin)	//get input pin value

#define GPIOIRQ 2		//XPT

struct regulator *regu_buck;

#ifdef CONFIG_SPI_MT65XX
extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);
#endif

#include "teei_fp.h"
#include "tee_client_api.h"
struct TEEC_UUID uuid_ta_fpc = { 0x7778c03f, 0xc30c, 0x4dd0,
	{0xa3, 0x19, 0xea, 0x29, 0x64, 0x3d, 0x4d, 0x4b}
};

struct fpc1022_data {
	struct device *dev;
	struct platform_device *pldev;
	struct spi_device *spi;
	int irq_gpio;
	int irq_num;
	struct pinctrl *pinctrl;
	struct pinctrl_state *st_irq;	//xpt
	struct pinctrl_state *st_rst_l;
	struct pinctrl_state *st_rst_h;

	struct input_dev *idev;
	char idev_name[32];
	int event_type;
	int event_code;
	atomic_t wakeup_enabled;	/* Used both in ISR and non-ISR */
	struct mutex lock;
	bool prepared;
	struct notifier_block fb_notifier;
	bool fb_black;
	bool wait_finger_down;
	struct wakeup_source *ttw_wl;
	struct work_struct work;
	struct input_handler input_handler;
};

static int input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id) {
	int rc;
	struct input_handle *handle;
	struct fpc1022_data *fpc1022 =
		container_of(handler, struct fpc1022_data, input_handler);

	if (!strstr(dev->name, "uinput-fpc"))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = FPC1022_NAME;
	handle->private = fpc1022;

	rc = input_register_handle(handle);
	if (rc)
		goto err_input_register_handle;

	rc = input_open_device(handle);
	if (rc)
		goto err_input_open_device;

	return 0;

err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return rc;
}

static bool input_filter(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	return true;
}

static void input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },

};
int fp_idx_ic_exist;

extern bool goodix_fp_exist;
extern struct spi_device *spi_fingerprint;
bool fpc1022_fp_exist = false;

static struct fpc1022_data *fpc1022;

static int check_hwid(struct spi_device *spi);

#ifndef CONFIG_SPI_MT65XX
static const struct mt_chip_conf spi_mcc = {
	.setuptime = 20,
	.holdtime = 20,
	.high_time = 50,	/* 1MHz */
	.low_time = 50,
	.cs_idletime = 5,
	.ulthgh_thrsh = 0,

	.cpol = SPI_CPOL_0,
	.cpha = SPI_CPHA_0,

	.rx_mlsb = SPI_MSB,
	.tx_mlsb = SPI_MSB,

	.tx_endian = SPI_LENDIAN,
	.rx_endian = SPI_LENDIAN,

	.com_mod = FIFO_TRANSFER,

	.pause = 1,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif

#ifdef CONFIG_SPI_MT65XX
u32 spi_speed = 1 * 1000000;
#endif

static void fpc1022_get_irqNum(struct fpc1022_data *fpc1022)
{

	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,fpc1022_irq");

	if (node) {
		fpc1022->irq_num = irq_of_parse_and_map(node, 0);	//xpt
		fpc1022->irq_gpio = of_get_named_gpio(node, "fpc,gpio_irq", 0);
	}
}

static int hw_reset(struct fpc1022_data *fpc1022)
{
	struct device *dev = fpc1022->dev;

	pinctrl_select_state(fpc1022->pinctrl, fpc1022->st_rst_h);
	usleep_range(FPC1022_RESET_HIGH1_US, FPC1022_RESET_HIGH1_US + 100);

	pinctrl_select_state(fpc1022->pinctrl, fpc1022->st_rst_l);
	usleep_range(FPC1022_RESET_LOW_US, FPC1022_RESET_LOW_US + 100);

	pinctrl_select_state(fpc1022->pinctrl, fpc1022->st_rst_h);
	usleep_range(FPC1022_RESET_HIGH1_US, FPC1022_RESET_HIGH1_US + 100);

	return 0;
}

static ssize_t hw_reset_set(struct device *dev,
			    struct device_attribute *attr, const char *buf,
			    size_t count)
{
	int ret;
	struct fpc1022_data *fpc1022 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset")))
		ret = hw_reset(fpc1022);
	else
		return -EINVAL;

	return ret ? ret : count;
}

static DEVICE_ATTR(hw_reset, 0200, NULL, hw_reset_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct fpc1022_data *fpc1022 = dev_get_drvdata(dev);
	return count;
}

static DEVICE_ATTR(wakeup_enable, 0200, NULL, wakeup_enable_set);

/*
 * sysfs node for sending event to make the system interactive,
 * i.e. waking up
 */
static ssize_t do_wakeup_set(struct device *dev,
			     struct device_attribute *attr, const char *buf,
			     size_t count)
{
	struct fpc1022_data *fpc1022 = dev_get_drvdata(dev);

	if (count > 0) {
		/*
		 * Sending power key event creates a toggling
		 * effect that may be desired. It could be
		 * replaced by another event such as KEY_WAKEUP.
		 */
		input_report_key(fpc1022->idev, KEY_POWER, 1);
		input_report_key(fpc1022->idev, KEY_POWER, 0);
		input_sync(fpc1022->idev);
	} else
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(do_wakeup, 0200, NULL, do_wakeup_set);

static ssize_t clk_enable_set(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct fpc1022_data *fpc1022 = dev_get_drvdata(dev);

	if (!(fpc1022->spi)) {
		return 0;
	}
	if (*buf == 49) {
		mt_spi_enable_master_clk(fpc1022->spi);
		return 1;
	}
	if (*buf == 48) {
		mt_spi_disable_master_clk(fpc1022->spi);
		return 1;
	} else
		return 0;

}

static DEVICE_ATTR(clk_enable, 0200, NULL, clk_enable_set);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *device,
		       struct device_attribute *attribute, char *buffer)
{
	struct fpc1022_data *fpc1022 = dev_get_drvdata(device);
	int irq = __gpio_get_value(fpc1022->irq_gpio);

	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}

/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *device,
		       struct device_attribute *attribute,
		       const char *buffer, size_t count)
{
	struct fpc1022_data *fpc1022 = dev_get_drvdata(device);

	dev_dbg(fpc1022->dev, "%s\n", __func__);
	return count;
}

static DEVICE_ATTR(irq, 0600, irq_get, irq_ack);


static ssize_t fingerdown_wait_set(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct fpc1022_data *fpc1022 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable")))
		fpc1022->wait_finger_down = true;
	else if (!strncmp(buf, "disable", strlen("disable")))
		fpc1022->wait_finger_down = false;
	else
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(fingerdown_wait, 0200, NULL, fingerdown_wait_set);

static ssize_t fpc_ic_is_exist(struct device *device,
			       struct device_attribute *attribute, char *buffer)
{
	int fpc_exist = 0;

	if (fpc1022_fp_exist)
		fpc_exist = 1;
	else
		fpc_exist = 0;

	return scnprintf(buffer, PAGE_SIZE, "%i\n", fpc_exist);
}

static DEVICE_ATTR(fpid_get, 0600, fpc_ic_is_exist, NULL);

static struct attribute *attributes[] = {
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_do_wakeup.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	&dev_attr_fpid_get.attr,
	&dev_attr_fingerdown_wait.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1022_irq_handler(int irq, void *handle)
{
	struct fpc1022_data *fpc1022 = handle;

	/*
	 * Make sure 'wakeup_enabled' is updated before using it
	 * since this is interrupt context (other thread...)
	 */

	mutex_lock(&fpc1022->lock);

	if (atomic_read(&fpc1022->wakeup_enabled))
		__pm_wakeup_event(fpc1022->ttw_wl, msecs_to_jiffies(FPC_TTW_HOLD_TIME));

	mutex_unlock(&fpc1022->lock);

	sysfs_notify(&fpc1022->dev->kobj, NULL, dev_attr_irq.attr.name);

	if (fpc1022->wait_finger_down && fpc1022->fb_black) {
		fpc1022->wait_finger_down = false;
	}

	return IRQ_HANDLED;

}

static int fpc1022_platform_probe(struct platform_device *pldev)
{
	int ret = 0;
	int irqf;
	u32 val;
	const char *idev_name;
	struct device *dev = &pldev->dev;
	struct device_node *np = dev->of_node;

	if (!np) {
		ret = -EINVAL;
		goto err_no_of_node;
	}

	fpc1022 = devm_kzalloc(dev, sizeof(*fpc1022), GFP_KERNEL);
	if (!fpc1022) {
		ret = -ENOMEM;
		goto err_fpc1022_malloc;
	}

	//workaround to solve two spi device
	if (spi_fingerprint == NULL) {
		pr_debug("%s Line:%d spi device is NULL,cannot spi transfer\n",
			  __func__, __LINE__);
	} else {
		ret = check_hwid(spi_fingerprint);
		if (ret < 0) {
			return -EINVAL;
		}
		fpc1022_fp_exist = true;

		fpc1022->spi = spi_fingerprint;
		/********xinan_bp for dual_TA begain *********/
		memcpy(&uuid_fp, &uuid_ta_fpc, sizeof(struct TEEC_UUID));
		/********xinan_bp for dual_TA end *********/
	}

	fpc1022->dev = dev;
	dev_set_drvdata(dev, fpc1022);
	fpc1022->pldev = pldev;

	fpc1022->pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR(fpc1022->pinctrl)) {
		ret = PTR_ERR(fpc1022->pinctrl);
		goto err_pinctrl_get;
	}

	fpc1022->st_irq = pinctrl_lookup_state(fpc1022->pinctrl, "default");
	if (IS_ERR(fpc1022->st_irq))
		ret = PTR_ERR(fpc1022->st_irq);

	fpc1022->st_rst_h =
	    pinctrl_lookup_state(fpc1022->pinctrl, "reset_high");
	if (IS_ERR(fpc1022->st_rst_h)) {
		ret = PTR_ERR(fpc1022->st_rst_h);
		goto err_lookup_state;
	}

	fpc1022->st_rst_l = pinctrl_lookup_state(fpc1022->pinctrl, "reset_low");
	if (IS_ERR(fpc1022->st_rst_l)) {
		ret = PTR_ERR(fpc1022->st_rst_l);
		goto err_lookup_state;
	}
	fpc1022_get_irqNum(fpc1022);

	ret = of_property_read_u32(np, "fpc,event-type", &val);
	fpc1022->event_type = ret < 0 ? EV_MSC : val;

	ret = of_property_read_u32(np, "fpc,event-code", &val);
	fpc1022->event_code = ret < 0 ? MSC_SCAN : val;

	fpc1022->idev = devm_input_allocate_device(dev);

	if (!fpc1022->idev) {
		ret = -ENOMEM;
		goto err_input_malloc;
	}

	input_set_capability(fpc1022->idev, fpc1022->event_type,
			     fpc1022->event_code);

	if (!of_property_read_string(np, "input-device-name", &idev_name)) {
		fpc1022->idev->name = idev_name;
	} else {
		snprintf(fpc1022->idev_name, sizeof(fpc1022->idev_name),
			 "fpc_irq@%s", dev_name(dev));
		fpc1022->idev->name = fpc1022->idev_name;
	}

	/* Also register the key for wake up */
	set_bit(EV_KEY, fpc1022->idev->evbit);
	set_bit(EV_PWR, fpc1022->idev->evbit);
	set_bit(KEY_WAKEUP, fpc1022->idev->keybit);
	set_bit(KEY_POWER, fpc1022->idev->keybit);
	ret = input_register_device(fpc1022->idev);
	atomic_set(&fpc1022->wakeup_enabled, 1);

	if (ret)
		goto err_register_input;

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}

	mutex_init(&fpc1022->lock);

	ret = devm_request_threaded_irq(dev, fpc1022->irq_num,
					NULL, fpc1022_irq_handler, irqf,
					dev_name(dev), fpc1022);

	if (ret)
		goto err_request_irq;

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(fpc1022->irq_num);
	fpc1022->ttw_wl = wakeup_source_register(dev, "fpc_ttw_wl");

	fpc1022->input_handler.filter = input_filter;
	fpc1022->input_handler.connect = input_connect;
	fpc1022->input_handler.disconnect = input_disconnect;
	fpc1022->input_handler.name = FPC1022_NAME;
	fpc1022->input_handler.id_table = ids;
	ret = input_register_handler(&fpc1022->input_handler);
	if (ret) {
		dev_err(dev, "failed to register key handler\n");
		return ret;
	}

	ret = sysfs_create_group(&dev->kobj, &attribute_group);
	if (ret)
		goto err_create_sysfs;

	fpc1022->fb_black = false;
	fpc1022->wait_finger_down = false;

	hw_reset(fpc1022);

	return ret;

err_create_sysfs:
	wakeup_source_unregister(fpc1022->ttw_wl);

err_request_irq:
	mutex_destroy(&fpc1022->lock);

err_register_input:
	input_unregister_device(fpc1022->idev);

err_input_malloc:
err_lookup_state:
err_pinctrl_get:
	devm_kfree(dev, fpc1022);

err_fpc1022_malloc:
err_no_of_node:

	return ret;
}

static int fpc1022_platform_remove(struct platform_device *pldev)
{
	struct device *dev = &pldev->dev;
	struct fpc1022_data *fpc1022 = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	
	sysfs_remove_group(&dev->kobj, &attribute_group);

	mutex_destroy(&fpc1022->lock);
	wakeup_source_unregister(fpc1022->ttw_wl);

	input_unregister_device(fpc1022->idev);
	devm_kfree(dev, fpc1022);

	return 0;
}

static struct of_device_id fpc1022_of_match[] = {
	{.compatible = "mediatek,fpc1022_irq",},
	{}
};

MODULE_DEVICE_TABLE(of, fpc1022_of_match);

static struct platform_driver fpc1022_driver = {
	.driver = {
		   .name = "FPC1022_NAME",
		   .owner = THIS_MODULE,
		   .of_match_table = fpc1022_of_match,
		   },
	.probe = fpc1022_platform_probe,
	.remove = fpc1022_platform_remove
};

static int spi_read_hwid(struct spi_device *spi, u8 *rx_buf)
{
	int error;
	struct spi_message msg;
	struct spi_transfer *xfer;
	u8 tmp_buf[10] = { 0 };
	u8 addr = FPC102X_REG_HWID;

	xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
	if (xfer == NULL) {
		return -ENOMEM;
	}

	spi_message_init(&msg);

	tmp_buf[0] = addr;
	xfer[0].tx_buf = tmp_buf;
	xfer[0].len = 1;
	xfer[0].delay_usecs = 5;

#ifdef CONFIG_SPI_MT65XX
	xfer[0].speed_hz = spi_speed;
#endif

	spi_message_add_tail(&xfer[0], &msg);

	xfer[1].tx_buf = tmp_buf + 2;
	xfer[1].rx_buf = tmp_buf + 4;
	xfer[1].len = 2;
	xfer[1].delay_usecs = 5;

#ifdef CONFIG_SPI_MT65XX
	xfer[1].speed_hz = spi_speed;
#endif

	spi_message_add_tail(&xfer[1], &msg);
	error = spi_sync(spi, &msg);

	memcpy(rx_buf, (tmp_buf + 4), 2);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

static int check_hwid(struct spi_device *spi)
{
	int error = 0;
	u32 time_out = 0;
	u8 tmp_buf[2] = { 0 };
	u16 hardware_id;

	do {
		spi_read_hwid(spi, tmp_buf);
		time_out++;

		hardware_id = ((tmp_buf[0] << 8) | (tmp_buf[1]));

		if ((FPC1022_CHIP_MASK_SENSOR_TYPE & hardware_id) ==
		    FPC1022_CHIP)
			error = 0;
		else
			error = -1;

		if (!error)
			return 0;

	} while (time_out < 2);

	return -1;
}

MODULE_DEVICE_TABLE(of, fpc1022_spi_of_match);

static int __init fpc1022_init(void)
{
	if (goodix_fp_exist)
		return -EINVAL;

	if (platform_driver_register(&fpc1022_driver) != 0)
		return -EINVAL;

	return 0;
}

static void __exit fpc1022_exit(void)
{
	platform_driver_unregister(&fpc1022_driver);
}

late_initcall(fpc1022_init);
module_exit(fpc1022_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("fpc1022 Fingerprint sensor device driver.");
