/*
 * Backlight emulation LED trigger
 *
 * Copyright 2008 (C) Rodolfo Giometti <giometti@linux.it>
 * Copyright 2008 (C) Eurotech S.p.A. <info@eurotech.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/leds.h>
#include "../leds.h"

#if !defined(CONFIG_DRM_MEDIATEK)
#include <linux/fb.h>
#else
#include <drm/drm_notifier_mi.h>
#endif

#define BLANK		1
#define UNBLANK		0

struct bl_trig_notifier {
	struct led_classdev *led;
	int brightness;
	int old_status;
	struct notifier_block notifier;
	unsigned invert;
};

#if !defined(CONFIG_DRM_MEDIATEK)
static int fb_notifier_callback(struct notifier_block *p,
				unsigned long event, void *data)
{
	struct bl_trig_notifier *n = container_of(p,
					struct bl_trig_notifier, notifier);
	struct led_classdev *led = n->led;
	struct fb_event *fb_event = data;
	int *blank;
	int new_status;

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EVENT_BLANK)
		return 0;

	blank = fb_event->data;
	new_status = *blank ? BLANK : UNBLANK;

	if (new_status == n->old_status)
		return 0;

	if ((n->old_status == UNBLANK) ^ n->invert) {
		n->brightness = led->brightness;
		led_set_brightness_nosleep(led, LED_OFF);
	} else {
		led_set_brightness_nosleep(led, n->brightness);
	}

	n->old_status = new_status;

	return 0;
}
#else
static int drm_notifier_callback(struct notifier_block *p,
				unsigned long event, void *data)
{
	struct bl_trig_notifier *n = container_of(p,
					struct bl_trig_notifier, notifier);
	struct led_classdev *led = n->led;
	struct drm_notifier_data *drm_event = data;
	int *blank;
	int new_status;

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != DRM_EVENT_BLANK)
		return 0;

	if (drm_event->id != MSM_DRM_PRIMARY_DISPLAY)
		return 0;

	blank = drm_event->data;

	if (*blank == MSM_DRM_BLANK_UNBLANK)
		new_status = BLANK;
	else if (*blank == MSM_DRM_BLANK_POWERDOWN)
		new_status = UNBLANK;
	else
		return 0;

	if (new_status == n->old_status)
		return 0;

	if ((n->old_status == UNBLANK) ^ n->invert) {
		n->brightness = led->brightness;
		led_set_brightness_nosleep(led, LED_OFF);
	} else {
		led_set_brightness_nosleep(led, n->brightness);
	}

	n->old_status = new_status;

	return 0;
}
#endif

static ssize_t bl_trig_invert_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led = dev_get_drvdata(dev);
	struct bl_trig_notifier *n = led->trigger_data;

	return sprintf(buf, "%u\n", n->invert);
}

static ssize_t bl_trig_invert_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t num)
{
	struct led_classdev *led = dev_get_drvdata(dev);
	struct bl_trig_notifier *n = led->trigger_data;
	unsigned long invert;
	int ret;

	ret = kstrtoul(buf, 10, &invert);
	if (ret < 0)
		return ret;

	if (invert > 1)
		return -EINVAL;

	n->invert = invert;

	/* After inverting, we need to update the LED. */
	if ((n->old_status == BLANK) ^ n->invert)
		led_set_brightness_nosleep(led, LED_OFF);
	else
		led_set_brightness_nosleep(led, n->brightness);

	return num;
}
static DEVICE_ATTR(inverted, 0644, bl_trig_invert_show, bl_trig_invert_store);

static void bl_trig_activate(struct led_classdev *led)
{
	int ret;

	struct bl_trig_notifier *n;

	n = kzalloc(sizeof(struct bl_trig_notifier), GFP_KERNEL);
	led->trigger_data = n;
	if (!n) {
		dev_err(led->dev, "unable to allocate backlight trigger\n");
		return;
	}

	ret = device_create_file(led->dev, &dev_attr_inverted);
	if (ret)
		goto err_invert;

	n->led = led;
	n->brightness = led->brightness;
	n->old_status = UNBLANK;
#if !defined(CONFIG_DRM_MEDIATEK)
	n->notifier.notifier_call = fb_notifier_callback;
#else
	n->notifier.notifier_call = drm_notifier_callback;
#endif

#if !defined(CONFIG_DRM_MEDIATEK)
	ret = fb_register_client(&n->notifier);
#else
	ret = drm_register_client(&n->notifier);
#endif
	if (ret)
		dev_err(led->dev, "unable to register backlight trigger\n");
	led->activated = true;

	return;

err_invert:
	led->trigger_data = NULL;
	kfree(n);
}

static void bl_trig_deactivate(struct led_classdev *led)
{
	struct bl_trig_notifier *n =
		(struct bl_trig_notifier *) led->trigger_data;

	if (led->activated) {
		device_remove_file(led->dev, &dev_attr_inverted);
#if !defined(CONFIG_DRM_MEDIATEK)
		fb_unregister_client(&n->notifier);
#else
		drm_unregister_client(&n->notifier);
#endif
		kfree(n);
		led->activated = false;
	}
}

static struct led_trigger bl_led_trigger = {
	.name		= "backlight",
	.activate	= bl_trig_activate,
	.deactivate	= bl_trig_deactivate
};

static int __init bl_trig_init(void)
{
	return led_trigger_register(&bl_led_trigger);
}

static void __exit bl_trig_exit(void)
{
	led_trigger_unregister(&bl_led_trigger);
}

module_init(bl_trig_init);
module_exit(bl_trig_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("Backlight emulation LED trigger");
MODULE_LICENSE("GPL v2");
