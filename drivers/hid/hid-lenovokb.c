/*
 *  HID driver for Lenovo Keyboard Docking devices
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>

#include "hid-ids.h"

#define lk_map_key_clear(c)	hid_map_usage_clear(hi, usage, bit, max, \
					EV_KEY, (c))

static int lk_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	if ((usage->hid & HID_USAGE_PAGE) != HID_UP_CONSUMER)
		return 0;

	switch (usage->hid & HID_USAGE) {
    case 0x07:  lk_map_key_clear(KEY_BACK); break;
    case 0x08:  lk_map_key_clear(KEY_HOME); break;
    case 0x40:  lk_map_key_clear(KEY_MENU); break;
    case 0x1a3: lk_map_key_clear(KEY_F15); break;
    case 0x09:  lk_map_key_clear(KEY_SEARCH); break;
    case 0x10:  lk_map_key_clear(KEY_F16); break;
    case 0x1a:  lk_map_key_clear(KEY_F17); break;
    case 0xb6:  lk_map_key_clear(KEY_PREVIOUSSONG); break;
    case 0xcd:  lk_map_key_clear(KEY_PLAYPAUSE); break;
    case 0xb5:  lk_map_key_clear(KEY_NEXTSONG); break;
    case 0xe2:  lk_map_key_clear(KEY_MUTE); break;
    case 0x206: lk_map_key_clear(KEY_VOLUMEDOWN); break;
    case 0x205: lk_map_key_clear(KEY_VOLUMEUP); break;
    case 0x1b:  lk_map_key_clear(KEY_BRIGHTNESSDOWN); break;
    case 0x1c:  lk_map_key_clear(KEY_BRIGHTNESSUP); break;
    case 0x1d:  lk_map_key_clear(KEY_POWER); break;
    case 0x1e:  lk_map_key_clear(KEY_KEYBOARD); break;
    case 0x37:  lk_map_key_clear(KEY_F19); break;
    case 0x38:  lk_map_key_clear(KEY_F20); break;
    case 0x1f:  lk_map_key_clear(KEY_F21); break;
	default:
		return 0;
	}
	return 1;
}

static const struct hid_device_id lk_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_LENOVO_KEYBOARD_DOCKING, USB_DEVICE_ID_LENOVO_KEYBOARD_DOCKING) },
	{ }
};
MODULE_DEVICE_TABLE(hid, lk_devices);

static struct hid_driver lk_driver = {
	.name = "Lenovo KeyboardDocking",
	.id_table = lk_devices,
	.input_mapping = lk_input_mapping,
};

static int __init lk_init(void)
{
	return hid_register_driver(&lk_driver);
}

static void __exit lk_exit(void)
{
	hid_unregister_driver(&lk_driver);
}

module_init(lk_init);
module_exit(lk_exit);
MODULE_LICENSE("GPL");
