/**
 * Copyright (c) 2008 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ltc3216.h>

#define LTC3216_MAX_RETRIES (3)

struct ltc3216_info {
	struct i2c_client *i2c_client;
	struct flash_ltc3216_platform_data *pdata;
};

struct ltc3216_info *info_flash = NULL;

static int ltc3216_write(struct i2c_client *client, u16 value)
{
	int count;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (value >> 8);
	data[1] = (u8) (value & 0xFF);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(data);
	msg[0].buf = data;

	do {
		count = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (count == ARRAY_SIZE(msg))
			return 0;
		retry++;
		pr_err("ltc3216: i2c transfer failed, retrying %x\n",
		       value);
		msleep(3);
	} while (retry <= LTC3216_MAX_RETRIES);
	return -EIO;
}


static long ltc3216_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct ltc3216_info *info_flash = file->private_data;

	switch (cmd) {
	case LTC3216_IOCTL_FLASH_ON:
		pr_err("ltc3216: flash kernel IOCTL flash on %lx\n",arg);
		ltc3216_write(info_flash->i2c_client,0x0200);
		ltc3216_write(info_flash->i2c_client, 0x90);
		ltc3216_write(info_flash->i2c_client, (u16) arg);
		break;

	case LTC3216_IOCTL_FLASH_OFF:
		pr_err("ltc3216: flash kernel IOCTL flash off %lx\n",arg);
		ltc3216_write(info_flash->i2c_client, 0x00);
		ltc3216_write(info_flash->i2c_client, 0x104);
		break;

	case LTC3216_IOCTL_TORCH_ON:
		pr_err("ltc3216: flash kernel IOCTL torch on %lx\n",arg);
		//set DIR=0,Tx-MASK=1 for 500ma current
		ltc3216_write(info_flash->i2c_client,0x0204);
		ltc3216_write(info_flash->i2c_client, (u16) arg);
		break;

	case LTC3216_IOCTL_TORCH_OFF:
		pr_err("ltc3216: flash kernel IOCTL torch off %lx\n",arg);
		ltc3216_write(info_flash->i2c_client, 0x00);
		ltc3216_write(info_flash->i2c_client, 0x104);
		ltc3216_write(info_flash->i2c_client,0x0200);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int ltc3216_open(struct inode *inode, struct file *file)
{
       pr_info("ltc3216_open ...\n");
	file->private_data = info_flash;
	if (info_flash->pdata && info_flash->pdata->power_on)
				info_flash->pdata->power_on();
	return 0;
}

int ltc3216_release(struct inode *inode, struct file *file)
{
       pr_info("ltc3216_release ...\n");
	 //shutdown flash
        ltc3216_write(info_flash->i2c_client, 0x00);
        ltc3216_write(info_flash->i2c_client, 0x104);
	 ltc3216_write(info_flash->i2c_client,0x0200);
	 if (info_flash->pdata && info_flash->pdata->power_off)
				info_flash->pdata->power_off();
	file->private_data = NULL;
	return 0;
}


static const struct file_operations ltc3216_fileops = {
	.owner = THIS_MODULE,
	.open = ltc3216_open,
	.unlocked_ioctl = ltc3216_ioctl,
	.release = ltc3216_release,
};

static struct miscdevice ltc3216_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltc3216",
	.fops = &ltc3216_fileops,
};

/* COMPAL - START - [2011/3/24  16:7] - Jamin - [flash]: add system node for debug   */
static int myatoi(const char *a)
{
    int s = 0;
    while(*a >= '0' && *a <= '9')
        s = (s << 3) + (s << 1) + *a++ - '0';
    return s;
}
static ssize_t Torch_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    return (sizeof(buf));
}
static ssize_t Torch_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int dim;
   unsigned int val=0x0050;
   dim = myatoi(buf);

	 if(dim >=0 && dim <=7){
		val=val&0xfff8;
		val=val|dim;
		//power enable
		if (info_flash->pdata && info_flash->pdata->power_on)
				info_flash->pdata->power_on();
		//set flash enter torch mode
   		ltc3216_write(info_flash->i2c_client, val);
		//set DIR=0,Tx-MASK=1 for 500ma current
		ltc3216_write(info_flash->i2c_client,0x0204);
		printk("[Flash] enter Torch mode \n");
       	msleep(3000);
		if(info_flash->pdata&&info_flash->pdata->power_off)
			info_flash->pdata->power_off();
		//shutdown flash led
		ltc3216_write(info_flash->i2c_client,0x0010);
		ltc3216_write(info_flash->i2c_client,0x0200);
		printk("[Flash] shutdown \n");
	}
	else {
		 printk("[Flash]: not support input =%d  \n",dim);
	}
    return n;
}
static ssize_t Flash_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    return (sizeof(buf));
}
static ssize_t Flash_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
   int dim;
   unsigned int val=0x018c;
   dim = myatoi(buf);

    if(dim >=1 && dim <=8){
                //set power current
                dim=dim-1;
                val=val&0xfff8;
                val=val|(dim<<0);
                printk("[Flah]: enter flash mode ... Got input =%x ,val=%x \n",dim,val);
                //set flash enter flash mode and tourch current 0mA
                ltc3216_write(info_flash->i2c_client, 0x0090);
       	   msleep(5);
                //do flash 
                ltc3216_write(info_flash->i2c_client, val);
        }
        else if(dim==0){
                printk("[Flah]: shutdown \n");
                ltc3216_write(info_flash->i2c_client, 0x0010);
                ltc3216_write(info_flash->i2c_client, 0x0104);
        }
        else {
                 printk("[Flash]: not support =%d  \n",dim);
        }
    return n;
}
static struct kobject *flashdebug_kobj;

#define debug_attr(_name) \
static struct kobj_attribute _name##_attr = { \
    .attr = { \
        .name = __stringify(_name), \
        .mode = 0660, \
    }, \
    .show = _name##_show, \
    .store = _name##_store, \
}

debug_attr(Torch_mode);
debug_attr(Flash_mode);

static struct attribute * g[] = {
    &Torch_mode_attr.attr,
    &Flash_mode_attr.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = g,
};
/* COMPAL - END */

static int ltc3216_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("ltc3216: probing sensor.\n");

	info_flash = kzalloc(sizeof(struct ltc3216_info), GFP_KERNEL);
	if (!info_flash) {
		pr_err("ltc3216: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&ltc3216_device);
	if (err) {
		pr_err("ltc3216: Unable to register misc device!\n");
		kfree(info_flash);
		return err;
	}
      info_flash->pdata = client->dev.platform_data;
	info_flash->i2c_client = client;

	/* COMPAL - START - [2011/3/24  16:11] - Jamin - [flash]: add system node for debug   */
        flashdebug_kobj = kobject_create_and_add("Flash", NULL);
        if (flashdebug_kobj == NULL) {
         printk("%s: subsystem_register failed\n", __FUNCTION__);
        }
        err = sysfs_create_group(flashdebug_kobj, &attr_group);
        if(err) {
                printk("%s: sysfs_create_group failed, %d\n", __FUNCTION__, __LINE__);
        }
	/* COMPAL - END */


	return 0;
}

static int ltc3216_remove(struct i2c_client *client)
{
	struct ltc3216_info *info_flash;
	info_flash = i2c_get_clientdata(client);
	misc_deregister(&ltc3216_device);
	kfree(info_flash);
	sysfs_remove_group(flashdebug_kobj,&attr_group);
	return 0;
}

static const struct i2c_device_id ltc3216_id[] = {
	{ "ltc3216", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ltc3216_id);

static struct i2c_driver ltc3216_i2c_driver = {
	.driver = {
		.name = "ltc3216",
		.owner = THIS_MODULE,
	},
	.probe = ltc3216_probe,
	.remove = ltc3216_remove,
	.id_table = ltc3216_id,
};

static int __init ltc3216_init(void)
{
	pr_info("ltc3216 sensor driver loading\n");
	return i2c_add_driver(&ltc3216_i2c_driver);
}

static void __exit ltc3216_exit(void)
{
	i2c_del_driver(&ltc3216_i2c_driver);
}

module_init(ltc3216_init);
module_exit(ltc3216_exit);

