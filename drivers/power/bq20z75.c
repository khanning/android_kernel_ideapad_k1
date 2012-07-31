/*
 * drivers/power/bq20z75.c
 *
 * Gas Gauge driver for TI's BQ20Z75
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#define DEBUG
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>   //peter
#include <linux/gpio.h>   //peter
#include <linux/timer.h>   //peter
#include <linux/interrupt.h>  //peter
//+++[ASD2_ES1|Jaco_Gao|2011.03.08] for EC Tool
#include "compal_ec.h"
#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
//---[ASD2_ES1|Jaco_Gao|2011.03.08] for EC Tool
enum {
	REG_MANUFACTURER_DATA,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CYCLE_COUNT,
	REG_CAPACITY,
	REG_SERIAL_NUMBER,
	REG_MAX
};

#define BATTERY_MANUFACTURER_SIZE	12
#define BATTERY_NAME_SIZE		8

/* manufacturer access defines */
#define MANUFACTURER_ACCESS_STATUS	0x0006
#define MANUFACTURER_ACCESS_SLEEP	0x0011

/* battery status value bits */
#define BATTERY_CHARGING 		0x40
#define BATTERY_FULL_CHARGED		0x20
#define BATTERY_FULL_DISCHARGED 	0x10

#define NVBATTERY_POLLING_INTERVAL 30000 /* 30 Seconds */   //peter

#define BQ20Z75_DATA(_psp, _addr, _min_value, _max_value) { \
	.psp = _psp, \
	.addr = _addr, \
	.min_value = _min_value, \
	.max_value = _max_value, \
}

static struct bq20z75_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} 
bq20z75_data[];

static enum power_supply_property bq20z75_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

//peter
static enum power_supply_property bq20z75_charge_properties[] = {
        POWER_SUPPLY_PROP_ONLINE,
};

static int bq20z75_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);


//peter
static int bq20z75_get_charge_property(struct power_supply *psy,
        enum power_supply_property psp, union power_supply_propval *val);

static s32 i2c_smbus_read_word_data_retry(struct i2c_client *client, u8 command);
static s32 i2c_smbus_write_word_data_retry(struct i2c_client *client, u8 command, u16 value);

//peter
static char *supply_list[] = {
        "battery",
};
//peter
static struct power_supply bq20z75_supply[] = {
{
        .name = "battery",
        .type = POWER_SUPPLY_TYPE_BATTERY,
        .properties = bq20z75_properties,
        .num_properties = ARRAY_SIZE(bq20z75_properties),
        .get_property = bq20z75_get_property,
},
{
        .name = "ac",
        .type = POWER_SUPPLY_TYPE_MAINS,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
        .properties = bq20z75_charge_properties,
        .num_properties = ARRAY_SIZE(bq20z75_charge_properties),
        .get_property = bq20z75_get_charge_property,
},

};
struct bq20z75_device_info {
    struct i2c_client	*client;
    struct compalec_dev *ec;
} *bq20z75_device;

struct timer_list poll_timer;    //peter
unsigned int batt_status_poll_period;    //peter
int gpio;       //peter
s16 BatCtlEnable_val;
s16 MicSwitch_val;
s16 Coldboot_val;
s16 Resume_val;
bool ECflashMode = 0;    //peter for EC flash

//[ASD2-ES1 - Darren - 2011.3.16] Add EC new function
//+++[ASD2-ES1 - Darren]
s16 BrightnessLevel_val;
//---[ASD2-ES1 - Darren]

//peter
typedef enum
{
        NvCharger_Type_Battery = 0,
        NvCharger_Type_AC,
} NvCharger_Type;

//+++[ASD2_ES1|Jaco_Gao|2011.03.08] for EC Tool
//#define DEVICE_NAME 	 		        "ec_i2c"		/* Dev name as it appears in /proc/devices */
#define DEVICE_NAME                     "hybrid_events"
#define MAJOR_NUM  		 		        100 
#define SUCCESS 				        0
#define BUF_LEN 				        80              /* Max length of the message from the device */
#define SELECT_CRITICAL_SECTION_SKILL 	0

#define CREATE_SYSFS 					1
#define SELF_DETECT_TEST				0
#define ENABLE_EC_GPIO					1
#define ENABLE_LVDS_BACKLIGHT			1
#define CREATE_BATTERY_INFO				1
#define CREATE_AC_INFO					1
#define CREATE_AC_UEVENT				1
#define SELECT_CRITICAL_SECTION_SKILL 	0
#define CREATE_HYBRID_EVENT				1
#define USE_EC_COMMAND_FOR_ALS			1
#define CREATE_AUDIO_MUTE				1
#define CREATE_AUDIO_SWITCH				1
#define CREATE_PM_INFORM				1
#define CREATE_PROC_NODE				1
#define CREATE_RESUME_HYBRID_EVENT		0

static struct compalec_dev *global_ec;
static int Device_Open = 0;                     /* Is device open?  Used to prevent multiple access to device */
static char *msg_Ptr;
static char msg[BUF_LEN];                       /* The msg the device will give when asked */

static int ec_control(struct compalec_dev* ec, EC_I2C_COMMAND_PACKAGE* package);

static ssize_t ecdev_read(struct file *file, S8 __user * buffer, size_t length, loff_t * offset) 
{ 
	int bytesRead = 0;
#if CREATE_HYBRID_EVENT
	int ret;
	U8 c;
    /* 
     * Number of bytes actually written to the buffer 
     */ 
    struct compalec_dev *ec;

    pr_info("+%s\n", __func__);
    
	ec = (struct compalec_dev *)file->private_data;
    
    pr_info("device_read(%p,%p,%d)\n", file, buffer, length); 
    
	ret = wait_event_interruptible(ec->read_wait,
				       kfifo_len(ec->fifo) != 0);
    pr_info("wait_event_interruptible done\n");
	if (ret == -ERESTARTSYS)
		return -ERESTARTSYS;
    //+++[ASD2_ES1|Jaco_Gao|2011.03.10] Modify for k36
    #if 0
	if(kfifo_get(ec->fifo, &c, sizeof(c)) == sizeof(c)){
    #else   // for k36
    if(kfifo_get(ec->fifo, &c) == sizeof(c)){
    #endif
    //---[ASD2_ES1|Jaco_Gao|2011.03.10] Modify for k36
    
		pr_info("kfifo_get = %x\n", c); 
		if (put_user(c, buffer)){
			pr_err("put_user error\n"); 
			return -EFAULT;
		}
	}
#endif
    pr_info("-%s\n", __func__);
    return bytesRead; 
}
 
/* 
 * This function is called when somebody tries to 
 * write into our device file. 
 */ 
static ssize_t ecdev_write(struct file *file, const S8 __user * buffer, size_t length, loff_t * offset) 
{ 
	int res = 0;
#if 0
	struct compalec_dev *ec;
	ec = (struct compalec_dev *)file->private_data;
	unsigned long p = *offset;
	
	unsigned char *in_buf;
	
	in_buf = kmalloc( length, GFP_KERNEL);
	
	if(!in_buf) return -ENOMEM;
    else
		pr_info("device_write(%d, %s, %d)\n", file, buffer, length); 

    if(copy_from_user(in_buf, buffer, length))
		return -EFAULT;
	else{
		*offset += length;
		res = length;
		pr_info("written %d bytes from %d is %s \n", length, p, in_buf);
	}
	/*
    for (i = 0; i < length && i < BUF_LEN; i++)
    { 
        get_user(Message[i], buffer + i); 
    } 
    */
#endif
    return res; 
}
#if 0
static int ecdev_ioctl(struct inode *inode, struct file *file, unsigned int ioctlNum, U32 ioctlParam)
#else
static int ecdev_ioctl(struct file *file, unsigned int ioctlNum, U32 ioctlParam)
#endif
{
	struct compalec_dev *ec;
	int ret;
    //int minor = 0;
    EC_I2C_COMMAND_PACKAGE package;

    pr_info("+%s\n", __func__);
    ec = (struct compalec_dev *)file->private_data;
    //minor=(MINOR(inode->i_rdev) &  0xf);
	/*
    if(bInitial == 0) 
    { 
        set_current_state(TASK_INTERRUPTIBLE);   // For schedule_timeout 
        bInitial=1; 
    } 
    schedule_timeout(1*HZ); // To avoid system busy 
	*/
    ret = copy_from_user(&package, (U32 *)ioctlParam, sizeof(EC_I2C_COMMAND_PACKAGE));    
	if(ret){
		printk(KERN_ERR "copy_from_user error !!!\n");
		goto out;
	}
	if(package.command == EC_CMD_FLASH_EC){
		pr_info("ec_i2c Enter EC flash mode !!!\n");
		//ec->ec_flash_flag = 0;	
		if(package.data == 0x100){
			printk(KERN_INFO "Enter EC flash mode !!!\n");
			ec->ec_flash_flag = 0;
		}
		/*
		else if(package.data == 0x400){
			printk(KERN_INFO "Out EC flash mode !!!\n");
			ec->ec_flash_flag = 1;
		}
		*/
	} else if(package.command == EC_CMD_LONG_DATA_START){
		ec->ec_flash_flag = 0;
		goto out;
	} else if(package.command == EC_CMD_LONG_DATA_END){
		ec->ec_flash_flag = 1;
		goto out;
	}
	/*
	package.data = 2;
	printk("%s,file command = 0x%02x\n", __func__, package.command);
	printk("%s,file action = 0x%02x\n", __func__, package.action);
	printk("file out data = %d\n", package.data);
	*/
	//spin_lock(&ec->lock);
	//ec->package = &package;
	ec_control(ec, &package);
	//spin_unlock(&ec->lock);
    ret = copy_to_user ((U32 *)ioctlParam, &package, sizeof(EC_I2C_COMMAND_PACKAGE));
	if(ret){
		printk(KERN_ERR "copy_to_user error !!!\n");
		goto out;
	}

out:
    pr_info("-%s\n", __func__);
    return ret;
}

static int ecdev_open(struct inode *inode, struct file *file)
{
    pr_info("+%s\n", __func__);
    pr_info("Device Open = %d\n", Device_Open);
	file->private_data = global_ec;
	if (Device_Open > 2)
	  return -EBUSY;
	Device_Open++;
	msg_Ptr = msg;
	try_module_get(THIS_MODULE);
    pr_info("-%s\n", __func__);
	return SUCCESS;
}

static int ecdev_release(struct inode *inode, struct file *file)
{
	struct compalec_dev *ec;
    pr_info("+%s\n", __func__);
    pr_info("Device Open = %d\n", Device_Open);
	ec = (struct compalec_dev *)file->private_data;
	Device_Open --;
	module_put(THIS_MODULE);
	file->private_data = NULL;
    pr_info("-%s\n", __func__);
	return 0;
}

static const struct file_operations ecdev_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= ecdev_read,
	.write		= ecdev_write,
    //+++[ASD2_ES1|Jaco_Gao|2011.03.10] Modify for k36
    #if 0
    .ioctl		= ecdev_ioctl,
    #else
    .unlocked_ioctl   = ecdev_ioctl,
    #endif
    //---[ASD2_ES1|Jaco_Gao|2011.03.10] Modify for k36
	.open		= ecdev_open,
	.release	= ecdev_release,
};

static struct miscdevice ec_dev_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &ecdev_fops,
};

void ec_dev_register( void )
{
    #if 0
    if ( register_chrdev(MAJOR_NUM, DEVICE_NAME, &ecdev_fops) < 0 )
    {
        pr_err("fail to register EC dev\n");
    }
    #else
    if ( misc_register(&ec_dev_device) < 0 )
    {
        pr_err("fail to register EC dev\n");
    }
    #endif
}

void ec_dev_unregister( void )
{
    unregister_chrdev(MAJOR_NUM, DEVICE_NAME);
}

void ec_dev_init( struct i2c_client *client )
{
    struct compalec_dev *ec;

    pr_info("+%s\n", __func__);
    
	ec = kzalloc(sizeof(struct compalec_dev), GFP_KERNEL);
	global_ec = ec;
	if (ec == NULL) {
		printk(KERN_ERR "out of mem\n");
		return;
	}
	ec->ec_flash_flag = 1;
	
	//i2c_set_clientdata(client, ec);
	ec->dev = &client->dev;
	ec->client = client;
	ec->databuf = 0x0;
    
#if CREATE_HYBRID_EVENT
	init_waitqueue_head(&ec->read_wait);
    //+++[ASD2_ES1|Jaco_Gao|2011.03.10] Modify for k36
    #if 0
	spin_lock_init(&ec->fifo_lock);
	ec->fifo = kfifo_alloc(BUFSIZE, GFP_KERNEL, &ec->fifo_lock);
	if (IS_ERR(ec->fifo)){
		printk(KERN_ERR "kfifo_alloc failed\n");
		//return PTR_ERR(ec->fifo);
	}
    #else // for k36
    ec->fifo = kzalloc(sizeof(typeof(*ec->fifo)), GFP_KERNEL);
    if( !ec->fifo )         //[ASD2_ES1|Jaco_Gao|2011.05.06] Fix syntax error in checking return value of kzalloc
    {
        pr_err("fail to kzalloc for ec->fifo\n");
    }
    if ( kfifo_alloc(ec->fifo, BUFSIZE, GFP_KERNEL) )
    {
        printk(KERN_ERR "kfifo_alloc failed\n");
    }
    #endif
    //---[ASD2_ES1|Jaco_Gao|2011.03.10] Modify for k36
#endif
// Initialize critical section method 
#if SELECT_CRITICAL_SECTION_SKILL
	spin_lock_init(&ec->lock);
#else
	sema_init(&ec->mutex, 1);
#endif

    pr_info("-%s\n", __func__);

}
//used to sending/getting ec command
static int ec_control(struct compalec_dev* ec, EC_I2C_COMMAND_PACKAGE* package)
{
	//U16 data = 0;
	//u8 buf[4];
    int rc = 0;

    pr_info("+%s\n", __func__);
    
#if SELECT_CRITICAL_SECTION_SKILL    
    spin_lock(&ec->lock);
#else
	if (down_interruptible(&ec->mutex)){
		printk(KERN_ERR "Didn't get the semaphore before ec_control!!!\n");
		return -1;
	}
#endif
	//receive data
	if(!package->action){
		rc = i2c_smbus_read_word_data_retry(ec->client, package->command);
		pr_info("\n%s: Command = 0x%x Read Data 0x%x...\n", 
		__func__, package->command, package->data);
		if (rc < 0){
			printk(KERN_ERR "%s: error: receive data\n", __func__);
			goto RETURN_POINT;
		} else {
			package->data = rc;
		}

	} else {
	//write data
		pr_info("%s: Command = 0x%x Write Data 0x%x...\n", 
		__func__, package->command, package->data);
		rc = i2c_smbus_write_word_data_retry(ec->client, package->command, package->data);
		if (rc){
			printk(KERN_ERR "%s: error: write data\n", __func__);
			goto RETURN_POINT;
		}
	}

RETURN_POINT:
#if SELECT_CRITICAL_SECTION_SKILL 
	spin_unlock(&ec->lock);
#else
	up(&ec->mutex);
#endif

    pr_info("-%s\n", __func__);

	return rc;
	
}
//---[ASD2_ES1|Jaco_Gao|2011.03.08] for EC Tool

//peter
static void tegra_battery_poll_timer_func(unsigned long unused)
{
        power_supply_changed(&bq20z75_supply[NvCharger_Type_Battery]);
        mod_timer(&poll_timer, jiffies + msecs_to_jiffies(batt_status_poll_period));
}




//peter
static irqreturn_t ac_interrupt(int irq, void *dev_id)
{
        if(ECflashMode == 0)       //peter not polling EC for battery info when ECflash
          {
              power_supply_changed(&bq20z75_supply[NvCharger_Type_Battery]);
              power_supply_changed(&bq20z75_supply[NvCharger_Type_AC]);
          }
        return IRQ_HANDLED;
}



//peter SYS_START
static struct kobject *Ecdebug_kobj;

//Compal_Start + Thomas Wu : open EC permission on non user mode
#if DEBUGMODE
	#define debug_attr(_name) \
		static struct kobj_attribute _name##_attr = { \
		.attr = { \
		.name = __stringify(_name), \
		.mode = 0666, \
		}, \
		.show = _name##_show, \
		.store = _name##_store, \
		}
#else
	#define debug_attr(_name) \
		static struct kobj_attribute _name##_attr = { \
		.attr = { \
		.name = __stringify(_name), \
		.mode = 0660, \
		}, \
		.show = _name##_show, \
		.store = _name##_store, \
		}

#endif
//Compal_End

int atoi(const char *a)
{
int s = 0;
    while(*a >= '0' && *a <= '9')
        s = (s << 3) + (s << 1) + *a++ - '0';
    return s;
}

static ssize_t PowerLED_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "PowerLED");
        return (s - buf);
}

static ssize_t PowerLED_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x42,0);
        msleep(100);
        return n;
}

static ssize_t ChargeLED_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "ChargeLED");
        return (s - buf);
}

static ssize_t ChargeLED_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x43,0);
        msleep(100);
        return n;
}
static ssize_t OriStsLED_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "OriStsLED");
        return (s - buf);
}

static ssize_t OriStsLED_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x40,0);
        msleep(100);
        return n;
}
static ssize_t OffLED_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "OffLED");
        return (s - buf);
}

static ssize_t OffLED_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x41,0);
        msleep(100);
        return n;
}
static ssize_t BatCtlEnable_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n",BatCtlEnable_val);
        return (s - buf);
}
static ssize_t BatCtlEnable_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        int buffer;
        buffer = atoi(buf);
        BatCtlEnable_val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x50,BatCtlEnable_val);
        msleep(100);
        return n;
}
static ssize_t BatCtlDisable_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "BatCtlDisable");
        return (s - buf);
}
static ssize_t BatCtlDisable_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x51,0);
        msleep(100);
        return n;
}

static ssize_t EcVer_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 PjIDMaVer, MiVerTestVer, ret;
        PjIDMaVer = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x30);
        msleep(100);
        MiVerTestVer = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x31);
        msleep(100);
        ret = PjIDMaVer;
        ret = ret << 16 | (MiVerTestVer & 0xFFFF);
        s += sprintf(s, "%x\n", ret);
        return (s - buf);
}

static ssize_t EcVer_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}

static void TransformToByte(u16 val16, u8 *val8_0, u8 *val8_1)
{
     *val8_0 = val16 & 0x00ff;
     *val8_1 = val16 >> 8 & 0x00ff;


}

static ssize_t UUID_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{

        int i;
        char * s = buf;
        u8 val8[16] = {0};
        u16 val16;
        s32 val32;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);
        msleep(100);

        for(i=0;i<=7;i++)
           {
               val32 = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x60);
               val16 = val32 & 0x0000ffff;
               TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
               msleep(10);
           }
        s += sprintf(s, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",val8[15],val8[14],val8[13],val8[12],val8[11],val8[10],val8[9],val8[8],val8[7],val8[6],val8[5],val8[4],val8[3],val8[2],val8[1],val8[0]);
         

        return (s - buf);
}

static u8 TransformCharToByte(const char *buf, int index)
{


       u8  val8;
       char locbuf[2] = {0};
       int buffer = 0, j;

 
      for(j=0;j<=1;j++)
           {
                  locbuf[j] = buf[index+j];
           }

               if(locbuf[1] == 'a' || locbuf[1] == 'A')
                  buffer = 10;
               else if(locbuf[1] == 'b' || locbuf[1] == 'B')
                       buffer = 11;
               else if(locbuf[1] == 'c' || locbuf[1] == 'C')
                       buffer = 12;
               else if(locbuf[1] == 'd' || locbuf[1] == 'D')
                       buffer = 13;
               else if(locbuf[1] == 'e' || locbuf[1] == 'E')
                       buffer = 14;
               else if(locbuf[1] == 'f' || locbuf[1] == 'F')
                       buffer = 15;
               else if(locbuf[1] == '0')
                       buffer = 0;
               else if(locbuf[1] == '1')
                       buffer = 1;
               else if(locbuf[1] == '2')
                       buffer = 2;
               else if(locbuf[1] == '3')
                       buffer = 3;
               else if(locbuf[1] == '4')
                       buffer = 4;
               else if(locbuf[1] == '5')
                       buffer = 5;
               else if(locbuf[1] == '6')
                       buffer = 6;
               else if(locbuf[1] == '7')
                       buffer = 7;
               else if(locbuf[1] == '8')
                       buffer = 8;
               else if(locbuf[1] == '9')
                       buffer = 9;


               if(locbuf[0] == 'a' || locbuf[0] == 'A')
                  buffer += 10*16;
               else if(locbuf[0] == 'b' || locbuf[0] == 'B')
                       buffer += 11*16;
               else if(locbuf[0] == 'c' || locbuf[0] == 'C')
                       buffer += 12*16;
               else if(locbuf[0] == 'd' || locbuf[0] == 'D')
                       buffer += 13*16;
               else if(locbuf[0] == 'e' || locbuf[0] == 'E')
                       buffer += 14*16;
               else if(locbuf[0] == 'f' || locbuf[0] == 'F')
                       buffer += 15*16;
               else if(locbuf[0] == '0')
                       buffer += 0*16;
               else if(locbuf[0] == '1')
                       buffer += 1*16;
               else if(locbuf[0] == '2')
                       buffer += 2*16;
               else if(locbuf[0] == '3')
                       buffer += 3*16;
               else if(locbuf[0] == '4')
                       buffer += 4*16;
               else if(locbuf[0] == '5')
                       buffer += 5*16;
               else if(locbuf[0] == '6')
                       buffer += 6*16;
               else if(locbuf[0] == '7')
                       buffer += 7*16;
               else if(locbuf[0] == '8')
                       buffer += 8*16;
               else if(locbuf[0] == '9')
                       buffer += 9*16;

                  val8 = buffer & 0x000000ff;


          return val8;

}

static u16 TransformToWord(u8 val8_0, u8 val8_1)
{

      u16 val16;
      val16 = val8_1;
      val16 = val16 << 8 | val8_0;

      return val16;

}

static ssize_t UUID_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{


        u8  val8_0, val8_1;
        u16 val16;
        int i;
        
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);
        msleep(100);

        for(i=0;i<=7;i++)
           {
                val8_0 = TransformCharToByte(buf,30-4*i);
                val8_1 = TransformCharToByte(buf,28-4*i);

                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data_retry(bq20z75_device->client,0x61,val16);
                msleep(10);
           }
 

      return n;
}

static ssize_t BatCapacity_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 ret;
        ret = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x00);   //peter
        msleep(100);
        s += sprintf(s, "%d\n", ret);    //peter
        return (s - buf);
}

static ssize_t BatCapacity_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}

static ssize_t BatDesignCapacity_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 ret;
        ret = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x08);
        msleep(100);
        s += sprintf(s, "%d\n", ret);
        return (s - buf);
}

static ssize_t BatDesignCapacity_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}
 
static ssize_t BTMAC_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        int i;
        char * s = buf;
        u8 val8[6] = {0};
        u16 val16;
        s32 val32;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);
        msleep(100);

        for(i=0;i<=2;i++)
           {
               val32 = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x62);
               val16 = val32 & 0x0000ffff;
               TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
               msleep(10);
           }

        s += sprintf(s, "%02x%02x%02x%02x%02x%02x\n",val8[5],val8[4],val8[3],val8[2],val8[1],val8[0]);

        return (s - buf);

}

static ssize_t BTMAC_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;
        int i;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);
        msleep(100);

	for(i=0;i<=2;i++)
           {
                val8_0 = TransformCharToByte(buf,10-4*i);
                val8_1 = TransformCharToByte(buf,8-4*i);

                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data_retry(bq20z75_device->client,0x63,val16);
                msleep(10);
           }

        return n;
}
static ssize_t WIFIMAC_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        int i;
        char * s = buf;
        u8 val8[6] = {0};
        u16 val16;
        s32 val32;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);
        msleep(100);

        for(i=0;i<=2;i++)
           {
               val32 = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x64);
               val16 = val32 & 0x0000ffff;
               TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
               msleep(10);
           }

        s += sprintf(s, "%02x%02x%02x%02x%02x%02x\n",val8[5],val8[4],val8[3],val8[2],val8[1],val8[0]);

        return (s - buf);

}

static ssize_t WIFIMAC_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{       
        u8  val8_0, val8_1;
        u16 val16;
        int i;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);
        msleep(100);

        for(i=0;i<=2;i++)
           {
                val8_0 = TransformCharToByte(buf,10-4*i);
                val8_1 = TransformCharToByte(buf,8-4*i);

                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data_retry(bq20z75_device->client,0x65,val16);
                msleep(10);
           }

        return n;
}


static ssize_t BatStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        int ACStatus;
        s32 Capacity, Present;
        Present = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x08);   //peter read designcapacity to judge present
        msleep(100);
        Capacity = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x00);    //peter read capacity
        ACStatus = gpio_get_value(gpio);                                //peter read ac exist gpio, 0 is exist

        if(Present == 0)
          {
                  s += sprintf(s, "Absence\n");
          }
        else
          {
                  if(Capacity < 100)
                    {
               		if(ACStatus == 1)
                 	 s += sprintf(s, "Discharging\n");
              		else
                  	 s += sprintf(s, "Charging\n");
                    }
                  else
                    {
                        s += sprintf(s, "Full\n");
                    }
          }
        msleep(100);
        return (s - buf);
}

static ssize_t BatStatus_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}

static ssize_t ECRead_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        u16 val16;
        s32 val32;


        val32 = i2c_smbus_read_word_data_retry(bq20z75_device->client,0xF1);
        val16 = val32 & 0x0000ffff;

        s += sprintf(s, "0x%04x\n",val16);

        return (s - buf);

}

//echo addresshbyte_addresslbyte > ECRead
static ssize_t ECRead_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;

        val8_0 = TransformCharToByte(buf,2);
        val8_1 = TransformCharToByte(buf,0);

        val16 = TransformToWord(val8_0, val8_1);
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0xF0,val16);

        return n;
}

static ssize_t ECWrite_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "ECWrite");
        return (s - buf);

}

//echo addresshbyte_addresslbyte_val > ECWrite
static ssize_t ECWrite_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;

      
        val8_0 = TransformCharToByte(buf,2);
        val8_1 = TransformCharToByte(buf,0);
        val16 = TransformToWord(val8_0, val8_1);
   
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0xF0,val16);   //write address
        
        val8_1 = 0;
        val8_0 = TransformCharToByte(buf,4);
        val16 = TransformToWord(val8_0, val8_1);
    
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0xF2,val16);   //write value 00xx
        
        return n;
}


static ssize_t Shutdown_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "Shutdown");
        return (s - buf);
}

static ssize_t Shutdown_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x52,0);
        msleep(100);
        return n;
}

static ssize_t Suspend_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "Suspend");
        return (s - buf);
}
//+++[ASD2-ES1 - Darren - 2011.4.4] [Modify] Modify suspend write 1 to 0x53
static ssize_t Suspend_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        //i2c_smbus_write_word_data_retry(bq20z75_device->client,0x53,0);
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x53,1);
        msleep(100);
        return n;
}
//---[ASD2-ES1 - Darren - 2011.4.4] [Modify] Change suspend write 1 to 0x53

static ssize_t Coldboot_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n",Coldboot_val);
        return (s - buf);
}

static ssize_t Coldboot_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        int buffer;
        buffer = atoi(buf);
        Coldboot_val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x55,Coldboot_val);
        msleep(100);
        return n;

}

static ssize_t Resume_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n",Resume_val);
        return (s - buf);
}

static ssize_t Resume_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
 
        int buffer;
        buffer = atoi(buf);
        Resume_val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x56,Resume_val);
        msleep(100);
        return n;

}


static ssize_t RecoveryMode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "RecoveryMode");
        return (s - buf);
}

static ssize_t RecoveryMode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        int buffer;
        s16 val;
        buffer = atoi(buf);
        val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x58,val);
        msleep(100);
        return n;

}




static ssize_t ECflashwrite_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
       
        char * s = buf;
        s += sprintf(s, "ECflashwrite");
        return (s - buf);

}

//echo cmd_val16 > ECflashwrite
static ssize_t ECflashwrite_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        u16 val16;
        val16 = TransformToWord(buf[2], buf[1]);
        i2c_smbus_write_word_data_retry(bq20z75_device->client, buf[0], val16);   

        return n;

}


char cmd;

static ssize_t ECflashread_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{

        s32 ret;


        ret = i2c_smbus_read_word_data_retry(bq20z75_device->client,cmd);

        buf[0] = (ret & 0x0000ff00) >> 8; 
        buf[1] = ret & 0x000000ff;
        
        return 2; 

}

static ssize_t ECflashread_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        cmd = buf[0];
        return n;

}



static ssize_t MicSwitch_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n",MicSwitch_val);
        return (s - buf);
}


//echo 1 > MicSwitch(front) ,  echo 0 > MicSwitch(back)
static ssize_t MicSwitch_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        int buffer;
        buffer = atoi(buf);
        MicSwitch_val = buffer & 0x0000FFFF;
        if(MicSwitch_val != 0 && MicSwitch_val != 1)
           {
               printk("MicSwitch: echo value should be 0 or 1\n");
               goto fail;
           }
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x44,MicSwitch_val);
        msleep(100);
fail:
        return n;
}

//+++[ASD2ES1 | Darren | 2011.05.25 | Add ThreeGPower & SimDetect ]
static ssize_t ThreeGPower_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 ThreeGPower_val, ret;
        ThreeGPower_val = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x45);
        msleep(100);
        ret = ThreeGPower_val;
        s += sprintf(s, "%d\n", ret);
        return (s - buf);

}

//+++[ASD2ES1 | Darren | 2011.07.04 | Add ThreeGPower function]
//echo 1 > ThreeGPower(poweron) ,  echo 0 > ThreeGPower(poweroff), echo 2 > ThreeGPower(backtoOriginstate)
static ssize_t ThreeGPower_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int buffer;
	s16 val16;
	buffer = atoi(buf);
	val16 = buffer & 0x0000FFFF;
	i2c_smbus_write_word_data(bq20z75_device->client,0x45,val16);
	msleep(100);
	return n;
}
//---[ASD2ES1 | Darren | 2011.07.04 | Add ThreeGPower function]


static ssize_t SimDetect_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s32 SimDetect_val, ret;
        SimDetect_val = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x73);
        msleep(100);
        ret = SimDetect_val;
        s += sprintf(s, "%d\n",ret);
        return (s - buf);
}


static ssize_t SimDetect_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        return n;
}
//---[ASD2ES1 | Darren | 2011.05.25 | Add ThreeGPower & SimDetect ]



static ssize_t SerialNumber_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        int i;
        char * s = buf;
        u8 val8[14] = {0};
        u16 val16;
        s32 val32;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);
        msleep(100);

        for(i=0;i<=6;i++)
           {
               val32 = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x66);
               val16 = val32 & 0x0000ffff;
               TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
               msleep(10);
           }

//+++[ASD1ES2 | Darren | 2011.04.12 | Modify return ASCII SN ]

#if 0
        s += sprintf(s, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		val8[12],val8[11],val8[10],val8[9],val8[8],val8[7],val8[6],val8[5],val8[4],val8[3],val8[2],val8[1],val8[0]);
#else
        //return ASCII SN
        s += sprintf(s, "%c%c%c%c%c%c%c%c%c%c%c%c%c\n",val8[12],val8[11],
		val8[10],val8[9],val8[8],val8[7],val8[6],val8[5],val8[4],val8[3],val8[2],val8[1],val8[0]);
#endif

//---[ASD1ES2 | Darren | 2011.04.12 | Modify return ASCII SN ]

        return (s - buf);

}

static ssize_t SerialNumber_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;
        int i;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);
        msleep(100);

        for(i=0;i<=5;i++)
           {
                val8_0 = TransformCharToByte(buf,24-4*i);
                val8_1 = TransformCharToByte(buf,22-4*i);

                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data_retry(bq20z75_device->client,0x67,val16);
                msleep(10);
           }


        val8_0 = TransformCharToByte(buf,0);
        val8_1 = 0;

        val16 = TransformToWord(val8_0, val8_1);
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x67,val16);
        msleep(10);

        return n;
}
                            

//peter temperary ec flash solution, need to remove 
static ssize_t ECflashMode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "%d\n", ECflashMode);
        return (s - buf);
}

//peter echo 1 > ECflashMode (ecflash on)
//      echo 0 > ECflashMode (ecflash off)
static ssize_t ECflashMode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        int buffer;
        buffer = atoi(buf);
        ECflashMode = buffer & 0x00000001;
        if(ECflashMode == 0)
          {
               setup_timer(&poll_timer, tegra_battery_poll_timer_func, 0);
               mod_timer(&poll_timer, jiffies + msecs_to_jiffies(batt_status_poll_period));
          }
        else
          {
               del_timer_sync(&poll_timer);
          }
        return n;

}



static ssize_t SkuNumber_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        
        char * s = buf;
        u8 val8[2] = {0};
        u16 val16;
        s32 val32;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);
        msleep(100);

        val32 = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x68);
        val16 = val32 & 0x0000ffff;
        TransformToByte(val16, &val8[0], &val8[1]);
        msleep(10);

        s += sprintf(s, "%02x%02x\n",val8[1],val8[0]);

        return (s - buf);

}



static ssize_t SkuNumber_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        u8  val8_0, val8_1;
        u16 val16;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);

        val8_0 = TransformCharToByte(buf,2);
        val8_1 = TransformCharToByte(buf,0);

        val16 = TransformToWord(val8_0, val8_1);
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x69,val16);
        msleep(10);

        return n;
}



//echo 1 > LEDAndroidOff
static ssize_t LEDAndroidOff_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        char * s = buf;
        s += sprintf(s, "LEDAndroidOff");
        return (s - buf);
}

static ssize_t LEDAndroidOff_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x5a,0);
        msleep(100);
        return n;
}

static ssize_t ManufactureDate_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	int i;
     	char * s = buf;
        u8 val8[16] = {0};
        u16 val16;
        s32 val32;
 
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);

        for(i=0;i<=1;i++)
        {
        	val32 = i2c_smbus_read_word_data_retry(bq20z75_device->client, 0x6e);
                val16 = val32 & 0x0000ffff;
                TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
                msleep(10);
        }

        s += sprintf(s, "%02x%02x-%02x-%02x\n",val8[3],val8[2],val8[1],val8[0]);

        return (s - buf);
}

static ssize_t ManufactureDate_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        u8  val8_0, val8_1;
        u16 val16;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);

        val8_0 = TransformCharToByte(buf,6);
        val8_1 = TransformCharToByte(buf,4);
        val16 = TransformToWord(val8_0, val8_1);
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x6f,val16);
        msleep(10);

        val8_0 = TransformCharToByte(buf,2);
        val8_1 = TransformCharToByte(buf,0);
        val16 = TransformToWord(val8_0, val8_1);
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x6f,val16);
        msleep(10);

	return n;
}

static ssize_t SerialNumberwithoutBarcode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	int i;
     	char * s = buf;
	u32 val32;
	u16 val16;
	u8 val8[22] = {0};
	u8 nostore_before = 0;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);

	for(i=10;i>=0;i--)
	{
		val32 = i2c_smbus_read_word_data_retry(bq20z75_device->client, 0x6a);
		val16 = val32 & 0x0000ffff;
		msleep(10);
		val8[2*i+1] = val16 & 0x000000ff;
		val8[2*i] = (val16 >> 8) & 0x000000ff;
	}

	for(i=0;i<=21;i++)
	{
		if(val8[i] != 0xff)
			nostore_before = 0;
		else
			nostore_before = 1;
	}

	if(nostore_before)
		s += sprintf(s, "ffffffffffffffffffffff\n");
	else
		s += sprintf(s, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n", val8[21],val8[20],val8[19],val8[18],val8[17],val8[16],val8[15],val8[14],val8[13],val8[12],val8[11],val8[10],val8[9],val8[8],val8[7],val8[6],val8[5],val8[4],val8[3],val8[2],val8[1],val8[0]);

	return (s - buf);
}
static ssize_t SerialNumberwithoutBarcode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

	u8  val8_0, val8_1;
        u16 val16;
        int i;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);

        for(i=0;i<11;i++)
        {
        	val8_0 = buf[2*i];
                val8_1 = buf[2*i+1];

                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data_retry(bq20z75_device->client,0x6b,val16);

                msleep(10);
        }

	return n;
}

static ssize_t IMEIwithBarcode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
        int i,j=0;
        char * s = buf;
        u8 val8[16] = {0};
        u16 val16;
        s32 val32;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);

        for(i=0;i<=3;i++)
        {
                val32 = i2c_smbus_read_word_data_retry(bq20z75_device->client, 0x6c);
                val16 = val32 & 0x0000ffff;
                TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
                msleep(10);
        }

	if(val8[0]&0x80)
		j+=8;
	if(val8[0]&0x40)
		j+=4;
	if(val8[0]&0x20)
		j+=2;
	if(val8[0]&0x10)
		j+=1;
	
        s += sprintf(s, "%02x%02x%02x%02x%02x%02x%02x%d\n",val8[7],val8[6],val8[5],val8[4],val8[3],val8[2],val8[1],j);

	return (s - buf);
}
static ssize_t IMEIwithBarcode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	u8  val8_0, val8_1;
        u16 val16;
        int i;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);

        for(i=0;i<=3;i++)
        {
        	val8_0 = TransformCharToByte(buf,14-4*i);
                val8_1 = TransformCharToByte(buf,12-4*i);

                val16 = TransformToWord(val8_0, val8_1);
                i2c_smbus_write_word_data_retry(bq20z75_device->client,0x6d,val16);
                msleep(10);
        }

	return n;
}


//[ASD2-ES1 - Darren - 2011.3.16] Add EC new function
//+++[ASD2-ES1 - Darren]
static ssize_t BrightnessLevel_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "%d\n",BrightnessLevel_val);
    return (s - buf);
}

static ssize_t BrightnessLevel_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int buffer;
    buffer = atoi(buf);
    BrightnessLevel_val = buffer & 0x0000FFFF;
    if(BrightnessLevel_val > 13)
    {
        printk("BrightnessLevel: echo value should be 0->12 \n");
        goto fail;
    }
    i2c_smbus_write_word_data_retry(bq20z75_device->client,0x20,BrightnessLevel_val);
    msleep(100);
fail:
    return n;
}


static ssize_t BaseState_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s32 BaseState_val, ret;
    BaseState_val = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x21);
    msleep(100);
    ret = BaseState_val;
    s += sprintf(s, "%d\n",BaseState_val);
    return (s - buf);
}


static ssize_t BaseState_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

   return n;
}


static ssize_t QueryCommand_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s32 QueryCommand_val, ret;
    QueryCommand_val = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x25);
    msleep(100);
    ret = QueryCommand_val;
    s += sprintf(s, "%d\n", ret);
    return (s - buf);
}

static ssize_t QueryCommand_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return n;
}

static ssize_t BaseWirelessState_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s32 BaseWirelessState_val, ret;
    BaseWirelessState_val = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x5B);
    msleep(100);
    ret = BaseWirelessState_val;
    s += sprintf(s, "%d\n", ret);
    return (s - buf);
}

static ssize_t BaseWirelessState_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return n;
}


static ssize_t PlugAction_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s32 PlugAction_val, ret; 
    PlugAction_val = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x5C);
    msleep(100);
    ret = PlugAction_val;
    s += sprintf(s, "%d\n", ret);
    return (s - buf);
}

static ssize_t PlugAction_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return n;
}

static ssize_t BatLifeTest_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
//+++[ASD2 ES1 -- Darren -- 2011.04.17 -- Modify query command to 70]

	int i;
	char * s = buf;

        u8 val8[16] = {0};
        u16 val16;
        s32 val32;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);

        for(i=0;i<=1;i++)
        {
		val32 = i2c_smbus_read_word_data_retry(bq20z75_device->client, 0x70);
                val16 = val32 & 0x0000ffff;
                TransformToByte(val16, &val8[2*i], &val8[2*i+1]);
                msleep(10);
        }

        s += sprintf(s, "%02x%02x%02x%02x\n",val8[3],val8[2],val8[1],val8[0]);

        return (s - buf);

//---[ASD2 ES1 -- Darren -- 2011.04.17 -- Modify query command to 70]
}

static ssize_t BatLifeTest_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        int buffer;
        s16 val;
        buffer = atoi(buf);
        val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x59,val);
        msleep(100);
        return n;

}
//---[ASD2-ES1 - Darren]

//+++[ASD2-ES1 - Darren - 2011.04.14 - Add TPDL command]

static ssize_t TPDLFlashStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s32 FlashStatus_val, ret;
    FlashStatus_val = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x72);
    msleep(100);
    ret = FlashStatus_val;
    s += sprintf(s, "%x\n", ret);
    return (s - buf);
}

static ssize_t TPDLFlashStatus_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return n;
}

static ssize_t TPDLFlashStart_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 FlashStart_val, ret;
	FlashStart_val = i2c_smbus_read_word_data_retry(bq20z75_device->client,0xD9);
	msleep(100);
	ret = FlashStart_val;
	s += sprintf(s, "%d\n", ret);
	return (s - buf);
}

static ssize_t TPDLFlashStart_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        int buffer;
        s16 val;
        buffer = atoi(buf);
        val = buffer & 0x0000FFFF;
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0xD9,val);
        msleep(100);
        return n;

}

//+++[ASD2-ES1 - Darren - 2011.07.25 - Add MBID function]
static ssize_t MBID_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{

        char * s = buf;
        u16 val16;
        s32 val32;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);
        msleep(100);

        val32 = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x5f);
        val16 = val32 & 0x000000ff;
        msleep(10);
        s += sprintf(s, "%02x\n",val16);
        return (s - buf);
}

static ssize_t MBID_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

        u8  val8_0;
        u16 val16;

        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x49,1);
        val8_0 = TransformCharToByte(buf,0);

        val16 = TransformToWord(val8_0, 0);
        i2c_smbus_write_word_data_retry(bq20z75_device->client,0x5f,val16);
        msleep(10);
        return n;
}
//---[ASD2ES1 | Darren | 2011.07.25 | Add MBID function]

debug_attr(EcVer);
debug_attr(UUID);
debug_attr(PowerLED);
debug_attr(ChargeLED);
debug_attr(OriStsLED);
debug_attr(OffLED);
debug_attr(BatCtlEnable);
debug_attr(BatCtlDisable);
debug_attr(BatCapacity);
debug_attr(BatDesignCapacity);
debug_attr(BatStatus);
debug_attr(BTMAC);
debug_attr(WIFIMAC);
debug_attr(ECRead);
debug_attr(ECWrite);
debug_attr(Shutdown);
debug_attr(Suspend);
debug_attr(Coldboot);
debug_attr(Resume);
debug_attr(RecoveryMode);
debug_attr(ECflashwrite);
debug_attr(ECflashread);
debug_attr(MicSwitch);
debug_attr(ThreeGPower);
debug_attr(SerialNumber);
debug_attr(ECflashMode);
debug_attr(SkuNumber);
debug_attr(LEDAndroidOff);
debug_attr(ManufactureDate);
debug_attr(SerialNumberwithoutBarcode);
debug_attr(IMEIwithBarcode);
//[ASD2-ES1 - Darren - 2011.3.16] Add EC new function
//+++[ASD2-ES1 - Darren]
debug_attr(BrightnessLevel);
debug_attr(BaseState);
debug_attr(QueryCommand);
debug_attr(BaseWirelessState);
debug_attr(PlugAction);
debug_attr(BatLifeTest);
//[ASD2-ES1 - Darren - 2011.3.16] Add EC new function
//---[ASD2-ES1 - Darren]

//+++[ASD2-ES1 - Darren - 2011.4.14 - Add TPDL command ]
debug_attr(TPDLFlashStatus);
debug_attr(TPDLFlashStart);
//---[ASD2-ES1 - Darren]

debug_attr(SimDetect);
debug_attr(MBID);

static struct attribute * g[] = {
        &EcVer_attr.attr,
	&UUID_attr.attr,
	&PowerLED_attr.attr,
	&ChargeLED_attr.attr,
	&OriStsLED_attr.attr,
	&OffLED_attr.attr,
	&BatCtlEnable_attr.attr,
	&BatCtlDisable_attr.attr,
	&BatCapacity_attr.attr,
	&BatDesignCapacity_attr.attr,
	&BatStatus_attr.attr,
	&BTMAC_attr.attr,
	&WIFIMAC_attr.attr,
	&ECRead_attr.attr,
	&ECWrite_attr.attr,
	&Shutdown_attr.attr,
	&Suspend_attr.attr,
	&Coldboot_attr.attr,
	&Resume_attr.attr,
	&ECflashwrite_attr.attr,
	&ECflashread_attr.attr,
	&RecoveryMode_attr.attr,
	&MicSwitch_attr.attr,
	&ThreeGPower_attr.attr,
        &SerialNumber_attr.attr,
	&ECflashMode_attr.attr,
	&SkuNumber_attr.attr,
	&LEDAndroidOff_attr.attr,
	&ManufactureDate_attr.attr,
	&SerialNumberwithoutBarcode_attr.attr,
	&IMEIwithBarcode_attr.attr,
	//[ASD2-ES1 - Darren - 2011.3.16] Add EC new function
	//+++[ASD2-ES1 - Darren]
	&BrightnessLevel_attr.attr,
	&BaseState_attr.attr,
	&QueryCommand_attr.attr,
	&BaseWirelessState_attr.attr,
	&PlugAction_attr.attr,
	&BatLifeTest_attr.attr,
	//---[ASD2-ES1 - Darren]
	//+++[ASD2-ES1 - Darren - 2011.4.14 - Add TPDL command ]
	&TPDLFlashStatus_attr.attr,
	&TPDLFlashStart_attr.attr,
	//---[ASD2-ES1 - Darren - 2011.4.14 - Add TPDL command ]
	&SimDetect_attr.attr,
	&MBID_attr.attr,
        NULL,
};


static struct attribute_group attr_group = {
        .attrs = g,
};
//peter SYS_END



//peter
void SysShutdown(void )
{
	i2c_smbus_write_word_data_retry(bq20z75_device->client,0x52,0);

}
EXPORT_SYMBOL(SysShutdown);

//+++ [ASD2 ES1 - Darren - Modify reboot to call EC coldboot]

void SysRestart(void )
{
	i2c_smbus_write_word_data_retry(bq20z75_device->client,0x55,1);

}
EXPORT_SYMBOL(SysRestart);

//+++ [ASD2 ES1 - Darren - Modify reboot to call EC coldboot]

static int bq20z75_get_battery_presence_and_health(enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;

        if (psp == POWER_SUPPLY_PROP_PRESENT) 
           {

         	ret = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x08);   //peter read designcapacity to judge present
        	if (ret < 0) {
	        	dev_err(&bq20z75_device->client->dev,
		        	"%s: i2c read for battery presence failed\n", __func__);
	        	return -EINVAL;
	                 }
                if(ret == 0)
                      val->intval = 0;
                else
                      val->intval = 1;

               // msleep(100); //remove msleep by lenovo suggestion

           }
        else if (psp == POWER_SUPPLY_PROP_HEALTH) 
               {
  
	               val->intval = POWER_SUPPLY_HEALTH_GOOD;

               } 

	return 0;
}



static int bq20z75_get_battery_property(int reg_offset,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;
        int BatPresent, ACStatus;
        s32 Capacity;
	if (psp == POWER_SUPPLY_PROP_STATUS) 
           {

                  BatPresent = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x08);     //peter read designcapacity to judge present
                  //msleep(100); //remove msleep by lenovo suggestion                                               //because upper level judge 
                                                                              //POWER_SUPPLY_PROP_PRESENT is after POWER_SUPPLY_PROP_STATUS
                 if(BatPresent)      //peter judge present
                   {
            	       Capacity = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x00);    //peter read capacity
        	       ACStatus = gpio_get_value(gpio);                                //peter read ac exist gpio, 0 is exist
      		       if(Capacity < 100)
         	  	 {
                        	if(ACStatus == 1)
                          	 {
			    		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
                                 }
                       		 else
                          	 {
			    		val->intval = POWER_SUPPLY_STATUS_CHARGING;
                          	 }
                   	}
                       else
                        {
                        	if(ACStatus == 1)
                         	 {
		            		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
                           	 }
                        	else
                           	 {
			    		val->intval = POWER_SUPPLY_STATUS_FULL;
                          	 }
                   	}
                     //msleep(100); //remove msleep by lenovo suggestion
                   }
                 else                    //peter no battery
                   {
		      val->intval = POWER_SUPPLY_STATUS_UNKNOWN;   
                   }
           }
        else if (psp == POWER_SUPPLY_PROP_TEMP)
               {
                    ret = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x0a);    
		    val->intval = ret - 2731;   //peter
                    //msleep(100); //remove msleep by lenovo suggestion

               }
        else if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW)
               {
                    ret = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x01);    
		    val->intval = ret * 1000;     //peter workaround *1000, ret origin is mV, but to upper level divide 1000, why ?
                    //msleep(100); //remove msleep by lenovo suggestion
               }
        
	return 0;
}

static int bq20z75_get_battery_capacity(union power_supply_propval *val)
{
	s32 ret;
        
        ret = i2c_smbus_read_word_data_retry(bq20z75_device->client,0x00);

	if (ret < 0) {
		dev_err(&bq20z75_device->client->dev,
			"%s: i2c read for %d failed\n", __func__, REG_CAPACITY);
		return -EINVAL;
	}
	/* bq20z75 spec says that this can be >100 %
	 * even if max value is 100 % */
	val->intval = ( (ret >= 100) ? 100 : ret);

        //msleep(100); //remove msleep by lenovo suggestion
	return 0;
}

//peter
static int bq20z75_get_ac_status(union power_supply_propval *val)
{
       int ACStatus;
       ACStatus = gpio_get_value(gpio);                                //peter read ac exist gpio, 0 is exist
       if(ACStatus)
          val->intval = 0;
       else
          val->intval = 1;
       return 0;
}

//peter
static int bq20z75_get_charge_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
       switch (psp) {
              case POWER_SUPPLY_PROP_ONLINE:
                  if (bq20z75_get_ac_status(val))
                      return -EINVAL;

                   break;
              default:
                        dev_err(&bq20z75_device->client->dev,
                                "%s: INVALID property\n", __func__);
                        return -EINVAL;
        }
       return 0;
}

static int bq20z75_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{

	u8 count;
	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_HEALTH:
//peter

			if (bq20z75_get_battery_presence_and_health(psp, val))
				return -EINVAL;

			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (bq20z75_get_battery_capacity(val))
				return -EINVAL;
                 
			break;

		case POWER_SUPPLY_PROP_STATUS:
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		case POWER_SUPPLY_PROP_TEMP:

			if (bq20z75_get_battery_property(count, psp, val))
				return -EINVAL;

			break;

		default:
			dev_err(&bq20z75_device->client->dev,
				"%s: INVALID property\n", __func__);
			return -EINVAL;
	}
	return 0;
}

//[ASD2-ES1 - Darren - 2011.3.16] Add EC detect function
//+++[ASD2-ES1 - Darren]
static int ec_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
         s16 id;
         //struct i2c_adapter *adapter = client->adapter;
         if(client->addr != 0x58){
                printk(KERN_ERR "client->addr = %d\n ec_i2c_detect check addr error\n", client->addr);
                 return -ENODEV;
         }
         id = i2c_smbus_read_word_data_retry(client, 0xBA);
         id = id >> 8;
         if (id != 0x5A){
                 printk(KERN_ERR "ec_i2c_detect command error\n");
                 return -ENODEV;
         }

         printk("ec_i2c_detect command pass = %d\n", id);

         return 0;
}
//---[ASD2-ES1 - Darren]

static int bq20z75_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc, err, i, ret;

        gpio = irq_to_gpio(client->irq);  //peter AC detect gpio
	bq20z75_device = kzalloc(sizeof(*bq20z75_device), GFP_KERNEL);
	if (!bq20z75_device) {
		return -ENOMEM;
                goto fail1;
	}
	memset(bq20z75_device, 0, sizeof(*bq20z75_device));

        ret = request_irq(client->irq, ac_interrupt, IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                 client->name, bq20z75_device);
        if( ret )
        {
                printk(KERN_ERR "egalax_i2c request irq(%d) failed\n", client->irq);
                goto fail2;
        }

	bq20z75_device->client = client;
	i2c_set_clientdata(client, bq20z75_device);

//[ASD2-ES1 - Darren - 2011.3.16] Add EC detect function
//+++[ASD2-ES1 - Darren]
         /* detect and init hardware */
         if ((ret = ec_i2c_detect(client, NULL)) < 0) {
                 printk(KERN_ERR "Initial i2c detect error !!!\n");
                 goto fail2;
         }
//---[ASD2-ES1 - Darren]

        //peter
        for (i = 0; i < ARRAY_SIZE(bq20z75_supply); i++)
            {
	         rc = power_supply_register(&client->dev, &bq20z75_supply[i]);
	         if (rc) {
	        	dev_err(&bq20z75_device->client->dev,
		        	"%s: Failed to register power supply\n", __func__);
	        	kfree(bq20z75_device);
		       return rc;
                        	}
            }
	dev_info(&bq20z75_device->client->dev,
		"%s: battery driver registered\n", client->name);

       //peter
        batt_status_poll_period = NVBATTERY_POLLING_INTERVAL;
        setup_timer(&poll_timer, tegra_battery_poll_timer_func, 0);
        mod_timer(&poll_timer,
                jiffies + msecs_to_jiffies(batt_status_poll_period));

//peter SYS_START
        Ecdebug_kobj = kobject_create_and_add("EcControl", NULL);
        if (Ecdebug_kobj == NULL) {
            printk("%s: subsystem_register failed\n", __FUNCTION__);
          }
        err = sysfs_create_group(Ecdebug_kobj, &attr_group);
        if(err) {
            printk("%s: sysfs_create_group failed, %d\n", __FUNCTION__, __LINE__);
          }

//peter SYS_END

    //+++[ASD2_ES1|Jaco_Gao|2011.03.09] for EC Tool       
    ec_dev_init(client);
    ec_dev_register();
    //---[ASD2_ES1|Jaco_Gao|2011.03.09] for EC Tool       

	return 0;

//peter
fail2:
        free_irq(client->irq, bq20z75_device);
fail1:
        i2c_set_clientdata(client, NULL);
        kfree(bq20z75_device);
        bq20z75_device = NULL;

	return ret;

}

static int bq20z75_remove(struct i2c_client *client)
{
	struct bq20z75_device_info *bq20z75_device = i2c_get_clientdata(client);
        int i;

        //peter
        del_timer_sync(&poll_timer);
        if(client->irq)
           {
                free_irq(client->irq, bq20z75_device);
           }

        for (i = 0; i < ARRAY_SIZE(bq20z75_supply); i++)
            {
	         power_supply_unregister(&bq20z75_supply[i]);
            }
	if (bq20z75_device) {
		kfree(bq20z75_device);
		bq20z75_device = NULL;
	}

    ec_dev_unregister();        //+++[ASD2_ES1|Jaco_Gao|2011.03.08] for EC Tool

	return 0;
}
//+++[ASD2-ES1 - Darren - 2011.4.4] [Modify] When suspend write 1 to 0x53
static int bq20z75_suspend(struct i2c_client *client,
	pm_message_t state)
{
	s32 ret;
	struct bq20z75_device_info *bq20z75_device = i2c_get_clientdata(client);

//peter
        del_timer_sync(&poll_timer);
        //ret = i2c_smbus_write_word_data_retry(bq20z75_device->client,0x53,0);
        ret = i2c_smbus_write_word_data_retry(bq20z75_device->client,0x53,1);
        if (ret < 0) {
                dev_err(&bq20z75_device->client->dev,
                        "%s: i2c write for %d failed\n",
                        __func__, MANUFACTURER_ACCESS_SLEEP);
                return -EINVAL;
        }
        
	return 0;
}
//---[ASD2-ES1 - Darren - 2011.4.4] [Modify] When suspend write 1 to 0x53


//+++[ASD2-ES1 - Darren - 2011.4.4] [Modify] When resume write 0 to 0x53
// any smbus transaction will wake up bq20z75 //
static int bq20z75_resume(struct i2c_client *client,
	pm_message_t state)
{
	s32 ret;
	struct bq20z75_device_info *bq20z75_device = i2c_get_clientdata(client);

	mod_timer(&poll_timer,
                jiffies + msecs_to_jiffies(3000));

        ret = i2c_smbus_write_word_data_retry(bq20z75_device->client,0x53,0);
        if (ret < 0) {
                dev_err(&bq20z75_device->client->dev,
                        "%s: i2c write for %d failed\n",
                        __func__, MANUFACTURER_ACCESS_SLEEP);
                return -EINVAL;
        }
     
	return 0;
}
//---[ASD2-ES1 - Darren - 2011.4.4] [Modify] When resume write 0 to 0x53

static const struct i2c_device_id bq20z75_id[] = {
	{ "bq20z75-battery", 0 },
	{},
};

static struct i2c_driver bq20z75_battery_driver = {
	.probe		= bq20z75_probe,
	.remove 	= bq20z75_remove,
	.suspend	= bq20z75_suspend,
	.resume 	= bq20z75_resume,
	.id_table	= bq20z75_id,
	.driver = {
		.name	= "bq20z75-battery",
	},
};

static int __init bq20z75_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq20z75_battery_driver);
	if (ret)
		dev_err(&bq20z75_device->client->dev,
			"%s: i2c_add_driver failed\n", __func__);

	return ret;
}
module_init(bq20z75_battery_init);

static void __exit bq20z75_battery_exit(void)
{
	i2c_del_driver(&bq20z75_battery_driver);
}
module_exit(bq20z75_battery_exit);

//i2c retry
static s32 i2c_smbus_read_word_data_retry(struct i2c_client *client, u8 command)
{

	int i, ret;

	for(i=0;i<=2;i++)
	{
		ret = i2c_smbus_read_word_data(client,command);

		if(ret >= 0)
		break;

		msleep(10);
		dev_err(&bq20z75_device->client->dev,
			"%s: i2c_smbus_read_word_data_retry failed, try again\n", __func__);
	}

	if(i == 3)
	{
		dev_err(&bq20z75_device->client->dev,
			"%s: i2c_smbus_read_word_data_retry failed\n", __func__);
		return -EINVAL;
	}

	return ret;

}

static s32 i2c_smbus_write_word_data_retry(struct i2c_client *client, u8 command, u16 value)
{
	int i, ret;

	for(i=0;i<=2;i++)
	{
		ret = i2c_smbus_write_word_data(client,command,value);

		if(ret == 0)
		break;

		msleep(10);
		dev_err(&bq20z75_device->client->dev,"%s: i2c_smbus_write_word_data_retry failed, try again\n", __func__);
	}

	if(i == 3)
	{
		dev_err(&bq20z75_device->client->dev,"%s: i2c_smbus_write_word_data_retry failed\n", __func__);
		return -EINVAL;
	}

	return ret;
}

MODULE_AUTHOR("NVIDIA Graphics Pvt Ltd");
MODULE_DESCRIPTION("BQ20z75 battery monitor driver");
MODULE_LICENSE("GPL");
