/* drivers/input/touchscreen/avago_ofn.c
 *
 * Copyright (C) 2010 Avago Technologies. 
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

/*
   1.add single tap and double tap function   
   2.update key value for spec

   by chenyig 2010/09/16

   1.update long press bug 
 
   by chenyig 2010/09/19

  
*/
#define DEBUG
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spi/spi.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include "avago_ofn.h"
#include <linux/io.h>
#include <asm/gpio.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>
#include <mach/pinmux.h>
#include "../../../arch/arm/mach-tegra/gpio-names.h"

static struct regulator *ldo5 = NULL;

/* AVAGO_RESET will driver NRST pin low then high to reset the chip */
//#define AVAGO_RESET
#define AVAGO_GPIO_CONTROL_SHUTDOWN     1
#define AVAGO_DOME                      1
#define AVAGO_FPD                       0

#define FPD_POLL_TIME	                70      // 70ms

#define THIRTEEN_MA		        0
#define FORTY_MA			    1
#define FINGER_ON_THRESHOLD		50              //12

#define I2C_M_WR				0
#define leswap(x) ( ((x << 8) & 0xff00) | ((x >> 8) & 0xff) )
#define MS_TO_NS(x) (x *1E6L)

#define EVENT_UP            KEY_F9
#define EVENT_DOWN          KEY_F10
#define EVENT_ONE_CLICK     KEY_HOME
#define EVENT_TWO_CLICK     KEY_F11
#define EVENT_LONG_PRESS    KEY_F12     //+++[ASD2_ES1|Jaco_Gao|2011.04.09] key-event for screenshot

#define EVENT_TEST_KEY_UP               KEY_A
#define EVENT_TEST_KEY_DOWN             KEY_B
#define EVENT_TEST_KEY_ONE_CLICK        KEY_C
#define EVENT_TEST_KEY_TWO_CLICK        KEY_E
#define EVENT_TEST_KEY_LONG_PRESS       KEY_D

#define OFN_GPIO_IN     1
#define OFN_GPIO_OUT    0
#define OFN_GPIO_HIGH   1
#define OFN_GPIO_LOW    0

//[ASD2-ES1|Soflen|2011.04.13] Test Mode
#define MODE_NORMAL     0
#define MODE_TEST       1

// ADBM_A320 Register Addresses
#define ADBM_A320_PRODUCTID_ADDR         0x83
#define ADBM_A320_REVISIONID_ADDR        0x01     //0x00
#define ADBM_A320_MOTION_ADDR            0x02
#define ADBM_A320_DELTAX_ADDR            0x03
#define ADBM_A320_DELTAY_ADDR            0x04
#define ADBM_A320_SQUAL_ADDR             0x05
#define ADBM_A320_SHUTTERUPPER_ADDR      0x06
#define ADBM_A320_SHUTTERLOWER_ADDR      0x07
#define ADBM_A320_MAXIMUMPIXEL_ADDR      0x08
#define ADBM_A320_PIXELSUM_ADDR          0x09
#define ADBM_A320_MINIMUMPIXEL_ADDR      0x0A
#define ADBM_A320_PIXELGRAB_ADDR         0x0B
#define ADBM_A320_CRC0_ADDR              0x0C
#define ADBM_A320_CRC1_ADDR              0x0D
#define ADBM_A320_CRC2_ADDR              0x0E
#define ADBM_A320_CRC3_ADDR              0x0F
#define ADBM_A320_SELFTEST_ADDR          0x10
#define ADBM_A320_CONFIGURATIONBITS_ADDR 0x11
#define ADBM_A320_LED_CONTROL_ADDR       0x1A
#define ADBM_A320_IO_MODE_ADDR           0x1C
#define ADBM_A320_OBSERVATION_ADDR       0x2E
#define ADBM_A320_SOFTRESET_ADDR      	 0x3A     //0x5A
#define ADBM_A320_SHUTTER_MAX_HI_ADDR    0x3B
#define ADBM_A320_SHUTTER_MAX_LO_ADDR    0x3C
#define ADBM_A320_INVERSEREVISIONID_ADDR 0x3E     //0xFF
#define ADBM_A320_INVERSEPRODUCTID_ADDR  0x3F     //0x7C

#define S_IRUG  (S_IRUSR|S_IRGRP)   
#define S_IWUG  (S_IWUSR|S_IWGRP)

#define ENABLE_IRQ      1
#define DISABLE_IRQ     0
#define ENABLE_INIT     1
#define DISABLE_INIT    0

typedef union
{
	uint16_t w;
	uint8_t b[2];
}
union16;

static struct workqueue_struct *avago_ofn_wq;

int finger_leave_counter;
int Zero_count;
int invalid_count;
int ofn_boot = 1;

static int i;
int ofn_init_finish = 0; //Charlie add flag to avoid hang up 2012.2.17
struct avago_ofn_platform_data {
	uint32_t version;
	int (*power)(int on);
};

struct avago_fpd_data {
	uint8_t motion2;
	uint8_t finger_on_counter;
	uint8_t finger_leave_counter;
        uint8_t Zero_count;
	uint8_t FPD_turned_off;
	uint8_t shutter_values_captured;
	uint8_t last_state;
	uint16_t shutter_at_40ma;
	uint16_t shutter_at_13ma;
	uint8_t switch_led_counter;
};

struct avago_ofn_data {
	struct spi_device *spi;
	struct input_dev *input_dev;
	int use_irq;
	uint8_t irq;
	struct hrtimer timer;
	struct hrtimer int_timer;
	struct hrtimer dome_timer;
	struct hrtimer fpd_timer;
    struct hrtimer up_timer;
    struct hrtimer dn_timer;
	struct work_struct  work;
	struct work_struct  dome;
	int shutdown_gpio;
	int dome_gpio;
	int irq_gpio;
	int up_led_gpio;
	int dn_led_gpio;
	uint16_t dome_count;
	int motion_x;
	int motion_y;
	int motion_count;
	int key_pressed;
	int detect_delay;
	uint8_t fpd_support;
	struct avago_fpd_data fpd;
	int int_timer_flag;
	
	int (*power)(int on);
        int flag2; 
	struct early_suspend early_suspend;
//++ Charles
	int is_reg_en;
//--
    //+++[ASD2_ES1|Jaco_Gao|2011.04.17] Flash LED via timer
    struct timer_list led_timer;
    enum {
        POWER_NONE = 0,
        POWER_ON,
        POWER_OFF,
    } led_action;
    int led_gpio_1;
    int led_gpio_2;
    unsigned long led_power_on_to_off_interval;             // unit in ms
    unsigned long led_power_off_to_on_interval;
    int led_multi;
    //---[ASD2_ES1|Jaco_Gao|2011.04.17] Flash LED via timer
};

unsigned int x,y,tempx,tempy;               //by chenyig 2010/10/20
#define fpd_click   0                       //add tap function by chenyig

static unsigned char avago_keycode[] = {
    EVENT_UP,
    EVENT_DOWN,
    EVENT_ONE_CLICK,
    EVENT_TWO_CLICK,
    EVENT_LONG_PRESS,
    EVENT_TEST_KEY_UP,
    EVENT_TEST_KEY_DOWN,
    EVENT_TEST_KEY_ONE_CLICK,
    EVENT_TEST_KEY_TWO_CLICK,
    EVENT_TEST_KEY_LONG_PRESS,
};	

enum OFN_SPI_GPIO_STATE {
    OFN_SPI_GPIO_LOW = 0,
    OFN_SPI_GPIO_HIGH = 1,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void avago_ofn_early_suspend(struct early_suspend *h);
static void avago_ofn_late_resume(struct early_suspend *h);
#endif

inline void ofn_soft_reset(struct avago_ofn_data *ts);
inline void ofn_reset( struct avago_ofn_data *tspi);        //[ASD2_ES1|Jaco_Gao|2011.05.19] check ofn status periodically
void ofn_drive_spi( enum OFN_SPI_GPIO_STATE hi, int request, int free );
static uint8_t avago_read_register_byte(struct spi_device *spi, u8 reg);
static int avago_write_register_byte(struct spi_device *spi, u8 reg, u8 val);
static int avago_init_ofn(struct avago_ofn_data *ts, int reset);
void ofn_hw_reset( int en_init );

//+++ sysfs
u8 ofn_reg = 0;
static struct avago_ofn_data *g_ts;
static int att_mode = MODE_NORMAL;

static ssize_t ofn_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	ssize_t ret = 0;

	if(att_mode == MODE_NORMAL)
		sprintf(buf, "Current mode is normal mode.\n");
	else if (att_mode == MODE_TEST)
		sprintf(buf, "Current mode is test mode.\n");
		
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t ofn_mode_store(struct kobject *kobj,
            struct kobj_attribute *attr, const char * buf, size_t count)
{
	char data[128];
	sscanf(buf, "%s", data);
	if(strcmp(data, "test") == 0)
	{
        att_mode = MODE_TEST;
		printk(KERN_ERR "enter test mode\n");
	}
	if(strcmp(data, "normal") == 0)
	{
		att_mode = MODE_NORMAL;
		printk(KERN_ERR "enter normal mode\n");
	}

	return count;
}

static ssize_t ofn_reg_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	int ret = 0;
	sprintf( buf, "OFN Reg. 0x%02x = 0x%02x\n", ofn_reg, avago_read_register_byte(g_ts->spi, ofn_reg) );
	ret = strlen(buf)+1;
	return ret;
}

static ssize_t ofn_reg_store(struct kobject *kobj,
            struct kobj_attribute *attr, const char * buf, size_t count)
{
	sscanf(buf, "%d", (int*)&ofn_reg);
	pr_debug("Set ofn_reg = 0x%02x", ofn_reg);
	return count;
}

static ssize_t ofn_val_store(struct kobject *kobj,
            struct kobj_attribute *attr, const char * buf, size_t count)
{
    u8 val = 0;
    sscanf(buf, "%d", (int*)&val);
    avago_write_register_byte(g_ts->spi, ofn_reg, val);
    printk("OFN : write 0x%02x to Reg. 0x%02x\n", val, ofn_reg);
	return count;
}

static ssize_t ofn_reg_table_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    int ret = 0;
    int i = 0;
    for( i = 0; i<= 0x77; i++)
        printk("Reg. 0x%02x = 0x%02x\n", i, avago_read_register_byte(g_ts->spi, i));

    return ret;
}

static ssize_t ofn_reset_store(struct kobject *kobj,
            struct kobj_attribute *attr, const char * buf, size_t count)
{
    struct avago_ofn_data *ts = g_ts;

    switch( *buf )
    {
    case '0':
        pr_info("Performing OFN soft reset...\n");
        ofn_reset(ts);
        pr_info("Done.\n");
    break;
    case '1':
        pr_info("Performing OFN hard reset...\n");
        ofn_hw_reset(ENABLE_INIT);
        pr_info("Done.\n");
    break;
    default:
        return -EINVAL;
    }

	return count;
}

static ssize_t ofn_status_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    int ret;
    sprintf( buf, "OFN status = 0x%02x\n", avago_read_register_byte(g_ts->spi, 2) );
    ret = strlen(buf) + 1;
    return ret;
}

static ssize_t ofn_led_up_store(struct kobject *kobj,
            struct kobj_attribute *attr, const char * buf, size_t count)
{
    int val;
	sscanf(buf, "%d", &val);
	if( val == 1 )
        gpio_set_value( AVAGO_LED_UP, 1);
    else
        gpio_set_value( AVAGO_LED_UP, 0);

	return count;
}

static ssize_t ofn_led_down_store(struct kobject *kobj,
            struct kobj_attribute *attr, const char * buf, size_t count)
{
    int val;
	sscanf(buf, "%d", &val);
	if( val == 1 )
        gpio_set_value( AVAGO_LED_DOWN, 1);
    else
        gpio_set_value( AVAGO_LED_DOWN, 0);

	return count;
}

#define spi_attr(__name, __mode, __show, __store)   \
    struct kobj_attribute dev_attr_##__name = __ATTR(__name, __mode, __show, __store)

static spi_attr(led_up, S_IWUG , NULL, ofn_led_up_store);
static spi_attr(led_down, S_IWUG , NULL, ofn_led_down_store);
static spi_attr(status, S_IRUG , ofn_status_show, NULL);
static spi_attr(reset, S_IWUG , NULL, ofn_reset_store);
static spi_attr(reg_table, S_IRUG , ofn_reg_table_show, NULL);
static spi_attr(mode, S_IRUG|S_IWUG, ofn_mode_show, ofn_mode_store);
static spi_attr(val, S_IWUG , NULL, ofn_val_store);
static spi_attr(reg, S_IWUG|S_IRUG, ofn_reg_show, ofn_reg_store);

static struct attribute *attributes[] = {
    &dev_attr_mode.attr,
    &dev_attr_status.attr,
    &dev_attr_reset.attr,
    &dev_attr_reg.attr,
    &dev_attr_val.attr,
    &dev_attr_reg_table.attr,
    &dev_attr_led_up.attr,
    &dev_attr_led_down.attr,
    NULL,
};

static struct attribute_group attribute_group = {
    .attrs = attributes,
};

static struct kobject *ofn_kobj;
static struct kobject* ofn_sysfs_init(const char* name)
{
    int ret = 0;
    struct kobject *kobj = NULL;
	kobj = kobject_create_and_add(name, NULL);
	if (kobj == NULL) {
        pr_err("%s : sysfs subsystem_register failed\n", name);
		goto err_create_kobj;
	}

    ret = sysfs_create_group(kobj, &attribute_group);
	if (ret) {
        pr_err("%s : sysfs create group failed\n", name);
		goto err_create_group;
	}
	return kobj;

err_create_group:
	kobject_del(kobj);
err_create_kobj:
	return NULL;
}
//--- sysfs

void ofn_drive_spi( enum OFN_SPI_GPIO_STATE hi, int request, int free )
{
    int spi_gpio[4] = {
        TEGRA_GPIO_PX4,
        TEGRA_GPIO_PX5,
        TEGRA_GPIO_PX6,
        TEGRA_GPIO_PX7,
    };
    int i;
    if( request )
    {
        for ( i = 0; i < 4; i++)
        {
            tegra_gpio_enable(spi_gpio[i]);
            gpio_request( spi_gpio[i], "gpio");
        }
    }

    for( i = 0; i < 4; i++){
        gpio_direction_output(spi_gpio[i], hi);
    }
    if( free )
    {
        for ( i = 0; i < 4; i++)
        {
            gpio_free(spi_gpio[i]);
            tegra_gpio_disable(spi_gpio[i]);
        }
    }
}

#define T_SRR	500	//ns
#define T_SRW	500	//ns

static uint8_t avago_read_register_byte(struct spi_device *spi, u8 reg)
{
    struct spi_message           m;
    struct spi_transfer          t;
    int                          i;

    u8                           tx_buf[8];
    u8                           rx_buf[8];
    int                          rc;

    memset(&t, 0, sizeof t);
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    t.tx_buf = tx_buf;
    t.rx_buf = rx_buf;

    tx_buf[0] = reg;
    for (i = 1; i < 8; i++) tx_buf[i] = 0x00;
    t.len = 2;
    rc = spi_sync(spi, &m);
    ndelay(T_SRR);
    if (rc)
        goto dbfs_read_exit;

    return (rx_buf[1]);

dbfs_read_exit:
    pr_err("%s read reg[%x] error!\n", __func__, reg);
    return (0);
}

#define T_SWR	20	//us
#define T_SWW	30	//us

static int avago_write_register_byte(struct spi_device *spi, u8 reg, u8 val)
{
    struct spi_message           m;
    struct spi_transfer          t;
    int                          rc;

    u8					tx_buf[8];

    memset(&t, 0, sizeof t);
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    t.tx_buf = tx_buf;
    t.rx_buf = NULL;

    tx_buf[0] = reg | 0x80;
    tx_buf[1] = val;
    t.len = 2;
    rc = spi_sync(spi, &m);
    udelay(T_SWW);
    if (rc)
        pr_err("%s write reg[%x] error!\n", __func__, reg);

    return rc;
}

//+++[ASD2_ES1|Jaco_Gao|2011.04.18] Flash led by timer
void inline avago_flash_led_up( struct avago_ofn_data *ts,
    unsigned long on_to_off,
    unsigned long off_to_on)
{
    if( timer_pending( &ts->led_timer ) )
        return;
    ts->led_power_on_to_off_interval = on_to_off;
    ts->led_power_off_to_on_interval = off_to_on;
    ts->led_gpio_1 = AVAGO_LED_UP;
    ts->led_gpio_2 = 0;
    ts->led_action = POWER_OFF;
    ts->led_multi = 0;
    gpio_set_value(ts->led_gpio_1, 1);
    mod_timer(&ts->led_timer, jiffies+ts->led_power_on_to_off_interval*HZ/1000);
}
void inline avago_flash_led_down( struct avago_ofn_data *ts,
    unsigned long on_to_off,
    unsigned long off_to_on)
{
    if( timer_pending( &ts->led_timer ) )
        return;
    ts->led_power_on_to_off_interval = on_to_off;
    ts->led_power_off_to_on_interval = off_to_on;
    ts->led_gpio_1 = AVAGO_LED_DOWN;
    ts->led_gpio_2 = 0;
    ts->led_action = POWER_OFF;
    ts->led_multi = 0;
    gpio_set_value(ts->led_gpio_1, 1);
    mod_timer(&ts->led_timer, jiffies+ts->led_power_on_to_off_interval*HZ/1000);
}

void inline avago_flash_led( struct avago_ofn_data *ts,
    unsigned long on_to_off,
    unsigned long off_to_on)
{
    if( timer_pending( &ts->led_timer ) )
        return;
    ts->led_power_on_to_off_interval = on_to_off;
    ts->led_power_off_to_on_interval = off_to_on;
    ts->led_gpio_1 = AVAGO_LED_UP;
    ts->led_gpio_2 = AVAGO_LED_DOWN;
    ts->led_action = POWER_OFF;
    ts->led_multi = 0;
    gpio_set_value(ts->led_gpio_1, 1);
    gpio_set_value(ts->led_gpio_2, 1);
    mod_timer(&ts->led_timer, jiffies+ts->led_power_on_to_off_interval*HZ/1000);
}

void inline avago_flash_led_multi( struct avago_ofn_data *ts,
    unsigned long on_to_off,
    unsigned long off_to_on,
    int times)
{
    if( timer_pending( &ts->led_timer ) )
        return;
    ts->led_power_on_to_off_interval = on_to_off;
    ts->led_power_off_to_on_interval = off_to_on;
    ts->led_gpio_1 = AVAGO_LED_UP;
    ts->led_gpio_2 = AVAGO_LED_DOWN;
    ts->led_action = POWER_OFF;
    ts->led_multi = (times << 2);
    gpio_set_value(ts->led_gpio_1, 1);
    gpio_set_value(ts->led_gpio_2, 1);
    mod_timer(&ts->led_timer, jiffies+ts->led_power_on_to_off_interval*HZ/1000);
}
//---[ASD2_ES1|Jaco_Gao|2011.04.18] Flash led by timer

void inline avago_ofn_report_key(struct avago_ofn_data *pdata, u8 key)
{
    input_report_key(pdata->input_dev, key, 1);
    input_sync(pdata->input_dev);
    input_report_key(pdata->input_dev, key, 0);
    input_sync(pdata->input_dev);
}

void inline avago_ofn_report_key_val(struct avago_ofn_data *pdata, u8 key, int val)
{
    input_report_key(pdata->input_dev, key, val);
    input_sync(pdata->input_dev);
}

void inline avago_ofn_report_key_val_pressed(struct avago_ofn_data *pdata, u8 key, int val)
{
    avago_ofn_report_key_val(pdata, key, val);
    pdata->key_pressed = ((val == 1)? key : 0 );
}

#define ENABLE_RESET    1
#define DISABLE_RESET   0
static int avago_init_ofn(struct avago_ofn_data *ts, int reset)
{
    int ret = 0;

    /* Perform soft reset by writing 0x5A to address 0x3A */
    //	avago_write_register_byte(ts->spi, 0x3A, 0x5A);
    //	msleep(100); 	
    /* init as datasheet required 
       Write 0xE4 to address 0x60.
       Set Speed Switching, write 0x62 with 0x12, 0x63 with 0x0E, 0x64 with 0x08, 0x65 with 0x06, 0x66 with 0x40, 0x67 with 0x08, 
       0x68 with 0x48, 0x69 with 0x0A, 0x6A with 0x50, 0x6B with 0x48.
       Set Assert/De-assert, write 0x6D with 0xC4, 0x6E with 0x34, 0x6F with 0x3C, 0x70 with 0x18, 0x71 with 0x20.
       Set Finger Presence Detection threshold, write 0x75 with 0x50.
       IF XY Quantization is used, then write 0x73 with 0x99 and 0x74 with 0x02.
       Write 0x10 to register 0x1C. This will activate Burst mode. If Burst mode not used then skip this step.
       Read from registers 0x02, 0x03 and 0x04 (or read these same 3 bytes with Burst mode) one time
     */

    // Step 6 - Software Reset
    //    avago_write_register_byte(ts->spi, 0x3A, 0x5A);
    //	msleep(100); 

    //	avago_write_register_byte(ts->spi, 0x60, 0xE4);      
    if( reset )
        ofn_soft_reset(ts);
    avago_write_register_byte(ts->spi, 0x60, 0xB4); 	/* enable quatitation */

    avago_write_register_byte(ts->spi, 0x62, 0x12);
    avago_write_register_byte(ts->spi, 0x63, 0x0E);
    avago_write_register_byte(ts->spi, 0x64, 0x08);
    avago_write_register_byte(ts->spi, 0x65, 0x06);
    avago_write_register_byte(ts->spi, 0x66, 0x40);
    avago_write_register_byte(ts->spi, 0x67, 0x08);
    avago_write_register_byte(ts->spi, 0x68, 0x48);
    avago_write_register_byte(ts->spi, 0x69, 0x0A);
    avago_write_register_byte(ts->spi, 0x6A, 0x50);
    avago_write_register_byte(ts->spi, 0x6B, 0x48);

    avago_write_register_byte(ts->spi, 0x6D, 0xC4);

    avago_write_register_byte(ts->spi, 0x6E, 0x2F);
    avago_write_register_byte(ts->spi, 0x6F, 0x3C);
    avago_write_register_byte(ts->spi, 0x70, 0x2F);
    avago_write_register_byte(ts->spi, 0x71, 0x3C);

    avago_write_register_byte(ts->spi, 0x75, 0x50);

    avago_write_register_byte(ts->spi, 0x73, 0x99);
    avago_write_register_byte(ts->spi, 0x74, 0x02);

    avago_read_register_byte(ts->spi, 0x2);
    avago_read_register_byte(ts->spi, 0x3);
    avago_read_register_byte(ts->spi, 0x4);

    ts->motion_count = 0;
    ts->motion_x = 0;
    ts->motion_y = 0;
    ts->key_pressed = 0;

    return ret;
}

int avago_init_ofn_dont_reset( void )
{
    return avago_init_ofn(g_ts, 0);
}

static int init_fpd(struct avago_ofn_data *ts)
{
    struct avago_fpd_data *fpd;

    fpd = &ts->fpd;

    fpd->motion2 = 0;
    fpd->finger_on_counter = 0;
    fpd->finger_leave_counter = 0;
    fpd->FPD_turned_off = 0;
    fpd->shutter_values_captured = 0;
    fpd->last_state = THIRTEEN_MA;
    fpd->shutter_at_40ma = 0;
    fpd->shutter_at_13ma = 0;
    fpd->switch_led_counter = 0;

    return 0;
}

static enum hrtimer_restart avago_ts_int_timer_func(struct hrtimer *timer)
{
    struct avago_ofn_data *ts = container_of(timer, \
	    struct avago_ofn_data, timer);

    queue_work(avago_ofn_wq, &ts->work);

    return HRTIMER_NORESTART;
}

static enum hrtimer_restart avago_up_led_func(struct hrtimer *timer)
{
    struct avago_ofn_data *ts = container_of(timer, \
            struct avago_ofn_data, up_timer);
    avago_flash_led_up(ts, 80, 0);

    return HRTIMER_NORESTART;
}

static enum hrtimer_restart avago_dn_led_func(struct hrtimer *timer)
{
    struct avago_ofn_data *ts = container_of(timer, \
            struct avago_ofn_data, dn_timer);
    avago_flash_led_down(ts, 80, 0);

    return HRTIMER_NORESTART;
}

static int avago_int_timer(struct avago_ofn_data *ts)
{
    uint8_t status;
    int e,f;
    int temp;
	
    uint8_t pix_min, pix_max, squal;
    union16 shutter1; 
    //Charlie add flag to avoid hang up 2012.2.17
    if (ofn_init_finish==0)
	return HRTIMER_NORESTART;
    status = avago_read_register_byte(ts->spi,2); 

    if( (status & 0x80) || (status & 0x01) )
    {
        if (avago_read_register_byte(ts->spi, 0x00) != AVAGO_OFN_ID)
        {
            enable_irq(TEGRA_GPIO_TO_IRQ(ts->irq));
            return HRTIMER_NORESTART;
        }
		
	    squal   = avago_read_register_byte(ts->spi, ADBM_A320_SQUAL_ADDR);
	    pix_min = avago_read_register_byte(ts->spi, ADBM_A320_MINIMUMPIXEL_ADDR);
	    pix_max = avago_read_register_byte(ts->spi, ADBM_A320_MAXIMUMPIXEL_ADDR);
	    shutter1.b[1] = avago_read_register_byte(ts->spi, ADBM_A320_SHUTTERUPPER_ADDR);
	    shutter1.b[0] = avago_read_register_byte(ts->spi, ADBM_A320_SHUTTERLOWER_ADDR);
        temp = abs(pix_max-pix_min);
	    // shutter1.b[0] is upper byte of a 16 bit unsigned integer
	    // shutter1.w is the combined 16 bit unsigned integer.

	    // A comparison is done here to check is squal value is low (means finger is off),
	    // If pix_max - pix_min is high, it tends to mean a strong external light source (point source) 
	    // is present.
	    // if shutter is low, it tends to mean an external light is shining directly into the sensor, 
	    // and light reflected from sensor's LED cannot produce so might light.d
    
	    if ( (abs(pix_max - pix_min) > 230) || (squal < 15) || (shutter1.w < 25) )
	    {	
            finger_leave_counter++;
            e = avago_read_register_byte(ts->spi, 3); // Register: Delta_X
            f = avago_read_register_byte(ts->spi, 4); // Register: Delta_Y

            if(finger_leave_counter >=13)    //by strong light
            {
                goto fpd;
            }
	    }
	    else
	    {
            x = avago_read_register_byte(ts->spi, 3);
            y = avago_read_register_byte(ts->spi, 4); // Register: Delta_Y

            if(y != 0)
            {
                /* expand to 12bit */
                if(y & 0x80)
                    y |= 0x0f00;
                ts->motion_y += y;
                Zero_count = 0;
            }
            else
            {
                Zero_count++;
			    if(Zero_count >= 10)             //by sun lighti
                    goto fpd;
            }

			// don't handle overflow here
            //expand to 12bit */

            if(ts->motion_y & 0x800)
                tempy = (0x1000 - (ts->motion_y & 0xfff)) & 0xfff;
            else
                tempy = ts->motion_y & 0xfff;
        }
    }
    else
    {
        invalid_count++;

        if(invalid_count >= 8 )
        {
            invalid_count = 0;
            enable_irq(TEGRA_GPIO_TO_IRQ(ts->irq));
            return HRTIMER_NORESTART;
        }
    }  

fpd:
    status = avago_read_register_byte(ts->spi, 2); // Register: Motion 
	
    if (tempy >= AVAGO_MOTION_SCALE)
    {
        if(ts->motion_y & 0x800)//chenyig
        {
            if( ((status & 0x01) == 0 ) || (finger_leave_counter >= 13) || (Zero_count  >= 10) )
            {
                if (att_mode == MODE_TEST)
                    avago_ofn_report_key_val_pressed(ts, EVENT_TEST_KEY_UP, 1);
                else
                    avago_ofn_report_key_val_pressed(ts, EVENT_UP, 1);
                
                hrtimer_start(&ts->up_timer, ktime_set(0, MS_TO_NS(2)), HRTIMER_MODE_REL);

                avago_ofn_report_key_val_pressed(ts, ts->key_pressed, 0);

                e = avago_read_register_byte(ts->spi, 3); // Register: Delta_X
                f = avago_read_register_byte(ts->spi, 4); // Register: Delta_Y

                pr_debug("%s: EVENT_UP %x\n", __FUNCTION__, ts->key_pressed);

                ts->motion_x = 0;
                ts->motion_y = 0;
                finger_leave_counter = 0;
                Zero_count = 0;
                enable_irq(TEGRA_GPIO_TO_IRQ(ts->irq));
                return HRTIMER_NORESTART;
			}
		}
		else
		{
            if( ((status & 0x01) == 0 ) || (finger_leave_counter >= 13) || (Zero_count  >= 10) )
            {  		
                if (att_mode == MODE_TEST)
                    avago_ofn_report_key_val_pressed(ts, EVENT_TEST_KEY_DOWN, 1);
                else
                    avago_ofn_report_key_val_pressed(ts, EVENT_DOWN, 1);

                hrtimer_start(&ts->dn_timer, ktime_set(0, MS_TO_NS(2)), HRTIMER_MODE_REL);

                avago_ofn_report_key_val_pressed(ts, ts->key_pressed, 0);

                e = avago_read_register_byte(ts->spi, 3); // Register: Delta_X
                f = avago_read_register_byte(ts->spi, 4); // Register: Delta_Y

                pr_debug("%s: EVENT_DOWN %x\n", __FUNCTION__, ts->key_pressed);

				ts->motion_x = 0;
                ts->motion_y = 0;
                finger_leave_counter = 0;
				Zero_count = 0;
                enable_irq(TEGRA_GPIO_TO_IRQ(ts->irq));           
				return HRTIMER_NORESTART;
            }
        }
    } 
    else
    {
        if( (finger_leave_counter >= 13) || (Zero_count >= 10) )
        {
            ts->motion_x = 0;
            ts->motion_y = 0;
            finger_leave_counter = 0;
            Zero_count = 0;
            enable_irq(TEGRA_GPIO_TO_IRQ(ts->irq));
            return HRTIMER_NORESTART;                    
        }
    }

    /* 8ms timer */
    hrtimer_start(&ts->timer, ktime_set(0, MS_TO_NS(8)), HRTIMER_MODE_REL);

    return 0;
}

int avago_timer(struct avago_ofn_data *ts)
{
    char gpio_value;

    gpio_value = gpio_get_value(ts->dome_gpio);

    pr_debug("%s: hrtimer start again\n", __FUNCTION__);

    //turn off down and up led
    gpio_set_value(ts->dn_led_gpio, 0);            
    gpio_set_value(ts->up_led_gpio, 0);

    /* 100ms timer */
    pr_debug("hrtimer start again 1\n");

    return 1;
}


static void avago_ofn_work_func(struct work_struct *work)
{
    struct avago_ofn_data *ts = container_of(work, struct avago_ofn_data, work);
    avago_int_timer(ts);
}

static enum hrtimer_restart avago_ts_timer_func(struct hrtimer *timer)
{
    struct avago_ofn_data *ts = container_of(timer, \
	    struct avago_ofn_data, timer);
    // when touch ofn, irq_gpio is low, set timer to 8ms for detect
    // when no touch, irq_gpio is high, set timer to 50 ms for idle

	if( ts->detect_delay <= 300 ){
	    queue_work(avago_ofn_wq, &ts->work);
	    // 8ms timer
	    hrtimer_start(&ts->timer, ktime_set(0, MS_TO_NS(40)), HRTIMER_MODE_REL);
	    ts->detect_delay++;
	}
	else if  ( (gpio_get_value(ts->irq_gpio) == 0) ) { 
	    ts->detect_delay =0;
	    hrtimer_start(&ts->timer, ktime_set(0, MS_TO_NS(40)), HRTIMER_MODE_REL);
	}
	else{
	    hrtimer_start(&ts->timer, ktime_set(0, MS_TO_NS(5)), HRTIMER_MODE_REL);
	    ts->detect_delay = 300;
	}

    #ifdef AVAGO_DOME
    // handle dome(single click/ long press)
    if(ts->dome_count == 0)
    {
		if(!gpio_get_value(ts->dome_gpio))
		{
			ts->dome_count ++;
            // start timer, now is 5ms, adjust it for debounce
            hrtimer_start(&ts->dome_timer, ktime_set(0, MS_TO_NS(5)), HRTIMER_MODE_REL);
		}
    }
    #endif // AVAGO_DOME

    return HRTIMER_NORESTART;
}


static enum hrtimer_restart avago_dome_timer_func(struct hrtimer *timer)
{
    int gpio_value;
    struct avago_ofn_data *ts = container_of(timer, struct avago_ofn_data, dome_timer);
    //Charlie add flag to avoid hang up 2012.2.17
    if (ofn_init_finish==0)
        return HRTIMER_NORESTART;

    gpio_value = gpio_get_value(ts->dome_gpio);
	
    //turn of down and up led
    gpio_set_value(ts->dn_led_gpio, 0);            
    gpio_set_value(ts->up_led_gpio, 0);
 
    if(gpio_value)
    {
		// dome up, send report to upper layer
		if(ts->dome_count == 1)
		{
			// the first timer is 5ms to detect debouncing
			ts->dome_count = 0;
			return HRTIMER_NORESTART;
		}
        
		if( ts->dome_count < 10 ) //6
		{
            pr_debug("OFN : one click\n");
            if (att_mode == MODE_TEST)
                avago_ofn_report_key(ts, EVENT_TEST_KEY_ONE_CLICK);
            else
                avago_ofn_report_key(ts, EVENT_ONE_CLICK);

            avago_flash_led(ts, 80, 0);
			ts->dome_count = 0;
			return HRTIMER_NORESTART;
		}
	}
	else
	{
		// dome down, increase the counter
		if(ts->dome_count < 0x80)	/* set the max value to avoid overflow */
		{
			ts->dome_count++;		
		}
	
		if( ts->dome_count >= 10 ) 
		{
            pr_debug("OFN : long press\n");
            if (att_mode == MODE_TEST)
                avago_ofn_report_key(ts, EVENT_TEST_KEY_LONG_PRESS);
            else
                avago_ofn_report_key(ts, EVENT_LONG_PRESS);

            avago_flash_led(ts, 80, 0);
			ts->dome_count = 0;			
			return HRTIMER_NORESTART;
		}
	}

    /* 100ms timer */
    hrtimer_start(&ts->dome_timer, ktime_set(0, MS_TO_NS(100)), HRTIMER_MODE_REL);

    return HRTIMER_NORESTART;
}

static irqreturn_t avago_ts_irq_handler(int irq, void *dev_id)
{
    struct avago_ofn_data *ts = dev_id;

    if(!gpio_get_value(ts->irq))
    {
        disable_irq_nosync(TEGRA_GPIO_TO_IRQ(ts->irq));

        ts->motion_count = 0;
        ts->motion_x = 0;
        ts->motion_y = 0;
        tempy = 0;
        Zero_count = 0;
        finger_leave_counter = 0;
        invalid_count = 0 ;
        i = 0;
	
        queue_work(avago_ofn_wq, &ts->work);	
    }

    return IRQ_HANDLED;
}

static irqreturn_t avago_dome_irq_handler(int irq, void *dev_id)
{
	struct avago_ofn_data *ts = dev_id;
    //Charlie add flag to avoid hang up 2012.2.17
        if (ofn_init_finish==0)
            return IRQ_HANDLED;
    if(ts->dome_count == 0)
    {
		if(!gpio_get_value(ts->dome_gpio))
		{
			ts->dome_count ++;
            // start timer, now is 5ms, adjust it for debounce 
            hrtimer_start(&ts->dome_timer, ktime_set(0, MS_TO_NS(10)), HRTIMER_MODE_REL);
		}
    }
	return IRQ_HANDLED;
}

//+++[ASD2_ES1|Jaco_Gao|2011.04.17] Flash LED via timer
static void avago_led_timer_func(unsigned long arg)
{
    struct avago_ofn_data *ts = (struct avago_ofn_data *)arg;

    switch( ts->led_action )
    {
    case POWER_ON:
        if( ts->led_gpio_1 > 0 )
            gpio_set_value(ts->led_gpio_1, 1);
        if( ts->led_gpio_2 > 0 )
            gpio_set_value(ts->led_gpio_2, 1);
    break;
    case POWER_OFF:
        if( ts->led_gpio_1 > 0 )
            gpio_set_value(ts->led_gpio_1, 0);
        if( ts->led_gpio_2 > 0 )
            gpio_set_value(ts->led_gpio_2, 0);
    break;
    default:
    break;
    }
    if ( ts->led_multi-- > 0 )
    {
        // set next action
        ts->led_action = ( (ts->led_action == POWER_ON)? POWER_OFF : POWER_ON );
        // set up next timer
        mod_timer( &ts->led_timer,
                    ((ts->led_action == POWER_OFF )? ts->led_power_on_to_off_interval : ts->led_power_off_to_on_interval));
        //add_timer( &ts->led_timer);
    }
    else
    {
        // clear timer parameter
        ts->led_action = POWER_NONE;
        ts->led_gpio_1 = 0;
        ts->led_gpio_2 = 0;
    }
}

void avago_ofn_init_led_timer( struct avago_ofn_data *ts )
{
    init_timer(&ts->led_timer);
	ts->led_timer.data = (unsigned long) ts;
	ts->led_timer.function = avago_led_timer_func;

    ts->led_gpio_1 = 0;
    ts->led_gpio_2 = 0;
    ts->led_action = POWER_NONE;
}
//---[ASD2_ES1|Jaco_Gao|2011.04.17] Flash LED via timer

//+++[ASD2_ES1|Jaco_Gao|2011.04.08] re-write GPIO init function
// description :
// in : OFN_GPIO_IN , OFN_GPIO_OUT
// stat : OFN_GPIO_HIGH , OFN_GPIO_LOW
// gpio_in_ts : record gpio declaration in ts, can be NULL
// ( NULL means not to store )
int inline avago_init_gpio( struct avago_ofn_data *ts, int gpio, const char* name, int in, int stat, int *gpio_in_ts)
{
    int ret;
    
	ret = gpio_request(gpio, name);
	if (ret < 0) {
		printk("%s : failed to request GPIO %d , %s\n", __func__, gpio, name);
        if( gpio_in_ts )
            *gpio_in_ts = 0;
        return ret;
	}
    pr_debug("%s : request GPIO %s\n", __func__, name);
	ret = ((in == OFN_GPIO_IN )? gpio_direction_input(gpio) : gpio_direction_output(gpio, stat));
	if (ret < 0) {
        printk("%s: fail to direct GPIO %d , %s, to %s\n", __func__,
                gpio, name, ((in==OFN_GPIO_IN)?"input":"output"));
		gpio_free(gpio);
        if( gpio_in_ts )
            *gpio_in_ts = 0;
        return ret;
	}
    if( gpio_in_ts )
        *gpio_in_ts = gpio;

    return 0;
}
//---[ASD2_ES1|Jaco_Gao|2011.04.08] re-write GPIO init function

#define REPEAT_TIME_TO_CHK_ID   5
#define AVAGO_PRODUCT_ID        0x83
#define OFN_ERR                 -1
#define OFN_GOOD                0

int ofn_chk_id( struct avago_ofn_data *ts )
{
		//compal_start willy retry ofn id
        int times = 0;
        int pass = 0;
        int ret = 0;
        u8 u8_ret;
        while (times++ < 5) {
            u8_ret = avago_read_register_byte(ts->spi, 0x00);
            ret = (int)u8_ret;

            if (ret == AVAGO_OFN_ID) {
                printk(KERN_INFO "avago_ofn_probe: Product Major Version %x\n", ret);
                pass = 1;
                break;
            }else{
				pr_err("OFN:Reset OFN for %dth checking ID : 0x%x\n", times,ret);
				pr_debug("Hard reset OFN\n");
				ofn_hw_reset(DISABLE_INIT);
				pr_debug("Soft reset OFN\n");
				ofn_soft_reset(ts);
            }
        }

        if (pass == 0)
		return OFN_ERR;
        else
		return OFN_GOOD;
		//compal_end
}

#define T_MOT_RST   50      // ms, suggestiion by avago
#define T_VRT_NRST  100     // ms
void ofn_soft_reset(struct avago_ofn_data *ts)
{
    if( ts->shutdown_gpio )
    {
        if(gpio_get_value(ts->shutdown_gpio))
        {
            pr_err("%s Shutdown GPIO is high\n", __func__);
            gpio_set_value(ts->shutdown_gpio, 0);
        }
    }
    msleep(T_VRT_NRST);
    avago_write_register_byte(ts->spi, 0x3A, 0x5A);
    msleep(T_MOT_RST);
}

#define T_NRST  20      // us
void ofn_power_off( int en_irq, int en_delay )
{
    struct avago_ofn_data *tspi = g_ts;
    if( ldo5 == NULL )
    {
        pr_err("%s LDO5 is NULL\n", __func__);
        return;
    }

    if(!en_irq && !IS_ERR_OR_NULL(tspi) )
        disable_irq(gpio_to_irq(tspi->irq));

    msleep(T_VRT_NRST);

//++ Charles
    if(g_ts->is_reg_en){
	regulator_disable(ldo5);
	g_ts->is_reg_en = 0;
    }
//--

    ofn_drive_spi(OFN_SPI_GPIO_LOW, 1, 0);  // prevent leakage from drive low

    // suggestion by avago
    if(en_delay)
        mdelay(100);
}

void ofn_power_on( int en_irq , int en_init)
{
    struct avago_ofn_data *tspi = g_ts;
    if( ldo5 == NULL )
    {
        pr_err("%s LDO5 is NULL\n", __func__);
        return;
    }

    regulator_enable(ldo5);

//++ Charles
    g_ts->is_reg_en = 1;
//--

    ofn_drive_spi(OFN_SPI_GPIO_HIGH, 0, 1);
    mdelay(T_MOT_RST);

    if( !IS_ERR_OR_NULL(tspi) )
    {
        if(en_init)
            avago_init_ofn(tspi, 0);

        if(en_irq)
            enable_irq(gpio_to_irq(tspi->irq));
    }
}

void ofn_hw_reset( int en_init )
{
    ofn_power_off(DISABLE_IRQ, 1);
    ofn_power_on(ENABLE_IRQ,en_init);
}

#define T_P_SHTDOWN 150     // ms
#define T_WAKEUP    100     // ms
void inline ofn_shutdown( struct avago_ofn_data *ts )
{
    gpio_set_value(ts->shutdown_gpio, 1);
    msleep(T_P_SHTDOWN);
    gpio_set_value(ts->shutdown_gpio, 0);
    msleep(T_WAKEUP);
}

void ofn_reset( struct avago_ofn_data *tspi)
{
    pr_debug("%s\n", __func__);
    disable_irq(gpio_to_irq(tspi->irq));

    avago_init_ofn(tspi, ENABLE_RESET);

    // check product id again
    if( ofn_chk_id(tspi) == OFN_ERR )
    {
        pr_err("OFN:Fail to reset OFN\n");
    }

    enable_irq(gpio_to_irq(tspi->irq));
}

void inline ofn_free_gpio(struct avago_ofn_data *ts)
{
    if( ts->shutdown_gpio )
        gpio_free(ts->shutdown_gpio);
    if( ts->up_led_gpio )
        gpio_free(ts->up_led_gpio);
    if( ts->dn_led_gpio )
        gpio_free(ts->dn_led_gpio);
    if( ts->dome_gpio )
        gpio_free(ts->dome_gpio);
    if( ts->irq_gpio )
        gpio_free(ts->irq_gpio);
}

void ofn_clear_buffer(struct avago_ofn_data *ts, const char* func)
{
    struct spi_device *spi = ts->spi;
    int max,cnt = 0;
    if( avago_read_register_byte(spi, 2)&0x10 )
    {
        pr_info("%s overflow\n", func);
        avago_write_register_byte(spi, 2, 0x80);
    }

    if( avago_read_register_byte(spi, 0x11)&0x80 )
        max = 32;
    else
        max = 16;

    while( avago_read_register_byte(spi, 2)&0x80 &&
            cnt++ < max )
    {
        avago_read_register_byte(spi, 3);
        avago_read_register_byte(spi, 4);
    }

    if( ((avago_read_register_byte(spi, 0x2e)&0xc0)>>6) > 1 )
    {
        pr_info("%s clear buffer for idle of long time\n", func);
        cnt = 0;
        while(cnt++ < max )
        {
            avago_read_register_byte(spi, 2);
            avago_read_register_byte(spi, 3);
            avago_read_register_byte(spi, 4);
        }
    }
}

void ofn_set_spi_tristate( u32 tri )
{
    if( tri )
    {
        tegra_pinmux_set_tristate(TEGRA_PINGROUP_SPID, TEGRA_TRI_TRISTATE);
        tegra_pinmux_set_tristate(TEGRA_PINGROUP_SPIE, TEGRA_TRI_TRISTATE);
        tegra_pinmux_set_tristate(TEGRA_PINGROUP_SPIF, TEGRA_TRI_TRISTATE);
    }
    else
    {
        tegra_pinmux_set_tristate(TEGRA_PINGROUP_SPID, TEGRA_TRI_NORMAL);
        tegra_pinmux_set_tristate(TEGRA_PINGROUP_SPIE, TEGRA_TRI_NORMAL);
        tegra_pinmux_set_tristate(TEGRA_PINGROUP_SPIF, TEGRA_TRI_NORMAL);
    }
}

static int __devinit avago_ofn_probe(struct spi_device *spi)
{
    struct avago_ofn_data *ts;
    int ret = 0, i;

    printk("%s: start initialization\n", __func__);

    // recovery spi bus tristate to normal
    ofn_set_spi_tristate(0);

    // request LDO5 for hardware reset
    ldo5 = regulator_get(NULL, "vdd_ldo5");
    if( IS_ERR_OR_NULL(ldo5) ){
        pr_err("%s fail to get regulator LDO5\n", __func__);
    }

    spi->bits_per_word = 8;
    spi->mode = SPI_MODE_3;
    if (spi_setup(spi) < 0)
    {
        printk(KERN_ERR "avago_ofn_probe: init SPI failed.\n");
        ret = -ENODEV;
    }

    ts = kzalloc(sizeof(struct avago_ofn_data), GFP_KERNEL);
    if (ts == NULL)
    {
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }

    INIT_WORK(&ts->work, avago_ofn_work_func);

    ts->spi = spi;
    ts->irq = spi->irq;
    dev_set_drvdata(&spi->dev, ts);

    ts->shutdown_gpio = 0;

//++ Charles
    ts->is_reg_en = 0;
    g_ts = ts;
//--
    #ifdef AVAGO_GPIO_CONTROL_SHUTDOWN
    if ( avago_init_gpio( ts, AVAGO_SHUTDOWN_GPIO, "OFN SHUTDOWN", OFN_GPIO_OUT, OFN_GPIO_LOW, &ts->shutdown_gpio) < 0 )
        goto err_detect_failed;
    #endif // AVAGO_GPIO_CONTROL_SHUTDOWN

    // Handle UP/DN LED GPIO
    if( (avago_init_gpio(ts, AVAGO_LED_UP, "OFN_LED_UP", OFN_GPIO_OUT, OFN_GPIO_LOW, &ts->up_led_gpio) < 0) ||
        (avago_init_gpio(ts, AVAGO_LED_DOWN, "OFN_LED_DOWN", OFN_GPIO_OUT, OFN_GPIO_LOW, &ts->dn_led_gpio) < 0) )
        goto err_detect_failed;
    // End of Handle UP/DN LED GPIO

    ts->dome_gpio = 0;
    ts->dome_count = 0;
    
    //#ifdef AVAGO_DOME
    if ( avago_init_gpio( ts, AVAGO_DOME_GPIO, "OFN DOME", OFN_GPIO_IN, OFN_GPIO_LOW, &ts->dome_gpio) < 0 )
        goto err_detect_failed;
    //#endif // AVAGO_DOME

    ts->irq_gpio = 0;
    ts->detect_delay = 300;
    if ( avago_init_gpio( ts, spi->irq, "OFN IRQ", OFN_GPIO_IN, OFN_GPIO_LOW, &ts->irq_gpio) < 0 )
        goto err_gpio;
    ts->irq_gpio = spi->irq;

err_gpio:
    pr_debug("Hard reset OFN\n");
    ofn_hw_reset(DISABLE_INIT);
    pr_debug("Soft reset OFN\n");
    ofn_soft_reset(ts);

    {
        int times = 0;
        int pass = 0;
        u8 u8_ret;
        while (times++ < 5) {
            u8_ret = avago_read_register_byte(ts->spi, 0x00);
            ret = (int)u8_ret;

            if (ret == AVAGO_OFN_ID) {
                printk(KERN_INFO "avago_ofn_probe: Product Major Version %x\n", ret);
                pass = 1;
                break;
            }
            else
            {
                pr_err("OFN:Reset OFN for %dth checking ID\n", times);
                ofn_hw_reset(DISABLE_INIT);
            }
        }

        if (pass == 0) {
            printk(KERN_ERR "avago SPI read failed\n");
            goto err_detect_failed;
        }
    }

    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        ret = -ENOMEM;
        printk(KERN_ERR "avago_ofn_probe: Failed to allocate input device\n");
        goto err_input_dev_alloc_failed;
    }

    ts->input_dev->name = "avago-pt";
    ts->input_dev->phys = "aofn/input0";
    ts->input_dev->id.bustype = BUS_HOST;
    ts->input_dev->id.vendor = 0x0001;
    ts->input_dev->id.product = 0x0001;
    ts->input_dev->id.version = 0x0100;

    ts->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);
    ts->input_dev->keycode = avago_keycode;
    ts->input_dev->keycodesize = sizeof(unsigned char);
    ts->input_dev->keycodemax = ARRAY_SIZE(avago_keycode);

    for (i = 0; i < ARRAY_SIZE(avago_keycode); i++) {
        set_bit(avago_keycode[i], ts->input_dev->keybit);
    }

    ret = input_register_device(ts->input_dev);
    if (ret) {
        pr_err("%s: Unable to register %s input device\n", __func__, ts->input_dev->name);
        goto err_input_register_device_failed;
    }

    if (spi->irq) {
        ret = request_irq(gpio_to_irq(ts->irq), avago_ts_irq_handler, IRQF_TRIGGER_NONE,
            "avago-ofn motion", ts);
        if (ret == 0)
        {
            //+++[ASD2_ES1|Jaco_Gao|2011.05.16] prevent race condition during chip initialzation
            // disable interrupt of motion pin before chip initialization completed
            // Note : Enabling interrupt before chip initialization will have chance to irregular
            // interrupt. This will make race condition of resource in timer function may not be
            // initialized properly.
            disable_irq(gpio_to_irq(ts->irq));
            //---[ASD2_ES1|Jaco_Gao|2011.05.16] prevent race condition during chip initialzation

            pr_debug("%s: request ts irq %d", __func__, gpio_to_irq(spi->irq));
            ts->use_irq = 1;
            hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
            hrtimer_init(&ts->up_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
            hrtimer_init(&ts->dn_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

            ts->timer.function = avago_ts_int_timer_func;
            ts->up_timer.function = avago_up_led_func;
            ts->dn_timer.function = avago_dn_led_func;

            ts->int_timer_flag = 0;
        }
        else
            dev_err(&spi->dev, "%s: request ts irq failed\n", __FUNCTION__);
    }

    if (ts->dome_gpio)
    {
        // DOME interrupt
        ret = request_irq(TEGRA_GPIO_TO_IRQ(ts->dome_gpio), avago_dome_irq_handler, IRQF_TRIGGER_NONE,
            "avago-ofn dome", ts);

        if (ret == 0)
        {
            pr_debug("%s: dome gpio status %d\n", __func__, gpio_get_value(ts->dome_gpio));
            pr_debug("%s: request dome irq %d", __func__, gpio_to_irq(ts->dome_gpio));
            disable_irq(gpio_to_irq(ts->dome_gpio));
            ts->use_irq = 1;
            hrtimer_init(&ts->dome_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
            ts->dome_timer.function = avago_dome_timer_func;
        }
        else
            dev_err(&spi->dev, "%s: request dome irq failed\n", __func__);
    }
    
#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = avago_ofn_early_suspend;
    ts->early_suspend.resume = avago_ofn_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

    pr_debug("%s: Start OFN %s in %s mode\n", __func__,
	    ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

    // initial the chip
    ret = avago_init_ofn(ts, DISABLE_RESET);
//comapl_start willy don't need check ID again
#if 0
    if( ofn_chk_id(ts) == OFN_ERR )
    {
        pr_err("OFN:Fail to initialize chip\n");
        goto err_input_register_device_failed;
    }
#endif
//compal_end

    // initial sysfs
    ofn_kobj = ofn_sysfs_init("ofn");
//    g_ts = ts;

    avago_ofn_init_led_timer(ts);       //+++[ASD2_ES1|Jaco_Gao|2011.04.18] Flash led by timer

//++Charles
    //[ASD2_ES1|Jaco_Gao|20110630] clear buffer before enable interrupt
    ofn_clear_buffer(ts, __func__);

    irq_set_irq_type(gpio_to_irq(ts->irq), IRQF_TRIGGER_FALLING);
    irq_set_irq_type(gpio_to_irq(ts->dome_gpio), IRQF_TRIGGER_FALLING); 
    
    enable_irq(gpio_to_irq(ts->irq));
    enable_irq(gpio_to_irq(ts->dome_gpio));
// causing failure of OFN function
#if 0	
	ofn_clear_buffer(ts, __func__);//Charlie remove clear buffer after enable IRQ 2012.2.17
#endif
//--

    ofn_boot = 0;
    ofn_init_finish = 1;//Charlie add flag to avoid hang up 2012.2.17
    pr_info("%s : avago ofn probe done\n", __func__);
    return 0;

err_input_register_device_failed:
    input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
    //err_power_failed:
    ofn_free_gpio(ts);
    kfree(ts);
err_alloc_data_failed:
    //err_check_functionality_failed:
    regulator_put(ldo5);
    ofn_set_spi_tristate(1);
    return ret;
}

static int avago_ofn_remove(struct spi_device *spi)
{
    struct avago_ofn_data *ts = dev_get_drvdata(&spi->dev);
    //unregister_early_suspend(&ts->early_suspend);

    if(ts->shutdown_gpio)
        gpio_free(ts->shutdown_gpio);
    if(ts->dome_gpio)
        gpio_free(ts->dome_gpio);

    if (ts->use_irq)
        free_irq(spi->irq, ts);
    else
        hrtimer_cancel(&ts->timer);

    if(ts->dome_gpio)
        hrtimer_cancel(&ts->dome_timer);

    input_unregister_device(ts->input_dev);
    kfree(ts);
    return 0;
}

static int avago_ofn_suspend(struct spi_device *spi, pm_message_t mesg)
{
    int ret;
    struct avago_ofn_data *ts = dev_get_drvdata(&spi->dev);

    if (ts->use_irq)
        disable_irq(TEGRA_GPIO_TO_IRQ(ts->irq));
    else
        hrtimer_cancel(&ts->timer);
    ret = cancel_work_sync(&ts->work);

    disable_irq(gpio_to_irq(ts->dome_gpio));

    if(ts->shutdown_gpio)
	gpio_direction_output(ts->shutdown_gpio, 1);

    if(ts->dome_gpio)
		hrtimer_cancel(&ts->dome_timer);

//++ Charles
/*
    if (ts->power) {
        ret = ts->power(0);
        if (ret < 0) {
            printk(KERN_ERR "%s: power off failed\n", __func__);
        }
    }
*/
    regulator_disable(ldo5);
    ts->is_reg_en = 0;
//--
    return 0;
}

static int avago_ofn_resume(struct spi_device *spi)
{
    struct avago_ofn_data *ts = dev_get_drvdata(&spi->dev);
//++ Charles
    regulator_enable(ldo5);
    ts->is_reg_en = 1;
/*
    if (ts->power) {
        ret = ts->power(1);
        if (ret < 0) {
            printk(KERN_ERR "%s: power on failed\n", __func__);
        }
    }
*/
//--
    // reset, same effect as de-asserting shutdown pin.
    if(ts->shutdown_gpio)
    {
        gpio_direction_output(ts->shutdown_gpio, 0);
        // initial the chip
        avago_init_ofn(ts, 1);
        msleep(200);
    }

    enable_irq(gpio_to_irq(ts->dome_gpio));

    if (ts->use_irq)
        enable_irq(TEGRA_GPIO_TO_IRQ(spi->irq));

    if (!ts->use_irq) {
        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = avago_ts_timer_func;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    if (ts->dome_gpio)
    {
        ts->dome_count = 0;
    }

    if (ts->fpd_support)
    {
        init_fpd(ts);
    }

    //Compal_start Willy check chip status again
    if( ofn_chk_id(ts) == OFN_ERR )
    {
        pr_err("OFN:Fail to reset OFN\n");
    }
    //Compal_end Willy

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void avago_ofn_early_suspend(struct early_suspend *h)
{
    struct avago_ofn_data *ts;
    ts = container_of(h, struct avago_ofn_data, early_suspend);
    avago_ofn_suspend(ts->spi, PMSG_SUSPEND);
}

static void avago_ofn_late_resume(struct early_suspend *h)
{
    struct avago_ofn_data *ts;
    ts = container_of(h, struct avago_ofn_data, early_suspend);
    avago_ofn_resume(ts->spi);
}
#endif

static struct spi_driver avago_ofn_driver = {
    .driver    = {
	.name   = "avago-ofn",
	.bus    = &spi_bus_type,
	.owner  = THIS_MODULE,
    },
    .probe   = avago_ofn_probe,
    .remove  = avago_ofn_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = avago_ofn_suspend,
    .resume	 = avago_ofn_resume,
#endif   
};

static int __init avago_ofn_init(void)
{
    avago_ofn_wq = create_singlethread_workqueue("avago_ofn_wq");
    if (!avago_ofn_wq)
        return -ENOMEM;
    return spi_register_driver(&avago_ofn_driver);
}

static void __exit avago_ofn_exit(void)
{
    if (avago_ofn_wq)
        destroy_workqueue(avago_ofn_wq);

    spi_unregister_driver(&avago_ofn_driver);
}

module_init(avago_ofn_init);
module_exit(avago_ofn_exit);

MODULE_DESCRIPTION("AVAGO OFN SPI DRIVER");
MODULE_LICENSE("GPL");
