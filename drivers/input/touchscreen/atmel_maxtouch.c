/*
 *  Atmel maXTouch Touchscreen Controller
 *
 *
 *  Copyright (C) 2010 Atmel Corporation
 *  Copyright (C) 2009 Raphael Derosso Pereira <raphaelpereira@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <linux/uaccess.h>

#include <linux/i2c/atmel_maxtouch.h>
#if 0
/* COMPAL - START - [2011/6/16  16:5] - Jamin - [Touch]: update firmware 1.1  */
#include <linux/i2c/mxt1386_firmware.h>
/* COMPAL - END */
#endif
/* COMPAL - START - [2011/1/19  18:53] - Jamin - [Touch]: enable touch atmel   */
#include <linux/delay.h>
/* COMPAL - END */

/* COMPAL - START - [2011/2/17  15:57] - Jamin - [Touch]: fix suspend ,resume issue  */
//#define CONFIG_HAS_EARLYSUSPEND 1
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
/* COMPAL - END */


/*
 * This is a driver for the Atmel maXTouch Object Protocol
 *
 * When the driver is loaded, mxt_init is called.
 * mxt_driver registers the "mxt_driver" structure in the i2c subsystem
 * The mxt_idtable.name string allows the board support to associate
 * the driver with its own data.
 *
 * The i2c subsystem will call the mxt_driver.probe == mxt_probe
 * to detect the device.
 * mxt_probe will reset the maXTouch device, and then
 * determine the capabilities of the I2C peripheral in the
 * host processor (needs to support BYTE transfers)
 *
 * If OK; mxt_probe will try to identify which maXTouch device it is
 * by calling mxt_identify.
 *
 * If a known device is found, a linux input device is initialized
 * the "mxt" device data structure is allocated,
 * as well as an input device structure "mxt->input"
 * "mxt->client" is provided as a parameter to mxt_probe.
 *
 * mxt_read_object_table is called to determine which objects
 * are present in the device, and to determine their adresses.
 *
 *
 * Addressing an object:
 *
 * The object is located at a 16-bit address in the object address space.
 *
 * The address is provided through an object descriptor table in the beginning
 * of the object address space. This address can change between firmware
 * revisions, so it's important that the driver will make no assumptions
 * about addresses but instead reads the object table and gets the correct
 * addresses there.
 *
 * Each object type can have several instances, and the number of
 * instances is available in the object table as well.
 *
 * The base address of the first instance of an object is stored in
 * "mxt->object_table[object_type].chip_addr",
 * This is indexed by the object type and allows direct access to the
 * first instance of an object.
 *
 * Each instance of an object is assigned a "Report Id" uniquely identifying
 * this instance. Information about this instance is available in the
 * "mxt->report_id" variable, which is a table indexed by the "Report Id".
 *
 * The maXTouch object protocol supports adding a checksum to messages.
 * By setting the most significant bit of the maXTouch address,
 * an 8 bit checksum is added to all writes.
 *
 *
 * How to use driver.
 * -----------------
 * Example:
 * In arch/avr32/boards/atstk1000/atstk1002.c
 * an "i2c_board_info" descriptor is declared.
 * This contains info about which driver ("mXT224"),
 * which i2c address and which pin for CHG interrupts are used.
 *
 * In the "atstk1002_init" routine, "i2c_register_board_info" is invoked
 * with this information. Also, the I/O pins are configured, and the I2C
 * controller registered is on the application processor.
 *
 *
 */

static int debug = NO_DEBUG;
/* COMPAL - START - [2011/5/24  20:23] - Jamin - [Touch]: fix sometimes can not release issue    */
static int comms=1;
/* COMPAL - END */

/* COMPAL - START - [2011/5/26  9:25] - Jamin - [Touch]: add retry auto calibration mechanism     */
#define ENABLE_AUTO_CALIBRATION 
/* COMPAL - END */

/* COMPAL - START - [2011/6/30  10:58] - Jamin - [Touch]: update firmware 1.1  */
//#define ENABLE_AUTO_UPGRADE_FW11
/* COMPAL - END */

module_param(debug, int, 0644);
module_param(comms, int, 0644);

MODULE_PARM_DESC(debug, "Activate debugging output");
MODULE_PARM_DESC(comms, "Select communications mode");

/* Device Info descriptor */
/* Parsed from maXTouch "Id information" inside device */
struct mxt_device_info {
	u8 family_id;
	u8 variant_id;
	u8 major;
	u8 minor;
	u8 build;
	u8 num_objs;
	u8 x_size;
	u8 y_size;
	char family_name[16];	/* Family name */
	char variant_name[16];	/* Variant name */
	u16 num_nodes;		/* Number of sensor nodes */
};

/* object descriptor table, parsed from maXTouch "object table" */
struct mxt_object {
	u16 chip_addr;
	u8 type;
	u8 size;
	u8 instances;
	u8 num_report_ids;
};

/* Mapping from report id to object type and instance */
struct report_id_map {
	u8 object;
	u8 instance;
/*
 * This is the first report ID belonging to object. It enables us to
 * find out easily the touch number: each touch has different report
 * ID (which are assigned to touches in increasing order). By
 * subtracting the first report ID from current, we get the touch
 * number.
 */
	u8 first_rid;
};

/* Driver datastructure */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input;
	char phys_name[32];
	int irq;

	u16 last_read_addr;
	bool new_msgs;
	u8 *last_message;

	int valid_irq_counter;
	int invalid_irq_counter;
	int irq_counter;
	int message_counter;
	int read_fail_counter;

	int bytes_to_read;

	struct delayed_work dwork;
	/* COMPAL - START - [2011/5/25  13:53] - Jamin - [Touch]: add retry auto calibration mechanism     */
	#ifdef ENABLE_AUTO_CALIBRATION
    struct timer_list cal_timer; 
	struct delayed_work dcal_work;
	#endif
	/* COMPAL - END */

	u8 xpos_format;
	u8 ypos_format;

	u8 numtouch;

	struct mxt_device_info device_info;

	u32 info_block_crc;
	u32 configuration_crc;
	u16 report_id_count;
	struct report_id_map *rid_map;
	struct mxt_object *object_table;

	u16 msg_proc_addr;
	u8 message_size;

	u16 max_x_val;
	u16 max_y_val;

	void (*init_hw) (void);
	void (*exit_hw) (void);
	/* COMPAL - START - [2011/3/24  11:46] - Jamin - [Touch]: add reset node  */
       void (*reset_hw) (void);
	u8 (*read_reset)(void);
	/* COMPAL - END */
	 u8(*valid_interrupt) (void);
	 u8(*read_chg) (void);

	/* debugfs variables */
	struct dentry *debug_dir;
	int current_debug_datap;
	/* COMPAL - START - [2011/5/25  10:38] - Jamin - [Touch]: add sys node for read delta and refs data    */
       int current_delta_datap;
	int current_refs_datap;
	/* COMPAL - END */

	 /* COMPAL - START - [2011/5/17  15:35] - Jamin - [Touch]: check touch version  */
	 unsigned char check_version;
        unsigned char check_resistance;
	 u32 suspend_count;
	/* COMPAL - END */

	struct mutex debug_mutex;
	u16 *debug_data;

	/* Character device variables */
	struct cdev cdev;
	struct cdev cdev_messages;	/* 2nd Char dev for messages */
	dev_t dev_num;
	struct class *mxt_class;

	u16 address_pointer;
	bool valid_ap;

	/* Message buffer & pointers */
	char *messages;
	int msg_buffer_startp, msg_buffer_endp;
	/* Put only non-touch messages to buffer if this is set */
	char nontouch_msg_only;
	struct mutex msg_mutex;
	/* COMPAL - START - [2011/2/17  15:56] - Jamin - [Touch]: fix suspend ,resume issue  */
	#ifdef CONFIG_HAS_EARLYSUSPEND
    	struct early_suspend early_suspend;
       #endif
	/* COMPAL - END */   
};

/* COMPAL - START - [2011/2/16  11:37] - Jamin - [Touch]: add system node for debug  */
//change global variable for debug
struct mxt_data *mxt;
/* COMPAL - END */

/* COMPAL - START - [2011/3/2  13:25] - Jamin - [Touch]: fix suspend resume issue  */
//change global variable
 int stored_size[MXT_MAX_NUM_TOUCHES];
 int stored_x[MXT_MAX_NUM_TOUCHES];
 int stored_y[MXT_MAX_NUM_TOUCHES];
 u8 temp_buf[100];
 u8 str_buf[200];
/* COMPAL - END */

#define I2C_RETRY_COUNT 5
#define I2C_PAYLOAD_SIZE 254

/* Returns the start address of object in mXT memory. */ 
#define	MXT_BASE_ADDR(object_type, mxt)					\
	get_object_address(object_type, 0, mxt->object_table,           \
			   mxt->device_info.num_objs)

/* Maps a report ID to an object type (object type number). */
#define	REPORT_ID_TO_OBJECT(rid, mxt)			\
	(((rid) == 0xff) ? 0 : mxt->rid_map[rid].object)

/* Maps a report ID to an object type (string). */
#define	REPORT_ID_TO_OBJECT_NAME(rid, mxt)			\
	object_type_name[REPORT_ID_TO_OBJECT(rid, mxt)]

/* Returns non-zero if given object is a touch object */
#define IS_TOUCH_OBJECT(object) \
	((object == MXT_TOUCH_MULTITOUCHSCREEN_T9) || \
	 (object == MXT_TOUCH_KEYARRAY_T15) ||	\
	 (object == MXT_TOUCH_PROXIMITY_T23) || \
	 (object == MXT_TOUCH_SINGLETOUCHSCREEN_T10) || \
	 (object == MXT_TOUCH_XSLIDER_T11) || \
	 (object == MXT_TOUCH_YSLIDER_T12) || \
	 (object == MXT_TOUCH_XWHEEL_T13) || \
	 (object == MXT_TOUCH_YWHEEL_T14) || \
	 (object == MXT_TOUCH_KEYSET_T31) || \
	 (object == MXT_TOUCH_XSLIDERSET_T32) ? 1 : 0)

#define mypr_debug(fmt, ...) \
	printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
	
#define mxt_debug(level, ...) \
	do { \
		if (debug >= (level)) \
			mypr_debug(__VA_ARGS__); \
	} while (0)

static const u8 *object_type_name[] = {
	[0] = "Reserved",
	[5] = "GEN_MESSAGEPROCESSOR_T5",
	[6] = "GEN_COMMANDPROCESSOR_T6",
	[7] = "GEN_POWERCONFIG_T7",
	[8] = "GEN_ACQUIRECONFIG_T8",
	[9] = "TOUCH_MULTITOUCHSCREEN_T9",
	[15] = "TOUCH_KEYARRAY_T15",
	[18] = "SPT_COMMSCONFIG_T18",
	[19] = "SPT_GPIOPWM_T19",
	[20] = "PROCI_GRIPFACESUPPRESSION_T20",
	[22] = "PROCG_NOISESUPPRESSION_T22",
	[23] = "TOUCH_PROXIMITY_T23",
	[24] = "PROCI_ONETOUCHGESTUREPROCESSOR_T24",
	[25] = "SPT_SELFTEST_T25",
	[27] = "PROCI_TWOTOUCHGESTUREPROCESSOR_T27",
	[28] = "SPT_CTECONFIG_T28",
	[37] = "DEBUG_DIAGNOSTICS_T37",
	[38] = "SPT_USER_DATA_T38",
	[40] = "PROCI_GRIPSUPPRESSION_T40",
	[41] = "PROCI_PALMSUPPRESSION_T41",
	[42] = "PROCI_FACESUPPRESSION_T42",
	[43] = "SPT_DIGITIZER_T43",
	[44] = "SPT_MESSAGECOUNT_T44",
};

//30ohm
unsigned char UserdataOBJ[64] = {0};                                             //t38
/* COMPAL - START - [2011/5/27  19:23] - Jamin - [Touch]: tune 65ohm version 1 configure    */
unsigned char UserdataOBJ_65ohm_v1[64] = {65,1,0};                                             //t38
/* COMPAL - END */
/* COMPAL - START - [2011/5/27  19:23] - Jamin - [Touch]: tune 65ohm version 2 configure    */
unsigned char UserdataOBJ_65ohm_v2[64] = {65,2,0};                                             //t38
/* COMPAL - END */

//30ohm
unsigned char  PowerOBJ[3] = {65, 255, 50};                                 //t7
/* COMPAL - START - [2011/5/27  19:23] - Jamin - [Touch]: tune 65ohm version 1 configure    */
unsigned char  PowerOBJ_65ohm_v1[3] = {65, 255, 50};                                 //t7
/* COMPAL - END */
/* COMPAL - START - [2011/5/27  19:23] - Jamin - [Touch]: tune 65ohm version 2 configure    */
unsigned char  PowerOBJ_65ohm_v2[3] = {65, 255, 50};                                 //t7
/* COMPAL - END */

//30ohm
unsigned char AcqOBJ[10] = {10, 0, 20, 20, 0, 0, 5, 55, 10, 192};        //t8
/* COMPAL - START - [2011/5/27  17:56] - Jamin - [Touch]: tune 65ohm version 1 configure    */
unsigned char AcqOBJ_65ohm_v1[10] = {10, 0, 10, 10, 0, 0, 5, 55, 10, 192};        //t8
/* COMPAL - END */
/* COMPAL - START - [2011/5/27  18:9] - Jamin - [Touch]: tune 65ohm version 2 configure    */
unsigned char AcqOBJ_65ohm_v2[10] = {9, 0, 10, 10, 0, 0, 5, 55, 10, 192};        //t8
/* COMPAL - END */

//30ohm
unsigned char MTouchOBJ[34] = { 143, 0, 0, 28, 42, 0, 16, 50, 2, 5,     //change resolution 4096*4096   //t9   
                           			   	0, 15, 20, 65, MXT_MAX_NUM_TOUCHES, 10, 10, 10, 0xff, 0x0f, 0xff, 0x0f,
                          					0, 0, 0, 0, 0, 0, 64, 0, 30,15, 0, 0 };
/* COMPAL - START - [2011/5/27  17:57] - Jamin - [Touch]: tune 65ohm version 1 configure    */
unsigned char MTouchOBJ_65ohm_v1[34] = { 143, 0, 0, 28, 42, 0, 32, 85, 2, 5,     //change resolution 4096*4096   //t9   
                           			   	0, 15, 20, 65, MXT_MAX_NUM_TOUCHES, 10, 10, 10, 0xff, 0x0f, 0xff, 0x0f,
                          					0, 0, 0, 0, 0, 0, 64, 0, 50,15, 0, 0 };
/* COMPAL - END */
/* COMPAL - START - [2011/5/27  18:9] - Jamin - [Touch]: tune 65ohm version 2 configure    */
unsigned char MTouchOBJ_65ohm_v2[34] = { 143, 0, 0, 28, 42, 0, 32, 105, 2, 5,     //change resolution 4096*4096   //t9   
                           			   	0, 15, 20, 65, MXT_MAX_NUM_TOUCHES, 20, 20, 10, 0xff, 0x0f, 0xff, 0x0f,
                          					0, 0, 0, 0, 0, 0, 64, 0, 50,20, 0, 0 };
/* COMPAL - END */
unsigned char KeyArrayOBJ[22] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,         //t15 2 instances
                            					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char ComconfigOBJ[2] = {0, 0};                                   //t18

//30ohm
unsigned char NoiseOBJ[17] = { 5, 0, 0, 0, 0, 0, 0, 0, 32, 0, 0,            //t22  
                         					12, 17, 22, 27, 30, 0 };  
/* COMPAL - START - [2011/5/27  17:59] - Jamin - [Touch]: tune 65ohm version 1 configure    */
unsigned char NoiseOBJ_65ohm_v1[17] = { 5, 0, 0, 0, 0, 0, 0, 0, 45, 0, 0,            //t22  
                         					12, 17, 22, 27, 30, 0 };  
/* COMPAL - END */
/* COMPAL - START - [2011/5/27  17:59] - Jamin - [Touch]: tune 65ohm version 2 configure    */
unsigned char NoiseOBJ_65ohm_v2[17] = { 5, 0, 0, 0, 0, 0, 0, 0, 60, 0, 0,            //t22  
                         					10, 13, 20, 25, 30, 0 };  
/* COMPAL - END */

unsigned char  OneTGOBJ[19] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,         //t24
                         					0, 0, 0, 0, 0, 0, 0};
unsigned char  SelfTestOBJ[14] = { 3, 0/*0xfe*/, 0x10, 0x27, 0x50, 0x14, 0, 0, 0,         //t25 
                           				 0, 0, 0, 0, 0};  //set SelfTestOBJ[1]=0xfe start test
unsigned char TwoTGOBJ[7] = { 0, 0, 0, 0, 0, 0, 0};                        //t27
unsigned char CTEOBJ[6] = { 0, 0, 0, 8, 16, 60};                            //t28
unsigned char GripSUPOBJ[5] = { 0, 0, 0, 0, 0};                    //t40

//30ohm
unsigned char PalmSUPOBJ[6] = { 1, 0, 0, 30, 5, 20};                       //t41 
/* COMPAL - START - [2011/5/27  17:59] - Jamin - [Touch]: tune 65ohm version 1 configure    */
unsigned char PalmSUPOBJ_65ohm_v1[6] = { 0, 0, 0, 30, 5, 20};                       //t41 
unsigned char PalmSUPOBJ_65ohm_fw11_v1[7] = { 1, 0, 0, 30, 5, 5,200};                       //t41 
/* COMPAL - END */
/* COMPAL - START - [2011/5/27  17:59] - Jamin - [Touch]: tune 65ohm version 2 configure    */
unsigned char PalmSUPOBJ_65ohm_v2[6] = { 0, 0, 0, 30, 5, 20};                       //t41 
unsigned char PalmSUPOBJ_65ohm_fw11_v2[7] = { 1, 0, 0, 30, 5, 5,200};                       //t41 
/* COMPAL - END */

unsigned char DigitizerOBJ[6] = { 0, 0, 0, 0, 0, 0};                    //t43 


enum {
	TOUCH_30OHM= 0 ,
	TOUCH_65OHM_V1,
	TOUCH_65OHM_V2,
};
static unsigned char  *Power_table[] = {   //T7
	[TOUCH_30OHM]   = PowerOBJ,
	[TOUCH_65OHM_V1]  = PowerOBJ_65ohm_v1,
	[TOUCH_65OHM_V2] = PowerOBJ_65ohm_v2,
};
static unsigned char  *Acq_table[] = {   //T8
	[TOUCH_30OHM]   = AcqOBJ,
	[TOUCH_65OHM_V1]  = AcqOBJ_65ohm_v1,
	[TOUCH_65OHM_V2] = AcqOBJ_65ohm_v2,
};
static unsigned char  *MTouch_table[] = {   //T9
	[TOUCH_30OHM]   = MTouchOBJ,
	[TOUCH_65OHM_V1]  = MTouchOBJ_65ohm_v1,
	[TOUCH_65OHM_V2] = MTouchOBJ_65ohm_v2,
};
static unsigned char  *Noise_table[] = {   //T22
	[TOUCH_30OHM]   = NoiseOBJ,
	[TOUCH_65OHM_V1]  = NoiseOBJ_65ohm_v1,
	[TOUCH_65OHM_V2] = NoiseOBJ_65ohm_v2,
};
#if 0
static unsigned char  *PalmSUP_table[] = {   //T41
	[TOUCH_30OHM]   = PalmSUPOBJ,
	[TOUCH_65OHM_V1]  = PalmSUPOBJ_65ohm_v1,
	[TOUCH_65OHM_V2] = PalmSUPOBJ_65ohm_v2,	
};
#endif
/* COMPAL - START - [2011/5/24  20:30] - Jamin - [Touch]: add sys node for read delta and refs data    */
ssize_t debug_data_read(struct mxt_data *mxt, char *buf, size_t count,
					loff_t *ppos, u8 debug_command);
/* COMPAL - END */
static u16 get_object_address(uint8_t object_type,
			      uint8_t instance,
			      struct mxt_object *object_table, int max_objs);

static int mxt_write_ap(struct mxt_data *mxt, u16 ap);

static int mxt_read_block_wo_addr(struct i2c_client *client,
				  u16 length, u8 *value);

/* COMPAL - START - [2011/2/17  15:56] - Jamin - [Touch]: fix suspend ,resume issue  */
#ifdef CONFIG_HAS_EARLYSUSPEND
void mxt_early_suspend(struct early_suspend *h);
void mxt_late_resume(struct early_suspend *h);
#endif
/* COMPAL - END */
#if 0
/* COMPAL - START - [2011/6/20  15:32] - Jamin - [Touch]: update firmware 1.1  */
int update_firmware(bool do_reset);
static void Atmel_WriteBack(struct mxt_data *mxt);
static void Atmel_WriteAllConfig(struct mxt_data *mxt,int version);
/* COMPAL - END */
#endif
/* COMPAL - START - [2011/2/16  10:33] - Jamin - [Touch]: add system node for debug  */
char hbyte, lbyte, val;
int rlen;
bool Debug = 0;
int myatoi(const char *a)
{
    int s = 0;
    while(*a >= '0' && *a <= '9')
        s = (s << 3) + (s << 1) + *a++ - '0';
    return s;
}

int mystrncmp(const char *cs, const char *ct, size_t count)
{
	unsigned char c1, c2;

	while (count) {
		c1 = *cs++;
		c2 = *ct++;
		if (c1 != c2)
			return c1 < c2 ? -1 : 1;
		count--;
	}
	return 0;
}
static ssize_t hbyte_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, &hbyte);
    return (s - buf);
}

static ssize_t hbyte_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int ibuf;

    ibuf = myatoi(buf);
    hbyte = ibuf & 0x000000ff;
    printk("peter ===== hbyte is 0x%x\n", hbyte);

    return n;
}

static ssize_t lbyte_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "test\n");
    return (s - buf);
}

static ssize_t lbyte_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int ibuf;

    ibuf = myatoi(buf);
    lbyte = ibuf & 0x000000ff;
    printk("peter ===== lbyte is 0x%x\n", lbyte);
    return n;
}

static ssize_t rlen_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "test\n");
    return (s - buf);
}

static ssize_t rlen_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{

    rlen = myatoi(buf);
    printk("peter ===== rlen is %d\n", rlen);
    return n;
}

static ssize_t val_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "test\n");
    return (s - buf);
}

static ssize_t val_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int ibuf,i;
    char *rdatabuf;
    unsigned int addr=0;
    rdatabuf = kzalloc(sizeof(char)*rlen, GFP_KERNEL);
   
    ibuf = myatoi(buf);
    val = ibuf & 0x000000ff;
   
    printk("peter ===== val is 0x%x\n", val);
    printk("peter ===== hbyte is 0x%x, lbyte is 0x%x\n", hbyte, lbyte);
	
    addr= ((addr |hbyte)<<8) |(lbyte & 0xFF);

    if(!strncmp(buf,"r", 1))
    {
       printk("Jamin ===== addr is 0x%x\n",addr);
       mxt_read_block(mxt->client,addr,rlen,rdatabuf);
        for(i=0;i<rlen;i++)
            printk("peter ===== rdatabuf is 0x%x\n", rdatabuf[i]);
    }
    else if(!strncmp(buf,"b", 1))
    {
        printk("peter ==== backup\n");
      // Back_up(globaltouch->hTouchDevice);
    }
    else if(!strncmp(buf,"s", 1))
    {
        printk("peter ==== deepsleep\n");
     //   Deepsleepmode(globaltouch->hTouchDevice);
    }
    else if(!strncmp(buf,"w", 1))
    {
        printk("peter ==== deepsleepback\n");
     //   Deepsleepmodeback(globaltouch->hTouchDevice);
    }
    else 
    {
        mxt_write_byte(mxt->client,addr,val);
    }

    return n;
}


static ssize_t debugmsg_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "%d\n",Debug);
    return (s - buf);
}

static ssize_t debugmsg_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int ibuf;
    ibuf = myatoi(buf);
    if(ibuf > 0 ){
		debug=ibuf;
		printk("[Touch]: open debug set level=%d \n",debug);
    }
    else
	 debug=NO_DEBUG;
    return n;
}

static ssize_t softreset_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "%d\n",Debug);
    return (s - buf);
}

static ssize_t softreset_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
     unsigned char  ComOBJ[6] = { 1, 0, 0, 0, 0, 0};
  
    if ( *buf == '1' )
    {
      printk("[Touch]: do soft reset \n");
      mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt),sizeof(ComOBJ),ComOBJ);
    }
    return n;
}
static ssize_t writeback_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "%d\n",Debug);
    return (s - buf);
}

static ssize_t writeback_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
     unsigned char  ComOBJ[6] = { 0, 85, 0, 0, 0, 0};
  
    if ( *buf == '1' )
    {
      printk("[Touch]: do write back \n");
      mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt),sizeof(ComOBJ),ComOBJ);
      msleep(100);
     }
    return n;
}
static ssize_t hardreset_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "%d\n",Debug);
    return (s - buf);
}

static ssize_t hardreset_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
  
    if ( *buf == '1' )
    {
      printk("[Touch]: do hard reset \n");
      if (mxt->reset_hw != NULL)
		mxt->reset_hw();
    }
    return n;
}
static ssize_t suspendcount_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "%d",mxt->suspend_count);
    return (s - buf);
}

static ssize_t suspendcount_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{ 
    mxt->suspend_count= myatoi(buf);
    return n;
}

static ssize_t restore_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "%d\n",Debug);
    return (s - buf);
}

static ssize_t restore_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
  
    if ( *buf == '1' )
    {
      printk("[Touch]: do restore \n");
      mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt),sizeof(MTouchOBJ),MTouchOBJ);
      mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt),sizeof(PalmSUPOBJ),PalmSUPOBJ);
    }
    return n;
}
static ssize_t gpio_rst_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    char status=-1;
    status=mxt->read_reset();
    dev_info(&mxt->client->dev, "[Touch] RST=%d\n",status);
    s += sprintf(s, "[Touch] RST=%d\n",status);
	  
    return (s - buf);
}

static ssize_t gpio_rst_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return n;
}
static ssize_t gpio_int_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    char status=-1;
    status=mxt->read_chg();
    dev_info(&mxt->client->dev, "[Touch] INT=%d\n",status);
    s += sprintf(s, "[Touch] INT=%d\n",status);
	  
    return (s - buf);
}

static ssize_t gpio_int_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return n;
}
static ssize_t T6_T15_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    int obj_addr,i;

	obj_addr=MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_GEN_COMMANDPROCESSOR_T6] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_GEN_COMMANDPROCESSOR_T6] ,obj_addr);
	 if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6,mxt),6,temp_buf))
  	 {
		for(i=0;i<6;i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 	object_type_name[MXT_GEN_COMMANDPROCESSOR_T6] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 	object_type_name[MXT_GEN_COMMANDPROCESSOR_T6] ,i,temp_buf[i]);
   			}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_GEN_COMMANDPROCESSOR_T6] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_GEN_COMMANDPROCESSOR_T6] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_GEN_COMMANDPROCESSOR_T6] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_GEN_COMMANDPROCESSOR_T6] );
   	}

	obj_addr=MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_TOUCH_KEYARRAY_T15] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_TOUCH_KEYARRAY_T15] ,obj_addr);
	 if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15,mxt),sizeof(KeyArrayOBJ),temp_buf))
  	 {
		for(i=0;i<sizeof(KeyArrayOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 	object_type_name[MXT_TOUCH_KEYARRAY_T15] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 	object_type_name[MXT_TOUCH_KEYARRAY_T15] ,i,temp_buf[i]);
   			}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_TOUCH_KEYARRAY_T15] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_TOUCH_KEYARRAY_T15] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_TOUCH_KEYARRAY_T15] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_TOUCH_KEYARRAY_T15] );
   	}
			 
    	
    return (s - buf);
}

static ssize_t T6_T15_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return n;
}
static ssize_t T38_T25_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    int obj_addr,i;
	
    obj_addr=MXT_BASE_ADDR(MXT_USER_INFO_T38, mxt);
    dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_USER_INFO_T38] ,obj_addr);
    s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_USER_INFO_T38] ,obj_addr);
    if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_USER_INFO_T38,mxt),sizeof(UserdataOBJ),temp_buf))
  	{
		for(i=0;i<sizeof(UserdataOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 	object_type_name[MXT_USER_INFO_T38] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 	object_type_name[MXT_USER_INFO_T38] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_USER_INFO_T38] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_USER_INFO_T38] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_USER_INFO_T38] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_USER_INFO_T38] );
   	}

    	obj_addr=MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_SPT_SELFTEST_T25] ,obj_addr);
       s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_SPT_SELFTEST_T25] ,obj_addr);
       if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25,mxt),sizeof(SelfTestOBJ),temp_buf))
  	{
		for(i=0;i<sizeof(SelfTestOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 object_type_name[MXT_SPT_SELFTEST_T25] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 object_type_name[MXT_SPT_SELFTEST_T25] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_SPT_SELFTEST_T25] );
			s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_SPT_SELFTEST_T25] );
		}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_SPT_SELFTEST_T25] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_SPT_SELFTEST_T25] );
   	}
	
    return (s - buf);
}

static ssize_t T38_T25_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return n;
}

static ssize_t T7_T9_T44_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    int obj_addr,i;
	
   obj_addr=MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t  address=%2d\n",
				object_type_name[MXT_GEN_POWERCONFIG_T7] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_GEN_POWERCONFIG_T7] ,obj_addr);
	if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7,mxt),sizeof(PowerOBJ),temp_buf))
	{
		for(i=0;i<sizeof(PowerOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 	object_type_name[MXT_GEN_POWERCONFIG_T7] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 			object_type_name[MXT_GEN_POWERCONFIG_T7] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
		 			object_type_name[MXT_GEN_POWERCONFIG_T7] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_GEN_POWERCONFIG_T7] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
			 	object_type_name[MXT_GEN_POWERCONFIG_T7] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				object_type_name[MXT_GEN_POWERCONFIG_T7] );
   	}

   	obj_addr=MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t  address=%2xd\n",
				object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9] ,obj_addr);
       if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9,mxt),sizeof(MTouchOBJ),temp_buf))
  	{
		for(i=0;i<sizeof(MTouchOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 	object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 	object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9] );
   	}

	obj_addr=MXT_BASE_ADDR(MXT_SPT_MESSAGECOUNT_T44, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_SPT_MESSAGECOUNT_T44] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_SPT_MESSAGECOUNT_T44] ,obj_addr);
       if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_SPT_MESSAGECOUNT_T44,mxt),1,temp_buf))
	{
		for(i=0;i<1;i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 object_type_name[MXT_SPT_MESSAGECOUNT_T44] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 object_type_name[MXT_SPT_MESSAGECOUNT_T44] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_SPT_MESSAGECOUNT_T44] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_SPT_MESSAGECOUNT_T44] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_SPT_MESSAGECOUNT_T44] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_SPT_MESSAGECOUNT_T44] );
   	}
   	
    return (s - buf);
}

static ssize_t T7_T9_T44_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    
    return n;
}
static ssize_t T8_T18_T22_T24_T27_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    int obj_addr,i;
   
   	obj_addr=MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t  address=%2d\n",
				object_type_name[MXT_GEN_ACQUIRECONFIG_T8] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_GEN_ACQUIRECONFIG_T8] ,obj_addr);
       if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8,mxt),sizeof(AcqOBJ),temp_buf))
	{
		for(i=0;i<sizeof(AcqOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 	object_type_name[MXT_GEN_ACQUIRECONFIG_T8] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 	object_type_name[MXT_GEN_ACQUIRECONFIG_T8] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_GEN_ACQUIRECONFIG_T8] );
			s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_GEN_ACQUIRECONFIG_T8] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				object_type_name[MXT_GEN_ACQUIRECONFIG_T8] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_GEN_ACQUIRECONFIG_T8] );
   	}
			 
    	obj_addr=MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_SPT_COMMSCONFIG_T18] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_SPT_COMMSCONFIG_T18] ,obj_addr);
       if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18,mxt),sizeof(ComconfigOBJ),temp_buf))
  	{
		for(i=0;i<sizeof(ComconfigOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 	object_type_name[MXT_SPT_COMMSCONFIG_T18] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 	object_type_name[MXT_SPT_COMMSCONFIG_T18] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_SPT_COMMSCONFIG_T18] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_SPT_COMMSCONFIG_T18] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_SPT_COMMSCONFIG_T18] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_SPT_COMMSCONFIG_T18] );
   	}
	 
    	obj_addr=MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t   address=%2d\n",
				object_type_name[MXT_PROCG_NOISESUPPRESSION_T22] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t  address=%2d\n",
				object_type_name[MXT_PROCG_NOISESUPPRESSION_T22] ,obj_addr);
	if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22,mxt),sizeof(NoiseOBJ),temp_buf))
  	{
		for(i=0;i<sizeof(NoiseOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 	object_type_name[MXT_PROCG_NOISESUPPRESSION_T22] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 	object_type_name[MXT_PROCG_NOISESUPPRESSION_T22] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_PROCG_NOISESUPPRESSION_T22] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_PROCG_NOISESUPPRESSION_T22] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_PROCG_NOISESUPPRESSION_T22] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_PROCG_NOISESUPPRESSION_T22] );
   	}
			 
   	obj_addr=MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t  address=%2d\n",
				object_type_name[MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24] ,obj_addr);
	if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24,mxt),sizeof(OneTGOBJ),temp_buf))
  	{
		for(i=0;i<sizeof(OneTGOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 	object_type_name[MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 	object_type_name[MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24] );
			s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 	object_type_name[MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 	object_type_name[MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24] );
   	}
	
    	
    	obj_addr=MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt);
   	dev_info(&mxt->client->dev,"Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] ,obj_addr);
       if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27,mxt),sizeof(TwoTGOBJ),temp_buf))
  	{
		for(i=0;i<sizeof(TwoTGOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 object_type_name[MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 object_type_name[MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] );
			s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] );
   	}

    return (s - buf);
}

static ssize_t T8_T18_T22_T24_T27_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return n;
}
static ssize_t T28_T40_T41_T43_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    int obj_addr,i;

   	obj_addr=MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_SPT_CTECONFIG_T28] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_SPT_CTECONFIG_T28] ,obj_addr);
       if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28,mxt),sizeof(CTEOBJ),temp_buf))
  	{
		for(i=0;i<sizeof(CTEOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 object_type_name[MXT_SPT_CTECONFIG_T28] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 object_type_name[MXT_SPT_CTECONFIG_T28] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_SPT_CTECONFIG_T28] );
			s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_SPT_CTECONFIG_T28] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_SPT_CTECONFIG_T28] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_SPT_CTECONFIG_T28] );
   	}
	

    	obj_addr=MXT_BASE_ADDR(MXT_PROCI_GRIPSUPPRESSION_T40, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t   address=%2d\n",
				object_type_name[MXT_PROCI_GRIPSUPPRESSION_T40] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_PROCI_GRIPSUPPRESSION_T40] ,obj_addr);
       if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_GRIPSUPPRESSION_T40,mxt),sizeof(GripSUPOBJ),temp_buf))
  	{
		for(i=0;i<sizeof(GripSUPOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 object_type_name[MXT_PROCI_GRIPSUPPRESSION_T40] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 object_type_name[MXT_PROCI_GRIPSUPPRESSION_T40] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_PROCI_GRIPSUPPRESSION_T40] );
			s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_PROCI_GRIPSUPPRESSION_T40] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
		object_type_name[MXT_PROCI_GRIPSUPPRESSION_T40] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_PROCI_GRIPSUPPRESSION_T40] );
   	}
	
   /* COMPAL - START - [2011/6/3  17:21] - Jamin - [Touch]: update firmware version 1.1   */
   if(mxt->device_info.major==1 && mxt->device_info.minor==1 )  //check firmware version
   {
   		obj_addr=MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt);
   		dev_info(&mxt->client->dev,"Object Table=%s  \t\t   address=%2d\n",
					object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] ,obj_addr);
		s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
					object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] ,obj_addr);
       	if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41,mxt),sizeof(PalmSUPOBJ_65ohm_fw11_v1),temp_buf))
  		{
			for(i=0;i<sizeof(PalmSUPOBJ_65ohm_fw11_v1);i++){
  	 			dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 	object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] ,i,temp_buf[i]);
					 s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
					 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] ,i,temp_buf[i]);
   			}
			dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
					 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] );
			s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
					 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] );
		}
		else
		{
			dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
					 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] );
			s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
					 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] );
   	  	}
   	}
       else{
		obj_addr=MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt);
   		dev_info(&mxt->client->dev,"Object Table=%s  \t\t   address=%2d\n",
					object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] ,obj_addr);
		s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
					object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] ,obj_addr);
       	if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41,mxt),sizeof(PalmSUPOBJ),temp_buf))
  		{
			for(i=0;i<sizeof(PalmSUPOBJ);i++){
  	 			dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
					 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] ,i,temp_buf[i]);
				 	s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 	object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] ,i,temp_buf[i]);
   		}
			dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
					 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] );
			s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
					 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] );
		}
		else
		{
			dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
					 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] );
			s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
					 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] );
   		}
       }
	 /* COMPAL - END */

	obj_addr=MXT_BASE_ADDR(MXT_SPT_DIGITIZER_T43, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_SPT_DIGITIZER_T43] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_SPT_DIGITIZER_T43] ,obj_addr);
	 if(0<mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_SPT_DIGITIZER_T43,mxt),sizeof(DigitizerOBJ),temp_buf))
	 {
		for(i=0;i<sizeof(DigitizerOBJ);i++){
  	 		dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d \n",
				 	object_type_name[MXT_SPT_DIGITIZER_T43] ,i,temp_buf[i]);
			s+=sprintf(s, "Read Table=%s \t address[%d]=%d \n" ,
				 	object_type_name[MXT_SPT_DIGITIZER_T43] ,i,temp_buf[i]);
   		}
		dev_info(&mxt->client->dev, "Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_SPT_DIGITIZER_T43] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c successful \t\n" ,
				 object_type_name[MXT_SPT_DIGITIZER_T43] );
	}
	else
	{
		dev_info(&mxt->client->dev,"Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_SPT_DIGITIZER_T43] );
		s+=sprintf(s, "maXTouch 0-004c:  Read Table=%s \t, i2c fail \t\n" ,
				 object_type_name[MXT_SPT_DIGITIZER_T43] );
   	}
    return (s - buf);
}

static ssize_t T28_T40_T41_T43_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return n;
}
static ssize_t objtable_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
     char * s = buf;
     int obj_addr;

	 //T6
	obj_addr=MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_GEN_COMMANDPROCESSOR_T6] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_GEN_COMMANDPROCESSOR_T6] ,obj_addr);
       //T7
	 obj_addr=MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t  address=%2d\n",
				object_type_name[MXT_GEN_POWERCONFIG_T7] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_GEN_POWERCONFIG_T7] ,obj_addr);
	 //T8
      obj_addr=MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t  address=%2d\n",
				object_type_name[MXT_GEN_ACQUIRECONFIG_T8] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_GEN_ACQUIRECONFIG_T8] ,obj_addr);
	 //T9
	obj_addr=MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t  address=%2xd\n",
				object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9] ,obj_addr);
	//T15 
	obj_addr=MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_TOUCH_KEYARRAY_T15] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_TOUCH_KEYARRAY_T15] ,obj_addr);
	 //T18
	 obj_addr=MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_SPT_COMMSCONFIG_T18] ,obj_addr);
	 s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_SPT_COMMSCONFIG_T18] ,obj_addr);
	 //T22
	 obj_addr=MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t   address=%2d\n",
				object_type_name[MXT_PROCG_NOISESUPPRESSION_T22] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t  address=%2d\n",
				object_type_name[MXT_PROCG_NOISESUPPRESSION_T22] ,obj_addr);
	//T24
	obj_addr=MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t  address=%2d\n",
				object_type_name[MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24] ,obj_addr);
	//T27
	obj_addr=MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt);
   	dev_info(&mxt->client->dev,"Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] ,obj_addr);
	//T28
	obj_addr=MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_SPT_CTECONFIG_T28] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_SPT_CTECONFIG_T28] ,obj_addr);
	//T40
	obj_addr=MXT_BASE_ADDR(MXT_PROCI_GRIPSUPPRESSION_T40, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t   address=%2d\n",
				object_type_name[MXT_PROCI_GRIPSUPPRESSION_T40] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_PROCI_GRIPSUPPRESSION_T40] ,obj_addr);
	//T41
	obj_addr=MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt);
   	dev_info(&mxt->client->dev,"Object Table=%s  \t\t   address=%2d\n",
				object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] ,obj_addr);
	//T43
	obj_addr=MXT_BASE_ADDR(MXT_SPT_DIGITIZER_T43, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_SPT_DIGITIZER_T43] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_SPT_DIGITIZER_T43] ,obj_addr);
	//T44
	obj_addr=MXT_BASE_ADDR(MXT_SPT_MESSAGECOUNT_T44, mxt);
   	dev_info(&mxt->client->dev, "Object Table=%s  \t\t\t   address=%2d\n",
				object_type_name[MXT_SPT_MESSAGECOUNT_T44] ,obj_addr);
	s+=sprintf(s, "Object Table=%s  \t   address=%2d\n",
				object_type_name[MXT_SPT_MESSAGECOUNT_T44] ,obj_addr);
	
  return (s - buf);
}
static ssize_t objtable_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{ 
    return n;
}
static ssize_t writecfg_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    int ret=0;
    sprintf(buf,"%s\n",str_buf);
    ret=strlen(buf)+1;
    memset(str_buf,0,sizeof(str_buf));
    return ret;
}

static ssize_t writecfg_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int mxt_table,byte,val;
    sscanf(buf, "%d%d%d",&mxt_table,&byte,&val);
    memset(str_buf,0,sizeof(str_buf));
	
      switch (mxt_table)
    	{
    	       case 6:
		{
			 if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_GEN_COMMANDPROCESSOR_T6, mxt)+byte,val))
			 {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
		break;	   
		case 7:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_GEN_POWERCONFIG_T7, mxt)+byte,val))
			 {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
		break;
		case 8:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_GEN_ACQUIRECONFIG_T8, mxt)+byte,val))
                      {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
		break;
		case 9:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+byte,val))
                     {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
             break;
		case 15:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_TOUCH_KEYARRAY_T15, mxt)+byte,val))
                    {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
		break;
		case 18:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_SPT_COMMSCONFIG_T18, mxt)+byte,val))
                     {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
             break;
		case 22:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_PROCG_NOISESUPPRESSION_T22, mxt)+byte,val))
                    {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
		break;
		case 24:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+byte,val))
                     {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}	
		break;
		case 25:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_SPT_SELFTEST_T25, mxt)+byte,val))
                      {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
		break;
		case 27:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt)+byte,val))
                     {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
		break;
		case 28:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_SPT_CTECONFIG_T28, mxt)+byte,val))
			  {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
		break;
		case 38:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_USER_INFO_T38, mxt)+byte,val))
                      {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
		break;
		case 40:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_PROCI_GRIPSUPPRESSION_T40, mxt)+byte,val))
                    {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
		break;
		case 41:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_PROCI_PALMSUPPRESSION_T41, mxt)+byte,val))
                      {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		}
		break;
             case 43:
		{
			if(0==mxt_write_byte(mxt->client,MXT_BASE_ADDR( MXT_SPT_DIGITIZER_T43, mxt)+byte,val))
                     {
			 pr_info("[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d, I2C successful",mxt_table,byte,val);
			 }
			 else{
			 	pr_info( "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 	sprintf(str_buf, "[Touch]: Write Table=%d,buf[%d]=%d , I2C fail",mxt_table,byte,val);
			 }
		} 
		break;
		default:
			printk("[Touch]: not support write Table=%d \n",mxt_table);
			sprintf(str_buf, "[Touch]: not support write Table=%d \n",mxt_table);
			break;
	}
    return n;
}

static ssize_t workqueue_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "%d\n",Debug);
    return (s - buf);
}

static ssize_t workqueue_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
   int ibuf;
    ibuf = myatoi(buf);
    if(ibuf > 0 ){
		cancel_delayed_work(&mxt->dwork);
		schedule_delayed_work(&mxt->dwork, 0);
		printk("[Touch]: do work queue enable \n");
    }
    else 
    	{
    		cancel_delayed_work_sync(&mxt->dwork);
		 printk("[Touch]: do work queue disable \n");
    }
    return n;
}

static ssize_t irq_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "%d\n",Debug);
    return (s - buf);
}

static ssize_t irq_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int ibuf;
    ibuf = myatoi(buf);
    if(ibuf > 0 ){
		enable_irq(mxt->irq);
		printk("[Touch]: do enable irq \n");
    }
   else {
   	    disable_irq(mxt->irq);
	    printk("[Touch] : do disable irq \n");
   	}
    return n;
}

static ssize_t Disable_AntiTouchPalm_Calibrate_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s += sprintf(s, "%d\n",Debug);
    return (s - buf);
}

static ssize_t Disable_AntiTouchPalm_Calibrate_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int ibuf,retry=0;
    u8 resume_done=1;
    ibuf = myatoi(buf);
    if(ibuf > 0 ){
	do
	{
	     retry++;
	     mxt_read_block(mxt->client, 274, 1,&resume_done);  //T38[10]
	     if(resume_done==0)
	     {
	             dev_info(&mxt->client->dev, "Got unlock event ,after 20 second disable auto calibration\n");
			mxt_write_byte(mxt->client,260, 1);  //do calibration 
			msleep(100);
		
     			#ifdef ENABLE_AUTO_CALIBRATION
	 		//fix bug 3219, after 20 second disable auto calibration
			mod_timer( &mxt->cal_timer, jiffies+HZ*20);
    			#endif
		  break;
	     	}
		else
		{
		  dev_info(&mxt->client->dev, "Got unlock event , but do late resume not yet so retry %d \n",retry);
		}
	   }while(retry<5);
	 if(retry>=5)
	 	dev_info(&mxt->client->dev, "Got unlock event , but retry over %d \n",retry);
	 	
    }
	
    return n;
}

/* COMPAL - START - [2011/5/24  20:25] - Jamin - [Touch]: add sys node for read delta and refs data    */
static ssize_t Delta_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    int ret=0;
    int i=0;
    unsigned char *buffer_data=NULL;	

   //set position = 0 , read all data first	
    mxt->current_debug_datap = 0;
	
    	mxt->debug_data =kmalloc(mxt->device_info.num_nodes * sizeof(u16), GFP_KERNEL);
    	if (mxt->debug_data == NULL)
	{
	 sprintf(buf,"[Touch] mxt->debug_data memory allocate fail ,mxt->device_info.num_nodes=%d \n",mxt->device_info.num_nodes);
	 ret=strlen(buf)+1;
	 return ret;
	}
		
	buffer_data =kmalloc(mxt->device_info.num_nodes * 10, GFP_KERNEL);
    	if (buffer_data == NULL)
	{
	 sprintf(buf,"[Touch] buffer_data memory allocate fail ,mxt->device_info.num_nodes=%d \n",mxt->device_info.num_nodes);
	 ret=strlen(buf)+1;
	 return ret;
	}

	for (i = 0; i < mxt->device_info.num_nodes; i++)
			mxt->debug_data[i] = 7777;
	memset(buffer_data,0,sizeof(buffer_data));
	
   ret=debug_data_read(mxt, buffer_data,mxt->device_info.num_nodes*2,NULL,MXT_CMD_T6_DELTAS_MODE);
   if(ret <= 0)
   {
    sprintf(buf,"[Touch]  debug_data_read fail \n");
    ret=strlen(buf)+1;
    return ret;
   }

   if(mxt->current_delta_datap==0){
   	sprintf(buf,"%s\n",buffer_data);
   	ret=strlen(buf)+1;
   }
   else
   {
     //because location not 0 , so retry read data
     mxt->current_debug_datap=mxt->current_delta_datap;
      ret=debug_data_read(mxt, buffer_data,mxt->device_info.num_nodes*2,NULL,MXT_CMD_T6_DELTAS_MODE);
   	if(ret <= 0)
   	{
    	sprintf(buf,"[Touch]  debug_data_read fail \n");
    	ret=strlen(buf)+1;
    	return ret;
   	}
	sprintf(buf,"%s\n",buffer_data);
   	ret=strlen(buf)+1;
   }	
   
   kfree(mxt->debug_data);
   kfree(buffer_data); 
   return ret;
}
static ssize_t Delta_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int location;
    sscanf(buf, "%d",&location);
    if(location >= mxt->device_info.num_nodes)
		location=mxt->device_info.num_nodes-1;
   else if ( location < 0 )
   	     location=0;
   
   mxt->current_delta_datap=location;
   
    return n;
}
static ssize_t Refs_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    int ret=0;
    int i=0;
    unsigned char *buffer_data=NULL;	

   //set position = 0 , read all data first	
    mxt->current_debug_datap = 0;
	
    	mxt->debug_data =kmalloc(mxt->device_info.num_nodes * sizeof(u16), GFP_KERNEL);
    	if (mxt->debug_data == NULL)
	{
	 sprintf(buf,"[Touch] mxt->debug_data memory allocate fail ,mxt->device_info.num_nodes=%d \n",mxt->device_info.num_nodes);
	 ret=strlen(buf)+1;
	 return ret;
	}
		
	buffer_data =kmalloc(mxt->device_info.num_nodes * 10, GFP_KERNEL);
    	if (buffer_data == NULL)
	{
	 sprintf(buf,"[Touch] buffer_data memory allocate fail ,mxt->device_info.num_nodes=%d \n",mxt->device_info.num_nodes);
	 ret=strlen(buf)+1;
	 return ret;
	}

	for (i = 0; i < mxt->device_info.num_nodes; i++)
			mxt->debug_data[i] = 7777;
	memset(buffer_data,0,sizeof(buffer_data));
	
   ret=debug_data_read(mxt, buffer_data,mxt->device_info.num_nodes*2,NULL,MXT_CMD_T6_REFERENCES_MODE);
   if(ret <= 0)
   {
    sprintf(buf,"[Touch]  debug_data_read fail \n");
    ret=strlen(buf)+1;
    return ret;
   }

   if(mxt->current_refs_datap==0){
   	sprintf(buf,"%s\n",buffer_data);
   	ret=strlen(buf)+1;
   }
   else
   {
     //because location not 0 , so retry read data
     mxt->current_debug_datap=mxt->current_refs_datap;
      ret=debug_data_read(mxt, buffer_data,mxt->device_info.num_nodes*2,NULL,MXT_CMD_T6_REFERENCES_MODE);
   	if(ret <= 0)
   	{
    	sprintf(buf,"[Touch]  debug_data_read fail \n");
    	ret=strlen(buf)+1;
    	return ret;
   	}
	sprintf(buf,"%s\n",buffer_data);
   	ret=strlen(buf)+1;
   }	
   
   kfree(mxt->debug_data);
   kfree(buffer_data); 
   return ret;
}

static ssize_t Refs_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    int location;
    sscanf(buf, "%d",&location);
    if(location >= mxt->device_info.num_nodes)
		location=mxt->device_info.num_nodes-1;
   else if ( location < 0 )
   	     location=0;
   
   mxt->current_refs_datap=location;
   
    return n;
}
/* COMPAL - END */

#if 0
/* COMPAL - START - [2011/6/3  16:0] - Jamin - [Touch]: update firmware version 1.1   */
static ssize_t FWver_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char * s = buf;
    s +=sprintf(buf,"Firmware version [%d.%d] \n",
		 mxt->device_info.major,
		 mxt->device_info.minor);
    return (s - buf);
}

static ssize_t FWver_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return n;
}


static ssize_t update_firmware_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    return 0;
}

static ssize_t update_firmware_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
       unsigned int ibuf=0;
	ibuf = myatoi(buf);

if(ibuf>0)
{	
    	if( (mxt->device_info.major!=1) ||( mxt->device_info.minor!=1))
   	{
		//cancel work queue and disable irq
		cancel_delayed_work_sync(&mxt->dwork);
		disable_irq(mxt->irq);

		#ifdef ENABLE_AUTO_CALIBRATION
       	cancel_delayed_work_sync(&mxt->dcal_work);
       	del_timer(&mxt->cal_timer);
      	 	#endif
   		msleep(100);
  		update_firmware(true);
		//write machine version configure
		if(((mxt->check_resistance==65) && (mxt->check_version==2)))
		{
	  		Atmel_WriteAllConfig(mxt,TOUCH_65OHM_V2);
		}
		else if( ((mxt->check_resistance==65) && (mxt->check_version==1)))
		{
			Atmel_WriteAllConfig(mxt,TOUCH_65OHM_V1);
		}
		else
		{
	    		Atmel_WriteAllConfig(mxt,TOUCH_30OHM);
		}
		Atmel_WriteBack(mxt);
		enable_irq(mxt->irq);
    	}
	else
	{
		//detect firmware 1.1
	 	dev_info(&mxt->client->dev, "reject upgrade FW, because version [%d.%d] is the latest \n",mxt->device_info.major,mxt->device_info.minor);
	}
 }
return n;
}
/* COMPAL - END */
#endif

static struct kobject *touchdebug_kobj;

#define debug_attr(_name) \
static struct kobj_attribute _name##_attr = { \
    .attr = { \
        .name = __stringify(_name), \
        .mode = 0660, \
    }, \
    .show = _name##_show, \
    .store = _name##_store, \
}

debug_attr(hbyte);
debug_attr(lbyte);
debug_attr(rlen);
debug_attr(val);
debug_attr(debugmsg);
debug_attr(softreset);
debug_attr(hardreset);
debug_attr(restore);
debug_attr(writeback);
debug_attr(T6_T15);
debug_attr(T38_T25);
debug_attr(T7_T9_T44);
debug_attr(T8_T18_T22_T24_T27);
debug_attr(T28_T40_T41_T43);
debug_attr(objtable);
debug_attr(suspendcount);
debug_attr(writecfg);
debug_attr(gpio_int);
debug_attr(gpio_rst);
debug_attr(irq);
debug_attr(workqueue);
debug_attr(Disable_AntiTouchPalm_Calibrate);
/* COMPAL - START - [2011/5/24  20:26] - Jamin - [Touch]: add sys node for read delta and refs data    */
debug_attr(Delta);
debug_attr(Refs);
/* COMPAL - END */
//debug_attr(FWver);
//debug_attr(update_firmware);

static struct attribute * g[] = {
    &hbyte_attr.attr,
    &lbyte_attr.attr,
    &rlen_attr.attr,
    &val_attr.attr,
    &debugmsg_attr.attr,
    /* COMPAL - START - [2011/3/24  11:25] - Jamin - [Touch]: add reset node  */
    &softreset_attr.attr,
    &hardreset_attr.attr,
    &restore_attr.attr,
    &writeback_attr.attr,
    &T6_T15_attr.attr,
    &T38_T25_attr.attr,
    &T7_T9_T44_attr.attr,
    &T8_T18_T22_T24_T27_attr.attr,
    &T28_T40_T41_T43_attr.attr,
    &writecfg_attr.attr,
    &gpio_int_attr.attr,
    &gpio_rst_attr.attr,
    &irq_attr.attr,
    &workqueue_attr.attr,
    &Disable_AntiTouchPalm_Calibrate_attr.attr,
    &Delta_attr.attr,
    &Refs_attr.attr,
    &objtable_attr.attr,
    &suspendcount_attr.attr,
    //&FWver_attr.attr,
   //&update_firmware_attr.attr,
    /* COMPAL - END */
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = g,
};
/* COMPAL - END */

ssize_t debug_data_read(struct mxt_data *mxt, char *buf, size_t count,
			loff_t *ppos, u8 debug_command)
{
	int i;
	u16 *data;
	u16 diagnostics_reg;
	int offset = 0;
	int size;
	int read_size;
	int error;
	char *buf_start;
	u16 debug_data_addr;
	u16 page_address;
	u8 page;
	u8 debug_command_reg;

	data = mxt->debug_data;
	if (data == NULL)
		return -EIO;

	/* If first read after open, read all data to buffer. */
	if (mxt->current_debug_datap == 0) {

		diagnostics_reg = MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6,
						mxt) + MXT_ADR_T6_DIAGNOSTIC;
		if (count > (mxt->device_info.num_nodes * 2))
			count = mxt->device_info.num_nodes;

		debug_data_addr = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) +
		    MXT_ADR_T37_DATA;
		page_address = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) +
		    MXT_ADR_T37_PAGE;
		error = mxt_read_block(mxt->client, page_address, 1, &page);
		if (error < 0)
			return error;
		mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
		while (page != 0) {
			error = mxt_write_byte(mxt->client,
					       diagnostics_reg,
					       MXT_CMD_T6_PAGE_DOWN);
			if (error < 0)
				return error;
			/* Wait for command to be handled; when it has, the
			   register will be cleared. */
			debug_command_reg = 1;
			while (debug_command_reg != 0) {
				error = mxt_read_block(mxt->client,
						       diagnostics_reg, 1,
						       &debug_command_reg);
				if (error < 0)
					return error;
				mxt_debug(DEBUG_TRACE,
					  "Waiting for debug diag command "
					  "to propagate...\n");

			}
			error = mxt_read_block(mxt->client, page_address, 1,
					       &page);
			if (error < 0)
				return error;
			mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
		}

		/*
		 * Lock mutex to prevent writing some unwanted data to debug
		 * command register. User can still write through the char
		 * device interface though. TODO: fix?
		 */

		mutex_lock(&mxt->debug_mutex);
		/* Configure Debug Diagnostics object to show deltas/refs */
		error = mxt_write_byte(mxt->client, diagnostics_reg,
				       debug_command);

		/* Wait for command to be handled; when it has, the
		 * register will be cleared. */
		debug_command_reg = 1;
		while (debug_command_reg != 0) {
			error = mxt_read_block(mxt->client,
					       diagnostics_reg, 1,
					       &debug_command_reg);
			if (error < 0)
				return error;
			mxt_debug(DEBUG_TRACE, "Waiting for debug diag command "
				  "to propagate...\n");

		}

		if (error < 0) {
			printk(KERN_WARNING
			       "Error writing to maXTouch device!\n");
			return error;
		}

		size = mxt->device_info.num_nodes * sizeof(u16);

		while (size > 0) {
			read_size = size > 128 ? 128 : size;
			mxt_debug(DEBUG_TRACE,
				  "Debug data read loop, reading %d bytes...\n",
				  read_size);
			error = mxt_read_block(mxt->client,
					       debug_data_addr,
					       read_size,
					       (u8 *) &data[offset]);
			if (error < 0) {
				printk(KERN_WARNING
				       "Error reading debug data\n");
				goto error;
			}
			offset += read_size / 2;
			size -= read_size;

			/* Select next page */
			error = mxt_write_byte(mxt->client, diagnostics_reg,
					       MXT_CMD_T6_PAGE_UP);
			if (error < 0) {
				printk(KERN_WARNING
				       "Error writing to maXTouch device!\n");
				goto error;
			}
		}
		mutex_unlock(&mxt->debug_mutex);
	}

	buf_start = buf;
	i = mxt->current_debug_datap;

	while (((buf - buf_start) < (count - 6)) &&
	       (i < mxt->device_info.num_nodes)) {

		mxt->current_debug_datap++;
		if (debug_command == MXT_CMD_T6_REFERENCES_MODE)
			buf += sprintf(buf, "%d: %5d ", i,
				       (u16) le16_to_cpu(data[i]));
		else if (debug_command == MXT_CMD_T6_DELTAS_MODE)
			buf += sprintf(buf, "%d: %5d ", i,
				       (s16) le16_to_cpu(data[i]));

		if(i%10==0 && i >0)
		 	buf += sprintf(buf, "\n");

		i++;
	} 

	return buf - buf_start;
 error:
	mutex_unlock(&mxt->debug_mutex);
	return error;
}

ssize_t deltas_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos,
			       MXT_CMD_T6_DELTAS_MODE);
}

ssize_t refs_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos,
			       MXT_CMD_T6_REFERENCES_MODE);
}

int debug_data_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	int i;
	mxt = inode->i_private;
	if (mxt == NULL)
		return -EIO;
	mxt->current_debug_datap = 0;
	mxt->debug_data = kmalloc(mxt->device_info.num_nodes * sizeof(u16),
				  GFP_KERNEL);
	if (mxt->debug_data == NULL)
		return -ENOMEM;

	for (i = 0; i < mxt->device_info.num_nodes; i++)
		mxt->debug_data[i] = 7777;

	file->private_data = mxt;
	return 0;
}

int debug_data_release(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = file->private_data;
	kfree(mxt->debug_data);
	return 0;
}

const struct file_operations delta_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = deltas_read,
};

const struct file_operations refs_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = refs_read,
};

int mxt_memory_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = container_of(inode->i_cdev, struct mxt_data, cdev);
	if (mxt == NULL)
		return -EIO;
	file->private_data = mxt;
	return 0;
}

int mxt_message_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = container_of(inode->i_cdev, struct mxt_data, cdev_messages);
	if (mxt == NULL)
		return -EIO;
	file->private_data = mxt;
	return 0;
}

ssize_t mxt_memory_read(struct file *file, char *buf, size_t count,
			loff_t *ppos)
{
	int i;
	struct mxt_data *mxt;

	mxt = file->private_data;
	if (mxt->valid_ap) {
		mxt_debug(DEBUG_TRACE, "Reading %d bytes from current ap\n",
			  (int)count);
		i = mxt_read_block_wo_addr(mxt->client, count, (u8 *) buf);
	} else {
		mxt_debug(DEBUG_TRACE, "Address pointer changed since set;"
			  "writing AP (%d) before reading %d bytes",
			  mxt->address_pointer, (int)count);
		i = mxt_read_block(mxt->client, mxt->address_pointer, count,
				   buf);
	}

	return i;
}

ssize_t mxt_memory_write(struct file *file, const char *buf, size_t count,
			 loff_t *ppos)
{
	int i;
	int whole_blocks;
	int last_block_size;
	struct mxt_data *mxt;
	u16 address;

	mxt = file->private_data;
	address = mxt->address_pointer;

	mxt_debug(DEBUG_TRACE, "mxt_memory_write entered\n");
	whole_blocks = count / I2C_PAYLOAD_SIZE;
	last_block_size = count % I2C_PAYLOAD_SIZE;

	for (i = 0; i < whole_blocks; i++) {
		mxt_debug(DEBUG_TRACE, "About to write to %d...", address);
		mxt_write_block(mxt->client, address, I2C_PAYLOAD_SIZE,
				(u8 *) buf);
		address += I2C_PAYLOAD_SIZE;
		buf += I2C_PAYLOAD_SIZE;
	}

	mxt_write_block(mxt->client, address, last_block_size, (u8 *) buf);

	return count;
}

static int mxt_ioctl(struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	int retval;
	struct mxt_data *mxt;

	retval = 0;
	mxt = file->private_data;

	switch (cmd) {
	case MXT_SET_ADDRESS:
		retval = mxt_write_ap(mxt, (u16) arg);
		if (retval >= 0) {
			mxt->address_pointer = (u16) arg;
			mxt->valid_ap = 1;
		}
		break;
	case MXT_RESET:
		retval = mxt_write_byte(mxt->client,
					MXT_BASE_ADDR
					(MXT_GEN_COMMANDPROCESSOR_T6,
					 mxt) + MXT_ADR_T6_RESET, 1);
		break;
	case MXT_CALIBRATE:
		retval = mxt_write_byte(mxt->client,
					MXT_BASE_ADDR
					(MXT_GEN_COMMANDPROCESSOR_T6,
					 mxt) + MXT_ADR_T6_CALIBRATE, 1);

		break;
	case MXT_BACKUP:
		retval = mxt_write_byte(mxt->client,
					MXT_BASE_ADDR
					(MXT_GEN_COMMANDPROCESSOR_T6,
					 mxt) + MXT_ADR_T6_BACKUPNV,
					MXT_CMD_T6_BACKUP);
		break;
	case MXT_NONTOUCH_MSG:
		mxt->nontouch_msg_only = 1;
		break;
	case MXT_ALL_MSG:
		mxt->nontouch_msg_only = 0;
		break;
	default:
		return -EIO;
	}

	return retval;
}

/*
 * Copies messages from buffer to user space.
 *
 * NOTE: if less than (mxt->message_size * 5 + 1) bytes requested,
 * this will return 0!
 *
 */
ssize_t mxt_message_read(struct file *file, char *buf, size_t count,
			 loff_t *ppos)
{
	int i;
	struct mxt_data *mxt;
	char *buf_start;

	mxt = file->private_data;
	if (mxt == NULL)
		return -EIO;
	buf_start = buf;

	mutex_lock(&mxt->msg_mutex);
	/* Copy messages until buffer empty, or 'count' bytes written */
	while ((mxt->msg_buffer_startp != mxt->msg_buffer_endp) &&
	       ((buf - buf_start) < (count - 5 * mxt->message_size - 1))) {

		for (i = 0; i < mxt->message_size; i++) {
			buf += sprintf(buf, "[%2X] ",
				       *(mxt->messages + mxt->msg_buffer_endp *
					 mxt->message_size + i));
		}
		buf += sprintf(buf, "\n");
		if (mxt->msg_buffer_endp < MXT_MESSAGE_BUFFER_SIZE)
			mxt->msg_buffer_endp++;
		else
			mxt->msg_buffer_endp = 0;
	}
	mutex_unlock(&mxt->msg_mutex);
	return buf - buf_start;
}

const struct file_operations mxt_message_fops = {
	.owner = THIS_MODULE,
	.open = mxt_message_open,
	.read = mxt_message_read,
};

const struct file_operations mxt_memory_fops = {
	.owner = THIS_MODULE,
	.open = mxt_memory_open,
	.read = mxt_memory_read,
	.write = mxt_memory_write,
	.unlocked_ioctl = mxt_ioctl,
};

/* Writes the address pointer (to set up following reads). */

int mxt_write_ap(struct mxt_data *mxt, u16 ap)
{
	struct i2c_client *client;
	__le16 le_ap = cpu_to_le16(ap);
	client = mxt->client;
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	if (i2c_master_send(client, (u8 *) &le_ap, 2) == 2) {
		mxt_debug(DEBUG_TRACE, "Address pointer set to %d\n", ap);
		return 0;
	} else {
		mxt_debug(DEBUG_INFO, "Error writing address pointer!\n");
		return -EIO;
	}
}

/* Calculates the 24-bit CRC sum. */
static u32 CRC_24(u32 crc, u8 byte1, u8 byte2)
{
	static const u32 crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = ((((u16) byte2) << 8u) | byte1);
	result = ((crc << 1u) ^ data_word);
	if (result & 0x1000000)
		result ^= crcpoly;
	return result;
}

#ifdef ENABLE_AUTO_CALIBRATION
static void mxt_calibrate_work_func(struct work_struct *work)
{
    dev_info(&mxt->client->dev,"mxt_calibrate_work_func ...+ \n");	
    mxt_write_byte(mxt->client, 337, 0);  //Disable Anti touch and Palm
    mxt_write_byte(mxt->client, 338, 1);
    mxt_write_byte(mxt->client, 339, 0);
    mxt_write_byte(mxt->client, 340, 0);
    dev_info(&mxt->client->dev,"mxt_calibrate_work_func ...- \n");
}
static void mxt_calibrate_timer_func(unsigned long arg)
{
    struct mxt_data *mxt = (struct mxt_data *)arg;
    dev_info(&mxt->client->dev,"mxt_calibrate_timer_func ...+ \n");

    //start work queue
   cancel_delayed_work(&mxt->dcal_work);
   schedule_delayed_work(&mxt->dcal_work, 0);
	
    dev_info(&mxt->client->dev,"mxt_calibrate_timer_func ...- \n");
}
#endif

#if 0
/* COMPAL - START - [2011/6/20  16:19] - Jamin - [Touch]: update firmware 1.1  */
int update_firmware(bool do_reset)
{
	unsigned char boot_status;
	unsigned char boot_ver;
	unsigned char retry_cnt;
	unsigned long int character_position;
	unsigned int frame_count;
	unsigned int frame_size;
	unsigned int next_frame;
	unsigned int crc_error_count;
	unsigned int size1,size2;
	unsigned char unlock_command[2]={0xdc,0xaa};
	
	      dev_info(&mxt->client->dev, "update firmware 1.1 ... + \n");
		unsigned char  *firmware_data=NULL;
		firmware_data = mxt1386_firmware;
		boot_status=0; 
		crc_error_count = 0;
		character_position = 0;
		next_frame = 0;
             frame_size=0;
		
	if(do_reset==true)
	{
		 //let touch enter bootloader mode
	      	if(mxt_write_byte(mxt->client,258,0xa5)==-EIO)
	      	{
		       dev_info(&mxt->client->dev, "Enter Bootloader mode  fail\n");
			 goto error;
		}
		msleep(100);
	}
	
	for(retry_cnt = 0; retry_cnt < 30; retry_cnt++)
	{
		if((mxt_boot_read(0,1,&boot_status) == I2C_BOOT_READ_OK) && (boot_status & 0xC0) == 0xC0) 
		{
				boot_ver = boot_status & 0x3F;
				crc_error_count = 0;
				character_position = 0;
				next_frame = 0;
				frame_count=1;
				//dev_info(&mxt->client->dev,"TSP boot status is %x  stage 1 \n", boot_status);
				dev_info(&mxt->client->dev,"WAITING BOOTLOAD CMD... \n");
			       dev_info(&mxt->client->dev, "got bootloader version %d \n",boot_ver);
                        
				while(1)
				{ 
				        boot_status=0;
					  msleep(60); 
					  
					  if(mxt_boot_read(0,1,&boot_status) == I2C_BOOT_READ_FAILED)
					  {
						    dev_info(&mxt->client->dev,"read boot status fail \n");
					           goto error;
					 }
				
					 retry_cnt  = 0;
					// dev_info(&mxt->client->dev,"TSP boot status is %x   stage 2 \n", boot_status);
						if((boot_status & WAITING_BOOTLOAD_COMMAND) == WAITING_BOOTLOAD_COMMAND)
						{
						       dev_info(&mxt->client->dev,"WAITING BOOTLOAD CMD... \n");									
							if(mxt_boot_write(0,unlock_command,sizeof(unlock_command))==I2C_BOOT_WRITE_OK)
							{
                                                     mdelay(10);
								dev_info(&mxt->client->dev,"unlock device success\n"); 
							}
							else
							{
							 dev_info(&mxt->client->dev,"unlock device fail \n");
							 goto error;
							}
						}
						else if((boot_status & 0xC0) == WAITING_FRAME_DATA)
						{
							 /* Add 2 to frame size, as the CRC bytes are not included */
							size1 =  *(firmware_data+character_position);
							size2 =  *(firmware_data+character_position+1)+2;
							frame_size = (size1<<8) + size2;
							#if 0
			                           dev_info(&mxt->client->dev,"WAITING FRAME DATA... \n");
							dev_info(&mxt->client->dev,"Frame size:%d\n", frame_size);
							dev_info(&mxt->client->dev,"Firmware pos:%d\n", character_position);
							#endif
							/* Exit if frame data size is zero */
							if( 0 == frame_size )
							{
								dev_info(&mxt->client->dev,"0 == frame_size\n");
								goto error;
							}
							next_frame = 1;
							mxt_boot_write(0,(firmware_data +character_position),frame_size);
							mdelay(10);
							 #if 0
							dev_info(&mxt->client->dev,".");
							//dump write data
							unsigned long int position;
							unsigned char *p=firmware_data +character_position;
                                               int i;
							for(i=0;i<frame_size;i++)
                                               {
                                                 printk("%x.",p[i]);
							if(i%8==0)
								printk("\n");					 
                                               }
							#endif
						}
						else if(boot_status == FRAME_CRC_CHECK)
						{
							dev_info(&mxt->client->dev,"FRAME %d CRC CHECK \n",frame_count);
						}
						else if(boot_status == FRAME_CRC_PASS)
						{
							if( next_frame == 1)
							{
								dev_info(&mxt->client->dev,"FRAME %d CRC PASS\n\n",frame_count);
								frame_count++;
								character_position += frame_size;
								next_frame = 0;
                                                     //firmware 1.1 total frame 235
								if(frame_count>MXT_TOTAL_FRAME_SIZE)
											goto success;
							}
							else {
								dev_info(&mxt->client->dev,"next_frame != 1\n");
                                                      goto error; 
							}
							
							
						}
						else if(boot_status  == FRAME_CRC_FAIL)
						{
							dev_info(&mxt->client->dev,"FRAME CRC FAIL %d \n\n",crc_error_count);
							crc_error_count++;
						}
						if(crc_error_count >= 10)
						{
							dev_info(&mxt->client->dev,"check frame crc fail over %d \n",crc_error_count);
							goto error;
						}
			                  
					}
		 }
		 else
		{ 
			dev_info(&mxt->client->dev,"read bootloader status fail retry %d !!!\n",retry_cnt);
		}
	}

	if(retry_cnt>=30)
	{
		dev_info(&mxt->client->dev,"read bootloader status over retry %d !!!\n",retry_cnt);
	       goto error;
	}
	
success:	
	   //estimate must delay 5.5 second
          retry_cnt=0;
	   do
          {
            retry_cnt++;
	     msleep(100);
	   }while(((mxt->read_chg() !=0)  && (retry_cnt<100)) );
	   
	 if(retry_cnt >= 100)
		dev_info(&mxt->client->dev,"change pin keep hight !!\n");
	 else
	 	dev_info(&mxt->client->dev,"change pin low and count=%d !!\n",retry_cnt);
	 	
	
	dev_info(&mxt->client->dev,"!!!Update FW finish \n");
	dev_info(&mxt->client->dev,"update firmware 1.1 ... - \n");
	return UPDATE_FW_OK;
	
  error:
  dev_info(&mxt->client->dev,"!!!Update FW fail \n");
  dev_info(&mxt->client->dev,"update firmware 1.1 ... - \n");
   return UPDATE_FW_FAILED;
}
u8 mxt_boot_read(u16 start, u8 size, u8 *mem)
{
	struct i2c_msg rmsg;
	int retry=0;

	rmsg.addr=MXT_BOOT_ADDRESS;
	rmsg.flags = I2C_M_RD;
	rmsg.len = size;
	rmsg.buf = mem;

	for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {
		if ( i2c_transfer(mxt->client->adapter, &rmsg, 1)==I2C_BOOT_READ_OK)
			return I2C_BOOT_READ_OK;
		mdelay(10);
	}
	if (retry == I2C_RETRY_COUNT) {
		printk(KERN_ERR "mxt_boot_read retry over %d\n",I2C_RETRY_COUNT);
		return I2C_BOOT_READ_FAILED;
	}
    return I2C_BOOT_READ_FAILED;
}

int mxt_boot_write(u16 reg, u8 *read_val, unsigned int len)
{
	struct i2c_msg wmsg;
	unsigned char data[MXT_MAX_FRAME_SIZE];
	int i,retry=0;

	if(len+2 > MXT_MAX_FRAME_SIZE)
	{
		printk("[TSP][ERROR] %s() data length error\n", __FUNCTION__);
		return -ENODEV;
	}
	
	wmsg.addr = MXT_BOOT_ADDRESS;
	wmsg.flags = 0;
	wmsg.len = len;
	wmsg.buf = data;

	for (i = 0; i < len; i++)
	{
		data[i] = *(read_val+i);
	}


	for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {
		if (i2c_transfer(mxt->client->adapter, &wmsg,1)>=0)
			return I2C_BOOT_WRITE_OK;
		mdelay(10);
	}
	if (retry == I2C_RETRY_COUNT) {
		printk(KERN_ERR "mxt_boot_write retry over %d\n",I2C_RETRY_COUNT);
		return I2C_BOOT_WRITE_FAILED;
	}
    return I2C_BOOT_WRITE_FAILED;
}
/* COMPAL - END */
#endif

/* COMPAL - START - [2011/3/3  9:40] - Jamin - [Touch]: enable atmel touch  */
  static void Atmel_WriteBack(struct mxt_data *mxt)
{
      unsigned char  ComOBJ[6] = { 0, 85, 0, 0, 0, 0};
      dev_info(&mxt->client->dev,"Dangerous!!!  Do Write Back \n");
      mxt_write_block(mxt->client,258,sizeof(ComOBJ),ComOBJ); //t6
      msleep(150);
}

  static void Atmel_WriteAllConfig(struct mxt_data *mxt,int version)
{
   //for firmware 1.1 objtable table address
   //dev_info(&mxt->client->dev, "Atmel_WriteAllConfig()...+ \n");
    	if(-EIO==mxt_write_block(mxt->client,375,sizeof(KeyArrayOBJ),KeyArrayOBJ)) //T15
    	{
    	   dev_info(&mxt->client->dev, "Write Object Table=TOUCH_KEYARRAY_T15 fail\n");
    	}	   
    	if(-EIO==mxt_write_block(mxt->client,397,sizeof(ComconfigOBJ),ComconfigOBJ))//T18
    	{
    	  dev_info(&mxt->client->dev, "Write Object Table=SPT_COMMSCONFIG_T18 fail\n");
    	}	  
	if(-EIO==mxt_write_block(mxt->client,416,sizeof(OneTGOBJ),OneTGOBJ)) //T24
	{
	  dev_info(&mxt->client->dev, "Write Object Table=PROCI_ONETOUCHGESTUREPROCESSOR_T24 fail\n");
	}
	if(-EIO==mxt_write_block(mxt->client,435,sizeof(SelfTestOBJ),SelfTestOBJ)) //T25
	{
	    dev_info(&mxt->client->dev, "Write Object Table=SPT_SELFTEST_T25 fail\n");
	}
	if(-EIO==mxt_write_block(mxt->client,449,sizeof(TwoTGOBJ),TwoTGOBJ)) //T27
	{
	   dev_info(&mxt->client->dev, "Write Object Table=PROCI_TWOTOUCHGESTUREPROCESSOR_T27 fail\n");
	}  
	if(-EIO==mxt_write_block(mxt->client,456,sizeof(CTEOBJ),CTEOBJ)) //T28
	{
	  dev_info(&mxt->client->dev, "Write Object Table=SPT_CTECONFIG_T28 fail\n");
	}  
	if(-EIO==mxt_write_block(mxt->client,462,sizeof(GripSUPOBJ),GripSUPOBJ)) //T40
	{
	 dev_info(&mxt->client->dev, "Write Object Table=PROCI_GRIPSUPPRESSION_T40 fail\n");
	}  
	if(-EIO==mxt_write_block(mxt->client,474,sizeof(DigitizerOBJ),DigitizerOBJ)) //T43
	{
	 dev_info(&mxt->client->dev, "Write Object Table=SPT_DIGITIZER_T43 fail\n");
	}  	
	
   switch (version)
   {
     case TOUCH_30OHM:
     	{
		dev_info(&mxt->client->dev,"Write touch 30 ohm all configure  \n");
	 	if(-EIO==mxt_write_block(mxt->client,328,sizeof(PowerOBJ),PowerOBJ))	  //T7
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=GEN_POWERCONFIG_T7 fail\n");
		}  
	 	if(-EIO==mxt_write_block(mxt->client,331,sizeof(AcqOBJ),AcqOBJ)) //T8
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=GEN_ACQUIRECONFIG_T8 fail\n");
		}  
	 	if(-EIO==mxt_write_block(mxt->client,341,sizeof(MTouchOBJ),MTouchOBJ)) //T9
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=TOUCH_MULTITOUCHSCREEN_T9 fail\n");
		}  
	 	if(-EIO==mxt_write_block(mxt->client,399,sizeof(NoiseOBJ),NoiseOBJ)) //T22
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=PROCG_NOISESUPPRESSION_T22 fail\n");
		}  
		if(-EIO==mxt_write_block(mxt->client,264,sizeof(UserdataOBJ),UserdataOBJ)) //T38
		{
	 	dev_info(&mxt->client->dev, "Write Object Table=USER_INFO_T38 fail\n");
		}  
		if(-EIO==mxt_write_block(mxt->client,467,sizeof(PalmSUPOBJ),PalmSUPOBJ)) //T41
		{
	 	dev_info(&mxt->client->dev, "Write Object Table=PROCI_PALMSUPPRESSION_T41 fail\n");
		}  
     	}
	break;
	
     case TOUCH_65OHM_V1:
     	{
		dev_info(&mxt->client->dev,"Write touch 65 ohm version 1 all configure  \n");
		if(-EIO==mxt_write_block(mxt->client,328,sizeof(PowerOBJ_65ohm_v1),PowerOBJ_65ohm_v1))	  //T7
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=GEN_POWERCONFIG_T7 fail\n");
		}  
	 	if(-EIO==mxt_write_block(mxt->client,331,sizeof(AcqOBJ_65ohm_v1),AcqOBJ_65ohm_v1)) //T8
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=GEN_ACQUIRECONFIG_T8 fail\n");
		}  
	 	if(-EIO==mxt_write_block(mxt->client,341,sizeof(MTouchOBJ_65ohm_v1),MTouchOBJ_65ohm_v1)) //T9
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=MXT_TOUCH_MULTITOUCHSCREEN_T9 fail\n");
		}  
	 	if(-EIO==mxt_write_block(mxt->client,399,sizeof(NoiseOBJ_65ohm_v1),NoiseOBJ_65ohm_v1)) //T22
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=PROCG_NOISESUPPRESSION_T22 fail\n");
		}  
		if(-EIO==mxt_write_block(mxt->client,264,sizeof(UserdataOBJ_65ohm_v1),UserdataOBJ_65ohm_v1)) //T38
		{
	 	dev_info(&mxt->client->dev, "Write Object Table=USER_INFO_T38 fail\n");
		}  
		if(-EIO==mxt_write_block(mxt->client,467,sizeof(PalmSUPOBJ_65ohm_fw11_v1),PalmSUPOBJ_65ohm_fw11_v1)) //T41
		{
	 	dev_info(&mxt->client->dev, "Write Object Table=PROCI_PALMSUPPRESSION_T41 fail\n");
		}  
     	}
	break;

      case TOUCH_65OHM_V2:
	{
		dev_info(&mxt->client->dev,"Write touch 65 ohm version 2 all configure  \n");
	  	if(-EIO==mxt_write_block(mxt->client,328,sizeof(PowerOBJ_65ohm_v2),PowerOBJ_65ohm_v2))	  //T7
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=GEN_POWERCONFIG_T7 fail\n");
		}  
	 	if(-EIO==mxt_write_block(mxt->client,331,sizeof(AcqOBJ_65ohm_v2),AcqOBJ_65ohm_v2)) //T8
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=GEN_ACQUIRECONFIG_T8 fail\n");
		}  
	 	if(-EIO==mxt_write_block(mxt->client,341,sizeof(MTouchOBJ_65ohm_v2),MTouchOBJ_65ohm_v2)) //T9
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=MXT_TOUCH_MULTITOUCHSCREEN_T9 fail\n");
		}  
	 	if(-EIO==mxt_write_block(mxt->client,399,sizeof(NoiseOBJ_65ohm_v2),NoiseOBJ_65ohm_v2)) //T22
	 	{
	 	dev_info(&mxt->client->dev, "Write Object Table=PROCG_NOISESUPPRESSION_T22 fail\n");
		}  
		if(-EIO==mxt_write_block(mxt->client,264,sizeof(UserdataOBJ_65ohm_v2),UserdataOBJ_65ohm_v2)) //T38
		{
	 	dev_info(&mxt->client->dev, "Write Object Table=USER_INFO_T38 fail\n");
		}  
		if(-EIO==mxt_write_block(mxt->client,467,sizeof(PalmSUPOBJ_65ohm_fw11_v2),PalmSUPOBJ_65ohm_fw11_v2)) //T41
		{
	 	dev_info(&mxt->client->dev, "Write Object Table=PROCI_PALMSUPPRESSION_T41 fail\n");
		}  
      	}
	break;
	
	default:
			dev_info(&mxt->client->dev, "not support write version %d machine configure\n",version);
	break;
   }
    //dev_info(&mxt->client->dev, "Atmel_WriteAllConfig()...- \n");
 }
  
  static void Atmel_WriteConfig(struct mxt_data *mxt,int mxt_table,int version)
{
  
   //dev_info(&mxt->client->dev, "Atmel_WriteConfig()...+ \n");
  
   switch (mxt_table)
   {
     case MXT_GEN_POWERCONFIG_T7:
     	{
	 mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt),sizeof(PowerOBJ),Power_table[version]);		
     	}
	break;
	
     case MXT_GEN_ACQUIRECONFIG_T8:
     	{
		mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt),sizeof(AcqOBJ),Acq_table[version]);		
	}
	break;

      case MXT_TOUCH_MULTITOUCHSCREEN_T9:
	{
	  	mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt),sizeof(MTouchOBJ),MTouch_table[version]);
      	}
	break;

	case MXT_PROCG_NOISESUPPRESSION_T22:
	{     
		mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt),sizeof(NoiseOBJ),Noise_table[version]);
      	}
	break;
	
	case MXT_PROCI_PALMSUPPRESSION_T41:
      	{
           if(version==TOUCH_65OHM_V1){
			if(mxt->device_info.major==1 && mxt->device_info.minor==1)   //check firmware version
		 		mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt),sizeof(PalmSUPOBJ_65ohm_fw11_v1),PalmSUPOBJ_65ohm_fw11_v1);
			else
   				mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt),sizeof(PalmSUPOBJ_65ohm_v1),PalmSUPOBJ_65ohm_v1);
           }
	    else if(version==TOUCH_65OHM_V2){
			if(mxt->device_info.major==1 && mxt->device_info.minor==1)   //check firmware version
		 		mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt),sizeof(PalmSUPOBJ_65ohm_fw11_v2),PalmSUPOBJ_65ohm_fw11_v2);
			else
   				mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt),sizeof(PalmSUPOBJ_65ohm_v2),PalmSUPOBJ_65ohm_v2);
	    }
	   else
	   		mxt_write_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt),sizeof(PalmSUPOBJ),PalmSUPOBJ);
	}
	break;
	
	default:
			dev_info(&mxt->client->dev, "not support write Table %d=%s\n",
					mxt_table ,object_type_name[mxt_table]);
		break;
   }
   
// dev_info(&mxt->client->dev, "Atmel_WriteConfig()...- \n");
}
  static void Atmel_CheckConfig(struct mxt_data *mxt)
{
   int do_write_back=0;
   
   dev_info(&mxt->client->dev,"Atmel_CheckConfig()...+ \n");

   /* COMPAL - START - [2011/5/17  15:35] - Jamin - [Touch]: check touch version  */
   mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_USER_INFO_T38,mxt),1,&mxt->check_resistance);
  	 	dev_info(&mxt->client->dev, "Read Resistance=%d \n",mxt->check_resistance);
    mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_USER_INFO_T38,mxt)+1,1,&mxt->check_version);
  	 	dev_info(&mxt->client->dev, "Read Version=%d \n",mxt->check_version);
   /* COMPAL - END */
		
  if(mxt->check_resistance==65)
  {
      if(mxt->check_version==2)
	{
	        // do touch 65 ohm version 2 configure check
       	dev_info(&mxt->client->dev,"Do touch 65 ohm version 2 configure check  \n");
  		//read touch register
  		memset(temp_buf,0,sizeof(temp_buf));
  		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt),sizeof(PowerOBJ_65ohm_v2),temp_buf);
  		if(mystrncmp(PowerOBJ_65ohm_v2,temp_buf,sizeof(PowerOBJ_65ohm_v2))!=0)
 		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_GEN_POWERCONFIG_T7]);
			Atmel_WriteConfig(mxt, MXT_GEN_POWERCONFIG_T7,TOUCH_65OHM_V2);
			do_write_back=1;
  		}	  
  
  		memset(temp_buf,0,sizeof(temp_buf));
  		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt),sizeof(AcqOBJ_65ohm_v2),temp_buf);
  		if(mystrncmp(AcqOBJ_65ohm_v2,temp_buf,sizeof(AcqOBJ_65ohm_v2))!=0)
 		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_GEN_ACQUIRECONFIG_T8]);
			Atmel_WriteConfig(mxt, MXT_GEN_ACQUIRECONFIG_T8,TOUCH_65OHM_V2);
			do_write_back=1;
  		}	  
 
	 	memset(temp_buf,0,sizeof(temp_buf));
 		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9,mxt),sizeof(MTouchOBJ_65ohm_v2),temp_buf);
 		if(mystrncmp(MTouchOBJ_65ohm_v2,temp_buf,sizeof(MTouchOBJ_65ohm_v2))!=0)
		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9]);
			Atmel_WriteConfig(mxt, MXT_TOUCH_MULTITOUCHSCREEN_T9,TOUCH_65OHM_V2);
			do_write_back=1;
 		}

 		memset(temp_buf,0,sizeof(temp_buf));
 		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22,mxt),sizeof(NoiseOBJ_65ohm_v2),temp_buf);
 		if(mystrncmp(NoiseOBJ_65ohm_v2,temp_buf,sizeof(NoiseOBJ_65ohm_v2))!=0)
		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_PROCG_NOISESUPPRESSION_T22]);
			Atmel_WriteConfig(mxt, MXT_PROCG_NOISESUPPRESSION_T22,TOUCH_65OHM_V2);
			do_write_back=1;
 		}
 
 	    memset(temp_buf,0,sizeof(temp_buf));
 	    if(mxt->device_info.major==1 && mxt->device_info.minor==1)  
           {
             dev_info(&mxt->client->dev,"Firmware version [%d.%d]  so enable palm\n",mxt->device_info.major,mxt->device_info.minor);
 		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt),sizeof(PalmSUPOBJ_65ohm_fw11_v2),temp_buf);
 		if(mystrncmp(PalmSUPOBJ_65ohm_fw11_v2,temp_buf,sizeof(PalmSUPOBJ_65ohm_fw11_v2))!=0)
 		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41]);
			Atmel_WriteConfig(mxt, MXT_PROCI_PALMSUPPRESSION_T41,TOUCH_65OHM_V2);
			do_write_back=1;
 		}
           }
	    else
           {
             dev_info(&mxt->client->dev,"Firmware version [%d.%d] so disable palm \n",mxt->device_info.major,mxt->device_info.minor);
 		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt),sizeof(PalmSUPOBJ_65ohm_v2),temp_buf);
 		if(mystrncmp(PalmSUPOBJ_65ohm_v2,temp_buf,sizeof(PalmSUPOBJ_65ohm_v2))!=0)
 		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41]);
			Atmel_WriteConfig(mxt, MXT_PROCI_PALMSUPPRESSION_T41,TOUCH_65OHM_V2);
			do_write_back=1;
 		}
           }   
      	}
	else
	{
	       // do touch 65 ohm version 1 configure check
       	dev_info(&mxt->client->dev,"Do touch 65 ohm version 1 configure check  \n");
  		//read touch register
  		memset(temp_buf,0,sizeof(temp_buf));
  		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt),sizeof(PowerOBJ_65ohm_v1),temp_buf);
  		if(mystrncmp(PowerOBJ_65ohm_v1,temp_buf,sizeof(PowerOBJ_65ohm_v1))!=0)
 		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_GEN_POWERCONFIG_T7]);
			Atmel_WriteConfig(mxt, MXT_GEN_POWERCONFIG_T7,TOUCH_65OHM_V1);
			do_write_back=1;
  		}	  
  
  		memset(temp_buf,0,sizeof(temp_buf));
  		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt),sizeof(AcqOBJ_65ohm_v1),temp_buf);
  		if(mystrncmp(AcqOBJ_65ohm_v1,temp_buf,sizeof(AcqOBJ_65ohm_v1))!=0)
 		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_GEN_ACQUIRECONFIG_T8]);
			Atmel_WriteConfig(mxt, MXT_GEN_ACQUIRECONFIG_T8,TOUCH_65OHM_V1);
			do_write_back=1;
  		}	  
 
	 	memset(temp_buf,0,sizeof(temp_buf));
 		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9,mxt),sizeof(MTouchOBJ_65ohm_v1),temp_buf);
 		if(mystrncmp(MTouchOBJ_65ohm_v1,temp_buf,sizeof(MTouchOBJ_65ohm_v1))!=0)
		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9]);
			Atmel_WriteConfig(mxt, MXT_TOUCH_MULTITOUCHSCREEN_T9,TOUCH_65OHM_V1);
			do_write_back=1;
 		}

 		memset(temp_buf,0,sizeof(temp_buf));
 		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22,mxt),sizeof(NoiseOBJ_65ohm_v1),temp_buf);
 		if(mystrncmp(NoiseOBJ_65ohm_v1,temp_buf,sizeof(NoiseOBJ_65ohm_v1))!=0)
		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_PROCG_NOISESUPPRESSION_T22]);
			Atmel_WriteConfig(mxt, MXT_PROCG_NOISESUPPRESSION_T22,TOUCH_65OHM_V1);
			do_write_back=1;
 		}
		
           memset(temp_buf,0,sizeof(temp_buf));
           if(mxt->device_info.major==1 && mxt->device_info.minor==1)
           {
             dev_info(&mxt->client->dev,"Firmware version [%d.%d]  so enable palm\n",mxt->device_info.major,mxt->device_info.minor);
 		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt),sizeof(PalmSUPOBJ_65ohm_fw11_v1),temp_buf);
 		if(mystrncmp(PalmSUPOBJ_65ohm_fw11_v1,temp_buf,sizeof(PalmSUPOBJ_65ohm_fw11_v1))!=0)
 		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41]);
			Atmel_WriteConfig(mxt, MXT_PROCI_PALMSUPPRESSION_T41,TOUCH_65OHM_V1);
			do_write_back=1;
 		}
           }
	    else
           {
             dev_info(&mxt->client->dev,"Firmware version [%d.%d] so disable palm \n",mxt->device_info.major,mxt->device_info.minor);
 		mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt),sizeof(PalmSUPOBJ_65ohm_v1),temp_buf);
 		if(mystrncmp(PalmSUPOBJ_65ohm_v1,temp_buf,sizeof(PalmSUPOBJ_65ohm_v1))!=0)
 		{
   			dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
							 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41]);
			Atmel_WriteConfig(mxt, MXT_PROCI_PALMSUPPRESSION_T41,TOUCH_65OHM_V1);
			do_write_back=1;
 		}
           }   
      	}
  }
 else 
 { 
  	dev_info(&mxt->client->dev,"Do touch 30 ohm  configure check  \n");
	 //read touch register
  	memset(temp_buf,0,sizeof(temp_buf));
  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt),sizeof(PowerOBJ),temp_buf);
  	if(mystrncmp(PowerOBJ,temp_buf,sizeof(PowerOBJ))!=0)
 	{
   		dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
						 object_type_name[MXT_GEN_POWERCONFIG_T7]);
		Atmel_WriteConfig(mxt, MXT_GEN_POWERCONFIG_T7,TOUCH_30OHM);
		do_write_back=1;
  	}	  
  
  	memset(temp_buf,0,sizeof(temp_buf));
  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt),sizeof(AcqOBJ),temp_buf);
  	if(mystrncmp(AcqOBJ,temp_buf,sizeof(AcqOBJ))!=0)
 	{
   		dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
						 object_type_name[MXT_GEN_ACQUIRECONFIG_T8]);
		Atmel_WriteConfig(mxt, MXT_GEN_ACQUIRECONFIG_T8,TOUCH_30OHM);
		do_write_back=1;
  	}	  
 
	 memset(temp_buf,0,sizeof(temp_buf));
 	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9,mxt),sizeof(MTouchOBJ),temp_buf);
 	if(mystrncmp(MTouchOBJ,temp_buf,sizeof(MTouchOBJ))!=0)
	{
   		dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
						 object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9]);
		Atmel_WriteConfig(mxt, MXT_TOUCH_MULTITOUCHSCREEN_T9,TOUCH_30OHM);
		do_write_back=1;
 	}

 	memset(temp_buf,0,sizeof(temp_buf));
 	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22,mxt),sizeof(NoiseOBJ),temp_buf);
 	if(mystrncmp(NoiseOBJ,temp_buf,sizeof(NoiseOBJ))!=0)
	{
   		dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
						 object_type_name[MXT_PROCG_NOISESUPPRESSION_T22]);
		Atmel_WriteConfig(mxt, MXT_PROCG_NOISESUPPRESSION_T22,TOUCH_30OHM);
		do_write_back=1;
 	}
 
 	memset(temp_buf,0,sizeof(temp_buf));
 	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41, mxt),sizeof(PalmSUPOBJ),temp_buf);
 	if(mystrncmp(PalmSUPOBJ,temp_buf,sizeof(PalmSUPOBJ))!=0)
 	{
   		dev_info(&mxt->client->dev, "Find Table=%s different \t \n",
						 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41]);
		Atmel_WriteConfig(mxt, MXT_PROCI_PALMSUPPRESSION_T41,TOUCH_30OHM);
		do_write_back=1;
 	}
 }
/* COMPAL - END */
 
 //do write back
 if(do_write_back)
 	Atmel_WriteBack(mxt);

 //do hardware reset again
 mxt->reset_hw();
 msleep(150);
 
dev_info(&mxt->client->dev, "Atmel_CheckConfig()...- \n");
}

#if 0
 static void Atmel_ReadConfig(struct mxt_data *mxt)
{
   mxt_debug(DEBUG_INFO, "[Touch]: Atmel_ReadConfig()...+ \n");
   
  //debug read register
  if (debug >= DEBUG_VERBOSE) {
 	int i;
 	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_USER_INFO_T38,mxt),sizeof(UserdataOBJ),temp_buf);
  	for(i=0;i<sizeof(UserdataOBJ);i++){
  	 	dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_USER_INFO_T38] ,i,temp_buf[i]);
   	}
  	printk("**************************************\n");
  
  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7,mxt),sizeof(PowerOBJ),temp_buf);
  	for(i=0;i<sizeof(PowerOBJ);i++){
  	 	dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_GEN_POWERCONFIG_T7] ,i,temp_buf[i]);
   	}
   	printk("**************************************\n");
   
  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8,mxt),sizeof(AcqOBJ),temp_buf);
  	for(i=0;i<sizeof(AcqOBJ);i++){
  		 dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_GEN_ACQUIRECONFIG_T8] ,i,temp_buf[i]);
   	}
   	printk("**************************************\n");

  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9,mxt),sizeof(MTouchOBJ),temp_buf);
  	for(i=0;i<sizeof(MTouchOBJ);i++){
  		 dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_TOUCH_MULTITOUCHSCREEN_T9] ,i,temp_buf[i]);
  	 }
   	printk("**************************************\n");

  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15,mxt),sizeof(KeyArrayOBJ),temp_buf);
  	for(i=0;i<sizeof(KeyArrayOBJ);i++){
  		 dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_TOUCH_KEYARRAY_T15] ,i,temp_buf[i]);
   	}
   	printk("**************************************\n");

  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18,mxt),sizeof(ComconfigOBJ),temp_buf);
  	for(i=0;i<sizeof(ComconfigOBJ);i++){
  	 	dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_SPT_COMMSCONFIG_T18] ,i,temp_buf[i]);
  	 }
   	printk("**************************************\n");

 	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22,mxt),sizeof(NoiseOBJ),temp_buf);
  	for(i=0;i<sizeof(NoiseOBJ);i++){
  		 dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_PROCG_NOISESUPPRESSION_T22] ,i,temp_buf[i]);
   	}
   	printk("**************************************\n");

  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24,mxt),sizeof(OneTGOBJ),temp_buf);
  	for(i=0;i<sizeof(OneTGOBJ);i++){
  		 dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24] ,i,temp_buf[i]);
   	}
   	printk("**************************************\n");

  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25,mxt),sizeof(SelfTestOBJ),temp_buf);
  	for(i=0;i<sizeof(SelfTestOBJ);i++){
  		 dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_SPT_SELFTEST_T25] ,i,temp_buf[i]);
   	}
   	printk("**************************************\n");

  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27,mxt),sizeof(TwoTGOBJ),temp_buf);
  	for(i=0;i<sizeof(TwoTGOBJ);i++){
  		 dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] ,i,temp_buf[i]);
   	}
   	printk("**************************************\n");

  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28,mxt),sizeof(CTEOBJ),temp_buf);
  	for(i=0;i<sizeof(CTEOBJ);i++){
  		 dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_SPT_CTECONFIG_T28] ,i,temp_buf[i]);
   	}
   	printk("**************************************\n");

  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_GRIPSUPPRESSION_T40,mxt),sizeof(GripSUPOBJ),temp_buf);
  	for(i=0;i<sizeof(GripSUPOBJ);i++){
  		 dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_PROCI_GRIPSUPPRESSION_T40] ,i,temp_buf[i]);
   	}
   	printk("**************************************\n");

  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41,mxt),sizeof(PalmSUPOBJ),temp_buf);
  	for(i=0;i<sizeof(PalmSUPOBJ);i++){
  		 dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_PROCI_PALMSUPPRESSION_T41] ,i,temp_buf[i]);
   	}
   	printk("**************************************\n");

  	mxt_read_block(mxt->client,MXT_BASE_ADDR(MXT_SPT_DIGITIZER_T43,mxt),sizeof(DigitizerOBJ),temp_buf);
  	for(i=0;i<sizeof(DigitizerOBJ);i++){
  		 dev_info(&mxt->client->dev, "Read Table=%s \t address[%d]=%d\n",
					 object_type_name[MXT_SPT_DIGITIZER_T43] ,i,temp_buf[i]);
   	}
   	printk("**************************************\n");
   } 
   mxt_debug(DEBUG_INFO, "[Touch]: Atmel_ReadConfig()...- \n");
 }
#endif
/* COMPAL - END */



/* Returns object address in mXT chip, or zero if object is not found */
static u16 get_object_address(uint8_t object_type,
			      uint8_t instance,
			      struct mxt_object *object_table, int max_objs)
{
	uint8_t object_table_index = 0;
	uint8_t address_found = 0;
	uint16_t address = 0;
	struct mxt_object *obj;

	while ((object_table_index < max_objs) && !address_found) {
		obj = &object_table[object_table_index];
		if (obj->type == object_type) {
			address_found = 1;
			/* Are there enough instances defined in the FW? */
			if (obj->instances >= instance) {
				address = obj->chip_addr +
				    (obj->size + 1) * instance;
			} else {
				return 0;
			}
		}
		object_table_index++;
	}
	return address;
}

/*
 * Reads a block of bytes from given address from mXT chip. If we are
 * reading from message window, and previous read was from message window,
 * there's no need to write the address pointer: the mXT chip will
 * automatically set the address pointer back to message window start.
 */

static int mxt_read_block(struct i2c_client *client,
			  u16 addr, u16 length, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16 le_addr,retry=0;
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);


    /* COMPAL - START - [2011/1/19  18:55] - Jamin - [Touch]: enable touch atmel   */
    //fix sometimes i2c can not read issue
	#if 0
	if (mxt != NULL) {
		if ((mxt->last_read_addr == addr) &&
		    (addr == mxt->msg_proc_addr)) {
			if (i2c_master_recv(client, value, length) == length)
				return length;
			else
				return -EIO;
		} else {
			mxt->last_read_addr = addr;
		}
	}

	mxt_debug(DEBUG_TRACE, "Writing address pointer & reading %d bytes "
		  "in on i2c transaction...\n", length);
	#endif
	/* COMPAL - END */

	le_addr = cpu_to_le16(addr);
	msg[0].addr = client->addr;
	msg[0].flags = 0x00;
	msg[0].len = 2;
	msg[0].buf = (u8 *) &le_addr;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = (u8 *) value;

	/* COMPAL - START - [2011/1/19  11:9] - Jamin - [Touch]: enable touch atmel   */
	//add retry mechanism
	for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {
		if (i2c_transfer(adapter, msg, 2) == 2)
			return length;
		mdelay(10);
	}
	//else
	//	return -EIO;
	if (retry == I2C_RETRY_COUNT) {
		printk(KERN_ERR "mxt_read_block retry over %d\n",
			I2C_RETRY_COUNT);
		return -EIO;
	}
     /* COMPAL - END */
    return length;
}

/* Reads a block of bytes from current address from mXT chip. */

static int mxt_read_block_wo_addr(struct i2c_client *client,
				  u16 length, u8 *value)
{

	if (i2c_master_recv(client, value, length) == length) {
		mxt_debug(DEBUG_TRACE, "I2C block read ok\n");
		return length;
	} else {
		mxt_debug(DEBUG_INFO, "I2C block read failed\n");
		return -EIO;
	}

}

/* Writes one byte to given address in mXT chip. */

static int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value)
{
	struct {
		__le16 le_addr;
		u8 data;

	} i2c_byte_transfer;

	struct mxt_data *mxt;
       int retry=0;
	   
	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	i2c_byte_transfer.le_addr = cpu_to_le16(addr);
	i2c_byte_transfer.data = value;

	/* COMPAL - START - [2011/1/19  11:13] - Jamin - [Touch]: enable touch atmel   */
	//add retry mechanism
	for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {
		if (i2c_master_send(client, (u8 *) &i2c_byte_transfer, 3) == 3)
			return 0;
		 mdelay(10);
	}
	//else
	//	return -EIO;
	if (retry == I2C_RETRY_COUNT) {
		printk(KERN_ERR "mxt_write_byte retry over %d\n",
			I2C_RETRY_COUNT);
		return -EIO;
	}
	/* COMPAL - END */
	return 0;
}

/* Writes a block of bytes (max 256) to given address in mXT chip. */
static int mxt_write_block(struct i2c_client *client,
			   u16 addr, u16 length, u8 *value)
{
	int i,retry=0;
	struct {
		__le16 le_addr;
		u8 data[256];

	} i2c_block_transfer;

	struct mxt_data *mxt;

	mxt_debug(DEBUG_TRACE, "Writing %d bytes to %d...", length, addr);
	if (length > 256)
		return -EINVAL;
	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	for (i = 0; i < length; i++)
		i2c_block_transfer.data[i] = *value++;
	i2c_block_transfer.le_addr = cpu_to_le16(addr);

	/* COMPAL - START - [2011/1/19  11:20] - Jamin - [Touch]: enable touch atmel   */
	//add retry mechanism
	for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {
		i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
		if (i == (length + 2))
			return length;
		mdelay(10);
	}	
	//else
	//	return -EIO;
	if (retry == I2C_RETRY_COUNT) {
		printk(KERN_ERR "mxt_write_block retry over %d\n",
			I2C_RETRY_COUNT);
		return -EIO;
	}
	/* COMPAL - END */
	return length;
}

/* Calculates the CRC value for mXT infoblock. */
int calculate_infoblock_crc(u32 *crc_result, u8 *data, int crc_area_size)
{
	u32 crc = 0;
	int i;

	for (i = 0; i < (crc_area_size - 1); i = i + 2)
		crc = CRC_24(crc, *(data + i), *(data + i + 1));
	/* If uneven size, pad with zero */
	if (crc_area_size & 0x0001)
		crc = CRC_24(crc, *(data + i), 0);
	/* Return only 24 bits of CRC. */
	*crc_result = (crc & 0x00FFFFFF);

	return 0;
}

void process_T9_message(u8 *message, struct mxt_data *mxt, int last_touch)
{

	struct input_dev *input;
	u8 status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8 touch_size = 255;
	u8 touch_number;
	u8 amplitude;
	u8 report_id;

      /* COMPAL - START - [2011/3/2  13:25] - Jamin - [Touch]: fix suspend resume issue  */
	//change global variable
      #if 0
	static int stored_size[MXT_MAX_NUM_TOUCHES];
	static int stored_x[MXT_MAX_NUM_TOUCHES];
	static int stored_y[MXT_MAX_NUM_TOUCHES];
	#endif
	/* COMPAL - END */
	
	int i;
	int active_touches = 0;
	/*
	 * If the 'last_touch' flag is set, we have received
		all the touch messages
	 * there are available in this cycle, so send the
		events for touches that are
	 * active.
	 */

//++Charles 
	if (last_touch) {
		for (i = 0; i < MXT_MAX_NUM_TOUCHES; i++) {
			if (stored_size[i]) {
				active_touches++;
				input_report_abs(mxt->input, ABS_MT_TRACKING_ID,
						 i);
				input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR,
						 stored_size[i]);
				input_report_abs(mxt->input, ABS_MT_POSITION_X,
						 stored_x[i]);
				input_report_abs(mxt->input, ABS_MT_POSITION_Y,
						 stored_y[i]);
				input_mt_sync(mxt->input);
			}
		}
		
		input_report_key(mxt->input, BTN_TOUCH, active_touches > 0);

		if (stored_size[i] != 0) {
			input_report_abs(mxt->input, ABS_X, stored_x[0]);
			input_report_abs(mxt->input, ABS_Y, stored_y[0]);
		}

		input_sync(mxt->input);
//--

	} else {

		input = mxt->input;
		status = message[MXT_MSG_T9_STATUS];
		report_id = message[0];
		
            /* COMPAL - START - [2011/3/14  14:46] - Jamin - [Touch]: enable atmel touch  */
              //fix could not release issue
		 touch_number = message[MXT_MSG_REPORTID] -
			    mxt->rid_map[report_id].first_rid;
	     /* COMPAL - END */	
				
		if (status & MXT_MSGB_T9_SUPPRESS) {
			/* Touch has been suppressed by grip/face */
			/* detection */
			dev_info(&mxt->client->dev, "[Touch] : detect SUPRESS");
			/* COMPAL - START - [2011/3/14  14:46] - Jamin - [Touch]: enable atmel touch  */
			//fix could not release issue
			stored_size[touch_number] = 0;
			/* COMPAL - END */
		} else {
			xpos = message[MXT_MSG_T9_XPOSMSB] * 16 +
			    ((message[MXT_MSG_T9_XYPOSLSB] >> 4) & 0xF);
			ypos = message[MXT_MSG_T9_YPOSMSB] * 16 +
			    ((message[MXT_MSG_T9_XYPOSLSB] >> 0) & 0xF);
			if (mxt->max_x_val < 1024)
				xpos >>= 2;
			if (mxt->max_y_val < 1024)
				ypos >>= 2;

			touch_number = message[MXT_MSG_REPORTID] -
			    mxt->rid_map[report_id].first_rid;

			stored_x[touch_number] = xpos;
			stored_y[touch_number] = ypos;

			if (status & MXT_MSGB_T9_DETECT) {
				/*
				 * TODO: more precise touch size calculation?
				 * mXT224 reports the number of touched nodes,
				 * so the exact value for touch ellipse major
				 * axis length would be 2*sqrt(touch_size/pi)
				 * (assuming round touch shape).
				 */
				touch_size = message[MXT_MSG_T9_TCHAREA];
				touch_size = touch_size >> 2;
				if (!touch_size)
					touch_size = 1;

				stored_size[touch_number] = touch_size;

				if (status & MXT_MSGB_T9_AMP)
					/* Amplitude of touch has changed */
					amplitude =
					    message[MXT_MSG_T9_TCHAMPLITUDE];
			}

			if (status & MXT_MSGB_T9_RELEASE) {
				/* The previously reported touch has
					been removed. */
				stored_size[touch_number] = 0;
			}
		}

		if (status & MXT_MSGB_T9_SUPPRESS) {
			mxt_debug(DEBUG_TRACE, "SUPRESS");
		} else {
			if (status & MXT_MSGB_T9_DETECT) {
				mxt_debug(DEBUG_TRACE, "DETECT:%s%s%s%s",
					  ((status & MXT_MSGB_T9_PRESS) ?
					   " PRESS" : ""),
					  ((status & MXT_MSGB_T9_MOVE) ? " MOVE"
					   : ""),
					  ((status & MXT_MSGB_T9_AMP) ? " AMP" :
					   ""),
					  ((status & MXT_MSGB_T9_VECTOR) ?
					   " VECT" : ""));

			} else if (status & MXT_MSGB_T9_RELEASE) {
				mxt_debug(DEBUG_TRACE, "RELEASE");
			}
		}
		mxt_debug(DEBUG_TRACE, "X=%d, Y=%d, TOUCHSIZE=%d",
			  xpos, ypos, touch_size);
	}
	return;
}

int process_message(u8 *message, u8 object, struct mxt_data *mxt)
{
	struct i2c_client *client;
	u8 status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8 event;
	u8 length;
	u8 report_id;

	client = mxt->client;
	length = mxt->message_size;
	report_id = message[0];

	if ((mxt->nontouch_msg_only == 0) || (!IS_TOUCH_OBJECT(object))) {
		mutex_lock(&mxt->msg_mutex);
		/* Copy the message to buffer */
		if (mxt->msg_buffer_startp < MXT_MESSAGE_BUFFER_SIZE)
			mxt->msg_buffer_startp++;
		else
			mxt->msg_buffer_startp = 0;

		if (mxt->msg_buffer_startp == mxt->msg_buffer_endp) {
			mxt_debug(DEBUG_TRACE,
				  "Message buf full, discarding last entry.\n");
			if (mxt->msg_buffer_endp < MXT_MESSAGE_BUFFER_SIZE)
				mxt->msg_buffer_endp++;
			else
				mxt->msg_buffer_endp = 0;
		}
		memcpy((mxt->messages + mxt->msg_buffer_startp * length),
		       message, length);
		mutex_unlock(&mxt->msg_mutex);
	}

	switch (object) {
	//tune performance	
	case MXT_TOUCH_MULTITOUCHSCREEN_T9:
		process_T9_message(message, mxt, 0);
		break;
		
	case MXT_GEN_COMMANDPROCESSOR_T6:
		status = message[1];
		if (status & MXT_MSGB_T6_COMSERR)
			dev_err(&client->dev, "maXTouch checksum error\n");
		if (status & MXT_MSGB_T6_CFGERR) {
			/*
			 * Configuration error. A proper configuration
			 * needs to be written to chip and backed up. Refer
			 * to protocol document for further info.
			 */
			dev_err(&client->dev, "maXTouch configuration error\n");
		}
		if (status & MXT_MSGB_T6_CAL) {
			/* Calibration in action, no need to react */
			dev_info(&client->dev,
				 "maXTouch calibration in progress\n");
		}
		if (status & MXT_MSGB_T6_SIGERR) {
			/*
			 * Signal acquisition error, something is seriously
			 * wrong, not much we can in the driver to correct
			 * this
			 */
			dev_err(&client->dev, "maXTouch acquisition error\n");
		}
		if (status & MXT_MSGB_T6_OFL) {
			/*
			 * Cycle overflow, the acquisition is too short.
			 * Can happen temporarily when there's a complex
			 * touch shape on the screen requiring lots of
			 * processing.
			 */
			dev_err(&client->dev, "maXTouch cycle overflow\n");
		}
		if (status & MXT_MSGB_T6_RESET) {
			/* Chip has reseted, no need to react. */
			dev_info(&client->dev, "maXTouch chip reset\n");
		}
		if (status == 0) {
			/* Chip status back to normal. */
			dev_info(&client->dev, "maXTouch status normal\n");
		}
		break;

	case MXT_SPT_GPIOPWM_T19:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "Receiving GPIO message\n");
		break;

	case MXT_PROCI_GRIPFACESUPPRESSION_T20:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				 "Receiving face suppression msg\n");
		break;

	case MXT_PROCG_NOISESUPPRESSION_T22:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				 "Receiving noise suppression msg\n");
		status = message[MXT_MSG_T22_STATUS];
		if (status & MXT_MSGB_T22_FHCHG) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					 "maXTouch: Freq changed\n");
		}
		if (status & MXT_MSGB_T22_GCAFERR) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					 "maXTouch: High noise " "level\n");
		}
		if (status & MXT_MSGB_T22_FHERR) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					 "maXTouch: Freq changed - "
					 "Noise level too high\n");
		}
		break;

	case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				 "Receiving one-touch gesture msg\n");

		event = message[MXT_MSG_T24_STATUS] & 0x0F;
		xpos = message[MXT_MSG_T24_XPOSMSB] * 16 +
		    ((message[MXT_MSG_T24_XYPOSLSB] >> 4) & 0x0F);
		ypos = message[MXT_MSG_T24_YPOSMSB] * 16 +
		    ((message[MXT_MSG_T24_XYPOSLSB] >> 0) & 0x0F);
		xpos >>= 2;
		ypos >>= 2;

		switch (event) {
		case MT_GESTURE_RESERVED:
			break;
		case MT_GESTURE_PRESS:
			break;
		case MT_GESTURE_RELEASE:
			break;
		case MT_GESTURE_TAP:
			break;
		case MT_GESTURE_DOUBLE_TAP:
			break;
		case MT_GESTURE_FLICK:
			break;
		case MT_GESTURE_DRAG:
			break;
		case MT_GESTURE_SHORT_PRESS:
			break;
		case MT_GESTURE_LONG_PRESS:
			break;
		case MT_GESTURE_REPEAT_PRESS:
			break;
		case MT_GESTURE_TAP_AND_PRESS:
			break;
		case MT_GESTURE_THROW:
			break;
		default:
			break;
		}
		break;

	case MXT_SPT_SELFTEST_T25:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "Receiving Self-Test msg\n");

		if (message[MXT_MSG_T25_STATUS] == MXT_MSGR_T25_OK) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					 "maXTouch: Self-Test OK\n");

		} else {
			dev_err(&client->dev,
				"maXTouch: Self-Test Failed [%02x]:"
				"{%02x,%02x,%02x,%02x,%02x}\n",
				message[MXT_MSG_T25_STATUS],
				message[MXT_MSG_T25_STATUS + 0],
				message[MXT_MSG_T25_STATUS + 1],
				message[MXT_MSG_T25_STATUS + 2],
				message[MXT_MSG_T25_STATUS + 3],
				message[MXT_MSG_T25_STATUS + 4]
			    );
		}
		break;

	case MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				 "Receiving 2-touch gesture message\n");
		break;

	case MXT_SPT_CTECONFIG_T28:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "Receiving CTE message...\n");
		status = message[MXT_MSG_T28_STATUS];
		if (status & MXT_MSGB_T28_CHKERR)
			dev_err(&client->dev,
				"maXTouch: Power-Up CRC failure\n");

		break;
	default:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "maXTouch: Unknown message!\n");

		break;
	}

	return 0;
}

/*
 * Processes messages when the interrupt line (CHG) is asserted. Keeps
 * reading messages until a message with report ID 0xFF is received,
 * which indicates that there is no more new messages.
 *
 */

static void mxt_worker(struct work_struct *work)
{
	struct mxt_data *mxt;
	struct i2c_client *client;

	u8 *message;
	u16 message_length;
	u16 message_addr;
	u8 report_id;
	u8 object;
	int error;
	int i;
	char *message_string;
	char *message_start;

	message = NULL;
	mxt = container_of(work, struct mxt_data, dwork.work);
	disable_irq(mxt->irq);
	client = mxt->client;
	message_addr = mxt->msg_proc_addr;
	message_length = mxt->message_size;

	if (message_length < 256) {
		message = kmalloc(message_length, GFP_KERNEL);
		if (message == NULL) {
			dev_err(&client->dev, "Error allocating memory\n");
			return;
		}
	} else {
		dev_err(&client->dev,
			"Message length larger than 256 bytes not supported\n");
		return;
	}

	mxt_debug(DEBUG_TRACE,"maXTouch worker active: \n");
	do {
		/* Read next message, reread on failure. */
		/* -1 TO WORK AROUND A BUG ON 0.9 FW MESSAGING, needs */
		/* to be changed back if checksum is read */
		mxt->message_counter++;
		for (i = 1; i < I2C_RETRY_COUNT; i++) {
			error = mxt_read_block(client,
					       message_addr,
					       message_length - 1, message);
			if (error >= 0)
				break;
			mxt->read_fail_counter++;
			dev_err(&client->dev,
				"Failure reading maxTouch device\n");
		}
		if (error < 0) {
			kfree(message);
			return;
		}

		if (mxt->address_pointer != message_addr)
			mxt->valid_ap = 0;
		report_id = message[0];

		if (debug >= DEBUG_RAW) {
			mxt_debug(DEBUG_RAW, "%s message [msg count: %08x]:",
				  REPORT_ID_TO_OBJECT_NAME(report_id, mxt),
				  mxt->message_counter);
			/* 5 characters per one byte */
			message_string = kmalloc(message_length * 5,
						 GFP_KERNEL);
			if (message_string == NULL) {
				dev_err(&client->dev,
					"Error allocating memory\n");
				kfree(message);
				return;
			}
			message_start = message_string;
			for (i = 0; i < message_length; i++) {
				message_string +=
				    sprintf(message_string,
					    "0x%02X ", message[i]);
			}
			mxt_debug(DEBUG_RAW, "%s", message_start);
			kfree(message_start);
		}

		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)) {
			memcpy(mxt->last_message, message, message_length);
			mxt->new_msgs = 1;
			smp_wmb();
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;
			process_message(message, object, mxt);
		}
		mxt_debug(DEBUG_TRACE, "chgline: %d\n", mxt->read_chg());
	} while (comms ? (mxt->read_chg() == 0) :
		 ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)));

	/* All messages processed, send the events) */
	process_T9_message(NULL, mxt, 1);

	kfree(message);
	enable_irq(mxt->irq);
	/* Make sure we don't miss any interrupts and read changeline. */
	if (mxt->read_chg() == 0)
		schedule_delayed_work(&mxt->dwork, 0);
}

/*
 * The maXTouch device will signal the host about a new message by asserting
 * the CHG line. This ISR schedules a worker routine to read the message when
 * that happens.
 */

static irqreturn_t mxt_irq_handler(int irq, void *_mxt)
{
	struct mxt_data *mxt = _mxt;
      
	mxt->irq_counter++;
	if (mxt->valid_interrupt()) {
		/* Send the signal only if falling edge generated the irq. */
		//cancel_delayed_work_sync(&mxt->dwork);
		
		schedule_delayed_work(&mxt->dwork, 0);
		mxt->valid_irq_counter++;
		if (debug >= DEBUG_MESSAGES) {
			 dev_info(&mxt->client->dev, "[Touch]: valid interrupt \n");
		}
	} else {
		mxt->invalid_irq_counter++;
		if (debug >= DEBUG_MESSAGES) {
			 dev_info(&mxt->client->dev, "[Touch]: invalid interrupt \n");
		}
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/******************************************************************************/
/* Initialization of driver                                                   */
/******************************************************************************/

static int __devinit mxt_identify(struct i2c_client *client,
				  struct mxt_data *mxt, u8 * id_block_data)
{
	u8 buf[7];
	int error;
	int identified;

	identified = 0;

	/* Read Device info to check if chip is valid */
	error = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, MXT_ID_BLOCK_SIZE,
			       (u8 *) buf);

	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "Failure accessing maXTouch device\n");
		return -EIO;
	}

	memcpy(id_block_data, buf, MXT_ID_BLOCK_SIZE);

	mxt->device_info.family_id = buf[0];
	mxt->device_info.variant_id = buf[1];
	mxt->device_info.major = ((buf[2] >> 4) & 0x0F);
	mxt->device_info.minor = (buf[2] & 0x0F);
	mxt->device_info.build = buf[3];
	mxt->device_info.x_size = buf[4];
	mxt->device_info.y_size = buf[5];
	mxt->device_info.num_objs = buf[6];
	mxt->device_info.num_nodes = mxt->device_info.x_size *
	    mxt->device_info.y_size;

	/*
	 * Check Family & Variant Info; warn if not recognized but
	 * still continue.
	 */

	/* MXT224 */
	if (mxt->device_info.family_id == MXT224_FAMILYID) {
		strcpy(mxt->device_info.family_name, "mXT224");

		if (mxt->device_info.variant_id == MXT224_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else if (mxt->device_info.variant_id ==
			   MXT224_UNCAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Uncalibrated");
		} else {
			dev_err(&client->dev,
				"Warning: maXTouch Variant ID [%d] not "
				"supported\n", mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}

		/* MXT1386 */
	} else if (mxt->device_info.family_id == MXT1386_FAMILYID) {
		strcpy(mxt->device_info.family_name, "mXT1386");

		if (mxt->device_info.variant_id == MXT1386_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else {
			dev_err(&client->dev,
				"Warning: maXTouch Variant ID [%d] not "
				"supported\n", mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}
		/* Unknown family ID! */
	} else {
		dev_err(&client->dev,
			"Warning: maXTouch Family ID [%d] not supported\n",
			mxt->device_info.family_id);
		strcpy(mxt->device_info.family_name, "UNKNOWN");
		strcpy(mxt->device_info.variant_name, "UNKNOWN");
		/* identified = -ENXIO; */
	}

	dev_info(&client->dev,
		 "Atmel maXTouch (Family %s (%X), Variant %s (%X)) Firmware "
		 "version [%d.%d] Build %d\n",
		 mxt->device_info.family_name,
		 mxt->device_info.family_id,
		 mxt->device_info.variant_name,
		 mxt->device_info.variant_id,
		 mxt->device_info.major,
		 mxt->device_info.minor, mxt->device_info.build);
	dev_info(&client->dev,
		 "Atmel maXTouch Configuration "
		 "[X: %d] x [Y: %d]\n",
		 mxt->device_info.x_size, mxt->device_info.y_size);
	return identified;
}

/*
 * Reads the object table from maXTouch chip to get object data like
 * address, size, report id. For Info Block CRC calculation, already read
 * id data is passed to this function too (Info Block consists of the ID
 * block and object table).
 *
 */
static int __devinit mxt_read_object_table(struct i2c_client *client,
					   struct mxt_data *mxt,
					   u8 *raw_id_data)
{
	u16 report_id_count;
	u8 buf[MXT_OBJECT_TABLE_ELEMENT_SIZE];
	u8 *raw_ib_data;
	u8 object_type;
	u16 object_address;
	u16 object_size;
	u8 object_instances;
	u8 object_report_ids;
	u16 object_info_address;
	u32 crc;
	u32 calculated_crc;
	int i;
	int error;

	u8 object_instance;
	u8 object_report_id;
	u8 report_id;
	int first_report_id;
	int ib_pointer;
	struct mxt_object *object_table;

	mxt_debug(DEBUG_TRACE, "maXTouch driver reading configuration\n");

	object_table = kzalloc(sizeof(struct mxt_object) *
			       mxt->device_info.num_objs, GFP_KERNEL);
	if (object_table == NULL) {
		printk(KERN_WARNING "maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_object_table_alloc;
	}

	raw_ib_data = kmalloc(MXT_OBJECT_TABLE_ELEMENT_SIZE *
			      mxt->device_info.num_objs + MXT_ID_BLOCK_SIZE,
			      GFP_KERNEL);
	if (raw_ib_data == NULL) {
		printk(KERN_WARNING "maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_ib_alloc;
	}

	/* Copy the ID data for CRC calculation. */
	memcpy(raw_ib_data, raw_id_data, MXT_ID_BLOCK_SIZE);
	ib_pointer = MXT_ID_BLOCK_SIZE;

	mxt->object_table = object_table;

	mxt_debug(DEBUG_TRACE, "maXTouch driver Memory allocated\n");

	object_info_address = MXT_ADDR_OBJECT_TABLE;

	report_id_count = 0;
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		mxt_debug(DEBUG_TRACE, "Reading maXTouch at [0x%04x]: ",
			  object_info_address);

		error = mxt_read_block(client, object_info_address,
				       MXT_OBJECT_TABLE_ELEMENT_SIZE, buf);

		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&client->dev,
				"maXTouch Object %d could not be read\n", i);
			error = -EIO;
			goto err_object_read;
		}

		memcpy(raw_ib_data + ib_pointer, buf,
		       MXT_OBJECT_TABLE_ELEMENT_SIZE);
		ib_pointer += MXT_OBJECT_TABLE_ELEMENT_SIZE;

		object_type = buf[0];
		object_address = (buf[2] << 8) + buf[1];
		object_size = buf[3] + 1;
		object_instances = buf[4] + 1;
		object_report_ids = buf[5];
		mxt_debug(DEBUG_TRACE, "Type=%03d, Address=0x%04x, "
			  "Size=0x%02x, %d instances, %d report id's\n",
			  object_type,
			  object_address,
			  object_size, object_instances, object_report_ids);

		/* TODO: check whether object is known and supported? */

		/* Save frequently needed info. */
		if (object_type == MXT_GEN_MESSAGEPROCESSOR_T5) {
			mxt->msg_proc_addr = object_address;
			mxt->message_size = object_size;
			printk(KERN_ALERT "message length: %d", object_size);
		}

		object_table[i].type = object_type;
		object_table[i].chip_addr = object_address;
		object_table[i].size = object_size;
		object_table[i].instances = object_instances;
		object_table[i].num_report_ids = object_report_ids;
		report_id_count += object_instances * object_report_ids;

		object_info_address += MXT_OBJECT_TABLE_ELEMENT_SIZE;
	}

	mxt->rid_map =
	    kzalloc(sizeof(struct report_id_map) * (report_id_count + 1),
		    /* allocate for report_id 0, even if not used */
		    GFP_KERNEL);
	if (mxt->rid_map == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_rid_map_alloc;
	}

	mxt->messages = kzalloc(mxt->message_size * MXT_MESSAGE_BUFFER_SIZE,
				GFP_KERNEL);
	if (mxt->messages == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_msg_alloc;
	}

	mxt->last_message = kzalloc(mxt->message_size, GFP_KERNEL);
	if (mxt->last_message == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_msg_alloc;
	}

	mxt->report_id_count = report_id_count;
	if (report_id_count > 254) {	/* 0 & 255 are reserved */
		dev_err(&client->dev,
			"Too many maXTouch report id's [%d]\n",
			report_id_count);
		error = -ENXIO;
		goto err_max_rid;
	}

	/* Create a mapping from report id to object type */
	report_id = 1;		/* Start from 1, 0 is reserved. */

	/* Create table associating report id's with objects & instances */
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		for (object_instance = 0;
		     object_instance < object_table[i].instances;
		     object_instance++) {
			first_report_id = report_id;
			for (object_report_id = 0;
			     object_report_id < object_table[i].num_report_ids;
			     object_report_id++) {
				mxt->rid_map[report_id].object =
				    object_table[i].type;
				mxt->rid_map[report_id].instance =
				    object_instance;
				mxt->rid_map[report_id].first_rid =
				    first_report_id;
				report_id++;
			}
		}
	}

	/* Read 3 byte CRC */
	error = mxt_read_block(client, object_info_address, 3, buf);
	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "Error reading CRC\n");
	}

	crc = (buf[2] << 16) | (buf[1] << 8) | buf[0];

	if (calculate_infoblock_crc(&calculated_crc, raw_ib_data, ib_pointer)) {
		printk(KERN_WARNING "Error while calculating CRC!\n");
		calculated_crc = 0;
	}
	kfree(raw_ib_data);

	mxt_debug(DEBUG_TRACE, "\nReported info block CRC = 0x%6X\n", crc);
	mxt_debug(DEBUG_TRACE, "Calculated info block CRC = 0x%6X\n\n",
		  calculated_crc);

	if (crc == calculated_crc) {
		mxt->info_block_crc = crc;
	} else {
		mxt->info_block_crc = 0;
		printk(KERN_ALERT "maXTouch: Info block CRC invalid!\n");
	}

	if (debug >= DEBUG_VERBOSE) {

		dev_info(&client->dev, "maXTouch: %d Objects\n",
			 mxt->device_info.num_objs);

		for (i = 0; i < mxt->device_info.num_objs; i++) {
			dev_info(&client->dev, "Type:\t\t\t[%d]: %s\n",
				 object_table[i].type,
				 object_type_name[object_table[i].type]);
			dev_info(&client->dev, "\tAddress:\t0x%04X\n",
				 object_table[i].chip_addr);
			dev_info(&client->dev, "\tSize:\t\t%d Bytes\n",
				 object_table[i].size);
			dev_info(&client->dev, "\tInstances:\t%d\n",
				 object_table[i].instances);
			dev_info(&client->dev, "\tReport Id's:\t%d\n",
				 object_table[i].num_report_ids);
		}
	}

	return 0;

 err_max_rid:
	kfree(mxt->last_message);
 err_msg_alloc:
	kfree(mxt->rid_map);
 err_rid_map_alloc:
 err_object_read:
	kfree(raw_ib_data);
 err_ib_alloc:
	kfree(object_table);
 err_object_table_alloc:
	return error;
}

static int __devinit mxt_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	/* COMPAL - START - [2011/2/16  11:40] - Jamin - [Touch]: add system node for debug  */
	//change global variable for debug
	//struct mxt_data *mxt;
	/* COMPAL - END */
	struct mxt_platform_data *pdata;
	struct input_dev *input;
	u8 *id_data;
	int error;
	unsigned char boot_status; 
	
	mxt_debug(DEBUG_INFO, "mXT224: mxt_probe\n");

	if (client == NULL) {
		pr_debug("maXTouch: client == NULL\n");
		return -EINVAL;
	} else if (client->adapter == NULL) {
		pr_debug("maXTouch: client->adapter == NULL\n");
		return -EINVAL;
	} else if (&client->dev == NULL) {
		pr_debug("maXTouch: client->dev == NULL\n");
		return -EINVAL;
	} else if (&client->adapter->dev == NULL) {
		pr_debug("maXTouch: client->adapter->dev == NULL\n");
		return -EINVAL;
	} else if (id == NULL) {
		pr_debug("maXTouch: id == NULL\n");
		return -EINVAL;
	}

	mxt_debug(DEBUG_INFO, "maXTouch driver\n");
	mxt_debug(DEBUG_INFO, "\t \"%s\"\n", client->name);
	mxt_debug(DEBUG_INFO, "\taddr:\t0x%04x\n", client->addr);
	mxt_debug(DEBUG_INFO, "\tirq:\t%d\n", client->irq);
	mxt_debug(DEBUG_INFO, "\tflags:\t0x%04x\n", client->flags);
	mxt_debug(DEBUG_INFO, "\tadapter:\"%s\"\n", client->adapter->name);
	mxt_debug(DEBUG_INFO, "\tdevice:\t\"%s\"\n", client->dev.init_name);
	    
	/* Check if the I2C bus supports BYTE transfer */		
	error = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE);
	dev_info(&client->dev, "RRC:  i2c_check_functionality = %i\n", error);
	error = 0xff;
/*
	if (!error) {
		dev_err(&client->dev, "maXTouch driver\n");
		dev_err(&client->dev, "\t \"%s\"\n", client->name);
		dev_err(&client->dev, "\taddr:\t0x%04x\n", client->addr);
		dev_err(&client->dev, "\tirq:\t%d\n", client->irq);
		dev_err(&client->dev, "\tflags:\t0x%04x\n", client->flags);
		dev_err(&client->dev, "\tadapter:\"%s\"\n",
			client->adapter->name);
		dev_err(&client->dev, "\tdevice:\t\"%s\"\n",
			client->dev.init_name);
		dev_err(&client->dev, "%s adapter not supported\n",
				dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}
*/
	mxt_debug(DEBUG_TRACE, "maXTouch driver functionality OK\n");

	/* Allocate structure - we need it to identify device */
	mxt = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (mxt == NULL) {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_mxt_alloc;
	}

	id_data = kmalloc(MXT_ID_BLOCK_SIZE, GFP_KERNEL);
	if (id_data == NULL) {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_id_alloc;
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "error allocating input device\n");
		error = -ENOMEM;
		goto err_input_dev_alloc;
	}

	/* Initialize Platform data */

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "platform data is required!\n");
		error = -EINVAL;
		goto err_pdata;
	}
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "Platform OK: pdata = 0x%08x\n",
		       (unsigned int)pdata);

	mxt->read_fail_counter = 0;
	mxt->message_counter = 0;
	mxt->max_x_val = pdata->max_x;
	mxt->max_y_val = pdata->max_y;

	/* Get data that is defined in board specific code. */
	mxt->init_hw = pdata->init_platform_hw;
	mxt->exit_hw = pdata->exit_platform_hw;
	/* COMPAL - START - [2011/3/24  11:47] - Jamin - [Touch]: add reset node  */
       mxt->reset_hw = pdata->reset_platform_hw;
	mxt->read_reset= pdata->read_reset;
	/* COMPAL - END */
	mxt->read_chg = pdata->read_chg;

	if (pdata->valid_interrupt != NULL)
		mxt->valid_interrupt = pdata->valid_interrupt;
	else
		mxt->valid_interrupt = mxt_valid_interrupt_dummy;

	if (mxt->init_hw != NULL)
		mxt->init_hw();

	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "maXTouch driver identifying chip\n");

       mxt->client = client;
	mxt->input = input;
	
	/* COMPAL - START - [2011/6/20  20:19] - Jamin - [Touch]: update firmware 1.1  */
	#ifdef ENABLE_AUTO_UPGRADE_FW11
	if(-EIO==mxt_write_byte(mxt->client, 275, 0)) 
	{
  	//can not use app i2c address write,so try i2c boot address
  	  	if(mxt_boot_read(0,1,&boot_status) == I2C_BOOT_READ_OK)
  	  	{
  	 		dev_info(&mxt->client->dev, "detect bootloader mode ...\n");
                    //with no reset
			update_firmware(false);
			//read machine version again
			mxt_read_block(mxt->client,264,1,&mxt->check_resistance); //T38[0]
    			mxt_read_block(mxt->client,265,1,&mxt->check_version); //T38[1]
			//write machine version configure
			if(((mxt->check_resistance==65) && (mxt->check_version==2)))
			{
	  			Atmel_WriteAllConfig(mxt,TOUCH_65OHM_V2);
			}
			else if( ((mxt->check_resistance==65) && (mxt->check_version==1)))
			{
				Atmel_WriteAllConfig(mxt,TOUCH_65OHM_V1);
			}
			else
			{
	    			Atmel_WriteAllConfig(mxt,TOUCH_30OHM);
			}
			Atmel_WriteBack(mxt);
  	  	}
	}
	#endif
	/* COMPAL - END */

	if (mxt_identify(client, mxt, id_data) < 0) {
		dev_err(&client->dev, "Chip could not be identified\n");
		error = -ENODEV;
		goto err_identify;
	}

	/* Chip is valid and active. */
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "maXTouch driver allocating input device\n");

      /* COMPAL - START - [2011/6/20  21:11] - Jamin - [Touch]: update firmware 1.1  */
	  #ifdef ENABLE_AUTO_UPGRADE_FW11
	   if( (mxt->device_info.major!=1) ||( mxt->device_info.minor!=1))
	   {
	        //detect firmware not 1.1
	        dev_info(&client->dev, "Firmware version [%d.%d] \n",mxt->device_info.major,mxt->device_info.minor);
              //read machine version first
		 mxt_read_block(mxt->client,264,1,&mxt->check_resistance); //T38[0]
    		 mxt_read_block(mxt->client,265,1,&mxt->check_version); //T38[1]
    		  //do upgrade firmware1.1
		  update_firmware(true);
		 //write machine version configure
		 if(((mxt->check_resistance==65) && (mxt->check_version==2)))
		 {
	  		Atmel_WriteAllConfig(mxt,TOUCH_65OHM_V2);
		 }
		 else if( ((mxt->check_resistance==65) && (mxt->check_version==1)))
		 {
			Atmel_WriteAllConfig(mxt,TOUCH_65OHM_V1);
		 }
		 else
		 {
	    		Atmel_WriteAllConfig(mxt,TOUCH_30OHM);
		 }
		Atmel_WriteBack(mxt);
		
		  //read chip information again
		  memset(id_data,0,MXT_ID_BLOCK_SIZE);
		  if (mxt_identify(client, mxt, id_data) < 0) {
			dev_err(&client->dev, "Chip could not be identified after update firmware 1.1\n");
			error = -ENODEV;
			goto err_identify;
		 } 
	   }
	  #endif 
      /* COMPAL - END */
	  
	INIT_DELAYED_WORK(&mxt->dwork, mxt_worker);
	/* COMPAL - START - [2011/5/25  13:56] - Jamin - [Touch]: add retry auto calibration mechanism     */
	#ifdef ENABLE_AUTO_CALIBRATION
	INIT_DELAYED_WORK(&mxt->dcal_work, mxt_calibrate_work_func);
       init_timer(&mxt->cal_timer);
	mxt->cal_timer.data = (unsigned long) mxt;
	mxt->cal_timer.function = mxt_calibrate_timer_func;
      #endif
	/* COMPAL - END */
	
	mutex_init(&mxt->debug_mutex);
	mutex_init(&mxt->msg_mutex);
	mxt_debug(DEBUG_TRACE, "maXTouch driver creating device name\n");

	snprintf(mxt->phys_name,
		 sizeof(mxt->phys_name), "%s/input0", dev_name(&client->dev)
	    );
	input->name = "atmel-maxtouch";
	input->phys = mxt->phys_name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	mxt_debug(DEBUG_INFO, "maXTouch name: \"%s\"\n", input->name);
	mxt_debug(DEBUG_INFO, "maXTouch phys: \"%s\"\n", input->phys);
	mxt_debug(DEBUG_INFO, "maXTouch driver setting abs parameters\n");

	set_bit(BTN_TOUCH, input->keybit);

	/* Single touch */
	input_set_abs_params(input, ABS_X, 0, mxt->max_x_val, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, mxt->max_y_val, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, MXT_MAX_REPORTED_PRESSURE,
			     0, 0);
	input_set_abs_params(input, ABS_TOOL_WIDTH, 0, MXT_MAX_REPORTED_WIDTH,
			     0, 0);

	/* Multitouch */
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, mxt->max_x_val, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, mxt->max_y_val, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, MXT_MAX_TOUCH_SIZE,
			     0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, MXT_MAX_NUM_TOUCHES,
			     0, 0);

	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
//+Charles
	__set_bit(BTN_TOUCH, input->keybit);
//--

	mxt_debug(DEBUG_TRACE, "maXTouch driver setting client data\n");
	i2c_set_clientdata(client, mxt);
	mxt_debug(DEBUG_TRACE, "maXTouch driver setting drv data\n");
	input_set_drvdata(input, mxt);
	mxt_debug(DEBUG_TRACE, "maXTouch driver input register device\n");
	error = input_register_device(mxt->input);
	if (error < 0) {
		dev_err(&client->dev, "Failed to register input device\n");
		goto err_register_device;
	}

	error = mxt_read_object_table(client, mxt, id_data);
	if (error < 0)
		goto err_read_ot;
	
      /* COMPAL - START - [2011/3/3  11:8] - Jamin - [Touch]: enable atmel touch  */
	 //do check config , if find different set do write back
	 Atmel_CheckConfig(mxt);
	 //do read all register table for debug
	//Atmel_ReadConfig(mxt);
      /* COMPAL - END */
	  
     /* COMPAL - START - [2011/2/17  15:59] - Jamin - [Touch]: fix suspend ,resume issue  */
      #ifdef CONFIG_HAS_EARLYSUSPEND
    	mxt->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 21;
    	mxt->early_suspend.suspend = mxt_early_suspend;
    	mxt->early_suspend.resume = mxt_late_resume;
    	register_early_suspend(&mxt->early_suspend);
      #endif 
	/* COMPAL - END */

    /* COMPAL - START - [2011/1/20  10:40] - Jamin - [Touch]: enable touch atmel   */
    // not use debug entries
    #if 0    
	/* Create debugfs entries. */
	mxt->debug_dir = debugfs_create_dir("maXTouch", NULL);
	if (mxt->debug_dir == -ENODEV) {
		/* debugfs is not enabled. */
		printk(KERN_WARNING "debugfs not enabled in kernel\n");
	} else if (mxt->debug_dir == NULL) {
		printk(KERN_WARNING "error creating debugfs dir\n");
	} else {
		mxt_debug(DEBUG_TRACE, "created \"maXTouch\" debugfs dir\n");

		debugfs_create_file("deltas", S_IRUSR, mxt->debug_dir, mxt,
				    &delta_fops);
		debugfs_create_file("refs", S_IRUSR, mxt->debug_dir, mxt,
				    &refs_fops);
	}

	/* Create character device nodes for reading & writing registers */
	mxt->mxt_class = class_create(THIS_MODULE, "maXTouch_memory");
	/* 2 numbers; one for memory and one for messages */
	error = alloc_chrdev_region(&mxt->dev_num, 0, 2, "maXTouch_memory");
	mxt_debug(DEBUG_VERBOSE,
		  "device number %d allocated!\n", MAJOR(mxt->dev_num));
	if (error)
		printk(KERN_WARNING "Error registering device\n");
	cdev_init(&mxt->cdev, &mxt_memory_fops);
	cdev_init(&mxt->cdev_messages, &mxt_message_fops);

	mxt_debug(DEBUG_VERBOSE, "cdev initialized\n");
	mxt->cdev.owner = THIS_MODULE;
	mxt->cdev_messages.owner = THIS_MODULE;

	error = cdev_add(&mxt->cdev, mxt->dev_num, 1);
	if (error)
		printk(KERN_WARNING "Bad cdev\n");

	error = cdev_add(&mxt->cdev_messages, mxt->dev_num + 1, 1);
	if (error)
		printk(KERN_WARNING "Bad cdev\n");

	mxt_debug(DEBUG_VERBOSE, "cdev added\n");

	device_create(mxt->mxt_class, NULL, MKDEV(MAJOR(mxt->dev_num), 0), NULL,
		      "maXTouch");

	device_create(mxt->mxt_class, NULL, MKDEV(MAJOR(mxt->dev_num), 1), NULL,
		      "maXTouch_messages");
	#endif
    /* COMPAL - END */

	mxt->msg_buffer_startp = 0;
	mxt->msg_buffer_endp = 0;

	/* Allocate the interrupt */
	mxt_debug(DEBUG_TRACE, "maXTouch driver allocating interrupt...\n");
	mxt->irq = client->irq;
	mxt->valid_irq_counter = 0;
	mxt->invalid_irq_counter = 0;
	mxt->irq_counter = 0;
	if (mxt->irq) {
		/* Try to request IRQ with falling edge first. This is
		 * not always supported. If it fails, try with any edge. */
		error = request_irq(mxt->irq,
				    mxt_irq_handler,
				    IRQF_TRIGGER_FALLING,
				    client->dev.driver->name, mxt);
		if (error < 0) {
			/* TODO: why only 0 works on STK1000? */
			error = request_irq(mxt->irq,
					    mxt_irq_handler,
					    0, client->dev.driver->name, mxt);
		}

		if (error < 0) {
			dev_err(&client->dev,
				"failed to allocate irq %d\n", mxt->irq);
			goto err_irq;
		}
	}

	if (debug > DEBUG_INFO)
		dev_info(&client->dev, "touchscreen, irq %d\n", mxt->irq);

	/* Schedule a worker routine to read any messages that might have
	 * been sent before interrupts were enabled. */
	cancel_delayed_work(&mxt->dwork);
	schedule_delayed_work(&mxt->dwork, 0);
	kfree(id_data);

	/*
	   TODO: REMOVE!!!!!!!!!!!!!!!!!!!!!!!

	   REMOVE!!!!!!!!!!!!!!!!!!!!!!!
	 */
	 
    /* COMPAL - START - [2011/1/19  13:34] - Jamin - [Touch]: enable touch atmel */
	//jamin mark
	//mxt_write_byte(mxt->client,
	//	       MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), 15);
    /* COMPAL - END */
  

/* COMPAL - START - [2011/2/16  10:31] - Jamin - [Touch]: add system node for debug  */
 touchdebug_kobj = kobject_create_and_add("Touch", NULL);
    if (touchdebug_kobj == NULL) {
        printk("%s: subsystem_register failed\n", __FUNCTION__);
    }
    error = sysfs_create_group(touchdebug_kobj, &attr_group);
    if(error) {
        printk("%s: sysfs_create_group failed, %d\n", __FUNCTION__, __LINE__);
    }
/* COMPAL - END */ 
	return 0;

 err_irq:
	kfree(mxt->rid_map);
	kfree(mxt->object_table);
	kfree(mxt->last_message);
 err_read_ot:
 err_register_device:
 err_identify:
 err_pdata:
	input_free_device(input);
 err_input_dev_alloc:
	kfree(id_data);
 err_id_alloc:
	if (mxt->exit_hw != NULL)
		mxt->exit_hw();
	kfree(mxt);
 err_mxt_alloc:
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);

       /* COMPAL - START - [2011/1/20  10:40] - Jamin - [Touch]: enable touch atmel   */
   	// not use debug entries
   	#if 0
	/* Remove debug dir entries */
	debugfs_remove_recursive(mxt->debug_dir);
      #endif
	/* COMPAL - END */
	  
	if (mxt != NULL) {

		if (mxt->exit_hw != NULL)
			mxt->exit_hw();

		if (mxt->irq)
			free_irq(mxt->irq, mxt);
		
		/* COMPAL - START - [2011/1/20  10:40] - Jamin - [Touch]: enable touch atmel   */
   		 // not use debug entries
   		 #if 0
		unregister_chrdev_region(mxt->dev_num, 2);
		device_destroy(mxt->mxt_class, MKDEV(MAJOR(mxt->dev_num), 0));
		device_destroy(mxt->mxt_class, MKDEV(MAJOR(mxt->dev_num), 1));
		cdev_del(&mxt->cdev);
		cdev_del(&mxt->cdev_messages);
		#endif
		/* COMPAL - END */
		cancel_delayed_work_sync(&mxt->dwork);
		/* COMPAL - START - [2011/5/25  15:16] - Jamin - [Touch]: add retry auto calibration mechanism     */
             #ifdef ENABLE_AUTO_CALIBRATION
		cancel_delayed_work_sync(&mxt->dcal_work);
		del_timer(&mxt->cal_timer);
		#endif
		/* COMPAL - END */
		
		input_unregister_device(mxt->input);
		
		 /* COMPAL - START - [2011/1/20  10:40] - Jamin - [Touch]: enable touch atmel   */
   		 // not use debug entries
   		 #if 0
		class_destroy(mxt->mxt_class);
		debugfs_remove(mxt->debug_dir);
              #endif
		/* COMPAL - END */
			  
		kfree(mxt->rid_map);
		kfree(mxt->object_table);
		kfree(mxt->last_message);
	}
   	sysfs_remove_group(touchdebug_kobj,&attr_group);
	kfree(mxt);

	i2c_set_clientdata(client, NULL);
	if (debug >= DEBUG_TRACE)
		dev_info(&client->dev, "Touchscreen unregistered\n");

	return 0;
}

/* COMPAL - START - [2011/2/17  15:59] - Jamin - [Touch]: fix suspend ,resume issue  */
#if 0
static void mxt_start(struct mxt_data *mxt)
{
	//let touch enter normal state
	mxt_write_byte(mxt->client,328, 255);
	mxt_write_byte(mxt->client,329, 255);
	//mxt_write_byte(mxt->client,330, 0);
}
#endif
static void mxt_stop(struct mxt_data *mxt)
{
	//let touch enter sleep state
	mxt_write_byte(mxt->client,328, 0x0);
	mxt_write_byte(mxt->client,329, 0x0);
	// fix no ack issue	
	//	mxt_write_byte(mxt->client,330, 0x0);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void force_release_pos(void)
{
	int i;
	for (i=0; i<=9; i++){
		if (stored_size[i] ==0) continue;
		
		input_report_abs(mxt->input, ABS_MT_TRACKING_ID, i);
		input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, 0);
		stored_size[i]=0;
		input_mt_sync(mxt->input);
	}
	input_sync(mxt->input);

	 //clear all point
	memset(stored_size,0,sizeof(stored_size));
	memset(stored_x,0,sizeof(stored_x));
	memset(stored_y,0,sizeof(stored_y));
}
void mxt_early_suspend(struct early_suspend *h)
{
    	struct mxt_data *mxt = container_of(h, struct mxt_data, early_suspend);
       mxt->suspend_count++;
     	dev_info(&mxt->client->dev,"[Touch]: mxt_early_suspend...%d+\n",mxt->suspend_count); 

    //fix no ack issue
	cancel_delayed_work_sync(&mxt->dwork);
	disable_irq(mxt->irq);
	cancel_delayed_work_sync(&mxt->dwork);

	 /* COMPAL - START - [2011/5/25  15:23] - Jamin - [Touch]: add retry auto calibration mechanism     */
       //cancel work queue
       #ifdef ENABLE_AUTO_CALIBRATION
       cancel_delayed_work_sync(&mxt->dcal_work);
       del_timer(&mxt->cal_timer);
        #endif
	 /* COMPAL - END */

    msleep(100);

	 //let atmel touch enter sleep state
	 mxt_stop(mxt);

	//clear all point location and force release
	 force_release_pos();
	
	dev_info(&mxt->client->dev,"[Touch]: mxt_early_suspend...%d-\n",mxt->suspend_count); 
}

void mxt_late_resume(struct early_suspend *h)
{
       unsigned char do_reset=0;
	struct mxt_data *mxt = container_of(h, struct mxt_data, early_suspend);
	dev_info(&mxt->client->dev,"[Touch]: mxt_late_resume...%d+\n",mxt->suspend_count); 


	//write T38[10]=1 ,fix no ack  and indicate do late resume now
	if(-EIO==mxt_write_byte(mxt->client, 274, 1))  //T38[10]
	{
	  	do_reset=1;
	}

	 msleep(100);
			
	 //let touch enter normal state
	 if(-EIO==mxt_write_byte(mxt->client,328,65))
	 {
	       do_reset=1;
	 }
        if(-EIO==mxt_write_byte(mxt->client,329,255))
	 {
	       do_reset=1;
	 }
		 
	  //enable anti touch and auto calibration
         if(-EIO==mxt_write_block(mxt->client,337,4,&AcqOBJ[6]))
	  {
	       do_reset=1;
	  }

       //write T38[10]=0 --> indicate do late resume done
	if(-EIO==mxt_write_byte(mxt->client, 274, 0))  //T38[10]
	{
	  do_reset=1;
	}

	if(do_reset==1 || mxt->read_chg() == 0)
	{
	  mxt->reset_hw();
	  msleep(100);
	  printk(KERN_ERR "mxt i2c write fail or chg pin keep low so do hardware reset  \n");
	}

	enable_irq(mxt->irq);
	 
	dev_info(&mxt->client->dev,"[Touch]: mxt_late_resume...%d-\n",mxt->suspend_count); 
}
#endif
/* COMPAL - END */

#if defined(CONFIG_PM)
static int mxt_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mxt_data *mxt = i2c_get_clientdata(client);
       dev_info(&mxt->client->dev,"[Touch]: mxt_suspend ...+ \n"); 
	  #if 0
	//disable irq
	disable_irq(mxt->irq);
       cancel_work_sync(&mxt->dwork);
       #endif
	dev_info(&mxt->client->dev,"[Touch]: mxt_suspend ...- \n"); 
	return 0;
}

static int mxt_resume(struct i2c_client *client)
{
	struct mxt_data *mxt = i2c_get_clientdata(client);
       dev_info(&mxt->client->dev,"[Touch]: mxt_resume ...+ \n"); 
	  #if 0 
	//enable irq
    	enable_irq(mxt->irq);
       #endif
	dev_info(&mxt->client->dev,"[Touch]: mxt_resume ...- \n"); 
	return 0;
}
#else
#define mxt_suspend NULL
#define mxt_resume NULL
#endif

static const struct i2c_device_id mxt_idtable[] = {
	{"maXTouch", 0,},
	{}
};

MODULE_DEVICE_TABLE(i2c, mxt_idtable);

static struct i2c_driver mxt_driver = {
	.driver = {
		   .name = "maXTouch",
		   .owner = THIS_MODULE,
		   },

	.id_table = mxt_idtable,
	.probe = mxt_probe,
	.remove = __devexit_p(mxt_remove),
	/* COMPAL - START - [2011/4/7  20:8] - Jamin - [Touch]: disable suspend resume but keep early suspend and late resume function  */
	#if 0
	.suspend = mxt_suspend,
	.resume = mxt_resume,
       #endif 
	/* COMPAL - END */

};

static int __init mxt_init(void)
{
	int err;
	err = i2c_add_driver(&mxt_driver);
	if (err) {
		printk(KERN_WARNING "Adding maXTouch driver failed "
		       "(errno = %d)\n", err);
	} else {
		mxt_debug(DEBUG_TRACE, "Successfully added driver %s\n",
			  mxt_driver.driver.name);
	}
	return err;
}

static void __exit mxt_cleanup(void)
{
	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_cleanup);

MODULE_AUTHOR("Iiro Valkonen");
MODULE_DESCRIPTION("Driver for Atmel maXTouch Touchscreen Controller");
MODULE_LICENSE("GPL");
