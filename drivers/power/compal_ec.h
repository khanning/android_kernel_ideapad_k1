#ifndef _COMPAL_EC_H
#define _COMPAL_EC_H

#include <linux/i2c.h>
//#include <linux/earlysuspend.h>

////////////////////////////////////////////////////////////////////////////////
//
// Type definition
//
////////////////////////////////////////////////////////////////////////////////
// Basic types
typedef int                     INT;
typedef long                    S32;
typedef short                   S16;
typedef char                    S8;

typedef unsigned long           U32;
typedef unsigned short          U16;
typedef unsigned char           U8;

enum{
	//ec command for U1
	EC_CMD_GSET_BRIGHTNESS = 0x20,
	EC_CMD_GET_BASE_SYSTEM_STATE,
	EC_CMD_GSET_DEVICE_POWER,
	EC_CMD_GSET_VOLUME, 
	EC_CMD_SET_SLATE_SYSTEM_STATUS,
	EC_CMD_GET_QUERY_VALUE,
	EC_CMD_INVALID_MCU_REQUIREMENT,
	EC_CMD_GSET_VGA_SWITCH = 0x28,
	EC_CMD_GET_DOCKING_TYPE,
	EC_CMD_SET_BATTERY_INDEX = 0x32,
	EC_CMD_GET_BATTERY_REMAINING_CAPACITY,
	EC_CMD_GET_BATTERY_TEMP,
	EC_CMD_GET_BATTERY_VOLTAGE,
	EC_CMD_GET_BATTERY_STATUS,
	EC_CMD_GET_BATTERY_HEALTH,
	EC_CMD_GET_AC_ONLINE,
	EC_CMD_GSET_TRANSPARENT = 0x50,
	EC_CMD_GET_MCU_FW_VERSION,
	EC_CMD_GSET_WIRELESS_SWITCH_STATUS,
	EC_CMD_SET_WAKEUP_TIMER,
	EC_CMD_CHANGE_BASE_SYSTEM_POWER_STATE,
	EC_CMD_GSET_ALS_CONTROL_MODE,
	EC_CMD_DETECT_ACTION,
	EC_CMD_ASK_SLATE_CHANGE_POWER_STATE,
	EC_CMD_POWER_ON_REASON,
	EC_CMD_GSET_SERIAL_NUMBER = 0x63,
	EC_CMD_GET_SERIAL_NUMBER_FROM_EEPROM = 0x64,
	EC_CMD_INIT_DETECT = 0xb0,
	EC_CMD_FLASH_EC = 0xb7,
	EC_CMD_GET_MCU_FW_T_VERSION = 0xd0,
	EC_CMD_GET_ALS_VALUE = 0xd1,
	EC_CMD_SET_SE_TIMER = 0xdc,
	EC_CMD_GET_BATTERY_SOURCE = 0xe0,
	EC_CMD_LONG_DATA_START = 0xfe,
	EC_CMD_LONG_DATA_END = 0xff,
};

enum{
	//ec command 0x21 return value from MCU
	EC_CMD_21_BASE_OFF = 0x0,
	EC_CMD_21_BASE_OS_RUNNING,
	EC_CMD_21_BASE_SUSPENDED,
	EC_CMD_21_BASE_HIBERNATED,
	EC_CMD_21_BASE_SHUTDOWN,
	EC_CMD_21_BASE_GOING_TO_SHUTDOWN,
	EC_CMD_21_BASE_GOING_TO_REBOOT,
	EC_CMD_21_BASE_GOING_TO_SUSPEND,
	EC_CMD_21_BASE_GOING_TO_HIBERNATION,
	EC_CMD_21_BASE_RESUMING_FROM_SUSPEND,
	EC_CMD_21_BASE_POST
};

enum{
	//ec command 0x22 return value from MCU
	EC_CMD_22_POWER_HUB = 0x01,
	EC_CMD_22_POWER_CAMERA = 0x02,
	EC_CMD_22_POWER_GSENSOR = 0x04,
	EC_CMD_22_POWER_TOUCH = 0x08, 
	EC_CMD_22_POWER_OFN = 0x10,
	EC_CMD_22_POWER_HDMI = 0x20,
	EC_CMD_22_POWER_COMPASS = 0x40,
	EC_CMD_22_POWER_DCR = 0x80
};

enum{
	//ec command 0x24 return value from MCU
	EC_CMD_24_SET_SLATE_RUNNING = 0xa0,
	EC_CMD_24_SET_SLATE_SUSPEND,
	EC_CMD_24_SET_SLATE_POWER_OFF,
	EC_CMD_24_SET_SLATE_POWER_REBOOT
};

enum{
	//ec command 0x28 return value from MCU
	EC_CMD_28_SET_VGA_SWITCH_TO_QSD = 0x1,
	EC_CMD_28_SET_VGA_SWITCH_TO_X86
};

enum{
	//ec command 0x29 return value from MCU
	EC_CMD_29_DOCKING_TYPE_NONE = 0x0,
	EC_CMD_29_DOCKING_TYPE_BASE,
	EC_CMD_29_DOCKING_TYPE_KEYBOARD,
	EC_CMD_29_DOCKING_TYPE_CABLE,
	EC_CMD_29_DOCKING_TYPE_DPCABLE
};

enum{
	//ec command 0x32 return value from MCU
	EC_CMD_32_BATT1 = 0x1,
	EC_CMD_32_BATT2
};

enum{
	//ec command 0x38 return value from MCU
	EC_CMD_38_AC_ONLY = 0x1,
	EC_CMD_38_DC_ONLY,
	EC_CMD_38_AC_DC,
};

enum{
	//ec command 0x52 return value from MCU
	EC_CMD_52_WIRELESS_SWITCH_OFF = 0x0,
	EC_CMD_52_WIRELESS_SWITCH_ON
};

enum{
	//ec command 0x54 return value from MCU
	EC_CMD_54_CHANGE_BASE_SYSTEM_POWER_STATE_S0 = 0xa0,
	EC_CMD_54_CHANGE_BASE_SYSTEM_POWER_STATE_S1,
	EC_CMD_54_CHANGE_BASE_SYSTEM_POWER_STATE_S2,
	EC_CMD_54_CHANGE_BASE_SYSTEM_POWER_STATE_S3,
	EC_CMD_54_CHANGE_BASE_SYSTEM_POWER_STATE_S4,
	EC_CMD_54_CHANGE_BASE_SYSTEM_POWER_STATE_S5
};

 enum{
	//ec command 0x55 return value from MCU
	EC_CMD_55_AUTO_MODE = 0x0,
	EC_CMD_55_MANUAL_MODE
};

enum{
	//ec command 0x56 return value from MCU
	EC_CMD_56_PLUG_IN = 0xC0,
	EC_CMD_56_PLUG_OUT
};

enum{
	//ec command 0x57 return value from MCU
	EC_CMD_57_BASE_ASK_SLATE_RUNNING = 0xa0,
	EC_CMD_57_BASE_ASK_SLATE_SUSPEND,
	EC_CMD_57_BASE_ASK_SLATE_POWER_OFF
};

enum{
	//ec command 0x58 return value from MCU
	EC_CMD_57_POWER_ON_REASON_POWERBUTTON = 0xa0,
	EC_CMD_57_POWER_ON_REASON_CHARGING,
	EC_CMD_57_POWER_ON_REASON_OTHER = 0xa3
};

enum{
	//ec command 0xE0 return value from MCU
	EC_CMD_E0_BATTERY_SOURCE_SLATE = 0x0,
	EC_CMD_E0_BATTERY_SOURCE_BASE,
	EC_CMD_E0_BATTERY_SOURCE_AC,
	EC_CMD_E0_BATTERY_SOURCE_UNKNOWN,
};

// Types used in EC Lower Layer API
// Data buffer which is used for input & output data
// A package of command, buffer of input data and buffer of output dat
typedef struct
{
	U8 	action;
    U8 	command;                    // command sent to EC
    U16  data;      			   // buffer which contains input data
} EC_I2C_COMMAND_PACKAGE;

// EC Device data structure
#define BUFSIZE     128

struct compalec_dev {
    struct semaphore         mutex;
	struct work_struct       task;
	unsigned char            databuf;
    #if 0
	int                      irq_gpio;
	int                      sim_door_gpio;
	int                      glock_gpio;
    #endif
	int                      tpd_index;
	int                      ec_flash_flag;
    struct i2c_client		 *client;
    //sysfs
    #if 0
    struct kobject 			 *kobj_hybrid;
    struct kobject 			 *kobj_switch;
    struct kobject 			 *kobj_power;
    #endif
    struct device			 *dev;
    #if 0
    struct power_supply		 bat;
    struct power_supply		 ac;
    #endif
    spinlock_t		 		 lock;
    //hybrid event
    wait_queue_head_t 		 read_wait;
    struct fasync_struct	 *fifo_async;
	struct kfifo			 *fifo;
	spinlock_t				 fifo_lock;
    #if 0
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend 	 early_suspend;
#endif
	struct proc_dir_entry 	 *proc_sn;
	struct proc_dir_entry 	 *proc_pcb;
    #endif
    //EC_I2C_COMMAND_PACKAGE 	 *package;
};

#endif
