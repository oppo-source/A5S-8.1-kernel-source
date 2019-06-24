/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2018, Focaltech Ltd. All rights reserved.
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
/*****************************************************************************
*
* File Name: focaltech_core.h

* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

#ifndef __LINUX_FOCALTECH_CORE_H__
#define __LINUX_FOCALTECH_CORE_H__
/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/unistd.h>
#include <linux/ioctl.h>
#include <linux/vmalloc.h>
#include "focaltech_common.h"

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp,2018/10/11,Add touch-info file function
#include<linux/input/touch-info.h>
#include<linux/workqueue.h>
#endif

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_MAX_POINTS_SUPPORT              10 /* constant value, can't be changed */
#define FTS_MAX_KEYS                        4
#define FTS_KEY_WIDTH                       50
#define FTS_ONE_TCH_LEN                     6
#define FTS_SPI_BUFSIZ_MAX                  (4 * 1024)
#define FTS_SPI_CLK_MAX                     6000000

#define FTS_MAX_ID                          0x0A
#define FTS_TOUCH_X_H_POS                   3
#define FTS_TOUCH_X_L_POS                   4
#define FTS_TOUCH_Y_H_POS                   5
#define FTS_TOUCH_Y_L_POS                   6
#define FTS_TOUCH_PRE_POS                   7
#define FTS_TOUCH_AREA_POS                  8
#define FTS_TOUCH_POINT_NUM                 2
#define FTS_TOUCH_EVENT_POS                 3
#define FTS_TOUCH_ID_POS                    5
#define FTS_COORDS_ARR_SIZE                 4

#define FTS_GESTURE_POINT_MAX               6
#define FTS_GESTRUE_POINTS_HEADER           4 /* Enable + Reserve + Header */
#define FTS_GESTURE_DATA_LEN                (FTS_GESTRUE_POINTS_HEADER + FTS_GESTURE_POINT_MAX * 4)
#define FTS_GESTURE_OFF                     63

#define FTS_TOUCH_DOWN                      0
#define FTS_TOUCH_UP                        1
#define FTS_TOUCH_CONTACT                   2
#define EVENT_DOWN(flag)                    ((FTS_TOUCH_DOWN == flag) || (FTS_TOUCH_CONTACT == flag))
#define EVENT_UP(flag)                      (FTS_TOUCH_UP == flag)
#define EVENT_NO_DOWN(data)                 (!data->point_num)
#define KEY_EN(data)                        (data->pdata->have_key)
#define TOUCH_IS_KEY(y, key_y)              (y == key_y)
#define TOUCH_IN_RANGE(val, key_val, half)  ((val > (key_val - half)) && (val < (key_val + half)))
#define TOUCH_IN_KEY(x, key_x)              TOUCH_IN_RANGE(x, key_x, FTS_KEY_WIDTH)

#define FTX_MAX_COMPATIBLE_TYPE             4
#define SPI_BUF_LENGTH                      256
#define FTS_TOUCH_MAX_WIDTH 				720
#define FTS_TOUCH_MAX_HEIGHT 				1520


#define FTS_MTK_PLATFORM					1

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
typedef enum {
	MODE_EDGE = 0,
	MODE_EDGE_ENABLE,	
	MODE_CHARGE,
	MODE_GAME
} FTS_TP_MODE;

/*
* header        -   byte0:gesture id
*                   byte1:pointnum
*                   byte2~7:reserved
* coordinate_x  -   All gesture point x coordinate
* coordinate_y  -   All gesture point y coordinate
* mode          -   1:enable gesture function(default)
*               -   0:disable
* active        -   1:enter into gesture(suspend)
*                   0:gesture disable or resume
*/
struct fts_gesture_st {
    u8 header[FTS_GESTRUE_POINTS_HEADER];
    u16 coordinate_x[FTS_GESTURE_POINT_MAX];
    u16 coordinate_y[FTS_GESTURE_POINT_MAX];
    u8 mode;   /*host driver enable gesture flag*/
    u8 active;  /*gesture actutally work*/
};

struct fts_oppo_gesture {
    u8 gesture_id;
    bool clock_wise;    //for 'O'
    u16 start_x;
    u16 start_y;
    u16 end_x;
    u16 end_y;
    u16 point1_x;
    u16 point1_y;
    u16 point2_x;
    u16 point2_y;
    u16 point3_x;
    u16 point3_y;
    u16 point4_x;
    u16 point4_y;
};

struct _tp_limit {
	/* BIT0: global control,
	   BIT1: left up,
	   BIT2: right up,
	   BIT3, left bottom,
	   BIT4: right bottom */
	union _tp_limit_enable {
		u32 value;
		struct bits {
			u32 edge_limit		: 1;
			u32 lu				: 1;
			u32 ru				: 1;
			u32 lb				: 1;
			u32 rb				: 1;
			u32 reserved		: 27;
		} bit;
	} enable;
	u16 xlu;
	u16 ylu;
	u16 xru;
	u16 yru;
	u16 xlb;
	u16 ylb;
	u16 xrb;
	u16 yrb;
} ;


struct fts_oppo_data {
	struct proc_dir_entry *touchpanel_dir_entry;
	struct proc_dir_entry *baseline_test_entry;
	struct proc_dir_entry *coordinate_entry;
	struct proc_dir_entry *debug_level_entry;
	struct proc_dir_entry *double_tap_enable_entry;
	struct proc_dir_entry *irq_depth_entry;
	struct proc_dir_entry *oppo_register_info_entry;
	struct proc_dir_entry *oppo_tp_limit_area_entry;
	struct proc_dir_entry *oppo_tp_limit_enable_entry;
	struct proc_dir_entry *ps_status_entry;
	struct proc_dir_entry *tp_fw_update_entry;
	struct proc_dir_entry *black_screen_test_entry;

	struct proc_dir_entry *debug_info_dir_entry;
	struct proc_dir_entry *delta_entry;
	struct proc_dir_entry *baseline_entry;
	struct proc_dir_entry *main_register_entry;

	struct proc_dir_entry *touchscreen_dir_entry;
	struct proc_dir_entry *openshort_entry;

	struct fts_oppo_gesture gesture_data;
	int gesture_test_flag;
	int gesture_backup;
	int lcm_gesture_power;
	
	struct _tp_limit tp_limit;

};

struct ftxxxx_proc {
    struct proc_dir_entry *proc;
    u8 opmode;
    u8 cmd_len;
    u8 cmd[SPI_MAX_COMMAND_LENGTH];
};

struct fts_ts_platform_data {
    u32 irq_gpio;
    u32 irq_gpio_flags;
    u32 reset_gpio;
    u32 reset_gpio_flags;
    bool have_key;
    u32 key_number;
    u32 keys[FTS_MAX_KEYS];
    u32 key_y_coord;
    u32 key_x_coords[FTS_MAX_KEYS];
    u32 x_max;
    u32 y_max;
    u32 x_min;
    u32 y_min;
    u32 max_touch_number;
};

struct ts_event {
    int x; /*x coordinate */
    int y; /*y coordinate */
    int p; /* pressure */
    int flag; /* touch event flag: 0 -- down; 1-- up; 2 -- contact */
    int id;   /*touch ID */
    int area;
	bool in_limit_area;
	bool need_report;
};

struct fts_ts_data {
    struct spi_device *spi;
    struct spi_device *client;
    struct input_dev *input_dev;
    struct fts_ts_platform_data *pdata;
    struct ts_ic_info ic_info;
    struct workqueue_struct *ts_workqueue;
    struct delayed_work esdcheck_work;
    struct work_struct fwupg_work;
    struct delayed_work prc_work;
	struct task_struct *thread_tpd;
    struct ftxxxx_proc *proc;
	struct fts_oppo_data *oppo_vendor_data;
	struct fts_gesture_st *gesture;
    spinlock_t irq_lock;
    struct mutex report_mutex;
    struct mutex spilock;
    int irq;
    int fw_is_running;      /* confirm fw is running when using spi:default 0 */
    bool suspended;
    bool fw_loading;
    bool irq_disabled;
    bool glove_mode;
    bool cover_mode;
    bool charger_mode;
    /* multi-touch */
    struct ts_event *events;
    u8 *spibuf;
    u8 *point_buf;
    int pnt_buf_size;
    int touchs;
    bool key_down;
    int touch_point;
    int point_num;
#if defined(CONFIG_FB)
    struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend early_suspend;
#endif
    struct touch_info_dev *tid;
};


/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern struct fts_ts_data *fts_data;
extern u8 fts_oppo_debug_level;
extern int g_gesture;


extern int fts_proc_init(struct fts_ts_data *ts_data);

/* spi interface communication*/
int fts_read(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen);
int fts_read_reg(u8 regaddr, u8 *regvalue);
int fts_write(u8 *writebuf, u32 writelen);
int fts_write_reg(u8 regaddr, u8 regvalue);
int fts_read_status(u8 *status);

/* Gesture functions */
#if FTS_GESTURE_EN
int fts_gesture_init(struct fts_ts_data *ts_data);
int fts_gesture_exit(struct fts_ts_data *ts_data);
void fts_gesture_recovery(void);
int fts_gesture_readdata(struct fts_ts_data *ts_data, u8 *);
int fts_gesture_suspend(struct fts_ts_data *ts_data);
int fts_gesture_resume(struct fts_ts_data *ts_data);
#endif

/* Apk and functions */
#if FTS_APK_NODE_EN
int fts_create_apk_debug_channel(struct fts_ts_data *);
void fts_release_apk_debug_channel(struct fts_ts_data *);
#endif

/* ADB functions */
#if FTS_SYSFS_NODE_EN
int fts_create_sysfs(struct device *dev);
int fts_remove_sysfs(struct device *dev);
#endif

/* ESD */
#if FTS_ESDCHECK_EN
int fts_esdcheck_init(struct fts_ts_data *ts_data);
int fts_esdcheck_exit(struct fts_ts_data *ts_data);
int fts_esdcheck_switch(bool enable);
int fts_esdcheck_proc_busy(bool proc_debug);
int fts_esdcheck_set_intr(bool intr);
int fts_esdcheck_suspend(void);
int fts_esdcheck_resume(void);
#endif

/* Production test */
#if FTS_TEST_EN
int fts_oppo_get_rawdata_diff(bool is_diff, char *buf, int *buflen);
int fts_oppo_test(bool in_blackscreen, int *test_result);
int fts_test_init(struct device *dev);
int fts_test_exit(struct device *dev);
#endif

/* Point Report Check*/
#if FTS_POINT_REPORT_CHECK_EN
int fts_point_report_check_init(struct fts_ts_data *ts_data);
int fts_point_report_check_exit(struct fts_ts_data *ts_data);
void fts_prc_queue_work(struct fts_ts_data *ts_data);
#endif

/* FW init */
int fts_fw_init(void);
int fts_fw_exit(void);
int fts_fw_resume(void);
int fts_fw_recovery(void);
int fts_upgrade_bin(char *fw_name, bool force);
int fts_fw_enter_test_environment(int test_state);
int fts_upgrade_oppo(bool force);

/* Other */
int fts_reset_proc(int hdelayms);
int fts_wait_tp_to_valid(void);
void fts_release_all_finger(void);
void fts_tp_state_recovery(void);
int fts_ex_mode_init(struct device *dev);
int fts_ex_mode_exit(struct device *dev);
int fts_ex_mode_recovery(void);
int fts_mode_switch(u8 mode, u8 value);

void fts_irq_disable(void);
void fts_irq_enable(void);
#endif /* __LINUX_FOCALTECH_CORE_H__ */
