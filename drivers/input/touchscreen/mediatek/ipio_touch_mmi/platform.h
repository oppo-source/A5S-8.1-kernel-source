/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - platform.c
** Description : This program is for ili9881 driver platform.c
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#ifndef __PLATFORM_H
#define __PLATFORM_H

struct ilitek_platform_data {

	struct i2c_client *client;

	struct input_dev *input_device;

	const struct i2c_device_id *i2c_id;

#ifdef REGULATOR_POWER_ON
	struct regulator *vdd;
	struct regulator *vdd_i2c;
#endif

	struct mutex plat_mutex;
    struct mutex report_mutex;
	struct mutex gesture_mutex;

	struct regulator *gpio_pwr;
	struct regulator *lab_pwr;
	struct regulator *ibb_pwr;

	spinlock_t plat_spinlock;

	uint32_t chip_id;

	int int_gpio;
	int reset_gpio;
	int lcd_reset_gpio;
	int isr_gpio;
	int boot_mode;
	int delay_time_high;
	int delay_time_low;
	int edge_delay;

	bool isEnableIRQ;
	bool isEnablePollCheckPower;

#ifdef USE_KTHREAD
	struct task_struct *irq_thread;
	bool irq_trigger;
	bool free_irq_thread;
#else
	struct work_struct report_work_queue;
#endif
    struct work_struct resume_work_queue;
#ifdef CONFIG_FB
	struct notifier_block notifier_fb;
#else
	struct early_suspend early_suspend;
#endif

#ifdef BOOT_FW_UPGRADE
	struct task_struct *update_thread;
#endif

	/* obtain msg when battery status has changed */
	struct delayed_work check_power_status_work;
	struct workqueue_struct *check_power_status_queue;
	unsigned long work_delay;
	bool vpower_reg_nb;
	bool black_test_flag;

	/* Sending report data to users for the debug */
	bool debug_node_open;
	int debug_data_frame;
	wait_queue_head_t inq;
	unsigned char debug_buf[1024][2048];
	struct mutex ilitek_debug_mutex;
	struct mutex ilitek_debug_read_mutex;
#if 1
#ifdef MING_TEST
	bool delta_node_open;
	int delta_data_frame;
	wait_queue_head_t delta_inq;

	wait_queue_head_t otp_inq;
	int otp_move_flag;
	//unsigned char delta_buf[2048];
	struct mutex ilitek_delta_mutex;
	struct mutex ilitek_delta_read_mutex;
#endif
#endif

	//char *fw_name;
	int common_reset;
	struct spi_device *spi;

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp,2018/10/11,Add touch-info file function
	struct work_struct resume_work;
	bool suspend;
	struct touch_info_dev *tid;
#endif
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/05,four ICs adjust
	int vendor_id;
#endif
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/10, add ear phone function
	unsigned long headset_state;
	struct work_struct headset_work_queue;
	struct notifier_block notifier_headset;
#endif
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/17, add presure report function
	int presure_speed;
#endif
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/1/2, add LCD ID PIN  function
	int lcd_id0;
	int lcd_id1;
	int lcd_id2;
#endif

};

extern struct ilitek_platform_data *ipd;

/* exported from platform.c */
extern void ilitek_platform_disable_irq(void);
extern void ilitek_platform_enable_irq(void);
extern int ilitek_platform_tp_hw_reset(bool isEnable);
extern void ilitek_esd_gesture_reset(void);
extern void oppo_platform_tp_hw_reset(bool isEnable);

#ifdef ENABLE_REGULATOR_POWER_ON
extern void ilitek_regulator_power_on(bool status);
#endif

/* exported from userspsace.c */
extern void netlink_reply_msg(void *raw, int size);
extern int ilitek_proc_init(void);
extern void ilitek_proc_remove(void);

#endif /* __PLATFORM_H */
