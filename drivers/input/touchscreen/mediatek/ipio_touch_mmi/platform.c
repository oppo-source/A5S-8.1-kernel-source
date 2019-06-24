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
#include "common.h"
#include "core/config.h"
#include "core/i2c.h"
#include "core/spi.h"
#include "core/firmware.h"
#include "core/finger_report.h"
#include "core/flash.h"
#include "core/protocol.h"
#include "platform.h"
#include "core/mp_test.h"
#include "core/gesture.h"
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>

#include <linux/regulator/consumer.h>
#include <linux/pm_wakeup.h>

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2019/01/11, add tpd_summer
#include <linux/tpd_summer.h>
#endif

#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"
#define DTS_LCD_RESET "share,lcd_reset-gpio"

extern int core_config_get_fw_ver(void);
extern char ili_version[20];
int ilitek_tp = 0;
#if (TP_PLATFORM == PT_MTK)
#define DTS_OF_NAME		"mediatek,cap_touch"
#include "tpd.h"
extern struct tpd_device *tpd;
#define MTK_RST_GPIO GTP_RST_PORT
#define MTK_INT_GPIO GTP_INT_PORT
#else
#define DTS_OF_NAME		"novatek,NVT-ts-spi"
#endif /* PT_MTK */

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/10, add ear phone function
#include <linux/headset_notifier.h>
#endif

//static struct wake_lock ilitek_gestrue_wakelock;
extern int get_boot_mode(void);
#define DEVICE_ID	"ILITEK_TDDI"
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/18,Add for compatible txd and hlt ctp
int ili_ctpmodule = -1;
#endif

extern bool gesture_done;
extern bool psensor_close;
#if (ILITEK_TYPE == ILITEK_TYPE_LH)
int is_tp_fw_done = 0;
#endif
#ifdef USE_KTHREAD
static DECLARE_WAIT_QUEUE_HEAD(waiter);
#endif
bool SysHaveBoot = false;
/* Debug level */
uint32_t ipio_debug_level = DEBUG_ALL;
//mfeng
uint32_t oppo_debug_level = 0;
struct wakeup_source *reload_fw_ws = NULL;
EXPORT_SYMBOL(ipio_debug_level);
EXPORT_SYMBOL(oppo_debug_level);
struct ilitek_platform_data *ipd = NULL;

#ifdef ILITEK_ESD_CHECK
	 static struct workqueue_struct *esd_wq = NULL;
	 static struct delayed_work esd_work;
	 static atomic_t ilitek_cmd_response = ATOMIC_INIT(0);
	 static unsigned long delay = 1*HZ;
	 int out_use = 0;
#endif

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp.Function.1372106,2018/05/18,Modify for tp suspend and resume
void lcd_resume_load_ili_fw(void);
#endif

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/05,four ICs adjust
unsigned char fw_txd[] = {
#include ILI_UPGRADE_FW_FILE_TXD
};

unsigned char fw_hlt[] = {
#include ILI_UPGRADE_FW_FILE_HLT
};

unsigned char fw_txd_tf[] = {
#include ILI_UPGRADE_FW_FILE_TXD_TF
};

unsigned char fw_lead_tf[] = {
#include ILI_UPGRADE_FW_FILE_LEAD_TF
};

unsigned char fw_inx[] = {
#include ILI_UPGRADE_FW_FILE_INX
};

struct upgrade_ili_fw_info ili_fw_list[] = {
    {TXD, "txd", fw_txd,(int)sizeof(fw_txd), TXD_FIRMWARE_NAME_PATH, TXD_INI_NAME_PATH},
    {HLT, "hlt", fw_hlt, (int)sizeof(fw_hlt),HLT_FIRMWARE_NAME_PATH, HLT_INI_NAME_PATH},
    {TXD_TF, "txd_tf", fw_txd_tf,(int)sizeof(fw_txd_tf), TXD_TF_FIRMWARE_NAME_PATH, TXD_TF_INI_NAME_PATH},
    {LEAD_TF, "lead_tf", fw_lead_tf, (int)sizeof(fw_lead_tf),LEAD_TF_FIRMWARE_NAME_PATH, LEAD_TF_INI_NAME_PATH},
    {INX, "inx", fw_inx, (int)sizeof(fw_inx),INX_FIRMWARE_NAME_PATH, INX_INI_NAME_PATH},
};
struct upgrade_ili_fw_info *ili_fw;
unsigned char* CTPM_FW =NULL;
#endif

#ifdef ILITEK_ESD_CHECK
void ilitek_cancel_esd_check(void)
{
	cancel_delayed_work_sync(&esd_work);
}

void ilitek_start_esd_check(void)
{
	queue_delayed_work(esd_wq, &esd_work, delay);
}
EXPORT_SYMBOL(ilitek_cancel_esd_check);
EXPORT_SYMBOL(ilitek_start_esd_check);
//int ili_power_on(struct ilitek_platform_data *ts, bool on);

static void ilitek_touch_esd_func(struct work_struct *work)
{
	//int i = 0;
	//unsigned char buf[4]={0};
	int res=0;
	ipio_debug(DEBUG_IRQ, "ILITEK esd %s: enter.......\n", __func__);

	if(core_gesture->suspend == true)
	{
		return;
	}

	if(out_use == 1){
		printk("ILITEK esd %s APK USE SO not check\n", __func__);
		goto ilitek_esd_check_out;
	}
	if(atomic_read(&ilitek_cmd_response) == 0){
		ipio_debug(DEBUG_IRQ, "ILITEK esd  check\n");
        mutex_lock(&ipd->report_mutex);
		if (!core_firmware->isUpgrading) {
		  res = core_config_get_esd_data();
		  if((res == CHECK_RECOVER)||(core_config->crc_check==false)) {
		  	core_gesture->entry = 0;
			ipio_info("ILITEK esd %s: need reset.......\n", __func__);
		  	ilitek_platform_tp_hw_reset(true);
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/10, add ear phone function
			schedule_work(&ipd->headset_work_queue);
#endif
		  } else {
		  	ipio_debug(DEBUG_IRQ, "not need esd recover\n");
		  }
		}
        mutex_unlock(&ipd->report_mutex);

	}
	else{
		printk("ILITEK esd %s: have interrupt so not check!!!\n", __func__);
		goto ilitek_esd_check_out;
	}

	//ilitek_reset(i2c.reset_gpio);
ilitek_esd_check_out:

	atomic_set(&ilitek_cmd_response, 0);
	queue_delayed_work(esd_wq, &esd_work, delay);
	return;
}

#endif

void ilitek_platform_disable_irq(void)
{
	unsigned long nIrqFlag;

	ipio_debug(DEBUG_IRQ, "IRQ = %d\n", ipd->isEnableIRQ);

	spin_lock_irqsave(&ipd->plat_spinlock, nIrqFlag);

	if (ipd->isEnableIRQ) {
		if (ipd->isr_gpio) {
			disable_irq_nosync(ipd->isr_gpio);
			ipd->isEnableIRQ = false;
			ipio_debug(DEBUG_IRQ, "Disable IRQ: %d\n", ipd->isEnableIRQ);
		} else
			ipio_err("The number of gpio to irq is incorrect\n");
	} else
		ipio_debug(DEBUG_IRQ, "IRQ was already disabled\n");

	spin_unlock_irqrestore(&ipd->plat_spinlock, nIrqFlag);
}
EXPORT_SYMBOL(ilitek_platform_disable_irq);

void ilitek_platform_enable_irq(void)
{
	unsigned long nIrqFlag;

	ipio_debug(DEBUG_IRQ, "IRQ = %d\n", ipd->isEnableIRQ);

	spin_lock_irqsave(&ipd->plat_spinlock, nIrqFlag);

	if (!ipd->isEnableIRQ) {
		if (ipd->isr_gpio) {
			enable_irq(ipd->isr_gpio);
			ipd->isEnableIRQ = true;
			ipio_debug(DEBUG_IRQ, "Enable IRQ: %d\n", ipd->isEnableIRQ);
		} else
			ipio_err("The number of gpio to irq is incorrect\n");
	} else
		ipio_debug(DEBUG_IRQ, "IRQ was already enabled\n");

	spin_unlock_irqrestore(&ipd->plat_spinlock, nIrqFlag);
}
EXPORT_SYMBOL(ilitek_platform_enable_irq);

int ilitek_platform_tp_hw_reset(bool isEnable)
{
	int ret=0,i;
	ipio_info("HW Reset: %d\n", isEnable);
	ilitek_platform_disable_irq();
	if (isEnable) {
		for(i= 0;i< 3;i++){
#if (TP_PLATFORM == PT_MTK)
			tpd_gpio_output(ipd->reset_gpio, 1);
			mdelay(ipd->delay_time_high);
			tpd_gpio_output(ipd->reset_gpio, 0);
			mdelay(ipd->delay_time_low);
			tpd_gpio_output(ipd->reset_gpio, 1);
			mdelay(ipd->edge_delay);
#else
			gpio_direction_output(ipd->reset_gpio, 1);
			mdelay(ipd->delay_time_high);
			gpio_set_value(ipd->reset_gpio, 0);
			mdelay(ipd->delay_time_low);
			gpio_set_value(ipd->reset_gpio, 1);
			mdelay(ipd->edge_delay);
#endif
#ifdef HOST_DOWNLOAD
			ret = core_firmware_upgrade(UPDATE_FW_PATH, true);
			if(ret>= 0){
				break;
			}else{
				ipio_info("upgrade fail retry = %d\n",i);
			}
	//	core_firmware_boot_host_download();
#endif
		}
	} else {
#if (TP_PLATFORM == PT_MTK)
		tpd_gpio_output(ipd->reset_gpio, 0);
#else
		gpio_set_value(ipd->reset_gpio, 0);
#endif
	}

//	#ifdef HOST_DOWNLOAD
	//	ret = core_firmware_upgrade(UPDATE_FW_PATH, true);
	//	core_firmware_boot_host_download();
	//#endif
	ilitek_platform_enable_irq();
	return ret;
}
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
void oppo_platform_tp_hw_reset(bool isEnable)
{
	ipio_info("HW Reset: %d\n", isEnable);
	ilitek_platform_disable_irq();
	if (isEnable) {
#if (TP_PLATFORM == PT_MTK)
		tpd_gpio_output(ipd->reset_gpio, 1);
		msleep(ipd->delay_time_high);
		tpd_gpio_output(ipd->reset_gpio, 0);
		msleep(ipd->delay_time_low);
		tpd_gpio_output(ipd->reset_gpio, 1);
		msleep(ipd->edge_delay);
#else
		gpio_direction_output(ipd->reset_gpio, 1);
		mdelay(ipd->delay_time_high);
		gpio_set_value(ipd->reset_gpio, 0);
		mdelay(ipd->delay_time_low);
		gpio_set_value(ipd->reset_gpio, 1);
		mdelay(ipd->edge_delay);
#endif /* PT_MTK */
	} else {
#if (TP_PLATFORM == PT_MTK)
		tpd_gpio_output(ipd->reset_gpio, 0);
#else
		gpio_set_value(ipd->reset_gpio, 0);
#endif /* PT_MTK */
	}
	#ifdef HOST_DOWNLOAD
		core_firmware_upgrade(UPDATE_FW_PATH, true);
		//core_firmware_boot_host_download();
		msleep(100);
		core_config_get_fw_ver();
	#endif
	ilitek_platform_enable_irq();
}
EXPORT_SYMBOL(oppo_platform_tp_hw_reset);
#endif
EXPORT_SYMBOL(ilitek_platform_tp_hw_reset);
EXPORT_SYMBOL(ilitek_esd_gesture_reset);

#ifdef REGULATOR_POWER_ON
void ilitek_regulator_power_on(bool status)
{
	int res = 0;

	ipio_info("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status) {
		if (ipd->vdd) {
			res = regulator_enable(ipd->vdd);
			if (res < 0)
				ipio_err("regulator_enable vdd fail\n");
		}
		if (ipd->vdd_i2c) {
			res = regulator_enable(ipd->vdd_i2c);
			if (res < 0)
				ipio_err("regulator_enable vdd_i2c fail\n");
		}
	} else {
		if (ipd->vdd) {
			res = regulator_disable(ipd->vdd);
			if (res < 0)
				ipio_err("regulator_enable vdd fail\n");
		}
		if (ipd->vdd_i2c) {
			res = regulator_disable(ipd->vdd_i2c);
			if (res < 0)
				ipio_err("regulator_enable vdd_i2c fail\n");
		}
	}
	core_config->icemodeenable = false;
	msleep(5);
}
EXPORT_SYMBOL(ilitek_regulator_power_on);
#endif /* REGULATOR_POWER_ON */

#ifdef BATTERY_CHECK
static void read_power_status(uint8_t *buf)
{
	struct file *f = NULL;
	mm_segment_t old_fs;
	ssize_t byte = 0;

	old_fs = get_fs();
	set_fs(get_ds());

	f = filp_open(POWER_STATUS_PATH, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open %s\n", POWER_STATUS_PATH);
		return;
	}

	f->f_op->llseek(f, 0, SEEK_SET);
	byte = f->f_op->read(f, buf, 20, &f->f_pos);

	ipio_debug(DEBUG_BATTERY, "Read %d bytes\n", (int)byte);

	set_fs(old_fs);
	filp_close(f, NULL);
}

static void ilitek_platform_vpower_notify(struct work_struct *pWork)
{
	uint8_t charge_status[20] = { 0 };
	static int charge_mode = 0;

	ipio_debug(DEBUG_BATTERY, "isEnableCheckPower = %d\n", ipd->isEnablePollCheckPower);
	read_power_status(charge_status);
	ipio_debug(DEBUG_BATTERY, "Batter Status: %s\n", charge_status);

	if (strstr(charge_status, "Charging") != NULL || strstr(charge_status, "Full") != NULL
	    || strstr(charge_status, "Fully charged") != NULL) {
		if (charge_mode != 1) {
			ipio_debug(DEBUG_BATTERY, "Charging mode\n");
			core_config_plug_ctrl(false);
			charge_mode = 1;
		}
	} else {
		if (charge_mode != 2) {
			ipio_debug(DEBUG_BATTERY, "Not charging mode\n");
			core_config_plug_ctrl(true);
			charge_mode = 2;
		}
	}

	if (ipd->isEnablePollCheckPower)
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);
}
#endif

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp.Function.1372106,2018/05/18,Modify for tp MP suspend and resume
#if (TP_PLATFORM == PT_MTK)
void lcd_resume_load_ili_fw(void)
{
	ipio_info("lcd resume load ili fw begin");

	//ipio_info("lcd resume load ili fw gesture esd work cancel \n");
	//cancel_delayed_work_sync(&gesture_esd_work);

	if(tpd_load_status != 1)
		return;

	if(ipd->black_test_flag == 1) {
		ipio_info("doing black mp test\n");
		return;
	}

	if(!core_gesture->suspend)
		return;
	core_gesture->suspend = false;

	schedule_work(&ipd->resume_work_queue);

	ipio_info("lcd resume load ili fw end");
	return;
}

static void tpd_resume(struct device *h)
{
	int i;
	ipio_info("TP Resuem\n");
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/10, add ear phone function
	for (i = 0; i < 10; i++) {
		if (!core_firmware->isUpgrading) {
				break;
		}
		msleep(5);
		if(i == 9){
			ipio_info("firmware is not load success");
			return;
		}
	}
#endif
}

static void tpd_suspend(struct device *h)
{


	ipio_info("TP Suspend\n");

/*if (!core_firmware->isUpgrading) */{
	//if (!core_config->isEnableGesture)
	ilitek_platform_disable_irq();
	if (ipd->isEnablePollCheckPower)
		cancel_delayed_work_sync(&ipd->check_power_status_work);
	if (core_gesture->suspend)
		return ;
	core_gesture->suspend = true;

	#ifdef ILITEK_ESD_CHECK
		ilitek_cancel_esd_check();
	#endif

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/10/30,Add for TP black screen test
	mutex_lock(&ipd->report_mutex);
	core_config_ic_suspend();
	mutex_unlock(&ipd->report_mutex);
#endif
	core_fr_touch_all_release(core_fr);
	}
}
#elif defined CONFIG_FB

void lcd_resume_load_ili_fw(void)
{
	ipio_info("lcd resume load ili fw begin");

	//ipio_info("lcd resume load ili fw gesture esd work cancel \n");
	//cancel_delayed_work_sync(&gesture_esd_work);

	if(ipd->black_test_flag == 1) {
		ipio_info("doing black mp test\n");
		return;
	}

	if(!core_gesture->suspend)
		return;
	core_gesture->suspend = false;

	schedule_work(&ipd->resume_work_queue);

	ipio_info("lcd resume load ili fw end");
	return;
}

static int ilitek_platform_notifier_fb(struct notifier_block *self, unsigned long event, void *data)
{
	int *blank;
	struct fb_event *evdata = data;
	//int res = 0;
	int i = 0;

	ipio_info("Notifier's event = %ld\n", event);
	if(ipd->black_test_flag == 1) {
		ipio_info("doing black mp test\n");
		return 0;
	}


	/*
	 *  FB_EVENT_BLANK(0x09): A hardware display blank change occurred.
	 *  FB_EARLY_EVENT_BLANK(0x10): A hardware display blank early change occurred.
	 */
	if (evdata && evdata->data &&  event == FB_EARLY_EVENT_BLANK) {
		blank = evdata->data;

#if (TP_PLATFORM == PT_SPRD)
		if (*blank == DRM_MODE_DPMS_OFF)
#else
		if (*blank == FB_BLANK_POWERDOWN)
#endif /* PT_SPRD */
		{
			ipio_info("TP Suspend\n");

			/*if (!core_firmware->isUpgrading) */{
				//if (!core_config->isEnableGesture)
					ilitek_platform_disable_irq();

				if (ipd->isEnablePollCheckPower)
					cancel_delayed_work_sync(&ipd->check_power_status_work);
				if (core_gesture->suspend)
					return 0;
				core_gesture->suspend = true;

				#ifdef ILITEK_ESD_CHECK
					ilitek_cancel_esd_check();
				#endif
				mutex_lock(&ipd->report_mutex);

				core_config_ic_suspend();
				mutex_unlock(&ipd->report_mutex);
				//ilitek_platform_enable_irq();
			}
		}
		}
	else if(evdata && evdata->data && event == FB_EVENT_BLANK)
	{
		blank = evdata->data;
#if (TP_PLATFORM == PT_SPRD)
		else if (*blank == DRM_MODE_DPMS_ON)
#else
		if (*blank == FB_BLANK_UNBLANK )
#endif /* PT_SPRD */
		{
			ipio_info("TP Resuem\n");
			core_fr_touch_all_release(core_fr);
			for (i = 0; i < 10; i++) {
				if (!core_firmware->isUpgrading) {
					break;
				}
				msleep(5);
			}
			#if (ILITEK_TYPE == ILITEK_TYPE_LH)
			ipio_info("ilitek_platform_notifier_fb resume: led Polling start\n");

			for(i = 0; i < 20; i++) {
				if( is_tp_fw_done == 1)
					break;
				msleep(10);
			}

			msleep(50);
			ipio_info("ilitek_platform_notifier_fb resume: led on %d \n",i);
			is_tp_fw_done = 0;
			#endif
			/*if (!core_firmware->isUpgrading)*/ {

				ipio_info("TP  resume end\n");
				//core_config_ic_resume();
				//ilitek_platform_tp_hw_reset(true);
				//core_firmware_boot_host_download();
				//if (core_config->isEnableGesture == 0) {
					//res=ili_power_on(ipd,true);
					//gpio_set_value(ipd->reset_gpio,1);
				//}
				//if(res!=0)
				//{
				 // printk(" ili_power_on fail");
				//}
				//schedule_work(&ipd->resume_work_queue);
				//ilitek_platform_enable_irq();



			}
		}
	}

	return NOTIFY_OK;
}
#else /* CONFIG_HAS_EARLYSUSPEND */
static void ilitek_platform_early_suspend(struct early_suspend *h)
{
	ipio_info(" yangyang TP Suspend\n");

	/* TODO: there is doing nothing if an upgrade firmware's processing. */

	core_fr_touch_release(0, 0, 0);

	input_sync(core_fr->input_device);

	core_fr->isEnableFR = false;

	if (!core_config->isEnableGesture)
		ilitek_platform_disable_irq();

	if (ipd->isEnablePollCheckPower)
		cancel_delayed_work_sync(&ipd->check_power_status_work);
	//gpio_direction_output(ipd->reset_gpio, 1);
	//msleep(ipd->delay_time_high);
	//gpio_set_value(ipd->reset_gpio, 0);
	msleep(10);

	//core_config_ic_suspend();
}

static void ilitek_platform_late_resume(struct early_suspend *h)
{
	ipio_info("TP Resuem\n");

	core_fr->isEnableFR = true;
	//core_config_ic_resume();
	ilitek_platform_enable_irq();

	if (ipd->isEnablePollCheckPower)
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);
}
#endif /* PT_MTK */
#endif

/**
 * reg_power_check - register a thread to inquery status at certain time.
 */
static int ilitek_platform_reg_power_check(void)
{
	int res = 0;

#ifdef BATTERY_CHECK
	INIT_DELAYED_WORK(&ipd->check_power_status_work, ilitek_platform_vpower_notify);
	ipd->check_power_status_queue = create_workqueue("ili_power_check");
	ipd->work_delay = msecs_to_jiffies(CHECK_BATTERY_TIME);
	if (!ipd->check_power_status_queue) {
		ipio_err("Failed to create a work thread to check power status\n");
		ipd->vpower_reg_nb = false;
		res = -1;
	} else {
		ipio_info("Created a work thread to check power status at every %u jiffies\n",
			 (unsigned)ipd->work_delay);

		if (ipd->isEnablePollCheckPower) {
			queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work,
					   ipd->work_delay);
			ipd->vpower_reg_nb = true;
		}
	}
#endif /* BATTERY_CHECK */

	return res;
}

/**
 * Register a callback function when the event of suspend and resume occurs.
 *
 * The default used to wake up the cb function comes from notifier block mechnaism.
 * If you'd rather liek to use early suspend, CONFIG_HAS_EARLYSUSPEND in kernel config
 * must be enabled.
 */
static int ilitek_platform_reg_suspend(void)
{
	int res = 0;

#if (TP_PLATFORM == PT_MTK)
	ipio_info("It does nothing if platform is MTK\n");
#else
	ipio_info("Register suspend/resume callback function\n");
#ifdef CONFIG_FB
	ipd->notifier_fb.notifier_call = ilitek_platform_notifier_fb;
#if (TP_PLATFORM == PT_SPRD)
	res = adf_register_client(&ipd->notifier_fb);
#else
	res = fb_register_client(&ipd->notifier_fb);
#endif /* PT_SPRD */
#else
	ipd->early_suspend->suspend = ilitek_platform_early_suspend;
	ipd->early_suspend->esume = ilitek_platform_late_resume;
	ipd->early_suspend->level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	res = register_early_suspend(ipd->early_suspend);
#endif /* CONFIG_FB */
#endif /* PT_MTK */

	return res;
}

#ifndef USE_KTHREAD
static void ilitek_platform_work_queue(struct work_struct *work)
{
	ipio_debug(DEBUG_IRQ, "work_queue: IRQ = %d\n", ipd->isEnableIRQ);
    core_fr_handler();
	if (!ipd->isEnableIRQ)
		ilitek_platform_enable_irq();

	//core_fr_handler();
}
#endif /* USE_KTHREAD */

static void ilitek_resume_work_queue(struct work_struct *work)
{
	/*if (!core_config->isEnableGesture)
	{
    core_firmware_boot_host_download();
	ilitek_platform_enable_irq();
	}
	else
	{
	core_config_ic_resume();
	ilitek_platform_enable_irq();
	}*/
	gesture_done = false;
	psensor_close = false;
	core_fr->isEnableFR = false;
	mutex_lock(&ipd->report_mutex);
 if(core_config->isEnableGesture)
	{
	ilitek_platform_disable_irq();
	//core_config_ic_resume();
	core_gesture->entry =0;
	core_fr->actual_fw_mode = P5_0_FIRMWARE_DEMO_MODE;
	ilitek_platform_tp_hw_reset(true);
	//core_firmware_boot_host_download();
	ilitek_platform_enable_irq();
	}
else
	{
	core_gesture->entry =0;
	core_fr->actual_fw_mode = P5_0_FIRMWARE_DEMO_MODE;
	ilitek_platform_tp_hw_reset(true);
	//core_firmware_boot_host_download();
	ilitek_platform_enable_irq();
	}

	mutex_unlock(&ipd->report_mutex);
	schedule_work(&ipd->headset_work_queue);
	if (ipd->isEnablePollCheckPower)
			queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work,
			ipd->work_delay);
#ifdef ILITEK_ESD_CHECK
	ilitek_start_esd_check();
#endif
	core_fr->isEnableFR = true;
	ipio_info("%s\n",ili_version);

	ipio_info("TP workqueue resume end\n");


	//if (!ipd->isEnableIRQ)
	//core_fr_handler();
}
void ilitek_esd_gesture_reset(void)
{

	uint8_t temp[64] = {0};
	uint32_t reg_data = 0;
	int retry = 100;

	ipio_info("esd gesture reset begin\n");
	core_fr->isEnableFR = false;
	core_gesture->entry = false;
	core_firmware->esd_fail_enter_gesture = 1;
	ilitek_platform_tp_hw_reset(true);
	core_firmware->esd_fail_enter_gesture = 0;

	mdelay(150);
	core_config_ice_mode_enable();
	while(retry--) {
	  reg_data = core_config_ice_mode_read(0x25FF8);
	  if (reg_data == 0x5B92E7F4) {
		  ipio_info("check ok 0x25FF8 read 0x%X\n", reg_data);
		  break;
	  }
	  mdelay(10);
	}
	if (retry <= 0) {
	  ipio_info("check	error 0x25FF8 read 0x%X\n", reg_data);
	}
	core_config_ice_mode_disable();
	core_gesture->entry = true;
	host_download(true);
	temp[0] = 0x01;
	temp[1] = 0x0A;
	temp[2] = 0x06;
	if ((core_write(core_config->slave_i2c_addr, temp, 3)) < 0) {
	  ipio_info("write command error\n");
	}
	core_fr->isEnableFR = true;
	ipio_info("esd gesture reset Done\n");

}
static irqreturn_t ilitek_platform_irq_handler(int irq, void *dev_id)
{
	ipio_debug(DEBUG_IRQ, "IRQ = %d\n", ipd->isEnableIRQ);
#ifdef ILITEK_ESD_CHECK
	//atomic_set(&ilitek_cmd_response, 1);
#endif
	if (core_gesture->suspend) {
	//	wake_lock_timeout(&ilitek_gestrue_wakelock, msecs_to_jiffies(5000));
		pm_wakeup_event(&ipd->spi->dev, 5000);
	}

	if (ipd->isEnableIRQ) {
		ilitek_platform_disable_irq();
#ifdef USE_KTHREAD
		ipd->irq_trigger = true;
		ipio_debug(DEBUG_IRQ, "kthread: irq_trigger = %d\n", ipd->irq_trigger);
		wake_up_interruptible(&waiter);
#else
		schedule_work(&ipd->report_work_queue);
#endif /* USE_KTHREAD */
	}

	return IRQ_HANDLED;
}

static int ilitek_platform_input_init(void)
{
	int res = 0;

#if (TP_PLATFORM == PT_MTK)
	int i;

	ipd->input_device = tpd->dev;

	if (tpd_dts_data.use_tpd_button) {
		for (i = 0; i < tpd_dts_data.tpd_key_num; i++) {
			input_set_capability(ipd->input_device, EV_KEY, tpd_dts_data.tpd_key_local[i]);
		}
	}
	core_fr_input_set_param(ipd->input_device);
	return res;
#else
	ipd->input_device = input_allocate_device();

	if (ERR_ALLOC_MEM(ipd->input_device)) {
		ipio_err("Failed to allocate touch input device\n");
		res = -ENOMEM;
		goto fail_alloc;
	}
#if(INTERFACE == I2C_INTERFACE)
	ipd->input_device->name = ipd->client->name;
	ipd->input_device->phys = "I2C";
	ipd->input_device->dev.parent = &ipd->client->dev;
	ipd->input_device->id.bustype = BUS_I2C;
#elif (INTERFACE == SPI_INTERFACE)
	ipd->input_device->name = DEVICE_ID;
	ipd->input_device->phys = "SPI";
	ipd->input_device->dev.parent = &ipd->spi->dev;
	ipd->input_device->id.bustype = BUS_SPI;
#endif
	core_fr_input_set_param(ipd->input_device);
	/* register the input device to input sub-system */
	res = input_register_device(ipd->input_device);
	if (res < 0) {
		ipio_err("Failed to register touch input device, res = %d\n", res);
		goto out;
	}

	return res;

fail_alloc:
	input_free_device(core_fr->input_device);
	return res;

out:
	input_unregister_device(ipd->input_device);
	input_free_device(core_fr->input_device);
	return res;
#endif /* PT_MTK */
}

static int kthread_handler(void *arg)
{
	int res = 0;
	char *str = (char *)arg;

	if (strcmp(str, "boot_fw") == 0) {
		/* FW Upgrade event */
		core_firmware->isboot = true;

		ilitek_platform_disable_irq();

#ifdef BOOT_FW_UPGRADE
		res = core_firmware_boot_upgrade();
		if (res < 0)
			ipio_err("Failed to upgrade FW at boot stage\n");
#endif
		ilitek_platform_enable_irq();

		ilitek_platform_input_init();

		core_firmware->isboot = false;
	} else if (strcmp(str, "irq") == 0) {
		/* IRQ event */
		struct sched_param param = {.sched_priority = 4 };

		sched_setscheduler(current, SCHED_RR, &param);

		while (!kthread_should_stop() && !ipd->free_irq_thread) {
			ipio_debug(DEBUG_IRQ, "kthread: before->irq_trigger = %d\n", ipd->irq_trigger);
			//set_current_state(TASK_INTERRUPTIBLE);
			wait_event_interruptible(waiter, ipd->irq_trigger);
			ipd->irq_trigger = false;
			set_current_state(TASK_RUNNING);
			ipio_debug(DEBUG_IRQ, "kthread: after->irq_trigger = %d\n", ipd->irq_trigger);
			if (reload_fw_ws) {
				__pm_stay_awake(reload_fw_ws);
			}

			core_fr_handler();
			if (reload_fw_ws) {
				__pm_relax(reload_fw_ws);
			}

			ilitek_platform_enable_irq();
		}
	} else {
		ipio_err("Unknown EVENT\n");
	}

	return res;
}

static int ilitek_platform_isr_register(void)
{
	int res = 0;
#if (TP_PLATFORM == PT_MTK)
	struct device_node *node;
#endif /* PT_MTK */


#ifdef USE_KTHREAD
	ipd->irq_thread = kthread_run(kthread_handler, "irq", "ili_irq_thread");
	if (ipd->irq_thread == (struct task_struct *)ERR_PTR) {
		ipd->irq_thread = NULL;
		ipio_err("Failed to create kthread\n");
		res = -ENOMEM;
		goto out;
	}
	ipd->irq_trigger = false;
	ipd->free_irq_thread = false;
#else
	INIT_WORK(&ipd->report_work_queue, ilitek_platform_work_queue);
#endif /* USE_KTHREAD */
    INIT_WORK(&ipd->resume_work_queue, ilitek_resume_work_queue);
#if (TP_PLATFORM == PT_MTK)
	node = of_find_matching_node(NULL, touch_of_match);
	if (node) {
		ipd->isr_gpio = irq_of_parse_and_map(node, 0);
	}
#else
	ipd->isr_gpio = gpio_to_irq(ipd->int_gpio);
#endif /* PT_MTK */

	ipio_info("ipd->isr_gpio = %d\n", ipd->isr_gpio);

	res = request_threaded_irq(ipd->isr_gpio,
				   NULL,
				   ilitek_platform_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT , "ilitek", NULL);

	if (res != 0) {
		ipio_err("Failed to register irq handler, irq = %d, res = %d\n", ipd->isr_gpio, res);
		goto out;
	}

	ipd->isEnableIRQ = true;

out:
	return res;
}

static int ilitek_platform_gpio(void)
{
	int res;
	uint32_t flag;

#ifdef CONFIG_OF
#if(INTERFACE == I2C_INTERFACE)
	struct device_node *dev_node = ipd->client->dev.of_node;
#elif(INTERFACE == SPI_INTERFACE)
	struct device_node *dev_node = ipd->spi->dev.of_node;
#endif

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/1/11, add tpd_summer function
	tpd_summer_lcd_id(dev_node);
#endif
	ipd->int_gpio = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	ipd->reset_gpio = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);
	ipd->lcd_reset_gpio = of_get_named_gpio_flags(dev_node, DTS_LCD_RESET, 0, &flag);
#endif /* CONFIG_OF */

	ipio_info("GPIO INT: %d\n", ipd->int_gpio);
	ipio_info("GPIO RESET: %d\n", ipd->reset_gpio);
	ipio_info("GPIO LCD RESET: %d\n", ipd->lcd_reset_gpio);

	if (!gpio_is_valid(ipd->int_gpio)) {
		ipio_err("Invalid INT gpio: %d\n", ipd->int_gpio);
		return -EBADR;
	}

	if (!gpio_is_valid(ipd->reset_gpio)) {
		ipio_err("Invalid RESET gpio: %d\n", ipd->reset_gpio);
		return -EBADR;
	}
#ifdef HUZHONGHUA
	if (!gpio_is_valid(ipd->lcd_reset_gpio)) {
		ipio_err("Invalid lcd gpio: %d\n", ipd->lcd_reset_gpio);
		return -EBADR;
	}
#endif

	res = gpio_request(ipd->reset_gpio, "ILITEK_TP_RESET");
	if (res < 0) {
		ipio_err("Request RESET GPIO failed, res = %d\n", res);
		goto out;
	}

	res = gpio_request(ipd->int_gpio, "ILITEK_TP_IRQ");
	if (res < 0) {
		ipio_err("Request IRQ GPIO failed, res = %d\n", res);
		goto out;
	}

	gpio_direction_input(ipd->int_gpio);

out:
	return res;
}

/*static int ilitek_platform_read_tp_info(void)
{
	if (core_config_get_chip_id() < 0)
		return CHIP_ID_ERR;
	if (core_config_get_protocol_ver() < 0)
		return -1;
	if (core_config_get_fw_ver() < 0)
		return -1;
	if (core_config_get_core_ver() < 0)
		return -1;
	if (core_config_get_tp_info() < 0)
		return -1;
	if (core_config_get_key_info() < 0)
		return -1;

	return 0;
}*/

/**
 * Remove Core APIs memeory being allocated.
 */
static void ilitek_platform_core_remove(void)
{
	ipio_info("Remove all core's compoenets\n");
	ilitek_proc_remove();
	core_flash_remove();
	core_firmware_remove();
	core_fr_remove();
	core_config_remove();
	core_i2c_remove();
	core_protocol_remove();
	core_gesture_remove();
}

/**
 * The function is to initialise all necessary structurs in those core APIs,
 * they must be called before the i2c dev probes up successfully.
 */
static int ilitek_platform_core_init(void)
{
	ipio_info("Initialise core's components\n");

	if (core_config_init() < 0 || core_protocol_init() < 0 ||
		core_firmware_init() < 0 || core_fr_init() < 0) {
		ipio_err("Failed to initialise core components\n");
		return -EINVAL;
	}
	core_gesture = kmalloc(sizeof(*core_gesture), GFP_KERNEL);
		if (ERR_ALLOC_MEM(core_gesture)) {
		ipio_err("Failed to alllocate core_i2c mem %ld\n", PTR_ERR(core_gesture));
		core_gesture_remove();
	}
	core_gesture->entry = false;
	core_gesture->suspend = false;

	#if (INTERFACE == I2C_INTERFACE)
	if(core_i2c_init(ipd->client) < 0)
	#elif (INTERFACE == SPI_INTERFACE)
	if(core_spi_init(ipd->spi) < 0)
	#endif
	{
		ipio_err("Failed to initialise core components\n");
		return -EINVAL;
	}
	return 0;
}
#if (INTERFACE == I2C_INTERFACE)
static int ilitek_platform_remove(struct i2c_client *client)
#elif (INTERFACE == SPI_INTERFACE)
static int ilitek_platform_remove(struct spi_device *spi)
#endif
{
	ipio_info("Remove platform components\n");

	if (ipd->isEnableIRQ) {
		disable_irq_nosync(ipd->isr_gpio);
	}

	if (ipd->isr_gpio != 0 && ipd->int_gpio != 0 && ipd->reset_gpio != 0) {
		free_irq(ipd->isr_gpio, (void *)ipd->i2c_id);
		gpio_free(ipd->int_gpio);
		gpio_free(ipd->reset_gpio);
	}
#ifdef CONFIG_FB
	fb_unregister_client(&ipd->notifier_fb);
#else
	unregister_early_suspend(&ipd->early_suspend);
#endif /* CONFIG_FB */

#ifdef USE_KTHREAD
	if (ipd->irq_thread != NULL) {
		ipd->irq_trigger = true;
		ipd->free_irq_thread = true;
		wake_up_interruptible(&waiter);
		kthread_stop(ipd->irq_thread);
		ipd->irq_thread = NULL;
	}
#endif /* USE_KTHREAD */

	if (ipd->input_device != NULL) {
		input_unregister_device(ipd->input_device);
		input_free_device(ipd->input_device);
	}

	if (ipd->vpower_reg_nb) {
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		destroy_workqueue(ipd->check_power_status_queue);
	}

	ipio_kfree((void **)&ipd);
	ilitek_platform_core_remove();

	return 0;
}
/*
int  ili_power_on(struct ilitek_platform_data *ts, bool on)
{
    int rc = 0;
    if (!on) {
       printk("ctp power_off\n");
       goto power_off;
	}
	printk("ctp power_on start\n");
    rc = regulator_enable(ts->gpio_pwr);
    if ( rc != 0 ) {
        dev_err(&ts->spi->dev,
                "%s Regulator ts->gpio_pwr enable failed rc=%d\n", __func__,rc);
        goto gpio_pwr_err;
    }
	rc = regulator_enable(ts->lab_pwr);
    if ( rc != 0 ) {
        dev_err(&ts->spi->dev,
                "%s Regulator ts->lab_pwr enable failed rc=%d\n", __func__,rc);
        goto lab_pwr_err;
    }
    rc = regulator_enable(ts->ibb_pwr);
    if ( rc != 0 ) {
        dev_err(&ts->spi->dev,
                "%s Regulator ts->ibb_pwr enable failed rc=%d\n", __func__,rc);
        goto ibb_pwr_err;
    }
	printk("ctp power_on end\n");

    return rc;

power_off:
ibb_pwr_err:
	if (ts->ibb_pwr) {
		regulator_disable(ts->ibb_pwr);
	}
lab_pwr_err:
	if (ts->lab_pwr) {
		regulator_disable(ts->lab_pwr);
	}
gpio_pwr_err:
	if (ts->gpio_pwr) {
		regulator_disable(ts->gpio_pwr);
	}

    return rc;
}

static int ili_power_init(struct ilitek_platform_data *ts, bool on)
{
    int rc = 0;
    if ( !on ) {
		printk("power_init is deny\n");
        goto pwr_deny;
	}
    ts->gpio_pwr = regulator_get(&ts->spi->dev, "vdd");
	if ( IS_ERR(ts->gpio_pwr) ) {
        rc = PTR_ERR(ts->gpio_pwr);
        dev_err(&ts->spi->dev,
                "%s Regulator get failed ts->gpio_pwr rc=%d\n", __func__,rc);

		goto gpio_pwr_err;
    }
	ts->lab_pwr = regulator_get(&ts->spi->dev, "lab");
	if ( IS_ERR(ts->lab_pwr) ) {
        rc = PTR_ERR(ts->lab_pwr);
        dev_err(&ts->spi->dev,
                "%s Regulator get failed ts->lab_pwr rc=%d\n", __func__,rc);

		goto lab_pwr_err;
    }
	ts->ibb_pwr = regulator_get(&ts->spi->dev, "ibb");
	if ( IS_ERR(ts->ibb_pwr) ) {
        rc = PTR_ERR(ts->ibb_pwr);
       dev_err(&ts->spi->dev,
                "%s Regulator get failed ts->ibb_pwr rc=%d\n", __func__,rc);

		goto ibb_pwr_err;
    }

    return rc;

pwr_deny:
ibb_pwr_err:
	if (ts->ibb_pwr) {
		regulator_put(ts->ibb_pwr);
		ts->ibb_pwr = NULL;
	}
lab_pwr_err:
	if (ts->lab_pwr) {
		regulator_put(ts->lab_pwr);
		ts->lab_pwr = NULL;
	}
gpio_pwr_err:
	if (ts->gpio_pwr) {
		regulator_put(ts->gpio_pwr);
		ts->gpio_pwr = NULL;
	}

    return rc;

}*/

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp,2018/10/11,Add touch-info file function
/**
 * touch information implementation
 */
static int __maybe_unused tid_reset(struct device *dev)
{
	return 0;
}

static int __maybe_unused tid_get_version(struct device *dev, int *major, int *minor)
{
	*major = core_config->firmware_ver[2];
	*minor = core_config->firmware_ver[3];

	return 0;
}

static int __maybe_unused tid_firmware_upgrade(struct device *dev, const struct firmware *fw,
				bool force)
{
	return 0;
}

static int __maybe_unused tid_open_short_test(struct device *dev, struct seq_file *seq,
			       const struct firmware *fw)
{
	return true;
}

static int __maybe_unused tid_get_lockdown_info(struct device *dev, char *out_values)
{
#ifdef ODM_WT_EDIT
	//Bin.Su@ODM_WT.BSP.Tp.Init.2018/10/15,Add for compatible HLT and TXD module
	printk("ipd->vendor_id = %d\n",ipd->vendor_id);
	switch (ipd->vendor_id) {
		case 0 :
			out_values[LOCKDOWN_INFO_PANEL_MAKER_INDEX] = 0x52;
			printk("this is TXD touchscreen\n");
			break;
		case 1 :
			out_values[LOCKDOWN_INFO_PANEL_MAKER_INDEX] = 0x53;
			printk("this is HLT touchscreen\n");
			break;
		case 2 :
			out_values[LOCKDOWN_INFO_PANEL_MAKER_INDEX] = 0x56;
			printk("this is TXD_TF touchscreen\n");
			break;
		case 3 :
			out_values[LOCKDOWN_INFO_PANEL_MAKER_INDEX] = 0x57;
			printk("this is LEAD_TF touchscreen\n");
			break;
		case 4 :
			out_values[LOCKDOWN_INFO_PANEL_MAKER_INDEX] = 0x58;
			printk("this is INX touchscreen\n");
			break;
		default :
			printk("this is No match touch lockdown\n");
			return -1;
	}
#endif
	return 0;
}

static int __maybe_unused tid_gesture_set_capability(struct device *dev, bool enable)
{
	return 0;
}

static int __maybe_unused tid_resume(struct device *dev)
{
	struct ilitek_platform_data *ipd = dev_get_drvdata(dev);

	queue_work(system_unbound_wq, &ipd->resume_work);

	return 0;
}

static int __maybe_unused tid_suspend_early(struct device *dev)
{
	tpd_suspend(dev);

	return 0;
}

static inline struct ilitek_platform_data  __maybe_unused *wokr_to_ipd(struct work_struct *work)
{
	return container_of(work, struct ilitek_platform_data, resume_work);
}

static void __maybe_unused tid_resume_work(struct work_struct *work)
{
	struct ilitek_platform_data *ipd = wokr_to_ipd(work);

	tpd_resume(&ipd->spi->dev);
}
#endif

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/10, add ear phone function
/**
 * add aer phone function
 */
static void ipd_headset_work_queue(struct work_struct *work)
 {
	ipio_info("ipd_headset_work_queue +++++headset_state = %ld\n",ipd->headset_state);
	if ((core_gesture->suspend)||(core_firmware->isUpgrading)){
		ipio_info("suspend = %d,isUpgrading = %d,don't need headset function",core_gesture->suspend,core_firmware->isUpgrading);
		return;
	}
	mutex_lock(&ipd->report_mutex);
	if(ipd->headset_state == 1) {
		core_config_ear_phone_ctrl(true);
	}
	else if(ipd->headset_state == 0) {
			core_config_ear_phone_ctrl(false);
	}
		else
			ipio_info("invild headset statues value");
	mutex_unlock(&ipd->report_mutex);
 }

static int ipd_notifie_headset(struct notifier_block *nb,unsigned long value, void *data)
{
	ipd->headset_state = value;
	ipio_info("value == %ld,ipd->headset_state = %ld data = %p",value,ipd->headset_state,data);
	schedule_work(&ipd->headset_work_queue);
	return 0;
}

static void ipd_headset_notifier_init(void)
{
	int res = 0;
	ipio_info("ipd_headset_notifier_init\n");
	ipd->notifier_headset.notifier_call = ipd_notifie_headset;
	res = headset_register_client(&ipd->notifier_headset);
}
#endif

/**
 * The probe func would be called after an i2c device was detected by kernel.
 *
 * It will still return zero even if it couldn't get a touch ic info.
 * The reason for why we allow it passing the process is because users/developers
 * might want to have access to ICE mode to upgrade a firwmare forcelly.
 */
#if (INTERFACE == I2C_INTERFACE)
static int ilitek_platform_probe(struct i2c_client *client, const struct i2c_device_id *id)
#elif(INTERFACE == SPI_INTERFACE)
static int ilitek_platform_probe(struct spi_device *spi)
#endif
{
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp,2018/10/11,Add touch-info file function
	int ret ;
    struct touch_info_dev *tid;
	struct touch_info_dev_operations *tid_ops;
#endif
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/21,Add for compatible HLT and TXD module
	char *temp = NULL;
	char *cmdline_tp = NULL;
	int tf_value = 0;
#endif
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp,2018/10/11,Add touch-info file function
#if (INTERFACE == I2C_INTERFACE)
	struct device *dev = &client->dev;
#else
	struct device *dev = &spi->dev;
#endif
	ipio_info("Probe Enter\n");
	/* initialise the struct of touch ic memebers. */
	ipd = devm_kzalloc(dev, sizeof(*ipd), GFP_KERNEL);
	if (!ipd) {
		ipio_err("Failed to allocate ipd memory, %ld\n", PTR_ERR(ipd));
		return -ENOMEM;
	}
#endif

#ifdef ODM_WT_EDIT
	//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/21,Add for compatible HLT and TXD module
	//Bin.Su@ODM_WT.BSP.Tp.Init.2018/10/15,Add for compatible HLT and TXD module
		cmdline_tp = strstr(saved_command_line,"ilt9881h_");
		printk("%s cmdline_tp = %s\n",__func__,cmdline_tp);
		if(cmdline_tp == NULL){
			cmdline_tp = strstr(saved_command_line,"ili9881tfh_");
			if(cmdline_tp != NULL){
				temp = cmdline_tp + strlen("ili9881tfh_");
				tf_value = 1;
			}else{
				printk("%s get 9881h IC fail ",__func__);
				return -1;
			}
		}else{
			temp = cmdline_tp + strlen("ilt9881h_");
		}
		printk("%s temp = %s\n",__func__,temp);
		ili_ctpmodule = strncmp(temp,"txd_hdp_dsi_vdo_lcm_drv",strlen("txd_hdp_dsi_vdo_lcm_drv"));
		if ( ili_ctpmodule == 0 ) {
			printk("%s this is TXD touchscreen,ili_ctpmodule = %d\n",__func__,ili_ctpmodule);
			if(tf_value == 1){
				ili_ctpmodule = 2;
				printk("%s this is TXD TF touchscreen,ili_ctpmodule = %d\n",__func__,ili_ctpmodule);
			}
		}
		else{
			if(strncmp(temp,"hlt_hdp_dsi_vdo_lcm_drv",strlen("hlt_hdp_dsi_vdo_lcm_drv")) == 0) {
				ili_ctpmodule = 1;
				printk("%s this is HLT touchscreen,ili_ctpmodule = %d\n",__func__,ili_ctpmodule);
			}else{
				if(strncmp(temp,"lide_hdp_dsi_vdo_lcm_drv",strlen("lide_hdp_dsi_vdo_lcm_drv")) == 0) {
					ili_ctpmodule = 3;
					printk("%s this is LEAD touchscreen,ili_ctpmodule = %d\n",__func__,ili_ctpmodule);
				}else{
					if(strncmp(temp,"inx_hdp_dsi_vdo_lcm_drv",strlen("inx_hdp_dsi_vdo_lcm_drv")) == 0){
						ili_ctpmodule = 4;
						printk("%s this is INX touchscreen,ili_ctpmodule = %d\n",__func__,ili_ctpmodule);
					}else{
						printk("%s this is another ILI touchscreen,exit probe\n",__func__);
						return -1;
					}
				}
			}
		}
#endif

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp,2018/10/11,Add touch-info file function
	dev_set_drvdata(dev, ipd);
#endif

#if (INTERFACE == I2C_INTERFACE)
#ifdef REGULATOR_POWER_ON
#if (TP_PLATFORM == PT_MTK)
	const char *vdd_name = "vtouch";
#else
	const char *vdd_name = "vdd";
#endif /* PT_MTK */
	const char *vcc_i2c_name = "vcc_i2c";
#endif /* REGULATOR_POWER_ON */

	if (client == NULL) {
		ipio_err("i2c client is NULL\n");
		return -ENODEV;
	}

	/* Set i2c slave addr if it's not configured */
	ipio_info("I2C Slave address = 0x%x\n", client->addr);
	if (client->addr != ILI7807_SLAVE_ADDR || client->addr != ILI9881_SLAVE_ADDR) {
		client->addr = ILI9881_SLAVE_ADDR;
		ipio_err("I2C Slave addr doesn't be set up, use default : 0x%x\n", client->addr);
	}
	ipd->client = client;
	ipd->i2c_id = id;
#elif(INTERFACE == SPI_INTERFACE)
	if(!spi)
	{
		return -ENOMEM;
	}

	ipd->spi = spi;
	ipd->boot_mode = get_boot_mode();
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/10, KERNEL_POWER_OFF_CHARGING_BOOT no need TP
	if(ipd->boot_mode == 8){
		ipio_info("boot_mode is KERNEL_POWER_OFF_CHARGING_BOOT , not need to enable tp\n");
		return -1;
	}
#endif
	ipio_info("==========ipd->boot_mode = %d\n",ipd->boot_mode);
	//if ((ts->boot_mode == MSM_BOOT_MODE__FACTORY || ts->boot_mode == MSM_BOOT_MODE__RF || ts->boot_mode == MSM_BOOT_MODE__WLAN))
	if ((ipd->boot_mode == 3 || ipd->boot_mode == 4 || ipd->boot_mode == 5)) {
		ipio_info("boot_mode is FACTORY,RF and WLAN not need to enable irq\n");
		//return -1;
	}
#endif

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp,2018/10/11,Add touch-info file function
	tid = devm_tid_and_ops_allocate(dev);
	if (unlikely(!tid))
		return -ENOMEM;
	tid_ops = tid->tid_ops;
	ipd->tid = tid;

	tid_ops->reset			   = tid_reset;
	tid_ops->get_version		   = tid_get_version;
	tid_ops->get_lockdown_info 	   = tid_get_lockdown_info;

	//Bin.Su@ODM_WT.BSP.Tp.Function.1372106,2018/05/18,Modify for tp MP test
	//tid_ops->firmware_upgrade	   = tid_firmware_upgrade;
	tid_ops->open_short_test	   = tid_open_short_test;
	tid_ops->gesture_set_capability	   = tid_gesture_set_capability;

	ipd->vendor_id = ili_ctpmodule;
	ipio_info("vendor_id is %d",ipd->vendor_id);
	ili_fw = &ili_fw_list[ipd->vendor_id];
	ipio_info("ili_fw->id = %d,ili_fw->module_vendor =%s,ili_fw->fw_len =%d, \
		ili_fw->firmware_mp_name = %s,ili_fw->mp_ini_name = %s\n", \
		ili_fw->id,ili_fw->module_vendor, ili_fw->fw_len, \
		ili_fw->firmware_bin_name,ili_fw->mp_ini_name);

	CTPM_FW = ili_fw->firmware_i;
#endif

	ipd->chip_id = TP_TOUCH_IC;
	ipd->isEnableIRQ = false;
	ipd->isEnablePollCheckPower = false;
	ipd->vpower_reg_nb = false;
	ipd->black_test_flag = 0;
	ilitek_tp = 1;
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/17, add presure report function
	ipd->presure_speed = 60;
	ipio_info("report presure speed :  %d\n", ipd->presure_speed);
#endif
	ipio_info("Driver Version : %s\n", DRIVER_VERSION);
	ipio_info("Driver for Touch IC :  %x\n", TP_TOUCH_IC);
	ipio_info("Driver on platform :  %x\n", TP_PLATFORM);

/*
	ret = ili_power_init(ipd, true);
	if (ret) {
		printk("ili power init fail\n");
	}
	ret = ili_power_on(ipd, true);
	if (ret) {
		printk("ili power on fail\n");
	}
*/
	/*
	 * Different ICs may require different delay time for the reset.
	 * They may also depend on what your platform need to.
	 */
	if (ipd->chip_id == CHIP_TYPE_ILI7807) {
		ipd->delay_time_high = 10;
		ipd->delay_time_low = 5;
		ipd->edge_delay = 200;
	} else if (ipd->chip_id == CHIP_TYPE_ILI9881) {
		ipd->delay_time_high = 10;
		ipd->delay_time_low = 5;
#if(INTERFACE == I2C_INTERFACE)
		ipd->edge_delay = 100;
#elif (INTERFACE == SPI_INTERFACE)
		ipd->edge_delay = 3;
#endif
		ipio_info("\n");
	} else {
		ipd->delay_time_high = 10;
		ipd->delay_time_low = 10;
		ipd->edge_delay = 10;
		ipio_info("\n");
	}

	mutex_init(&ipd->plat_mutex);
    mutex_init(&ipd->report_mutex);
	mutex_init(&ipd->gesture_mutex);
	spin_lock_init(&ipd->plat_spinlock);

	/* Init members for debug */
	mutex_init(&ipd->ilitek_debug_mutex);
	mutex_init(&ipd->ilitek_debug_read_mutex);
	init_waitqueue_head(&(ipd->inq));
	ipd->debug_data_frame = 0;
	ipd->debug_node_open = false;
#ifdef MING_TEST
mutex_init(&ipd->ilitek_delta_mutex);
mutex_init(&ipd->ilitek_delta_read_mutex);
init_waitqueue_head(&(ipd->delta_inq));

init_waitqueue_head(&(ipd->otp_inq));
ipd->delta_node_open = false;

#endif
mutex_init(&ipd->ilitek_delta_mutex);
reload_fw_ws = wakeup_source_register("vooc_wake_lock");
#ifdef REGULATOR_POWER_ON
#if (TP_PLATFORM == PT_MTK)
	ipd->vdd = regulator_get(tpd->tpd_dev, vdd_name);
	tpd->reg = ipd->vdd;
#else
	ipd->vdd = regulator_get(&ipd->client->dev, vdd_name);
#endif /* PT_MTK */
	if (ERR_ALLOC_MEM(ipd->vdd)) {
		ipio_err("regulator_get vdd fail\n");
		ipd->vdd = NULL;
	} else {
		if (regulator_set_voltage(ipd->vdd, VDD_VOLTAGE, VDD_VOLTAGE) < 0)
			ipio_err("Failed to set vdd %d.\n", VDD_VOLTAGE);
	}

	ipd->vdd_i2c = regulator_get(&ipd->client->dev, vcc_i2c_name);
	if (ERR_ALLOC_MEM(ipd->vdd_i2c)) {
		ipio_err("regulator_get vdd_i2c fail.\n");
		ipd->vdd_i2c = NULL;
	} else {
		if (regulator_set_voltage(ipd->vdd_i2c, VDD_I2C_VOLTAGE, VDD_I2C_VOLTAGE) < 0)
			ipio_err("Failed to set vdd_i2c %d\n", VDD_I2C_VOLTAGE);
	}
	ilitek_regulator_power_on(true);
#endif /* REGULATOR_POWER_ON */

	if (ilitek_platform_gpio() < 0)
		ipio_err("Failed to request gpios\n ");

	/* If kernel failes to allocate memory to the core components, driver will be unloaded. */
	if (ilitek_platform_core_init() < 0) {
		ipio_err("Failed to allocate cores' mem\n");
		return -ENOMEM;
	}
	// if (core_config_get_chip_id() < 0)
	// 	return CHIP_ID_ERR;


#ifdef HOST_DOWNLOAD
	core_firmware_boot_host_download();
#else
	ilitek_platform_tp_hw_reset(true);
#endif
	/* get our tp ic information */
	// ret = ilitek_platform_read_tp_info();
	// if (ret == CHIP_ID_ERR) {
	// 	ipio_err("CHIP ID is incorrect, need to rebuild driver\n");
	// 	return -ENODEV;
	// } else if (ret < 0) {
	// 	ipio_err("Failed to get TP info, need to upgrade a correct FW\n");
	// }

	/* If it defines boot upgrade, input register will be done inside boot function. */
#ifndef BOOT_FW_UPGRADE
	if (ilitek_platform_input_init() < 0)
		ipio_err("Failed to init input device in kernel\n");
#endif /* BOOT_FW_UPGRADE */

	//wake_lock_init(&ilitek_gestrue_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
#if(INTERFACE == I2C_INTERFACE)
	device_init_wakeup(&ipd->client->dev, true);
#elif (INTERFACE == SPI_INTERFACE)
        device_init_wakeup(&ipd->spi->dev, true);
#endif
	/*
	 * To make sure our ic running well before the work,
	 * pulling RESET pin as low/high once after read TP info.
	 */
	//ilitek_platform_tp_hw_reset(true);

	if (ilitek_platform_reg_suspend() < 0)
		ipio_err("Failed to register suspend/resume function\n");

	if (ilitek_platform_reg_power_check() < 0)
		ipio_err("Failed to register power check function\n");

	/* Create nodes for users */
	ilitek_proc_init();
	msleep(100);
	core_config_get_fw_ver();
	core_config_get_chip_id();
	core_config_get_protocol_ver();
	core_config_get_tp_info();

#if 1
	if (ilitek_platform_isr_register() < 0)
	{
		ipio_err("Failed to register ISR\n");
	}
#endif

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp,2018/10/11,Add touch-info file function
	INIT_WORK(&ipd->resume_work, tid_resume_work);

	ret = devm_tid_register(dev, tid);
	if (unlikely(ret))
		return ret;
#endif

	if ((ipd->boot_mode == 3 || ipd->boot_mode == 4 || ipd->boot_mode == 5)) {
		ipio_info("boot_mode is FACTORY,RF and WLAN not need to enable irq\n");
		disable_irq_nosync(ipd->isr_gpio);
	}
#ifdef ILITEK_ESD_CHECK
			 INIT_DELAYED_WORK(&esd_work, ilitek_touch_esd_func);
			 esd_wq = create_singlethread_workqueue("esd_wq");
			 if (!esd_wq) {
				 return -ENOMEM;
			 }
			 queue_delayed_work(esd_wq, &esd_work, delay);
#endif

#if (TP_PLATFORM == PT_MTK)
	tpd_load_status = 1;
#endif /* PT_MTK */
SysHaveBoot =true;
ipio_debug_level =0;


#ifdef BOOT_FW_UPGRADE
	ipd->update_thread = kthread_run(kthread_handler, "boot_fw", "ili_fw_boot");
	if (ipd->update_thread == (struct task_struct *)ERR_PTR) {
		ipd->update_thread = NULL;
		ipio_err("Failed to create fw upgrade thread\n");
	}
#endif /* BOOT_FW_UPGRADE */

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/10, add ear phone function
	INIT_WORK(&ipd->headset_work_queue, ipd_headset_work_queue);
	ipd_headset_notifier_init();
#endif
	return 0;
}

static const struct i2c_device_id tp_device_id[] = {
	{DEVICE_ID, 0},
	{},			/* should not omitted */
};

MODULE_DEVICE_TABLE(i2c, tp_device_id);

/*
 * The name in the table must match the definiation
 * in a dts file.
 *
 */
static struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

#if (TP_PLATFORM == PT_MTK)
#if (INTERFACE == I2C_INTERFACE)
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	ipio_info("TPD detect i2c device\n");
	strcpy(info->type, TPD_DEVICE);
	return 0;
}
#endif /*I2C_INTERFACE*/
#endif /* PT_MTK */
#if (INTERFACE == I2C_INTERFACE)
static struct i2c_driver tp_i2c_driver = {
	.driver = {
		   .name = DEVICE_ID,
		   .owner = THIS_MODULE,
		   .of_match_table = tp_match_table,
		   },
	.probe = ilitek_platform_probe,
	.remove = ilitek_platform_remove,
	.id_table = tp_device_id,
#if (TP_PLATFORM == PT_MTK)
	.detect = tpd_detect,
#endif /* PT_MTK */
};
#elif (INTERFACE == SPI_INTERFACE)
static struct spi_driver tp_spi_driver = {
	.driver = {
		.name	= DEVICE_ID,
		.owner = THIS_MODULE,
		.of_match_table = tp_match_table,
	},
	.probe = ilitek_platform_probe,
	.remove = ilitek_platform_remove,
};
#endif
#if (TP_PLATFORM == PT_MTK)
static int tpd_local_init(void)
{
	ipio_info("TPD init device driver\n");

#if (INTERFACE == I2C_INTERFACE)
	if (i2c_add_driver(&tp_i2c_driver) != 0) {
		ipio_err("Unable to add i2c driver\n");
		return -1;
	}
	if (tpd_load_status == 0) {
		ipio_err("Add error touch panel driver\n");

		i2c_del_driver(&tp_i2c_driver);
		return -1;
	}
#else
	if (spi_register_driver(&tp_spi_driver) != 0) {
		ipio_err("Unable to add spi driver\n");
		return -1;
	}
	if (tpd_load_status == 0) {
		ipio_err("Add error touch panel driver\n");

		spi_unregister_driver(&tp_spi_driver);
		return -1;
	}
#endif
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num,
				   tpd_dts_data.tpd_key_local,
				   tpd_dts_data.tpd_key_dim_local);
	}

	tpd_type_cap = 1;

	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = DEVICE_ID,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
};
#endif /* PT_MTK */

static int __init ilitek_platform_init(void)
{
	int res = 0;

	ipio_info("TP driver init\n");

#if (TP_PLATFORM == PT_MTK)
	tpd_get_dts_info();
	res = tpd_driver_add(&tpd_device_driver);
	if (res < 0) {
		ipio_err("TPD add TP driver failed\n");
		tpd_driver_remove(&tpd_device_driver);
		return -ENODEV;
	}
#elif (INTERFACE == I2C_INTERFACE)
	ipio_info("TP driver add i2c interface\n");
	res = i2c_add_driver(&tp_i2c_driver);
	if (res < 0) {
		ipio_err("Failed to add i2c driver\n");
		i2c_del_driver(&tp_i2c_driver);
		return -ENODEV;
	}
#elif(INTERFACE == SPI_INTERFACE)
	ipio_info("TP driver add spi interface\n");
	res = spi_register_driver(&tp_spi_driver);
	if (res < 0) {
		ipio_err("Failed to add i2c driver\n");
		spi_unregister_driver(&tp_spi_driver);
		return -ENODEV;
	}
#endif /* PT_MTK */

	ipio_info("Succeed to add i2c driver\n");
	return res;
}

static void __exit ilitek_platform_exit(void)
{
	ipio_info("I2C driver has been removed\n");

#if (TP_PLATFORM == PT_MTK)
	tpd_driver_remove(&tpd_device_driver);
#else
#if(INTERFACE == I2C_INTERFACE)
	i2c_del_driver(&tp_i2c_driver);
#elif(INTERFACE == SPI_INTERFACE)
	spi_unregister_driver(&tp_spi_driver);
#endif
#endif
}

module_init(ilitek_platform_init);
module_exit(ilitek_platform_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
