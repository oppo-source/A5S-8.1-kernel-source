/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"
/* device tree should be defined in flashlight-dt.h */
#ifndef OPPO_DTNAME
#define OPPO_DTNAME "mediatek,flashlights_oppo"
#endif
#ifndef OPPO_DTNAME_I2C
#define OPPO_DTNAME_I2C   "mediatek,strobe_main"
#endif

#define OPPO_NAME "flashlights-oppo"

/* define registers */
#define OPPO_REG_ENABLE           (0x01)
#define OPPO_MASK_ENABLE_LED1     (0x01)
#define OPPO_MASK_ENABLE_LED2     (0x02)
#define OPPO_DISABLE              (0x00)
#define OPPO_TORCH_MODE           (0x08)
#define OPPO_FLASH_MODE           (0x0C)
#define OPPO_ENABLE_LED1          (0x01)
#define OPPO_ENABLE_LED1_TORCH    (0x09)
#define OPPO_ENABLE_LED1_FLASH    (0x0D)
#define OPPO_ENABLE_LED2          (0x02)
#define OPPO_ENABLE_LED2_TORCH    (0x0A)
#define OPPO_ENABLE_LED2_FLASH    (0x0E)

#define OPPO_REG_TORCH_LEVEL_LED1 (0x05)
#define OPPO_REG_FLASH_LEVEL_LED1 (0x03)
#define OPPO_REG_TORCH_LEVEL_LED2 (0x06)
#define OPPO_REG_FLASH_LEVEL_LED2 (0x04)

#define OPPO_REG_TIMING_CONF      (0x08)
#define OPPO_TORCH_RAMP_TIME      (0x10)
#define OPPO_FLASH_TIMEOUT        (0x0F)



/* define channel, level */
#define OPPO_CHANNEL_NUM          2
#define OPPO_CHANNEL_CH1          0
#define OPPO_CHANNEL_CH2          1
/* define level */
#define OPPO_LEVEL_NUM 26
#define OPPO_LEVEL_TORCH 7

#define OPPO_HW_TIMEOUT 400 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(oppo_mutex);
static struct work_struct oppo_work_ch1;
static struct work_struct oppo_work_ch2;

/* define pinctrl */
#define OPPO_PINCTRL_PIN_HWEN 0
#define OPPO_PINCTRL_PINSTATE_LOW 0
#define OPPO_PINCTRL_PINSTATE_HIGH 1
#define OPPO_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define OPPO_PINCTRL_STATE_HWEN_LOW  "hwen_low"

extern struct flashlight_operations sy7806_ops;
extern struct i2c_client *sy7806_i2c_client;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t oppo_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t oppo_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t oppo_set_hwen(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t oppo_get_hwen(struct device* cd, struct device_attribute *attr, char* buf);

static DEVICE_ATTR(reg, 0660, oppo_get_reg,  oppo_set_reg);
static DEVICE_ATTR(hwen, 0660, oppo_get_hwen,  oppo_set_hwen);

struct i2c_client *oppo_flashlight_client;

static struct pinctrl *oppo_pinctrl;
static struct pinctrl_state *oppo_hwen_high;
static struct pinctrl_state *oppo_hwen_low;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *oppo_i2c_client;

/* platform data */
struct oppo_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* oppo chip data */
struct oppo_chip_data {
	struct i2c_client *client;
	struct oppo_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int oppo_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	oppo_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(oppo_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(oppo_pinctrl);
	}

	/* Flashlight HWEN pin initialization */
	oppo_hwen_high = pinctrl_lookup_state(oppo_pinctrl, OPPO_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(oppo_hwen_high)) {
		pr_err("Failed to init (%s)\n", OPPO_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(oppo_hwen_high);
	}
	oppo_hwen_low = pinctrl_lookup_state(oppo_pinctrl, OPPO_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(oppo_hwen_low)) {
		pr_err("Failed to init (%s)\n", OPPO_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(oppo_hwen_low);
	}

	return ret;
}

static int oppo_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(oppo_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case OPPO_PINCTRL_PIN_HWEN:
		if (state == OPPO_PINCTRL_PINSTATE_LOW && !IS_ERR(oppo_hwen_low))
			pinctrl_select_state(oppo_pinctrl, oppo_hwen_low);
		else if (state == OPPO_PINCTRL_PINSTATE_HIGH && !IS_ERR(oppo_hwen_high))
			pinctrl_select_state(oppo_pinctrl, oppo_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_info("pin(%d) state(%d)\n", pin, state);

	return ret;
}

/******************************************************************************
 * oppo operations
 *****************************************************************************/
static const int *oppo_current;
static const unsigned char *oppo_torch_level;
static const unsigned char *oppo_flash_level;


static const int aw3643_current[OPPO_LEVEL_NUM] = {
	22,  46,  70,  93,  116, 140, 163, 198, 245, 304,
	351, 398, 445, 503,  550, 597, 656, 703, 750, 796,
	855, 902, 949, 996, 1054, 1101
};

static const unsigned char aw3643_torch_level[OPPO_LEVEL_NUM] = {
    0x06, 0x0F, 0x17, 0x1F, 0x27, 0x2F, 0x37, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char aw3643_flash_level[OPPO_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D};


static const int sy7806_current[OPPO_LEVEL_NUM] = {
	24,  46,	70,  93,  116, 140, 164, 199, 246, 304,
	351, 398, 445, 504,  551, 598, 656, 703, 750, 797,
	856, 903, 949, 996, 1055, 1102
};

static const unsigned char sy7806_torch_level[OPPO_LEVEL_NUM] = {
    0x08, 0x10, 0x18, 0x21, 0x29, 0x31, 0x3A, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char sy7806_flash_level[OPPO_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D};

static volatile unsigned char oppo_reg_enable;
static volatile int oppo_level_ch1 = -1;
static volatile int oppo_level_ch2 = -1;

static int oppo_is_torch(int level)
{
	if (level >= OPPO_LEVEL_TORCH)
		return -1;

	return 0;
}

static int oppo_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= OPPO_LEVEL_NUM)
		level = OPPO_LEVEL_NUM - 1;

	return level;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int oppo_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct oppo_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

/* i2c wrapper function */
static int oppo_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct oppo_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);
	if (val < 0)
		pr_err("failed read at 0x%02x\n", reg);

	return val;
}

/* flashlight enable function */
static int oppo_enable_ch1(void)
{
	unsigned char reg, val;

	reg = OPPO_REG_ENABLE;
	if (!oppo_is_torch(oppo_level_ch1)) {
		/* torch mode */
		oppo_reg_enable |= OPPO_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		oppo_reg_enable |= OPPO_ENABLE_LED1_FLASH;
	}
	val = oppo_reg_enable;

	return oppo_write_reg(oppo_i2c_client, reg, val);
}

static int oppo_enable_ch2(void)
{
	unsigned char reg, val;

	reg = OPPO_REG_ENABLE;
	if (!oppo_is_torch(oppo_level_ch2)) {
		/* torch mode */
		oppo_reg_enable |= OPPO_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		oppo_reg_enable |= OPPO_ENABLE_LED2_FLASH;
	}
	val = oppo_reg_enable;

	return oppo_write_reg(oppo_i2c_client, reg, val);
}

static int oppo_enable(int channel)
{
	if (channel == OPPO_CHANNEL_CH1)
		oppo_enable_ch1();
	else if (channel == OPPO_CHANNEL_CH2)
		oppo_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight disable function */
static int oppo_disable_ch1(void)
{
	unsigned char reg, val;

	reg = OPPO_REG_ENABLE;
	if (oppo_reg_enable & OPPO_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		oppo_reg_enable &= (~OPPO_ENABLE_LED1);
	} else {
		/* if LED 2 is enable, disable LED 1 and clear mode */
		oppo_reg_enable &= (~OPPO_ENABLE_LED1_FLASH);
	}
	val = oppo_reg_enable;

	return oppo_write_reg(oppo_i2c_client, reg, val);
}

static int oppo_disable_ch2(void)
{
	unsigned char reg, val;

	reg = OPPO_REG_ENABLE;
	if (oppo_reg_enable & OPPO_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		oppo_reg_enable &= (~OPPO_ENABLE_LED2);
	} else {
		/* if LED 1 is enable, disable LED 2 and clear mode */
		oppo_reg_enable &= (~OPPO_ENABLE_LED2_FLASH);
	}
	val = oppo_reg_enable;

	return oppo_write_reg(oppo_i2c_client, reg, val);
}

static int oppo_disable(int channel)
{
	if (channel == OPPO_CHANNEL_CH1)
		oppo_disable_ch1();
	else if (channel == OPPO_CHANNEL_CH2)
		oppo_disable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int oppo_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = oppo_verify_level(level);

	/* set torch brightness level */
	reg = OPPO_REG_TORCH_LEVEL_LED1;
	val = oppo_torch_level[level];
	ret = oppo_write_reg(oppo_i2c_client, reg, val);

	oppo_level_ch1 = level;

	/* set flash brightness level */
	reg = OPPO_REG_FLASH_LEVEL_LED1;
	val = oppo_flash_level[level];
	ret = oppo_write_reg(oppo_i2c_client, reg, val);

	return ret;
}

int oppo_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = oppo_verify_level(level);

	/* set torch brightness level */
	reg = OPPO_REG_TORCH_LEVEL_LED2;
	val = oppo_torch_level[level];
	ret = oppo_write_reg(oppo_i2c_client, reg, val);

	oppo_level_ch2 = level;

	/* set flash brightness level */
	reg = OPPO_REG_FLASH_LEVEL_LED2;
	val = oppo_flash_level[level];
	ret = oppo_write_reg(oppo_i2c_client, reg, val);

	return ret;
}

static int oppo_set_level(int channel, int level)
{

	if (channel == OPPO_CHANNEL_CH1)
		oppo_set_level_ch1(level);
	else if (channel == OPPO_CHANNEL_CH2)
		oppo_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight init */
int oppo_init(void)
{
	int ret;
	unsigned char reg, val;

	oppo_pinctrl_set(OPPO_PINCTRL_PIN_HWEN, OPPO_PINCTRL_PINSTATE_HIGH);
    msleep(2);

	/* clear enable register */
	reg = OPPO_REG_ENABLE;
	val = OPPO_DISABLE;
	ret = oppo_write_reg(oppo_i2c_client, reg, val);

	oppo_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = OPPO_REG_TIMING_CONF;
	val = OPPO_TORCH_RAMP_TIME | OPPO_FLASH_TIMEOUT;
	ret = oppo_write_reg(oppo_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int oppo_uninit(void)
{
	oppo_disable(OPPO_CHANNEL_CH1);
	oppo_disable(OPPO_CHANNEL_CH2);
	oppo_pinctrl_set(OPPO_PINCTRL_PIN_HWEN, OPPO_PINCTRL_PINSTATE_LOW);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer oppo_timer_ch1;
static struct hrtimer oppo_timer_ch2;
static unsigned int oppo_timeout_ms[OPPO_CHANNEL_NUM];

static void oppo_work_disable_ch1(struct work_struct *data)
{
	pr_info("ht work queue callback\n");

	oppo_disable_ch1();
}

static void oppo_work_disable_ch2(struct work_struct *data)
{
	pr_info("lt work queue callback\n");

	oppo_disable_ch2();
}

static enum hrtimer_restart oppo_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&oppo_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart oppo_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&oppo_work_ch2);
	return HRTIMER_NORESTART;
}

int oppo_timer_start(int channel, ktime_t ktime)
{
	if (channel == OPPO_CHANNEL_CH1)
		hrtimer_start(&oppo_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == OPPO_CHANNEL_CH2)
		hrtimer_start(&oppo_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

int oppo_timer_cancel(int channel)
{
	if (channel == OPPO_CHANNEL_CH1)
		hrtimer_cancel(&oppo_timer_ch1);
	else if (channel == OPPO_CHANNEL_CH2)
		hrtimer_cancel(&oppo_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int oppo_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= OPPO_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_info("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		oppo_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		oppo_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (oppo_timeout_ms[channel]) {
				ktime = ktime_set(oppo_timeout_ms[channel] / 1000,
						(oppo_timeout_ms[channel] % 1000) * 1000000);
				oppo_timer_start(channel, ktime);
			}
			oppo_enable(channel);
		} else {
			oppo_disable(channel);
			oppo_timer_cancel(channel);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = OPPO_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = OPPO_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = oppo_verify_level(fl_arg->arg);
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = oppo_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_info("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = OPPO_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int oppo_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int oppo_release(void)
{
	/* uninit chip and clear usage count */
/*
	mutex_lock(&oppo_mutex);
	use_count--;
	if (!use_count)
		oppo_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&oppo_mutex);

	pr_info("Release: %d\n", use_count);
*/
	return 0;
}

static int oppo_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&oppo_mutex);
	if (set) {
		if (!use_count)
			ret = oppo_init();
		use_count++;
		pr_info("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = oppo_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_info("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&oppo_mutex);

	return ret;
}

static ssize_t oppo_strobe_store(struct flashlight_arg arg)
{
	oppo_set_driver(1);
	oppo_set_level(arg.channel, arg.level);
	oppo_timeout_ms[arg.channel] = 0;
	oppo_enable(arg.channel);
	msleep(arg.dur);
	oppo_disable(arg.channel);
	//oppo_release(NULL);
	oppo_set_driver(0);
	return 0;
}

static struct flashlight_operations oppo_ops = {
	oppo_open,
	oppo_release,
	oppo_ioctl,
	oppo_strobe_store,
	oppo_set_driver
};

/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int oppo_chip_init(struct oppo_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * oppo_init();
	 */

	return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OPPO Debug file
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t oppo_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;
	for(i=0;i<0x0E;i++)
	{
		reg_val = oppo_read_reg(oppo_i2c_client,i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%2X \n, ", i,reg_val);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t oppo_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		//i2c_write_reg(databuf[0],databuf[1]);
		oppo_write_reg(oppo_i2c_client,databuf[0],databuf[1]);
	}
	return len;
}

static ssize_t oppo_get_hwen(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "//oppo_hwen_on(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 1 > hwen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "//oppo_hwen_off(void)\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "echo 0 > hwen\n");
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");

	return len;
}

static ssize_t oppo_set_hwen(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned char databuf[16];

	sscanf(buf,"%c",&databuf[0]);
#if 1
	if(databuf[0] == 0) {			// OFF
		//oppo_hwen_low();
	} else {				// ON
		//oppo_hwen_high();
	}
#endif
	return len;
}

static int oppo_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);
	err = device_create_file(dev, &dev_attr_hwen);

	return err;
}

static int oppo_parse_dt(struct device *dev,
		struct oppo_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				OPPO_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

#ifdef ODM_WT_EDIT
#ifdef OPPO_FLASHLIGHT_TEST
/* Xuegui.Bao@Camera.Driver, 2018/10/08, add for [wingtech ATO factory app camera] */
int high_cct_led_strobe_enable_part1(void)
{
    oppo_pinctrl_set(OPPO_PINCTRL_PIN_HWEN, OPPO_PINCTRL_PINSTATE_HIGH);

    mdelay(1);

    pr_info("!!!\n");
    return 0;
}

int high_cct_led_strobe_setduty_part1(int duty)
{
    int ret = 0;
    /* set torch brightness level */
    ret = oppo_write_reg(oppo_i2c_client, OPPO_REG_TORCH_LEVEL_LED1, duty);

    return 0;

}

int high_cct_led_strobe_on_part1(int onoff)
{
    unsigned int temp;

    oppo_write_reg(oppo_i2c_client, OPPO_REG_TIMING_CONF, 0x1A);

    temp = oppo_read_reg(oppo_i2c_client, OPPO_REG_ENABLE);
    temp = onoff ? ((temp & 0x83) | (OPPO_TORCH_MODE | OPPO_ENABLE_LED1)) : (((temp & 0x83) | OPPO_TORCH_MODE) & (~OPPO_ENABLE_LED1));
    oppo_write_reg(oppo_i2c_client, OPPO_REG_ENABLE, temp);

    return 0;
}

int low_cct_led_strobe_enable_part1(void)
{
    oppo_pinctrl_set(OPPO_PINCTRL_PIN_HWEN, OPPO_PINCTRL_PINSTATE_HIGH);

    mdelay(1);

    pr_info("!!!\n");
    return 0;
}

int low_cct_led_strobe_setduty_part1(int duty)
{
  int ret = 0;
  /* set torch brightness level */
  ret = oppo_write_reg(oppo_i2c_client, OPPO_REG_TORCH_LEVEL_LED2, duty);

  return 0;

}

int low_cct_led_strobe_on_part1(int onoff)
{
    unsigned int temp;

    oppo_write_reg(oppo_i2c_client, OPPO_REG_TIMING_CONF, 0x1A);

    temp = oppo_read_reg(oppo_i2c_client, OPPO_REG_ENABLE);
    temp = onoff ? ((temp & 0x83) | (OPPO_TORCH_MODE | OPPO_ENABLE_LED2)) : (((temp & 0x83) | OPPO_TORCH_MODE) & (~OPPO_ENABLE_LED2));
    oppo_write_reg(oppo_i2c_client, OPPO_REG_ENABLE, temp);

    return 0;
}
#endif
#endif

enum FLASHLIGHT_DEVICE {
	AW3643_SM = 0x12,
	SY7806_SM = 0x1c,
};

#define USE_AW3643_IC	0x0001
#define USE_SY7806_IC	0x0011
#define USE_NOT_PRO		0x1111

static int oppo_chip_id(void)
{
	char chip_id;

	oppo_pinctrl_set(OPPO_PINCTRL_PIN_HWEN, OPPO_PINCTRL_PINSTATE_HIGH);
	chip_id = oppo_read_reg(oppo_i2c_client, 0x0c);
	pr_info("flashlight chip id: reg:0x0c, data:0x%x", chip_id);
	oppo_pinctrl_set(OPPO_PINCTRL_PIN_HWEN, OPPO_PINCTRL_PINSTATE_LOW);

	if (chip_id == AW3643_SM) {
		pr_info(" the device's flashlight driver IC is AW3643\n");
		return USE_AW3643_IC;
	} else if (chip_id == SY7806_SM){
		pr_info(" the device's flashlight driver IC is SY7806\n");
		return USE_SY7806_IC;
	} else {
		pr_err(" the device's flashlight driver IC is not used in our project!\n");
		return USE_NOT_PRO;
	}
}

static int oppo_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct oppo_chip_data *chip;
	struct oppo_platform_data *pdata = client->dev.platform_data;
	int err;
	int i;
	int chip_id;

	pr_info("Probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct oppo_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_err("Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct oppo_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		client->dev.platform_data = pdata;
		err = oppo_parse_dt(&client->dev, pdata);
		if (err)
			goto err_free;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	oppo_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&oppo_work_ch1, oppo_work_disable_ch1);
	INIT_WORK(&oppo_work_ch2, oppo_work_disable_ch2);

	/* init timer */
	hrtimer_init(&oppo_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	oppo_timer_ch1.function = oppo_timer_func_ch1;
	hrtimer_init(&oppo_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	oppo_timer_ch2.function = oppo_timer_func_ch2;
	oppo_timeout_ms[OPPO_CHANNEL_CH1] = 100;
	oppo_timeout_ms[OPPO_CHANNEL_CH2] = 100;

	/* init chip hw */
	oppo_chip_init(chip);

	chip_id = oppo_chip_id();
	if(chip_id == USE_AW3643_IC){
		oppo_current = aw3643_current;
		oppo_torch_level = aw3643_torch_level;
		oppo_flash_level = aw3643_flash_level;
	}
	else if (chip_id == USE_SY7806_IC){
		oppo_current = sy7806_current;
		oppo_torch_level = sy7806_torch_level;
		oppo_flash_level = sy7806_flash_level;
	} else if (chip_id == USE_NOT_PRO){
		oppo_current = aw3643_current;
		oppo_torch_level = aw3643_torch_level;
		oppo_flash_level = aw3643_flash_level;
	}

	/* register flashlight operations */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&oppo_ops)) {
                pr_err("Failed to register flashlight device.\n");
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(OPPO_NAME, &oppo_ops)) {
			pr_err("Failed to register flashlight device.\n");
			err = -EFAULT;
			goto err_free;
		}
	}

    oppo_create_sysfs(client);

	pr_info("Probe done.\n");

	return 0;

err_free:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int oppo_i2c_remove(struct i2c_client *client)
{
	struct oppo_platform_data *pdata = dev_get_platdata(&client->dev);
	struct oppo_chip_data *chip = i2c_get_clientdata(client);
	int i;

	pr_info("Remove start.\n");

	client->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(OPPO_NAME);
	/* flush work queue */
	flush_work(&oppo_work_ch1);
	flush_work(&oppo_work_ch2);

	/* unregister flashlight operations */
	flashlight_dev_unregister(OPPO_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	pr_info("Remove done.\n");

	return 0;
}

static const struct i2c_device_id oppo_i2c_id[] = {
	{OPPO_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id oppo_i2c_of_match[] = {
	{.compatible = OPPO_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver oppo_i2c_driver = {
	.driver = {
		   .name = OPPO_NAME,
#ifdef CONFIG_OF
		   .of_match_table = oppo_i2c_of_match,
#endif
		   },
	.probe = oppo_i2c_probe,
	.remove = oppo_i2c_remove,
	.id_table = oppo_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int oppo_probe(struct platform_device *dev)
{
	pr_info("Probe start.\n");

	/* init pinctrl */
	if (oppo_pinctrl_init(dev)) {
		printk("Failed to init pinctrl.\n");
		return -1;
	}

	if (i2c_add_driver(&oppo_i2c_driver)) {
		printk("Failed to add i2c driver.\n");
		return -1;
	}

	pr_info("Probe done.\n");

	return 0;
}

static int oppo_remove(struct platform_device *dev)
{
	pr_info("Remove start.\n");

	i2c_del_driver(&oppo_i2c_driver);

	pr_info("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id oppo_of_match[] = {
	{.compatible = OPPO_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, oppo_of_match);
#else
static struct platform_device oppo_platform_device[] = {
	{
		.name = OPPO_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, oppo_platform_device);
#endif

static struct platform_driver oppo_platform_driver = {
	.probe = oppo_probe,
	.remove = oppo_remove,
	.driver = {
		.name = OPPO_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = oppo_of_match,
#endif
	},
};

static int __init flashlight_oppo_init(void)
{
	int ret;

	pr_info("flashlight_oppo-Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&oppo_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&oppo_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_info("flashlight_oppo Init done.\n");

	return 0;
}

static void __exit flashlight_oppo_exit(void)
{
	pr_info("flashlight_oppo-Exit start.\n");

	platform_driver_unregister(&oppo_platform_driver);

	pr_info("flashlight_oppo Exit done.\n");
}


module_init(flashlight_oppo_init);
module_exit(flashlight_oppo_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight OPPO Driver");


