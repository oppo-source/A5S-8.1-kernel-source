/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - config.c
** Description : This program is for ili9881 driver confic.c
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#include "../common.h"
#include "../platform.h"
#include "config.h"
#include "protocol.h"
#include "i2c.h"
#include "flash.h"
#include "gesture.h"
#include "finger_report.h"
#include "mp_test.h"
#include "firmware.h"

#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/21,Add for hardware info and compatible txd and hlt ctp
#include <linux/hardware_info.h>
extern int ili_ctpmodule;
#endif
extern struct ili_gesture_info * gesture_report_data;
#ifdef ILITEK_ESD_CHECK
extern void ilitek_cancel_esd_check(void);
#endif
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Function.1372106,2018/5/21,Add for ctp hardware info
	char ili_version[20] = {"0"};
#endif

bool gesture_done = false;
extern bool psensor_close;

#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/21,Add for hardware info for OPPO
extern void devinfo_info_tp_set(char *version, char *manufacture, char *fw_path);

#endif
//extern int  ili_power_on(struct ilitek_platform_data *ts, bool on);
extern struct ilitek_platform_data *ipd;

/* the list of support chip */
uint32_t ipio_chip_list[] = {
	CHIP_TYPE_ILI7807,
	CHIP_TYPE_ILI9881,
};

uint8_t g_read_buf[128] = { 0 };

struct core_config_data *core_config = NULL;
static void read_flash_info(uint8_t cmd, int len)
{
	int i;
	uint16_t flash_id = 0, flash_mid = 0;
	uint8_t buf[4] = { 0 };

	/*
	 * This command is used to fix the bug of spi clk for 7807F-AB
	 * when operating with its flash.
	 */
	if (core_config->chip_id == CHIP_TYPE_ILI7807 && core_config->chip_type == ILI7807_TYPE_F_AB) {
		core_config_ice_mode_write(0x4100C, 0x01, 1);
		msleep(25);
	}

	core_config_ice_mode_write(0x41000, 0x0, 1);	/* CS high */
	core_config_ice_mode_write(0x41004, 0x66aa55, 3);	/* Key */

	core_config_ice_mode_write(0x41008, cmd, 1);
	for (i = 0; i < len; i++) {
		core_config_ice_mode_write(0x041008, 0xFF, 1);
		buf[i] = core_config_ice_mode_read(0x41010);
	}

	core_config_ice_mode_write(0x041000, 0x1, 1);	/* CS high */

	/* look up flash info and init its struct after obtained flash id. */
	flash_mid = buf[0];
	flash_id = buf[1] << 8 | buf[2];
	core_flash_init(flash_mid, flash_id);
}

/*
 * It checks chip id shifting sepcific bits based on chip's requirement.
 *
 * @pid_data: 4 bytes, reading from firmware.
 *
 */
static uint32_t check_chip_id(uint32_t pid_data)
{
	int i;
	uint32_t id = 0;
	uint32_t type = 0;

	id = pid_data >> 16;
	type = (pid_data & 0x0000FF00) >> 8;  //mfeng modify

	ipio_info("id = 0x%x, type = 0x%x\n", id, type);

	if(id == CHIP_TYPE_ILI9881) {
		for(i = ILI9881_TYPE_F; i <= ILI9881_TYPE_H; i++) {
			if (i == type) {
				core_config->chip_type = i;
				core_config->ic_reset_addr = 0x040050;
				return id;
			}
		}
	}

	if(id == CHIP_TYPE_ILI7807) {
		for(i = ILI7807_TYPE_F_AA; i <= ILI7807_TYPE_H; i++) {
			if (i == type) {
				core_config->chip_type = i;
				if (i == ILI7807_TYPE_F_AB)
					core_config->ic_reset_addr = 0x04004C;
				else if (i == ILI7807_TYPE_H)
					core_config->ic_reset_addr = 0x040050;

				return id;
			}
		}
	}

	return 0;
}

/*
 * Read & Write one byte in ICE Mode.
 */
uint32_t core_config_read_write_onebyte(uint32_t addr)
{
	int res = 0;
	uint32_t data = 0;
	uint8_t szOutBuf[64] = { 0 };

	szOutBuf[0] = 0x25;
	szOutBuf[1] = (char)((addr & 0x000000FF) >> 0);
	szOutBuf[2] = (char)((addr & 0x0000FF00) >> 8);
	szOutBuf[3] = (char)((addr & 0x00FF0000) >> 16);

	res = core_write(core_config->slave_i2c_addr, szOutBuf, 4);
	if (res < 0)
		goto out;

	mdelay(1);

	res = core_read(core_config->slave_i2c_addr, szOutBuf, 1);
	if (res < 0)
		goto out;

	data = (szOutBuf[0]);

	return data;

out:
	ipio_err("Failed to read/write data in ICE mode, res = %d\n", res);
	return res;
}
EXPORT_SYMBOL(core_config_read_write_onebyte);

uint32_t core_config_ice_mode_read(uint32_t addr)
{
	int res = 0;
	uint8_t szOutBuf[64] = { 0 };
	uint32_t data = 0;

	szOutBuf[0] = 0x25;
	szOutBuf[1] = (char)((addr & 0x000000FF) >> 0);
	szOutBuf[2] = (char)((addr & 0x0000FF00) >> 8);
	szOutBuf[3] = (char)((addr & 0x00FF0000) >> 16);

	res = core_write(core_config->slave_i2c_addr, szOutBuf, 4);
	if (res < 0)
		goto out;

	mdelay(10);

	res = core_read(core_config->slave_i2c_addr, szOutBuf, 4);
	if (res < 0)
		goto out;

	data = (szOutBuf[0] + szOutBuf[1] * 256 + szOutBuf[2] * 256 * 256 + szOutBuf[3] * 256 * 256 * 256);

	return data;

out:
	ipio_err("Failed to read data in ICE mode, res = %d\n", res);
	return res;
}
EXPORT_SYMBOL(core_config_ice_mode_read);


/*
 * Write commands into firmware in ICE Mode.
 *
 */
int core_config_ice_mode_write(uint32_t addr, uint32_t data, uint32_t size)
{
	int res = 0, i;
	uint8_t szOutBuf[64] = { 0 };

	szOutBuf[0] = 0x25;
	szOutBuf[1] = (char)((addr & 0x000000FF) >> 0);
	szOutBuf[2] = (char)((addr & 0x0000FF00) >> 8);
	szOutBuf[3] = (char)((addr & 0x00FF0000) >> 16);

	for (i = 0; i < size; i++) {
		szOutBuf[i + 4] = (char)(data >> (8 * i));
	}

	res = core_write(core_config->slave_i2c_addr, szOutBuf, size + 4);

	if (res < 0)
		ipio_err("Failed to write data in ICE mode, res = %d\n", res);

	return res;
}
EXPORT_SYMBOL(core_config_ice_mode_write);

/*
 * Doing soft reset on ic.
 *
 * It resets ic's status, moves code and leave ice mode automatically if in
 * that mode.
 */
void core_config_ic_reset(void)
{
#ifdef HOST_DOWNLOAD
	core_config_ice_mode_disable();
#else
	uint32_t key = 0;
	if (core_config->chip_id == CHIP_TYPE_ILI7807) {
		if (core_config->chip_type == ILI7807_TYPE_H)
			key = 0x00117807;
		else
			key = 0x00017807;
	}
	if (core_config->chip_id == CHIP_TYPE_ILI9881) {
		key = 0x00019881;
	}

	ipio_debug(DEBUG_CONFIG, "key = 0x%x\n", key);
	if (key != 0) {
		core_config->do_ic_reset = true;
		core_config_ice_mode_write(core_config->ic_reset_addr, key, 4);
		core_config->do_ic_reset = false;
	}

	msleep(300);
#endif
}
EXPORT_SYMBOL(core_config_ic_reset);
uint32_t ili9881_read_pc_counter(void)
{
	uint32_t pc_cnt = 0x0;

	if (!core_config->icemodeenable) {
		if (core_config_ice_mode_enable() < 0)
			ipio_err("Failed to enter ice mode\n");
	}

	/* Read fw status if it was hanging on a unknown status */
	pc_cnt = core_config_ice_mode_read(ILI9881_PC_COUNTER_ADDR);

	ipio_err("pc counter = 0x%x\n", pc_cnt);

	if (core_config->icemodeenable) {
		if (core_config_ice_mode_disable() < 0)
			ipio_err("Failed to disable ice mode\n");
	}
	return pc_cnt;
}
EXPORT_SYMBOL(ili9881_read_pc_counter);

void core_config_sense_ctrl(bool start)
{
	ipio_debug(DEBUG_CONFIG, "sense start = %d\n", start);

	return core_protocol_func_control(0, start);
}
EXPORT_SYMBOL(core_config_sense_ctrl);

void core_config_sleep_ctrl(bool out)
{
	ipio_debug(DEBUG_CONFIG, "Sleep Out = %d\n", out);

	return core_protocol_func_control(1, out);
}
EXPORT_SYMBOL(core_config_sleep_ctrl);

void core_config_glove_ctrl(bool enable, bool seamless)
{
	int cmd = 0x2;		/* default as semaless */

	if (!seamless) {
		if (enable)
			cmd = 0x1;
		else
			cmd = 0x0;
	}

	ipio_debug(DEBUG_CONFIG, "Glove = %d, seamless = %d, cmd = %d\n", enable, seamless, cmd);

	return core_protocol_func_control(2, cmd);
}
EXPORT_SYMBOL(core_config_glove_ctrl);

void core_config_stylus_ctrl(bool enable, bool seamless)
{
	int cmd = 0x2;		/* default as semaless */

	if (!seamless) {
		if (enable)
			cmd = 0x1;
		else
			cmd = 0x0;
	}

	ipio_debug(DEBUG_CONFIG, "stylus = %d, seamless = %d, cmd = %x\n", enable, seamless, cmd);

	return core_protocol_func_control(3, cmd);
}
EXPORT_SYMBOL(core_config_stylus_ctrl);

void core_config_tp_scan_mode(bool mode)
{
	ipio_debug(DEBUG_CONFIG, "TP Scan mode = %d\n", mode);

	return core_protocol_func_control(4, mode);
}
EXPORT_SYMBOL(core_config_tp_scan_mode);

void core_config_lpwg_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "LPWG = %d\n", enable);

	return core_protocol_func_control(5, enable);
}
EXPORT_SYMBOL(core_config_lpwg_ctrl);

void core_config_gesture_ctrl(uint8_t func)
{
	uint8_t max_byte = 0x0, min_byte = 0x0;

	ipio_debug(DEBUG_CONFIG, "Gesture function = 0x%x\n", func);

	max_byte = 0x3F;
	min_byte = 0x20;

	if (func > max_byte || func < min_byte) {
		ipio_err("Gesture ctrl error, 0x%x\n", func);
		return;
	}

	return core_protocol_func_control(6, func);
}
EXPORT_SYMBOL(core_config_gesture_ctrl);

#ifdef ILITEK_EDGE_LIMIT
void core_config_edge_limit_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "edge limit = %d\n", enable);

	return core_protocol_func_control(13, enable);
}
EXPORT_SYMBOL(core_config_edge_limit_ctrl);
#endif

#ifdef ODM_WT_EDIT
void core_config_game_switch_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "play_mode = %d\n", enable);

	return core_protocol_func_control(14, enable);
}
EXPORT_SYMBOL(core_config_game_switch_ctrl);
#endif

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/10, add ear phone function
void core_config_ear_phone_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "ear_phone_mode = %d\n", enable);

	return core_protocol_func_control(15, enable);
}
EXPORT_SYMBOL(core_config_ear_phone_ctrl);
#endif

void core_config_phone_cover_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "Phone Cover = %d\n", enable);

	return core_protocol_func_control(7, enable);
}
EXPORT_SYMBOL(core_config_phone_cover_ctrl);

void core_config_finger_sense_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "Finger sense = %d\n", enable);

	return core_protocol_func_control(0, enable);
}
EXPORT_SYMBOL(core_config_finger_sense_ctrl);

void core_config_proximity_ctrl(bool enable)
{
	ipio_debug(DEBUG_CONFIG, "Proximity = %d\n", enable);

	return core_protocol_func_control(11, enable);
}
EXPORT_SYMBOL(core_config_proximity_ctrl);

void core_config_plug_ctrl(bool out)
{
	ipio_debug(DEBUG_CONFIG, "Plug Out = %d\n", out);

	return core_protocol_func_control(12, out);
}
EXPORT_SYMBOL(core_config_plug_ctrl);

void core_config_set_phone_cover(uint8_t *pattern)
{
	int i;

	if (pattern == NULL) {
		ipio_err("Invaild pattern\n");
		return;
	}

	for(i = 0; i < 8; i++)
		protocol->phone_cover_window[i+1] = pattern[i];

	ipio_debug(DEBUG_CONFIG, "window: cmd = 0x%x\n", protocol->phone_cover_window[0]);
	ipio_debug(DEBUG_CONFIG, "window: ul_x_l = 0x%x, ul_x_h = 0x%x\n", protocol->phone_cover_window[1],
		 protocol->phone_cover_window[2]);
	ipio_debug(DEBUG_CONFIG, "window: ul_y_l = 0x%x, ul_y_l = 0x%x\n", protocol->phone_cover_window[3],
		 protocol->phone_cover_window[4]);
	ipio_debug(DEBUG_CONFIG, "window: br_x_l = 0x%x, br_x_l = 0x%x\n", protocol->phone_cover_window[5],
		 protocol->phone_cover_window[6]);
	ipio_debug(DEBUG_CONFIG, "window: br_y_l = 0x%x, br_y_l = 0x%x\n", protocol->phone_cover_window[7],
		 protocol->phone_cover_window[8]);

	core_protocol_func_control(9, 0);
}
EXPORT_SYMBOL(core_config_set_phone_cover);

/*
 * ic_suspend: Get IC to suspend called from system.
 *
 * The timing when goes to sense stop or houw much times the command need to be called
 * is depending on customer's system requirement, which might be different due to
 * the DDI design or other conditions.
 */
void core_config_ic_suspend(void)
{
	//uint8_t temp[4] = {0};
	//int res = 0;
	u8 otp;
	int i = 0;
	u8 otp_check = 0x0;
    u8 otp_ok_byte = 0xAA;
    u8 bank2_start_addr = 0x20;
	ipio_info("Starting to suspend ...\n");
	if(otp_wrie_contol_leave == 1){
		core_config_ice_mode_enable();
		core_config_otp_read(&otp, 0, 0, 0);
		core_config_otp_read(&otp_check, 0, bank2_start_addr, 0);
		core_config_ice_mode_disable();
		ipio_info("otp_check = 0x%x\n", otp_check);
		if (otp_check != otp_ok_byte) {
			ipio_err("OTP Bank 2 is not correct in first byte! Recovery\n");
			core_config_otp_copy_bank1_to_bank2();
		}
/*		core_config_ice_mode_enable();
		core_config_otp_read(&otp_move_result, 0, bank2_start_addr, 0);
		core_config_ice_mode_disable();*/
		ipd->otp_move_flag=1;
		wake_up(&(ipd->otp_inq));
	}
	for (i = 0; i < 40; i++) {
		if (!core_firmware->isUpgrading) {
			break;
		}
		msleep(5);
	}
//#ifdef ILITEK_ESD_CHECK
	//ilitek_cancel_esd_check();
//#endif
	/* sense stop */
	core_config_sense_ctrl(false);

	/* check system busy */
	if (core_config_check_cdc_busy(5, 50) < 0) {
		ipio_err("Check busy is timout !\n");
		core_config_sense_ctrl(false);
		core_config_check_cdc_busy(5, 50);
	}

ipio_debug(DEBUG_CONFIG, "core_config->isEnableGesture = %d before\n", core_config->isEnableGesture);
if(core_config->isEnableGesture)
{
	ipio_debug(DEBUG_CONFIG, "core_config->isEnableGesture = %d after\n", core_config->isEnableGesture);

	enable_irq_wake(ipd->isr_gpio);

	ipio_info("Enabled Gesture = %d\n", core_config->isEnableGesture);

		core_fr->actual_fw_mode = P5_0_FIRMWARE_GESTURE_MODE;
		#ifdef HOST_DOWNLOAD
		if(core_load_gesture_code() < 0)
			{
			ipio_err("load gesture code fail\n");
			goto out;
			}
		ilitek_platform_enable_irq();
		#endif
		if(psensor_close == true) {
			core_config->isEnableGesture = false;
			core_config_sleep_ctrl(false);
			ipio_info("suspend set sleep ctrl redo\n");
		}
		gesture_done = true;
 }else {
	/*res = gpio_request(ipd->lcd_reset_gpio, "ILITEK_LCD_RESET");
	if (res < 0) {
		ipio_err("Request LCD RESET GPIO failed, res = %d\n", res);
		gpio_free(ipd->lcd_reset_gpio);
		res = gpio_request(ipd->lcd_reset_gpio, "ILITEK_LCD_RESET");
		if (res < 0) {
			ipio_err("Retrying request LCD RESET GPIO still failed , res = %d\n", res);
			goto out;
		}
	}*/
	//gpio_direction_output(ipd->lcd_reset_gpio,0);
	//gpio_free(ipd->lcd_reset_gpio);
	gesture_done = false;
	//gpio_set_value(ipd->reset_gpio,0);
	//res = ili_power_on(ipd,false);
	//if (res) {
		//ipio_info("power off fail\n");
	//}
}
out:
	ipio_info("Suspend done\n");

}
EXPORT_SYMBOL(core_config_ic_suspend);

/*
 * ic_resume: Get IC to resume called from system.
 *
 * The timing when goes to sense start or houw much times the command need to be called
 * is depending on customer's system requirement, which might be different due to
 * the DDI design or other conditions.
 */
void core_config_ic_resume(void)
{
 //if(core_gesture->suspend)
 {
	//core_gesture->suspend = false;
	//int res = 0;
	ipio_info("Starting to resume ...\n");
	if (core_config->isEnableGesture) {
		#ifdef HOST_DOWNLOAD
		if(core_load_ap_code() < 0)
		{
			ipio_err("load ap code fail\n");
			ilitek_platform_tp_hw_reset(true);
		}
		#endif
	}
	else{
		/*res = ili_power_on(ipd,true);
		if (res) {
			ipio_info("power on fail\n");
		}
		ilitek_platform_tp_hw_reset(true);*/
	}
	/* sleep out */
	core_config_sleep_ctrl(true);

	/* check system busy */
	if (core_config_check_cdc_busy(50, 50) < 0)
		ipio_err("Check busy is timout !\n");

	/* sense start for TP */
	core_config_sense_ctrl(true);
	core_fr_mode_control( &protocol->demo_mode);
	/* Soft reset */
	// core_config_ice_mode_enable();
	// mdelay(10);
	// core_config_ic_reset();
	ipio_info("Resume done\n");
 }
}
EXPORT_SYMBOL(core_config_ic_resume);

static int core_config_ice_mode_chipid_check(void)
{
	int ret = 0;
	uint32_t pid = 0;

	pid = core_config_ice_mode_read(core_config->pid_addr);

	if (((pid >> 16) != CHIP_TYPE_ILI9881) && ((pid >> 16) != CHIP_TYPE_ILI7807)) {
		ipio_info("read PID Fail  pid = 0x%x\n", (pid >> 16));
		ret = -EINVAL;
	} else {
		core_config->chip_pid = pid;
		core_config->chip_id = pid >> 16;
		core_config->chip_type = (pid & 0x0000FF00) >> 8;
		core_config->core_type = pid & 0xFF;
	}

	return ret;
}
int core_config_ice_mode_disable(void)
{
	uint32_t res = 0;
	uint8_t cmd[4];
	cmd[0] = 0x1b;
	cmd[1] = 0x62;
	cmd[2] = 0x10;
	cmd[3] = 0x18;

	ipio_debug(DEBUG_CONFIG, "ICE Mode disabled\n");
	res = core_write(core_config->slave_i2c_addr, cmd, 4);
	core_config->icemodeenable = false;
	return res;
}
EXPORT_SYMBOL(core_config_ice_mode_disable);

int core_config_ice_mode_enable(void)
{
	int retry = 3;
	ipio_debug(DEBUG_CONFIG, "ICE Mode enabled\n");
	core_config->icemodeenable = true;
	do {
		if (core_config_ice_mode_write(0x181062, 0x0, 0) < 0) {
			ipio_info("ICE Mode enabled fail \n");
			return -1;
		}
		if (core_config_ice_mode_chipid_check() >= 0) {
			core_config_ice_mode_write(0x047002, 0x00, 1);
			ipio_debug(DEBUG_CONFIG, "chipid check scuess\n");
			return 0;
		}
		ipio_info("ICE Mode enabled fail %d time\n", (4 - retry));
	} while (--retry > 0);
	ipio_info("ICE Mode enabled time out \n");
	return -1;
}
EXPORT_SYMBOL(core_config_ice_mode_enable);
#if 0
u8 read_opt_first_byte(void)
{
	u8 otp;
    core_config_ice_mode_enable();
    core_config_ice_mode_write(0x43008, 0x80, 1);
    core_config_ice_mode_write(0x43000, 0x0, 2);
    core_config_ice_mode_write(0x4300C, 0x01, 2);

    otp = core_config_read_write_onebyte(0x4300D);
    ipio_debug(DEBUG_CONFIG,"otp = %x\n", otp);
	if (otp != 0xaa) {
	ipio_info("otp error = %x\n", otp);
	}
    core_config_ice_mode_write(0x4300C, 0x02, 2);
    core_config_ice_mode_disable();
	return otp;
}
EXPORT_SYMBOL(read_opt_first_byte);
#else
static int core_config_wr_pack(int packet)
{
	int retry = 5;
	uint32_t reg_data = 0;
	while(retry--) {
		reg_data = core_config_read_write_onebyte(0x73010);
		if ((reg_data & 0x02) == 0) {
			ipio_info("check ok 0x73010 read 0x%X retry = %d\n", reg_data, retry);
			break;
		}
		mdelay(10);
	}
	core_config_ice_mode_write(0x73000, packet, 4);

	if (retry <= 0) {
		ipio_info("check 0x73010 error read 0x%X\n", reg_data);
		return -1;
	}
	return 0;
}

static void core_config_after_write_otp(void)
{
	int ret = 0;
	ipio_info();

	// Switch VGH to display mode
	// Write ON
	ret = core_config_wr_pack(0x1FFF9527);
	if(ret < 0){
		ipio_info("check 0x1FFF9527 Write ON error\n");
	}
	// Switch to Page 00
	ret = core_config_wr_pack(0x1FFFFF00);
	if(ret < 0){
		ipio_info("check 0x1FFFFF00 Page 00 error\n");
	}
	// DDI Sleep In
	ret = core_config_wr_pack(0x1F100101);
	if(ret < 0){
		ipio_info("check 0x1F100101 DDI Sleep In error\n");
	}
	// Write OFF
	ret = core_config_wr_pack(0x1FFF9500);
	if(ret < 0){
		ipio_info("check 0x1FFF9500 Write OFF error\n");
	}
	mdelay(150);

	// Write ON
	ret = core_config_wr_pack(0x1FFF9527);
	if(ret < 0){
		ipio_info("check 0x1FFF9527 Write ON error\n");
	}

	// Switch to Page 06
	ret = core_config_wr_pack(0x1FFFFF06);
	if(ret < 0){
		ipio_info("check 0x1FFFFF06 Switch to Page 06 error\n");
	}

	// Write CMD 3C to 00
	ret = core_config_wr_pack(0x1F3C0100);
	if(ret < 0){
		ipio_info("check 0x1F3C0100 Write CMD 3C to 00 error\n");
	}

	// Write OFF
	ret = core_config_wr_pack(0x1FFF9500);
	if(ret < 0){
		ipio_info("check 0x1FFF9500 Write OFF error\n");
	}

	mdelay(150);

	// Write ON
	ret = core_config_wr_pack(0x1FFF9527);
	if(ret < 0){
		ipio_info("check 0x1FFF9527 Write ON error\n");
	}

	// Switch to Page 00
	ret = core_config_wr_pack(0x1FFFFF00);
	if(ret < 0){
		ipio_info("check 0x1FFFFF00 Switch to Page 00 error\n");
	}

	// Sleep Out
	ret = core_config_wr_pack(0x1F110101);
	if(ret < 0){
		ipio_info("check 0x1F110101 Sleep Out error\n");
	}

	// Display On
	//core_config_wr_pack(0x1F290101);
	// Write OFF
	ret = core_config_wr_pack(0x1FFF9500);
	if(ret < 0){
		ipio_info("check 0x1FFF9500 Write OFF error\n");
	}

	mdelay(150);
	/* ---- Display On ---- */
	// Write ON
	ret = core_config_wr_pack(0x1FFF9527);
	if(ret < 0){
		ipio_info("check 0x1FFF9527 Write ON error\n");
	}

	// Switch to Page 00
	ret = core_config_wr_pack(0x1FFFFF00);
	if(ret < 0){
		ipio_info("check 0x1FFFFF00 Switch to Page 00 error\n");
	}

	// Write CMD10 to 01
	ret = core_config_wr_pack(0x1F290101);
	if(ret < 0){
		ipio_info("check 0x1F290101 Write CMD10 to 01 error\n");
	}

	// Write OFF
	ret = core_config_wr_pack(0x1FFF9500);
	if(ret < 0){
		ipio_info("check 0x1FFF9500 Write OFF error\n");
	}

}

static int core_config_before_write_otp(void)
{
	int ret = 0, retry = 1000;
	u8 sleep_out = 0;

	ipio_info();

	/* --- LCD Off --- */
	// Write ON
	ret = core_config_wr_pack(0x1FFF9527);
	if(ret < 0){
		ipio_info("check 0x1FFF9527 Write ON error\n");
	}
	// Switch to Page 00
	ret = core_config_wr_pack(0x1FFFFF00);
	if(ret < 0){
		ipio_info("check 0x1FFFFF00 Switch to Page 00 error\n");
	}
	// Write Display Off
	ret = core_config_wr_pack(0x1F280101);
	if(ret < 0){
		ipio_info("check 0x1F280101 Write Display Off error\n");
	}
	// Write OFF
	ret = core_config_wr_pack(0x1FFF9500);
	if(ret < 0){
		ipio_info("check 0x1FFF9500 Write OFF error\n");
	}

	mdelay(150);

	// Write ON
	ret = core_config_wr_pack(0x1FFF9527);
	if(ret < 0){
		ipio_info("check 0x1FFF9527 Write ON error\n");
	}
	// DDI Sleep In
	ret = core_config_wr_pack(0x1F100101);
	if(ret < 0){
		ipio_info("check 0x1F100101 DDI Sleep In error\n");
	}
	// Write OFF
	ret = core_config_wr_pack(0x1FFF9500);
	if(ret < 0){
		ipio_info("check 0x1FFF9500 Write OFF error\n");
	}

	mdelay(150);

	// Write ON
	ret = core_config_wr_pack(0x1FFF9527);
	if(ret < 0){
		ipio_info("check 0x1FFF9527 Write ON error\n");
	}
	// Switch VGH to write mode
	// Switch to Page 06
	ret = core_config_wr_pack(0x1FFFFF06);
	if(ret < 0){
		ipio_info("check 0x1FFFFF06 Switch to Page 06 error\n");
	}
	// Write CMD 3C to 01
	ret = core_config_wr_pack(0x1F3C0101);
	if(ret < 0){
		ipio_info("check 0x1F3C0101 Write CMD 3C to 01 error\n");
	}
	// Write OFF
	ret = core_config_wr_pack(0x1FFF9500);
	if(ret < 0){
		ipio_info("check 0x1FFF9500 Write OFF error\n");
	}

	mdelay(150);

	if (core_config->chip_id == CHIP_TYPE_ILI9881)
		sleep_out = core_config_read_write_onebyte(0x45028);
	else
		sleep_out = core_config_read_write_onebyte(0x4502C);

	ipio_info("OTP: sleep_out (page06)= %x\n", sleep_out);

	if ((sleep_out & 0x02) != 0x02) {
		// Write ON
		ret = core_config_wr_pack(0x1FFF9527);
		if(ret < 0){
			ipio_info("check 0x1FFF9527 Write ON error\n");
		}
		// Switch to Page 00
		ret = core_config_wr_pack(0x1FFFFF00);
		if(ret < 0){
			ipio_info("check 0x1FFFFF00 Switch to Page 00 error\n");
		}
		// Write CMD11 to 01
		ret = core_config_wr_pack(0x1F110101);
		if(ret < 0){
			ipio_info("check 0x1F110101 Write CMD11 to 01 error\n");
		}
		// Write OFF
		ret = core_config_wr_pack(0x1FFF9500);
		if(ret < 0){
			ipio_info("check 0x1FFF9500 Write OFF error\n");
		}
		do {
			if (core_config->chip_id == CHIP_TYPE_ILI9881)
				sleep_out = core_config_read_write_onebyte(0x45028);
			else
				sleep_out = core_config_read_write_onebyte(0x4502C);
			retry--;
			ipio_info("OTP: sleep_out (page00) = %x\n", sleep_out);
		} while (((sleep_out & 0x02) != 0x02) && retry > 0);
	}

	if (retry <= 0) {
		ipio_err("OTP: sleep out timeout!\n");
		ret = -1;
	}
	mdelay(150);
	return ret;
}

void core_config_otp_read(u8 *otp_data, size_t data_size, u32 start, size_t len)
{
	int i;

	ipio_info("data_size = %ld, start addr = 0x%x, read_len = %ld\n", data_size, start, len);

	core_config_ice_mode_write(0x43008, 0x80, 1);
	core_config_ice_mode_write(0x43000, start, 2);
	core_config_ice_mode_write(0x4300C, 0x01, 2);

	otp_data[0] = core_config_read_write_onebyte(0x4300D);
    ipio_debug(DEBUG_CONFIG,"otp = %x\n", otp_data[0]);
	if (otp_data[0] != 0xaa) {
	ipio_info("otp error = %x\n", otp_data[0]);
	}

	for (i = start; i < (start + len); i++) {
		core_config_ice_mode_write(0x43000, i, 2);
		otp_data[i - start] = core_config_read_write_onebyte(0x4300D);
		ipio_debug(DEBUG_CONFIG,"Reading OTP data .... %ld%c", (((i - start) * 100) / len), '%');
	}

	core_config_ice_mode_write(0x4300C, 0x02, 2);
}
EXPORT_SYMBOL(core_config_otp_read);
int core_config_otp_write_check(u8 *otp_data, u32 start, size_t w_len)
{
	int i, ret = 0, retry = 1000;
	u8 prog_one_done = 0x00;

	ipio_info("OTP: start addr = 0x%x, len = %ld\n", start, w_len);

	dump_data1(otp_data, 8, w_len, 0, "Write OTP");

	core_config_ice_mode_write(0x43000, start, 2);
	core_config_ice_mode_write(0x43004, otp_data[0], 1);
	core_config_ice_mode_write(0x43008, 0x91, 1);//9881h
	core_config_ice_mode_write(0x4300C, 0x01, 1);

	do {
		prog_one_done = core_config_read_write_onebyte(0x4300C);
		retry--;
	} while (((prog_one_done & 0x40) != 0x40) && retry > 0);

	if (retry <= 0 && (prog_one_done & 0x40) != 0x40) {
		ipio_err("OTP: prog_one_done (0x%x) error, retry = %d\n", prog_one_done, retry);
		ret = -1;
		goto out;
	} else {
		for (i = start + 1; i < start + w_len; i++) {
			core_config_ice_mode_write(0x43000, i, 2);
			core_config_ice_mode_write(0x43004, otp_data[i - start], 1);
			ipio_info("Writing OTP data .... %ld%c", (((i - start) * 100) / w_len), '%');

			prog_one_done = 0x0;
			retry = 1000;

			do {
				prog_one_done = core_config_read_write_onebyte(0x4300C);
				retry--;
			} while (((prog_one_done & 0x40) != 0x40) && retry > 0);

			if (retry <= 0 && (prog_one_done & 0x40) != 0x40) {
				ipio_err("OTP: prog_one_done (0x%x) error, retry = %d\n", prog_one_done, retry);
				break;
			}
		}
	}

out:
	core_config_ice_mode_write(0x4300C, 0x04, 1);
	return ret;
}

void core_config_otp_copy_bank1_to_bank2(void)
{
	int i;
	int ret = 0;
	int otp_max_size = 512;
	u8 buf[512] = {0};//otp data
	u8 tmp[32] = {0};//mirror

	ret = core_config_ice_mode_enable();
	if (ret < 0) {
		ipio_err("ICE enable fail\n");
		goto out;
	}
	core_config_set_watch_dog(false);
	ret = core_config_before_write_otp();
	if (ret < 0) {
		ipio_err("OTP: write before failed\n");
		goto out;
	}

	core_config_otp_read(buf, otp_max_size, 0, otp_max_size);

	dump_data1(buf, 8, otp_max_size, 0, "Original OTP");

	ipio_info("OTP: copy bank 1 to tmp \n");
	for(i = 0; i < 32; i ++)
		tmp[i] = buf[i]; // copy 32byte of bank 1 to tmp buf

	ipio_info("OTP: write tmp to bank 2\n");
	//tmp[0] = 0xAA;
	//ret = core_config_otp_write_check(tmp, 0, 1);
	tmp[0] = 0xAA;
	ret = core_config_otp_write_check(tmp, 0x20, 32);
	if (ret < 0)
		ipio_err("OTP: write and check error\n");

	memset(buf, 0x0, otp_max_size);
	core_config_otp_read(buf, otp_max_size, 0, otp_max_size);

	dump_data1(buf, 8, otp_max_size, 0, "After write OTP");

	/* compare result */
	for(i = 0; i < 32; i ++) {
		ipio_info("tmp[%d] = %x, buf[32+%d] = %x\n", i, tmp[i], i, buf[32 + i]);
		if (tmp[i] != buf[32 + i]) {
			ipio_err("OTP: write OTP data error, index = %d, data = %x\n", i, buf[32 + i]);
		}
	}

	core_config_after_write_otp();
out:
	core_config_ice_mode_disable();
}
EXPORT_SYMBOL(core_config_otp_copy_bank1_to_bank2);

#endif
int core_config_set_watch_dog(bool enable)
{
	int timeout = 10, ret = 0;
	uint8_t off_bit = 0x5A, on_bit = 0xA5;
	uint8_t value_low = 0x0, value_high = 0x0;
	uint32_t wdt_addr = core_config->wdt_addr;

	if (wdt_addr <= 0 || core_config->chip_id <= 0) {
		ipio_err("WDT/CHIP ID is invalid\n");
		return -EINVAL;
	}

	/* Config register and values by IC */
	if (core_config->chip_id == CHIP_TYPE_ILI7807) {
		value_low = 0x07;
		value_high = 0x78;
	} else if (core_config->chip_id == CHIP_TYPE_ILI9881 ) {
		value_low = 0x81;
		value_high = 0x98;
	} else {
		ipio_err("Unknown CHIP type (0x%x)\n",core_config->chip_id);
		return -ENODEV;
	}

	if (enable) {
		core_config_ice_mode_write(wdt_addr, 1, 1);
	} else {
		core_config_ice_mode_write(wdt_addr, value_low, 1);
		core_config_ice_mode_write(wdt_addr, value_high, 1);
		/* need to delay 300us after stop mcu to wait fw relaod */
		udelay(300);
	}

	while (timeout > 0) {
		ret = core_config_ice_mode_read(0x51018);
		ipio_debug(DEBUG_CONFIG, "bit = %x\n", ret);

		if (enable) {
			if (CHECK_EQUAL(ret, on_bit) == 0)
				break;
		} else {
			if (CHECK_EQUAL(ret, off_bit) == 0)
				break;
		}

		timeout--;
		mdelay(10);
	}

	if (timeout > 0) {
		if (enable) {
			ipio_debug(DEBUG_CONFIG, "WDT turn on succeed\n");
		} else {
			core_config_ice_mode_write(wdt_addr, 0, 1);
			ipio_debug(DEBUG_CONFIG, "WDT turn off succeed\n");
		}
	} else {
		ili9881_read_pc_counter();
		ipio_err("WDT turn on/off timeout !\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(core_config_set_watch_dog);

int core_config_check_cdc_busy(int conut, int delay)
{
	int timer = conut, res = -1;
	uint8_t cmd[2] = { 0 };
	uint8_t busy[2] = { 0 };

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_cdc_busy;

	while (timer > 0) {
		mdelay(delay);
		core_write(core_config->slave_i2c_addr, cmd, 2);
		core_write(core_config->slave_i2c_addr, &cmd[1], 1);
		core_read(core_config->slave_i2c_addr, busy, 2);
		ipio_info("core_config_check_cdc_busy busy[0] = %x,busy[1] = %x\n", busy[0], busy[1]);
		if (core_fr->actual_fw_mode == P5_0_FIRMWARE_DEMO_MODE && busy[0] == 0x41) {
			ipio_debug(DEBUG_CONFIG, "Check busy is free\n");
			res = 0;
			break;
		}
		if (core_fr->actual_fw_mode == P5_0_FIRMWARE_TEST_MODE && busy[0] == 0x51) {
			ipio_debug(DEBUG_CONFIG, "Check busy is free\n");
			res = 0;
			break;
		}
		timer--;
	}

	if (res < -1) {
		ili9881_read_pc_counter();
		ipio_info("%d mode Check busy timeout !!\n",core_fr->actual_fw_mode);
	}
	return res;
}
EXPORT_SYMBOL(core_config_check_cdc_busy);

int core_config_check_int_status(bool high)
{
	int timer = 1000, res = -1;

	/* From FW request, timeout should at least be 5 sec */
	while (timer) {
		if(high) {
			if (gpio_get_value(ipd->int_gpio)) {
				ipio_debug(DEBUG_CONFIG, "Check busy is free\n");
				res = 0;
				break;
			}
		} else {
			if (!gpio_get_value(ipd->int_gpio)) {
				ipio_debug(DEBUG_CONFIG, "Check busy is free\n");
				res = 0;
				break;
			}
		}

		mdelay(5);
		timer--;
	}

	if (res < -1) {
		ili9881_read_pc_counter();
		ipio_info("Check busy timeout !!\n");
	}
	return res;
}
EXPORT_SYMBOL(core_config_check_int_status);

int core_config_get_project_id(uint8_t *pid_data)
{
	int i = 0, res = 0;
	uint32_t pid_addr = 0x1D000, pid_size = 10;

	res = core_config_ice_mode_enable();
	if (res < 0) {
		ipio_err("Failed to enter ICE mode, res = %d\n", res);
		return -1;
	}

	/* Disable watch dog */
	core_config_set_watch_dog(false);

	core_config_ice_mode_write(0x041000, 0x0, 1);   /* CS low */
	core_config_ice_mode_write(0x041004, 0x66aa55, 3);  /* Key */

	core_config_ice_mode_write(0x041008, 0x06, 1);
	core_config_ice_mode_write(0x041000, 0x01, 1);
	core_config_ice_mode_write(0x041000, 0x00, 1);
	core_config_ice_mode_write(0x041004, 0x66aa55, 3);  /* Key */
	core_config_ice_mode_write(0x041008, 0x03, 1);

	core_config_ice_mode_write(0x041008, (pid_addr & 0xFF0000) >> 16, 1);
	core_config_ice_mode_write(0x041008, (pid_addr & 0x00FF00) >> 8, 1);
	core_config_ice_mode_write(0x041008, (pid_addr & 0x0000FF), 1);

	for(i = 0; i < pid_size; i++) {
		core_config_ice_mode_write(0x041008, 0xFF, 1);
		pid_data[i] = core_config_ice_mode_read(0x41010);
		ipio_info("pid_data[%d] = 0x%x\n", i, pid_data[i]);
	}

	core_config_ice_mode_write(0x041010, 0x1, 0);   /* CS high */
	core_config_ic_reset();

	return res;
}
EXPORT_SYMBOL(core_config_get_project_id);
#if 0
int core_config_get_key_info(void)
{
	int res = 0, i;
	uint8_t cmd[2] = { 0 };

	memset(g_read_buf, 0, sizeof(g_read_buf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_key_info;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_read(core_config->slave_i2c_addr, &g_read_buf[0], protocol->key_info_len);
	if (res < 0) {
		ipio_err("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	if (core_config->tp_info->nKeyCount) {
		/* NOTE: Firmware not ready yet */
		core_config->tp_info->nKeyAreaXLength = (g_read_buf[0] << 8) + g_read_buf[1];
		core_config->tp_info->nKeyAreaYLength = (g_read_buf[2] << 8) + g_read_buf[3];

		ipio_info("key: length of X area = %x\n", core_config->tp_info->nKeyAreaXLength);
		ipio_info("key: length of Y area = %x\n", core_config->tp_info->nKeyAreaYLength);

		for (i = 0; i < core_config->tp_info->nKeyCount; i++) {
			core_config->tp_info->virtual_key[i].nId = g_read_buf[i * 5 + 4];
			core_config->tp_info->virtual_key[i].nX = (g_read_buf[i * 5 + 5] << 8) + g_read_buf[i * 5 + 6];
			core_config->tp_info->virtual_key[i].nY = (g_read_buf[i * 5 + 7] << 8) + g_read_buf[i * 5 + 8];
			core_config->tp_info->virtual_key[i].nStatus = 0;

			ipio_info("key: id = %d, X = %d, Y = %d\n", core_config->tp_info->virtual_key[i].nId,
				 core_config->tp_info->virtual_key[i].nX, core_config->tp_info->virtual_key[i].nY);
		}
	}

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_key_info);

#endif


int core_config_get_tp_info(void)
{
	int res = 0;
	uint8_t cmd[2] = { 0 };

	memset(g_read_buf, 0, sizeof(g_read_buf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_tp_info;

	core_config->tp_info->nMinX = minX;
	core_config->tp_info->nMinY = minY;
	core_config->tp_info->nMaxX = maxX;
	core_config->tp_info->nMaxY = maxY;
	core_config->tp_info->nXChannelNum = xchannel;
	core_config->tp_info->nYChannelNum = ychannel;
	core_config->tp_info->self_tx_channel_num = self_tx;
	core_config->tp_info->self_rx_channel_num = self_rx;
	core_config->tp_info->side_touch_type = dside_touch_type;
	core_config->tp_info->nMaxTouchNum = max_touch_num;
	core_config->tp_info->nKeyCount = max_key_num;

	core_config->tp_info->nMaxKeyButtonNum = 5;


	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}
	msleep(1);
	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}
	msleep(1);
	res = core_read(core_config->slave_i2c_addr, &g_read_buf[0], protocol->tp_info_len);
	if (res < 0) {
		ipio_err("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	/* in protocol v5, ignore the first btye because of a header. */
/*	core_config->tp_info->nMinX = g_read_buf[1];
	core_config->tp_info->nMinY = g_read_buf[2];
	core_config->tp_info->nMaxX = (g_read_buf[4] << 8) + g_read_buf[3];
	core_config->tp_info->nMaxY = (g_read_buf[6] << 8) + g_read_buf[5];
	core_config->tp_info->nXChannelNum = g_read_buf[7];
	core_config->tp_info->nYChannelNum = g_read_buf[8];
	core_config->tp_info->self_tx_channel_num = g_read_buf[11];
	core_config->tp_info->self_rx_channel_num = g_read_buf[12];
	core_config->tp_info->side_touch_type = g_read_buf[13];
	core_config->tp_info->nMaxTouchNum = g_read_buf[9];
	core_config->tp_info->nKeyCount = g_read_buf[10];
*/
	ipio_info("minX = %d, minY = %d, maxX = %d, maxY = %d\n",
		 core_config->tp_info->nMinX, core_config->tp_info->nMinY,
		 core_config->tp_info->nMaxX, core_config->tp_info->nMaxY);
	ipio_info("xchannel = %d, ychannel = %d, self_tx = %d, self_rx = %d\n",
		 core_config->tp_info->nXChannelNum, core_config->tp_info->nYChannelNum,
		 core_config->tp_info->self_tx_channel_num, core_config->tp_info->self_rx_channel_num);
	ipio_info("side_touch_type = %d, max_touch_num= %d, touch_key_num = %d, max_key_num = %d\n",
		 core_config->tp_info->side_touch_type, core_config->tp_info->nMaxTouchNum,
		 core_config->tp_info->nKeyCount, core_config->tp_info->nMaxKeyButtonNum);

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_tp_info);

int core_config_get_protocol_ver(void)
{
	int res = 0, i = 0;
	int major, mid, minor;
	uint8_t cmd[2] = { 0 };

	memset(g_read_buf, 0, sizeof(g_read_buf));
	memset(core_config->protocol_ver, 0x0, sizeof(core_config->protocol_ver));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_pro_ver;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_read(core_config->slave_i2c_addr, &g_read_buf[0], protocol->pro_ver_len);
	if (res < 0) {
		ipio_err("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	/* ignore the first btye because of a header. */
	for (; i < protocol->pro_ver_len - 1; i++) {
		core_config->protocol_ver[i] = g_read_buf[i + 1];
	}

	ipio_info("Procotol Version = %d.%d.%d\n",
		 core_config->protocol_ver[0], core_config->protocol_ver[1], core_config->protocol_ver[2]);

	major = core_config->protocol_ver[0];
	mid = core_config->protocol_ver[1];
	minor = core_config->protocol_ver[2];

	/* update protocol if they're different with the default ver set by driver */
	if (major != PROTOCOL_MAJOR || mid != PROTOCOL_MID || minor != PROTOCOL_MINOR) {
		res = core_protocol_update_ver(major, mid, minor);
		if (res < 0)
			ipio_err("Protocol version is invalid\n");
	}

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_protocol_ver);

int core_config_get_core_ver(void)
{
	int res = 0, i = 0;
	uint8_t cmd[2] = { 0 };

	memset(g_read_buf, 0, sizeof(g_read_buf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_core_ver;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_read(core_config->slave_i2c_addr, &g_read_buf[0], protocol->core_ver_len);
	if (res < 0) {
		ipio_err("Failed to read data via I2C, %d\n", res);
		goto out;
	}

	for (; i < protocol->core_ver_len - 1; i++)
		core_config->core_ver[i] = g_read_buf[i + 1];

	/* in protocol v5, ignore the first btye because of a header. */
	ipio_info("Core Version = %d.%d.%d.%d\n",
		 core_config->core_ver[1], core_config->core_ver[2],
		 core_config->core_ver[3], core_config->core_ver[4]);

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_core_ver);

/*
 * Getting the version of firmware used on the current one.
 *
 */
int core_config_get_fw_ver(void)
{
	int res = 0, i = 0;
	uint8_t cmd[2] = { 0 };
	memset(g_read_buf, 0, sizeof(g_read_buf));

	cmd[0] = protocol->cmd_read_ctrl;
	cmd[1] = protocol->cmd_get_fw_ver;

	res = core_write(core_config->slave_i2c_addr, cmd, 2);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_write(core_config->slave_i2c_addr, &cmd[1], 1);
	if (res < 0) {
		ipio_err("Failed to write data via I2C, %d\n", res);
		goto out;
	}

	msleep(1);

	res = core_read(core_config->slave_i2c_addr, &g_read_buf[0], protocol->fw_ver_len);
	if (res < 0) {
		ipio_err("Failed to read fw version %d\n", res);
		goto out;
	}

	for (; i < protocol->fw_ver_len; i++)
		core_config->firmware_ver[i] = g_read_buf[i];

	if (protocol->mid >= 0x3) {
		ipio_info("Firmware Version = %d.%d.%d.%d\n", core_config->firmware_ver[1], core_config->firmware_ver[2], core_config->firmware_ver[3], core_config->firmware_ver[4]);
	} else {
		ipio_info("Firmware Version = %d.%d.%d\n",
			core_config->firmware_ver[1], core_config->firmware_ver[2], core_config->firmware_ver[3]);
	}
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Function.1372106,2018/5/21,Add for ctp hardware info for OPPO
	if ( ili_ctpmodule == 0 ) {
	    sprintf(ili_version,"Txd_Ili_%x",core_config->firmware_ver[3]);
	    devinfo_info_tp_set(ili_version, "TXD",OPPO_FIRMWARE_NAME_PATH);
	} else {
	    sprintf(ili_version,"Hlt_Ili_%x",core_config->firmware_ver[3]);
	    devinfo_info_tp_set(ili_version, "HLT",OPPO_FIRMWARE_NAME_PATH);
	}
#endif

out:
	return res;
}
EXPORT_SYMBOL(core_config_get_fw_ver);

int core_config_get_chip_id(void)
{
	int res = 0;
	static int do_once = 0;
	uint32_t RealID = 0, PIDData = 0, otp_id = 0, ana_id = 0;

	res = core_config_ice_mode_enable();
	if (res < 0) {
		ipio_err("Failed to enter ICE mode, res = %d\n", res);
		goto out;
	}

	msleep(20);

	PIDData = core_config_ice_mode_read(core_config->pid_addr);
	otp_id = core_config_ice_mode_read(core_config->otp_id_addr);
	ana_id = core_config_ice_mode_read(core_config->ana_id_addr);
	core_config->chip_pid = PIDData;
	core_config->core_type = PIDData & 0xFF;
	core_config->chip_otp_id = otp_id & 0xFF;
	core_config->chip_ana_id = ana_id & 0xFF;
	ipio_info("PID = 0x%x, Core type = 0x%x\n",
		core_config->chip_pid, core_config->core_type);
	ipio_info("otp_id = 0x%x, ana_id = 0x%x\n",
		core_config->chip_otp_id, core_config->chip_ana_id);

	if (PIDData) {
		RealID = check_chip_id(PIDData);
		if (RealID != core_config->chip_id) {
			ipio_err("CHIP ID ERROR: 0x%x, TP_TOUCH_IC = 0x%x\n", RealID, TP_TOUCH_IC);
			res = -ENODEV;
			goto out;
		}
	} else {
		ipio_err("PID DATA error : 0x%x\n", PIDData);
		res = -EINVAL;
		goto out;
	}

	if (do_once == 0) {
		/* reading flash id needs to let ic entry to ICE mode */
		read_flash_info(0x9F, 4);
		do_once = 1;
	}

out:
	core_config_ic_reset();
	msleep(150);
	return res;
}
EXPORT_SYMBOL(core_config_get_chip_id);

int core_config_init(void)
{
	int i = 0;

	core_config = kzalloc(sizeof(*core_config) * sizeof(uint8_t) * 6, GFP_KERNEL);
	if (ERR_ALLOC_MEM(core_config)) {
		ipio_err("Failed to allocate core_config mem, %ld\n", PTR_ERR(core_config));
		core_config_remove();
		return -ENOMEM;
	}

	core_config->tp_info = kzalloc(sizeof(*core_config->tp_info), GFP_KERNEL);
	if (ERR_ALLOC_MEM(core_config->tp_info)) {
		ipio_err("Failed to allocate core_config->tp_info mem, %ld\n", PTR_ERR(core_config->tp_info));
		core_config_remove();
		return -ENOMEM;
	}
#ifdef ILITEK_GESTURE_WAKEUP
	gesture_report_data = kzalloc(sizeof(struct ili_gesture_info), GFP_KERNEL);
	if (ERR_ALLOC_MEM(gesture_report_data)) {
		ipio_err("Failed to allocate gesture_report_data mem, %ld\n", PTR_ERR(gesture_report_data));
		core_config_remove();
		return -ENOMEM;
	}
#endif
	for (; i < ARRAY_SIZE(ipio_chip_list); i++) {
		if (ipio_chip_list[i] == TP_TOUCH_IC) {
			core_config->chip_id = ipio_chip_list[i];
			core_config->chip_type = 0x0000;

			core_config->do_ic_reset = false;
			core_config->isEnableGesture = false;
			core_config->gesture_backup = false;
			core_config->gameSwitch = false;

			if (core_config->chip_id == CHIP_TYPE_ILI7807) {
				core_config->slave_i2c_addr = ILI7807_SLAVE_ADDR;
				core_config->ice_mode_addr = ILI7807_ICE_MODE_ADDR;
				core_config->pid_addr = ILI7807_PID_ADDR;
				core_config->wdt_addr = ILI7808_WDT_ADDR;
			} else if (core_config->chip_id == CHIP_TYPE_ILI9881) {
				core_config->slave_i2c_addr = ILI9881_SLAVE_ADDR;
				core_config->ice_mode_addr = ILI9881_ICE_MODE_ADDR;
				core_config->pid_addr = ILI9881_PID_ADDR;
				core_config->wdt_addr = ILI9881_WDT_ADDR;
				core_config->otp_id_addr = ILI9881_OTP_ID_ADDR;
				core_config->ana_id_addr = ILI9881_ANA_ID_ADDR;
			}
			return 0;
		}
	}

	ipio_err("Can't find this chip in support list\n");
	return 0;
}
EXPORT_SYMBOL(core_config_init);

void core_config_remove(void)
{
	ipio_info("Remove core-config memebers\n");

	if (core_config != NULL) {
		ipio_kfree((void **)&core_config->tp_info);
		ipio_kfree((void **)&core_config);
	}
}
EXPORT_SYMBOL(core_config_remove);
