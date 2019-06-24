/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - firmware.h
** Description : This program is for ili9881 driver firmware.h
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#ifndef __FIRMWARE_H
#define __FIRMWARE_H

struct core_firmware_data {
	uint8_t new_fw_ver[4];
	uint8_t old_fw_ver[4];

	uint32_t start_addr;
	uint32_t end_addr;
	uint32_t checksum;
	uint32_t crc32;

	uint32_t update_status;
	uint32_t max_count;

	int delay_after_upgrade;

	bool isUpgrading;
	bool isCRC;
	bool isboot;
	bool hasBlockInfo;

	int (*upgrade_func)(bool isIRAM);
	const struct firmware *fw;
	int common_reset;
/*2018-8-27 begin*/
	int esd_fail_enter_gesture;
/*2018-8-27 end */

};

extern struct core_firmware_data *core_firmware;

#ifdef BOOT_FW_UPGRADE
extern int core_firmware_boot_upgrade(void);
#endif
/* extern int core_firmware_iram_upgrade(const char* fpath); */
extern int core_firmware_upgrade(const char *, bool isIRAM);
extern int core_firmware_init(void);
extern void core_firmware_remove(void);

#endif /* __FIRMWARE_H */
