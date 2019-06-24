/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - flash.h
** Description : This program is for ili9881 driver flash.h
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#ifndef __FLASH_H
#define __FLASH_H

struct flash_table {
	uint16_t mid;
	uint16_t dev_id;
	int mem_size;
	int program_page;
	int sector;
	int block;
};

extern struct flash_table *flashtab;

extern int core_flash_poll_busy(void);
extern int core_flash_write_enable(void);
extern void core_flash_enable_protect(bool status);
extern void core_flash_init(uint16_t mid, uint16_t did);
extern void core_flash_remove(void);

#endif /* __FLASH_H */
