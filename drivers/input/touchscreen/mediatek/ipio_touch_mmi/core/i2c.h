/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - i2c.h
** Description : This program is for ili9881 driver i2c.h
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#ifndef __I2C_H
#define __I2C_H

struct core_i2c_data {
	struct i2c_client *client;
	int clk;
	int seg_len;
};

extern struct core_i2c_data *core_i2c;

extern int core_i2c_write(uint8_t, uint8_t *, uint16_t);
extern int core_i2c_read(uint8_t, uint8_t *, uint16_t);

extern int core_i2c_segmental_read(uint8_t, uint8_t *, uint16_t);

extern int core_i2c_init(struct i2c_client *);
extern void core_i2c_remove(void);

#endif
