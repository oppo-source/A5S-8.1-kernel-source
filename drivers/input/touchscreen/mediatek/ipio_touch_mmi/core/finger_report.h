/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - finger_report.h
** Description : This program is for ili9881 driver finger_report.c
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#ifndef __FINGER_REPORT_H
#define __FINGER_REPORT_H
typedef enum {
    ILITEK_AREA_NORMAL,     /*When Phone Face you in portrait top right corner*/
	ILITEK_AREA_CORNER, 	 /*When Phone Face you in portrait top left corner*/
	ILITEK_AREA_DEFAULT,
}ilitek_area;

struct core_fr_data {
	struct input_dev *input_device;

	/* the default of finger report is enabled */
	bool isEnableFR;
	/* used to send finger report packet to user psace */
	bool isEnableNetlink;
	/* allow input dev to report the value of physical touch */
	bool isEnablePressure;
	/* get screen resloution from fw if it's true */
	bool isSetResolution;
	/* used to change I2C Uart Mode when fw mode is in this mode */
	uint8_t i2cuart_mode;

	/* current firmware mode in driver */
	uint16_t actual_fw_mode;

	/* mutual firmware info */
	uint16_t log_packet_length;
	uint8_t log_packet_header;
	uint8_t type;
	uint8_t Mx;
	uint8_t My;
	uint8_t Sd;
	uint8_t Ss;
};
#ifdef ILITEK_EDGE_LIMIT
struct ilitek_limit_window{
	uint8_t limit_area;
	uint16_t left_x1;
	uint16_t left_x2;
	uint16_t right_x1;
	uint16_t right_x2;
	uint16_t left_y1;
	uint16_t left_y2;
	uint16_t right_y1;
	uint16_t right_y2;
	ilitek_area in_which_area;

};

struct ilitek_limit_data{
	uint8_t limit_data;
	uint8_t limit_edge_enable;
	uint8_t limit_corner_lu_enable;
	uint8_t limit_corner_ld_enable;
	uint8_t limit_corner_ru_enable;
	uint8_t limit_corner_rd_enable;
	struct ilitek_limit_window edge_limit;
};

#endif
extern struct core_fr_data *core_fr;

extern uint8_t core_fr_calc_checksum(uint8_t *pMsg, uint32_t nLength);
extern void core_fr_touch_press(int32_t x, int32_t y, int32_t w, uint32_t pressure, int32_t id);
extern void core_fr_touch_release(int32_t x, int32_t y, int32_t id);
extern void core_fr_touch_all_release(struct core_fr_data *core_fr);
extern void core_fr_mode_control(uint8_t *from_user);
extern void core_fr_handler(void);
extern void core_fr_input_set_param(struct input_dev *input_device);
extern int core_fr_init(void);
extern void core_fr_remove(void);

#endif /* __FINGER_REPORT_H */
