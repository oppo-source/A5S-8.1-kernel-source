/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - gesture.c
** Description : This program is for ili9881 driver gesture.c
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
#include "finger_report.h"
#include "gesture.h"
#include "protocol.h"
#include "config.h"
#include "core/firmware.h"
#include "../platform.h"

struct core_gesture_data *core_gesture;
extern uint8_t ap_fw[MAX_AP_FIRMWARE_SIZE];
#ifdef ILITEK_GESTURE_WAKEUP


extern uint8_t buf_gesture[GESTURE_INFO_LENGTH+1];

struct ili_gesture_info * gesture_report_data;

int ilitek_get_gesture_info(struct ili_gesture_info * gesture)
{
    uint8_t gesture_id = 0, score = 0;
	int lu_x = 0, lu_y = 0, rd_x = 0, rd_y = 0;
    uint8_t point_data[GESTURE_INFO_LENGTH + 1] = {0};
	uint8_t i =0;
	for(i=0;i<GESTURE_INFO_LENGTH;i++)
	{
		point_data[i] = buf_gesture[i];
	}

    gesture_id = (uint8_t)(point_data[1]);
	score = point_data[36];
    switch (gesture_id)     //judge gesture type
    {
        case GESTURE_RIGHT :
            gesture->gesture_type  = Left2RightSwip;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));
            break;

        case GESTURE_LEFT :
            gesture->gesture_type  = Right2LeftSwip;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));
            break;

        case GESTURE_DOWN  :
            gesture->gesture_type  = Up2DownSwip;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));
            break;

        case GESTURE_UP :
            gesture->gesture_type  = Down2UpSwip;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));
            break;

        case GESTURE_DOUBLECLICK:
            gesture->gesture_type  = DouTap;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end     = gesture->Point_start;
            break;

        case GESTURE_V :
            gesture->gesture_type  = UpVee;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));
            gesture->Point_1st.x   = (((point_data[16] & 0xF0) << 4) | (point_data[17]));
            gesture->Point_1st.y   = (((point_data[16] & 0x0F) << 8) | (point_data[18]));
            break;

        case GESTURE_V_DOWN :
            gesture->gesture_type  = DownVee;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));
            gesture->Point_1st.x   = (((point_data[16] & 0xF0) << 4) | (point_data[17]));
            gesture->Point_1st.y   = (((point_data[16] & 0x0F) << 8) | (point_data[18]));
            break;

        case GESTURE_V_LEFT :
            gesture->gesture_type  = LeftVee;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));
            gesture->Point_1st.x   = (((point_data[16] & 0xF0) << 4) | (point_data[17]));
            gesture->Point_1st.y   = (((point_data[16] & 0x0F) << 8) | (point_data[18]));
            break;

        case GESTURE_V_RIGHT :
            gesture->gesture_type  = RightVee;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));
            gesture->Point_1st.x   = (((point_data[16] & 0xF0) << 4) | (point_data[17]));
            gesture->Point_1st.y   = (((point_data[16] & 0x0F) << 8) | (point_data[18]));
            break;

        case GESTURE_O  :
            gesture->gesture_type = Circle;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->clockwise = point_data[34];
            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));

			lu_x = (((point_data[28] & 0xF0) << 4) | (point_data[29]));
			lu_y = (((point_data[28] & 0x0F) << 8) | (point_data[30]));
			rd_x = (((point_data[31] & 0xF0) << 4) | (point_data[32]));
			rd_y = (((point_data[31] & 0x0F) << 8) | (point_data[33]));

            gesture->Point_1st.x   = ((rd_x + lu_x) / 2);  //ymain
            gesture->Point_1st.y   = lu_y;
            gesture->Point_2nd.x   = lu_x;  //xmin
            gesture->Point_2nd.y   = ((rd_y + lu_y) / 2);
            gesture->Point_3rd.x   = ((rd_x + lu_x) / 2);  //ymax
            gesture->Point_3rd.y   = rd_y;
            gesture->Point_4th.x   = rd_x;  //xmax
            gesture->Point_4th.y   = ((rd_y + lu_y) / 2);
            break;

        case GESTURE_M  :
            gesture->gesture_type  = Mgestrue;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));

            gesture->Point_1st.x   = (((point_data[16] & 0xF0) << 4) | (point_data[17]));
            gesture->Point_1st.y   = (((point_data[16] & 0x0F) << 8) | (point_data[18]));
            gesture->Point_2nd.x   = (((point_data[19] & 0xF0) << 4) | (point_data[20]));  //xmin
            gesture->Point_2nd.y   = (((point_data[19] & 0x0F) << 8) | (point_data[21]));
            gesture->Point_3rd.x   = (((point_data[22] & 0xF0) << 4) | (point_data[23]));  //ymax
            gesture->Point_3rd.y   = (((point_data[22] & 0x0F) << 8) | (point_data[24]));
            break;

        case GESTURE_W :
            gesture->gesture_type  = Wgestrue;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));

            gesture->Point_1st.x   = (((point_data[16] & 0xF0) << 4) | (point_data[17]));
            gesture->Point_1st.y   = (((point_data[16] & 0x0F) << 8) | (point_data[18]));
            gesture->Point_2nd.x   = (((point_data[19] & 0xF0) << 4) | (point_data[20]));  //xmin
            gesture->Point_2nd.y   = (((point_data[19] & 0x0F) << 8) | (point_data[21]));
            gesture->Point_3rd.x   = (((point_data[22] & 0xF0) << 4) | (point_data[23]));  //ymax
            gesture->Point_3rd.y   = (((point_data[22] & 0x0F) << 8) | (point_data[24]));
            break;

		case GESTURE_TWOLINE_DOWN :
            gesture->gesture_type  = DouSwip;

			gesture->Point_1st.x   = 0;  //ymain
            gesture->Point_1st.y   = 0;
            gesture->Point_2nd.x   = 0;  //xmin
            gesture->Point_2nd.y   = 0;
            gesture->Point_3rd.x   = 0;  //ymax
            gesture->Point_3rd.y   = 0;
            gesture->Point_4th.x   = 0;  //xmax
            gesture->Point_4th.y   = 0;
			gesture->clockwise     = 1;

            gesture->Point_start.x = (((point_data[4] & 0xF0) << 4) | (point_data[5]));
            gesture->Point_start.y = (((point_data[4] & 0x0F) << 8) | (point_data[6]));
            gesture->Point_end.x   = (((point_data[7] & 0xF0) << 4) | (point_data[8]));
            gesture->Point_end.y   = (((point_data[7] & 0x0F) << 8) | (point_data[9]));

            gesture->Point_1st.x   = (((point_data[10] & 0xF0) << 4) | (point_data[11]));
            gesture->Point_1st.y   = (((point_data[10] & 0x0F) << 8) | (point_data[12]));
            gesture->Point_2nd.x   = (((point_data[13] & 0xF0) << 4) | (point_data[14]));
            gesture->Point_2nd.y   = (((point_data[13] & 0x0F) << 8) | (point_data[15]));
            break;

        default:
            gesture->gesture_type = UnkownGesture;
            break;
    }
 	ipio_debug(DEBUG_FINGER_REPORT, "gesture data 0-17 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", \
		point_data[0], point_data[1], point_data[2], point_data[3], point_data[4], point_data[5], point_data[6], point_data[7], point_data[8], \
		point_data[9], point_data[10], point_data[11], point_data[12], point_data[13], point_data[14], point_data[15], point_data[16], point_data[17]);

	ipio_debug(DEBUG_FINGER_REPORT, "gesture data 18-35 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", \
		point_data[18], point_data[19], point_data[20], point_data[21], point_data[22], point_data[23], point_data[24], point_data[25], point_data[26], \
		point_data[27], point_data[28], point_data[29], point_data[30], point_data[31], point_data[32], point_data[33], point_data[34], point_data[35]);

	ipio_info("gesture debug data 160-168 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", \
		point_data[160], point_data[161], point_data[162], point_data[163], point_data[164], point_data[165], point_data[166], point_data[167], point_data[168]);

	ipio_debug(DEBUG_FINGER_REPORT, "before scale gesture_id: 0x%x, score: %d, gesture_type: %d, clockwise: %d, points: (%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)\n", \
                gesture_id, score, gesture->gesture_type, gesture->clockwise, \
                gesture->Point_start.x, gesture->Point_start.y, \
                gesture->Point_end.x, gesture->Point_end.y, \
                gesture->Point_1st.x, gesture->Point_1st.y, \
                gesture->Point_2nd.x, gesture->Point_2nd.y, \
                gesture->Point_3rd.x, gesture->Point_3rd.y, \
                gesture->Point_4th.x, gesture->Point_4th.y);

    if (!core_fr->isSetResolution) {
		gesture->Point_start.x = gesture->Point_start.x * TOUCH_SCREEN_X_MAX / TPD_WIDTH;
		gesture->Point_start.y = gesture->Point_start.y * TOUCH_SCREEN_Y_MAX / TPD_HEIGHT;
		gesture->Point_end.x = gesture->Point_end.x * TOUCH_SCREEN_X_MAX / TPD_WIDTH;
		gesture->Point_end.y = gesture->Point_end.y * TOUCH_SCREEN_Y_MAX / TPD_HEIGHT;
		gesture->Point_1st.x = gesture->Point_1st.x * TOUCH_SCREEN_X_MAX / TPD_WIDTH;
		gesture->Point_1st.y = gesture->Point_1st.y * TOUCH_SCREEN_Y_MAX / TPD_HEIGHT;

		gesture->Point_2nd.x = gesture->Point_2nd.x * TOUCH_SCREEN_X_MAX / TPD_WIDTH;
		gesture->Point_2nd.y = gesture->Point_2nd.y * TOUCH_SCREEN_Y_MAX / TPD_HEIGHT;

		gesture->Point_3rd.x = gesture->Point_3rd.x * TOUCH_SCREEN_X_MAX / TPD_WIDTH;
		gesture->Point_3rd.y = gesture->Point_3rd.y * TOUCH_SCREEN_Y_MAX / TPD_HEIGHT;

		gesture->Point_4th.x = gesture->Point_4th.x * TOUCH_SCREEN_X_MAX / TPD_WIDTH;
		gesture->Point_4th.y = gesture->Point_4th.y * TOUCH_SCREEN_Y_MAX / TPD_HEIGHT;
	}
    ipio_info("gesture_id: 0x%x, score: %d, gesture_type: %d, clockwise: %d, points: (%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)\n", \
                gesture_id, score, gesture->gesture_type, gesture->clockwise, \
                gesture->Point_start.x, gesture->Point_start.y, \
                gesture->Point_end.x, gesture->Point_end.y, \
                gesture->Point_1st.x, gesture->Point_1st.y, \
                gesture->Point_2nd.x, gesture->Point_2nd.y, \
                gesture->Point_3rd.x, gesture->Point_3rd.y, \
                gesture->Point_4th.x, gesture->Point_4th.y);
		if (gesture->gesture_type == UnkownGesture) {
			return -1;
		}

    return 1;
}

EXPORT_SYMBOL(ilitek_get_gesture_info);

#endif
int core_load_gesture_code(void)
{
	int res = 0, i = 0;
	uint8_t temp[64] = {0};
	//uint32_t gesture_end_addr = 0, gesture_start_addr = 0, ap_start_addr = 0, ap_end_addr = 0, area = 0;
	core_gesture->entry = true;
	//read loader info
	// temp[0] = 0x01;
	// temp[1] = 0x0A;
	// temp[2] = 0x07;
	// if ((core_write(core_config->slave_i2c_addr, temp, 3)) < 0) {
	// 	ipio_err("write command error\n");
	// }
	// if ((core_read(core_config->slave_i2c_addr, temp, 20)) < 0) {
	// 	ipio_err("Read command error\n");
	// }
	// for(i = 0; i < 20; i++)
	// {
	// 	printk("0x%x,", temp[i]);
	// }
	// printk("\n");
	/*core_gesture->ap_start_addr = (ap_fw[0xFFD3] << 24) + (ap_fw[0xFFD2] << 16) + (ap_fw[0xFFD1] << 8) + ap_fw[0xFFD0];
	core_gesture->ap_length = MAX_GESTURE_FIRMWARE_SIZE;
	core_gesture->start_addr = (ap_fw[0xFFDB] << 24) + (ap_fw[0xFFDA] << 16) + (ap_fw[0xFFD9] << 8) + ap_fw[0xFFD8];
	core_gesture->length = MAX_GESTURE_FIRMWARE_SIZE;
	core_gesture->area_section = (ap_fw[0xFFCF] << 24) + (ap_fw[0xFFCE] << 16) + (ap_fw[0xFFCD] << 8) + ap_fw[0xFFCC];*/
	//write load gesture flag
	temp[0] = 0x01;
	temp[1] = 0x0A;
	temp[2] = 0x03;
	if ((core_write(core_config->slave_i2c_addr, temp, 3)) < 0) {
		ipio_err("write command error\n");
	}
	temp[0] = 0x01;
	temp[1] = 0x0A;
	temp[2] = core_gesture->mode + 1;
	if ((core_write(core_config->slave_i2c_addr, temp, 3)) < 0) {
		ipio_err("write command error\n");
	}
	for(i = 0; i < 3; i++)
	{
		mdelay(50);
		temp[0] = 0x01;
		temp[1] = 0x0A;
		temp[2] = 0x05;
		if ((core_write(core_config->slave_i2c_addr, temp, 3)) < 0) {
			ipio_err("write command error\n");
		}
		if ((core_read(core_config->slave_i2c_addr, temp, 1)) < 0) {
			ipio_err("Read command error\n");
		}
		if(temp[0] == 0x91)
		{
			ipio_debug(DEBUG_GESTURE, "check fw ready\n");
			break;
		}
		if(i==2){
			ipio_info("ilitek_esd_gesture_reset i==2 end");
			ilitek_esd_gesture_reset();

			return 0;
		}
	}
	if(i == 3 && temp[0] != 0x01)
			ipio_err("FW is busy, error\n");

	//load gesture code
	if (core_config_ice_mode_enable() < 0) {
		ipio_err("Failed to enter ICE mode\n");
		return -1;
	}
	//host_download(true);
	core_firmware_upgrade(UPDATE_FW_PATH, true);
	ipio_debug(DEBUG_GESTURE, "gesture_start_addr = 0x%x, gesture_end_addr = 0x%x\n", core_gesture->start_addr, core_gesture->length);
	ipio_debug(DEBUG_GESTURE, "area = %d, ap_start_addr = 0x%x, ap_end_addr = 0x%x\n", core_gesture->area_section, core_gesture->ap_start_addr, core_gesture->ap_length);
	temp[0] = 0x01;
	temp[1] = 0x0A;
	temp[2] = 0x06;
	if ((core_write(core_config->slave_i2c_addr, temp, 3)) < 0) {
		ipio_err("write command error\n");
	}
	return res;
}

int core_load_ap_code(void)
{
	int res = 0, i = 0;
	uint8_t temp[64] = {0};
	uint32_t gesture_end_addr = 0, gesture_start_addr = 0, ap_start_addr = 0, ap_end_addr = 0, area = 0;
	core_gesture->entry = true;
	//Write Load AP Flag
	temp[0] = 0x01;
	temp[1] = 0x01;
	temp[2] = 0x00;
	if ((core_write(core_config->slave_i2c_addr, temp, 3)) < 0) {
		ipio_err("write command error\n");
	}
	if ((core_read(core_config->slave_i2c_addr, temp, 20)) < 0) {
		ipio_err("Read command error\n");
	}
	area = (temp[0] << 24) + (temp[1] << 16) + (temp[2] << 8) + temp[3];
	ap_start_addr = (temp[4] << 24) + (temp[5] << 16) + (temp[6] << 8) + temp[7];
	ap_end_addr = (temp[8] << 24) + (temp[9] << 16) + (temp[10] << 8) + temp[11];
	gesture_start_addr = (temp[12] << 24) + (temp[13] << 16) + (temp[14] << 8) + temp[15];
	gesture_end_addr = (temp[16] << 24) + (temp[17] << 16) + (temp[18] << 8) + temp[19];
	ipio_debug(DEBUG_FIRMWARE, "gesture_start_addr = 0x%x, gesture_end_addr = 0x%x\n", gesture_start_addr, gesture_end_addr);
	ipio_debug(DEBUG_FIRMWARE, "area = %d, ap_start_addr = 0x%x, ap_end_addr = 0x%x\n", area, ap_start_addr, ap_end_addr);
	//Leave Gesture Cmd LPWG Stop
	temp[0] = 0x01;
	temp[1] = 0x0A;
	temp[2] = 0x00;
	if ((core_write(core_config->slave_i2c_addr, temp, 3)) < 0) {
		ipio_err("write command error\n");
	}
	for(i = 0; i < 20; i++)
	{
		mdelay(i*100+100);
		temp[0] = 0x01;
		temp[1] = 0x0A;
		temp[2] = 0x05;
		if ((core_write(core_config->slave_i2c_addr, temp, 3)) < 0) {
			ipio_err("write command error\n");
		}
		if ((core_read(core_config->slave_i2c_addr, temp, 1)) < 0) {
			ipio_err("Read command error\n");
		}
		if(temp[0] == 0x91)
		{
			ipio_debug(DEBUG_FIRMWARE, "check fw ready\n");
			break;
		}
	}
	if(i == 3 && temp[0] != 0x01)
			ipio_err("FW is busy, error\n");

	//load AP code
	if (core_config_ice_mode_enable() < 0) {
		ipio_err("Failed to enter ICE mode\n");
		return -1;
	}
	res = host_download(false);

	temp[0] = 0x01;
	temp[1] = 0x0A;
	temp[2] = 0x06;
	if ((core_write(core_config->slave_i2c_addr, temp, 3)) < 0) {
		ipio_err("write command error\n");
	}
	core_gesture->entry = false;
	return res;
}
int core_gesture_key(uint8_t gdata)
{
	int gcode;

	switch (gdata) {
	case GESTURE_LEFT:
		gcode = KEY_GESTURE_LEFT;
		break;
	case GESTURE_RIGHT:
		gcode = KEY_GESTURE_RIGHT;
		break;
	case GESTURE_UP:
		gcode = KEY_GESTURE_UP;
		break;
	case GESTURE_DOWN:
		gcode = KEY_GESTURE_DOWN;
		break;
	case GESTURE_DOUBLECLICK:
		gcode = KEY_GESTURE_D;
		break;
	case GESTURE_O:
		gcode = KEY_GESTURE_O;
		break;
	case GESTURE_W:
		gcode = KEY_GESTURE_W;
		break;
	case GESTURE_M:
		gcode = KEY_GESTURE_M;
		break;
	case GESTURE_E:
		gcode = KEY_GESTURE_E;
		break;
	case GESTURE_S:
		gcode = KEY_GESTURE_S;
		break;
	case GESTURE_V:
		gcode = KEY_GESTURE_V;
		break;
	case GESTURE_Z:
		gcode = KEY_GESTURE_Z;
		break;
	case GESTURE_C:
		gcode = KEY_GESTURE_C;
		break;
	default:
		gcode = -1;
		break;
	}

	ipio_debug(DEBUG_GESTURE, "gcode = %d\n", gcode);
	return gcode;
}
EXPORT_SYMBOL(core_gesture_key);

void core_gesture_remove(void)
{
	ipio_info("Remove core-gesture members\n");
	ipio_kfree((void **)&core_gesture);
}
EXPORT_SYMBOL(core_gesture_remove);

void core_gesture_init(struct core_fr_data *fr_data)
{
	struct input_dev *input_dev = fr_data->input_device;

	if (input_dev != NULL) {
		input_set_capability(input_dev, EV_KEY, KEY_POWER);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
		input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);
		input_set_capability(input_dev, EV_KEY, KEY_F4);

		__set_bit(KEY_POWER, input_dev->keybit);
		__set_bit(KEY_GESTURE_UP, input_dev->keybit);
		__set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
		__set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
		__set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
		__set_bit(KEY_GESTURE_O, input_dev->keybit);
		__set_bit(KEY_GESTURE_E, input_dev->keybit);
		__set_bit(KEY_GESTURE_M, input_dev->keybit);
		__set_bit(KEY_GESTURE_W, input_dev->keybit);
		__set_bit(KEY_GESTURE_S, input_dev->keybit);
		__set_bit(KEY_GESTURE_V, input_dev->keybit);
		__set_bit(KEY_GESTURE_Z, input_dev->keybit);
		__set_bit(KEY_GESTURE_C, input_dev->keybit);
		return;
	}

	ipio_err("GESTURE: input dev is NULL\n");
}
EXPORT_SYMBOL(core_gesture_init);
