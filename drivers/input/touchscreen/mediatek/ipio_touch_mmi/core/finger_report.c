/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - finger_report.c
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
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/i2c.h>
#include <linux/list.h>

#include "../common.h"
#include "../platform.h"
#include "config.h"
#include "i2c.h"
#include "finger_report.h"
#include "gesture.h"
#include "mp_test.h"
#include "protocol.h"
/*****************MING***************/
#ifdef ODM_WT_EDIT
extern uint32_t buf_rawdata[576];
extern uint32_t buf_delta[576];
extern uint8_t buf_gesture[GESTURE_INFO_LENGTH+1];
extern int ilitek_get_gesture_info(struct ili_gesture_info * gesture);
extern void core_gesture_init(struct core_fr_data *fr_data);
extern int core_spi_check_read_size(void);
extern int core_spi_read_data_after_checksize(uint8_t *pBuf, uint16_t nSize);

extern struct ili_gesture_info * gesture_report_data;

#endif
//extern uint16_t buf_gesture[10];

/* An id with position in each fingers */
struct mutual_touch_point {
	uint16_t id;
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t pressure;
#ifdef ILITEK_EDGE_LIMIT
	ilitek_area type;
#endif
};

/* Keys and code with each fingers */
struct mutual_touch_info {
	uint8_t touch_num;
	uint8_t key_code;
	struct mutual_touch_point mtp[10];

};

/* Store the packet of finger report */
struct fr_data_node {
	uint8_t *data;
	uint16_t len;
};
/* record the status of touch being pressed or released currently and previosuly */
uint8_t g_current_touch[MAX_TOUCH_NUM];
uint8_t g_previous_touch[MAX_TOUCH_NUM];
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
bool b_first_point =true;
uint32_t last_nX = 0, last_nY = 0;
#endif
/* the total length of finger report packet */
int g_total_len = 0;

struct mutual_touch_info g_mutual_data;
struct fr_data_node *g_fr_node = NULL, *g_fr_uart = NULL;
struct core_fr_data *core_fr = NULL;
#if 1
#ifdef ILITEK_EDGE_LIMIT
typedef enum {
    ILITEK_CORNER_TOPLEFT,      /*When Phone Face you in portrait top left corner*/
    ILITEK_CORNER_TOPRIGHT,     /*When Phone Face you in portrait top right corner*/
    ILITEK_CORNER_BOTTOMLEFT,   /*When Phone Face you in portrait bottom left corner*/
    ILITEK_CORNER_BOTTOMRIGHT,  /*When Phone Face you in portrait bottom right corner*/
}ilitek_corner_type;




struct ilitek_corner_info{
	uint16_t id;
	struct mutual_touch_point point;
	bool flag;

};

extern struct ilitek_limit_data edge_limit_data;
struct ilitek_corner_info ilitek_corner[10];


static bool ilitek_corner_point_process(int i)
{
    int j;
        //节点/proc/touchpanel/oppo_tp_limit_enable的bit1位来控制控制
        //ipio_info("enable :%d %d %d %d \n",edge_limit_data.limit_corner_lu_enable,edge_limit_data.limit_corner_ru_enable,edge_limit_data.limit_corner_ld_enable,edge_limit_data.limit_corner_rd_enable);
		//ipio_info("x=%d,y=%d\n",g_mutual_data.mtp[i].x,g_mutual_data.mtp[i].y);
		//ipio_info("lx1=%d,ly1=%d\n",edge_limit_data.edge_limit.left_x1,edge_limit_data.edge_limit.left_y1);
        if ((edge_limit_data.limit_corner_lu_enable) && (g_mutual_data.mtp[i].x < edge_limit_data.edge_limit.left_x1 && g_mutual_data.mtp[i].y < edge_limit_data.edge_limit.left_y1)) {
            g_mutual_data.mtp[i].type  = ILITEK_AREA_CORNER;
            if (edge_limit_data.edge_limit.in_which_area == ILITEK_AREA_NORMAL)
            	{
            	//ipio_info("edge_limit_data.edge_limit.in_which_area %d\n",edge_limit_data.edge_limit.in_which_area);
                return true;
            	}
			//ipio_info("ILITEK_CORNER_TOPLEFT i=%d\n",i);

            ilitek_corner[ILITEK_CORNER_TOPLEFT].id = i;
            ilitek_corner[ILITEK_CORNER_TOPLEFT].point = g_mutual_data.mtp[i];
            ilitek_corner[ILITEK_CORNER_TOPLEFT].flag = true;

            edge_limit_data.edge_limit.in_which_area = g_mutual_data.mtp[i].type;
        }
         //节点/proc/touchpanel/oppo_tp_limit_enable的bit2位来控制控制
        if ((edge_limit_data.limit_corner_ru_enable)  && (g_mutual_data.mtp[i].x > edge_limit_data.edge_limit.right_x1 && g_mutual_data.mtp[i].y < edge_limit_data.edge_limit.right_y1)) {
            g_mutual_data.mtp[i].type  = ILITEK_AREA_CORNER;
            if (edge_limit_data.edge_limit.in_which_area == ILITEK_AREA_NORMAL)
                return true;
			//ipio_info("ILITEK_CORNER_TOPRIGHT i=%d\n",i);

            ilitek_corner[ILITEK_CORNER_TOPRIGHT].id = i;
            ilitek_corner[ILITEK_CORNER_TOPRIGHT].point = g_mutual_data.mtp[i];
            ilitek_corner[ILITEK_CORNER_TOPRIGHT].flag = true;

            edge_limit_data.edge_limit.in_which_area = g_mutual_data.mtp[i].type;
        }
         //节点/proc/touchpanel/oppo_tp_limit_enable的bit3位来控制控制
        if ((edge_limit_data.limit_corner_ld_enable)  && (g_mutual_data.mtp[i].x < edge_limit_data.edge_limit.left_x2 && g_mutual_data.mtp[i].y > edge_limit_data.edge_limit.left_y2)) {
            g_mutual_data.mtp[i].type  = ILITEK_AREA_CORNER;
            if (edge_limit_data.edge_limit.in_which_area == ILITEK_AREA_NORMAL)
                return true;
			//ipio_info("ILITEK_CORNER_BOTTOMLEFT i=%d\n",i);

            ilitek_corner[ILITEK_CORNER_BOTTOMLEFT].id = i;
            ilitek_corner[ILITEK_CORNER_BOTTOMLEFT].point = g_mutual_data.mtp[i];
            ilitek_corner[ILITEK_CORNER_BOTTOMLEFT].flag = true;

            edge_limit_data.edge_limit.in_which_area = g_mutual_data.mtp[i].type;
        }
         //节点/proc/touchpanel/oppo_tp_limit_enable的bit4位来控制控制
        if ((edge_limit_data.limit_corner_rd_enable)  && (g_mutual_data.mtp[i].x > edge_limit_data.edge_limit.right_x2 && g_mutual_data.mtp[i].y > edge_limit_data.edge_limit.right_y2)) {
            g_mutual_data.mtp[i].type  = ILITEK_AREA_CORNER;
            if (edge_limit_data.edge_limit.in_which_area == ILITEK_AREA_NORMAL)
                return true;
			//ipio_info("ILITEK_CORNER_BOTTOMRIGHT i=%d\n",i);

            ilitek_corner[ILITEK_CORNER_BOTTOMRIGHT].id = i;
            ilitek_corner[ILITEK_CORNER_BOTTOMRIGHT].point = g_mutual_data.mtp[i];
            ilitek_corner[ILITEK_CORNER_BOTTOMRIGHT].flag = true;

            edge_limit_data.edge_limit.in_which_area = g_mutual_data.mtp[i].type;
        }
        //坐标点为非边角区域时，弹起前面记录的边角坐标点
		//ipio_info("check corner\n");
		//ipio_info("i=%d\n",i);
		//ipio_info("%d  %d\n",g_mutual_data.mtp[i].type,edge_limit_data.edge_limit.in_which_area);
        if (g_mutual_data.mtp[i].type!= ILITEK_AREA_CORNER) {
            if (edge_limit_data.edge_limit.in_which_area == ILITEK_AREA_CORNER) {
                for (j = 0; j < 4; j++) {
                    if (ilitek_corner[j].flag) {
#ifdef MT_B_TYPE
                        input_mt_slot(core_fr->input_device, ilitek_corner[j].id);
                        input_mt_report_slot_state(core_fr->input_device, MT_TOOL_FINGER, 0);
						//ipio_info("release corner\n");
#endif
                    }
                }
            }
            g_mutual_data.mtp[i].type = ILITEK_AREA_NORMAL;
            edge_limit_data.edge_limit.in_which_area = g_mutual_data.mtp[i].type;
        }


    return false;
}

#endif
#endif
/**
 * Calculate the check sum of each packet reported by firmware
 *
 * @pMsg: packet come from firmware
 * @nLength : the length of its packet
 */
uint8_t core_fr_calc_checksum(uint8_t *pMsg, uint32_t nLength)
{
	int i;
	int32_t nCheckSum = 0;

	for (i = 0; i < nLength; i++) {
		nCheckSum += pMsg[i];
	}

	return (uint8_t) ((-nCheckSum) & 0xFF);
}
EXPORT_SYMBOL(core_fr_calc_checksum);
#if (INTERFACE == I2C_INTERFACE)

/**
 *  Receive data when fw mode stays at i2cuart mode.
 *
 *  the first is to receive N bytes depending on the mode that firmware stays
 *  before going in this function, and it would check with i2c buffer if it
 *  remains the rest of data.
 */
static void i2cuart_recv_packet(void)
{
	int res = 0, need_read_len = 0, one_data_bytes = 0;
	int type = g_fr_node->data[3] & 0x0F;
	int actual_len = g_fr_node->len - 5;

	ipio_debug(DEBUG_FINGER_REPORT, "pid = %x, data[3] = %x, type = %x, actual_len = %d\n",
	    g_fr_node->data[0], g_fr_node->data[3], type, actual_len);

	need_read_len = g_fr_node->data[1] * g_fr_node->data[2];

	if (type == 0 || type == 1 || type == 6) {
		one_data_bytes = 1;
	} else if (type == 2 || type == 3) {
		one_data_bytes = 2;
	} else if (type == 4 || type == 5) {
		one_data_bytes = 4;
	}

	ipio_debug(DEBUG_FINGER_REPORT, "need_read_len = %d  one_data_bytes = %d\n", need_read_len, one_data_bytes);

	need_read_len = need_read_len * one_data_bytes + 1;

	if (need_read_len > actual_len) {
		g_fr_uart = kmalloc(sizeof(*g_fr_uart), GFP_ATOMIC);
		if (ERR_ALLOC_MEM(g_fr_uart)) {
			ipio_err("Failed to allocate g_fr_uart memory %ld\n", PTR_ERR(g_fr_uart));
			return;
		}

		g_fr_uart->len = need_read_len - actual_len;
		g_fr_uart->data = kzalloc(g_fr_uart->len, GFP_ATOMIC);
		if (ERR_ALLOC_MEM(g_fr_uart->data)) {
			ipio_err("Failed to allocate g_fr_uart memory %ld\n", PTR_ERR(g_fr_uart->data));
			return;
		}

		g_total_len += g_fr_uart->len;
		res = core_read(core_config->slave_i2c_addr, g_fr_uart->data, g_fr_uart->len);
		if (res < 0)
			ipio_err("Failed to read finger report packet\n");
	}
}
#endif
/*
 * It'd be called when a finger's touching down a screen. It'll notify the event
 * to the uplayer from input device.
 *
 * @x: the axis of X
 * @y: the axis of Y
 * @pressure: the value of pressue on a screen
 * @id: an id represents a finger pressing on a screen
 */
void core_fr_touch_press(int32_t x, int32_t y, int32_t w, uint32_t pressure, int32_t id)
{
	ipio_debug(DEBUG_FINGER_REPORT, "DOWN: id = %d, x = %d, y = %d, w = %d\n", id, x, y, w);

#ifdef MT_B_TYPE
	input_mt_slot(core_fr->input_device, id);
	input_mt_report_slot_state(core_fr->input_device, MT_TOOL_FINGER, true);
	input_report_abs(core_fr->input_device, ABS_MT_POSITION_X, x);
	input_report_abs(core_fr->input_device, ABS_MT_POSITION_Y, y);
	input_report_abs(core_fr->input_device, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(core_fr->input_device, ABS_MT_WIDTH_MAJOR, w);

	if (core_fr->isEnablePressure)
		input_report_abs(core_fr->input_device, ABS_MT_PRESSURE, pressure);
#else
	input_report_key(core_fr->input_device, BTN_TOUCH, 1);

	input_report_abs(core_fr->input_device, ABS_MT_TRACKING_ID, id);
	input_report_abs(core_fr->input_device, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(core_fr->input_device, ABS_MT_WIDTH_MAJOR, 1);
	input_report_abs(core_fr->input_device, ABS_MT_POSITION_X, x);
	input_report_abs(core_fr->input_device, ABS_MT_POSITION_Y, y);

	if (core_fr->isEnablePressure)
		input_report_abs(core_fr->input_device, ABS_MT_PRESSURE, pressure);

	input_mt_sync(core_fr->input_device);
#endif /* MT_B_TYPE */
}
EXPORT_SYMBOL(core_fr_touch_press);

/*
 * It'd be called when a finger's touched up from a screen. It'll notify
 * the event to the uplayer from input device.
 *
 * @x: the axis of X
 * @y: the axis of Y
 * @id: an id represents a finger leaving from a screen.
 */
void core_fr_touch_release(int32_t x, int32_t y, int32_t id)
{
	ipio_debug(DEBUG_FINGER_REPORT, "UP: id = %d, x = %d, y = %d\n", id, x, y);

#ifdef MT_B_TYPE
	input_mt_slot(core_fr->input_device, id);
	input_mt_report_slot_state(core_fr->input_device, MT_TOOL_FINGER, false);
#else
	input_report_key(core_fr->input_device, BTN_TOUCH, 0);
	input_mt_sync(core_fr->input_device);
#endif /* MT_B_TYPE */
}
EXPORT_SYMBOL(core_fr_touch_release);

void core_fr_touch_all_release(struct core_fr_data *core_fr)
{
	int i = 0;

	for (i = 0; i < MAX_TOUCH_NUM; i++) {
#ifdef MT_B_TYPE
	input_mt_slot(core_fr->input_device, i);
	input_mt_report_slot_state(core_fr->input_device, MT_TOOL_FINGER, false);
#else
	input_report_key(core_fr->input_device, BTN_TOUCH, 0);
	input_mt_sync(core_fr->input_device);
#endif /* MT_B_TYPE */

	}
	input_report_key(core_fr->input_device, BTN_TOUCH, 0);
	input_sync(core_fr->input_device);
}
EXPORT_SYMBOL(core_fr_touch_all_release);

static int parse_touch_package_v3_2(void)
{
	ipio_info("Not implemented yet\n");
	return 0;
}

static int finger_report_ver_3_2(void)
{
	ipio_info("Not implemented yet\n");
	parse_touch_package_v3_2();
	return 0;
}

/*
 * It mainly parses the packet assembled by protocol v5.0
 */
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/17, add presure report function
uint32_t last_pressure = 0;
int package_times = 0;
#endif

static int parse_touch_package_v5_0(uint8_t pid)
{
	int i, res = 0;
	uint8_t check_sum = 0;
	uint32_t nX = 0, nY = 0, nW =0;
	#if 1
	#ifdef MING_TEST
	uint32_t count = (core_config->tp_info->nXChannelNum * core_config->tp_info->nYChannelNum) * 2;
	#endif
	#endif
	for (i = 0; i < 9; i++)
		ipio_debug(DEBUG_FINGER_REPORT, "data[%d] = %x\n", i, g_fr_node->data[i]);

	check_sum = core_fr_calc_checksum(&g_fr_node->data[0], (g_fr_node->len - 1));
	ipio_debug(DEBUG_FINGER_REPORT, "data = %x  ;  check_sum : %x\n", g_fr_node->data[g_fr_node->len - 1], check_sum);

	if (g_fr_node->data[g_fr_node->len - 1] != check_sum) {
		ipio_err("Wrong checksum %d\n",g_fr_node->len);
		res = -1;
		goto out;
	}

	/* start to parsing the packet of finger report */
	if (pid == protocol->demo_pid) {
		ipio_debug(DEBUG_FINGER_REPORT, " **** Parsing DEMO packets : 0x%x ****\n", pid);

		for (i = 0; i < MAX_TOUCH_NUM; i++) {
			if ((g_fr_node->data[(4 * i) + 1] == 0xFF) && (g_fr_node->data[(4 * i) + 2] && 0xFF)
			    && (g_fr_node->data[(4 * i) + 3] == 0xFF)) {
#ifdef MT_B_TYPE
				g_current_touch[i] = 0;
#endif
				continue;
			}

			nX = (((g_fr_node->data[(4 * i) + 1] & 0xF0) << 4) | (g_fr_node->data[(4 * i) + 2]));
			nY = (((g_fr_node->data[(4 * i) + 1] & 0x0F) << 8) | (g_fr_node->data[(4 * i) + 3]));
			nW = g_fr_node->data[(4 * i) + 4];

			if (nW< 0x0c && nW > 0){
				nW = 0x05;
			}

			if (!core_fr->isSetResolution) {
				g_mutual_data.mtp[g_mutual_data.touch_num].x = nX * TOUCH_SCREEN_X_MAX / TPD_WIDTH;
				g_mutual_data.mtp[g_mutual_data.touch_num].y = nY * TOUCH_SCREEN_Y_MAX / TPD_HEIGHT;
				g_mutual_data.mtp[g_mutual_data.touch_num].w = nW;
				g_mutual_data.mtp[g_mutual_data.touch_num].id = i;
			} else {
				g_mutual_data.mtp[g_mutual_data.touch_num].x = nX;
				g_mutual_data.mtp[g_mutual_data.touch_num].y = nY;
				g_mutual_data.mtp[g_mutual_data.touch_num].w = nW;
				g_mutual_data.mtp[g_mutual_data.touch_num].id = i;
			}

			if (core_fr->isEnablePressure){
				g_mutual_data.mtp[g_mutual_data.touch_num].pressure = g_fr_node->data[(4 * i) + 4];
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/17, add presure report function
				if(last_pressure == g_mutual_data.mtp[g_mutual_data.touch_num].pressure){
					if(package_times %(ipd->presure_speed) != 0){
						g_mutual_data.mtp[g_mutual_data.touch_num].pressure++;
					}
					package_times++;
				}else{
					last_pressure = g_mutual_data.mtp[g_mutual_data.touch_num].pressure;
				}
#endif
			}
			else
				g_mutual_data.mtp[g_mutual_data.touch_num].pressure = 1;

			ipio_debug(DEBUG_FINGER_REPORT, "[x,y]=[%d,%d]\n", nX, nY);
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
			last_nX =nX;
			last_nY =nY;
			if(b_first_point)
			{
			  oppo_debug(1, "[x,y]=[%d,%d]\n", nX, nY);
			  b_first_point =false;
			}
			oppo_debug(2, "[x,y]=[%d,%d]\n", nX, nY);
#endif
			ipio_debug(DEBUG_FINGER_REPORT, "point[%d] : (%d,%d,%d) = %d\n",
			    g_mutual_data.mtp[g_mutual_data.touch_num].id,
			    g_mutual_data.mtp[g_mutual_data.touch_num].x,
			    g_mutual_data.mtp[g_mutual_data.touch_num].y,
			    g_mutual_data.mtp[g_mutual_data.touch_num].w, g_mutual_data.mtp[g_mutual_data.touch_num].pressure);

			g_mutual_data.touch_num++;

#ifdef MT_B_TYPE
			g_current_touch[i] = 1;
#endif
		}
	} else if (pid == protocol->debug_pid) {
		ipio_debug(DEBUG_FINGER_REPORT, " **** Parsing DEBUG packets : 0x%x ****\n", pid);
		ipio_debug(DEBUG_FINGER_REPORT, "Length = %d\n", (g_fr_node->data[1] << 8 | g_fr_node->data[2]));

		for (i = 0; i < MAX_TOUCH_NUM; i++) {
			if ((g_fr_node->data[(3 * i) + 5] == 0xFF) && (g_fr_node->data[(3 * i) + 6] && 0xFF)
			    && (g_fr_node->data[(3 * i) + 7] == 0xFF)) {
#ifdef MT_B_TYPE
				g_current_touch[i] = 0;
#endif
				continue;
			}

			nX = (((g_fr_node->data[(3 * i) + 5] & 0xF0) << 4) | (g_fr_node->data[(3 * i) + 6]));
			nY = (((g_fr_node->data[(3 * i) + 5] & 0x0F) << 8) | (g_fr_node->data[(3 * i) + 7]));
			nW = g_fr_node->data[(4 * i) + 4];
			if (nW< 0x10 && nW > 0){
				nW = 0x05;
			}

			if (!core_fr->isSetResolution) {
				g_mutual_data.mtp[g_mutual_data.touch_num].x = nX * TOUCH_SCREEN_X_MAX / TPD_WIDTH;
				g_mutual_data.mtp[g_mutual_data.touch_num].y = nY * TOUCH_SCREEN_Y_MAX / TPD_HEIGHT;
				g_mutual_data.mtp[g_mutual_data.touch_num].w = nW;
				g_mutual_data.mtp[g_mutual_data.touch_num].id = i;
			} else {
				g_mutual_data.mtp[g_mutual_data.touch_num].x = nX;
				g_mutual_data.mtp[g_mutual_data.touch_num].y = nY;
				g_mutual_data.mtp[g_mutual_data.touch_num].w = nW;
				g_mutual_data.mtp[g_mutual_data.touch_num].id = i;
			}

			if (core_fr->isEnablePressure)
				g_mutual_data.mtp[g_mutual_data.touch_num].pressure = g_fr_node->data[(4 * i) + 4];
			else
				g_mutual_data.mtp[g_mutual_data.touch_num].pressure = 1;

			ipio_debug(DEBUG_FINGER_REPORT, "[x,y]=[%d,%d]\n", nX, nY);
			ipio_debug(DEBUG_FINGER_REPORT, "point[%d] : (%d,%d,%d) = %d\n",
			    g_mutual_data.mtp[g_mutual_data.touch_num].id,
			    g_mutual_data.mtp[g_mutual_data.touch_num].x,
			    g_mutual_data.mtp[g_mutual_data.touch_num].y,
			    g_mutual_data.mtp[g_mutual_data.touch_num].w, g_mutual_data.mtp[g_mutual_data.touch_num].pressure);

			g_mutual_data.touch_num++;

#ifdef MT_B_TYPE
			g_current_touch[i] = 1;
#endif
		}
#if 1
#ifdef MING_TEST
{
 // if (ipd->delta_node_open){
	//mutex_lock(&ipd->ilitek_delta_mutex);
		for (i = 35; i < count + 35; i+=2) {
			if((uint8_t)(g_fr_node->data[i] & 0x80) == (uint8_t)0x80)
			{
				buf_rawdata[(i-35)/2] = 0x10000 - ((g_fr_node->data[i] << 8) + g_fr_node->data[i+1]);
				buf_delta[(i-35)/2] = 0x10000 - ((g_fr_node->data[i] << 8) + g_fr_node->data[i+1]);
				//printk(KERN_CONT ", -%d, ", 0x10000 - ((g_fr_node->data[i] << 8) + g_fr_node->data[i+1]));
			}
			else
			{
				buf_rawdata[(i-35)/2] = (g_fr_node->data[i] << 8) + g_fr_node->data[i+1];
				buf_delta[(i-35)/2] = (g_fr_node->data[i] << 8) + g_fr_node->data[i+1];
				//printk(KERN_CONT ", %d, ", (g_fr_node->data[i] << 8) + g_fr_node->data[i+1]);
			}
		}
	//mutex_unlock(&ipd->ilitek_delta_mutex);
	wake_up(&(ipd->delta_inq));
	ipd->delta_data_frame =1;
 // }
}
#endif
#endif

	} else {
		if (pid != 0) {
			/* ignore the pid with 0x0 after enable irq at once */
			ipio_err(" **** Unknown PID : 0x%x ****\n", pid);
			res = -1;
		}
	}

out:
	return res;
}

/*
 * The function is called by an interrupt and used to handle packet of finger
 * touch from firmware. A differnece in the process of the data is acorrding to the protocol
 */
static int finger_report_ver_5_0(void)
{
	int i,  res = 0;
	static int last_touch = 0;
	uint8_t pid = 0x0;
#if (INTERFACE == SPI_INTERFACE)
    int rlen=0;
#endif
	memset(&g_mutual_data, 0x0, sizeof(struct mutual_touch_info));
	edge_limit_data.edge_limit.in_which_area = ILITEK_AREA_DEFAULT;
	for(i=0;i<10;i++)
	{
	 memset(&ilitek_corner[i], 0x0, sizeof(struct ilitek_corner_info));
	}
#if (INTERFACE == SPI_INTERFACE)
        rlen = core_spi_read_data_after_checksize(g_fr_node->data, g_fr_node->len);
#else
#ifdef I2C_SEGMENT
        res = core_i2c_segmental_read(core_config->slave_i2c_addr, g_fr_node->data, g_fr_node->len);
#else
        res = core_read(core_config->slave_i2c_addr, g_fr_node->data, g_fr_node->len);
#endif
#endif
	if (res < 0) {
		ipio_err("Failed to read finger report packet\n");
		goto out;
	}

	pid = g_fr_node->data[0];
	ipio_debug(DEBUG_FINGER_REPORT, "PID = 0x%x\n", pid);

	if (pid == protocol->i2cuart_pid) {
        #if (INTERFACE == I2C_INTERFACE)
		ipio_debug(DEBUG_FINGER_REPORT, "I2CUART(0x%x): prepare to receive rest of data\n", pid);
		i2cuart_recv_packet();
        #endif
		goto out;
	}
#ifdef ODM_WT_EDIT

	if (pid == protocol->ges_pid && core_config->isEnableGesture) {
		ipio_debug(DEBUG_FINGER_REPORT, "pid = 0x%x, code = %x\n", pid, g_fr_node->data[1]);
		for(i=0;i<g_fr_node->len;i++)
			{
				buf_gesture[i] = g_fr_node->data[i];
			}
		#if 0
		//gesture = core_gesture_key(g_fr_node->data[1]);
	//	if (gesture != -1) {
			input_report_key(core_fr->input_device, KEY_POWER, 1);
			input_sync(core_fr->input_device);
			input_report_key(core_fr->input_device, KEY_POWER, 0);
			input_sync(core_fr->input_device);
		//}
		goto out;
		#endif
		#ifdef ILITEK_GESTURE_WAKEUP
		res = ilitek_get_gesture_info(gesture_report_data);
		if(res)
			{
			input_report_key(core_fr->input_device, KEY_F4, 1);
			input_sync(core_fr->input_device);
			input_report_key(core_fr->input_device, KEY_F4, 0);
			input_sync(core_fr->input_device);
			}
		goto out;
		#endif

	}
#endif
	res = parse_touch_package_v5_0(pid);
	if (res < 0) {
		ipio_err("Failed to parse packet of finger touch\n");
		goto out;
	}

	ipio_debug(DEBUG_FINGER_REPORT, "Touch Num = %d, LastTouch = %d\n", g_mutual_data.touch_num, last_touch);

	/* interpret parsed packat and send input events to system */
	if (g_mutual_data.touch_num > 0) {
#ifdef MT_B_TYPE
		for (i = 0; i < g_mutual_data.touch_num; i++) {
			input_report_key(core_fr->input_device, BTN_TOUCH, 1);
			core_fr_touch_press(g_mutual_data.mtp[i].x, g_mutual_data.mtp[i].y, g_mutual_data.mtp[i].w,g_mutual_data.mtp[i].pressure, g_mutual_data.mtp[i].id);

			input_report_key(core_fr->input_device, BTN_TOOL_FINGER, 1);
#ifdef ILITEK_EDGE_LIMIT
			ilitek_corner_point_process(i);
#endif
		}

		for (i = 0; i < MAX_TOUCH_NUM; i++) {
			ipio_debug(DEBUG_FINGER_REPORT, "g_previous_touch[%d]=%d, g_current_touch[%d]=%d\n", i,
			    g_previous_touch[i], i, g_current_touch[i]);

			if (g_current_touch[i] == 0 && g_previous_touch[i] == 1) {
				core_fr_touch_release(0, 0, i);
			}

			g_previous_touch[i] = g_current_touch[i];
		}
#else
		for (i = 0; i < g_mutual_data.touch_num; i++) {
			core_fr_touch_press(g_mutual_data.mtp[i].x, g_mutual_data.mtp[i].y, g_mutual_data.mtp[i].w, g_mutual_data.mtp[i].pressure, g_mutual_data.mtp[i].id);
		}
#endif
		input_sync(core_fr->input_device);

		last_touch = g_mutual_data.touch_num;
	} else {
		if (last_touch > 0) {
#ifdef MT_B_TYPE
			for (i = 0; i < MAX_TOUCH_NUM; i++) {
				ipio_debug(DEBUG_FINGER_REPORT, "g_previous_touch[%d]=%d, g_current_touch[%d]=%d\n", i,
				    g_previous_touch[i], i, g_current_touch[i]);

				if (g_current_touch[i] == 0 && g_previous_touch[i] == 1) {
					core_fr_touch_release(0, 0, i);
				}
				g_previous_touch[i] = g_current_touch[i];
			}
			input_report_key(core_fr->input_device, BTN_TOUCH, 0);
			input_report_key(core_fr->input_device, BTN_TOOL_FINGER, 0);
#else
			core_fr_touch_release(0, 0, 0);
#endif

			input_sync(core_fr->input_device);

			last_touch = 0;
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/17, add presure report function
			last_pressure = 0;
			package_times = 0;
#endif
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
			b_first_point =true;
			oppo_debug(1, "[x,y]=[%d,%d]\n", last_nX, last_nY);
#endif
		}
	}

out:
	return res;
}

void core_fr_mode_control(uint8_t *from_user)
{
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/11/14,remove no useless codes for open short test
	int mode;
	//int codeLength = 8;
	//uint8_t mp_code[8] = { 0 };
#endif
	uint8_t cmd[4] = { 0 };

	ilitek_platform_disable_irq();

	if (from_user == NULL) {
		ipio_err("Arguments from user space are invaild\n");
		mode = P5_0_FIRMWARE_DEMO_MODE;
		goto out;
	}

	ipio_debug(DEBUG_FINGER_REPORT, "mode = %x, b1 = %x, b2 = %x, b3 = %x\n",
	    from_user[0], from_user[1], from_user[2], from_user[3]);

	mode = from_user[0];

	if (protocol->major == 0x5) {
		if (mode == protocol->i2cuart_mode) {
			cmd[0] = protocol->cmd_i2cuart;
			cmd[1] = *(from_user + 1);
			cmd[2] = *(from_user + 2);

			ipio_info("Switch to I2CUART mode, cmd = %x, b1 = %x, b2 = %x\n", cmd[0], cmd[1], cmd[2]);

			if ((core_write(core_config->slave_i2c_addr, cmd, 3)) < 0) {
				ipio_err("Failed to switch I2CUART mode\n");
				goto out;
			}

		} else if (mode == protocol->demo_mode || mode == protocol->debug_mode) {
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/11/14,remove no useless codes for open short test
			if(mode == protocol->debug_mode){
				cmd[0] = protocol->cmd_mode_ctrl;
				cmd[1] = mode;
			}
			//cmd[0] = protocol->cmd_mode_ctrl;
			//cmd[1] = mode;

			ipio_info("Switch to Demo/Debug mode, cmd = 0x%x, b1 = 0x%x\n", cmd[0], cmd[1]);
			core_gesture->entry =0;
			if(mode == protocol->debug_mode){
				if ((core_write(core_config->slave_i2c_addr, cmd, 2)) < 0) {
					ipio_err("Failed to switch Demo/Debug mode\n");
					goto out;
				}
			}
			/*if ((core_write(core_config->slave_i2c_addr, cmd, 2)) < 0) {
				ipio_err("Failed to switch Demo/Debug mode\n");
				goto out;
			}*/
#endif
			core_fr->actual_fw_mode = mode;
			ipio_info("mfeng\n");

		} else if (mode == protocol->test_mode) {
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/11/14,remove no useless codes for open short test
			//cmd[0] = protocol->cmd_mode_ctrl;
			//cmd[1] = mode;

			ipio_info("Switch to Test mode, cmd = 0x%x, b1 = 0x%x\n", cmd[0], cmd[1]);

			/*if ((core_write(core_config->slave_i2c_addr, cmd, 2)) < 0) {
				ipio_err("Failed to switch Test mode\n");
				goto out;
			}

			cmd[0] = 0xFE;*/


			/* Read MP Test information to ensure if fw supports test mode. */
			//core_write(core_config->slave_i2c_addr, cmd, 1);
			//msleep(10);
			//core_read(core_config->slave_i2c_addr, mp_code, codeLength);

			/*for (i = 0; i < codeLength - 1; i++)
				checksum += mp_code[i];

			if ((-checksum & 0xFF) != mp_code[codeLength - 1]) {
				ipio_info("checksume error (0x%x), FW doesn't support test mode.\n",
						(-checksum & 0XFF));
				goto out;
			}*/
#endif
			/* FW enter to Test Mode */
			core_fr->actual_fw_mode = mode;
			if (core_mp_move_code() == 0)
				core_fr->actual_fw_mode = mode;

		} else {
			ipio_err("Unknown firmware mode: %x\n", mode);
		}
	} else {
		ipio_err("Wrong the major version of protocol, 0x%x\n", protocol->major);
	}

out:
	core_fr->actual_fw_mode = mode;
	ilitek_platform_enable_irq();
}
EXPORT_SYMBOL(core_fr_mode_control);

/**
 * Calculate the length with different modes according to the format of protocol 5.0
 *
 * We compute the length before receiving its packet. If the length is differnet between
 * firmware and the number we calculated, in this case I just print an error to inform users
 * and still send up to users.
 */
    static int calc_packet_length(void)
    {

#if (INTERFACE == SPI_INTERFACE)
        int res = 0;
        res = core_spi_check_read_size();
        ipio_debug(DEBUG_FINGER_REPORT, "calc_packet_length %d\n", res);
        return res;
#else
        uint16_t xch = 0, ych = 0, stx = 0, srx = 0;
        /* FIXME: self_key not defined by firmware yet */
        uint16_t self_key = 2;
        int rlen = 0;
        if (protocol->major == 0x5) {
            if (!ERR_ALLOC_MEM(core_config->tp_info)) {
                xch = core_config->tp_info->nXChannelNum;
                ych = core_config->tp_info->nYChannelNum;
                stx = core_config->tp_info->self_tx_channel_num;
                srx = core_config->tp_info->self_rx_channel_num;
            }

            ipio_debug(DEBUG_FINGER_REPORT, "firmware mode : 0x%x\n", core_fr->actual_fw_mode);

            if (protocol->demo_mode == core_fr->actual_fw_mode) {
                rlen = protocol->demo_len;
            } else if (protocol->test_mode == core_fr->actual_fw_mode) {
                if (ERR_ALLOC_MEM(core_config->tp_info)) {
                    rlen = protocol->test_len;
                } else {
                    rlen = (2 * xch * ych) + (stx * 2) + (srx * 2) + 2 * self_key + 1;
                    rlen += 1;
                }
            } else if (protocol->debug_mode == core_fr->actual_fw_mode) {
                if (ERR_ALLOC_MEM(core_config->tp_info)) {
                    rlen = protocol->debug_len;
                } else {
                    rlen = (2 * xch * ych) + (stx * 2) + (srx * 2) + 2 * self_key + (8 * 2) + 1;
                    rlen += 35;
                }
            } else if (protocol->gesture_mode == core_fr->actual_fw_mode) {
                if(core_gesture->mode == GESTURE_NORMAL_MODE)
                    rlen = GESTURE_MORMAL_LENGTH;
                else
                    rlen = GESTURE_INFO_LENGTH;
                ipio_debug(DEBUG_FINGER_REPORT, "rlen = %d\n", rlen);
            }
            else {
                ipio_err("Unknown firmware mode : %d\n", core_fr->actual_fw_mode);
                rlen = 0;
            }
        } else {
            ipio_err("Wrong the major version of protocol, 0x%x\n", protocol->major);
            return -1;
        }
        ipio_debug(DEBUG_FINGER_REPORT, "rlen = %d\n", rlen);
        return rlen;
#endif
    }


/**
 * The table is used to handle calling functions that deal with packets of finger report.
 * The callback function might be different of what a protocol is used on a chip.
 *
 * It's possible to have the different protocol according to customer's requirement on the same
 * touch ic with customised firmware, so I don't have to identify which of the ic has been used; instead,
 * the version of protocol should match its parsing pattern.
 */
typedef struct {
	uint8_t protocol_marjor_ver;
	uint8_t protocol_minor_ver;
	int (*finger_report)(void);
} fr_hashtable;

fr_hashtable fr_t[] = {
	{0x3, 0x2, finger_report_ver_3_2},
	{0x5, 0x0, finger_report_ver_5_0},
	{0x5, 0x1, finger_report_ver_5_0},
	{0x5, 0x2, finger_report_ver_5_0},
	{0x5, 0x3, finger_report_ver_5_0},
	{0x5, 0x4, finger_report_ver_5_0},
	{0x5, 0x5, finger_report_ver_5_0},
};

/**
 * The function is an entry for the work queue registered by ISR activates.
 *
 * Here will allocate the size of packet depending on what the current protocol
 * is used on its firmware.
 */
void core_fr_handler(void)
{
	int i = 0;
	uint8_t *tdata = NULL;

	if (core_gesture->suspend && core_config->isEnableGesture) {
		msleep(100);
	}

	if (core_fr->isEnableFR) {
        mutex_lock(&ipd->report_mutex);
		g_total_len = calc_packet_length();
        if(g_total_len>2048){
            ipio_err("get the longer len %d\n",g_total_len);
            mutex_unlock(&ipd->report_mutex);
            return;
            }
		if (g_total_len>0) {
			g_fr_node = kmalloc(sizeof(*g_fr_node), GFP_ATOMIC);
			if (ERR_ALLOC_MEM(g_fr_node)) {
				ipio_err("Failed to allocate g_fr_node memory1 %ld\n", PTR_ERR(g_fr_node));
				//mutex_unlock(&ipd->report_mutex);
				goto out;
			}

			//g_fr_node->data = kcalloc(g_total_len, sizeof(uint8_t), GFP_ATOMIC);
			g_fr_node->data = kcalloc(2048, sizeof(uint8_t), GFP_ATOMIC);
			if (ERR_ALLOC_MEM(g_fr_node->data)) {
				ipio_err("Failed to allocate g_fr_node memory2 %ld\n", PTR_ERR(g_fr_node->data));
				//mutex_unlock(&ipd->report_mutex);
				goto out;
			}

			g_fr_node->len = g_total_len;
			memset(g_fr_node->data, 0xFF, (uint8_t) sizeof(uint8_t) * g_total_len);

			while (i < ARRAY_SIZE(fr_t)) {
				if (protocol->major == fr_t[i].protocol_marjor_ver) {
					mutex_lock(&ipd->plat_mutex);
					fr_t[i].finger_report();
					mutex_unlock(&ipd->plat_mutex);
                    //mutex_unlock(&ipd->report_mutex);

					/* 2048 is referred to the defination by user */
					if (g_total_len < 2048) {
						tdata = kmalloc(2048, GFP_ATOMIC);
						if (ERR_ALLOC_MEM(tdata)) {
							ipio_err("Failed to allocate g_fr_node memory 3%ld\n",
								PTR_ERR(tdata));
							goto out;
						}

						memcpy(tdata, g_fr_node->data, g_fr_node->len);
						/* merge uart data if it's at i2cuart mode */
						if (g_fr_uart != NULL)
							memcpy(tdata + g_fr_node->len, g_fr_uart->data, g_fr_uart->len);
					} else {
						ipio_err("total length (%d) is too long than user can handle\n",
							g_total_len);
						goto out;
					}

					if (core_fr->isEnableNetlink)
						netlink_reply_msg(tdata, g_total_len);

					if (ipd->debug_node_open) {
						mutex_lock(&ipd->ilitek_debug_mutex);
						memset(ipd->debug_buf[ipd->debug_data_frame], 0x00,
						       (uint8_t) sizeof(uint8_t) * 2048);
						memcpy(ipd->debug_buf[ipd->debug_data_frame], tdata, g_total_len);
						ipd->debug_data_frame++;
						if (ipd->debug_data_frame > 1) {
							ipio_info("ipd->debug_data_frame = %d\n", ipd->debug_data_frame);
						}
						if (ipd->debug_data_frame > 1023) {
							ipio_err("ipd->debug_data_frame = %d > 1024\n",
								ipd->debug_data_frame);
							ipd->debug_data_frame = 1023;
						}
						mutex_unlock(&ipd->ilitek_debug_mutex);
						wake_up(&(ipd->inq));
					}
					break;
				}
				i++;
			}

			if (i >= ARRAY_SIZE(fr_t))
				ipio_err("Can't find any callback functions to handle INT event\n");
		} else {
			ipio_err("Wrong the length of packet\n");
		}
	} else {
		ipio_err("The figner report was disabled\n");
		return;
	}

out:
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/11/14,remove no useless codes for open short test
	//mutex_unlock(&ipd->report_mutex);

/*2018-8-27 begin*/
	if (CHECK_RECOVER == g_total_len && core_gesture->suspend && core_config->isEnableGesture) {
		ipio_info("esd gesture reset\n");
		mdelay(150);
		ilitek_esd_gesture_reset();
	}

	mutex_unlock(&ipd->report_mutex);
#endif
/*2018-8-27 end*/



	ipio_kfree((void **)&tdata);

	if(g_fr_node != NULL) {
		ipio_kfree((void **)&g_fr_node->data);
		ipio_kfree((void **)&g_fr_node);
	}

	if(g_fr_uart != NULL) {
		ipio_kfree((void **)&g_fr_uart->data);
		ipio_kfree((void **)&g_fr_uart);
	}

	g_total_len = 0;
	ipio_debug(DEBUG_IRQ, "handle INT done\n\n");
}
EXPORT_SYMBOL(core_fr_handler);

void core_fr_input_set_param(struct input_dev *input_device)
{
	int max_x = 0, max_y = 0, min_x = 0, min_y = 0;
	int max_tp = 0;

	core_fr->input_device = input_device;

	/* set the supported event type for input device */
	set_bit(EV_ABS, core_fr->input_device->evbit);
	set_bit(EV_SYN, core_fr->input_device->evbit);
	set_bit(EV_KEY, core_fr->input_device->evbit);
	set_bit(BTN_TOUCH, core_fr->input_device->keybit);
	set_bit(BTN_TOOL_FINGER, core_fr->input_device->keybit);
	set_bit(INPUT_PROP_DIRECT, core_fr->input_device->propbit);

	if (core_fr->isSetResolution) {
		max_x = core_config->tp_info->nMaxX;
		max_y = core_config->tp_info->nMaxY;
		min_x = core_config->tp_info->nMinX;
		min_y = core_config->tp_info->nMinY;
		max_tp = core_config->tp_info->nMaxTouchNum;
	} else {
		max_x = TOUCH_SCREEN_X_MAX;
		max_y = TOUCH_SCREEN_Y_MAX;
		min_x = TOUCH_SCREEN_X_MIN;
		min_y = TOUCH_SCREEN_Y_MIN;
		max_tp = MAX_TOUCH_NUM;
	}

	ipio_debug(DEBUG_FINGER_REPORT, "input resolution : max_x = %d, max_y = %d, min_x = %d, min_y = %d\n", max_x, max_y, min_x, min_y);
	ipio_debug(DEBUG_FINGER_REPORT, "input touch number: max_tp = %d\n", max_tp);

#if (TP_PLATFORM != PT_MTK)
	input_set_abs_params(core_fr->input_device, ABS_MT_POSITION_X, min_x, max_x - 1, 0, 0);
	input_set_abs_params(core_fr->input_device, ABS_MT_POSITION_Y, min_y, max_y - 1, 0, 0);

	input_set_abs_params(core_fr->input_device, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(core_fr->input_device, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
#endif /* PT_MTK */
	ipio_debug(DEBUG_FINGER_REPORT, "\n");
	if (core_fr->isEnablePressure)
		input_set_abs_params(core_fr->input_device, ABS_MT_PRESSURE, 0, 255, 0, 0);

#ifdef MT_B_TYPE
	#if KERNEL_VERSION(3, 7, 0) <= LINUX_VERSION_CODE
	input_mt_init_slots(core_fr->input_device, max_tp, INPUT_MT_DIRECT);
	#else
	input_mt_init_slots(core_fr->input_device, max_tp);
	#endif /* LINUX_VERSION_CODE */
#else
	input_set_abs_params(core_fr->input_device, ABS_MT_TRACKING_ID, 0, max_tp, 0, 0);
#endif /* MT_B_TYPE */
	ipio_debug(DEBUG_FINGER_REPORT, "\n");
	/* Set up virtual key with gesture code */
	core_gesture_init(core_fr);
}
EXPORT_SYMBOL(core_fr_input_set_param);

int core_fr_init(void)
{
	int i = 0;

	core_fr = kzalloc(sizeof(*core_fr), GFP_KERNEL);
	if (ERR_ALLOC_MEM(core_fr)) {
		ipio_err("Failed to allocate core_fr mem, %ld\n", PTR_ERR(core_fr));
		core_fr_remove();
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(ipio_chip_list); i++) {
		if (ipio_chip_list[i] == TP_TOUCH_IC) {
			core_fr->isEnableFR = true;
			core_fr->isEnableNetlink = false;
			core_fr->isEnablePressure = true;
			core_fr->isSetResolution = false;
			core_fr->actual_fw_mode = protocol->demo_mode;
			return 0;
		}
	}

	ipio_err("Can't find this chip in support list\n");
	return 0;
}
EXPORT_SYMBOL(core_fr_init);

void core_fr_remove(void)
{
	ipio_info("Remove core-FingerReport members\n");
	ipio_kfree((void **)&core_fr);
}
EXPORT_SYMBOL(core_fr_remove);
