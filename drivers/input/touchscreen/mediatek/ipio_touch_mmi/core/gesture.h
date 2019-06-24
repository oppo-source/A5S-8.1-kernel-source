/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - generate.h
** Description : This program is for ili9881 driver gesture.h
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#ifndef __GESTURE_H
#define __GESTURE_H
#define GESTURE_NORMAL_MODE          0
#define GESTURE_INFO_MPDE            1
#define GESTURE_INFO_LENGTH          170
#define GESTURE_MORMAL_LENGTH        8

/* The example for the gesture virtual keys */
#define GESTURE_DOUBLECLICK			    0x58
#define GESTURE_UP						0x60
#define GESTURE_DOWN					0x61
#define GESTURE_LEFT					0x62
#define GESTURE_RIGHT					0x63
#define GESTURE_M						0x64
#define GESTURE_W						0x65
#define GESTURE_C						0x66
#define GESTURE_E						0x67
#define GESTURE_V						0x68
#define GESTURE_O						0x69
#define GESTURE_S						0x6A
#define GESTURE_Z						0x6B
//oppo gesture define
#define GESTURE_CODE_V_DOWN						0x6C
#define GESTURE_CODE_V_LEFT						0x6D
#define GESTURE_CODE_V_RIGHT					0x6E
#define GESTURE_CODE_TWO_LINE_2_BOTTOM			0x6F

#define KEY_GESTURE_D					KEY_D
#define KEY_GESTURE_UP					KEY_UP
#define KEY_GESTURE_DOWN				KEY_DOWN
#define KEY_GESTURE_LEFT				KEY_LEFT
#define KEY_GESTURE_RIGHT				KEY_RIGHT
#define KEY_GESTURE_O					KEY_O
#define KEY_GESTURE_E					KEY_E
#define KEY_GESTURE_M					KEY_M
#define KEY_GESTURE_W					KEY_W
#define KEY_GESTURE_S					KEY_S
#define KEY_GESTURE_V					KEY_V
#define KEY_GESTURE_C					KEY_C
#define KEY_GESTURE_Z					KEY_Z
struct core_gesture_data {
    uint32_t start_addr;
    uint32_t length;
    bool entry;
    uint8_t mode; //normal:0 info:1
    uint32_t ap_start_addr;
    uint32_t ap_length;
    uint32_t area_section;
    bool suspend;
};
#ifdef ILITEK_GESTURE_WAKEUP
#define Left2RightSwip						8
#define Right2LeftSwip						9
#define Up2DownSwip							10
#define Down2UpSwip							11
#define DouTap								1
#define UpVee								2
#define DownVee								3
#define LeftVee								4
#define RightVee							5
#define Circle								6
#define	Mgestrue							12
#define Wgestrue							13
#define DouSwip								7
#define UnkownGesture						0xFF

#define GESTURE_V_DOWN						0x6C
#define GESTURE_V_LEFT						0x6D
#define GESTURE_V_RIGHT						0x6E
#define GESTURE_TWOLINE_DOWN				0x6F
struct gesture_point {
	uint16_t x;
	uint16_t y;
};

struct ili_gesture_info{
	uint8_t gesture_type;
	struct gesture_point Point_start;
	struct gesture_point Point_end;
	struct gesture_point Point_1st;
	struct gesture_point Point_2nd;
	struct gesture_point Point_3rd;
	struct gesture_point Point_4th;
	uint8_t clockwise;
};


#endif
extern struct core_gesture_data *core_gesture;

extern int core_gesture_key(uint8_t gid);

extern int core_load_gesture_code(void);
extern void core_gesture_remove(void);
extern int host_download(bool isIRAM);
extern int core_load_ap_code(void);
#endif

