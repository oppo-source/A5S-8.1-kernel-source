 /*
 *
 * Filename:
 * ---------
 *     sp2509vmipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *-----------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 */

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "sp2509vmipiraw_Sensor.h"

/* Xuegui.Bao@Camera.Driver, 2018/10/10, add for [summer otp bringup] */
#include "cam_cal_define.h"
#include <linux/hardware_info.h>
extern int hardwareinfo_set_prop(int cmd, const char *name);

//define ENABLE_SP2509V_OTP 1

/****************************Modify Following Strings for Debug****************************/
#define PFX "SP2509V_camera_sensor"
#define LOG_1 LOG_INF("SP2509V,MIPI 1LANE\n")
#define LOG_2 LOG_INF("preview 1280*960@30fps,420Mbps/lane; video 1280*960@30fps,420Mbps/lane; capture 5M@15fps,420Mbps/lane\n")
/****************************   Modify end    *******************************************/
/**************************** Modify end *****************************/

#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_DBG(format, args...)    pr_err(PFX "[%s] " format, __FUNCTION__, ##args)

/* Xuegui.Bao@Camera.Driver, 2018/10/10, add for [summer otp bringup] */
#ifdef ENABLE_SP2509V_OTP
extern struct global_otp_struct hw_info_main2_otp;
#endif

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = SP2509V_SENSOR_ID,

	.checksum_value = 0x3ca80b08,        //checksum value for Camera Auto Test

	.pre = {
		.pclk = 36000000,            //record different mode's pclk
		.linelength = 961,            //record different mode's linelength
		.framelength = 1249,            //record different mode's framelength
		.startx = 0,                    //record different mode's startx of grabwindow
		.starty = 0,                    //record different mode's starty of grabwindow
		.grabwindow_width = 1600,        //record different mode's width of grabwindow
		.grabwindow_height = 1200,        //record different mode's height of grabwindow
		/*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*     following for GetDefaultFramerateByScenario()    */
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 36000000,
		.linelength = 961,
		.framelength = 1249,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,//4192,
		.grabwindow_height = 1200,//3104,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 36000000,
		.linelength = 961,
		.framelength = 1249,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,//4192,
		.grabwindow_height = 1200,//3104,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 36000000,
		.linelength = 961,
		.framelength = 1249,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,//4192,
		.grabwindow_height = 1200,//3104,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 36000000,
		.linelength = 961,
		.framelength = 1249,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,//4192,
		.grabwindow_height = 1200,//3104,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 36000000,
		.linelength = 961,
		.framelength = 1249,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,//4192,
		.grabwindow_height = 1200,//3104,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.custom1 = {	//Copy from cap
		.pclk = 36000000,
		.linelength = 946,
		.framelength = 1585,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,//4192,
		.grabwindow_height = 1200,//3104,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
	},
	.custom2 = {	//Copy from pre
		.pclk = 36000000,            //record different mode's pclk
		.linelength = 961,            //record different mode's linelength
		.framelength = 1249,            //record different mode's framelength
		.startx = 0,                    //record different mode's startx of grabwindow
		.starty = 0,                    //record different mode's starty of grabwindow
		.grabwindow_width = 1600,        //record different mode's width of grabwindow
		.grabwindow_height = 1200,        //record different mode's height of grabwindow
		/*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*     following for GetDefaultFramerateByScenario()    */
		.max_framerate = 300,
	},

	.margin = 4,            //sensor framelength & shutter margin
	.min_shutter = 1,        //min shutter
	.max_frame_length = 0x7fff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.frame_time_delay_frame = 1,//The delay frame of setting frame length
	.ihdr_support = 0,      //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 7,      //support sensor mode num

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	.custom1_delay_frame = 2,        //enter custom1 delay frame num
	.custom2_delay_frame = 2,         //enter custom2 delay frame num

	.isp_driving_current = ISP_DRIVING_4MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_1_LANE,//mipi lane num
	.i2c_addr_table = {0x7a, 0xff},
	.i2c_speed = 400
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,                //mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,                    //current shutter
	.gain = 0x100,                        //current gain
	.dummy_pixel = 0,                    //current dummypixel
	.dummy_line = 0,                    //current dummyline
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x7a,//record current sensor's i2c write id
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[7]=
{
	{  1600, 1200,  0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200,  0, 0, 1600, 1200}, // Preview
	{  1600, 1200,  0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200,  0, 0, 1600, 1200}, // capture
	{  1600, 1200,  0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200,  0, 0, 1600, 1200}, // video 
	{  1600, 1200,  0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200,  0, 0, 1600, 1200}, // hight speed video
	{  1600, 1200,  0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200,  0, 0, 1600, 1200}, // slim video
	{  1600, 1200,  0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200,  0, 0, 1600, 1200}, // custom1
	{  1600, 1200,  0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200,  0, 0, 1600, 1200}, // custom2
}; // slim video

#define MULTI_WRITE 1
#if MULTI_WRITE
#define I2C_BUFFER_LEN 1024
#else
#define I2C_BUFFER_LEN 2
#endif

static kal_uint16 sp2509_table_write_cmos_sensor(
					kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		if ((I2C_BUFFER_LEN - tosend) < 2 ||
			len == IDX ||
			addr != addr_last) {
			LOG_INF("sp2509_table_write_cmos_sensor multi write tosend=%d, len=%d, IDX=%d, addr=%x, addr_last=%x\n", tosend, len, IDX, addr, addr_last);
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id,
				2, imgsensor_info.i2c_speed);

			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 2, imgsensor.i2c_write_id);
		tosend = 0;
#endif
	}
	return 0;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = {(char)(addr & 0xFF)};

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

/* Xuegui.Bao@Camera.Driver, 2018/10/10, add for [summer otp bringup] */
#ifdef ENABLE_SP2509V_OTP
static kal_uint16 read_eeprom(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, EEPROM_FM34C32_ID);

	return get_byte;
}
#endif

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = {(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}

#undef DEBUG_SENSOR
#ifdef DEBUG_SENSOR
#define sp2509MIPI_OP_CODE_INI		0x00		/* Initial value. */
#define sp2509MIPI_OP_CODE_REG		0x01		/* Register */
#define sp2509MIPI_OP_CODE_DLY		0x02		/* Delay */
#define sp2509MIPI_OP_CODE_END		0x03		/* End of initial setting. */


typedef struct
{
	u16 init_reg;
	u16 init_val;	/* Save the register value and delay tick */
	u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
} sp2509MIPI_initial_set_struct;

sp2509MIPI_initial_set_struct sp2509MIPI_Init_Reg[1000];
static UINT32 fromsd;//gpwdebug

static u32 strtol(const char *nptr, u8 base)
{
	u8 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{
		if((base==16 && *nptr>='A' && *nptr<='F') || 
				(base==16 && *nptr>='a' && *nptr<='f') || 
				(base>=10 && *nptr>='0' && *nptr<='9') ||
				(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

u8 sp2509MIPI_Initialize_from_T_Flash(void)
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0, j = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */


	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static u8 data_buff[10*1024] ;

	fp = filp_open("/mnt/sdcard/sp2509_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) 
	{ 
		printk("create file error\n"); 
		return -1; 
	} 
	fs = get_fs(); 
	set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	//printk("%s %d %d\n", buf,iFileLen,pos); 
	filp_close(fp, NULL); 
	set_fs(fs);

	/* Start parse the setting witch read from t-flash. */
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size))
	{
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
		{
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
			{
				curr_ptr++;		/* Skip block comment code. */
			}

			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}

		if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
		{
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}
		/* This just content one enter line. */
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		//printk(" curr_ptr1 = %s\n",curr_ptr);
		memcpy(func_ind, curr_ptr, 3);


		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */
			sp2509MIPI_Init_Reg[i].op_code = sp2509MIPI_OP_CODE_REG;

			sp2509MIPI_Init_Reg[i].init_reg = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */

			sp2509MIPI_Init_Reg[i].init_val = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */

		}
		else 									/* DLY */
		{
			/* Need add delay for this setting. */
			curr_ptr += 4;	
			sp2509MIPI_Init_Reg[i].op_code = sp2509MIPI_OP_CODE_DLY;

			sp2509MIPI_Init_Reg[i].init_reg = 0xFF;
			sp2509MIPI_Init_Reg[i].init_val = strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
		}
		i++;


		/* Skip to next line directly. */
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}

	/* (0xFFFF, 0xFFFF) means the end of initial setting. */
	sp2509MIPI_Init_Reg[i].op_code = sp2509MIPI_OP_CODE_END;
	sp2509MIPI_Init_Reg[i].init_reg = 0xFF;
	sp2509MIPI_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
	//printk(" %x  ==  %x\n",sp2509MIPI_Init_Reg[j].init_reg, sp2509MIPI_Init_Reg[j].init_val);

	/* Start apply the initial setting to sensor. */
#if 1
	for (j=0; j<i; j++)
	{
		if (sp2509MIPI_Init_Reg[j].op_code == sp2509MIPI_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (sp2509MIPI_Init_Reg[j].op_code == sp2509MIPI_OP_CODE_DLY)
		{
			msleep(sp2509MIPI_Init_Reg[j].init_val);		/* Delay */
		}
		else if (sp2509MIPI_Init_Reg[j].op_code == sp2509MIPI_OP_CODE_REG)
		{

			write_cmos_sensor(sp2509MIPI_Init_Reg[j].init_reg, sp2509MIPI_Init_Reg[j].init_val);
			printk("%x = %x\n",sp2509MIPI_Init_Reg[j].init_reg,sp2509MIPI_Init_Reg[j].init_val);
		}
		else
		{
			printk("REG ERROR!\n");
		}
	}
#endif

	return 1;	
}
#endif

static kal_uint16 read_cmos_sensor0(kal_uint32 addr)
{
	return 0;
}

static void write_cmos_sensor0(kal_uint32 addr, kal_uint32 para)
{
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0x05, (imgsensor.frame_length - 0x4c8) >> 8);
	write_cmos_sensor(0x06, (imgsensor.frame_length - 0x4c8) & 0xFF);
	write_cmos_sensor(0x01, 0x01);
}    /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	write_cmos_sensor(0xfd, 0x00);
	return ((read_cmos_sensor(0x02) << 8) | read_cmos_sensor(0x03));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
	//imgsensor.dummy_line = 0;
	//else
	//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);

	set_dummy();
}    /*    set_max_framerate  */

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=SW Standby, 1=Streaming): %d\n", enable);

	if (enable) {
		write_cmos_sensor(0xfd,0x01);
		write_cmos_sensor(0xac,0x01);
	} else {
		write_cmos_sensor(0xfd,0x01);
		write_cmos_sensor(0xac,0x00);
	}

	return ERROR_NONE;
}

static void write_shutter(kal_uint16 shutter)
{
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0x03, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x04, shutter  & 0xFF); 
	write_cmos_sensor(0x01, 0x01); 
	LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
}

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	kal_uint16 realtime_fps = 0;

	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;

	if (frame_length > 1)
		imgsensor.frame_length = frame_length;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	//frame_length and shutter should be an even number.
	shutter = (shutter >> 1) << 1;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	//auroflicker:need to avoid 15fps and 30 fps
	if (imgsensor.autoflicker_en == KAL_TRUE) {
		realtime_fps = imgsensor.pclk /
			imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
		set_max_framerate(realtime_fps, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
		set_max_framerate(realtime_fps, 0);
		}
	}
	set_dummy();
	//Update Shutter
	if (shutter == (imgsensor.frame_length-1))
		shutter += 1;
	if(shutter < 4) shutter = 4;
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0x03, (shutter>>8) & 0xFF);
	write_cmos_sensor(0x04, shutter & 0xFF);
	write_cmos_sensor(0x01, 0x01);
	LOG_DBG("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
}

/*************************************************************************
 * FUNCTION
 *    set_shutter
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}
#if 0
static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	reg_gain = ((gain / BASEGAIN) << 4) + ((gain % BASEGAIN) * 16 / BASEGAIN);
	reg_gain = reg_gain & 0xFFFF;

	return (kal_uint16)reg_gain;
}
#endif
/*************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint8  iReg;

	if(gain >= BASEGAIN && gain <= 15*BASEGAIN)
	{   
		iReg = 0x10 * gain/BASEGAIN;        //change mtk gain base to aptina gain base              

		if(iReg<=0x10)
		{
			write_cmos_sensor(0xfd, 0x01);
			write_cmos_sensor(0x24, 0x10);//0x23
			write_cmos_sensor(0x01, 0x01);
			LOG_INF("SP2509MIPI_SetGain = 16");
		}
		else if(iReg>= 0xa0)//gpw
		{
			write_cmos_sensor(0xfd, 0x01);
			write_cmos_sensor(0x24,0xa0);
			write_cmos_sensor(0x01, 0x01); 
			LOG_INF("SP2509MIPI_SetGain = 160"); 
		}        	
		else
		{
			write_cmos_sensor(0xfd, 0x01);
			write_cmos_sensor(0x24, (kal_uint8)iReg);
			write_cmos_sensor(0x01, 0x01);
			LOG_INF("SP2509MIPI_SetGain = %d",iReg);		 
		}	
	}	
	else
		LOG_INF("error gain setting");	 

	return gain;
}    /*    set_gain  */

/*
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	//not support HDR
	//LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
}
*/

static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	 *

	 *   ISP and Sensor flip or mirror register bit should be the same!!
	 *
	 ********************************************************/

	switch (image_mirror) {
	case IMAGE_NORMAL:
        write_cmos_sensor(0x3f,0x00);
		break;
	case IMAGE_H_MIRROR:
        write_cmos_sensor(0x3f,0x01);
		break;
	case IMAGE_V_MIRROR:
        write_cmos_sensor(0x3f,0x02);
		break;
	case IMAGE_HV_MIRROR:
        write_cmos_sensor(0x3f,0x03);
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}

}

/*************************************************************************
 * FUNCTION
 *    night_mode
 *
 * DESCRIPTION
 *    This function night mode of sensor.
 *
 * PARAMETERS
 *    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function*/
}    /*    night_mode    */

#if MULTI_WRITE
kal_uint16 addr_data_pair_init_sp2509[] = {
	0xfd,0x00,
	0x2f,0x0c,
	0x34,0x00,
	0x35,0x21,
	0x30,0x1d,
	0x33,0x05,
	0xfd,0x01,
	0x44,0x00,
	0x2a,0x4c,
	0x2b,0x1e,
	0x2c,0x60,
	0x25,0x11,
	0x03,0x04,
	0x04,0x74,
	0x09,0x00,
	0x0a,0x10,
	0x05,0x00,
	0x06,0x19,
	0x24,0x20,
	0x01,0x01,
	0xfb,0x73,
	0xfd,0x01,
	0x16,0x04,
	0x1c,0x09,
	0x21,0x42,
	0x6c,0x00,
	0x6b,0x00,
	0x84,0x00,
	0x85,0x0c,
	0x86,0x0c,
	0x87,0x00,
	0x12,0x04,
	0x13,0x40,
	0x11,0x20,
	0x33,0x40,
	0xd0,0x03,
	0xd1,0x01,
	0xd2,0x00,
	0xd3,0x01,
	0xd4,0x20,
	0x50,0x00,
	0x51,0x14,
	0x52,0x10,
	0x55,0x30,
	0x58,0x10,
	0x71,0x10,
	0x6f,0x40,
	0x75,0x60,
	0x76,0x34,
	0x8a,0x22,
	0x8b,0x22,
	0x19,0xf1,
	0x29,0x01,
	0xfd,0x01,
	0x9d,0xd9,
	0xa0,0x29,
	0xa1,0x04,
	0xad,0x62,
	0xae,0x00,
	0xaf,0x85,
	0xb1,0x01,
	0xac,0x01,
	0xfd,0x01,
	0xf0,0xfd,
	0xf1,0xfd,
	0xf2,0xfd,
	0xf3,0xfd,
	0xf4,0x00,
	0xf5,0x00,
	0xf6,0x00,
	0xf7,0x00,
	0xfc,0x10,
	0xfe,0x10,
	0xf9,0x00,
	0xfa,0x00,
	0x8e,0x06,
	0x8f,0x40,
	0x90,0x04,
	0x91,0xb0,
	0x45,0x01,
	0x46,0x00,
	0x47,0x6c,
	0x48,0x03,
	0x49,0x8b,
	0x4a,0x00,
	0x4b,0x07,
	0x4c,0x04,
	0x4d,0xb7,
	0xfd,0x00,
	0x1f,0x01,
	0x40,0x01,
	0xfd,0x01,
	0x0b,0x02,
};
#endif

static void sensor_init(void)
{
#ifdef DEBUG_SENSOR
	if(fromsd == 1)//是否从SD读取//gepeiwei   120903
	{
		LOG_INF("preview_setting1\n");
		sp2509MIPI_Initialize_from_T_Flash();//从SD卡读取的主要函数

	}
	else
#endif
	{
		#if MULTI_WRITE
		LOG_INF("sensor_init multi write\n");
		sp2509_table_write_cmos_sensor(
			addr_data_pair_init_sp2509,
			sizeof(addr_data_pair_init_sp2509) / sizeof(kal_uint16));
		#else
		LOG_INF("sensor_init normal write\n");
		write_cmos_sensor(0xfd,0x00);
		write_cmos_sensor(0x2f,0x0c);
		write_cmos_sensor(0x34,0x00);
		write_cmos_sensor(0x35,0x21);
		write_cmos_sensor(0x30,0x1d);
		write_cmos_sensor(0x33,0x05);
		write_cmos_sensor(0xfd,0x01);
		write_cmos_sensor(0x44,0x00);
		write_cmos_sensor(0x2a,0x4c);//9.29canshu 7c
		write_cmos_sensor(0x2b,0x1e);
		write_cmos_sensor(0x2c,0x60);
		write_cmos_sensor(0x25,0x11);
		write_cmos_sensor(0x03,0x04);
		write_cmos_sensor(0x04,0x74);
		write_cmos_sensor(0x09,0x00);
		write_cmos_sensor(0x0a,0x10);
		write_cmos_sensor(0x05,0x00);
		write_cmos_sensor(0x06,0x19);
		write_cmos_sensor(0x24,0x20);
		write_cmos_sensor(0x01,0x01);
		write_cmos_sensor(0xfb,0x73);//9.29canshu 63
		write_cmos_sensor(0xfd,0x01);
		write_cmos_sensor(0x16,0x04);
		write_cmos_sensor(0x1c,0x09);
		write_cmos_sensor(0x21,0x42);
		write_cmos_sensor(0x6c,0x00);
		write_cmos_sensor(0x6b,0x00);
		write_cmos_sensor(0x84,0x00);
		write_cmos_sensor(0x85,0x0c);
		write_cmos_sensor(0x86,0x0c);
		write_cmos_sensor(0x87,0x00);
		write_cmos_sensor(0x12,0x04);
		write_cmos_sensor(0x13,0x40);
		write_cmos_sensor(0x11,0x20);
		write_cmos_sensor(0x33,0x40);
		write_cmos_sensor(0xd0,0x03);
		write_cmos_sensor(0xd1,0x01);
		write_cmos_sensor(0xd2,0x00);
		write_cmos_sensor(0xd3,0x01);
		write_cmos_sensor(0xd4,0x20);
		write_cmos_sensor(0x50,0x00);
		write_cmos_sensor(0x51,0x14);
		write_cmos_sensor(0x52,0x10);
		write_cmos_sensor(0x55,0x30);
		write_cmos_sensor(0x58,0x10);
		write_cmos_sensor(0x71,0x10);
		write_cmos_sensor(0x6f,0x40);
		write_cmos_sensor(0x75,0x60);
		write_cmos_sensor(0x76,0x34);
		write_cmos_sensor(0x8a,0x22);
		write_cmos_sensor(0x8b,0x22);
		write_cmos_sensor(0x19,0xf1);
		write_cmos_sensor(0x29,0x01);
		write_cmos_sensor(0xfd,0x01);
		write_cmos_sensor(0x9d,0xd9);//ea
		write_cmos_sensor(0xa0,0x29);
		write_cmos_sensor(0xa1,0x04);
		write_cmos_sensor(0xad,0x62);//
		write_cmos_sensor(0xae,0x00);//
		write_cmos_sensor(0xaf,0x85);//
		write_cmos_sensor(0xb1,0x01);
		write_cmos_sensor(0xac,0x01);
		write_cmos_sensor(0xfd,0x01);
		write_cmos_sensor(0xf0,0xfd);
		write_cmos_sensor(0xf1,0xfd);
		write_cmos_sensor(0xf2,0xfd);
		write_cmos_sensor(0xf3,0xfd);
		write_cmos_sensor(0xf4,0x00);
		write_cmos_sensor(0xf5,0x00);
		write_cmos_sensor(0xf6,0x00);
		write_cmos_sensor(0xf7,0x00);
		write_cmos_sensor(0xfc,0x10);
		write_cmos_sensor(0xfe,0x10); // 9.29wu 
		write_cmos_sensor(0xf9,0x00); // 9.29wu
		write_cmos_sensor(0xfa,0x00); // 9.29wu  

		write_cmos_sensor(0x8e,0x06);
		write_cmos_sensor(0x8f,0x40);
		write_cmos_sensor(0x90,0x04);
		write_cmos_sensor(0x91,0xb0);
		write_cmos_sensor(0x45,0x01);
		write_cmos_sensor(0x46,0x00);
		write_cmos_sensor(0x47,0x6c);
		write_cmos_sensor(0x48,0x03);
		write_cmos_sensor(0x49,0x8b);
		write_cmos_sensor(0x4a,0x00);
		write_cmos_sensor(0x4b,0x07);
		write_cmos_sensor(0x4c,0x04);
		write_cmos_sensor(0x4d,0xb7);
		write_cmos_sensor(0xfd,0x00);
		write_cmos_sensor(0x1f,0x01);
		write_cmos_sensor(0x40,0x01);
		write_cmos_sensor(0xfd,0x01);
		write_cmos_sensor(0x0b,0x02);
		#endif
	}                   

}    /*    MIPI_sensor_Init  */


static void preview_setting(void)
{
	///write_cmos_sensor(0xfd,0x01);
	//write_cmos_sensor(0xac,0x01);
	/********************************************************
	 *
	 *   1296x972 30fps 2 lane MIPI 420Mbps/lane
	 *
	 ********************************************************/


}    /*    preview_setting  */


static void capture_setting(kal_uint16 currefps)
{
	//write_cmos_sensor(0xfd,0x01);
	//write_cmos_sensor(0xac,0x01);

}    /*    capture_setting  */

static void normal_video_setting(kal_uint16 currefps)
{
	//write_cmos_sensor(0xfd,0x01);
	//write_cmos_sensor(0xac,0x01);

}    /*    preview_setting  */


static void video_1080p_setting(void)
{
	//write_cmos_sensor(0xfd,0x01);
	//write_cmos_sensor(0xac,0x01);

}    /*    preview_setting  */

static void video_720p_setting(void)
{
	//write_cmos_sensor(0xfd,0x01);
	//write_cmos_sensor(0xac,0x01);
}    /*    preview_setting  */


static void hs_video_setting(void)
{
	LOG_INF("E\n");
	video_1080p_setting();
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	video_720p_setting();
}


#if MULTI_WRITE
kal_uint16 addr_data_pair_custom1_sp2509[] = {
	0xfd,0x00,
	0x2f,0x0c,
	0x34,0x00,
	0x35,0x21,
	0x30,0x1d,
	0x33,0x05,
	0xfd,0x01,
	0x44,0x00,
	0x2a,0x4c,
	0x2b,0x1e,
	0x2c,0x60,
	0x25,0x11,
	0x03,0x04,
	0x04,0x74,
	0x09,0x00,
	0x0a,0x0a,
	0x05,0x01,
	0x06,0x69,
	0x24,0x20,
	0x01,0x01,
	0xfb,0x73,
	0xfd,0x01,
	0x16,0x04,
	0x1c,0x09,
	0x21,0x42,
	0x6c,0x00,
	0x6b,0x00,
	0x84,0x00,
	0x85,0x0c,
	0x86,0x0c,
	0x87,0x00,
	0x12,0x04,
	0x13,0x40,
	0x11,0x20,
	0x33,0x40,
	0xd0,0x03,
	0xd1,0x01,
	0xd2,0x00,
	0xd3,0x01,
	0xd4,0x20,
	0x50,0x00,
	0x51,0x14,
	0x52,0x10,
	0x55,0x30,
	0x58,0x10,
	0x71,0x10,
	0x6f,0x40,
	0x75,0x60,
	0x76,0x34,
	0x8a,0x22,
	0x8b,0x22,
	0x19,0xf1,
	0x29,0x01,
	0xfd,0x01,
	0x9d,0xea,
	0xa0,0x29,
	0xa1,0x04,
	0xad,0x62,
	0xae,0x00,
	0xaf,0x85,
	0xb1,0x01,
	0xac,0x01,
	0xfd,0x01,
	0xf0,0xfd,
	0xf1,0xfd,
	0xf2,0xfd,
	0xf3,0xfd,
	0xf4,0x00,
	0xf5,0x00,
	0xf6,0x00,
	0xf7,0x00,
	0xfc,0x10,
	0xfe,0x10,
	0xf9,0x00,
	0xfa,0x00,
	0x8e,0x06,
	0x8f,0x40,
	0x90,0x04,
	0x91,0xb0,
	0x45,0x01,
	0x46,0x00,
	0x47,0x6c,
	0x48,0x03,
	0x49,0x8b,
	0x4a,0x00,
	0x4b,0x07,
	0x4c,0x04,
	0x4d,0xb7,
	0xfd,0x00,
	0x1f,0x01,
	0x40,0x01,
	0xfd,0x01,
	0x0b,0x02,
};
#endif

static void custom1_setting(kal_uint16 currefps)
{
    LOG_INF("E\n");
#if MULTI_WRITE
	LOG_INF("custom1_setting multi write\n");
	sp2509_table_write_cmos_sensor(
		addr_data_pair_custom1_sp2509,
		sizeof(addr_data_pair_custom1_sp2509) / sizeof(kal_uint16));
#else
	LOG_INF("custom1_setting normal write\n");
	write_cmos_sensor(0xfd,0x00);
	write_cmos_sensor(0x2f,0x0c);
	write_cmos_sensor(0x34,0x00);
	write_cmos_sensor(0x35,0x21);
	write_cmos_sensor(0x30,0x1d);
	write_cmos_sensor(0x33,0x05);
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0x44,0x00);
	write_cmos_sensor(0x2a,0x4c);//9.29canshu 7c
	write_cmos_sensor(0x2b,0x1e);
	write_cmos_sensor(0x2c,0x60);
	write_cmos_sensor(0x25,0x11);
	write_cmos_sensor(0x03,0x04);
	write_cmos_sensor(0x04,0x74);
	write_cmos_sensor(0x09,0x00);
	write_cmos_sensor(0x0a,0x0a);
	write_cmos_sensor(0x05,0x01);
	write_cmos_sensor(0x06,0x58);
	write_cmos_sensor(0x24,0x20);
	write_cmos_sensor(0x01,0x01);
	write_cmos_sensor(0xfb,0x73);//9.29canshu 63
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0x16,0x04);
	write_cmos_sensor(0x1c,0x09);
	write_cmos_sensor(0x21,0x42);
	write_cmos_sensor(0x6c,0x00);
	write_cmos_sensor(0x6b,0x00);
	write_cmos_sensor(0x84,0x00);
	write_cmos_sensor(0x85,0x0c);
	write_cmos_sensor(0x86,0x0c);
	write_cmos_sensor(0x87,0x00);
	write_cmos_sensor(0x12,0x04);
	write_cmos_sensor(0x13,0x40);
	write_cmos_sensor(0x11,0x20);
	write_cmos_sensor(0x33,0x40);
	write_cmos_sensor(0xd0,0x03);
	write_cmos_sensor(0xd1,0x01);
	write_cmos_sensor(0xd2,0x00);
	write_cmos_sensor(0xd3,0x01);
	write_cmos_sensor(0xd4,0x20);
	write_cmos_sensor(0x50,0x00);
	write_cmos_sensor(0x51,0x14);
	write_cmos_sensor(0x52,0x10);
	write_cmos_sensor(0x55,0x30);
	write_cmos_sensor(0x58,0x10);
	write_cmos_sensor(0x71,0x10);
	write_cmos_sensor(0x6f,0x40);
	write_cmos_sensor(0x75,0x60);
	write_cmos_sensor(0x76,0x34);
	write_cmos_sensor(0x8a,0x22);
	write_cmos_sensor(0x8b,0x22);
	write_cmos_sensor(0x19,0xf1);
	write_cmos_sensor(0x29,0x01);
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0x9d,0xea);
	write_cmos_sensor(0xa0,0x29);
	write_cmos_sensor(0xa1,0x04);
	write_cmos_sensor(0xad,0x62);//
	write_cmos_sensor(0xae,0x00);//
	write_cmos_sensor(0xaf,0x85);//
	write_cmos_sensor(0xb1,0x01);
	write_cmos_sensor(0xac,0x01);
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0xf0,0xfd);
	write_cmos_sensor(0xf1,0xfd);
	write_cmos_sensor(0xf2,0xfd);
	write_cmos_sensor(0xf3,0xfd);
	write_cmos_sensor(0xf4,0x00);
	write_cmos_sensor(0xf5,0x00);
	write_cmos_sensor(0xf6,0x00);
	write_cmos_sensor(0xf7,0x00);
	write_cmos_sensor(0xfc,0x10);
	write_cmos_sensor(0xfe,0x10);
	write_cmos_sensor(0xf9,0x00);
	write_cmos_sensor(0xfa,0x00);
	write_cmos_sensor(0x8e,0x06);
	write_cmos_sensor(0x8f,0x40);
	write_cmos_sensor(0x90,0x04);
	write_cmos_sensor(0x91,0xb0);
	write_cmos_sensor(0x45,0x01);
	write_cmos_sensor(0x46,0x00);
	write_cmos_sensor(0x47,0x6c);
	write_cmos_sensor(0x48,0x03);
	write_cmos_sensor(0x49,0x8b);
	write_cmos_sensor(0x4a,0x00);
	write_cmos_sensor(0x4b,0x07);
	write_cmos_sensor(0x4c,0x04);
	write_cmos_sensor(0x4d,0xb7);
	write_cmos_sensor(0xfd,0x00);
	write_cmos_sensor(0x1f,0x01);
	write_cmos_sensor(0x40,0x01);
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0x0b,0x02);
#endif
}    /* custom1_setting */

#if MULTI_WRITE
kal_uint16 addr_data_pair_custom2_sp2509[] = {
	0xfd,0x00,
	0x2f,0x0c,
	0x34,0x00,
	0x35,0x21,
	0x30,0x1d,
	0x33,0x05,
	0xfd,0x01,
	0x44,0x00,
	0x2a,0x4c,
	0x2b,0x1e,
	0x2c,0x60,
	0x25,0x11,
	0x03,0x04,
	0x04,0x74,
	0x09,0x00,
	0x0a,0x10,
	0x05,0x00,
	0x06,0x19,
	0x24,0x20,
	0x01,0x01,
	0xfb,0x73,
	0xfd,0x01,
	0x16,0x04,
	0x1c,0x09,
	0x21,0x42,
	0x6c,0x00,
	0x6b,0x00,
	0x84,0x00,
	0x85,0x0c,
	0x86,0x0c,
	0x87,0x00,
	0x12,0x04,
	0x13,0x40,
	0x11,0x20,
	0x33,0x40,
	0xd0,0x03,
	0xd1,0x01,
	0xd2,0x00,
	0xd3,0x01,
	0xd4,0x20,
	0x50,0x00,
	0x51,0x14,
	0x52,0x10,
	0x55,0x30,
	0x58,0x10,
	0x71,0x10,
	0x6f,0x40,
	0x75,0x60,
	0x76,0x34,
	0x8a,0x22,
	0x8b,0x22,
	0x19,0xf1,
	0x29,0x01,
	0xfd,0x01,
	0x9d,0xea,
	0xa0,0x29,
	0xa1,0x04,
	0xad,0x62,
	0xae,0x00,
	0xaf,0x85,
	0xb1,0x01,
	0xac,0x01,
	0xfd,0x01,
	0xf0,0xfd,
	0xf1,0xfd,
	0xf2,0xfd,
	0xf3,0xfd,
	0xf4,0x00,
	0xf5,0x00,
	0xf6,0x00,
	0xf7,0x00,
	0xfc,0x10,
	0xfe,0x10,
	0xf9,0x00,
	0xfa,0x00,
	0x8e,0x06,
	0x8f,0x40,
	0x90,0x04,
	0x91,0xb0,
	0x45,0x01,
	0x46,0x00,
	0x47,0x6c,
	0x48,0x03,
	0x49,0x8b,
	0x4a,0x00,
	0x4b,0x07,
	0x4c,0x04,
	0x4d,0xb7,
	0xfd,0x00,
	0x1f,0x01,
	0x40,0x01,
	0xfd,0x01,
	0x0b,0x02,
};
#endif

static void custom2_setting(void)
{
#if MULTI_WRITE
	LOG_INF("custom2_setting multi write\n");
	sp2509_table_write_cmos_sensor(
		addr_data_pair_custom2_sp2509,
		sizeof(addr_data_pair_custom2_sp2509) / sizeof(kal_uint16));
#else
	LOG_INF("custom2_setting normal write\n");
	write_cmos_sensor(0xfd,0x00);
	write_cmos_sensor(0x2f,0x0c);
	write_cmos_sensor(0x34,0x00);
	write_cmos_sensor(0x35,0x21);
	write_cmos_sensor(0x30,0x1d);
	write_cmos_sensor(0x33,0x05);
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0x44,0x00);
	write_cmos_sensor(0x2a,0x4c);//9.29canshu 7c
	write_cmos_sensor(0x2b,0x1e);
	write_cmos_sensor(0x2c,0x60);
	write_cmos_sensor(0x25,0x11);
	write_cmos_sensor(0x03,0x04);
	write_cmos_sensor(0x04,0x74);
	write_cmos_sensor(0x09,0x00);
	write_cmos_sensor(0x0a,0x10);
	write_cmos_sensor(0x05,0x00);
	write_cmos_sensor(0x06,0x19);
	write_cmos_sensor(0x24,0x20);
	write_cmos_sensor(0x01,0x01);
	write_cmos_sensor(0xfb,0x73);//9.29canshu 63
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0x16,0x04);
	write_cmos_sensor(0x1c,0x09);
	write_cmos_sensor(0x21,0x42);
	write_cmos_sensor(0x6c,0x00);
	write_cmos_sensor(0x6b,0x00);
	write_cmos_sensor(0x84,0x00);
	write_cmos_sensor(0x85,0x0c);
	write_cmos_sensor(0x86,0x0c);
	write_cmos_sensor(0x87,0x00);
	write_cmos_sensor(0x12,0x04);
	write_cmos_sensor(0x13,0x40);
	write_cmos_sensor(0x11,0x20);
	write_cmos_sensor(0x33,0x40);
	write_cmos_sensor(0xd0,0x03);
	write_cmos_sensor(0xd1,0x01);
	write_cmos_sensor(0xd2,0x00);
	write_cmos_sensor(0xd3,0x01);
	write_cmos_sensor(0xd4,0x20);
	write_cmos_sensor(0x50,0x00);
	write_cmos_sensor(0x51,0x14);
	write_cmos_sensor(0x52,0x10);
	write_cmos_sensor(0x55,0x30);
	write_cmos_sensor(0x58,0x10);
	write_cmos_sensor(0x71,0x10);
	write_cmos_sensor(0x6f,0x40);
	write_cmos_sensor(0x75,0x60);
	write_cmos_sensor(0x76,0x34);
	write_cmos_sensor(0x8a,0x22);
	write_cmos_sensor(0x8b,0x22);
	write_cmos_sensor(0x19,0xf1);
	write_cmos_sensor(0x29,0x01);
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0x9d,0xea);
	write_cmos_sensor(0xa0,0x29);
	write_cmos_sensor(0xa1,0x04);
	write_cmos_sensor(0xad,0x62);//
	write_cmos_sensor(0xae,0x00);//
	write_cmos_sensor(0xaf,0x85);//
	write_cmos_sensor(0xb1,0x01);
	write_cmos_sensor(0xac,0x01);
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0xf0,0xfd);
	write_cmos_sensor(0xf1,0xfd);
	write_cmos_sensor(0xf2,0xfd);
	write_cmos_sensor(0xf3,0xfd);
	write_cmos_sensor(0xf4,0x00);
	write_cmos_sensor(0xf5,0x00);
	write_cmos_sensor(0xf6,0x00);
	write_cmos_sensor(0xf7,0x00);
	write_cmos_sensor(0xfc,0x10);
	write_cmos_sensor(0xfe,0x10);
	write_cmos_sensor(0xf9,0x00);
	write_cmos_sensor(0xfa,0x00);
	write_cmos_sensor(0x8e,0x06);
	write_cmos_sensor(0x8f,0x40);
	write_cmos_sensor(0x90,0x04);
	write_cmos_sensor(0x91,0xb0);
	write_cmos_sensor(0x45,0x01);
	write_cmos_sensor(0x46,0x00);
	write_cmos_sensor(0x47,0x6c);
	write_cmos_sensor(0x48,0x03);
	write_cmos_sensor(0x49,0x8b);
	write_cmos_sensor(0x4a,0x00);
	write_cmos_sensor(0x4b,0x07);
	write_cmos_sensor(0x4c,0x04);
	write_cmos_sensor(0x4d,0xb7);
	write_cmos_sensor(0xfd,0x00);
	write_cmos_sensor(0x1f,0x01);
	write_cmos_sensor(0x40,0x01);
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0x0b,0x02);
#endif
}    /* custom2_setting */

/*************************************************************************
 * FUNCTION
 *    get_imgsensor_id
 *
 * DESCRIPTION
 *    This function get the sensor ID
 *
 * PARAMETERS
 *    *sensorID : return the sensor ID
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
#include "../imgsensor_i2c.h"
#define SP2509V_I2CBUS    (4)
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	int I2C_bus = 0;
/* Xuegui.Bao@Camera.Driver, 2018/10/10, add for [summer otp bringup] */
#ifdef ENABLE_SP2509V_OTP
	kal_int16 vendorId = 0;
#endif

	I2C_bus = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
	LOG_INF("SP2509V_I2CBUS = %d, I2C_bus = %d\n", SP2509V_I2CBUS, I2C_bus);
	if (I2C_bus != SP2509V_I2CBUS) {
		*sensor_id = 0xFFFFFFFF;
		pr_err("SP2509V_I2CBUS: %d, I2C_bus = %d, Check Error!\n", SP2509V_I2CBUS, I2C_bus);
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);

		i++;
		do {
			*sensor_id = return_sensor_id();

			if (*sensor_id == imgsensor_info.sensor_id) {
				hardwareinfo_set_prop(HARDWARE_BACK_SUB_CAM_MOUDULE_ID, "helitai");
/* Xuegui.Bao@Camera.Driver, 2018/10/10, add for [summer otp bringup] */
#ifdef ENABLE_SP2509V_OTP
				vendorId=read_eeprom(0x0001);
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x vendorID=0x%x\n",
						imgsensor.i2c_write_id, *sensor_id,vendorId);

				if (vendorId==0x55){
					hw_info_main2_otp.sensor_name = SENSOR_DRVNAME_SP2509V_MIPI_RAW;
					return ERROR_NONE;

				}else{
					pr_err("invalid vendorId!Not error vendorId=0x%x\n",vendorId);
					*sensor_id = 0xFFFFFFFF;
				}
#endif
			}
		    pr_err("Read sensor id fail, i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);
			retry--;
		} while(retry > 0);

		retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *    open
 *
 * DESCRIPTION
 *    This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

#ifdef DEBUG_SENSOR
	struct file *fp;
	mm_segment_t fs;
	//loff_t pos = 0;
	//static char buf[10*1024] ;
#endif

	LOG_1;
	LOG_2;

	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("gpw i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("gpw Read sensor id fail, write id:0x%x id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
#ifdef DEBUG_SENSOR  //
	fp = filp_open("/mnt/sdcard/sp2509_sd", O_RDONLY , 0); 
	fs = get_fs();
	if (IS_ERR(fp)) { 
		fromsd = 0;   
		printk("gpww open file error\n");
		//return 0;
	} 	

	else 
	{
		fromsd = 1;
		//sp2509MIPI_Initialize_from_T_Flash();
		printk("gpww read ok!\n");

		filp_close(fp, NULL); 
		set_fs(fs);
	}
#endif

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}    /*    open  */



/*************************************************************************
 * FUNCTION
 *    close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}    /*    close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *    This function start the sensor preview.
 *
 * PARAMETERS
 *    *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}    /*    preview   */

/*************************************************************************
 * FUNCTION
 *    capture
 *
 * DESCRIPTION
 *    This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}    /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);

	return ERROR_NONE;
}    /*    slim_video     */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting(imgsensor.current_fps);
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	return ERROR_NONE;
}/* custom1() */

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	return ERROR_NONE;
}/* preview */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height = imgsensor_info.custom2.grabwindow_height;

	return ERROR_NONE;
}    /*    get_resolution    */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_INFO_STRUCT *sensor_info,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		custom2(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if(framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default:  //coding with  preview scenario by default
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
	default:
		break;
	}

	return ERROR_NONE;
}



static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if(enable)
	{
		write_cmos_sensor(0xfd,0x01);
		write_cmos_sensor(0x0d,0x01);
	}
	else
	{
		write_cmos_sensor(0xfd,0x01);
		write_cmos_sensor(0x0d,0x00);
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 get_sensor_temperature(void)
{
    INT32 temperature_convert = 25;
    return temperature_convert;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
		UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data=(unsigned long long *) feature_para;
	//unsigned long long *feature_return_para=(unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16)*feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor0(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor0(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
		// if EEPROM does not exist in camera module.
		*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len=4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len=4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_16;
		spin_unlock(&imgsensor_drv_lock);
        	LOG_INF("current fps :%d\n", imgsensor.current_fps);
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(
			(UINT16) *feature_data, (UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_SET_HDR:
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[6],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		//LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
		//ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_DBG("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_DBG("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = get_sensor_temperature();
		*feature_para_len = 4;
	break;
	default:
		break;
	}

	return ERROR_NONE;
}    /*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 SP2509V_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}    /*    SP2509VMIPISensorInit    */
