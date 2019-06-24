/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - userspce.c
** Description : This program is for ili9881 driver userspace.c
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#include "common.h"
#include "platform.h"
#include "core/config.h"
#include "core/firmware.h"
#include "core/finger_report.h"
#include "core/flash.h"
#include "core/i2c.h"
#include "core/protocol.h"
#include "core/mp_test.h"
#include "core/parser.h"
#include "core/gesture.h"

#define USER_STR_BUFF	128
#define ILITEK_IOCTL_MAGIC	100
#define ILITEK_IOCTL_MAXNR	19

#define ILITEK_IOCTL_I2C_WRITE_DATA			_IOWR(ILITEK_IOCTL_MAGIC, 0, uint8_t*)
#define ILITEK_IOCTL_I2C_SET_WRITE_LENGTH	_IOWR(ILITEK_IOCTL_MAGIC, 1, int)
#define ILITEK_IOCTL_I2C_READ_DATA			_IOWR(ILITEK_IOCTL_MAGIC, 2, uint8_t*)
#define ILITEK_IOCTL_I2C_SET_READ_LENGTH	_IOWR(ILITEK_IOCTL_MAGIC, 3, int)

#define ILITEK_IOCTL_TP_HW_RESET			_IOWR(ILITEK_IOCTL_MAGIC, 4, int)
#define ILITEK_IOCTL_TP_POWER_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 5, int)
#define ILITEK_IOCTL_TP_REPORT_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 6, int)
#define ILITEK_IOCTL_TP_IRQ_SWITCH			_IOWR(ILITEK_IOCTL_MAGIC, 7, int)

#define ILITEK_IOCTL_TP_DEBUG_LEVEL			_IOWR(ILITEK_IOCTL_MAGIC, 8, int)
#define ILITEK_IOCTL_TP_FUNC_MODE			_IOWR(ILITEK_IOCTL_MAGIC, 9, int)

#define ILITEK_IOCTL_TP_FW_VER				_IOWR(ILITEK_IOCTL_MAGIC, 10, uint8_t*)
#define ILITEK_IOCTL_TP_PL_VER				_IOWR(ILITEK_IOCTL_MAGIC, 11, uint8_t*)
#define ILITEK_IOCTL_TP_CORE_VER			_IOWR(ILITEK_IOCTL_MAGIC, 12, uint8_t*)
#define ILITEK_IOCTL_TP_DRV_VER				_IOWR(ILITEK_IOCTL_MAGIC, 13, uint8_t*)
#define ILITEK_IOCTL_TP_CHIP_ID				_IOWR(ILITEK_IOCTL_MAGIC, 14, uint32_t*)

#define ILITEK_IOCTL_TP_NETLINK_CTRL		_IOWR(ILITEK_IOCTL_MAGIC, 15, int*)
#define ILITEK_IOCTL_TP_NETLINK_STATUS		_IOWR(ILITEK_IOCTL_MAGIC, 16, int*)

#define ILITEK_IOCTL_TP_MODE_CTRL			_IOWR(ILITEK_IOCTL_MAGIC, 17, uint8_t*)
#define ILITEK_IOCTL_TP_MODE_STATUS			_IOWR(ILITEK_IOCTL_MAGIC, 18, int*)
#define ILITEK_IOCTL_ICE_MODE_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 19, int)

#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
#define X_CHANGLE 18
#define Y_CHANGLE 32
#define FRAME_NODE 18*32
unsigned char g_user_buf[USER_STR_BUFF] = { 0 };
unsigned char g_buf_to_user[10] = { 0 };
unsigned char g_user_len =0;
uint32_t buf_rawdata[FRAME_NODE] ={0};
int32_t buf_delta[FRAME_NODE] ={0};
uint8_t buf_gesture[GESTURE_INFO_LENGTH+1];

int gesture_test_flag = 0;
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/10/30,Add for TP black screen test
extern int g_gesture;
#endif
static int lcm_gesture_power = 0;
extern bool mp_test_result;
extern bool gesture_done;
bool psensor_close = false;

#ifdef ILITEK_ESD_CHECK
extern void ilitek_cancel_esd_check(void);
extern void ilitek_start_esd_check(void);
#endif
extern struct ili_gesture_info * gesture_report_data;
extern int request_firmware_select(const struct firmware **firmware_p, const char *name,struct device *device);
const struct firmware *ili_fw_oppo = NULL;

bool ILITEK_PROC_SEND_FLAG =false;
#endif


uint32_t otp_wrie_contol_leave = 0;

EXPORT_SYMBOL(otp_wrie_contol_leave);

int katoi(char *string)
{
	int result = 0;
	unsigned int digit;
	int sign;

	if (*string == '-') {
		sign = 1;
		string += 1;
	} else {
		sign = 0;
		if (*string == '+') {
			string += 1;
		}
	}

	for (;; string += 1) {
		digit = *string - '0';
		if (digit > 9)
			break;
		result = (10 * result) + digit;
	}

	if (sign) {
		return -result;
	}
	return result;
}
EXPORT_SYMBOL(katoi);

int str2hex(char *str)
{
	int strlen, result, intermed, intermedtop;
	char *s = str;

	while (*s != 0x0) {
		s++;
	}

	strlen = (int)(s - str);
	s = str;
	if (*s != 0x30) {
		return -1;
	}

	s++;

	if (*s != 0x78 && *s != 0x58) {
		return -1;
	}
	s++;

	strlen = strlen - 3;
	result = 0;
	while (*s != 0x0) {
		intermed = *s & 0x0f;
		intermedtop = *s & 0xf0;
		if (intermedtop == 0x60 || intermedtop == 0x40) {
			intermed += 0x09;
		}
		intermed = intermed << (strlen << 2);
		result = result | intermed;
		strlen -= 1;
		s++;
	}
	return result;
}
EXPORT_SYMBOL(str2hex);

static ssize_t ili9881_proc_fw_pc_counter_read(struct file *pFile, char __user *buff, size_t nCount, loff_t *pPos)
{
	int ret = 0;
	uint32_t pc_count = 0x0;

	if (*pPos != 0)
		return 0;

	memset(g_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));

	mutex_lock(&ipd->report_mutex);

	pc_count = ili9881_read_pc_counter();

	mutex_unlock(&ipd->report_mutex);

	nCount = snprintf(g_user_buf, PAGE_SIZE, "pc counter = 0x%x\n", pc_count);

	*pPos += nCount;

	ret = copy_to_user(buff, g_user_buf, nCount);
	if (ret < 0) {
		ipio_err("Failed to copy data to user space");
	}

	return nCount;
}
static ssize_t ilitek_proc_otp_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
    int i;
    u8 otp[512] = {0};
    struct file *f = NULL;
    mm_segment_t old_fs;
    loff_t pos = 0;

    if (*pPos != 0) {
        return 0;
    }
	mutex_lock(&ipd->report_mutex);
    core_config_ice_mode_enable();
    core_config_ice_mode_write(0x43008, 0x80, 1);
    core_config_ice_mode_write(0x43000, 0x0, 2);
    core_config_ice_mode_write(0x4300C, 0x01, 2);

    otp[0] = core_config_read_write_onebyte(0x4300D);
    ipio_info("otp[0] = %x\n", otp[0]);

    for (i = 1; i < sizeof(otp); i++) {
        core_config_ice_mode_write(0x43000, i, 2);
        otp[i] = core_config_read_write_onebyte(0x4300D);
        ipio_info("read OTP data .... %ld%c", ((i * 100) / sizeof(otp)), '%');
    }

    dump_data1(otp, 8, 512, 0, "OTP");

    core_config_ice_mode_write(0x4300C, 0x02, 2);
    core_config_ice_mode_disable();
 	mutex_unlock(&ipd->report_mutex);
    f = filp_open("/sdcard/otp_dump.txt", O_WRONLY | O_CREAT | O_TRUNC, 644);
    if (ERR_ALLOC_MEM(f)) {
        ipio_err("Failed to open the file at %ld.\n", PTR_ERR(f));
        return 0;
    }

    old_fs = get_fs();
    set_fs(get_ds());
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_write(f, otp, sizeof(otp), &pos);
    set_fs(old_fs);
    filp_close(f, NULL);
    return 0;
}

 static ssize_t ilitek_proc_otp_move_result(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
 {
	 //u8 otp[2] = {0};
	 char *ptr = NULL;
	 int ret = 0;
	 int len = 0;
	 u8 otp_move_result = 0x0;
	 if (*pPos != 0) {
		 return 0;
	 }
	 ipio_info("get otp result begin\n");
	*pPos += size;
	ptr = (char*)kzalloc(512,GFP_KERNEL);
	if (ERR_ALLOC_MEM(ptr)) {
		ipio_err("Failed to allocate ptr mem, %ld\n", PTR_ERR(ptr));
		 return -ENOMEM;
	}
	if(core_gesture->suspend == 1){
			 len += snprintf(ptr+len, 512-len,"is alread in suspend\n");
			 ret = copy_to_user(buff,ptr,len);
			 return len;
	}
		 //min = min(count,size);
		 //memset(buf_rawdata, 0, size);
#ifdef ILITEK_ESD_CHECK
		 ilitek_cancel_esd_check();
#endif

	 ipd->otp_move_flag = 0;
	 otp_wrie_contol_leave = 1;
		ret = wait_event_interruptible_timeout(ipd->otp_inq, ipd->otp_move_flag > 0, msecs_to_jiffies(8000));
		ipio_err("ret = %d\n",ret);
		if(!ret){
			len += snprintf(ptr+len, size-len,"get otp result timeout\n");
			ret = copy_to_user(buff,ptr,len);
			goto out;
		}
	 mutex_lock(&ipd->report_mutex);
	 core_config_ice_mode_enable();
	 core_config_otp_read(&otp_move_result, 0, 0x20, 0);
	 core_config_ice_mode_disable();
	 mutex_unlock(&ipd->report_mutex);
	 if (otp_move_result ==0xAA) {
		 len += snprintf(ptr+len, 512-len,"get otp result pass\n");
		 ret = copy_to_user(buff,ptr,len);
	 }
	 else {
		 len += snprintf(ptr+len, 512-len,"get otp result fail\n");
		 ret = copy_to_user(buff,ptr,len);
	 }
out:
#ifdef ILITEK_ESD_CHECK
	 ilitek_start_esd_check();
#endif
	otp_wrie_contol_leave = 0;
	ipio_info("get otp result end\n");
	kfree(ptr);

	 return len;
 }



static ssize_t ilitek_proc_debug_switch_read(struct file *pFile, char __user *buff, size_t nCount, loff_t *pPos)
{
	int res = 0;

	if (*pPos != 0)
		return 0;

	memset(g_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));

	ipd->debug_node_open = !ipd->debug_node_open;

	ipio_info(" %s debug_flag message = %x\n", ipd->debug_node_open ? "Enabled" : "Disabled", ipd->debug_node_open);

	nCount = sprintf(g_user_buf, "ipd->debug_node_open : %s\n", ipd->debug_node_open ? "Enabled" : "Disabled");

	*pPos += nCount;

	res = copy_to_user(buff, g_user_buf, nCount);
	if (res < 0) {
		ipio_err("Failed to copy data to user space");
	}

	return nCount;
}

static ssize_t ilitek_proc_debug_message_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int ret = 0;
	unsigned char buffer[512] = { 0 };

	/* check the buffer size whether it exceeds the local buffer size or not */
	if (size > 512) {
		ipio_err("buffer exceed 512 bytes\n");
		size = 512;
	}

	ret = copy_from_user(buffer, buff, size - 1);
	if (ret < 0) {
		ipio_err("copy data from user space, failed");
		return -1;
	}

	if (strcmp(buffer, "dbg_flag") == 0) {
		ipd->debug_node_open = !ipd->debug_node_open;
		ipio_info(" %s debug_flag message(%X).\n", ipd->debug_node_open ? "Enabled" : "Disabled",
			 ipd->debug_node_open);
	}
	return size;
}

static ssize_t ilitek_proc_debug_message_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	unsigned long p = *pPos;
	unsigned int count = size;
	int i = 0;
	int send_data_len = 0;
	size_t ret = 0;
	int data_count = 0;
	int one_data_bytes = 0;
	int need_read_data_len = 0;
	int type = 0;
	unsigned char *tmpbuf = NULL;
	unsigned char tmpbufback[128] = { 0 };

	mutex_lock(&ipd->ilitek_debug_read_mutex);

	while (ipd->debug_data_frame <= 0) {
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		wait_event_interruptible(ipd->inq, ipd->debug_data_frame > 0);
	}

	mutex_lock(&ipd->ilitek_debug_mutex);

	tmpbuf = vmalloc(4096);	/* buf size if even */
	if (ERR_ALLOC_MEM(tmpbuf)) {
		ipio_err("buffer vmalloc error\n");
		send_data_len += sprintf(tmpbufback + send_data_len, "buffer vmalloc error\n");
		ret = copy_to_user(buff, tmpbufback, send_data_len); /*ipd->debug_buf[0] */
	} else {
		if (ipd->debug_data_frame > 0) {
			if (ipd->debug_buf[0][0] == 0x5A) {
				//need_read_data_len = 43;
			} else if (ipd->debug_buf[0][0] == 0x7A) {
				type = ipd->debug_buf[0][3] & 0x0F;

				data_count = ipd->debug_buf[0][1] * ipd->debug_buf[0][2];

				if (type == 0 || type == 1 || type == 6) {
					one_data_bytes = 1;
				} else if (type == 2 || type == 3) {
					one_data_bytes = 2;
				} else if (type == 4 || type == 5) {
					one_data_bytes = 4;
				}
				//need_read_data_len = data_count * one_data_bytes + 1 + 5;
			}

			send_data_len = 0;	/* ipd->debug_buf[0][1] - 2; */
			need_read_data_len = 2040;
			if (need_read_data_len <= 0) {
				ipio_err("parse data err data len = %d\n", need_read_data_len);
				send_data_len +=
				    sprintf(tmpbuf + send_data_len, "parse data err data len = %d\n",
					    need_read_data_len);
			} else {
				for (i = 0; i < need_read_data_len; i++) {
					send_data_len += sprintf(tmpbuf + send_data_len, "%02X", ipd->debug_buf[0][i]);
					if (send_data_len >= 4096) {
						ipio_err("send_data_len = %d set 4096 i = %d\n", send_data_len, i);
						send_data_len = 4096;
						break;
					}
				}
			}
			send_data_len += sprintf(tmpbuf + send_data_len, "\n\n");

			if (p == 5 || size == 4096 || size == 2048) {
				ipd->debug_data_frame--;
				if (ipd->debug_data_frame < 0) {
					ipd->debug_data_frame = 0;
				}

				for (i = 1; i <= ipd->debug_data_frame; i++) {
					memcpy(ipd->debug_buf[i - 1], ipd->debug_buf[i], 2048);
				}
			}
		} else {
			ipio_err("no data send\n");
			send_data_len += sprintf(tmpbuf + send_data_len, "no data send\n");
		}

		/* Preparing to send data to user */
		if (size == 4096)
			ret = copy_to_user(buff, tmpbuf, send_data_len);
		else
			ret = copy_to_user(buff, tmpbuf + p, send_data_len - p);

		if (ret) {
			ipio_err("copy_to_user err\n");
			ret = -EFAULT;
		} else {
			*pPos += count;
			ret = count;
			ipio_debug(DEBUG_FINGER_REPORT, "Read %d bytes(s) from %ld\n", count, p);
		}
	}
	/* ipio_err("send_data_len = %d\n", send_data_len); */
	if (send_data_len <= 0 || send_data_len > 4096) {
		ipio_err("send_data_len = %d set 2048\n", send_data_len);
		send_data_len = 4096;
	}
	if (tmpbuf != NULL) {
		vfree(tmpbuf);
		tmpbuf = NULL;
	}

	mutex_unlock(&ipd->ilitek_debug_mutex);
	mutex_unlock(&ipd->ilitek_debug_read_mutex);
	return send_data_len;
}
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
static ssize_t ilitek_proc_oppo_upgrade_fw_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char cmd[10] = { 0 };
	if (*pPos != 0)
		return 0;

	if (buff != NULL) {
		res = copy_from_user(cmd, buff, size - 1);
		if (res < 0) {
			ipio_err("copy data from user space, failed\n");
			return -1;
		}
	}
	*pPos += size;

	ipio_debug(DEBUG_FIRMWARE, "size = %d, cmd = %d\n", (int)size, cmd[0]);

	if (strcmp(cmd, "0") == 0) {
			ilitek_platform_disable_irq();

			if (ipd->isEnablePollCheckPower)
				cancel_delayed_work_sync(&ipd->check_power_status_work);
			mutex_lock(&ipd->report_mutex);
#ifdef HOST_DOWNLOAD
			oppo_platform_tp_hw_reset(true);
			//ilitek_platform_tp_hw_reset(true);
#else
			res = core_firmware_upgrade(UPDATE_FW_PATH, false);
#endif
			mutex_unlock(&ipd->report_mutex);
			ilitek_platform_enable_irq();

			if (ipd->isEnablePollCheckPower)
				queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);

			if (res < 0) {
				core_firmware->update_status = res;
				ipio_err("Failed to upgrade firwmare\n");
			} else {
				core_firmware->update_status = 100;
				ipio_info("Succeed to upgrade firmware\n");
			}

	}
	if(cmd[0] != '0'  && cmd[0] != '1' ) {
		res = request_firmware_select(&ili_fw_oppo, ili_fw->firmware_bin_name,&ipd->spi->dev);
		if (res) {
				ipio_info("update for oppo error\n");
		}

	}

	return size;
}
static ssize_t oppo_proc_irq_depth_read(struct file *file, char *buf,
								  size_t len, loff_t *pos)
{
	size_t ret = 0;
	char *temp_buf;
	struct irq_desc *desc = irq_to_desc(ipd->isr_gpio);
	ipio_debug(DEBUG_IRQ, "%s: enter, %d \n", __func__, __LINE__);

	if (!ILITEK_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		ret += snprintf(temp_buf + ret, len - ret, "now depth=%d\n", desc->depth);

		if (copy_to_user(buf, temp_buf, len))
			ipio_debug(DEBUG_IRQ, "%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		ILITEK_PROC_SEND_FLAG = 1;
	} else {
		ILITEK_PROC_SEND_FLAG = 0;
	}

	return ret;
}
static ssize_t oppo_proc_i2c_device_read(struct file *file, char *buf,
								  size_t len, loff_t *pos)
{
	size_t ret = 0;
	char *temp_buf;
	bool spi_read_result = false;

	ilitek_platform_disable_irq();
	if(core_config_get_fw_ver())
		spi_read_result = true;
	ilitek_platform_enable_irq();


	if (!ILITEK_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(spi_read_result)
		ret += snprintf(temp_buf + ret, len - ret, "ILITEK SPI devices\n");
		else
		ret += snprintf(temp_buf + ret, len - ret, "NO devices\n");
		if (copy_to_user(buf, temp_buf, len))
			ipio_info("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		ILITEK_PROC_SEND_FLAG = 1;
	} else {
		ILITEK_PROC_SEND_FLAG = 0;
	}

	return ret;
}

#if 0
static ssize_t ilitek_proc_coordinate_read(struct file *filp, char __user *buff, size_t len, loff_t *pPos)
{
	size_t ret = 0;
	char *temp_buf;
	int i;

	if (!ILITEK_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		for(i=0;i<10;i++)
		{
		ret += snprintf(temp_buf + ret, len - ret, "%d,",buf_gesture[i]);
		}
			ret += snprintf(temp_buf+ret, len-ret,"\n");

			if (copy_to_user(buff, temp_buf, len))
				ipio_info("%s,here:%d\n", __func__, __LINE__);
		kfree(temp_buf);
		ILITEK_PROC_SEND_FLAG = 1;
	} else {
		ILITEK_PROC_SEND_FLAG = 0;
	}

	return ret;
}
#endif
static ssize_t ilitek_proc_CDC_delta_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
#ifdef MING_TEST
	int ret = 0;
	int len = 0;
	//size_t size = 0;
	char *ptr = NULL;
	uint8_t temp[256] = { 0 };
	int i=0;
	//size = FRAME_NODE;
	if ( *ppos ) {
	    printk("is already read the file\n");
        return 0;
	}
	ipio_info("proc delta begin\n");
	*ppos += count;
	ptr = (char*)kzalloc(count,GFP_KERNEL);
		if (ERR_ALLOC_MEM(ptr)) {
		ipio_err("Failed to allocate ptr mem, %ld\n", PTR_ERR(ptr));
		return -ENOMEM;
	}
	if(core_gesture->suspend == 1){
		len += snprintf(ptr+len, count-len,"is alread in suspend\n");
		ret = copy_to_user(buff,ptr,len);
		return len;
	}
    //min = min(count,size);
    //memset(buf_rawdata, 0, size);
#ifdef ILITEK_ESD_CHECK
	ilitek_cancel_esd_check();
#endif
//	if (!ILITEK_PROC_SEND_FLAG)
	{
		ipd->delta_data_frame =0;
		mutex_lock(&ipd->report_mutex);
		ilitek_platform_disable_irq();
		core_fr->isEnableFR = false;
		temp[0] = protocol->debug_mode;
		core_fr_mode_control(temp);

		temp[0] = 0xFA;
		temp[1] = 0x03;
		ret = core_write(core_config->slave_i2c_addr, temp, 2);
		if(ret < 0){
			goto out;
		}
		core_fr->isEnableFR = true;
		ilitek_platform_enable_irq();
		mutex_unlock(&ipd->report_mutex);

		//mutex_lock(&ipd->ilitek_delta_read_mutex);
		ipd->delta_node_open =1;
		ret = wait_event_interruptible_timeout(ipd->delta_inq, ipd->delta_data_frame>0, msecs_to_jiffies(3000));
		ipio_err("ret = %d\n",ret);
		if(!ret){
			len += snprintf(ptr+len, count-len,"get delta timeout\n");
			ret = copy_to_user(buff,ptr,len);
			goto out;
		}
		ipio_info("test get delta\n");
		ilitek_platform_disable_irq();
		core_fr->isEnableFR = false;
		/*core_fr->actual_fw_mode = P5_0_FIRMWARE_TEST_MODE;
		oppo_platform_tp_hw_reset(true);
		ret = oppo_get_cdc_data(2);
		if (ret < 0)
			{
			ipio_err("Failed to initialise CDC data, %d\n", ret);
			}
		core_fr->actual_fw_mode = P5_0_FIRMWARE_DEMO_MODE;
		oppo_platform_tp_hw_reset(true);*/
	//mutex_lock(&ipd->ilitek_delta_mutex);
	for(i=0;i<FRAME_NODE;i++)
	{
		//if(buf_delta[i]<0)
		//	len += snprintf(ptr+len, count-len,"-0x%2.2X,",0xFFFFFFFF-buf_delta[i]);
		//else
        //len += snprintf(ptr+len, count-len,"0x%2.2X,",buf_delta[i]);
        len += snprintf(ptr+len, count-len,"%d,",buf_delta[i]);
	 	if ((i % 18) == 17)
	 	len += snprintf(ptr+len, count-len,"\n");
	}
	len += snprintf(ptr+len, count-len,"\n");
	ret = copy_to_user(buff,ptr,len);
out:
	mutex_lock(&ipd->report_mutex);
	ipd->delta_node_open =0;
	temp[0] = protocol->demo_mode;
	core_fr_mode_control(temp);
	ilitek_platform_tp_hw_reset(true);
	core_fr->isEnableFR = true;
	ilitek_platform_enable_irq();
	mutex_unlock(&ipd->report_mutex);

#ifdef ILITEK_ESD_CHECK
	ilitek_start_esd_check();
#endif
	//ILITEK_PROC_SEND_FLAG = 1;
    }
	//else
	//{
	//ILITEK_PROC_SEND_FLAG = 0;
	//}

	//mutex_unlock(&ipd->ilitek_delta_mutex);
	//mutex_unlock(&ipd->ilitek_delta_read_mutex);
	ipio_info("proc delta end\n");
	kfree(ptr);

	return len;
#else
	return 0;
#endif

}
static ssize_t ilitek_proc_rawdata_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
	{
#ifdef MING_TEST
		int ret = 0;
		int len = 0;
		//size_t size = 0;
		char *ptr = NULL;
		uint8_t temp[256] = { 0 };
		int i=0;
		//size = FRAME_NODE;
		if ( *ppos ) {
			printk("is already read the file\n");
			return 0;
		}
		ipio_info("proc raw begin\n");
		*ppos += count;
	ptr = (char*)kzalloc(count,GFP_KERNEL);
		if (ERR_ALLOC_MEM(ptr)) {
		ipio_err("Failed to allocate ptr mem, %ld\n", PTR_ERR(ptr));
		return -ENOMEM;
	}
	if(core_gesture->suspend == 1){
		len += snprintf(ptr+len, count-len,"is alread in suspend\n");
		ret = copy_to_user(buff,ptr,len);
		return len;
	}
#ifdef ILITEK_ESD_CHECK
		ilitek_cancel_esd_check();
#endif
		//min = min(count,size);
		//memset(buf_rawdata, 0, size);
		//if (!ILITEK_PROC_SEND_FLAG)
		//{
			ipd->delta_data_frame =0;
			mutex_lock(&ipd->report_mutex);
			ilitek_platform_disable_irq();
			core_fr->isEnableFR = false;
			temp[0] = protocol->debug_mode;
			core_fr_mode_control(temp);

			temp[0] = 0xFA;
			temp[1] = 0x08;
		ret = core_write(core_config->slave_i2c_addr, temp, 2);
		if(ret < 0){
			goto out;
		}
			core_fr->isEnableFR = true;
			ilitek_platform_enable_irq();
			mutex_unlock(&ipd->report_mutex);

			//mutex_lock(&ipd->report_mutex);
			ipd->delta_node_open =1;
		ret = wait_event_interruptible_timeout(ipd->delta_inq, ipd->delta_data_frame>0, msecs_to_jiffies(3000));
		ipio_err("ret = %d\n",ret);
		if(!ret){
			len += snprintf(ptr+len, count-len,"get delta timeout\n");
			ret = copy_to_user(buff,ptr,len);
			goto out;
		}
			ipio_info("test get raw\n");
			ilitek_platform_disable_irq();
			core_fr->isEnableFR = false;
			/*core_fr->actual_fw_mode = P5_0_FIRMWARE_TEST_MODE;
			oppo_platform_tp_hw_reset(true);
			ret = oppo_get_cdc_data(2);
			if (ret < 0)
				{
				ipio_err("Failed to initialise CDC data, %d\n", ret);
				}
			core_fr->actual_fw_mode = P5_0_FIRMWARE_DEMO_MODE;
			oppo_platform_tp_hw_reset(true);*/
	//mutex_lock(&ipd->ilitek_delta_mutex);
		for(i=0;i<FRAME_NODE;i++)
		{
			//if(buf_delta[i]<0)
			//	len += snprintf(ptr+len, count-len,"-0x%2.2X,",0xFFFFFFFF-buf_delta[i]);
			//else
			//len += snprintf(ptr+len, count-len,"0x%2.2X,",buf_rawdata[i]);
			len += snprintf(ptr+len, count-len,"%d,",buf_rawdata[i]);
			if ((i % 18) == 17)
			len += snprintf(ptr+len, count-len,"\n");
		}
		len += snprintf(ptr+len, count-len,"\n");
		ret = copy_to_user(buff,ptr,len);
out:
		mutex_lock(&ipd->report_mutex);
		ipd->delta_node_open =0;
		temp[0] = protocol->demo_mode;
		core_fr_mode_control(temp);
		ilitek_platform_tp_hw_reset(true);
		core_fr->isEnableFR = true;
		ilitek_platform_enable_irq();
		mutex_unlock(&ipd->report_mutex);
#ifdef ILITEK_ESD_CHECK
		ilitek_start_esd_check();
#endif
		ipio_info("proc raw end\n");
		kfree(ptr);

		return len;
#else
		return 0;
#endif

	}

static ssize_t ilitek_proc_main_register_read(struct file *filp, char __user *buff, size_t count, loff_t *pPos)
{
		int ret = 0;
		int len = 0;
		char *ptr = NULL;
		if (!ILITEK_PROC_SEND_FLAG)
		{
		    ptr = (char*)kzalloc(count,GFP_KERNEL);
			len += snprintf(ptr+len, count-len,"0x%2.2X,0x%2.2X,0x%2.2X,0x%2.2X\n",core_config->firmware_ver[3],core_config->firmware_ver[2],core_config->firmware_ver[1],core_config->firmware_ver[0]);
		    ret = copy_to_user(buff,ptr,len);
		    ILITEK_PROC_SEND_FLAG = 1;
		}
		else
		{
		    ILITEK_PROC_SEND_FLAG = 0;
		}
		return len;

}

#if 0

static ssize_t oppo_proc_mp_test_read(struct file *file, char __user *buff, size_t count, loff_t *ppos)
{
	uint32_t len = 0;
	size_t size = 0;
	char *ptr = NULL;
	char result_pass[40] = "0 error(s),All test passed";
	char result_fail[20] = "MP test fail";
	uint32_t res = 0;
	uint8_t test_cmd[2] = { 0 };
	//bool mp_test_result =false;
	if (*ppos != 0)
		return 0;
#ifdef ILITEK_ESD_CHECK
		ilitek_cancel_esd_check();
#endif

		if (core_parser_path(ili_fw->mp_ini_name) < 0) {
			ipio_err("Failed to parsing INI file\n");
			goto out;
		}

		/* Init MP structure */
		if(core_mp_init() < 0) {
			ipio_err("Failed to init mp\n");
			goto out;
		}

		/* Switch to Test mode */
		test_cmd[0] = protocol->test_mode;
		core_fr_mode_control(test_cmd);

		ilitek_platform_disable_irq();

		core_mp_run_test("Short Test -ILI9881", true);
		core_mp_run_test("Open Test(integration)_SP", true);
		core_mp_run_test("Calibration Data(DAC)", true);
		core_mp_run_test("Raw Data(Have BK)", true);
		core_mp_run_test("Noise Peak to Peak(IC Only)", true);
		core_mp_run_test("Noise Peak To Peak(With Panel)", true);
		core_mp_run_test("Raw Data(No BK)", true);
		core_mp_run_test("Raw Data(Have BK)(LCM OFF)", true);
		core_mp_run_test("Noise Peak to Peak(IC Only)(LCM OFF)", true);
		core_mp_run_test("Doze Peak To Peak", true);
		//core_mp_run_test("Pin Test(INT & RST)", true);

		core_mp_show_result();

		core_mp_test_free();
#ifndef HOST_DOWNLOAD
		/* Code reset */
		core_config_ice_mode_enable();

		/* Disable watch dog */
		core_config_reset_watch_dog();

		core_config_ic_reset();
#endif
		/* Switch to Demo mode */
		test_cmd[0] = protocol->demo_mode;
		core_fr_mode_control(test_cmd);
#ifdef HOST_DOWNLOAD
		ilitek_platform_tp_hw_reset(true);
#endif
		ilitek_platform_enable_irq();

out:
#ifdef ILITEK_ESD_CHECK
		ilitek_start_esd_check();
#endif
    //if(mp_test_result)
    if(1)
	{
		size = ARRAY_SIZE(result_pass);
	    //len = snprintf(ptr, size,"%s\n",result_pass);
	}
    else
	{
		size = ARRAY_SIZE(result_fail);
	    //len = snprintf(ptr, size,"%s\n",result_fail);
	}
	ptr = (char*)kzalloc(size,GFP_KERNEL);
	//if(mp_test_result)
	if (1)
	{
		//size = ARRAY_SIZE(result_pass);
	    len = snprintf(ptr, size,"%s\n",result_pass);
	}
    else
	{
		//size = ARRAY_SIZE(result_fail);
	    len = snprintf(ptr, size,"%s\n",result_fail);
	}
    res = copy_to_user(buff,ptr,len);
    if (res < 0)
	{
	  ipio_err("Failed to copy data to user space\n");
	}
	*ppos = len;
	ipio_info("MP Test DONE\n");
	return len;

}
#else

/* Created only for oppo */
static ssize_t oppo_proc_mp_lcm_on_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int len = 0;
	size_t count = 0;
	char result_pass[40] = "0 error(s),All test passed";
	char result_fail[20] = "MP test fail";
	char *ptr = NULL;
	uint32_t res = 0;

	mp_test_result = false;

	if (*pPos != 0)
		return 0;

	ptr = (char*)kzalloc(256,GFP_KERNEL);
    if (ptr == NULL) {
        ipio_err("failed to alloc ptr memory\n");
        return 0;
    }

#ifdef ILITEK_ESD_CHECK
	ilitek_cancel_esd_check();
#endif
	mutex_lock(&ipd->report_mutex);

	if (core_parser_path(ili_fw->mp_ini_name) < 0) {
		ipio_err("Failed to parsing INI file\n");
		goto out;
	}

	/* Init MP structure */
	if(core_mp_init() < 0) {
		ipio_err("Failed to init mp\n");
		goto out;
	}

	/* Switch to test mode */
	core_fr_mode_control(&protocol->test_mode);

	ilitek_platform_disable_irq();

	core_mp->oppo_run = true;

	/* Do not chang the sequence of test */
	core_mp_run_test("Noise Peak To Peak(With Panel)", true);
	core_mp_run_test("Noise Peak to Peak(IC Only)", true);
	core_mp_run_test("Short Test -ILI9881", true);
	core_mp_run_test("Open Test(integration)_SP", true);
	core_mp_run_test("Raw Data(Have BK)", true);
	core_mp_run_test("Calibration Data(DAC)", true);
	core_mp_run_test("Raw Data(No BK)", true);
	core_mp_run_test("Doze Raw Data", true);
	core_mp_run_test("Doze Peak To Peak", true);

	core_mp_show_result();

	core_mp->oppo_run = false;

	core_mp_test_free();

	/* Switch to demo mode */
	core_fr_mode_control(&protocol->demo_mode);

#ifdef HOST_DOWNLOAD
	ilitek_platform_tp_hw_reset(true);
#endif

	ilitek_platform_enable_irq();
	ipio_info("MP Test mp_test_result = %d\n", mp_test_result);
if (mp_test_result == true)
{
	count = ARRAY_SIZE(result_pass);
	//len = snprintf(ptr, size,"%s\n",result_pass);
}
else
{
	count = ARRAY_SIZE(result_fail);
	//len = snprintf(ptr, size,"%s\n",result_fail);
}
if (mp_test_result == true)
{
	//size = ARRAY_SIZE(result_pass);
	len = snprintf(ptr, count,"%s\n",result_pass);
}
else
{
	//size = ARRAY_SIZE(result_fail);
	len = snprintf(ptr, count,"%s\n",result_fail);
}
res = copy_to_user(buff,ptr,len);
if (res < 0)
{
  ipio_err("Failed to copy data to user space\n");
}
ipio_debug(DEBUG_MP_TEST, "MP Test DONE\n");
out:
	mutex_unlock(&ipd->report_mutex);
#ifdef ILITEK_ESD_CHECK
	ilitek_start_esd_check();
#endif
	//mutex_unlock(&ipd->report_mutex);
	*pPos = len;
	kfree(ptr);
	return len;
}
#endif
/* Created only for oppo */
static int ilitek_mp_lcm_off_read(void)
{
	int len = 0;

	mp_test_result = false;

	if (core_parser_path(ili_fw->mp_ini_name) < 0) {
		ipio_err("Failed to parsing INI file\n");
		goto out;
	}

	/* Init MP structure */
	if(core_mp_init() < 0) {
		ipio_err("Failed to init mp\n");
		goto out;
	}

	ilitek_platform_disable_irq();
#if 0
	/* Enter to suspend and move gesture code to iram */
	core_config->isEnableGesture = true;
	core_gesture->mode = GESTURE_INFO_MPDE;

	/* sense stop */
	core_config_sense_ctrl(false);

	if (core_config_check_cdc_busy(50, 50) < 0)
		ipio_err("Check busy is timout !\n");

	core_fr->actual_fw_mode = P5_0_FIRMWARE_GESTURE_MODE;

	core_gesture_load_code();
#endif
	/* Switch to test mode which moves mp code to iram */
	core_fr_mode_control(&protocol->test_mode);

	ilitek_platform_disable_irq();

	/* Indicates running mp test is called by oppo node */
	core_mp->oppo_run = true;
	core_mp->oppo_lcm = true;

	/* Do not chang the sequence of test */
	core_mp_run_test("Raw Data(No BK) (LCM OFF)", true);
	core_mp_run_test("Noise Peak To Peak(With Panel) (LCM OFF)", true);
	core_mp_run_test("Raw Data_TD (LCM OFF)", true);
	core_mp_run_test("Peak To Peak_TD (LCM OFF)", true);

	core_mp_show_result();

	core_mp->oppo_run = false;
	core_mp->oppo_lcm = false;

	//core_mp_ctrl_lcm_status(true);

	core_mp_test_free();

	core_fr_mode_control(&protocol->demo_mode);
//	if(core_config->gesture_backup == true)
//	{
	ilitek_platform_tp_hw_reset(true);
	ilitek_platform_enable_irq();
//	}
	if (mp_test_result == true) {
		len = 1;
	}
out:
	return len;
}
static ssize_t oppo_proc_black_screen_test_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    int retry = 20;
	int len = 0;
	char *gesture_test_message = NULL;
	char *ptr = NULL;
	if ( *ppos ) {
		printk("is already read the file\n");
		return 0;
	}
	*ppos += count;
	if (!gesture_test_flag) {
		printk("gesture_test_flag = %d\n",gesture_test_flag);
		//return 0;
	}

    gesture_test_message = kzalloc(256, GFP_KERNEL);
    if (!gesture_test_message) {
        ipio_err("failed to alloc memory\n");
		ipd->black_test_flag = 0;
        return 0;
    }
    ptr = kzalloc(256, GFP_KERNEL);
    if (ptr == NULL) {
        ipio_err("failed to alloc ptr memory\n");
		kfree(gesture_test_message);
		ipd->black_test_flag = 0;
        return 0;
    }

	if(core_config->isEnableGesture == false)
	{
        ipio_debug(DEBUG_MP_TEST, "%s need to open gesture\n",__func__);
		sprintf(gesture_test_message, "gesture mode is off");
		goto OUT;
	}

    /* wait until tp is in sleep, then sleep 500ms to make sure tp is in gesture mode*/
    do {
        if (core_gesture->suspend) {
            msleep(500);
            break;
        }
        msleep(200);
    } while(--retry);

    ipio_info("%s retry times %d\n", __func__, retry);
    if (retry == 0 && !core_gesture->suspend) {
		ipio_debug(DEBUG_MP_TEST, "%s 1 errors: not in sleep\n", __func__);
        sprintf(gesture_test_message, "1 errors: not in sleep ");
        goto OUT;
    }
	ipd->black_test_flag = 1;
	mutex_lock(&ipd->report_mutex);
    ret = ilitek_mp_lcm_off_read();
	mutex_unlock(&ipd->report_mutex);

	if (ret){
		ipio_err("can not complete blackscreen test\n");
	}
	if (ret == 1) {
		sprintf(gesture_test_message, "0 error(s). All test passed");
	} else {
		sprintf(gesture_test_message, "result=0");
	}

OUT:
    len = snprintf(ptr,count,"%s\n",gesture_test_message);
	g_gesture = lcm_gesture_power;
    gesture_test_flag = 0;
	core_config->isEnableGesture = core_config->gesture_backup;
	if(core_config->isEnableGesture == false)
	{
		core_gesture->entry = false;
	}
	core_config->gesture_backup = false;
	ipio_debug(DEBUG_MP_TEST, "%s core_config->isEnableGesture: %d,core_config->gesture_backup: %d\n", __func__,core_config->isEnableGesture,core_config->gesture_backup);

    ret=copy_to_user(user_buf,ptr,len);

	ipio_debug(DEBUG_MP_TEST, "%s **************6*********ret = %d*****gesture_test_message = %s***\n", __func__,ret,gesture_test_message);

	kfree(gesture_test_message);
	kfree(ptr);
	ipd->black_test_flag = 0;
    return len;
}

static ssize_t oppo_proc_black_screen_test_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0;
	char buf[4] = {0};
	char *ptr = NULL;
	ptr = kzalloc(count,GFP_KERNEL);
	if ( ptr == NULL ) {
		return -1;
	}
	if ( copy_from_user(buf, buffer, count) ) {
		printk("%s: copy from user error.", __func__);
		count = -1;
		goto OUT;
	}
	sscanf(buf, "%d", &value);

	core_config->gesture_backup = core_config->isEnableGesture;
    core_config->isEnableGesture = true;
	lcm_gesture_power = g_gesture;
	g_gesture = 1;
	core_gesture->mode = GESTURE_INFO_MPDE;
    gesture_test_flag = !!value;
OUT:
	kfree(ptr);
    return count;
}

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/17, add presure report function
static ssize_t ilitek_proc_debug_presure_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int value = 0;
	char buf[4] = {0};
	char *ptr = NULL;
	ptr = kzalloc(count,GFP_KERNEL);
	if ( ptr == NULL ) {
		return -1;
	}
	if ( copy_from_user(buf, buffer, count) ) {
		printk("%s: copy from user error.", __func__);
		count = -1;
		goto OUT;
	}
	sscanf(buf, "%d", &value);
	if(value > 0)
		ipd->presure_speed = value;
	ipio_info("ipd->presure_speed = %d,value = %d",ipd->presure_speed,value);
OUT:
	kfree(ptr);
    return count;
}

static ssize_t ilitek_proc_debug_presure_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char *ptr = NULL;

	if ( *ppos ) {
	    printk("is already read the file\n");
        return 0;
	}
    *ppos += count;

    ptr = kzalloc(count,GFP_KERNEL);
	if(ptr == NULL){
		printk("allocate memory fail\n");
		return -1;
	}

    len = snprintf(ptr, count,"%d\n",ipd->presure_speed);

    ret = copy_to_user(buf,ptr,len);

	kfree(ptr);
	return len;
}
#endif

static ssize_t oppo_proc_game_switch_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char *ptr = NULL;

	if ( *ppos ) {
	    printk("is already read the file\n");
        return 0;
	}
    *ppos += count;

    ptr = kzalloc(count,GFP_KERNEL);
	if(ptr == NULL){
		printk("allocate memory fail\n");
		return -1;
	}

    len = snprintf(ptr, count,"%x\n",core_config->gameSwitch);

    ret = copy_to_user(buf,ptr,len);

	kfree(ptr);
	return len;
}
static ssize_t oppo_proc_game_switch_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char *ptr = NULL;
	ptr = kzalloc(size,GFP_KERNEL);
	if (ptr == NULL) {
		ipio_err("allocate the memory fail\n");
		return -1;
	}
	res = copy_from_user(ptr, buff, size);
	if (res) {
		ipio_err("copy data from user space, failed\n");
		size = -1;
		goto OUT;
	}
	//ipio_info("size = %d, cmd = %s data %d\n", (int)size, cmd[0],data);
	mutex_lock(&ipd->report_mutex);
	if (ptr[0] == '1') {
		ipio_info("enable game play mode\n");
		core_config->gameSwitch= true;
		core_config_game_switch_ctrl(core_config->gameSwitch);
	} else if (ptr[0] == '0') {
		ipio_info("disable game play mode\n");
		core_config->gameSwitch = false;
		core_config_game_switch_ctrl(core_config->gameSwitch);
	} else {
		ipio_err("Unknown command\n");
	}
	mutex_unlock(&ipd->report_mutex);
OUT:
	kfree(ptr);
	return size;
}

#endif

static ssize_t ilitek_proc_mp_test_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	uint32_t len = 0;

	if (*pPos != 0)
		return 0;

#ifdef ILITEK_ESD_CHECK
	ilitek_cancel_esd_check();
#endif
	mutex_lock(&ipd->report_mutex);

	if (core_parser_path(ili_fw->mp_ini_name) < 0) {
		ipio_err("Failed to parsing INI file\n");
		goto out;
	}

	/* Init MP structure */
	if(core_mp_init() < 0) {
		ipio_err("Failed to init mp\n");
		goto out;
	}

	/* Switch to Test mode */
	core_fr_mode_control(&protocol->test_mode);

	ilitek_platform_disable_irq();

	/* Start to run MP test */
	core_mp->run = true;

	/*
	 * Get timing parameters first.
	 * Howerver, this can be ignored if read them from ini.
	 */
	if (protocol->major >= 5 && protocol->mid >= 4) {
		if (core_mp_calc_timing_nodp() < 0) {
			ipio_err("Can't get timing parameters\n");
			goto out;
		}
	}

	/* Do not chang the sequence of test */
	core_mp_run_test("Noise Peak To Peak(With Panel)", true);
	core_mp_run_test("Noise Peak to Peak(IC Only)", true);
	core_mp_run_test("Short Test -ILI9881", true);
	core_mp_run_test("Open Test(integration)_SP", true);
	core_mp_run_test("Raw Data(Have BK)", true);
	//core_mp_run_test("Raw Data(Have BK) (LCM OFF)", true);
	core_mp_run_test("Calibration Data(DAC)", true);
	core_mp_run_test("Raw Data(No BK)", true);
	core_mp_run_test("Raw Data(No BK) (LCM OFF)", true);
	core_mp_run_test("Noise Peak to Peak(With Panel) (LCM OFF)", true);
	//core_mp_run_test("Noise Peak to Peak(IC Only) (LCM OFF)", true);
	core_mp_run_test("Raw Data_TD (LCM OFF)", true);
	core_mp_run_test("Peak To Peak_TD (LCM OFF)", true);
	core_mp_run_test("Doze Raw Data", true);
	core_mp_run_test("Doze Peak To Peak", true);
	//core_mp_run_test("Pin Test ( INT and RST )", true);

	core_mp_show_result();

	core_mp->run = false;

	core_mp_test_free();

#ifndef HOST_DOWNLOAD
	core_config_ice_mode_enable();

	if (core_config_set_watch_dog(false) < 0) {
		ipio_err("Failed to disable watch dog\n");
	}

	core_config_ic_reset();
#endif
	/* Switch to Demo mode */
	core_fr_mode_control(&protocol->demo_mode);

#ifdef HOST_DOWNLOAD
	ilitek_platform_tp_hw_reset(true);
#endif
	ilitek_platform_enable_irq();
out:
	mutex_unlock(&ipd->report_mutex);
	#ifdef ILITEK_ESD_CHECK
	ilitek_start_esd_check();
	#endif
	*pPos = len;
	return len;
}

static ssize_t ilitek_proc_mp_test_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int i, res = 0, count = 0;
	char cmd[64] = {0}, str[512] = {0};
	char *token = NULL, *cur = NULL;
	uint8_t *va = NULL;

	if (buff != NULL) {
		res = copy_from_user(cmd, buff, size - 1);
		if (res < 0) {
			ipio_err("copy data from user space, failed\n");
			return -1;
		}
	}

	ipio_debug(DEBUG_MP_TEST, "size = %d, cmd = %s\n", (int)size, cmd);

	if (size > 64) {
		ipio_err("The size of string is too long\n");
		return size;
	}

	token = cur = cmd;

	va = kcalloc(64, sizeof(uint8_t), GFP_KERNEL);

	while ((token = strsep(&cur, ",")) != NULL) {
		va[count] = katoi(token);
		ipio_debug(DEBUG_MP_TEST, "data[%d] = %x\n", count, va[count]);
		count++;
	}

	ipio_debug(DEBUG_MP_TEST, "cmd = %s\n", cmd);

	/* Init MP structure */
	if(core_mp_init() < 0) {
		ipio_err("Failed to init mp\n");
		return size;
	}

	/* Switch to Test mode */
	core_fr_mode_control(&protocol->test_mode);

	ilitek_platform_disable_irq();

	for (i = 0; i < core_mp->mp_items; i++) {
		if (strcmp(cmd, tItems[i].name) == 0) {
			strcpy(str, tItems[i].desp);
			tItems[i].run = 1;
			tItems[i].max = va[1];
			tItems[i].min = va[2];
			tItems[i].frame_count = va[3];
			break;
		}
	}

	core_mp_run_test(str, false);

	core_mp_show_result();

	core_mp_test_free();

#ifndef HOST_DOWNLOAD
	/* Code reset */
	core_config_ice_mode_enable();

	if (core_config_set_watch_dog(false) < 0) {
		ipio_err("Failed to disable watch dog\n");
	}

	core_config_ic_reset();
#endif

	/* Switch to Demo mode it prevents if fw fails to be switched */
	core_fr_mode_control(&protocol->demo_mode);

#ifdef HOST_DOWNLOAD
	ilitek_platform_tp_hw_reset(true);
#endif

	ilitek_platform_enable_irq();

	ipio_debug(DEBUG_MP_TEST, "MP Test DONE\n");
	ipio_kfree((void **)&va);
	return size;
}

static ssize_t ilitek_proc_debug_level_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	uint32_t len = 0;

	if (*pPos != 0)
		return 0;

	memset(g_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));

	len = sprintf(g_user_buf, "%d", ipio_debug_level);

	ipio_info("Current DEBUG Level = %d\n", ipio_debug_level);
	ipio_info("You can set one of levels for debug as below:\n");
	ipio_info("DEBUG_NONE = %d\n", DEBUG_NONE);
	ipio_info("DEBUG_IRQ = %d\n", DEBUG_IRQ);
	ipio_info("DEBUG_FINGER_REPORT = %d\n", DEBUG_FINGER_REPORT);
	ipio_info("DEBUG_FIRMWARE = %d\n", DEBUG_FIRMWARE);
	ipio_info("DEBUG_CONFIG = %d\n", DEBUG_CONFIG);
	ipio_info("DEBUG_I2C = %d\n", DEBUG_I2C);
	ipio_info("DEBUG_BATTERY = %d\n", DEBUG_BATTERY);
	ipio_info("DEBUG_MP_TEST = %d\n", DEBUG_MP_TEST);
	ipio_info("DEBUG_IOCTL = %d\n", DEBUG_IOCTL);
	ipio_info("DEBUG_NETLINK = %d\n", DEBUG_NETLINK);
	ipio_info("DEBUG_ALL = %d\n", DEBUG_ALL);

	res = copy_to_user((uint32_t *) buff, &ipio_debug_level, len);
	if (res < 0) {
		ipio_err("Failed to copy data to user space\n");
	}

	*pPos = len;

	return len;
}

static ssize_t ilitek_proc_debug_level_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char cmd[10] = { 0 };

	if (buff != NULL) {
		res = copy_from_user(cmd, buff, size - 1);
		if (res < 0) {
			ipio_err("copy data from user space, failed\n");
			return -1;
		}
	}

	ipio_debug_level = katoi(cmd);

	ipio_info("ipio_debug_level = %d\n", ipio_debug_level);

	return size;
}
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
static ssize_t ilitek_proc_oppo_debug_level_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char cmd[10] = { 0 };

	if (buff != NULL) {
		res = copy_from_user(cmd, buff, size - 1);
		if (res < 0) {
			ipio_info("copy data from user space, failed\n");
			return -1;
		}
	}

	oppo_debug_level = katoi(cmd);

	ipio_info("oppo_debug_level = %d\n", oppo_debug_level);

	return size;
}
#endif
static ssize_t ilitek_proc_gesture_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	uint32_t len = 0;

	if (*pPos != 0)
		return 0;

	memset(g_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));

	len = sprintf(g_user_buf, "%d", core_config->isEnableGesture);

	ipio_info("isEnableGesture = %d\n", core_config->isEnableGesture);

	res = copy_to_user((uint32_t *) buff, &core_config->isEnableGesture, len);
	if (res < 0) {
		ipio_err("Failed to copy data to user space\n");
	}

	*pPos = len;

	return len;
}

static ssize_t ilitek_proc_gesture_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char cmd[10] = { 0 };

	if (buff != NULL) {
		res = copy_from_user(cmd, buff, size - 1);
		if (res < 0) {
			ipio_err("copy data from user space, failed\n");
			return -1;
		}
	}

	ipio_info("size = %d, cmd = %s\n", (int)size, cmd);

	if (strcmp(cmd, "on") == 0) {
		ipio_info("enable gesture mode\n");
		core_config->isEnableGesture = true;
	} else if (strcmp(cmd, "off") == 0) {
		ipio_info("disable gesture mode\n");
		core_config->isEnableGesture = false;
	} else
		ipio_err("Unknown command\n");

	return size;
}
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
static ssize_t oppo_proc_gesture_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char *ptr = NULL;
	uint8_t temp[64] = {0};
	//char cmd[10] = {0};
	//int data =0;
	/*if (core_gesture->suspend != false) {
		ipio_info("%s,is already suspend\n",__func__);
		return -1;
	}*/
	ptr = kzalloc(size,GFP_KERNEL);
	if (ptr == NULL) {
		ipio_err("allocate the memory fail\n");
		return -1;
	}
	res = copy_from_user(ptr, buff, size);
	if (res) {
		ipio_err("copy data from user space, failed\n");
		return -1;
	}
	//data = cmd[0]-48;
	//ipio_info("size = %d, cmd = %s data %d\n", (int)size, cmd[0],data);
	ipio_debug(DEBUG_CONFIG, "core_gesture->suspend = %d, gesture_done = %d\n", core_gesture->suspend, gesture_done);
	if (ptr[0] == '1') {
		if (core_gesture->suspend) {
			psensor_close = false;
			mutex_lock(&ipd->report_mutex);
			core_config->isEnableGesture = true;
			g_gesture = 1;
			core_gesture->mode = GESTURE_INFO_MPDE;
			temp[0] = 0xF6;
			temp[1] = 0x0A;
			 ipio_info("write prepare gesture command 0xF6 0x0A \n");
			if ((core_write(core_config->slave_i2c_addr, temp, 2)) < 0) {
				ipio_info("write prepare gesture command error\n");
			}
			temp[0] = 0x01;
			temp[1] = 0x0A;
			temp[2] = core_gesture->mode + 1;
			ipio_info("write gesture command 0x01 0x0A, 0x%02X\n", core_gesture->mode + 1);
			if ((core_write(core_config->slave_i2c_addr, temp, 3)) < 0) {
				ipio_info("write gesture command error\n");
			}
			mutex_unlock(&ipd->report_mutex);
		} else {

			ipio_info("enable gesture mode\n");
			core_config->isEnableGesture = true;
			g_gesture = 1;
			core_gesture->mode = GESTURE_INFO_MPDE;
		}


	} else if (ptr[0] == '0') {
		ipio_info("disable gesture mode\n");
		core_config->isEnableGesture = false;
		g_gesture = 0;
	} else if(ptr[0] == '2') {
		if ((core_gesture->suspend) && (gesture_done == true)) {
			/* sleep in */
			ipio_debug(DEBUG_CONFIG, "core_config->isEnableGesture = false, enter gesture sleep\n");
			core_config->isEnableGesture = false;
			mutex_lock(&ipd->report_mutex);
			core_config_sleep_ctrl(false);
			mutex_unlock(&ipd->report_mutex);
		} else {
			ipio_debug(DEBUG_CONFIG, "gesture sleep doing nothing\n");
		}
		ipio_info("psensor_close = true\n");
		psensor_close = true;
	}
	else {
		ipio_err("Unknown command\n");
	}
	/*if (strcmp(cmd, '1') == 0) {
		ipio_info("enable gesture mode\n");
		core_config->isEnableGesture = true;
		core_gesture->mode = GESTURE_INFO_MPDE;
	} else if (strcmp(cmd, "0") == 0) {
		ipio_info("disable gesture mode\n");
		core_config->isEnableGesture = false;
	} else
		ipio_err("Unknown command\n");*/

	kfree(ptr);
	return size;
}

static ssize_t oppo_proc_gesture_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	//size_t size = 0;
	char *ptr = NULL;
	//char result_enable[20] = "gesture=1";
	//char result_disable[20] = "gesture=0";
	//size = ARRAY_SIZE(result_enable);
	if ( *ppos ) {
	    printk("is already read the file\n");
        return 0;
	}
    *ppos += count;
    //min = min(count,size);
    ptr = kzalloc(count,GFP_KERNEL);
	if(ptr == NULL){
		printk("allocate memory fail\n");
		return -1;
	}
	//if(core_config->isEnableGesture)
    len = snprintf(ptr, count,"%x\n",core_config->isEnableGesture);

    ret = copy_to_user(buf,ptr,len);

	kfree(ptr);
	return len;
}

#endif
static ssize_t ilitek_proc_check_battery_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	uint32_t len = 0;

	if (*pPos != 0)
		return 0;

	memset(g_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));

	len = sprintf(g_user_buf, "%d", ipd->isEnablePollCheckPower);

	ipio_debug(DEBUG_BATTERY, "isEnablePollCheckPower = %d\n", ipd->isEnablePollCheckPower);

	res = copy_to_user((uint32_t *) buff, &ipd->isEnablePollCheckPower, len);
	if (res < 0) {
		ipio_err("Failed to copy data to user space\n");
	}

	*pPos = len;

	return len;
}

static ssize_t ilitek_proc_check_battery_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	char cmd[10] = { 0 };

	if (buff != NULL) {
		res = copy_from_user(cmd, buff, size - 1);
		if (res < 0) {
			ipio_err("copy data from user space, failed\n");
			return -1;
		}
	}

	ipio_debug(DEBUG_BATTERY, "size = %d, cmd = %s\n", (int)size, cmd);

#ifdef ENABLE_BATTERY_CHECK
	if (strcmp(cmd, "on") == 0) {
		ipio_debug(DEBUG_BATTERY, "Start the thread of check power status\n");
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);
		ipd->isEnablePollCheckPower = true;
	} else if (strcmp(cmd, "off") == 0) {
		ipio_debug(DEBUG_BATTERY, "Cancel the thread of check power status\n");
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		ipd->isEnablePollCheckPower = false;
	} else
		ipio_err("Unknown command\n");
#else
	ipio_err("You need to enable its MACRO before operate it.\n");
#endif

	return size;
}
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/21,Modify for ito test
static ssize_t wt_mptest_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int len = 0;
	size_t count = 0;
	char result_pass[40] = "result=1";
	char result_fail[20] = "result=0";
	char result_osc_offset[40] = {0};
	char *ptr = NULL;
	uint32_t res = 0;
	bool final_result = false;
	int osc_offset = 65535;
	int osc_result = 0;
	int osc_test = 0;
	int osc_threshold_min = 0;
	int osc_threshold_max = 0;
	uint8_t osc_org_data[4] = {0};

	mp_test_result = false;

	if (*pPos != 0)
		return 0;

	ptr = (char*)kzalloc(256,GFP_KERNEL);
	if (ptr == NULL) {
		ipio_err("failed to alloc ptr memory\n");
		return 0;
	}

	if(core_gesture->suspend){
		printk("%s,not in resume,can not to do mp test\n",__func__);
		count = ARRAY_SIZE(result_pass);
		len = snprintf(ptr, count,"%s\n",result_pass);
		res = copy_to_user(buff,ptr,len);
		*pPos += len;
		return len;
	}
#ifdef ILITEK_ESD_CHECK
	ilitek_cancel_esd_check();
#endif
	mutex_lock(&ipd->report_mutex);

	if (core_parser_path(ili_fw->mp_ini_name) < 0) {
		ipio_err("Failed to parsing INI file\n");
		goto out;
	}

	/* Init MP structure */
	if(core_mp_init() < 0) {
		ipio_err("Failed to init mp\n");
		goto out;
	}
	core_mp->oppo_run = true;
	core_mp->oppo_lcm = false;

	/* Switch to test mode */
	core_fr_mode_control(&protocol->test_mode);

	ilitek_platform_disable_irq();
	ilitek_osc_check();


	/* Do not chang the sequence of test */
	core_mp_run_test("Noise Peak To Peak(With Panel)", true);
	core_mp_run_test("Noise Peak to Peak(IC Only)", true);
	core_mp_run_test("Short Test -ILI9881", true);
	core_mp_run_test("Open Test(integration)_SP", true);
	core_mp_run_test("Raw Data(Have BK)", true);
	core_mp_run_test("Calibration Data(DAC)", true);
	core_mp_run_test("Raw Data(No BK)", true);
	core_mp_run_test("Doze Raw Data", true);
	core_mp_run_test("Doze Peak To Peak", true);

	core_mp_show_result();

	core_mp->oppo_run = false;

	if (core_mp->osc_result == MP_FAIL && core_mp->osc_test == 1) {
		osc_result = 1;
	}
	else {
		osc_result = 0;
	}
	osc_test = core_mp->osc_test;
	osc_offset = core_mp->osc_offset;
	sprintf(result_osc_offset,"%s%d.%d",(osc_offset < 0 && osc_offset > -10) ? "-" : " ", osc_offset / 10, abs(osc_offset % 10));
	if(osc_result == 0){
		strcat(result_osc_offset,":pass");
	}else{
		strcat(result_osc_offset,":fail");
	}

	osc_org_data[0] = core_mp->osc_org_data[0];
	osc_org_data[1] = core_mp->osc_org_data[1];
	osc_org_data[2] = core_mp->osc_org_data[2];
	osc_org_data[3] = core_mp->osc_org_data[3];
	osc_threshold_max = core_mp->osc_threshold_max;
	osc_threshold_min = core_mp->osc_threshold_min;

	core_mp_test_free();

	/* Switch to demo mode */
	core_fr_mode_control(&protocol->demo_mode);

#ifdef HOST_DOWNLOAD
	ilitek_platform_tp_hw_reset(true);
#endif

	ilitek_platform_enable_irq();
	ipio_info("MP Test mp_test_result = %d\n", mp_test_result);
		if (core_config->core_type == CORE_TYPE_B) {
			if (mp_test_result == true)
			{
				final_result = true;
			}
			else
			{
				final_result = false;
			}
		} else {
			if (osc_test) {
				if((mp_test_result == true)&&(osc_result == 0))
				{
					final_result = true;
				} else if((mp_test_result == false)||(osc_result == 1)){
					final_result = false;
					ipio_info("Test Failed osc_result = %d\n", osc_result);
				}
			} else {
				if (mp_test_result == true)
				{
					final_result = true;
				}
				else
				{
					final_result = false;
				}
			}
		}

	if (final_result == true)
	{
		count = ARRAY_SIZE(result_pass);
		count += ARRAY_SIZE(result_osc_offset);
		//len = snprintf(ptr, size,"%s\n",result_pass);
	}
	else
	{
		count = ARRAY_SIZE(result_fail);
		count += ARRAY_SIZE(result_osc_offset);
		//len = snprintf(ptr, size,"%s\n",result_fail);
	}
	if (final_result == true)
	{
		//size = ARRAY_SIZE(result_pass);
		len = snprintf(ptr, count,"%s;%s\n",result_pass,result_osc_offset);
	}
	else
	{
		//size = ARRAY_SIZE(result_fail);
		len = snprintf(ptr, count,"%s;%s\n",result_fail,result_osc_offset);
	}
	res = copy_to_user(buff,ptr,len);
	if (res < 0)
	{
	  ipio_err("Failed to copy data to user space\n");
	}
	ipio_info("MP Test DONE\n");
	out:
		mutex_unlock(&ipd->report_mutex);
#ifdef ILITEK_ESD_CHECK
		ilitek_start_esd_check();
#endif
	//mutex_unlock(&ipd->report_mutex);
	*pPos = len;
	kfree(ptr);
	return len;
}
#endif
static ssize_t ilitek_proc_fw_process_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	uint32_t len = 0;

	/*
	 * If file position is non-zero,  we assume the string has been read
	 * and indicates that there is no more data to be read.
	 */
	if (*pPos != 0)
		return 0;

	memset(g_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));

	len = sprintf(g_user_buf, "%02d", core_firmware->update_status);

	ipio_debug(DEBUG_FIRMWARE,"update status = %d\n", core_firmware->update_status);

	res = copy_to_user((uint32_t *) buff, &core_firmware->update_status, len);
	if (res < 0) {
		ipio_err("Failed to copy data to user space");
	}

	*pPos = len;

	return len;
}

/*
 * To avoid the restriction of selinux, we assigned a fixed path where locates firmware file,
 * reading (cat) this node to notify driver running the upgrade process from user space.
 */
static ssize_t ilitek_proc_fw_upgrade_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	uint32_t len = 0;

	ipio_debug(DEBUG_FIRMWARE,"Preparing to upgarde firmware\n");

	if (*pPos != 0)
		return 0;

	ilitek_platform_disable_irq();

	if (ipd->isEnablePollCheckPower)
		cancel_delayed_work_sync(&ipd->check_power_status_work);
#ifdef HOST_DOWNLOAD
	oppo_platform_tp_hw_reset(true);
#else
	res = core_firmware_upgrade(UPDATE_FW_PATH, false);
#endif
	ilitek_platform_enable_irq();

	if (ipd->isEnablePollCheckPower)
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);

	if (res < 0) {
		core_firmware->update_status = res;
		ipio_err("Failed to upgrade firwmare\n");
	} else {
		core_firmware->update_status = 100;
		ipio_debug(DEBUG_FIRMWARE,"Succeed to upgrade firmware\n");
	}

	*pPos = len;

	return len;
}

static ssize_t ilitek_proc_iram_upgrade_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	uint32_t len = 0;

	ipio_debug(DEBUG_FIRMWARE,"Preparing to upgarde firmware by IRAM\n");

	if (*pPos != 0)
		return 0;

	ilitek_platform_disable_irq();

	res = core_firmware_upgrade(UPDATE_FW_PATH, true);

	ilitek_platform_enable_irq();

	if (res < 0) {
		/* return the status to user space even if any error occurs. */
		core_firmware->update_status = res;
		ipio_err("Failed to upgrade firwmare by IRAM, res = %d\n", res);
	} else {
		ipio_debug(DEBUG_FIRMWARE,"Succeed to upgrade firmware by IRAM\n");
	}

	*pPos = len;

	return len;
}
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
static ssize_t oppo_proc_register_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
		int ret = 0;
		int len = 0;
		char *ptr = NULL;
		int i=0;
		if (!ILITEK_PROC_SEND_FLAG)
		{
		    ptr = (char*)kzalloc(count,GFP_KERNEL);
			len += snprintf(ptr+len, count-len,"0x%2.2X,0x%2.2X\n",g_user_buf[i],g_user_len);
		    ret = copy_to_user(buff,ptr,len);
		    ILITEK_PROC_SEND_FLAG = 1;
		}
		else
		{
		    ILITEK_PROC_SEND_FLAG = 0;
		}
		return len;
}

/* for debug */
static ssize_t oppo_proc_register_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0, count = 0, i;
	//int w_len = 0, r_len = 0, i2c_delay = 0;
	char cmd[512] = { 0 };
	char *token = NULL, *cur = NULL;
	uint8_t *data = NULL;

	if (buff != NULL) {
		res = copy_from_user(cmd, buff, size - 1);
		if (res < 0) {
			ipio_err("copy data from user space, failed\n");
			return -1;
		}
	}

	ipio_info("size = %d, cmd = %s\n", (int)size, cmd);

	token = cur = cmd;
	data = kmalloc(512 * sizeof(uint8_t), GFP_KERNEL);
	memset(data, 0, 512);
	//data[count] = str2hex(cmd);
    //count++;
	while ((token = strsep(&cur, ",")) != NULL) {
		data[count] = katoi(token);
		ipio_info("token = %s\n",token);
		//ipio_info("data[%d] = %x\n",count, data[count]);
		count++;
	}
	for(i=0;i<count;i++)
	{
	   ipio_info("data[%d] = %d\n", i,data[i]);
	   g_user_buf[i]=data[i];
	}
	g_user_len =count;
	ipio_kfree((void **)&data);
	return size;

}

#ifdef ILITEK_EDGE_LIMIT
#if 1

#endif
struct ilitek_limit_data edge_limit_data;
#if 1
static ssize_t ilitek_limit_area_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    char buf[8] = {0};
    int  temp;
	if (buffer != NULL)
	{
	    if (copy_from_user(buf, buffer, count)) {
	        ipio_err("%s: read proc input error.\n", __func__);
	        return count;
	    }
	}
    sscanf(buf, "%x", &temp);

    if (temp < 0 || temp > 10)
        return count;


    edge_limit_data.edge_limit.limit_area = temp;
    edge_limit_data.edge_limit.left_x1    = (edge_limit_data.edge_limit.limit_area*1000)/100;
    edge_limit_data.edge_limit.right_x1   = TOUCH_SCREEN_X_MAX - edge_limit_data.edge_limit.left_x1;
    edge_limit_data.edge_limit.left_x2    = 2 * edge_limit_data.edge_limit.left_x1;
    edge_limit_data.edge_limit.right_x2   = TOUCH_SCREEN_X_MAX - (2 * edge_limit_data.edge_limit.left_x1);

    ipio_info("limit_area = %d; left_x1 = %d; right_x1 = %d; left_x2 = %d; right_x2 = %d\n",
           edge_limit_data.edge_limit.limit_area, edge_limit_data.edge_limit.left_x1, edge_limit_data.edge_limit.right_x1, edge_limit_data.edge_limit.left_x2, edge_limit_data.edge_limit.right_x2);

    edge_limit_data.edge_limit.left_y1    = (edge_limit_data.edge_limit.limit_area*1000)/100;
    edge_limit_data.edge_limit.right_y1   =  edge_limit_data.edge_limit.left_y1;
    edge_limit_data.edge_limit.left_y2    = TOUCH_SCREEN_Y_MAX-2 * edge_limit_data.edge_limit.left_y1;
    edge_limit_data.edge_limit.right_y2   = TOUCH_SCREEN_Y_MAX - (2 * edge_limit_data.edge_limit.left_y1);

    return count;

}

static ssize_t ilitek_limit_area_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char *ptr = NULL;
	if ( *ppos ) {
    ipio_info("is already read the file\n");
    return 0;
	}
    *ppos += count;
	ptr = (char*)kzalloc(count,GFP_KERNEL);
	len += snprintf(ptr+len, count-len,"left(x1,y1)=(%d,%d)    ",edge_limit_data.edge_limit.left_x1,edge_limit_data.edge_limit.left_y1);
	len += snprintf(ptr+len, count-len,"left(x2,y2)=(%d,%d)\n",edge_limit_data.edge_limit.left_x2,edge_limit_data.edge_limit.left_y2);
	len += snprintf(ptr+len, count-len,"right(x1,y1)=(%d,%d)    ",edge_limit_data.edge_limit.right_x1,edge_limit_data.edge_limit.right_y1);
	len += snprintf(ptr+len, count-len,"right(x2,y2)=(%d,%d)\n",edge_limit_data.edge_limit.right_x2,edge_limit_data.edge_limit.right_y2);
	ret = copy_to_user(buffer,ptr,len);
	*ppos = len;
	return len;
}

#endif
static ssize_t ilitek_limit_control_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8] = {0};
	int  temp;
	uint8_t cmd[3] = {0};
	cmd[0] = 0x01;
	cmd[1] = 0x12;
	ipio_info("\n");
	if (buffer != NULL)
	{
		if (copy_from_user(buf, buffer, count)) {
			ipio_err("%s: read proc input error.\n", __func__);
			return count;
		}
	}
	sscanf(buf, "%x", &temp);
	if (temp > 0x1F) {
        ipio_info("%s: temp = 0x%x > 0x1F \n", __func__, temp);
        return count;
    }
	ipio_info("\n");
	mutex_lock(&ipd->report_mutex);
	#if 0
	edge_limit_data.limit_data = temp;
	edge_limit_data.limit_edge_enable = temp&0x01;
	edge_limit_data.limit_corner_ld_enable=(temp>>1)&0x01;
	edge_limit_data.limit_corner_lu_enable=(temp>>2)&0x01;
	edge_limit_data.limit_corner_rd_enable=(temp>>3)&0x01;
	edge_limit_data.limit_corner_ru_enable=(temp>>4)&0x01;
	ipio_info("limit_edge_enable=%d,limit_corner_ld_enable=%d,limit_corner_lu_enable=%d,limit_corner_rd_enable=%d,limit_corner_ru_enable=%d\n",edge_limit_data.limit_edge_enable,edge_limit_data.limit_corner_ld_enable,
edge_limit_data.limit_corner_lu_enable,edge_limit_data.limit_corner_rd_enable,edge_limit_data.limit_corner_ru_enable);
		#endif
		if (core_gesture->suspend==false) {
			if (core_config->direction == 0) {
				cmd[2] = 0x01;
				core_write(core_config->slave_i2c_addr, cmd, 3);
			} else if (core_config->direction == 1) {
				cmd[2] = 0x02;
				core_write(core_config->slave_i2c_addr, cmd, 3);
			} else if (core_config->direction == 2) {
				cmd[2] = 0x00;
				core_write(core_config->slave_i2c_addr, cmd, 3);
			}
		}
	mutex_unlock(&ipd->report_mutex);
    return count;
}

static ssize_t ilitek_limit_control_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char *ptr = NULL;
	if ( *ppos ) {
    ipio_info("is already read the file\n");
    return 0;
	}
    *ppos += count;
	ptr = (char*)kzalloc(count,GFP_KERNEL);
	len += snprintf(ptr+len, count-len,"limit_edge_enable=%d\n",edge_limit_data.limit_edge_enable);
	len += snprintf(ptr+len, count-len,"limit_corner_ld_enable=%d\n",edge_limit_data.limit_corner_ld_enable);
	len += snprintf(ptr+len, count-len,"limit_corner_lu_enable=%d\n",edge_limit_data.limit_corner_lu_enable);
	len += snprintf(ptr+len, count-len,"limit_corner_rd_enable=%d\n",edge_limit_data.limit_corner_rd_enable);
	len += snprintf(ptr+len, count-len,"limit_corner_ru_enable=%d\n",edge_limit_data.limit_corner_ru_enable);
	ret = copy_to_user(buffer,ptr,len);
	*ppos = len;
	return len;
}

#endif

static ssize_t ilitek_direction_read(struct file *file, char __user *userbuf, size_t count, loff_t *ppos)
{
       uint8_t buf[8];
       uint8_t len;
       int ret;
       len = 2;
       if(*ppos) {
              return 0;    /* the end */
       }
       sprintf(buf,"%d",core_config->direction);
       len += sprintf(buf+len, "\n");
       ret = copy_to_user(userbuf,buf,len);
       if(ret<0)
              ipio_err("copy to user error\n");

       *ppos += len;
       return len;
}

static ssize_t ilitek_direction_write(struct file *file, const char __user *userbuf, size_t count, loff_t *ppos)
{
       uint8_t buf[5] = {0};
       uint8_t cmd[3] = {0};
       cmd[0] = 0x01;
       cmd[1] = 0x12;

       if(count == 0) {
              ipio_err("count is 0 error\n");
              return -EINVAL;
       }
       ipio_info("count:%zd\n",count);
       if(copy_from_user(buf, userbuf, count)) {
              ipio_err("input value error\n");
              return -EINVAL;
       }

       core_config->direction = buf[0] -'0';
	   mutex_lock(&ipd->report_mutex);
       if (core_config->direction == 0) {
              cmd[2] = 0x01;
              core_write(core_config->slave_i2c_addr, cmd, 3);
       } else if (core_config->direction == 1) {
              cmd[2] = 0x02;
              core_write(core_config->slave_i2c_addr, cmd, 3);
       } else if (core_config->direction == 2) {
              cmd[2] = 0x00;
              core_write(core_config->slave_i2c_addr, cmd, 3);
       }
	   mutex_unlock(&ipd->report_mutex);
       ipio_info("core_config->direction:%d",core_config->direction);
       return count;
}

#endif
/* for debug */
static ssize_t ilitek_proc_ioctl_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	uint32_t len = 0;
	uint8_t cmd[2] = { 0 };

	if (*pPos != 0)
		return 0;

	if (size < 4095) {
		res = copy_from_user(cmd, buff, size - 1);
		if (res < 0) {
			ipio_err("copy data from user space, failed\n");
			return -1;
		}
	}

	ipio_info("size = %d, cmd = %d", (int)size, cmd[0]);

	/* test */
	if (cmd[0] == 0x1) {
		ipio_info("HW Reset\n");
		ilitek_platform_tp_hw_reset(true);
	} else if (cmd[0] == 0x02) {
		ipio_info("Disable IRQ\n");
		ilitek_platform_disable_irq();
	} else if (cmd[0] == 0x03) {
		ipio_info("Enable IRQ\n");
		ilitek_platform_enable_irq();
	} else if (cmd[0] == 0x04) {
		ipio_info("Get Chip id\n");
		core_config_get_chip_id();
	}

	*pPos = len;

	return len;
}

/* for debug */
static ssize_t ilitek_proc_ioctl_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int res = 0, count = 0, i;
	int w_len = 0, r_len = 0, i2c_delay = 0;
	char cmd[512] = { 0 };
	char *token = NULL, *cur = NULL;
	uint8_t i2c[256] = { 0 };
	uint8_t *data = NULL;

	if (buff != NULL) {
		res = copy_from_user(cmd, buff, size - 1);
		if (res < 0) {
			ipio_info("copy data from user space, failed\n");
			return -1;
		}
	}

	ipio_info("size = %d, cmd = %s\n", (int)size, cmd);

	token = cur = cmd;

	data = kmalloc(512 * sizeof(uint8_t), GFP_KERNEL);
	memset(data, 0, 512);

	while ((token = strsep(&cur, ",")) != NULL) {
		data[count] = str2hex(token);
		ipio_info("data[%d] = %x\n",count, data[count]);
		count++;
	}

	ipio_info("cmd = %s\n", cmd);

	if (strcmp(cmd, "reset") == 0) {
		ipio_info("HW Reset\n");
		ilitek_platform_tp_hw_reset(true);
	} else if (strcmp(cmd, "disirq") == 0) {
		ipio_info("Disable IRQ\n");
		ilitek_platform_disable_irq();
	} else if (strcmp(cmd, "enairq") == 0) {
		ipio_info("Enable IRQ\n");
		ilitek_platform_enable_irq();
	} else if (strcmp(cmd, "getchip") == 0) {
		ipio_info("Get Chip id\n");
		core_config_get_chip_id();
	} else if (strcmp(cmd, "dispcc") == 0) {
		ipio_info("disable phone cover\n");
		core_config_phone_cover_ctrl(false);
	} else if (strcmp(cmd, "enapcc") == 0) {
		ipio_info("enable phone cover\n");
		core_config_phone_cover_ctrl(true);
	} else if (strcmp(cmd, "disfsc") == 0) {
		ipio_info("disable finger sense\n")
		    core_config_finger_sense_ctrl(false);
	} else if (strcmp(cmd, "enafsc") == 0) {
		ipio_info("enable finger sense\n");
		core_config_finger_sense_ctrl(true);
	} else if (strcmp(cmd, "disprox") == 0) {
		ipio_info("disable proximity\n");
		core_config_proximity_ctrl(false);
	} else if (strcmp(cmd, "enaprox") == 0) {
		ipio_info("enable proximity\n");
		core_config_proximity_ctrl(true);
	} else if (strcmp(cmd, "disglove") == 0) {
		ipio_info("disable glove function\n");
		core_config_glove_ctrl(false, false);
	} else if (strcmp(cmd, "enaglove") == 0) {
		ipio_info("enable glove function\n");
		core_config_glove_ctrl(true, false);
	} else if (strcmp(cmd, "glovesl") == 0) {
		ipio_info("set glove as seamless\n");
		core_config_glove_ctrl(true, true);
	} else if (strcmp(cmd, "enastylus") == 0) {
		ipio_info("enable stylus\n");
		core_config_stylus_ctrl(true, false);
	} else if (strcmp(cmd, "disstylus") == 0) {
		ipio_info("disable stylus\n");
		core_config_stylus_ctrl(false, false);
	} else if (strcmp(cmd, "stylussl") == 0) {
		ipio_info("set stylus as seamless\n");
		core_config_stylus_ctrl(true, true);
	} else if (strcmp(cmd, "tpscan_ab") == 0) {
		ipio_info("set TP scan as mode AB\n");
		core_config_tp_scan_mode(true);
	} else if (strcmp(cmd, "tpscan_b") == 0) {
		ipio_info("set TP scan as mode B\n");
		core_config_tp_scan_mode(false);
	} else if (strcmp(cmd, "phone_cover") == 0) {
		ipio_info("set size of phone conver window\n");
		core_config_set_phone_cover(data);
	} else if (strcmp(cmd, "i2c_w") == 0) {
		w_len = data[1];
		ipio_info("w_len = %d\n", w_len);

		for (i = 0; i < w_len; i++) {
			i2c[i] = data[2 + i];
			ipio_info("i2c[%d] = %x\n", i, i2c[i]);
		}

		core_write(core_config->slave_i2c_addr, i2c, w_len);
	} else if (strcmp(cmd, "i2c_r") == 0) {
		r_len = data[1];
		ipio_info("r_len = %d\n", r_len);

		core_read(core_config->slave_i2c_addr, &i2c[0], r_len);

		for (i = 0; i < r_len; i++)
			ipio_info("i2c[%d] = %x\n", i, i2c[i]);
	} else if (strcmp(cmd, "i2c_w_r") == 0) {
		w_len = data[1];
		r_len = data[2];
		i2c_delay = data[3];
		ipio_info("w_len = %d, r_len = %d, delay = %d\n", w_len, r_len, i2c_delay);

		for (i = 0; i < w_len; i++) {
			i2c[i] = data[4 + i];
			ipio_info("i2c[%d] = %x\n", i, i2c[i]);
		}

		core_write(core_config->slave_i2c_addr, i2c, w_len);

		memset(i2c, 0, sizeof(i2c));
		msleep(i2c_delay);

		core_read(core_config->slave_i2c_addr, &i2c[0], r_len);

		for (i = 0; i < r_len; i++)
			ipio_info("i2c[%d] = %x\n", i, i2c[i]);
	} else {
		ipio_err("Unknown command\n");
	}

	ipio_kfree((void **)&data);
	return size;
}

static long ilitek_proc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int res = 0, length = 0;
	uint8_t szBuf[512] = { 0 };
	static uint16_t i2c_rw_length = 0;
	uint32_t id_to_user[3] = {0};
	char dbg[10] = { 0 };

	ipio_debug(DEBUG_IOCTL, "cmd = %d\n", _IOC_NR(cmd));

	if (_IOC_TYPE(cmd) != ILITEK_IOCTL_MAGIC) {
		ipio_err("The Magic number doesn't match\n");
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > ILITEK_IOCTL_MAXNR) {
		ipio_err("The number of ioctl doesn't match\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case ILITEK_IOCTL_I2C_WRITE_DATA:
		res = copy_from_user(szBuf, (uint8_t *) arg, i2c_rw_length);
		if (res < 0) {
			ipio_err("Failed to copy data from user space\n");
		} else {
			res = core_write(core_config->slave_i2c_addr, &szBuf[0], i2c_rw_length);
			if (res < 0) {
				ipio_err("Failed to write data via i2c\n");
			}
		}
		break;

	case ILITEK_IOCTL_I2C_READ_DATA:
		res = core_read(core_config->slave_i2c_addr, szBuf, i2c_rw_length);
		if (res < 0) {
			ipio_err("Failed to read data via i2c\n");
		} else {
			res = copy_to_user((uint8_t *) arg, szBuf, i2c_rw_length);
			if (res < 0) {
				ipio_err("Failed to copy data to user space\n");
			}
		}
		break;

	case ILITEK_IOCTL_I2C_SET_WRITE_LENGTH:
	case ILITEK_IOCTL_I2C_SET_READ_LENGTH:
		i2c_rw_length = arg;
		break;

	case ILITEK_IOCTL_TP_HW_RESET:
		ilitek_platform_tp_hw_reset(true);
		break;

	case ILITEK_IOCTL_TP_POWER_SWITCH:
		ipio_info("Not implemented yet\n");
		break;

	case ILITEK_IOCTL_TP_REPORT_SWITCH:
		res = copy_from_user(szBuf, (uint8_t *) arg, 1);
		if (res < 0) {
			ipio_err("Failed to copy data from user space\n");
		} else {
			if (szBuf[0]) {
				core_fr->isEnableFR = true;
				ipio_debug(DEBUG_IOCTL, "Function of finger report was enabled\n");
			} else {
				core_fr->isEnableFR = false;
				ipio_debug(DEBUG_IOCTL, "Function of finger report was disabled\n");
			}
		}
		break;

	case ILITEK_IOCTL_TP_IRQ_SWITCH:
		res = copy_from_user(szBuf, (uint8_t *) arg, 1);
		if (res < 0) {
			ipio_err("Failed to copy data from user space\n");
		} else {
			if (szBuf[0]) {
				ilitek_platform_enable_irq();
			} else {
				ilitek_platform_disable_irq();
			}
		}
		break;

	case ILITEK_IOCTL_TP_DEBUG_LEVEL:
		res = copy_from_user(dbg, (uint32_t *) arg, sizeof(uint32_t));
		if (res < 0) {
			ipio_err("Failed to copy data from user space\n");
		} else {
			ipio_debug_level = katoi(dbg);
			ipio_info("ipio_debug_level = %d", ipio_debug_level);
		}
		break;

	case ILITEK_IOCTL_TP_FUNC_MODE:
		ipio_info("\n");
		res = copy_from_user(szBuf, (uint8_t *) arg, 3);
		if (res < 0) {
			ipio_err("Failed to copy data from user space\n");
		} else {
			core_write(core_config->slave_i2c_addr, &szBuf[0], 3);
		}
		ipio_info("\n");
		break;

	case ILITEK_IOCTL_TP_FW_VER:
		ipio_info("\n");
		res = core_config_get_fw_ver();
		if (res < 0) {
			ipio_err("Failed to get firmware version\n");
		} else {
			res = copy_to_user((uint8_t *) arg, core_config->firmware_ver, protocol->fw_ver_len);
			if (res < 0) {
				ipio_err("Failed to copy firmware version to user space\n");
			}
		}
		ipio_info("\n");
		break;

	case ILITEK_IOCTL_TP_PL_VER:
		ipio_info("\n");
		res = core_config_get_protocol_ver();
		if (res < 0) {
			ipio_err("Failed to get protocol version\n");
		} else {
			res = copy_to_user((uint8_t *) arg, core_config->protocol_ver, protocol->pro_ver_len);
			if (res < 0) {
				ipio_err("Failed to copy protocol version to user space\n");
			}
		}
		ipio_info("\n");
		break;

	case ILITEK_IOCTL_TP_CORE_VER:
		ipio_info("\n");
		res = core_config_get_core_ver();
		if (res < 0) {
			ipio_err("Failed to get core version\n");
		} else {
			res = copy_to_user((uint8_t *) arg, core_config->core_ver, protocol->core_ver_len);
			if (res < 0) {
				ipio_err("Failed to copy core version to user space\n");
			}
		}
		ipio_info("\n");
		break;

	case ILITEK_IOCTL_TP_DRV_VER:
		length = sprintf(szBuf, "%s", DRIVER_VERSION);
		if (!length) {
			ipio_err("Failed to convert driver version from definiation\n");
		} else {
			res = copy_to_user((uint8_t *) arg, szBuf, length);
			if (res < 0) {
				ipio_err("Failed to copy driver ver to user space\n");
			}
		}
		break;

	case ILITEK_IOCTL_TP_CHIP_ID:
		res = core_config_get_chip_id();
		if (res < 0) {
			ipio_err("Failed to get chip id\n");
		} else {
			id_to_user[0] = core_config->chip_id << 16 | core_config->chip_type;
			id_to_user[1] = core_config->chip_otp_id;
			id_to_user[2] = core_config->chip_ana_id;

			res = copy_to_user((uint32_t *) arg, id_to_user, sizeof(id_to_user));
			if (res < 0) {
				ipio_err("Failed to copy chip id to user space\n");
			}
		}
		break;

	case ILITEK_IOCTL_TP_NETLINK_CTRL:
		res = copy_from_user(szBuf, (uint8_t *) arg, 1);
		if (res < 0) {
			ipio_err("Failed to copy data from user space\n");
		} else {
			if (szBuf[0]) {
				core_fr->isEnableNetlink = true;
				ipio_debug(DEBUG_IOCTL, "Netlink has been enabled\n");
			} else {
				core_fr->isEnableNetlink = false;
				ipio_debug(DEBUG_IOCTL, "Netlink has been disabled\n");
			}
		}
		break;

	case ILITEK_IOCTL_TP_NETLINK_STATUS:
		ipio_debug(DEBUG_IOCTL, "Netlink is enabled : %d\n", core_fr->isEnableNetlink);
		res = copy_to_user((int *)arg, &core_fr->isEnableNetlink, sizeof(int));
		if (res < 0) {
			ipio_err("Failed to copy chip id to user space\n");
		}
		break;

	case ILITEK_IOCTL_TP_MODE_CTRL:
		res = copy_from_user(szBuf, (uint8_t *) arg, 4);
		if (res < 0) {
			ipio_err("Failed to copy data from user space\n");
		} else {
			core_fr_mode_control(szBuf);
		}
		break;

	case ILITEK_IOCTL_TP_MODE_STATUS:
		ipio_debug(DEBUG_IOCTL, "Current firmware mode : %d", core_fr->actual_fw_mode);
		res = copy_to_user((int *)arg, &core_fr->actual_fw_mode, sizeof(int));
		if (res < 0) {
			ipio_err("Failed to copy chip id to user space\n");
		}
		break;

	case ILITEK_IOCTL_ICE_MODE_SWITCH:
		res = copy_from_user(szBuf, (uint8_t *) arg, 1);
		if (res < 0) {
			ipio_err("Failed to copy data from user space\n");
		} else {
			if (szBuf[0]) {
				core_config->icemodeenable = true;
			} else {
				core_config->icemodeenable = false;
			}
		}
		break;

	default:
		res = -ENOTTY;
		break;
	}

	return res;
}
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
struct proc_dir_entry *proc_dir_oppo;
struct proc_dir_entry *proc_dir_debug_info;
struct proc_dir_entry *proc_baseline_test;
struct proc_dir_entry *proc_blackscreen_test;
struct proc_dir_entry *proc_coordinate;
struct proc_dir_entry *proc_CDC_delta;
struct proc_dir_entry *proc_CDC_rawdata;
struct proc_dir_entry *proc_main_register;
struct proc_dir_entry *proc_oppo_debug_level;
struct proc_dir_entry *proc_oppo_register_info;
struct proc_dir_entry *proc_oppo_tp_direction_dir;
#endif
struct proc_dir_entry *proc_dir_ilitek;
struct proc_dir_entry *proc_ioctl;
struct proc_dir_entry *proc_fw_process;
struct proc_dir_entry *proc_fw_upgrade;
struct proc_dir_entry *proc_iram_upgrade;
struct proc_dir_entry *proc_gesture;
struct proc_dir_entry *proc_debug_level;
struct proc_dir_entry *proc_mp_test;
struct proc_dir_entry *proc_debug_message;
struct proc_dir_entry *proc_debug_message_switch;

struct file_operations proc_ioctl_fops = {
	.unlocked_ioctl = ilitek_proc_ioctl,
	.read = ilitek_proc_ioctl_read,
	.write = ilitek_proc_ioctl_write,
};

struct file_operations proc_fw_process_fops = {
	.read = ilitek_proc_fw_process_read,
};
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/21,Modify for ito test
struct file_operations wt_open_test_fops = {
	.read = wt_mptest_read,
};
#endif
struct file_operations proc_fw_upgrade_fops = {
	.read = ilitek_proc_fw_upgrade_read,
};

struct file_operations proc_iram_upgrade_fops = {
	.read = ilitek_proc_iram_upgrade_read,
};

struct file_operations proc_gesture_fops = {
	.write = ilitek_proc_gesture_write,
	.read = ilitek_proc_gesture_read,
};

struct file_operations proc_check_battery_fops = {
	.write = ilitek_proc_check_battery_write,
	.read = ilitek_proc_check_battery_read,
};

struct file_operations proc_debug_level_fops = {
	.write = ilitek_proc_debug_level_write,
	.read = ilitek_proc_debug_level_read,
};
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
struct file_operations proc_oppo_debug_level_fops = {
	.write = ilitek_proc_oppo_debug_level_write,
};
#endif
struct file_operations proc_mp_test_fops = {
	.write = ilitek_proc_mp_test_write,
	.read = ilitek_proc_mp_test_read,
};
static const struct file_operations ilitek_direction_fops = {
       .owner = THIS_MODULE,
       .read = ilitek_direction_read,
       .write = ilitek_direction_write,
};
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
#if 0
struct file_operations proc_coordinate_fops = {
	.read = ilitek_proc_coordinate_read,
};
#endif
#ifdef ILITEK_GESTURE_WAKEUP
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void c_stop(struct seq_file *m, void *v)
{
	return;
}

static int32_t c_oppo_ili_coordinate_show(struct seq_file *m, void *v)
{
	struct ili_gesture_info *gesture = gesture_report_data;
	char tmp[256] = {0};
	printk("c_oppo_coordinate_show\n");
	sprintf(tmp, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d",
		gesture->gesture_type,
		gesture->Point_start.x, gesture->Point_start.y,
		gesture->Point_end.x, gesture->Point_end.y,
		gesture->Point_1st.x, gesture->Point_1st.y,
		gesture->Point_2nd.x, gesture->Point_2nd.y,
		gesture->Point_3rd.x, gesture->Point_3rd.y,
		gesture->Point_4th.x, gesture->Point_4th.y,
		gesture->clockwise);

	/* oppo gesture formate */
	seq_printf(m, "%s\n", tmp);

	return 0;
}

const struct seq_operations oppo_ili_coordinate_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_oppo_ili_coordinate_show
};

static int32_t oppo_ili_coordinate_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &oppo_ili_coordinate_seq_ops);
}

static const struct file_operations oppo_ili_coordinate_fops = {
	.owner = THIS_MODULE,
	.open = oppo_ili_coordinate_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

#endif
struct file_operations proc_oppo_mp_lcm_on_fops = {
	.read = oppo_proc_mp_lcm_on_read,
};
struct file_operations proc_oppo_gesture_fops = {
	.write = oppo_proc_gesture_write,
	.read = oppo_proc_gesture_read,
};
struct file_operations proc_irq_depth_fops = {
	.read = oppo_proc_irq_depth_read,
};
struct file_operations proc_i2c_device_fops = {
	.read = oppo_proc_i2c_device_read,
};
struct file_operations proc_oppo_upgrade_fw_fops = {
	.write = ilitek_proc_oppo_upgrade_fw_write,
};
struct file_operations proc_CDC_delta_fops = {
	.read = ilitek_proc_CDC_delta_read,
};
struct file_operations proc_rawdata_fops = {
	.read = ilitek_proc_rawdata_read,
};
struct file_operations proc_main_register_fops = {
	.read = ilitek_proc_main_register_read,
};
struct file_operations proc_oppo_register_info_fops = {
	.write =oppo_proc_register_write,
	.read = oppo_proc_register_read,
};


struct file_operations proc_black_screen_test_fops = {
	.read  = oppo_proc_black_screen_test_read,
	.write = oppo_proc_black_screen_test_write,
};

struct file_operations proc_game_switch_enable_fops = {
	.read  = oppo_proc_game_switch_read,
	.write = oppo_proc_game_switch_write,
};

#ifdef ILITEK_EDGE_LIMIT
struct file_operations ilitek_limit_control_ops =
{
    .read  = ilitek_limit_control_read,
    .write = ilitek_limit_control_write,
    .owner = THIS_MODULE,
};
#if 1
struct file_operations ilitek_limit_area_ops =
{
    .read  = ilitek_limit_area_read,
    .write = ilitek_limit_area_write,
    .owner = THIS_MODULE,
};

#endif
#endif

#endif
struct file_operations proc_debug_message_fops = {
	.write = ilitek_proc_debug_message_write,
	.read = ilitek_proc_debug_message_read,
};

struct file_operations proc_debug_message_switch_fops = {
	.read = ilitek_proc_debug_switch_read,
};

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/17, add presure report function
struct file_operations proc_debug_message_presure_debug = {
	.read  = ilitek_proc_debug_presure_read,
	.write = ilitek_proc_debug_presure_write,
};
struct file_operations ili9881_proc_fw_pc_counter_fops = {
	.read = ili9881_proc_fw_pc_counter_read,
};
struct file_operations ili9881_proc_otp_read_fops = {
	.read = ilitek_proc_otp_read,
};
struct file_operations ili9881_proc_otp_move_result_fops = {
	.read = ilitek_proc_otp_move_result,
};
#endif


/**
 * This struct lists all file nodes will be created under /proc filesystem.
 *
 * Before creating a node that you want, declaring its file_operations structure
 * is necessary. After that, puts the structure into proc_table, defines its
 * node's name in the same row, and the init function lterates the table and
 * creates all nodes under /proc.
 *
 */
typedef struct {
	char *name;
	struct proc_dir_entry *node;
	struct file_operations *fops;
	bool isCreated;
} proc_node_t;

proc_node_t proc_table[] = {
	{"ioctl", NULL, &proc_ioctl_fops, false},
	{"fw_process", NULL, &proc_fw_process_fops, false},
	{"fw_upgrade", NULL, &proc_fw_upgrade_fops, false},
	{"iram_upgrade", NULL, &proc_iram_upgrade_fops, false},
	{"gesture", NULL, &proc_gesture_fops, false},
	{"check_battery", NULL, &proc_check_battery_fops, false},
	{"debug_level", NULL, &proc_debug_level_fops, false},
	{"mp_test", NULL, &proc_mp_test_fops, false},
	{"debug_message", NULL, &proc_debug_message_fops, false},
	{"debug_message_switch", NULL, &proc_debug_message_switch_fops, false},

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/12/17, add presure report function
	{"presure_debug", NULL, &proc_debug_message_presure_debug, false},
	{"fw_pc_counter", NULL, &ili9881_proc_fw_pc_counter_fops, false},
	{"otp_read", NULL, &ili9881_proc_otp_read_fops, false},
	{"otp_move_result", NULL, &ili9881_proc_otp_move_result_fops, false},

#endif

};

#define NETLINK_USER 21
struct sock *_gNetLinkSkb;
struct nlmsghdr *_gNetLinkHead;
struct sk_buff *_gSkbOut;
int _gPID;

void netlink_reply_msg(void *raw, int size)
{
	int res;
	int msg_size = size;
	uint8_t *data = (uint8_t *) raw;

	ipio_debug(DEBUG_NETLINK, "The size of data being sent to user = %d\n", msg_size);
	ipio_debug(DEBUG_NETLINK, "pid = %d\n", _gPID);
	ipio_debug(DEBUG_NETLINK, "Netlink is enable = %d\n", core_fr->isEnableNetlink);

	if (core_fr->isEnableNetlink) {
		_gSkbOut = nlmsg_new(msg_size, 0);

		if (!_gSkbOut) {
			ipio_err("Failed to allocate new skb\n");
			return;
		}

		_gNetLinkHead = nlmsg_put(_gSkbOut, 0, 0, NLMSG_DONE, msg_size, 0);
		NETLINK_CB(_gSkbOut).dst_group = 0;	/* not in mcast group */

		/* strncpy(NLMSG_DATA(_gNetLinkHead), data, msg_size); */
		memcpy(nlmsg_data(_gNetLinkHead), data, msg_size);

		res = nlmsg_unicast(_gNetLinkSkb, _gSkbOut, _gPID);
		if (res < 0)
			ipio_err("Failed to send data back to user\n");
	}
}
EXPORT_SYMBOL(netlink_reply_msg);

static void netlink_recv_msg(struct sk_buff *skb)
{
	_gPID = 0;

	ipio_debug(DEBUG_NETLINK, "Netlink is enable = %d\n", core_fr->isEnableNetlink);

	_gNetLinkHead = (struct nlmsghdr *)skb->data;

	ipio_debug(DEBUG_NETLINK, "Received a request from client: %s, %d\n",
	    (char *)NLMSG_DATA(_gNetLinkHead), (int)strlen((char *)NLMSG_DATA(_gNetLinkHead)));

	/* pid of sending process */
	_gPID = _gNetLinkHead->nlmsg_pid;

	ipio_debug(DEBUG_NETLINK, "the pid of sending process = %d\n", _gPID);

	/* TODO: may do something if there's not receiving msg from user. */
	if (_gPID != 0) {
		ipio_err("The channel of Netlink has been established successfully !\n");
		core_fr->isEnableNetlink = true;
	} else {
		ipio_err("Failed to establish the channel between kernel and user space\n");
		core_fr->isEnableNetlink = false;
	}
}

static int netlink_init(void)
{
	int res = 0;

#if KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE
	_gNetLinkSkb = netlink_kernel_create(&init_net, NETLINK_USER, netlink_recv_msg, NULL, THIS_MODULE);
#else
	struct netlink_kernel_cfg cfg = {
		.input = netlink_recv_msg,
	};

	_gNetLinkSkb = netlink_kernel_create(&init_net, NETLINK_USER, &cfg);
#endif

	ipio_info("Initialise Netlink and create its socket\n");

	if (!_gNetLinkSkb) {
		ipio_err("Failed to create nelink socket\n");
		res = -EFAULT;
	}

	return res;
}

int ilitek_proc_init(void)
{
	int i = 0, res = 0;
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
	struct proc_dir_entry *proc_oppo_upgrade_fw_dir;
	struct proc_dir_entry *proc_oppo_gesture_dir;
	struct proc_dir_entry *proc_oppo_irq_depth_dir;
	struct proc_dir_entry *proc_oppo_i2c_device_test_dir;
	struct proc_dir_entry *proc_game_switch_enable;
#ifdef ILITEK_EDGE_LIMIT
	struct proc_dir_entry *proc_oppo_edge_limit_enable;
#if 1
	struct proc_dir_entry *proc_oppo_edge_limit_area;
#endif
#endif
#endif
#ifdef ODM_WT_EDIT
	//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/21,Modify for ito test
	struct proc_dir_entry *proc_mp_dir;
	struct proc_dir_entry *proc_mptest_node;
	proc_mp_dir = proc_mkdir("touchscreen", NULL);
	proc_mptest_node = proc_create("ctp_openshort_test", 0666, proc_mp_dir, &wt_open_test_fops);
#endif
	proc_dir_ilitek = proc_mkdir("ilitek", NULL);
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Add for oppo factory test
    proc_dir_oppo = proc_mkdir("touchpanel", NULL);
	proc_dir_debug_info = proc_mkdir("debug_info", proc_dir_oppo);
	//proc_baseline_test = proc_create("baseline_test", 0666, proc_dir_oppo, &proc_oppo_mp_test_fops);
	proc_baseline_test = proc_create("baseline_test", 0666, proc_dir_oppo, &proc_oppo_mp_lcm_on_fops);
	proc_blackscreen_test =proc_create("black_screen_test", 0666, proc_dir_oppo, &proc_black_screen_test_fops);
	proc_game_switch_enable =proc_create("game_switch_enable", 0666, proc_dir_oppo, &proc_game_switch_enable_fops);
#if 0
	proc_coordinate = proc_create("coordinate", 0666, proc_dir_oppo, &proc_coordinate_fops);
#endif
#ifdef ILITEK_GESTURE_WAKEUP
	proc_coordinate= proc_create("coordinate",0444,proc_dir_oppo,&oppo_ili_coordinate_fops);
if ( proc_coordinate == NULL )
	{
	  ipio_err("create proc/touchpanel/coordinate Failed!\n");
	  return -1;
	}
#endif

	proc_CDC_delta = proc_create("delta", 0666, proc_dir_debug_info, &proc_CDC_delta_fops);
	proc_CDC_rawdata = proc_create("baseline", 0666, proc_dir_debug_info, &proc_rawdata_fops);
	proc_main_register= proc_create("main_register", 0666, proc_dir_debug_info, &proc_main_register_fops);
	proc_oppo_debug_level =proc_create("debug_level", 0644, proc_dir_oppo, &proc_oppo_debug_level_fops);
	proc_oppo_register_info =proc_create("oppo_register_info", 0664, proc_dir_oppo, &proc_oppo_register_info_fops);
	proc_oppo_upgrade_fw_dir =proc_create("tp_fw_update", 0644, proc_dir_oppo, &proc_oppo_upgrade_fw_fops);
	proc_oppo_gesture_dir =proc_create("double_tap_enable", 0666, proc_dir_oppo, &proc_oppo_gesture_fops);
	proc_oppo_irq_depth_dir =proc_create("irq_depth", 0666, proc_dir_oppo, &proc_irq_depth_fops);
	proc_oppo_i2c_device_test_dir =proc_create("i2c_device_test", 0666, proc_dir_oppo, &proc_i2c_device_fops);
#ifdef ILITEK_EDGE_LIMIT
	proc_oppo_edge_limit_enable = proc_create("oppo_tp_limit_enable", 0666, proc_dir_oppo, &ilitek_limit_control_ops);
#if 1
	proc_oppo_edge_limit_area = proc_create("oppo_tp_limit_area", 0666, proc_dir_oppo, &ilitek_limit_area_ops);
#endif
#endif
	proc_oppo_tp_direction_dir = proc_create("oppo_tp_direction", 0666, proc_dir_oppo, &ilitek_direction_fops);
#endif
	for (; i < ARRAY_SIZE(proc_table); i++) {
		proc_table[i].node = proc_create(proc_table[i].name, 0666, proc_dir_ilitek, proc_table[i].fops);

		if (proc_table[i].node == NULL) {
			proc_table[i].isCreated = false;
			ipio_err("Failed to create %s under /proc\n", proc_table[i].name);
			res = -ENODEV;
		} else {
			proc_table[i].isCreated = true;
			ipio_info("Succeed to create %s under /proc\n", proc_table[i].name);
		}
	}

	netlink_init();

	return res;
}
EXPORT_SYMBOL(ilitek_proc_init);

void ilitek_proc_remove(void)
{
	int i = 0;

	for (; i < ARRAY_SIZE(proc_table); i++) {
		if (proc_table[i].isCreated == true) {
			ipio_info("Removed %s under /proc\n", proc_table[i].name);
			remove_proc_entry(proc_table[i].name, proc_dir_ilitek);
		}
	}

	remove_proc_entry("ilitek", NULL);
	netlink_kernel_release(_gNetLinkSkb);
}
EXPORT_SYMBOL(ilitek_proc_remove);
