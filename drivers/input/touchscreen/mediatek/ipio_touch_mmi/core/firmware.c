/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - firmware.c
** Description : This program is for ili9881 driver firmware.c
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
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/fd.h>
#include <linux/file.h>
#include <linux/version.h>
#include <asm/uaccess.h>
#include <linux/firmware.h>

#include "../common.h"
#include "../platform.h"
#include "config.h"
#include "i2c.h"
#include "firmware.h"
#include "flash.h"
#include "protocol.h"
#include "finger_report.h"
#include "gesture.h"
#include "mp_test.h"

#ifdef BOOT_FW_UPGRADE
//#include "ilitek_fw.h"
#endif
#ifdef HOST_DOWNLOAD
//#include "ilitek_fw.h"
#endif

extern uint32_t SUP_CHIP_LIST[];
extern int nums_chip;
extern struct core_fr_data *core_fr;
extern bool SysHaveBoot;
extern unsigned char* CTPM_FW;
#if (ILITEK_TYPE == ILITEK_TYPE_LH)
extern int is_tp_fw_done;
#endif
/*
 * the size of two arrays is different depending on
 * which of methods to upgrade firmware you choose for.
 */
uint8_t *flash_fw = NULL;

#ifdef HOST_DOWNLOAD
uint8_t ap_fw[MAX_AP_FIRMWARE_SIZE] = { 0 };
uint8_t dlm_fw[MAX_DLM_FIRMWARE_SIZE] = { 0 };
uint8_t mp_fw[MAX_MP_FIRMWARE_SIZE] = { 0 };
uint8_t gesture_fw[MAX_GESTURE_FIRMWARE_SIZE] = { 0 };
#else

uint8_t iram_fw[MAX_IRAM_FIRMWARE_SIZE] = { 0 };
#endif
/* the length of array in each sector */
int g_section_len = 0;
int g_total_sector = 0;

#ifdef BOOT_FW_UPGRADE
/* The addr of block reserved for customers */
int g_start_resrv = 0x1C000;
int g_end_resrv = 0x1CFFF;
#endif

struct flash_sector {
	uint32_t ss_addr;
	uint32_t se_addr;
	uint32_t checksum;
	uint32_t crc32;
	uint32_t dlength;
	bool data_flag;
	bool inside_block;
};

struct flash_block_info {
	uint32_t start_addr;
	uint32_t end_addr;
};

struct flash_sector *g_flash_sector = NULL;
struct flash_block_info g_flash_block_info[4];
struct core_firmware_data *core_firmware = NULL;
const struct firmware *ili_fw_entry = NULL;

static uint32_t HexToDec(char *pHex, int32_t nLength)
{
	uint32_t nRetVal = 0, nTemp = 0, i;
	int32_t nShift = (nLength - 1) * 4;

	for (i = 0; i < nLength; nShift -= 4, i++) {
		if ((pHex[i] >= '0') && (pHex[i] <= '9')) {
			nTemp = pHex[i] - '0';
		} else if ((pHex[i] >= 'a') && (pHex[i] <= 'f')) {
			nTemp = (pHex[i] - 'a') + 10;
		} else if ((pHex[i] >= 'A') && (pHex[i] <= 'F')) {
			nTemp = (pHex[i] - 'A') + 10;
		} else {
			return -1;
		}

		nRetVal |= (nTemp << nShift);
	}

	return nRetVal;
}

static uint32_t calc_crc32(uint32_t start_addr, uint32_t end_addr, uint8_t *data)
{
	int i, j;
	uint32_t CRC_POLY = 0x04C11DB7;
	uint32_t ReturnCRC = 0xFFFFFFFF;
	uint32_t len = start_addr + end_addr;

	for (i = start_addr; i < len; i++) {
		ReturnCRC ^= (data[i] << 24);

		for (j = 0; j < 8; j++) {
			if ((ReturnCRC & 0x80000000) != 0) {
				ReturnCRC = ReturnCRC << 1 ^ CRC_POLY;
			} else {
				ReturnCRC = ReturnCRC << 1;
			}
		}
	}

	return ReturnCRC;
}

static uint32_t tddi_check_data(uint32_t start_addr, uint32_t end_addr)
{
	int timer = 500;
	uint32_t busy = 0;
	uint32_t write_len = 0;
	uint32_t iram_check = 0;
	uint32_t id = core_config->chip_id;
	uint32_t type = core_config->chip_type;

	write_len = end_addr;

	ipio_debug(DEBUG_FIRMWARE, "start = 0x%x , write_len = 0x%x, max_count = %x\n",
	    start_addr, end_addr, core_firmware->max_count);

	if (write_len > core_firmware->max_count) {
		ipio_err("The length (%x) written to firmware is greater than max count (%x)\n",
			write_len, core_firmware->max_count);
		goto out;
	}

	core_config_ice_mode_write(0x041000, 0x0, 1);	/* CS low */
	core_config_ice_mode_write(0x041004, 0x66aa55, 3);	/* Key */

	core_config_ice_mode_write(0x041008, 0x3b, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0xFF0000) >> 16, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x00FF00) >> 8, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x0000FF), 1);

	core_config_ice_mode_write(0x041003, 0x01, 1);	/* Enable Dio_Rx_dual */
	core_config_ice_mode_write(0x041008, 0xFF, 1);	/* Dummy */

	/* Set Receive count */
	if (core_firmware->max_count == 0xFFFF)
		core_config_ice_mode_write(0x04100C, write_len, 2);
	else if (core_firmware->max_count == 0x1FFFF)
		core_config_ice_mode_write(0x04100C, write_len, 3);

	if (id == CHIP_TYPE_ILI9881 && type == ILI9881_TYPE_F) {
		/* Checksum_En */
		core_config_ice_mode_write(0x041014, 0x10000, 3);
	} else if (id == CHIP_TYPE_ILI9881 && type == ILI9881_TYPE_H) {
		/* Clear Int Flag */
		core_config_ice_mode_write(0x048007, 0x02, 1);

		/* Checksum_En */
		core_config_ice_mode_write(0x041016, 0x00, 1);
		core_config_ice_mode_write(0x041016, 0x01, 1);
	}

	/* Start to receive */
	core_config_ice_mode_write(0x041010, 0xFF, 1);

	while (timer > 0) {

		mdelay(1);

		if (id == CHIP_TYPE_ILI9881 && type == ILI9881_TYPE_F)
			busy = core_config_read_write_onebyte(0x041014);
		else if (id == CHIP_TYPE_ILI9881 && type == ILI9881_TYPE_H) {
			busy = core_config_read_write_onebyte(0x048007);
			busy = busy >> 1;
		} else {
			ipio_err("Unknow chip type\n");
			break;
		}

		if ((busy & 0x01) == 0x01)
			break;

		timer--;
	}

	core_config_ice_mode_write(0x041000, 0x1, 1);	/* CS high */

	if (timer >= 0) {
		/* Disable dio_Rx_dual */
		core_config_ice_mode_write(0x041003, 0x0, 1);
		iram_check =  core_firmware->isCRC ? core_config_ice_mode_read(0x4101C) : core_config_ice_mode_read(0x041018);
	} else {
		ipio_err("TIME OUT\n");
		goto out;
	}

	return iram_check;

out:
	ipio_err("Failed to read Checksum/CRC from IC\n");
	return -1;

}

static void calc_verify_data(uint32_t sa, uint32_t se, uint32_t *check)
{
	uint32_t i = 0;
	uint32_t tmp_ck = 0, tmp_crc = 0;

	if (core_firmware->isCRC) {
		tmp_crc = calc_crc32(sa, se, flash_fw);
		*check = tmp_crc;
	} else {
		for (i = sa; i < (sa + se); i++)
			tmp_ck = tmp_ck + flash_fw[i];

		*check = tmp_ck;
	}
}

static int do_check(uint32_t start, uint32_t len)
{
	int res = 0;
	uint32_t vd = 0, lc = 0;

	calc_verify_data(start, len, &lc);
	vd = tddi_check_data(start, len);
	res = CHECK_EQUAL(vd, lc);

	ipio_debug(DEBUG_FIRMWARE, "%s (%x) : (%x)\n", (res < 0 ? "Invalid !" : "Correct !"), vd, lc);

	return res;
}

static int verify_flash_data(void)
{
	int i = 0, res = 0, len = 0;
	int fps = flashtab->sector;
	uint32_t ss = 0x0;

	/* check chip type with its max count */
	if (core_config->chip_id == CHIP_TYPE_ILI7807 && core_config->chip_type == ILI7807_TYPE_H) {
		core_firmware->max_count = 0x1FFFF;
		core_firmware->isCRC = true;
	}

	for (i = 0; i < g_section_len + 1; i++) {
		if (g_flash_sector[i].data_flag) {
			if (ss > g_flash_sector[i].ss_addr || len == 0)
				ss = g_flash_sector[i].ss_addr;

			len = len + g_flash_sector[i].dlength;

			/* if larger than max count, then committing data to check */
			if (len >= (core_firmware->max_count - fps)) {
				res = do_check(ss, len);
				if (res < 0)
					goto out;

				ss = g_flash_sector[i].ss_addr;
				len = 0;
			}
		} else {
			/* split flash sector and commit the last data to fw */
			if (len != 0) {
				res = do_check(ss, len);
				if (res < 0)
					goto out;

				ss = g_flash_sector[i].ss_addr;
				len = 0;
			}
		}
	}

	/* it might be lower than the size of sector if calc the last array. */
	if (len != 0 && res != -1)
		res = do_check(ss, core_firmware->end_addr - ss);

out:
	return res;
}

static int do_program_flash(uint32_t start_addr)
{
	int res = 0;
	uint32_t k;
	uint8_t buf[512] = { 0 };

	res = core_flash_write_enable();
	if (res < 0)
		goto out;

	core_config_ice_mode_write(0x041000, 0x0, 1);	/* CS low */
	core_config_ice_mode_write(0x041004, 0x66aa55, 3);	/* Key */

	core_config_ice_mode_write(0x041008, 0x02, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0xFF0000) >> 16, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x00FF00) >> 8, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x0000FF), 1);

	buf[0] = 0x25;
	buf[3] = 0x04;
	buf[2] = 0x10;
	buf[1] = 0x08;

	for (k = 0; k < flashtab->program_page; k++) {
		if (start_addr + k <= core_firmware->end_addr)
			buf[4 + k] = flash_fw[start_addr + k];
		else
			buf[4 + k] = 0xFF;
	}

	if (core_write(core_config->slave_i2c_addr, buf, flashtab->program_page + 4) < 0) {
		ipio_err("Failed to write data at start_addr = 0x%X, k = 0x%X, addr = 0x%x\n",
			start_addr, k, start_addr + k);
		res = -EIO;
		goto out;
	}

	core_config_ice_mode_write(0x041000, 0x1, 1);	/* CS high */

	res = core_flash_poll_busy();
	if (res < 0)
		goto out;

	core_firmware->update_status = (start_addr * 101) / core_firmware->end_addr;

	/* holding the status until finish this upgrade. */
	if (core_firmware->update_status > 90)
		core_firmware->update_status = 90;

	/* Don't use ipio_info to print log because it needs to be kpet in the same line */
	printk("%cUpgrading firmware ... start_addr = 0x%x, %02d%c", 0x0D, start_addr, core_firmware->update_status,
	       '%');

out:
	return res;
}

static int flash_program_sector(void)
{
	int i, j, res = 0;

	for (i = 0; i < g_section_len + 1; i++) {
		/*
		 * If running the boot stage, fw will only be upgrade data with the flag of block,
		 * otherwise data with the flag itself will be programed.
		 */
		if (core_firmware->isboot) {
			if (!g_flash_sector[i].inside_block)
				continue;
		} else {
			if (!g_flash_sector[i].data_flag)
				continue;
		}

		/* programming flash by its page size */
		for (j = g_flash_sector[i].ss_addr; j < g_flash_sector[i].se_addr; j += flashtab->program_page) {
			if (j > core_firmware->end_addr)
				goto out;

			res = do_program_flash(j);
			if (res < 0)
				goto out;
		}
	}

out:
	return res;
}

static int do_erase_flash(uint32_t start_addr)
{
	int res = 0;
	uint32_t temp_buf = 0;

	res = core_flash_write_enable();
	if (res < 0) {
		ipio_err("Failed to config write enable\n");
		goto out;
	}

	core_config_ice_mode_write(0x041000, 0x0, 1);	/* CS low */
	core_config_ice_mode_write(0x041004, 0x66aa55, 3);	/* Key */

	core_config_ice_mode_write(0x041008, 0x20, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0xFF0000) >> 16, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x00FF00) >> 8, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x0000FF), 1);

	core_config_ice_mode_write(0x041000, 0x1, 1);	/* CS high */

	mdelay(1);

	res = core_flash_poll_busy();
	if (res < 0)
		goto out;

	core_config_ice_mode_write(0x041000, 0x0, 1);	/* CS low */
	core_config_ice_mode_write(0x041004, 0x66aa55, 3);	/* Key */

	core_config_ice_mode_write(0x041008, 0x3, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0xFF0000) >> 16, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x00FF00) >> 8, 1);
	core_config_ice_mode_write(0x041008, (start_addr & 0x0000FF), 1);
	core_config_ice_mode_write(0x041008, 0xFF, 1);

	temp_buf = core_config_read_write_onebyte(0x041010);
	if (temp_buf != 0xFF) {
		ipio_err("Failed to erase data(0x%x) at 0x%x\n", temp_buf, start_addr);
		res = -EINVAL;
		goto out;
	}

	core_config_ice_mode_write(0x041000, 0x1, 1);	/* CS high */

	ipio_debug(DEBUG_FIRMWARE, "Earsing data at start addr: %x\n", start_addr);

out:
	return res;
}

static int flash_erase_sector(void)
{
	int i, res = 0;

	for (i = 0; i < g_total_sector; i++) {
		if (core_firmware->isboot) {
			if (!g_flash_sector[i].inside_block)
				continue;
		} else {
			if (!g_flash_sector[i].data_flag && !g_flash_sector[i].inside_block)
				continue;
		}

		res = do_erase_flash(g_flash_sector[i].ss_addr);
		if (res < 0)
			goto out;
	}

out:
	return res;
}

#ifndef HOST_DOWNLOAD
static int iram_upgrade(void)
{
	int i, j, res = 0;
	uint8_t buf[512];
	int upl = flashtab->program_page;

	/* doing reset for erasing iram data before upgrade it. */
	ilitek_platform_tp_hw_reset(true);

	mdelay(1);

	ipio_info("Upgrade firmware written data into IRAM directly\n");

	res = core_config_ice_mode_enable();
	if (res < 0) {
		ipio_err("Failed to enter ICE mode, res = %d\n", res);
		return res;
	}

	mdelay(20);

	core_config_set_watch_dog(false);

	ipio_debug(DEBUG_FIRMWARE, "nStartAddr = 0x%06X, nEndAddr = 0x%06X, nChecksum = 0x%06X\n",
	    core_firmware->start_addr, core_firmware->end_addr, core_firmware->checksum);

	/* write hex to the addr of iram */
	ipio_info("Writing data into IRAM ...\n");
	for (i = core_firmware->start_addr; i < core_firmware->end_addr; i += upl) {
		if ((i + 256) > core_firmware->end_addr) {
			upl = core_firmware->end_addr % upl;
		}

		buf[0] = 0x25;
		buf[3] = (char)((i & 0x00FF0000) >> 16);
		buf[2] = (char)((i & 0x0000FF00) >> 8);
		buf[1] = (char)((i & 0x000000FF));

		for (j = 0; j < upl; j++)
			buf[4 + j] = iram_fw[i + j];

		if (core_write(core_config->slave_i2c_addr, buf, upl + 4)) {
			ipio_err("Failed to write data via i2c, address = 0x%X, start_addr = 0x%X, end_addr = 0x%X\n",
				(int)i, (int)core_firmware->start_addr, (int)core_firmware->end_addr);
			res = -EIO;
			return res;
		}

		core_firmware->update_status = (i * 101) / core_firmware->end_addr;
		printk("%cupgrade firmware(ap code), %02d%c", 0x0D, core_firmware->update_status, '%');

		mdelay(3);
	}

	/* ice mode code reset */
	ipio_info("Doing code reset ...\n");
	core_config_ice_mode_write(0x40040, 0xAE, 1);
	core_config_ice_mode_write(0x40040, 0x00, 1);

	mdelay(10);

	core_config_set_watch_dog(true);

	core_config_ice_mode_disable();

	/*TODO: check iram status */

	return res;
}
#endif

int read_download(uint32_t start, uint32_t size, uint8_t *r_buf, uint32_t r_len)
{
	int res = 0, addr = 0, i = 0;
	uint32_t end = start + size;
	uint8_t *buf;
    buf = (uint8_t*)kmalloc(sizeof(uint8_t) * r_len + 4, GFP_KERNEL);
	if (ERR_ALLOC_MEM(buf)) {
		ipio_err("malloc read_ap_buf error\n");
		return -1;
	}
	memset(buf, 0xFF, (int)sizeof(uint8_t) * r_len + 4);
	for (addr = start, i = 0; addr < end; i += r_len, addr += r_len) {
		if ((addr + r_len) > end) {
			r_len = end % r_len;
		}
		buf[0] = 0x25;
		buf[3] = (char)((addr & 0x00FF0000) >> 16);
		buf[2] = (char)((addr & 0x0000FF00) >> 8);
		buf[1] = (char)((addr & 0x000000FF));
		if (core_write(core_config->slave_i2c_addr, buf, 4)) {
			ipio_err("Failed to Read data via SPI\n");
			res = -EIO;
			goto read_error;
		}
		res = core_read(core_config->slave_i2c_addr, buf, r_len);
		memcpy(r_buf + i, buf, r_len);
	}
read_error:
	kfree(buf);
	buf = NULL;
	return res;
}

int write_download(uint32_t start, uint32_t size, uint8_t *w_buf, uint32_t w_len)
{
	int res = 0, addr = 0, i = 0, update_status = 0, end = 0, j = 0;
	uint8_t *buf;
	end = start + size;
    buf = (uint8_t*)kmalloc(sizeof(uint8_t) * w_len + 4, GFP_KERNEL);
	if (ERR_ALLOC_MEM(buf)) {
		ipio_err("malloc read_ap_buf error\n");
		return -1;
	}
	memset(buf, 0xFF, (int)sizeof(uint8_t) * w_len + 4);
	for (addr = start, i = 0; addr < end; addr += w_len, i += w_len) {
		if ((addr + w_len) > end) {
			w_len = end % w_len;
		}
		buf[0] = 0x25;
		buf[3] = (char)((addr & 0x00FF0000) >> 16);
		buf[2] = (char)((addr & 0x0000FF00) >> 8);
		buf[1] = (char)((addr & 0x000000FF));
		for (j = 0; j < w_len; j++)
			buf[4 + j] = w_buf[i + j];

		if (core_write(core_config->slave_i2c_addr, buf, w_len + 4)) {
			ipio_err("Failed to write data via SPI, address = 0x%X, start_addr = 0x%X, end_addr = 0x%X\n",
				(int)addr, 0, end);
			res = -EIO;
			goto write_error;
		}
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/10/30,Add for TP black screen test
		update_status = (i * 101) / size;
#endif
		ipio_debug(DEBUG_FIRMWARE, "%cupgrade firmware(mp code), %02d%c", 0x0D, update_status, '%');
	}
write_error:
	kfree(buf);
	buf = NULL;
	return res;
}

#ifdef HOST_DOWNLOAD

static int host_download_dma_check(int block)
{
	int count = 50;
	uint8_t ap_block = 0, dlm_block = 1;
	uint32_t start_addr = 0, block_size = 0;
	uint32_t busy = 0;

	if (block == ap_block) {
		start_addr = 0;
		block_size = MAX_AP_FIRMWARE_SIZE - 0x4;
	} else if (block == dlm_block) {
		start_addr = DLM_START_ADDRESS;
		block_size = MAX_DLM_FIRMWARE_SIZE;
	}

	/* dma_ch1_start_clear */
	core_config_ice_mode_write(0x072103, 0x2, 1);

	/* dma1 src1 adress */
	core_config_ice_mode_write(0x072104, start_addr, 4);
	/* dma1 src1 format */
	core_config_ice_mode_write(0x072108, 0x80000001, 4);
	/* dma1 dest address */
	core_config_ice_mode_write(0x072114, 0x00030000, 4);
	/* dma1 dest format */
	core_config_ice_mode_write(0x072118, 0x80000000, 4);
	/* Block size*/
	core_config_ice_mode_write(0x07211C, block_size, 4);
	/* crc off */
	core_config_ice_mode_write(0x041014, 0x00000000, 4);
	/* dma crc */
	core_config_ice_mode_write(0x041048, 0x00000001, 4);
	/* crc on */
	core_config_ice_mode_write(0x041014, 0x00010000, 4);
	/* Dma1 stop */
	core_config_ice_mode_write(0x072100, 0x00000000, 4);
	/* clr int */
	core_config_ice_mode_write(0x048006, 0x1, 1);
	/* Dma1 start */
	core_config_ice_mode_write(0x072100, 0x01000000, 4);

	/* Polling BIT0 */
	while (count > 0) {
		mdelay(1);
		busy = core_config_read_write_onebyte(0x048006);

		if ((busy & 0x01) == 1)
			break;

		count--;
	}

	if (count <= 0) {
		ipio_err("BIT0 is busy\n");
		//return -1;
	}

	return core_config_ice_mode_read(0x04101C);
}

int host_download(bool mode)
{
#if 0
	int res = 0;
	uint8_t *buf, *read_ap_buf, *read_dlm_buf, *read_mp_buf, *read_gesture_buf, *gesture_ap_buf;
	ipio_debug(DEBUG_FIRMWARE, "\n");
	res = core_config_ice_mode_enable();
	if (res < 0) {
		ipio_err("Failed to enter ICE mode, res = %d\n", res);
		return res;
	}
    read_ap_buf = (uint8_t*)vmalloc(MAX_AP_FIRMWARE_SIZE);
	if (ERR_ALLOC_MEM(read_ap_buf)) {
		ipio_err("malloc read_ap_buf error\n");
		return -1;
	}
	ipio_debug(DEBUG_FIRMWARE, "\n");
	memset(read_ap_buf, 0xFF, MAX_AP_FIRMWARE_SIZE);
	//create ap buf
    read_dlm_buf = (uint8_t*)vmalloc(MAX_DLM_FIRMWARE_SIZE);
	if (ERR_ALLOC_MEM(read_dlm_buf)) {
		ipio_err("malloc read_dlm_buf error\n");
		return -1;
	}
	ipio_debug(DEBUG_FIRMWARE, "\n");
	memset(read_dlm_buf, 0xFF, MAX_DLM_FIRMWARE_SIZE);
	//create mp buf
    read_mp_buf = (uint8_t*)vmalloc(MAX_MP_FIRMWARE_SIZE);
	if (ERR_ALLOC_MEM(read_mp_buf)) {
		ipio_err("malloc read_mp_buf error\n");
		return -1;
	}
	ipio_debug(DEBUG_FIRMWARE, "\n");
	memset(read_mp_buf, 0xFF, MAX_MP_FIRMWARE_SIZE);
	//create buf
    buf = (uint8_t*)vmalloc(sizeof(uint8_t)*0x10000+4);
	if (ERR_ALLOC_MEM(buf)) {
		ipio_err("malloc buf error\n");
		return -1;
	}
	ipio_debug(DEBUG_FIRMWARE, "\n");
	memset(buf, 0xFF, (int)sizeof(uint8_t) * 0x10000+4);
	//create gesture buf
    read_gesture_buf = (uint8_t*)vmalloc(core_gesture->ap_length);
	if (ERR_ALLOC_MEM(read_gesture_buf)) {
		ipio_err("malloc read_gesture_buf error\n");
		return -1;
	}
	ipio_debug(DEBUG_FIRMWARE, "\n");
    gesture_ap_buf = (uint8_t*)vmalloc(core_gesture->ap_length);
	if (ERR_ALLOC_MEM(gesture_ap_buf)) {
		ipio_err("malloc gesture_ap_buf error\n");
		return -1;
	}
	memset(gesture_ap_buf, 0xFF, core_gesture->ap_length);

	ipio_info("core_gesture->entry = %d\n", core_gesture->entry);
	memset(read_gesture_buf, 0xFF, core_gesture->ap_length);
	ipio_info("Upgrade firmware written data into AP code directly\n");
	if (core_config_set_watch_dog(false) < 0) {
		ipio_err("Failed to disable watch dog\n");
		res = -EINVAL;
		goto upgrade_fail;
	}

	if(core_fr->actual_fw_mode == P5_0_FIRMWARE_TEST_MODE)
	{
		/* write hex to the addr of MP code */
		ipio_info("Writing data into MP code ...\n");
		if(write_download(0, MAX_MP_FIRMWARE_SIZE, mp_fw, SPI_UPGRADE_LEN) < 0)
		{
			ipio_err("SPI Write MP code data error\n");
		}
		if(read_download(0, MAX_MP_FIRMWARE_SIZE, read_mp_buf, SPI_UPGRADE_LEN))
		{
			ipio_err("SPI Read MP code data error\n");
		}
		if(memcmp(mp_fw, read_mp_buf, MAX_MP_FIRMWARE_SIZE) == 0)
		{
			ipio_info("Check MP Mode upgrade: PASS\n");
		}
		else
		{
			ipio_info("Check MP Mode upgrade: FAIL\n");
			res = UPDATE_FAIL;
			goto upgrade_fail;
		}
	}
	else if(core_gesture->entry)
	{
		//int i;
		if(mode)
		{
			/* write hex to the addr of Gesture code */
			ipio_info("Writing data into Gesture code ...\n");
			if(write_download(core_gesture->ap_start_addr, core_gesture->length, gesture_fw, core_gesture->length) < 0)
			{
				ipio_err("SPI Write Gesture code data error\n");
			}
			if(read_download(core_gesture->ap_start_addr, core_gesture->length, read_gesture_buf, core_gesture->length))
			{
				ipio_err("SPI Read Gesture code data error\n");
			}
			if(memcmp(gesture_fw, read_gesture_buf, core_gesture->length) == 0)
			{
				ipio_info("Check Gesture Mode upgrade: PASS\n");
			}
			else
			{
				ipio_info("Check Gesture Mode upgrade: FAIL\n");
				res = UPDATE_FAIL;
				goto upgrade_fail;
			}
			// for (i = core_gesture->start_addr; i < core_gesture->start_addr + core_gesture->length; i++) {
			// 	if(i%16 == 0)
			// 		printk("0x%4x:", i);
			// 	printk("0x%2x,", gesture_fw[i - core_gesture->start_addr]);
			// 	if( i%16 == 15)
			// 		printk("\n");
			// }
		}
		else{
			/* write hex to the addr of AP code */
			memcpy(gesture_ap_buf, ap_fw + core_gesture->ap_start_addr, core_gesture->ap_length);
			ipio_info("Writing data into AP code ...\n");
			if(write_download(core_gesture->ap_start_addr, core_gesture->ap_length, gesture_ap_buf, core_gesture->ap_length) < 0)
			{
				ipio_err("SPI Write AP code data error\n");
			}
			if(read_download(core_gesture->ap_start_addr, core_gesture->ap_length, read_ap_buf, core_gesture->ap_length))
			{
				ipio_err("SPI Read AP code data error\n");
			}
			// for (i = core_gesture->ap_start_addr; i < core_gesture->ap_start_addr + core_gesture->ap_length; i++) {
			// 	if(i%16 == 0)
			// 		printk("0x%4x:", i);
			// 	printk("0x%2x,", gesture_ap_buf[i - core_gesture->ap_start_addr]);
			// 	if( i%16 == 15)
			// 		printk("\n");
			// }
			if(memcmp(gesture_ap_buf, read_ap_buf, core_gesture->ap_length) == 0)
			{
				ipio_info("Check AP Mode upgrade: PASS\n");
			}
			else
			{
				ipio_info("Check AP Mode upgrade: FAIL\n");
				res = UPDATE_FAIL;
				goto upgrade_fail;
			}
		}
	}
	else
	{
		/* write hex to the addr of AP code */
		ipio_info("Writing data into AP code ...\n");
		if(write_download(0, MAX_AP_FIRMWARE_SIZE, ap_fw, SPI_UPGRADE_LEN) < 0)
		{
			ipio_err("SPI Write AP code data error\n");
		}
		/* write hex to the addr of DLM code */
		ipio_info("Writing data into DLM code ...\n");
		if(write_download(DLM_START_ADDRESS, MAX_DLM_FIRMWARE_SIZE, dlm_fw, SPI_UPGRADE_LEN) < 0)
		{
			ipio_err("SPI Write DLM code data error\n");
		}
		ipio_info("Writing data into DLM code ......\n");
		/* Check AP mode Buffer data */
		if(read_download(0, MAX_AP_FIRMWARE_SIZE, read_ap_buf, SPI_UPGRADE_LEN))
		{
			ipio_err("SPI Read MP code data error\n");
		}
		if(memcmp(ap_fw, read_ap_buf, MAX_AP_FIRMWARE_SIZE) == 0)
		{
			ipio_info("Check AP Mode upgrade: PASS\n");
		}
		else
		{
			ipio_info("Check AP Mode upgrade: FAIL\n");
			res = UPDATE_FAIL;
			goto upgrade_fail;
		}
		/* Check DLM mode Buffer data */
		if(read_download(DLM_START_ADDRESS, MAX_DLM_FIRMWARE_SIZE, read_dlm_buf, SPI_UPGRADE_LEN))
		{
			ipio_err("SPI Read DLM code data error\n");
		}
		if(memcmp(dlm_fw, read_dlm_buf, MAX_DLM_FIRMWARE_SIZE) == 0)
		{
			ipio_info("Check DLM Mode upgrade: PASS\n");
		}
		else
		{
			ipio_info("Check DLM Mode upgrade: FAIL\n");
			res = UPDATE_FAIL;
			goto upgrade_fail;
		}
	}

upgrade_fail:
	if(core_gesture->entry != true)
	{
		/* ice mode code reset */
		ipio_info("Doing code reset ...\n");
		core_config_ice_mode_write(0x40040, 0xAE, 1);
	}

	if (core_config_set_watch_dog(true) < 0) {
		ipio_err("Failed to enable watch dog\n");
		res = -EINVAL;
	}

	core_config_ice_mode_disable();
	if(core_fr->actual_fw_mode == P5_0_FIRMWARE_TEST_MODE)
		mdelay(1200);


	vfree(buf);
	buf = NULL;
	vfree(read_ap_buf);
	read_ap_buf = NULL;
	vfree(read_dlm_buf);
	read_dlm_buf = NULL;
	vfree(read_mp_buf);
	read_mp_buf = NULL;
	vfree(read_gesture_buf);
	read_gesture_buf = NULL;
	vfree(gesture_ap_buf);
	gesture_ap_buf = NULL;
	return res;
#else
	int res = 0, ap_crc, ap_dma, dlm_crc, dlm_dma, method;
	uint8_t *buf = NULL, *read_ap_buf = NULL, *read_dlm_buf = NULL, *read_mp_buf = NULL;
	uint8_t *read_gesture_buf = NULL, *gesture_ap_buf = NULL;
	int retry = 10;
	uint32_t reg_data = 0;
	//uint32_t i;
	ipio_info("host_download begin\n");
	core_config->crc_check = true;
	res = core_config_ice_mode_enable();
	if (res < 0) {
		ipio_err("Failed to enter ICE mode, res = %d\n", res);
		return res;
	}

	method = core_config_ice_mode_read(core_config->pid_addr);
	method = method & 0xff;
	ipio_info("method of calculation for crc = %x\n", method);

	read_ap_buf = (uint8_t*)vmalloc(MAX_AP_FIRMWARE_SIZE);
	if (ERR_ALLOC_MEM(read_ap_buf)) {
		ipio_err("malloc read_ap_buf error\n");
		goto out;
	}
	ipio_debug(DEBUG_FIRMWARE, "\n");
	memset(read_ap_buf, 0xFF, MAX_AP_FIRMWARE_SIZE);
	//create ap buf
	read_dlm_buf = (uint8_t*)vmalloc(MAX_DLM_FIRMWARE_SIZE);
	if (ERR_ALLOC_MEM(read_dlm_buf)) {
		ipio_err("malloc read_dlm_buf error\n");
		goto out;
	}
	ipio_debug(DEBUG_FIRMWARE, "\n");
	memset(read_dlm_buf, 0xFF, MAX_DLM_FIRMWARE_SIZE);
	//create mp buf
	read_mp_buf = (uint8_t*)vmalloc(MAX_MP_FIRMWARE_SIZE);
	if (ERR_ALLOC_MEM(read_mp_buf)) {
		ipio_err("malloc read_mp_buf error\n");
		goto out;
	}
	ipio_debug(DEBUG_FIRMWARE, "\n");
	memset(read_mp_buf, 0xFF, MAX_MP_FIRMWARE_SIZE);
	//create buf
	buf = (uint8_t*)vmalloc(sizeof(uint8_t)*0x10000+4);
	if (ERR_ALLOC_MEM(buf)) {
		ipio_err("malloc buf error\n");
		goto out;
	}
	ipio_debug(DEBUG_FIRMWARE, "\n");
	memset(buf, 0xFF, (int)sizeof(uint8_t) * 0x10000+4);
	//create gesture buf
	read_gesture_buf = (uint8_t*)vmalloc(core_gesture->ap_length);
	if (ERR_ALLOC_MEM(read_gesture_buf)) {
		ipio_err("malloc read_gesture_buf error\n");
		goto out;
	}
	ipio_debug(DEBUG_FIRMWARE, "\n");
	gesture_ap_buf = (uint8_t*)vmalloc(core_gesture->ap_length);
	if (ERR_ALLOC_MEM(gesture_ap_buf)) {
		ipio_err("malloc gesture_ap_buf error\n");
		goto out;
	}
	memset(gesture_ap_buf, 0xFF, core_gesture->ap_length);

	ipio_info("core_gesture->entry = %d\n", core_gesture->entry);
	memset(read_gesture_buf, 0xFF, core_gesture->ap_length);

	core_config_ice_mode_write(0x5100C, 0x81, 1);
	core_config_ice_mode_write(0x5100C, 0x98, 1);
	/* need to delay 300us after stop mcu to wait fw relaod */
	udelay(300);
	while(retry--) {
		reg_data = core_config_read_write_onebyte(0x51018);
		if (reg_data == 0x5A) {
			ipio_info("check wdt close ok 0x51018 read 0x%X\n", reg_data);
			break;
		}
		mdelay(10);
	}
	if (retry <= 0) {
		ipio_info("check wdt close error 0x51018 read 0x%X\n", reg_data);
		res = UPDATE_FAIL;
		goto out;
	}
	core_config_ice_mode_write(0x5100C, 0x00, 1);

	if (core_fr->actual_fw_mode == protocol->test_mode) {
		/* write hex to the addr of MP code */
		ipio_info("Writing data into MP code ...\n");

		if (write_download(0, MAX_MP_FIRMWARE_SIZE, mp_fw, SPI_UPGRADE_LEN) < 0) {
			ipio_err("SPI Write MP code data error\n");
		}

		if (read_download(0, MAX_MP_FIRMWARE_SIZE, read_mp_buf, SPI_UPGRADE_LEN)) {
			ipio_err("SPI Read MP code data error\n");
		}

		if (memcmp(mp_fw, read_mp_buf, MAX_MP_FIRMWARE_SIZE) == 0) {
			ipio_info("Check MP Mode upgrade: PASS\n");
		} else {
			ipio_info("Check MP Mode upgrade: FAIL\n");
			res = UPDATE_FAIL;
			goto out;
		}
	} else if (core_gesture->entry) {
		if (mode) {
			/* write hex to the addr of Gesture code */
			ipio_info("Writing data into Gesture code ...\n");
			if (write_download(core_gesture->ap_start_addr, core_gesture->length, gesture_fw, core_gesture->length) < 0) {
				ipio_err("SPI Write Gesture code data error\n");
			}

			if (read_download(core_gesture->ap_start_addr, core_gesture->length, read_gesture_buf, core_gesture->length)) {
				ipio_err("SPI Read Gesture code data error\n");
			}

			if (memcmp(gesture_fw, read_gesture_buf, core_gesture->length) == 0) {
				ipio_info("Check Gesture Mode upgrade: PASS\n");
			} else {
				ipio_info("Check Gesture Mode upgrade: FAIL\n");
				res = UPDATE_FAIL;
				goto out;
			}
		} else {
			/* write hex to the addr of AP code */
			memcpy(gesture_ap_buf, ap_fw + core_gesture->ap_start_addr, core_gesture->ap_length);
			ipio_info("Writing data into AP code ...\n");
			if (write_download(core_gesture->ap_start_addr, core_gesture->ap_length, gesture_ap_buf, core_gesture->ap_length) < 0) {
				ipio_err("SPI Write AP code data error\n");
			}
			if (read_download(core_gesture->ap_start_addr, core_gesture->ap_length, read_ap_buf, core_gesture->ap_length)) {
				ipio_err("SPI Read AP code data error\n");
			}

			if (memcmp(gesture_ap_buf, read_ap_buf, core_gesture->ap_length) == 0) {
				ipio_info("Check AP Mode upgrade: PASS\n");
			} else {
				ipio_info("Check AP Mode upgrade: FAIL\n");
				res = UPDATE_FAIL;
				goto out;
			}
		}
	} else {
		/* write hex to the addr of AP code */
		ipio_info("Writing data into AP code ...\n");
		if (write_download(0, MAX_AP_FIRMWARE_SIZE, ap_fw, SPI_UPGRADE_LEN) < 0) {
			ipio_err("SPI Write AP code data error\n");
		}

		/* write hex to the addr of DLM code */
		ipio_info("Writing data into DLM code ...\n");
		if (write_download(DLM_START_ADDRESS, MAX_DLM_FIRMWARE_SIZE, dlm_fw, SPI_UPGRADE_LEN) < 0) {
			ipio_err("SPI Write DLM code data error\n");
		}

		/* Check AP/DLM mode Buffer data */
		if (method >= CORE_TYPE_E) {
			ap_crc = calc_crc32(0, MAX_AP_FIRMWARE_SIZE - 4, ap_fw);
			ap_dma = host_download_dma_check(0);

			dlm_crc = calc_crc32(0, MAX_DLM_FIRMWARE_SIZE, dlm_fw);
			dlm_dma = host_download_dma_check(1);

			ipio_info("AP CRC %s (%x) : (%x)\n",
				(ap_crc != ap_dma ? "Invalid !" : "Correct !"), ap_crc, ap_dma);

			ipio_info("DLM CRC %s (%x) : (%x)\n",
				(dlm_crc != dlm_dma ? "Invalid !" : "Correct !"), dlm_crc, dlm_dma);

			if (CHECK_EQUAL(ap_crc, ap_dma) == UPDATE_FAIL ||
					CHECK_EQUAL(dlm_crc, dlm_dma) == UPDATE_FAIL ) {
					ipio_info("Check AP/DLM Mode upgrade: FAIL read data check\n");
					res = UPDATE_FAIL;
					read_download(0, MAX_AP_FIRMWARE_SIZE, read_ap_buf, SPI_UPGRADE_LEN);
					read_download(DLM_START_ADDRESS, MAX_DLM_FIRMWARE_SIZE, read_dlm_buf, SPI_UPGRADE_LEN);

					if (memcmp(ap_fw, read_ap_buf, MAX_AP_FIRMWARE_SIZE) != 0 ||
							memcmp(dlm_fw, read_dlm_buf, MAX_DLM_FIRMWARE_SIZE) != 0) {
						ipio_info("Check AP/DLM Mode upgrade: FAIL\n");
						core_config->crc_check = false;
						res = UPDATE_FAIL;
						goto out;
					} else {
						ipio_info("Check AP/DLM Mode upgrade: SUCCESS\n");
						res = 0;
					}
					//goto out;
			}
		} else {
			read_download(0, MAX_AP_FIRMWARE_SIZE, read_ap_buf, SPI_UPGRADE_LEN);
			read_download(DLM_START_ADDRESS, MAX_DLM_FIRMWARE_SIZE, read_dlm_buf, SPI_UPGRADE_LEN);

			if (memcmp(ap_fw, read_ap_buf, MAX_AP_FIRMWARE_SIZE) != 0 ||
					memcmp(dlm_fw, read_dlm_buf, MAX_DLM_FIRMWARE_SIZE) != 0) {
				ipio_info("Check AP/DLM Mode upgrade: FAIL\n");
				res = UPDATE_FAIL;
				goto out;
			} else {
				ipio_info("Check AP/DLM Mode upgrade: SUCCESS\n");
			}
		}
/*2018-8-27 begin*/
		if (core_firmware->esd_fail_enter_gesture == 1) {
			ipio_info("set 0x25FF8 = 0xF38A94EF for gesture\n");
			core_config_ice_mode_write(0x25FF8, 0xF38A94EF, 4);
		}
/*2018-8-27 end*/

	}

	core_config_ice_mode_write(0x5100C, 0x01, 1);
	while(retry--) {
		reg_data = core_config_read_write_onebyte(0x51018);
		if (reg_data == 0xA5) {
			ipio_info("check wdt open ok 0x51018 read 0x%X\n", reg_data);
			break;
		}
		mdelay(10);
	}
	if (retry <= 0) {
		ipio_info("check wdt open error 0x51018 read 0x%X retry set\n", reg_data);
		core_config_ice_mode_write(0x5100C, 0x01, 1);
	}

	if (!core_gesture->entry) {
		/* ice mode code reset */
		ipio_info("Doing code reset ...\n");
		core_config_ice_mode_write(0x40040, 0xAE, 1);
	}

	core_config_ice_mode_disable();
	//ipio_info("core_config_ice_mode_disable ...\n");
	if (core_fr->actual_fw_mode == protocol->test_mode)
		mdelay(1200);
	else
		mdelay(60);

out:

	ipio_vfree((void **)&buf);
	ipio_vfree((void **)&read_ap_buf);
	ipio_vfree((void **)&read_dlm_buf);
	ipio_vfree((void **)&read_mp_buf);
	ipio_vfree((void **)&read_gesture_buf);
	ipio_vfree((void **)&gesture_ap_buf);
	if (res == UPDATE_FAIL) {
		//core_config_ice_mode_disable();
	}

	ipio_info("host_download end\n");

	return res;

#endif
}
EXPORT_SYMBOL(host_download);
#endif

static int tddi_fw_upgrade(bool isIRAM)
{
	int res = 0;

#ifndef HOST_DOWNLOAD
	if (isIRAM) {
		res = iram_upgrade();
		return res;
	}
#endif
#ifdef ODM_WT_EDIT
//Zhonghua.Hu@ODM_WT.BSP.Tp.Init.1372106,2018/5/25,Modify for oppo factory test
	oppo_platform_tp_hw_reset(true);
#endif
	ipio_debug(DEBUG_FIRMWARE, "Enter to ICE Mode\n");

	res = core_config_ice_mode_enable();
	if (res < 0) {
		ipio_err("Failed to enable ICE mode\n");
		goto out;
	}

	mdelay(5);

	/*
	 * This command is used to fix the bug of spi clk in 7807F-AB
	 * while operating with flash.
	 */
	if (core_config->chip_id == CHIP_TYPE_ILI7807 && core_config->chip_type == ILI7807_TYPE_F_AB) {
		res = core_config_ice_mode_write(0x4100C, 0x01, 1);
		if (res < 0)
			goto out;
	}

	mdelay(25);

	if (core_config_set_watch_dog(false) < 0) {
		ipio_err("Failed to disable watch dog\n");
		res = -EINVAL;
		goto out;
	}

	/* Disable flash protection from being written */
	core_flash_enable_protect(false);

	res = flash_erase_sector();
	if (res < 0) {
		ipio_err("Failed to erase flash\n");
		goto out;
	}

	mdelay(1);

	res = flash_program_sector();
	if (res < 0) {
		ipio_err("Failed to program flash\n");
		goto out;
	}

	/* We do have to reset chip in order to move new code from flash to iram. */
	ipio_debug(DEBUG_FIRMWARE, "Doing Soft Reset ..\n");
	core_config_ic_reset();

	/* the delay time moving code depends on what the touch IC you're using. */
	mdelay(core_firmware->delay_after_upgrade);

	/* ensure that the chip has been updated */
	ipio_debug(DEBUG_FIRMWARE, "Enter to ICE Mode again\n");
	res = core_config_ice_mode_enable();
	if (res < 0) {
		ipio_err("Failed to enable ICE mode\n");
		goto out;
	}

	mdelay(20);

	/* check the data that we've just written into the iram. */
	res = verify_flash_data();
	if (res == 0)
		ipio_info("Data Correct !\n");

out:
	if (core_config_set_watch_dog(true) < 0) {
		ipio_err("Failed to enable watch dog\n");
		res = -EINVAL;
	}

	core_config_ice_mode_disable();
	return res;
}

#ifdef BOOT_FW_UPGRADE
static int convert_hex_array(void)
{
	int i, j, index = 0;
	int block = 0, blen = 0, bindex = 0;
	uint32_t tmp_addr = 0x0;

	core_firmware->start_addr = 0;
	core_firmware->end_addr = 0;
	core_firmware->checksum = 0;
	core_firmware->crc32 = 0;
	core_firmware->hasBlockInfo = false;

	ipio_info("CTPM_FW = %d\n", ili_fw->fw_len);

	if (ili_fw->fw_len <= 64) {
		ipio_err("The size of CTPM_FW is invaild (%d)\n", ili_fw->fw_len);
		goto out;
	}
	if(CTPM_FW == NULL){
		ipio_err("The CTPM_FW is NULL ");
		goto out;
	}
	/* Get new version from ILI array */
	core_firmware->new_fw_ver[0] = CTPM_FW[19];
	core_firmware->new_fw_ver[1] = CTPM_FW[20];
	core_firmware->new_fw_ver[2] = CTPM_FW[21];

	/* The process will be executed if the comparison is different with origin ver */
	for (i = 0; i < ARRAY_SIZE(core_firmware->old_fw_ver); i++) {
		if (core_firmware->old_fw_ver[i] != core_firmware->new_fw_ver[i]) {
			ipio_info("FW version is different, preparing to upgrade FW\n");
			break;
		}
	}

	if (i == ARRAY_SIZE(core_firmware->old_fw_ver)) {
		ipio_err("FW version is the same as previous version\n");
		goto out;
	}

	/* Extract block info */
	block = CTPM_FW[33];

	if (block > 0) {
		core_firmware->hasBlockInfo = true;

		/* Initialize block's index and length */
		blen = 6;
		bindex = 34;

		for (i = 0; i < block; i++) {
			for (j = 0; j < blen; j++) {
				if (j < 3)
					g_flash_block_info[i].start_addr =
					    (g_flash_block_info[i].start_addr << 8) | CTPM_FW[bindex + j];
				else
					g_flash_block_info[i].end_addr =
					    (g_flash_block_info[i].end_addr << 8) | CTPM_FW[bindex + j];
			}

			bindex += blen;
		}
	}

	/* Fill data into buffer */
	for (i = 0; i < ili_fw->fw_len - 64; i++) {
		flash_fw[i] = CTPM_FW[i + 64];
		index = i / flashtab->sector;
		if (!g_flash_sector[index].data_flag) {
			g_flash_sector[index].ss_addr = index * flashtab->sector;
			g_flash_sector[index].se_addr = (index + 1) * flashtab->sector - 1;
			g_flash_sector[index].dlength =
			    (g_flash_sector[index].se_addr - g_flash_sector[index].ss_addr) + 1;
			g_flash_sector[index].data_flag = true;
		}
	}

	g_section_len = index;

	if (g_flash_sector[g_section_len].se_addr > flashtab->mem_size) {
		ipio_err("The size written to flash is larger than it required (%x) (%x)\n",
			g_flash_sector[g_section_len].se_addr, flashtab->mem_size);
		goto out;
	}

	for (i = 0; i < g_total_sector; i++) {
		/* fill meaing address in an array where is empty */
		if (g_flash_sector[i].ss_addr == 0x0 && g_flash_sector[i].se_addr == 0x0) {
			g_flash_sector[i].ss_addr = tmp_addr;
			g_flash_sector[i].se_addr = (i + 1) * flashtab->sector - 1;
		}

		tmp_addr += flashtab->sector;

		/* set erase flag in the block if the addr of sectors is between them. */
		if (core_firmware->hasBlockInfo) {
			for (j = 0; j < ARRAY_SIZE(g_flash_block_info); j++) {
				if (g_flash_sector[i].ss_addr >= g_flash_block_info[j].start_addr
				    && g_flash_sector[i].se_addr <= g_flash_block_info[j].end_addr) {
					g_flash_sector[i].inside_block = true;
					break;
				}
			}
		}

		/*
		 * protects the reserved address been written and erased.
		 * This feature only applies on the boot upgrade. The addr is progrmmable in normal case.
		 */
		if (g_flash_sector[i].ss_addr == g_start_resrv && g_flash_sector[i].se_addr == g_end_resrv) {
			g_flash_sector[i].inside_block = false;
		}
	}

	/* DEBUG: for showing data with address that will write into fw or be erased */
	for (i = 0; i < g_total_sector; i++) {
		ipio_info
		    ("g_flash_sector[%d]: ss_addr = 0x%x, se_addr = 0x%x, length = %x, data = %d, inside_block = %d\n",
		     i, g_flash_sector[i].ss_addr, g_flash_sector[i].se_addr, g_flash_sector[index].dlength,
		     g_flash_sector[i].data_flag, g_flash_sector[i].inside_block);
	}

	core_firmware->start_addr = 0x0;
	core_firmware->end_addr = g_flash_sector[g_section_len].se_addr;
	ipio_info("start_addr = 0x%06X, end_addr = 0x%06X\n", core_firmware->start_addr, core_firmware->end_addr);
	return 0;

out:
	ipio_err("Failed to convert ILI FW array\n");
	return -1;
}

int core_firmware_boot_upgrade(void)
{
	int res = 0;
	bool power = false;

	ipio_info("BOOT: Starting to upgrade firmware ...\n");

	core_firmware->isUpgrading = true;
	core_firmware->update_status = 0;

	if (ipd->isEnablePollCheckPower) {
		ipd->isEnablePollCheckPower = false;
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		power = true;
	}

	/* store old version before upgrade fw */
	core_firmware->old_fw_ver[0] = core_config->firmware_ver[1];
	core_firmware->old_fw_ver[1] = core_config->firmware_ver[2];
	core_firmware->old_fw_ver[2] = core_config->firmware_ver[3];

	if (flashtab == NULL) {
		ipio_err("Flash table isn't created\n");
		res = -ENOMEM;
		goto out;
	}

	//flash_fw = kcalloc(flashtab->mem_size, sizeof(uint8_t), GFP_KERNEL);
	flash_fw = (uint8_t*)vmalloc(flashtab->mem_size);
	if (ERR_ALLOC_MEM(flash_fw)) {
		ipio_err("Failed to allocate flash_fw memory, %ld\n", PTR_ERR(flash_fw));
		res = -ENOMEM;
		goto out;
	}

	memset(flash_fw, 0xff, (int)sizeof(uint8_t) * flashtab->mem_size);

	g_total_sector = flashtab->mem_size / flashtab->sector;
	if (g_total_sector <= 0) {
		ipio_err("Flash configure is wrong\n");
		res = -1;
		goto out;
	}

	g_flash_sector = kcalloc(g_total_sector, sizeof(struct flash_sector), GFP_KERNEL);
	if (ERR_ALLOC_MEM(g_flash_sector)) {
		ipio_err("Failed to allocate g_flash_sector memory, %ld\n", PTR_ERR(g_flash_sector));
		res = -ENOMEM;
		goto out;
	}

	res = convert_hex_array();
	if (res < 0) {
		ipio_err("Failed to covert firmware data, res = %d\n", res);
		goto out;
	}
	/* calling that function defined at init depends on chips. */
	res = core_firmware->upgrade_func(false);
	if (res < 0) {
		core_firmware->update_status = res;
		ipio_err("Failed to upgrade firmware, res = %d\n", res);
		goto out;
	}

	core_firmware->update_status = 100;
	ipio_info("Update firmware information...\n");
	core_config_get_fw_ver();
	core_config_get_protocol_ver();
	core_config_get_core_ver();
	core_config_get_tp_info();
	core_config_get_key_info();

out:
	if (power) {
		ipd->isEnablePollCheckPower = true;
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);
	}

	//ipio_kfree((void **)&flash_fw);
	if (!ERR_ALLOC_MEM(flash_fw)) {
		vfree(flash_fw);
		flash_fw = NULL;
	}
	ipio_kfree((void **)&g_flash_sector);
	core_firmware->isUpgrading = false;
	return res;
}
#endif /* BOOT_FW_UPGRADE */

#ifdef HOST_DOWNLOAD
int core_firmware_boot_host_download(void)
{
	int res = 0, i = 0;
	bool power = false;
	ipio_info("MAX_AP_FIRMWARE_SIZE + MAX_DLM_FIRMWARE_SIZE + MAX_MP_FIRMWARE_SIZE = 0x%X\n", MAX_AP_FIRMWARE_SIZE + MAX_DLM_FIRMWARE_SIZE + MAX_MP_FIRMWARE_SIZE);
	//flash_fw = kcalloc(MAX_AP_FIRMWARE_SIZE + MAX_DLM_FIRMWARE_SIZE + MAX_MP_FIRMWARE_SIZE, sizeof(uint8_t), GFP_KERNEL);
	flash_fw = (uint8_t*)vmalloc(256 * 1024);
	if (ERR_ALLOC_MEM(flash_fw)) {
		ipio_err("Failed to allocate flash_fw memory, %ld\n", PTR_ERR(flash_fw));
		res = -ENOMEM;
		goto out;
	}

	memset(flash_fw, 0xff, (int)sizeof(uint8_t) * 256 * 1024);

	ipio_info("BOOT: Starting to upgrade firmware ...\n");

	core_firmware->isUpgrading = true;
	core_firmware->update_status = 0;

	if (ipd->isEnablePollCheckPower) {
		ipd->isEnablePollCheckPower = false;
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		power = true;
	}

	/* Fill data into buffer */
	for (i = 0; i < ili_fw->fw_len - 64; i++) {
		flash_fw[i] = CTPM_FW[i + 64];
	}
	memcpy(ap_fw, flash_fw, MAX_AP_FIRMWARE_SIZE);
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/10/30,Add for TP black screen test
	memcpy(dlm_fw, flash_fw + DLM_HEX_ADDRESS, MAX_DLM_FIRMWARE_SIZE);
	core_gesture->ap_start_addr = (ap_fw[0xFFC4+7] << 24) + (ap_fw[0xFFC4+6] << 16) + (ap_fw[0xFFC4+5] << 8) + ap_fw[0xFFC4+4];
	core_gesture->ap_length = MAX_GESTURE_FIRMWARE_SIZE;
	core_gesture->start_addr = (ap_fw[0xFFC4+15] << 24) + (ap_fw[0xFFC4+14] << 16) + (ap_fw[0xFFC4+13] << 8) + ap_fw[0xFFC4+12];
	core_gesture->length = MAX_GESTURE_FIRMWARE_SIZE;
	core_gesture->area_section = (ap_fw[0xFFC4+3] << 24) + (ap_fw[0xFFC4+2] << 16) + (ap_fw[0xFFC4+1] << 8) + ap_fw[0xFFC4];
#endif
	ipio_debug(DEBUG_FIRMWARE, "gesture_start_addr = 0x%x, length = 0x%x\n", core_gesture->start_addr, core_gesture->length);
	ipio_debug(DEBUG_FIRMWARE, "area = %d, ap_start_addr = 0x%x, ap_length = 0x%x\n", core_gesture->area_section, core_gesture->ap_start_addr, core_gesture->ap_length);
	memcpy(mp_fw, flash_fw + MP_HEX_ADDRESS, MAX_MP_FIRMWARE_SIZE);
	memcpy(gesture_fw, flash_fw + core_gesture->start_addr, core_gesture->length);
	//itek_platform_disable_irq();
	gpio_direction_output(ipd->reset_gpio, 1);
	mdelay(ipd->delay_time_high);
	gpio_set_value(ipd->reset_gpio, 0);
	mdelay(ipd->delay_time_low);
	gpio_set_value(ipd->reset_gpio, 1);
	mdelay(ipd->edge_delay);
	res = core_firmware->upgrade_func(true);
	if (res < 0) {
		core_firmware->update_status = res;
		ipio_err("Failed to upgrade firmware, res = %d\n", res);
		goto out;
	}
	core_firmware->update_status = 100;
	ipio_info("Update firmware information...\n");
	/*core_config_get_fw_ver();
	core_config_get_protocol_ver();
	core_config_get_core_ver();
	core_config_get_tp_info();
	core_config_get_key_info();*/

out:
	if (power) {
		ipd->isEnablePollCheckPower = true;
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);
	}

	//ipio_kfree((void **)&flash_fw);
	if (!ERR_ALLOC_MEM(flash_fw)) {
		vfree(flash_fw);
		flash_fw = NULL;
	}
	ipio_kfree((void **)&g_flash_sector);
	core_firmware->isUpgrading = false;
	//itek_platform_enable_irq();
	return res;
}


EXPORT_SYMBOL(core_firmware_boot_host_download);
#endif

static int convert_hex_file(uint8_t *pBuf, uint32_t nSize, bool isIRAM)
{
	uint32_t i = 0, j = 0, k = 0;
#ifdef HOST_DOWNLOAD
	static int do_once = 0;
#endif
	uint32_t nLength = 0, nAddr = 0, nType = 0;
	uint32_t nStartAddr = 0x0, nEndAddr = 0x0, nChecksum = 0x0, nExAddr = 0;
	uint32_t tmp_addr = 0x0;
	int index = 0, block = 0;

	core_firmware->start_addr = 0;

	core_firmware->end_addr = 0;
	core_firmware->checksum = 0;
	core_firmware->crc32 = 0;
	core_firmware->hasBlockInfo = false;

	memset(g_flash_block_info, 0x0, sizeof(g_flash_block_info));
#ifdef HOST_DOWNLOAD
	memset(ap_fw, 0xFF, sizeof(ap_fw));
	memset(dlm_fw, 0xFF, sizeof(dlm_fw));
	memset(mp_fw, 0xFF, sizeof(mp_fw));
	memset(gesture_fw, 0xFF, sizeof(gesture_fw));
#endif
	/* Parsing HEX file */
	for (; i < nSize;) {
		int32_t nOffset;

		nLength = HexToDec(&pBuf[i + 1], 2);
		nAddr = HexToDec(&pBuf[i + 3], 4);
		nType = HexToDec(&pBuf[i + 7], 2);

		/* calculate checksum */
		for (j = 8; j < (2 + 4 + 2 + (nLength * 2)); j += 2) {
			if (nType == 0x00) {
				/* for ice mode write method */
				nChecksum = nChecksum + HexToDec(&pBuf[i + 1 + j], 2);
			}
		}

		if (nType == 0x04) {
			nExAddr = HexToDec(&pBuf[i + 9], 4);
		}

		if (nType == 0x02) {
			nExAddr = HexToDec(&pBuf[i + 9], 4);
			nExAddr = nExAddr >> 12;
		}

		if (nType == 0xAE) {
			core_firmware->hasBlockInfo = true;
			/* insert block info extracted from hex */
			if (block < 4) {
				g_flash_block_info[block].start_addr = HexToDec(&pBuf[i + 9], 6);
				g_flash_block_info[block].end_addr = HexToDec(&pBuf[i + 9 + 6], 6);
				ipio_debug(DEBUG_FIRMWARE, "Block[%d]: start_addr = %x, end = %x\n",
				    block, g_flash_block_info[block].start_addr, g_flash_block_info[block].end_addr);
			}
			block++;
		}

		nAddr = nAddr + (nExAddr << 16);
		if (pBuf[i + 1 + j + 2] == 0x0D) {
			nOffset = 2;
		} else {
			nOffset = 1;
		}

		if (nType == 0x00) {
			if (nAddr > MAX_HEX_FILE_SIZE) {
				ipio_err("Invalid hex format\n");
				goto out;
			}

			if (nAddr < nStartAddr) {
				nStartAddr = nAddr;
			}
			if ((nAddr + nLength) > nEndAddr) {
				nEndAddr = nAddr + nLength;
			}

			/* fill data */
			for (j = 0, k = 0; j < (nLength * 2); j += 2, k++) {
				if (isIRAM) {
#ifdef HOST_DOWNLOAD
					if (nAddr < 0x10000) {
						ap_fw[nAddr + k] = HexToDec(&pBuf[i + 9 + j], 2);
					} else if ((nAddr >= DLM_HEX_ADDRESS) && (nAddr < MP_HEX_ADDRESS)) {
						if (nAddr < DLM_HEX_ADDRESS + MAX_DLM_FIRMWARE_SIZE) {
							dlm_fw[nAddr - DLM_HEX_ADDRESS + k] = HexToDec(&pBuf[i + 9 + j], 2);
						}
					} else if (nAddr >= MP_HEX_ADDRESS) {
						mp_fw[nAddr - MP_HEX_ADDRESS + k] = HexToDec(&pBuf[i + 9 + j], 2);
					}
					if (nAddr > MAX_AP_FIRMWARE_SIZE && do_once == 0) {
						do_once = 1;
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/10/30,Add for TP black screen test
						core_gesture->ap_start_addr = (ap_fw[0xFFC4+7] << 24) + (ap_fw[0xFFC4+6] << 16) + (ap_fw[0xFFC4+5] << 8) + ap_fw[0xFFC4+4];
						core_gesture->ap_length = MAX_GESTURE_FIRMWARE_SIZE;
						core_gesture->start_addr = (ap_fw[0xFFC4+15] << 24) + (ap_fw[0xFFC4+14] << 16) + (ap_fw[0xFFC4+13] << 8) + ap_fw[0xFFC4+12];
						core_gesture->length = MAX_GESTURE_FIRMWARE_SIZE;
						core_gesture->area_section = (ap_fw[0xFFC4+3] << 24) + (ap_fw[0xFFC4+2] << 16) + (ap_fw[0xFFC4+1] << 8) + ap_fw[0xFFC4];
						/*core_gesture->ap_start_addr = (ap_fw[0xFFD3] << 24) + (ap_fw[0xFFD2] << 16) + (ap_fw[0xFFD1] << 8) + ap_fw[0xFFD0];
						core_gesture->ap_length = MAX_GESTURE_FIRMWARE_SIZE;
						core_gesture->start_addr = (ap_fw[0xFFDB] << 24) + (ap_fw[0xFFDA] << 16) + (ap_fw[0xFFD9] << 8) + ap_fw[0xFFD8];
						core_gesture->length = MAX_GESTURE_FIRMWARE_SIZE;
						core_gesture->area_section = (ap_fw[0xFFCF] << 24) + (ap_fw[0xFFCE] << 16) + (ap_fw[0xFFCD] << 8) + ap_fw[0xFFCC];
					*/
#endif
					}
					if (nAddr >= core_gesture->start_addr && nAddr < core_gesture->start_addr + MAX_GESTURE_FIRMWARE_SIZE) {
						gesture_fw[nAddr - core_gesture->start_addr + k] = HexToDec(&pBuf[i + 9 + j], 2);
					}
#else
					iram_fw[nAddr + k] = HexToDec(&pBuf[i + 9 + j], 2);
#endif
				} else {
					flash_fw[nAddr + k] = HexToDec(&pBuf[i + 9 + j], 2);

					if ((nAddr + k) != 0) {
						index = ((nAddr + k) / flashtab->sector);
						if (!g_flash_sector[index].data_flag) {
							g_flash_sector[index].ss_addr = index * flashtab->sector;
							g_flash_sector[index].se_addr =
							    (index + 1) * flashtab->sector - 1;
							g_flash_sector[index].dlength =
							    (g_flash_sector[index].se_addr -
							     g_flash_sector[index].ss_addr) + 1;
							g_flash_sector[index].data_flag = true;
						}
					}
				}
			}
		}
		i += 1 + 2 + 4 + 2 + (nLength * 2) + 2 + nOffset;
	}
	// for (i = 0; i < MAX_AP_FIRMWARE_SIZE;) {
	// 	if(i%16 == 0)
	// 		printk("0x%4x:", i);
	// 	printk("0x%2x,", ap_fw[i]);
	// 	i++;
	// 	if( i%16 == 0)
	// 		printk("\n");
	// }
	// for (i = 0; i < MAX_DLM_FIRMWARE_SIZE;) {
	// 	if(i%16 == 0)
	// 		printk("0x%4x:", i);
	// 	printk("0x%2x,", dlm_fw[i]);
	// 	i++;
	// 	if( i%16 == 0)
	// 		printk("\n");
	// }
	#ifdef HOST_DOWNLOAD
		return 0;
	#endif

	/* Update the length of section */
	g_section_len = index;

	if (g_flash_sector[g_section_len - 1].se_addr > flashtab->mem_size) {
		ipio_err("The size written to flash is larger than it required (%x) (%x)\n",
			g_flash_sector[g_section_len - 1].se_addr, flashtab->mem_size);
		goto out;
	}

	for (i = 0; i < g_total_sector; i++) {
		/* fill meaing address in an array where is empty */
		if (g_flash_sector[i].ss_addr == 0x0 && g_flash_sector[i].se_addr == 0x0) {
			g_flash_sector[i].ss_addr = tmp_addr;
			g_flash_sector[i].se_addr = (i + 1) * flashtab->sector - 1;
		}

		tmp_addr += flashtab->sector;

		/* set erase flag in the block if the addr of sectors is between them. */
		if (core_firmware->hasBlockInfo) {
			for (j = 0; j < ARRAY_SIZE(g_flash_block_info); j++) {
				if (g_flash_sector[i].ss_addr >= g_flash_block_info[j].start_addr
				    && g_flash_sector[i].se_addr <= g_flash_block_info[j].end_addr) {
					g_flash_sector[i].inside_block = true;
					break;
				}
			}
		}
	}

	/* DEBUG: for showing data with address that will write into fw or be erased */
	for (i = 0; i < g_total_sector; i++) {
		ipio_debug(DEBUG_FIRMWARE,
		    "g_flash_sector[%d]: ss_addr = 0x%x, se_addr = 0x%x, length = %x, data = %d, inside_block = %d\n", i,
		    g_flash_sector[i].ss_addr, g_flash_sector[i].se_addr, g_flash_sector[index].dlength,
		    g_flash_sector[i].data_flag, g_flash_sector[i].inside_block);
	}

	core_firmware->start_addr = nStartAddr;
	core_firmware->end_addr = nEndAddr;
	ipio_debug(DEBUG_FIRMWARE, "nStartAddr = 0x%06X, nEndAddr = 0x%06X\n", nStartAddr, nEndAddr);
	return 0;

out:
	ipio_err("Failed to convert HEX data\n");
	return -1;
}

/*
 * It would basically be called by ioctl when users want to upgrade firmware.
 *
 * @pFilePath: pass a path where locates user's firmware file.
 *
 */
int core_firmware_upgrade(const char *pFilePath, bool isIRAM)
{
	int res = 0, fsize = 0;
	uint8_t *hex_buffer = NULL;

	bool power = false;
	int ret = 0;

#if (ILITEK_TYPE == ILITEK_TYPE_LH)
	int i = 0;
	uint8_t cmd[4] = { 0 };
#endif
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	loff_t pos = 0;

	//ipio_info("core_firmware_upgrade before request_firmwre\n");

	core_firmware->isUpgrading = true;
	core_firmware->update_status = 0;
	ipd->common_reset = 1;

	if (ipd->isEnablePollCheckPower) {
		ipd->isEnablePollCheckPower = false;
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		power = true;
	}

	if (core_firmware->fw == NULL) {
		ipio_info("core_firmware_upgrade before request_firmwre\n");
		ret = request_firmware(&ili_fw_entry, ili_fw->firmware_bin_name, &(ipd->spi->dev));
		if (ret != 0) {
		    ipio_err("%s : request test firmware failed! ret = %d\n", __func__, ret);
			goto pFile;
		}
		core_firmware->fw = ili_fw_entry;
	} else {
		ipio_info("core_firmware_upgrade not need request firmware\n");
		goto no_hex_file;
	}
	if (!ERR_ALLOC_MEM(core_firmware->fw) && (!ERR_ALLOC_MEM(core_firmware->fw->data)) && (core_firmware->fw->size != 0) && (ipd->common_reset == 1)) {
		ipio_debug(DEBUG_FIRMWARE, "fw from image file\n");
		goto convert_hex;
	}
pFile:
	ipio_debug(DEBUG_FIRMWARE, "core_firmware_upgrade before filp_open\n");
	pfile = filp_open(pFilePath, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(pfile)) {
		ipio_err("Failed to open the file at %s.\n", pFilePath);
		//if not hex file can ili file
		#ifdef HOST_DOWNLOAD
			goto no_hex_file;
		#endif
		res = -ENOENT;
		return res;
	}

	fsize = pfile->f_inode->i_size;

	ipio_debug(DEBUG_FIRMWARE, "fsize = %d\n", fsize);

	if (fsize <= 0) {
		ipio_err("The size of file is zero\n");
		res = -EINVAL;
		goto out;
	}

#ifndef HOST_DOWNLOAD
	if (flashtab == NULL) {
		ipio_err("Flash table isn't created\n");
		res = -ENOMEM;
		goto out;
	}

	flash_fw = kcalloc(flashtab->mem_size, sizeof(uint8_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(flash_fw)) {
		ipio_err("Failed to allocate flash_fw memory, %ld\n", PTR_ERR(flash_fw));
		res = -ENOMEM;
		goto out;
	}

	memset(flash_fw, 0xff, sizeof(uint8_t) * flashtab->mem_size);

	g_total_sector = flashtab->mem_size / flashtab->sector;
	if (g_total_sector <= 0) {
		ipio_err("Flash configure is wrong\n");
		res = -1;
		goto out;
	}

	g_flash_sector = kcalloc(g_total_sector, sizeof(*g_flash_sector), GFP_KERNEL);
	if (ERR_ALLOC_MEM(g_flash_sector)) {
		ipio_err("Failed to allocate g_flash_sector memory, %ld\n", PTR_ERR(g_flash_sector));
		res = -ENOMEM;
		goto out;
	}
#endif

	hex_buffer = kcalloc(fsize, sizeof(uint8_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(hex_buffer)) {
		ipio_err("Failed to allocate hex_buffer memory, %ld\n", PTR_ERR(hex_buffer));
		res = -ENOMEM;
		goto out;
	}

	/* store current userspace mem segment. */
	old_fs = get_fs();

	/* set userspace mem segment equal to kernel's one. */
	set_fs(get_ds());

	/* read firmware data from userspace mem segment */
	vfs_read(pfile, hex_buffer, fsize, &pos);

	/* restore userspace mem segment after read. */
	set_fs(old_fs);

convert_hex:
	ipio_debug(DEBUG_FIRMWARE, "core_firmware_upgrade begin convert_hex\n");
	if (!ERR_ALLOC_MEM(core_firmware->fw) && (!ERR_ALLOC_MEM(core_firmware->fw->data)) && (core_firmware->fw->size != 0) && (ipd->common_reset == 1)) {
		res = convert_hex_file((uint8_t *)core_firmware->fw->data, core_firmware->fw->size, true);
	}
	else {
		res = convert_hex_file(hex_buffer, fsize, isIRAM);
	}
	//res = convert_hex_file(hex_buffer, fsize, isIRAM);
	if (res < 0) {
		ipio_err("Failed to covert firmware data, res = %d\n", res);
		goto out;
	}
no_hex_file:
	if (core_config->isEnableGesture) {
		if(core_gesture->entry == true) {
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.TP.FUNCTION.2018/10/30,Add for TP black screen test
			core_gesture->ap_start_addr = (ap_fw[0xFFC4+7] << 24) + (ap_fw[0xFFC4+6] << 16) + (ap_fw[0xFFC4+5] << 8) + ap_fw[0xFFC4+4];
			core_gesture->ap_length = MAX_GESTURE_FIRMWARE_SIZE;
			core_gesture->start_addr = (ap_fw[0xFFC4+15] << 24) + (ap_fw[0xFFC4+14] << 16) + (ap_fw[0xFFC4+13] << 8) + ap_fw[0xFFC4+12];
			core_gesture->length = MAX_GESTURE_FIRMWARE_SIZE;
			core_gesture->area_section = (ap_fw[0xFFC4+3] << 24) + (ap_fw[0xFFC4+2] << 16) + (ap_fw[0xFFC4+1] << 8) + ap_fw[0xFFC4];
			/*core_gesture->ap_start_addr = (ap_fw[0xFFD3] << 24) + (ap_fw[0xFFD2] << 16) + (ap_fw[0xFFD1] << 8) + ap_fw[0xFFD0];
			core_gesture->ap_length = MAX_GESTURE_FIRMWARE_SIZE;
			core_gesture->start_addr = (ap_fw[0xFFDB] << 24) + (ap_fw[0xFFDA] << 16) + (ap_fw[0xFFD9] << 8) + ap_fw[0xFFD8];
			core_gesture->length = MAX_GESTURE_FIRMWARE_SIZE;
			core_gesture->area_section = (ap_fw[0xFFCF] << 24) + (ap_fw[0xFFCE] << 16) + (ap_fw[0xFFCD] << 8) + ap_fw[0xFFCC];
			*/
#endif
		}
	}
	/* calling that function defined at init depends on chips. */
	ipio_debug(DEBUG_FIRMWARE, "core_firmware_upgrade begin upgrade\n");
	res = core_firmware->upgrade_func(isIRAM);
	if (res < 0) {
		ipio_err("Failed to upgrade firmware, res = %d\n", res);
		goto out;
	}
#if (ILITEK_TYPE == ILITEK_TYPE_LH)
	if(core_fr->actual_fw_mode != protocol->test_mode && core_gesture->suspend != true)  {
		ipio_info("core upgrade check trim code,core_config->firmware_ver[3] = %d\n", core_config->firmware_ver[3]);
		//check tp set trim code status
		if (core_config->firmware_ver[3] >= 14) {
			for (i = 0; i < 20; i++) {
				cmd[0] = 0x04;
				res = core_write(core_config->slave_i2c_addr, cmd, 1);
				if (res < 0) {
					ipio_err("Failed to write data, %d\n", res);
				}

				res = core_read(core_config->slave_i2c_addr, cmd, 3);
				ipio_info("read value 0x%X 0x%X 0x%X\n", cmd[0], cmd[1], cmd[2]);
				if (res < 0) {
					ipio_err("Failed to read tp set ddi trim code %d\n", res);
				}
				if (cmd[0] == 0x55) {
					is_tp_fw_done = 1;
					ipio_info("TP set ddi trim code ok read value 0x%X i = %d\n", cmd[0], i);
					break;
				}
				else if (cmd[0] == 0x35) {
					ipio_info("TP set ddi trim code bypass read value 0x%X\n", cmd[0]);
					break;
				}
				mdelay(3);
			}
			if (i >= 20) {
				ipio_err("check TP set ddi trim code error\n");
			}
		}
	}
#endif
	//core_config_get_fw_ver();

#ifndef HOST_DOWNLOAD
	ipio_info("Update TP/Firmware information...\n");
	core_config_get_fw_ver();
	core_config_get_protocol_ver();
	core_config_get_core_ver();
	core_config_get_tp_info();
	//core_config_get_key_info();
#endif
out:
	ipio_debug(DEBUG_FIRMWARE, "core_firmware_upgrade begin out\n");

	if (power) {
		ipd->isEnablePollCheckPower = true;
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);
	}
	core_firmware->isUpgrading = false;
	ipd->common_reset = 0;
	//core_firmware->fw = NULL;

	if (ili_fw_entry) {
		ipio_info("core_firmware_upgrade  release firmwre\n");
		release_firmware(ili_fw_entry);
		ili_fw_entry = NULL;
	}

	if (ERR_ALLOC_MEM(pfile))
	{
		ipio_info("\n");
		return res;
	}
	filp_close(pfile, NULL);
	ipio_kfree((void **)&hex_buffer);
	ipio_kfree((void **)&flash_fw);
	ipio_kfree((void **)&g_flash_sector);
	ipio_debug(DEBUG_FIRMWARE, "core_firmware_upgrade success out\n");
	return res;
}

int core_firmware_init(void)
{
	int i = 0, j = 0;

	core_firmware = kzalloc(sizeof(*core_firmware), GFP_KERNEL);
	if (ERR_ALLOC_MEM(core_firmware)) {
		ipio_err("Failed to allocate core_firmware mem, %ld\n", PTR_ERR(core_firmware));
		core_firmware_remove();
		return -ENOMEM;
	}

	core_firmware->hasBlockInfo = false;
	core_firmware->isboot = false;

	for (; i < ARRAY_SIZE(ipio_chip_list); i++) {
		if (ipio_chip_list[i] == TP_TOUCH_IC) {
			for (j = 0; j < 4; j++) {
				core_firmware->old_fw_ver[i] = core_config->firmware_ver[i];
				core_firmware->new_fw_ver[i] = 0x0;
			}

			if (ipio_chip_list[i] == CHIP_TYPE_ILI7807) {
				core_firmware->max_count = 0xFFFF;
				core_firmware->isCRC = false;
				core_firmware->upgrade_func = tddi_fw_upgrade;
				core_firmware->delay_after_upgrade = 100;
			} else if (ipio_chip_list[i] == CHIP_TYPE_ILI9881) {
				core_firmware->max_count = 0x1FFFF;
				core_firmware->isCRC = true;
			#ifdef HOST_DOWNLOAD
				core_firmware->upgrade_func = host_download;
			#else
				core_firmware->upgrade_func = tddi_fw_upgrade;
			#endif
				core_firmware->delay_after_upgrade = 200;
			}
			return 0;
		}
	}

	ipio_err("Can't find this chip in support list\n");
	return 0;
}

void core_firmware_remove(void)
{
	ipio_info("Remove core-firmware members\n");
	ipio_kfree((void **)&core_firmware);
}
