/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2012-2018, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_flash.c
*
* Author: Focaltech Driver Team
*
* Created: 2017-12-06
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"
#include "focaltech_flash.h"

/*****************************************************************************
* Static variables
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
const u8 fw_file[] = {
#include FTS_UPGRADE_FW_FILE
};

const u8 fw_file2[] = {
#include FTS_UPGRADE_FW2_FILE
};

const u8 fw_file3[] = {
#include FTS_UPGRADE_FW3_FILE
};

struct upgrade_fw_info fw_list[] = {
    {FTS_MODULE_ID,  "auo", fw_file,  sizeof(fw_file)},
    {FTS_MODULE2_ID, "auo", fw_file2, sizeof(fw_file2)},
    {FTS_MODULE3_ID, "auo", fw_file3, sizeof(fw_file3)},
};

struct fts_upgrade *fwupgrade;

static int fts_check_bootid(void)
{
    int ret = 0;
    u8 cmd = 0;
    u8 id[2] = { 0 };
    struct ft_chip_t *chip_id = &fts_data->ic_info.ids;

    cmd = FTS_CMD_READ_ID;
    ret = fts_read(&cmd, 1, id, 2);
    if (ret < 0) {
        FTS_ERROR("read boot id fail");
        return ret;
    }

    FTS_INFO("read boot id:0x%02x 0x%02x", id[0], id[1]);
    if ((chip_id->chip_idh == id[0]) && (chip_id->chip_idl == id[1])) {
        return 0;
    }

    return -EIO;
}

static int fts_fwupg_hardware_reset_to_boot(void)
{
    fts_reset_proc(0);
    mdelay(8);

    return 0;
}

static int fts_enter_into_boot(void)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    u8 cmd = 0;

    FTS_INFO("enter into boot environment");
    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        /* hardware tp reset to boot */
        fts_fwupg_hardware_reset_to_boot();

        /* enter into boot & check boot id*/
        for (j = 0; j < FTS_READ_BOOT_ID_TIMEOUT; j++) {
            cmd = FTS_CMD_START1;
            ret = fts_write(&cmd, 1);
            if (ret >= 0) {
                mdelay(8);
                ret = fts_check_bootid();
                if (0 == ret) {
                    FTS_INFO("boot id check pass, retry=%d", i);
                    return 0;
                }
            }
        }
    }

    return -EIO;
}

static int fts_pram_write(u32 saddr, const u8 *buf, u32 len)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    u8 *cmd;
    u32 addr = 0;
    u32 offset = 0;
    u32 remainder = 0;
    u32 packet_number;
    u32 packet_len = 0;
    u32 packet_size = FTS_FLASH_PACKET_LENGTH;

    FTS_INFO("pram write");
    if (NULL == buf) {
        FTS_ERROR("fw buf is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_APP)) {
        FTS_ERROR("fw length(%d) fail", len);
        return -EINVAL;
    }

    cmd = vmalloc(packet_size + FTS_CMD_WRITE_LEN);
    if (NULL == cmd) {
        FTS_ERROR("malloc memory for pram write buffer fail");
        return -ENOMEM;
    }
    memset(cmd, 0, packet_size + FTS_CMD_WRITE_LEN);

    packet_number = len / packet_size;
    remainder = len % packet_size;
    if (remainder > 0)
        packet_number++;
    packet_len = packet_size;

    cmd[0] = FTS_ROMBOOT_CMD_WRITE;
    for (i = 0; i < packet_number; i++) {
        offset = i * packet_size;
        addr = saddr + offset;
        cmd[1] = BYTE_OFF_16(addr);
        cmd[2] = BYTE_OFF_8(addr);
        cmd[3] = BYTE_OFF_0(addr);

        /* last packet */
        if ((i == (packet_number - 1)) && remainder)
            packet_len = remainder;
        cmd[4] = BYTE_OFF_8(packet_len);
        cmd[5] = BYTE_OFF_0(packet_len);

        for (j = 0; j < packet_len; j++) {
            cmd[FTS_CMD_WRITE_LEN + j] = buf[offset + j];
        }
        ret = fts_write(cmd, FTS_CMD_WRITE_LEN + packet_len);
        if (ret < 0) {
            FTS_ERROR("write fw to pram(%d) fail", i);
            goto write_pram_err;
        }


    }

write_pram_err:
    if (cmd) {
        vfree(cmd);
        cmd = NULL;
    }
    return ret;
}

static int fts_ecc_cal_tp(u32 ecc_saddr, u32 ecc_len, u16 *ecc_value)
{
    int ret = 0;
    int i = 0;
    u8 cmd[FTS_ROMBOOT_CMD_ECC_LEN] = { 0 };
    u8 value[2] = { 0 };

    FTS_INFO("ecc calc in tp");
    cmd[0] = FTS_ROMBOOT_CMD_ECC;
    cmd[1] = BYTE_OFF_16(ecc_saddr);
    cmd[2] = BYTE_OFF_8(ecc_saddr);
    cmd[3] = BYTE_OFF_0(ecc_saddr);
    cmd[4] = BYTE_OFF_16(ecc_len);
    cmd[5] = BYTE_OFF_8(ecc_len);
    cmd[6] = BYTE_OFF_0(ecc_len);

    /* make boot to calculate ecc in pram */
    ret = fts_write(cmd, FTS_ROMBOOT_CMD_ECC_LEN);
    if (ret < 0) {
        FTS_ERROR("ecc calc cmd fail");
        return ret;
    }
    mdelay(3);

    /* wait boot calculate ecc finish */
    cmd[0] = FTS_ROMBOOT_CMD_ECC_FINISH;
    for (i = 0; i < FTS_ECC_FINISH_TIMEOUT; i++) {
        ret = fts_read(cmd, 1, value, 1);
        if (ret < 0) {
            FTS_ERROR("ecc finish cmd fail");
            return ret;
        }
        if (0 == value[0])
            break;
        mdelay(1);
    }
    if (i >= FTS_ECC_FINISH_TIMEOUT) {
        FTS_ERROR("wait ecc finish timeout");
        return -EIO;
    }

    /* get ecc value calculate in boot */
    cmd[0] = FTS_ROMBOOT_CMD_ECC_READ;
    ret = fts_read(cmd, 1, value, 2);
    if (ret < 0) {
        FTS_ERROR("ecc read cmd fail");
        return ret;
    }

    *ecc_value = ((u16)(value[0] << 8) + value[1]) & 0x0000FFFF;
    return 0;
}

static int fts_ecc_cal_host(const u8 *data, u32 data_len, u16 *ecc_value)
{
    u16 ecc = 0;
    u16 i = 0;
    u16 j = 0;
    u16 al2_fcs_coef = AL2_FCS_COEF;

    for (i = 0; i < data_len; i += 2 ) {
        ecc ^= ((data[i] << 8) | (data[i + 1]));
        for (j = 0; j < 16; j ++) {
            if (ecc & 0x01)
                ecc = (u16)((ecc >> 1) ^ al2_fcs_coef);
            else
                ecc >>= 1;
        }
    }

    *ecc_value = ecc & 0x0000FFFF;
    return 0;
}

static int fts_pram_start(void)
{
    int ret = 0;
    u8 cmd = FTS_ROMBOOT_CMD_START_APP;

    FTS_INFO("remap to start pram");
    ret = fts_write(&cmd, 1);
    if (ret < 0) {
        FTS_ERROR("write start pram cmd fail");
        return ret;
    }

    return 0;
}

/*
 * description: download fw to IC and run
 *
 * param - buf: const, fw data buffer
 *         len: length of fw
 *
 * return 0 if success, otherwise return error code
 */
static int fts_fw_write_start(const u8 *buf, u32 len, bool need_reset)
{
    int ret = 0;
    u16 ecc_in_host = 0;
    u16 ecc_in_tp = 0;
    u32 fw_start_addr = 0;
    u32 fw_len = 0;
    u16 code_len = 0;
    u16 code_len_n = 0;

    FTS_INFO("begin to write and start fw(bin len:%d)", len);
    /* get app length */
    code_len = ((u16)buf[FTS_APP_INFO_OFFSET + 0] << 8)
               + buf[FTS_APP_INFO_OFFSET + 1];
    code_len_n = ((u16)buf[FTS_APP_INFO_OFFSET + 2] << 8)
                 + buf[FTS_APP_INFO_OFFSET + 3];
    if ((code_len + code_len_n) != 0xFFFF) {
        FTS_ERROR("code len(%x %x) fail", code_len, code_len_n);
        return -EINVAL;
    }

    fw_len = (u32)code_len;
    if ((fw_len < FTS_MIN_LEN) || (fw_len > FTS_MAX_LEN_APP)) {
        FTS_ERROR("fw length(%d) is invalid", fw_len);
        return -EINVAL;
    }

    FTS_INFO("fw length in fact:%d", fw_len);
    fts_data->fw_is_running = false;

    if (need_reset) {
        /* enter into boot environment */
        ret = fts_enter_into_boot();
        if (ret < 0) {
            FTS_ERROR("enter into boot environment fail");
            return ret;
        }
    }

    /* write pram */
    ret = fts_pram_write(fw_start_addr, buf, fw_len);
    if (ret < 0) {
        FTS_ERROR("write pram fail");
        return ret;
    }

    /* ecc check */
    ret = fts_ecc_cal_host(buf, fw_len, &ecc_in_host);
    if (ret < 0) {
        FTS_ERROR("ecc in host calc fail");
        return ret;
    }

    ret = fts_ecc_cal_tp(fw_start_addr, fw_len, &ecc_in_tp);
    if (ret < 0) {
        FTS_ERROR("ecc in tp calc fail");
        return ret;
    }

    FTS_INFO("ecc in tp:%04x host:%04x", ecc_in_tp, ecc_in_host);
    if (ecc_in_tp != ecc_in_host) {
        FTS_ERROR("ecc check fail");
        return -EIO;
    }

    /* remap pram and run fw */
    ret = fts_pram_start();
    if (ret < 0) {
        FTS_ERROR("pram start fail");
        return ret;
    }

    fts_data->fw_is_running = true;
    FTS_INFO("fw download successfully");
    return 0;
}

static int fts_fw_download(const u8 *buf, u32 len, bool need_reset)
{
    int ret = 0;
    int i = 0;
    struct fts_ts_data *ts_data = fts_data;

    FTS_INFO("fw upgrade download function");
    if (NULL == buf) {
        FTS_ERROR("fw buf is null");
        return -EINVAL;
    }

    if (len < FTS_MIN_LEN) {
        FTS_ERROR("fw bin length(%d) is invalid", len);
        return -EINVAL;
    }

    ts_data->fw_loading = 1;
    fts_irq_disable();
#if FTS_ESDCHECK_EN
    fts_esdcheck_switch(DISABLE);
#endif

    for (i = 0; i < 3; i++) {
        FTS_INFO("fw download times:%d", i + 1);
        ret = fts_fw_write_start(buf, len, need_reset);
        if (0 == ret)
            break;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_switch(ENABLE);
#endif
    fts_irq_enable();
    ts_data->fw_loading = 0;

    if (i >= 3) {
        FTS_ERROR("fw download fail");
        return -EIO;
    }

    return 0;
}

int fts_read_file(char *file_name, u8 **file_buf)
{
    int ret = 0;
    char file_path[FILE_NAME_LENGTH] = { 0 };
    struct file *filp = NULL;
    struct inode *inode;
    mm_segment_t old_fs;
    loff_t pos;
    loff_t file_len = 0;

    if ((NULL == file_name) || (NULL == file_buf)) {
        FTS_ERROR("filename/filebuf is NULL");
        return -EINVAL;
    }

    snprintf(file_path, FILE_NAME_LENGTH, "%s%s", FTS_FW_BIN_FILEPATH, file_name);
    filp = filp_open(file_path, O_RDONLY, 0);
    if (IS_ERR(filp)) {
        FTS_ERROR("open %s file fail", file_path);
        return -ENOENT;
    }

#if 1
    inode = filp->f_inode;
#else
    /* reserved for linux earlier verion */
    inode = filp->f_dentry->d_inode;
#endif

    file_len = inode->i_size;
    *file_buf = (u8 *)vmalloc(file_len);
    if (NULL == *file_buf) {
        FTS_ERROR("file buf malloc fail");
        filp_close(filp, NULL);
        return -ENOMEM;
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    ret = vfs_read(filp, *file_buf, file_len , &pos);
    if (ret < 0)
        FTS_ERROR("read file fail");
    FTS_INFO("file len:%x read len:%x pos:%x", (u32)file_len, ret, (u32)pos);
    filp_close(filp, NULL);
    set_fs(old_fs);

    return ret;
}

int fts_upgrade_bin(char *fw_name, bool force)
{
    int ret = 0;
    u32 fw_file_len = 0;
    u8 *fw_file_buf = NULL;

    FTS_INFO("start upgrade with fw bin");
    if (fts_data->fw_loading) {
        FTS_INFO("fw is loading, not download again");
        return -EINVAL;
    }

    ret = fts_read_file(fw_name, &fw_file_buf);
    if ((ret < 0) || (ret < FTS_MIN_LEN)) {
        FTS_ERROR("read fw bin file(sdcard) fail, len:%d", fw_file_len);
        goto err_bin;
    }

    fw_file_len = ret;
    FTS_INFO("fw bin file len:%d", fw_file_len);
    ret = fts_fw_download(fw_file_buf, fw_file_len, true);
    if (ret < 0) {
        FTS_ERROR("upgrade fw bin failed");
        goto err_bin;
    }

    FTS_INFO("upgrade fw bin success");

err_bin:
    if (fw_file_buf) {
        vfree(fw_file_buf);
        fw_file_buf = NULL;
    }
    return ret;
}

int fts_upgrade_oppo(bool force)
{
    int ret = 0;
    struct fts_upgrade *upg = fwupgrade;

    FTS_INFO("start upgrade with fw bin");
    if (!upg || !upg->fw) {
        FTS_ERROR("upg/fw is null");
        return -EINVAL;
    }
    
    if (fts_data->fw_loading) {
        FTS_INFO("fw is loading, not download again");
        return -EINVAL;
    }

    FTS_INFO("fw file len:%d", upg->fw_length);
    ret = fts_fw_download(upg->fw, upg->fw_length, true);
    if (ret < 0) {
        FTS_ERROR("upgrade fw(oppo) failed");
        return ret;
    }

    FTS_INFO("upgrade fw bin success");

    return ret;
}

int fts_fw_enter_test_environment(int test_state)
{
    int ret = 0;
    u8 detach_flag = 0;
    struct fts_upgrade *upg = fwupgrade;

    FTS_INFO("fw test download function");
    if (!upg || !upg->fw) {
        FTS_ERROR("upg/fw is null");
        return -EINVAL;
    }

    if (fts_data->fw_loading) {
        FTS_INFO("fw is loading, not download again");
        return -EINVAL;
    }

    if (upg->fw_length <= FTS_MAX_LEN_APP) {
        FTS_INFO("not multi-app");
        return 0;
    }

    if (test_state) {
        ret = fts_fw_download(upg->fw + FTS_MAX_LEN_APP, upg->fw_length, true);
    } else {
        ret = fts_fw_download(upg->fw, upg->fw_length, true);
    }
    if (ret < 0) {
        FTS_ERROR("fw(app2) download fail");
        return ret;
    }

    msleep(50);
    ret = fts_read_reg(FTS_REG_FACTORY_MODE_DETACH_FLAG, &detach_flag);
    FTS_INFO("regb4:0x%02x", detach_flag);

    return 0;
}

int fts_fw_resume(void)
{
    int ret = 0;
    struct fts_upgrade *upg = fwupgrade;
    const struct firmware *fw = NULL;
    struct device *dev = NULL;

    FTS_INFO("fw upgrade resume function");
    if (!upg || !upg->fw) {
        FTS_ERROR("upg/fw is null");
        return -EINVAL;
    }

    if (fts_data->fw_loading) {
        FTS_INFO("fw is loading, not download again");
        return -EINVAL;
    }

    dev = &fts_data->input_dev->dev;
    /* 1. request firmware */
    ret = request_firmware(&fw, "focaltech_ts_fw.bin", dev);
    if (ret != 0) {
        FTS_ERROR("%s:firmware request fail, ret=%d, fw_name=%s\n",
                  __func__, ret, "focaltech_ts_fw.bin");
        FTS_INFO("download fw from bootimage");
        ret = fts_fw_download(upg->fw, upg->fw_length, false);
    } else {
        FTS_INFO("firmware request succeed");
        ret = fts_fw_download(fw->data, fw->size, false);
    }
    if (ret < 0) {
        FTS_ERROR("fw resume download failed");
        return ret;
    }

    if (fw != NULL) {
        release_firmware(fw);
        fw = NULL;
    }

    return 0;
}

int fts_fw_recovery(void)
{
    int ret = 0;
    u8 boot_state = 0;
    u8 chip_id = 0;

    FTS_INFO("check if boot recovery");
    if (fts_data->fw_loading) {
        FTS_INFO("fw is loading, not download again");
        return -EINVAL;
    }

    fts_data->fw_is_running = false;
    ret = fts_check_bootid();
    if (ret < 0) {
        FTS_INFO("check boot id fail");
		fts_data->fw_is_running = true;
        return ret;
    }

    ret = fts_read_reg(0xD0, &boot_state);
    if (ret < 0) {
        FTS_ERROR("read boot state failed, ret=%d", ret);
        return ret;
    }

    if (boot_state != 0x02) {
        FTS_INFO("not in boot mode(0x%x),exit", boot_state);
        fts_data->fw_is_running = true;
        return -EIO;
    }

    FTS_INFO("abnormal situation,need download fw");
    ret = fts_fw_resume();
    if (ret < 0) {
        FTS_ERROR("fts_fw_resume fail");
        return ret;
    }
    msleep(10);
    ret = fts_read_reg(FTS_REG_CHIP_ID, &chip_id);
    FTS_INFO("read chip id:0x%02x", chip_id);

    fts_tp_state_recovery();

    FTS_INFO("boot recovery pass");
    return ret;
}

static int fts_get_module_info(int module_id) 
{
    int ret = 0;
    int i = 0;
    struct upgrade_fw_info *fw = &fw_list[0];

    FTS_INFO("module id:%d", module_id);
    if (FTS_GET_MODULE_ID_NUM > 0) {
        for (i = 0; i < FTS_GET_MODULE_ID_NUM; i++) {
            fw = &fw_list[i];
            if (module_id == fw->module_id) {
                FTS_INFO("module id match, get fw file successfully");
                break;
            }
        }
        if (i >= FTS_GET_MODULE_ID_NUM) {
            FTS_ERROR("no module id match, don't get file");
            return -ENODATA;
        }    
    }

    fwupgrade->fw_info = fw;
    return ret;
}

static int fts_get_fw_file_via_request_firmware(void)
{

    int ret = 0;
    struct fts_upgrade *upg = fwupgrade;
    const struct firmware *fw = NULL;
    u8 *tmpbuf = NULL;
    struct device *dev = &fts_data->input_dev->dev;
    char fwname[FILE_NAME_LENGTH] = { 0 };

	FTS_INFO("get file via request_firmware");
    snprintf(fwname, FILE_NAME_LENGTH, "%s%s.bin", \
                 "focaltech_ts_fw_", upg->fw_info->module_vendor);
    
    ret = request_firmware(&fw, fwname, dev);
    if (0 == ret) {
        FTS_INFO("firmware request succeed");
        tmpbuf = vmalloc(fw->size);
        if (NULL == tmpbuf) {
            FTS_ERROR("fw buffer vmalloc fail");
            ret = -ENOMEM;
        } else {
            memcpy(tmpbuf, fw->data, fw->size);
            fwupgrade->fw= tmpbuf;
            fwupgrade->fw_length = fw->size;
            fwupgrade->fw_from_request = 1;
        }
    } else {
        FTS_INFO("%s:firmware request(%s) fail,ret=%d", __func__, fwname, ret);
    }

     if (fw != NULL) {
        release_firmware(fw);
        fw = NULL;
    }

    return ret;
}

static int fts_get_fw_file_via_i(void)
{
	FTS_INFO("get file via .i");
    fwupgrade->fw = fwupgrade->fw_info->fw_file;
    fwupgrade->fw_length= fwupgrade->fw_info->fw_len;
    fwupgrade->fw_from_request = 0;

    return 0;
}

static int fts_get_fw_file(bool fwrequest, int module_id)
{
    int ret = 0;
    struct fts_upgrade *upg = fwupgrade;
    bool get_fw_i_flag = 0;

    if (!upg) {
        FTS_ERROR("upg is null");
        return -EINVAL;
    }

    FTS_INFO("get firmware file:%d module_id:%d", fwrequest, module_id);
    ret = fts_get_module_info(module_id);
    if (ret < 0) {
        FTS_ERROR("module id not match");
        return ret;
    }
    
    if (fwrequest) {
        ret = fts_get_fw_file_via_request_firmware();
        if (ret != 0) {
            get_fw_i_flag = 1;
        }
    } else {
        get_fw_i_flag = 1;
    }

    if (get_fw_i_flag) {
        ret = fts_get_fw_file_via_i();
    }
   
    return ret;
}

static void fts_fwupg_work(struct work_struct *work)
{
    int ret = 0;
    u8 chip_id = 0;
    struct fts_upgrade *upg = fwupgrade;

    FTS_INFO("fw upgrade work function");
    if (!upg) {
        FTS_ERROR("upg is null");
        return ;
    }

    ret = fts_get_fw_file(1, 0);
    if (ret < 0) {
        FTS_ERROR("get fw file fail");
        return;
    }

    if (fts_data->fw_loading) {
        FTS_INFO("fw is loading, not download again");
        return ;
    }

    ret = fts_fw_download(upg->fw, upg->fw_length, true);
    if (ret < 0) {
        FTS_ERROR("fw auto download failed");
    } else {
        msleep(50);
        ret = fts_read_reg(FTS_REG_CHIP_ID, &chip_id);
        FTS_INFO("read chip id:0x%02x", chip_id);
    }
}

int fts_fw_init(void)
{
    FTS_INFO("fw upgrade init function");

    if (NULL == fts_data->ts_workqueue) {
        FTS_ERROR("fts workqueue is NULL, can't run upgrade function");
        return -EINVAL;
    }

    fwupgrade = (struct fts_upgrade *)kzalloc(sizeof(*fwupgrade), GFP_KERNEL);
    if (NULL == fwupgrade) {
        FTS_ERROR("malloc memory for upgrade fail");
        return -ENOMEM;
    }

    INIT_WORK(&fts_data->fwupg_work, fts_fwupg_work);
    queue_work(fts_data->ts_workqueue, &fts_data->fwupg_work);

    return 0;
}

int fts_fw_exit(void)
{
    FTS_FUNC_ENTER();
    if (fwupgrade->fw_from_request) {
        vfree(fwupgrade->fw);
        fwupgrade->fw = NULL;
    }
    
    if (fwupgrade) {
        kfree(fwupgrade);
        fwupgrade = NULL;
    }
    FTS_FUNC_EXIT();

    return 0;
}