/*
 *
 * FocalTech TouchScreen driver.
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
* File Name: Focaltech_ex_fun.c
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
/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
/*create apk debug channel*/
#define PROC_READ_REGISTER                      1
#define PROC_WRITE_REGISTER                     2
#define PROC_WRITE_DATA                         6
#define PROC_READ_DATA                          7
#define PROC_SET_TEST_FLAG                      8
#define PROC_HW_RESET                           11
#define PROC_READ_STATUS                        12
#define PROC_SET_BOOT_MODE                      13
#define PROC_ENTER_TEST_ENVIRONMENT             14
#define PROC_NAME                               "ftxxxx-debug"
#define PROC_BUF_SIZE                           256

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

/*****************************************************************************
* Static variables
*****************************************************************************/
enum {
    RWREG_OP_READ = 0,
    RWREG_OP_WRITE = 1,
};
static struct rwreg_operation_t {
    int type;         // 0: read, 1: write
    int reg;        // register
    int len;        // read/write length
    int val;      // length = 1; read: return value, write: op return
    int res;     // 0: success, otherwise: fail
    char *opbuf;        // length >= 1, read return value, write: op return
} rw_op;

struct fts_oppo_data _oppo_data;
/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/

/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
    u8 *writebuf = NULL;
    u8 tmpbuf[PROC_BUF_SIZE] = { 0 };
    int buflen = count;
    int writelen = 0;
    int ret = 0;
    char tmp[25];
    struct fts_ts_data *ts_data = fts_data;

    if ((buflen <= 0) || (buflen > PAGE_SIZE)) {
        FTS_ERROR("apk proc wirte count(%d>%d) fail", buflen, (int)PAGE_SIZE);
        return -EINVAL;
    }

    if (buflen > PROC_BUF_SIZE) {
        writebuf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
        if (NULL == writebuf) {
            FTS_ERROR("apk proc wirte buf zalloc fail");
            return -ENOMEM;
        }
    } else {
        writebuf = tmpbuf;
    }

    if (copy_from_user(writebuf, buff, buflen)) {
        FTS_ERROR("[APK]: copy from user error!!");
        ret = -EFAULT;
        goto proc_write_err;
    }

    ts_data->proc->opmode = writebuf[0];
    switch (ts_data->proc->opmode) {
    case PROC_SET_TEST_FLAG:
        FTS_DEBUG("[APK]: PROC_SET_TEST_FLAG = %x", writebuf[1]);
        if (writebuf[1] == 0) {
#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(ENABLE);
#endif
        } else {
#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(DISABLE);
#endif
        }
        break;

    case PROC_HW_RESET:
        sprintf(tmp, "%s", writebuf + 1);
        tmp[buflen - 1] = '\0';
        if (strncmp(tmp, "focal_driver", 12) == 0) {
            FTS_INFO("APK execute HW Reset");
            fts_reset_proc(0);
        }
        break;

    case PROC_READ_REGISTER:
        ts_data->proc->cmd[0] = writebuf[1];
        break;
    case PROC_WRITE_REGISTER:
        writelen = 2;
        ret = fts_write(writebuf + 1, writelen);
        if (ret < 0) {
            FTS_ERROR("PROC_WRITE_REGISTER write error");
            goto proc_write_err;
        }
        break;

    case PROC_READ_DATA:
        if (buflen - 1 > SPI_MAX_COMMAND_LENGTH) {
            FTS_ERROR("PROC_READ_DATA,buflen(%d) invalid", buflen);
            ret = -EINVAL;
            goto proc_write_err;
        } else {
            memcpy(ts_data->proc->cmd, writebuf + 1, buflen - 1);
            ts_data->proc->cmd_len = buflen - 1;
        }
        break;
    case PROC_WRITE_DATA:
        writelen = buflen - 1;
        if (writelen > 0) {
            ret = fts_write(writebuf + 1, writelen);
            if (ret < 0) {
                FTS_ERROR("PROC_WRITE_DATA write error");
                goto proc_write_err;
            }
        }
        break;
    case PROC_READ_STATUS:
        break;

    case PROC_SET_BOOT_MODE:
        FTS_DEBUG("[APK]: PROC_SET_BOOT_MODE = %x", writebuf[1]);
        if (0 == writebuf[1]) {
            ts_data->fw_is_running = true;
        } else {
            ts_data->fw_is_running = false;
        }
        break;
    case PROC_ENTER_TEST_ENVIRONMENT:
        FTS_DEBUG("[APK]: PROC_ENTER_TEST_ENVIRONMENT = %x", writebuf[1]);
        if (0 == writebuf[1]) {
            fts_fw_enter_test_environment(0);
        } else {
            fts_fw_enter_test_environment(1);
        }
        break;

    default:
        break;
    }

    ret = buflen;
proc_write_err:
    if ((buflen > PROC_BUF_SIZE) && writebuf) {
        kfree(writebuf);
        writebuf = NULL;
    }
    return ret;
}

/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    int num_read_chars = 0;
    int buflen = count;
    u8 *buf = NULL;
    u8 tmpbuf[PROC_BUF_SIZE] = { 0 };
    struct fts_ts_data *ts_data = fts_data;

    if ((buflen <= 0) || (buflen > PAGE_SIZE)) {
        FTS_ERROR("apk proc read count(%d>%d) fail", buflen, (int)PAGE_SIZE);
        return -EINVAL;
    }

    if (buflen > PROC_BUF_SIZE) {
        buf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
        if (NULL == buf) {
            FTS_ERROR("apk proc wirte buf zalloc fail");
            return -ENOMEM;
        }
    } else {
        buf = tmpbuf;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif

    switch (ts_data->proc->opmode) {
    case PROC_READ_REGISTER:
        num_read_chars = 1;
        ret = fts_read(ts_data->proc->cmd, 1, buf, num_read_chars);
        if (ret < 0) {
            FTS_ERROR("PROC_READ_REGISTER read error");
            goto proc_read_err;
        }
        break;
    case PROC_WRITE_REGISTER:
        break;

    case PROC_READ_DATA:
        num_read_chars = count;
        ret = fts_read(ts_data->proc->cmd, ts_data->proc->cmd_len, buf, num_read_chars);
        if (ret < 0) {
            FTS_ERROR("PROC_READ_DATA read error");
            goto proc_read_err;
        }
        break;
    case PROC_WRITE_DATA:
        break;

    case PROC_READ_STATUS:
        ret = fts_read_status(buf);
        if (ret < 0) {
            FTS_ERROR("read status error");
            goto proc_read_err;
        }
        num_read_chars = 1;
        break;
    default:
        break;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif

    if (copy_to_user(buff, buf, num_read_chars)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
        goto proc_read_err;
    }

    ret = num_read_chars;
proc_read_err:
    if ((buflen > PROC_BUF_SIZE) && buf) {
        kfree(buf);
        buf = NULL;
    }
    return ret;
}

static const struct file_operations fts_proc_fops = {
    .owner  = THIS_MODULE,
    .read   = fts_debug_read,
    .write  = fts_debug_write,
};

int fts_create_apk_debug_channel(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    ts_data->proc = kzalloc(sizeof(struct ftxxxx_proc), GFP_KERNEL);
    if (NULL == ts_data->proc) {
        FTS_ERROR("allocate memory for proc fail");
        return -ENOMEM;
    }
    ts_data->proc->proc = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);
    if (NULL == ts_data->proc->proc) {
        FTS_ERROR("create proc entry fail");
        kfree(ts_data->proc);
        ts_data->proc = NULL;
        return -ENOMEM;
    }

    FTS_FUNC_EXIT();
    return 0;
}

void fts_release_apk_debug_channel(struct fts_ts_data *ts_data)
{
    if (ts_data->proc) {
        if (ts_data->proc->proc) {
            proc_remove(ts_data->proc->proc);
        }

        kfree(ts_data->proc);
        ts_data->proc = NULL;
    }
}

/************************************************************************
 * sysfs interface
 ***********************************************************************/
/* fts_hw_reset interface */
static ssize_t fts_hw_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_hw_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct input_dev *input_dev = fts_data->input_dev;
    ssize_t count = 0;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    fts_reset_proc(0);
    count = snprintf(buf, PAGE_SIZE, "hw reset executed\n");
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

/* fts_irq interface */
static ssize_t fts_irq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        FTS_INFO("[EX-FUN]enable irq");
        fts_irq_enable();
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        FTS_INFO("[EX-FUN]disable irq");
        fts_irq_disable();
    }
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

static ssize_t fts_irq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EPERM;
}

/* fts_boot_mode interface */
static ssize_t fts_bootmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        FTS_INFO("[EX-FUN]set to boot mode");
        fts_data->fw_is_running = false;
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        FTS_INFO("[EX-FUN]set to fw mode");
        fts_data->fw_is_running = true;
    }
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

static ssize_t fts_bootmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t count = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    if (true == fts_data->fw_is_running) {
        count = snprintf(buf, PAGE_SIZE, "tp is in fw mode\n");
    } else {
        count = snprintf(buf, PAGE_SIZE, "tp is in boot mode\n");
    }
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

/* fts_tpfwver interface */
static ssize_t fts_tpfwver_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;
    ssize_t num_read_chars = 0;
    u8 fwver = 0;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif
    if (fts_read_reg(FTS_REG_FW_VER, &fwver) < 0) {
        num_read_chars = snprintf(buf, PAGE_SIZE, "SPI transfer error!\n");
    }
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif
    if ((fwver == 0xFF) || (fwver == 0x00))
        num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
    else
        num_read_chars = snprintf(buf, PAGE_SIZE, "%02x\n", fwver);

    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return num_read_chars;
}

/* fts_rw_reg */
static ssize_t fts_tprwreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    int i;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);

    if (rw_op.len < 0) {
        count = snprintf(buf, PAGE_SIZE, "Invalid cmd line\n");
    } else if (rw_op.len == 1) {
        if (RWREG_OP_READ == rw_op.type) {
            if (rw_op.res == 0) {
                count = snprintf(buf, PAGE_SIZE, "Read %02X: %02X\n", rw_op.reg, rw_op.val);
            } else {
                count = snprintf(buf, PAGE_SIZE, "Read %02X failed, ret: %d\n", rw_op.reg,  rw_op.res);
            }
        } else {
            if (rw_op.res == 0) {
                count = snprintf(buf, PAGE_SIZE, "Write %02X, %02X success\n", rw_op.reg,  rw_op.val);
            } else {
                count = snprintf(buf, PAGE_SIZE, "Write %02X failed, ret: %d\n", rw_op.reg,  rw_op.res);
            }
        }
    } else {
        if (RWREG_OP_READ == rw_op.type) {
            count = snprintf(buf, PAGE_SIZE, "Read Reg: [%02X]-[%02X]\n", rw_op.reg, rw_op.reg + rw_op.len);
            count += snprintf(buf + count, PAGE_SIZE, "Result: ");
            if (rw_op.res) {
                count += snprintf(buf + count, PAGE_SIZE, "failed, ret: %d\n", rw_op.res);
            } else {
                if (rw_op.opbuf) {
                    for (i = 0; i < rw_op.len; i++) {
                        count += snprintf(buf + count, PAGE_SIZE, "%02X ", rw_op.opbuf[i]);
                    }
                    count += snprintf(buf + count, PAGE_SIZE, "\n");
                }
            }
        } else {
            ;
            count = snprintf(buf, PAGE_SIZE, "Write Reg: [%02X]-[%02X]\n", rw_op.reg, rw_op.reg + rw_op.len - 1);
            count += snprintf(buf + count, PAGE_SIZE, "Write Data: ");
            if (rw_op.opbuf) {
                for (i = 1; i < rw_op.len; i++) {
                    count += snprintf(buf + count, PAGE_SIZE, "%02X ", rw_op.opbuf[i]);
                }
                count += snprintf(buf + count, PAGE_SIZE, "\n");
            }
            if (rw_op.res) {
                count += snprintf(buf + count, PAGE_SIZE, "Result: failed, ret: %d\n", rw_op.res);
            } else {
                count += snprintf(buf + count, PAGE_SIZE, "Result: success\n");
            }
        }
        /*if (rw_op.opbuf) {
            kfree(rw_op.opbuf);
            rw_op.opbuf = NULL;
        }*/
    }
    mutex_unlock(&input_dev->mutex);

    return count;
}

static int shex_to_int(const char *hex_buf, int size)
{
    int i;
    int base = 1;
    int value = 0;
    char single;

    for (i = size - 1; i >= 0; i--) {
        single = hex_buf[i];

        if ((single >= '0') && (single <= '9')) {
            value += (single - '0') * base;
        } else if ((single >= 'a') && (single <= 'z')) {
            value += (single - 'a' + 10) * base;
        } else if ((single >= 'A') && (single <= 'Z')) {
            value += (single - 'A' + 10) * base;
        } else {
            return -EINVAL;
        }

        base *= 16;
    }

    return value;
}


static u8 shex_to_u8(const char *hex_buf, int size)
{
    return (u8)shex_to_int(hex_buf, size);
}
/*
 * Format buf:
 * [0]: '0' write, '1' read(reserved)
 * [1-2]: addr, hex
 * [3-4]: length, hex
 * [5-6]...[n-(n+1)]: data, hex
 */
static int fts_parse_buf(const char *buf, size_t cmd_len)
{
    int length;
    int i;
    char *tmpbuf;

    rw_op.reg = shex_to_u8(buf + 1, 2);
    length = shex_to_int(buf + 3, 2);

    if (buf[0] == '1') {
        rw_op.len = length;
        rw_op.type = RWREG_OP_READ;
        FTS_DEBUG("read %02X, %d bytes", rw_op.reg, rw_op.len);
    } else {
        if (cmd_len < (length * 2 + 5)) {
            pr_err("data invalided!\n");
            return -EINVAL;
        }
        FTS_DEBUG("write %02X, %d bytes", rw_op.reg, length);

        /* first byte is the register addr */
        rw_op.type = RWREG_OP_WRITE;
        rw_op.len = length + 1;
    }

    if (rw_op.len > 0) {
        tmpbuf = (char *)kzalloc(rw_op.len, GFP_KERNEL);
        if (!tmpbuf) {
            FTS_ERROR("allocate memory failed!\n");
            return -ENOMEM;
        }

        if (RWREG_OP_WRITE == rw_op.type) {
            tmpbuf[0] = rw_op.reg & 0xFF;
            FTS_DEBUG("write buffer: ");
            for (i = 1; i < rw_op.len; i++) {
                tmpbuf[i] = shex_to_u8(buf + 5 + i * 2 - 2, 2);
                FTS_DEBUG("buf[%d]: %02X", i, tmpbuf[i] & 0xFF);
            }
        }
        rw_op.opbuf = tmpbuf;
    }

    return rw_op.len;
}



/************************************************************************
* Name: fts_tprwreg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_tprwreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;
    ssize_t cmd_length = 0;

    mutex_lock(&input_dev->mutex);
    cmd_length = count - 1;

    if (rw_op.opbuf) {
        kfree(rw_op.opbuf);
        rw_op.opbuf = NULL;
    }

    FTS_DEBUG("cmd len: %d, buf: %s", (int)cmd_length, buf);
    /* compatible old ops */
    if (2 == cmd_length) {
        rw_op.type = RWREG_OP_READ;
        rw_op.len = 1;
        rw_op.reg = shex_to_int(buf, 2);
    } else if (4 == cmd_length) {
        rw_op.type = RWREG_OP_WRITE;
        rw_op.len = 1;
        rw_op.reg = shex_to_int(buf, 2);
        rw_op.val = shex_to_int(buf + 2, 2);
    } else if (cmd_length < 5) {
        FTS_ERROR("Invalid cmd buffer");
        mutex_unlock(&input_dev->mutex);
        return -EINVAL;
    } else {
        rw_op.len = fts_parse_buf(buf, cmd_length);
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif
    if (rw_op.len < 0) {
        FTS_ERROR("cmd buffer error!");

    } else {
        if (RWREG_OP_READ == rw_op.type) {
            if (rw_op.len == 1) {
                u8 reg, val;
                reg = rw_op.reg & 0xFF;
                rw_op.res = fts_read_reg(reg, &val);
                rw_op.val = val;
            } else {
                char reg;
                reg = rw_op.reg & 0xFF;

                rw_op.res = fts_read(&reg, 1, rw_op.opbuf, rw_op.len);
            }

            if (rw_op.res < 0) {
                FTS_ERROR("Could not read 0x%02x", rw_op.reg);
            } else {
                FTS_INFO("read 0x%02x, %d bytes successful", rw_op.reg, rw_op.len);
                rw_op.res = 0;
            }

        } else {
            if (rw_op.len == 1) {
                u8 reg, val;
                reg = rw_op.reg & 0xFF;
                val = rw_op.val & 0xFF;
                rw_op.res = fts_write_reg(reg, val);
            } else {
                rw_op.res = fts_write(rw_op.opbuf, rw_op.len);
            }
            if (rw_op.res < 0) {
                FTS_ERROR("Could not write 0x%02x", rw_op.reg);

            } else {
                FTS_INFO("Write 0x%02x, %d bytes successful", rw_op.val, rw_op.len);
                rw_op.res = 0;
            }
        }
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif
    mutex_unlock(&input_dev->mutex);

    return count;
}

/* fts_upgrade_bin interface */
static ssize_t fts_fwupgradebin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char fwname[FILE_NAME_LENGTH] = { 0 };
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;

    if ((count <= 1) || (count >= FILE_NAME_LENGTH - 32)) {
        FTS_ERROR("fw bin name's length(%d) fail", (int)count);
        return -EINVAL;
    }
    memset(fwname, 0, sizeof(fwname));
    sprintf(fwname, "%s", buf);
    fwname[count - 1] = '\0';

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    FTS_INFO("upgrade with bin file through sysfs node");
    fts_upgrade_bin(fwname, 0);
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_fwupgradebin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EPERM;
}

/* fts_force_upgrade interface */
static ssize_t fts_fwforceupg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_fwforceupg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EPERM;
}

/* fts_driver_version interface */
static ssize_t fts_driverversion_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_driverversion_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    count = sprintf(buf, FTS_DRIVER_VERSION "\n");
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

/* fts_dump_reg interface */
static ssize_t fts_dumpreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_dumpreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif
    fts_read_reg(FTS_REG_POWER_MODE, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Power Mode:0x%02x\n", val);

    fts_read_reg( FTS_REG_FW_VER, &val);
    count += snprintf(buf + count, PAGE_SIZE, "FW Ver:0x%02x\n", val);

    fts_read_reg( FTS_REG_LIC_VER, &val);
    count += snprintf(buf + count, PAGE_SIZE, "LCD Initcode Ver:0x%02x\n", val);

    fts_read_reg( FTS_REG_IDE_PARA_VER_ID, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Param Ver:0x%02x\n", val);

    fts_read_reg( FTS_REG_IDE_PARA_STATUS, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Param status:0x%02x\n", val);

    fts_read_reg( FTS_REG_VENDOR_ID, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Vendor ID:0x%02x\n", val);

    fts_read_reg( FTS_REG_LCD_BUSY_NUM, &val);
    count += snprintf(buf + count, PAGE_SIZE, "LCD Busy Number:0x%02x\n", val);

    fts_read_reg( FTS_REG_GESTURE_EN, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture Mode:0x%02x\n", val);

    fts_read_reg( FTS_REG_CHARGER_MODE_EN, &val);
    count += snprintf(buf + count, PAGE_SIZE, "charge stat:0x%02x\n", val);

    fts_read_reg( FTS_REG_INT_CNT, &val);
    count += snprintf(buf + count, PAGE_SIZE, "INT count:0x%02x\n", val);

    fts_read_reg( FTS_REG_FLOW_WORK_CNT, &val);
    count += snprintf(buf + count, PAGE_SIZE, "ESD count:0x%02x\n", val);
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif

    mutex_unlock(&input_dev->mutex);

    return count;
}

/* fts_dump_reg interface */
static ssize_t fts_tpbuf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_tpbuf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    int i = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    count += snprintf(buf + count, PAGE_SIZE, "touch point buffer:\n");
    for (i = 0; i < fts_data->pnt_buf_size; i++) {
        count += snprintf(buf + count, PAGE_SIZE, "%02x ", fts_data->point_buf[i]);
    }
    count += snprintf(buf + count, PAGE_SIZE, "\n");
    mutex_unlock(&input_dev->mutex);

    return count;
}

/* get the fw version  example:cat fw_version */
static DEVICE_ATTR(fts_fw_version, S_IRUGO | S_IWUSR, fts_tpfwver_show, fts_tpfwver_store);

/* read and write register(s)
*   All data type is **HEX**
*   Single Byte:
*       read:   echo 88 > rw_reg ---read register 0x88
*       write:  echo 8807 > rw_reg ---write 0x07 into register 0x88
*   Multi-bytes:
*       [0:rw-flag][1-2: reg addr, hex][3-4: length, hex][5-6...n-n+1: write data, hex]
*       rw-flag: 0, write; 1, read
*       read:  echo 10005           > rw_reg ---read reg 0x00-0x05
*       write: echo 000050102030405 > rw_reg ---write reg 0x00-0x05 as 01,02,03,04,05
*  Get result:
*       cat rw_reg
*/

static DEVICE_ATTR(fts_rw_reg, S_IRUGO | S_IWUSR, fts_tprwreg_show, fts_tprwreg_store);
/*  upgrade from fw bin file   example:echo "*.bin" > fts_upgrade_bin */
static DEVICE_ATTR(fts_upgrade_bin, S_IRUGO | S_IWUSR, fts_fwupgradebin_show, fts_fwupgradebin_store);
static DEVICE_ATTR(fts_force_upgrade, S_IRUGO | S_IWUSR, fts_fwforceupg_show, fts_fwforceupg_store);
static DEVICE_ATTR(fts_driver_version, S_IRUGO | S_IWUSR, fts_driverversion_show, fts_driverversion_store);
static DEVICE_ATTR(fts_dump_reg, S_IRUGO | S_IWUSR, fts_dumpreg_show, fts_dumpreg_store);
static DEVICE_ATTR(fts_hw_reset, S_IRUGO | S_IWUSR, fts_hw_reset_show, fts_hw_reset_store);
static DEVICE_ATTR(fts_irq, S_IRUGO | S_IWUSR, fts_irq_show, fts_irq_store);
static DEVICE_ATTR(fts_boot_mode, S_IRUGO | S_IWUSR, fts_bootmode_show, fts_bootmode_store);
static DEVICE_ATTR(fts_touch_point, S_IRUGO | S_IWUSR, fts_tpbuf_show, fts_tpbuf_store);

/* add your attr in here*/
static struct attribute *fts_attributes[] = {
    &dev_attr_fts_fw_version.attr,
    &dev_attr_fts_rw_reg.attr,
    &dev_attr_fts_dump_reg.attr,
    &dev_attr_fts_upgrade_bin.attr,
    &dev_attr_fts_force_upgrade.attr,
    &dev_attr_fts_driver_version.attr,
    &dev_attr_fts_hw_reset.attr,
    &dev_attr_fts_irq.attr,
    &dev_attr_fts_boot_mode.attr,
    &dev_attr_fts_touch_point.attr,
    NULL
};

static struct attribute_group fts_attribute_group = {
    .attrs = fts_attributes
};

int fts_create_sysfs(struct device *dev)
{
    int ret = 0;

    ret = sysfs_create_group(&dev->kobj, &fts_attribute_group);
    if (ret) {
        FTS_ERROR("sysfs_create_group() fail");
        sysfs_remove_group(&dev->kobj, &fts_attribute_group);
        return -ENOMEM;
    } else {
        FTS_INFO("sysfs_create_group() success");
    }

    return ret;
}

int fts_remove_sysfs(struct device *dev)
{
    sysfs_remove_group(&dev->kobj, &fts_attribute_group);
    return 0;
}

/* baseline_test */
static ssize_t fts_oppo_baseline_test_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int test_result = 0;
	int len = 0;
	char result_pass[40] = "0 error(s),All test passed";
	char result_fail[20] = "MP test fail";
	char tmpbuf[40] = { 0 };

	FTS_FUNC_ENTER();
	if (*ppos != 0)
		return 0;

	ret = fts_oppo_test(0, &test_result);
	if ((ret >= 0) && (0 == test_result)) {
		len = snprintf(tmpbuf, count,"%s\n", result_pass);
	} else {
		len = snprintf(tmpbuf, count,"%s\n",result_fail);
	}

	FTS_DEBUG("write to user len:%d", len);
	ret = copy_to_user(buf, tmpbuf, len);
	if (ret < 0) {
	  	FTS_ERROR("copy data to user space fail");
	} else {
		*ppos += len;
		ret = len;
	}

	FTS_FUNC_EXIT();
	return len;
}


/* black_screen_test */
extern int g_gesture;
static ssize_t fts_oppo_black_screen_test_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	int value = 0;
	char tmpbuf[64] = {0};

	if (count > 64) {
		FTS_ERROR("count(%d)>64", (int)count);
		return -EINVAL;
	}

	if (copy_from_user(tmpbuf, buf, count)) {
		printk("%s: copy from user error.", __func__);
		return -EIO;
	}
	
	sscanf(tmpbuf, "%d", &value);
	FTS_INFO("black_screen write:%d", value);
	fts_data->oppo_vendor_data->gesture_backup = fts_data->gesture->mode;
	fts_data->gesture->mode = 1;
	fts_data->oppo_vendor_data->lcm_gesture_power = g_gesture;
	g_gesture = 1;
	fts_data->oppo_vendor_data->gesture_test_flag = value;
	
	return count;
}

/* baseline_test */
static ssize_t fts_oppo_black_screen_test_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int test_result = 0;
	int len = 0;
	char result_pass[40] = "0 error(s),All test passed";
	char result_fail[20] = "MP test fail";
	char tmpbuf[40] = { 0 };

	FTS_FUNC_ENTER();
	if (*ppos != 0)
		return 0;

	if (!fts_data->oppo_vendor_data->gesture_test_flag) {
		FTS_ERROR("gesture_test_flag is 0");
		ret = -EINVAL;
		goto proc_err;
	}

	if (!fts_data->gesture->mode || !fts_data->suspended) {
		FTS_ERROR("gesture is disable/unsuspend,no test");
		ret = -EINVAL;
		goto proc_err;
	}
	
	ret = fts_oppo_test(1, &test_result);
	if ((ret >= 0) && (0 == test_result)) {
		len = snprintf(tmpbuf, count,"%s\n", result_pass);
	} else {
		len = snprintf(tmpbuf, count,"%s\n",result_fail);
	}
	FTS_DEBUG("write to user len:%d", len);
	ret = copy_to_user(buf, tmpbuf, len);
	if (ret < 0) {
	  	FTS_ERROR("copy data to user space fail");
	} else {
		*ppos += len;
		ret = len;
	}

proc_err:
	g_gesture = fts_data->oppo_vendor_data->lcm_gesture_power;
	fts_data->oppo_vendor_data->gesture_test_flag = 0;
	fts_data->gesture->mode = fts_data->oppo_vendor_data->gesture_backup;

	FTS_FUNC_EXIT();
	return ret;
}

/* debug_info/delta */
static ssize_t fts_oppo_delta_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char *tmpbuf = NULL;

	FTS_FUNC_ENTER();
	FTS_INFO("count:%d,ppos:%d", (int)count, (int)(*ppos));
	if (*ppos != 0) {
		FTS_INFO("is already read the file\n");
		return 0;
	}
	
#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
    fts_esdcheck_switch(DISABLE);
#endif	
	len = count;
	tmpbuf = (char*)kzalloc(count, GFP_KERNEL);
	ret = fts_oppo_get_rawdata_diff(1, tmpbuf, &len);
	if (ret < 0) {
		FTS_ERROR("oppo get rawdata diff fail");
		len = ret;
		goto proc_err;
	}

	ret = copy_to_user(buf, tmpbuf, len);
	if (ret < 0) {
		FTS_ERROR("copy data to user space fail");
		goto proc_err;
	}

	*ppos += len;
	ret = len;

proc_err:
	if (tmpbuf) {
		kfree(tmpbuf);
		tmpbuf = NULL;
	}
#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
		fts_esdcheck_switch(ENABLE);
#endif

	FTS_FUNC_EXIT();
	return ret;
}

/* debug_info/baseline */
static ssize_t fts_oppo_baseline_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char *tmpbuf = NULL;

	FTS_FUNC_ENTER();
	FTS_INFO("count:%d,ppos:%d", (int)count, (int)(*ppos));
	if (*ppos != 0) {
		FTS_INFO("is already read the file\n");
		return 0;
	}
	len = count;
	tmpbuf = (char*)kzalloc(count, GFP_KERNEL);
	if (NULL == tmpbuf) {
		FTS_ERROR("tmpbuf zalloc fail");
		return -ENOMEM;
	}
	
#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
    fts_esdcheck_switch(DISABLE);
#endif	
	
	ret = fts_oppo_get_rawdata_diff(0, tmpbuf, &len);
	if (ret < 0) {
		FTS_ERROR("oppo get rawdata diff fail");
		len = ret;
		goto proc_err;
	}

	ret = copy_to_user(buf, tmpbuf, len);
	if (ret < 0) {
	  	FTS_ERROR("copy data to user space fail");
		goto proc_err;
	}
	*ppos += len;
	ret = len;

proc_err:
	if (tmpbuf) {
		kfree(tmpbuf);
		tmpbuf = NULL;
	}
#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
		fts_esdcheck_switch(ENABLE);
#endif

	FTS_FUNC_EXIT();
	return ret;
}

/* debug_info/main_register */
static ssize_t fts_oppo_main_register_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int i = 0;
	u8 data[128] = { 0 };
	int len = 0;
	int tmplen = 0;
	char *tmpbuf = NULL;

	if (*ppos != 0) {
		return 0;
	}

	len = count;
	tmpbuf = (char*)kzalloc(count, GFP_KERNEL);
	if (NULL == tmpbuf) {
		FTS_ERROR("tmpbuf zalloc fail");
		return -ENOMEM;
	}
	
#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
    fts_esdcheck_switch(DISABLE);
#endif

	buf[0] = 0;
	len = 128;
	fts_read(data, 1, data, len);
	for (i = 0; i < len; i++) {
		tmplen += snprintf(tmpbuf + tmplen, count - tmplen, "%x,", data[i]);
	}
	tmplen += snprintf(tmpbuf + tmplen, count - tmplen, "\n");
	ret = copy_to_user(buf, tmpbuf, tmplen);
	if (ret < 0) {
	  	FTS_ERROR("copy data to user space fail");
	} else {
		*ppos += tmplen;
		ret = tmplen;
	}	

#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
			fts_esdcheck_switch(ENABLE);
#endif

	kfree_safe(tmpbuf);

	return ret;

}


static ssize_t fts_oppo_coordinate_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int tmplen = 0;
	char *tmpbuf = NULL;
	struct fts_oppo_gesture *oppo_gesture = &fts_data->oppo_vendor_data->gesture_data;

	if (*ppos != 0) {
		return 0;
	}

	tmpbuf = (char*)kzalloc(count, GFP_KERNEL);
	if (NULL == tmpbuf) {
		FTS_ERROR("tmpbuf zalloc fail");
		return -ENOMEM;
	}

	tmplen += snprintf(tmpbuf, count, 
					 "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n",
					 oppo_gesture->gesture_id,
					 oppo_gesture->start_x, oppo_gesture->start_y,
					 oppo_gesture->end_x, oppo_gesture->end_y,
					 oppo_gesture->point1_x, oppo_gesture->point1_y,
					 oppo_gesture->point2_x, oppo_gesture->point2_y,
					 oppo_gesture->point3_x, oppo_gesture->point3_y,
					 oppo_gesture->point4_x, oppo_gesture->point4_y,
					 oppo_gesture->clock_wise
					 );
	
	ret = copy_to_user(buf, tmpbuf, tmplen);
	if (ret < 0) {
	  	FTS_ERROR("copy data to user space fail");
	} else {
		*ppos += tmplen;
		ret = tmplen;
	}
	kfree_safe(tmpbuf);
	return ret;
}

/* irq_depth */
static ssize_t fts_oppo_irq_depth_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int tmplen = 0;
	char *tmpbuf = NULL;
	struct irq_desc *desc = irq_to_desc(fts_data->irq);

	if (*ppos != 0) {
		return 0;
	}

	tmpbuf = (char*)kzalloc(count, GFP_KERNEL);
	if (NULL == tmpbuf) {
		FTS_ERROR("tmpbuf zalloc fail");
		return -ENOMEM;
	}

	tmplen = snprintf(tmpbuf, count, "now depth=%d\n", desc->depth);
	
	ret = copy_to_user(buf, tmpbuf, tmplen);
	if (ret < 0) {
	  	FTS_ERROR("copy data to user space fail");
	} else {
		*ppos += tmplen;
		ret = tmplen;
	}
	kfree_safe(tmpbuf);
	return ret;
}

/* debug_level */
static ssize_t fts_oppo_debug_level_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	int value = 0;
	char tmpbuf[64] = {0};

	if (count > 64) {
		FTS_ERROR("count(%d)>64", (int)count);
		return -EINVAL;
	}

	if (copy_from_user(tmpbuf, buf, count)) {
		printk("%s: copy from user error.", __func__);
		return -EIO;
	}
	
	sscanf(tmpbuf, "%d", &value);
	fts_oppo_debug_level = value;
	FTS_INFO("oppo debug level:%d", value);
	
	return count;
}

/* tp_fw_update */
static ssize_t fts_oppo_tp_fw_update_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	int value = 0;
	char tmpbuf[64] = {0};

	if (count > 64) {
		FTS_ERROR("count(%d)>64", (int)count);
		return -EINVAL;
	}

	if (copy_from_user(tmpbuf, buf, count)) {
		printk("%s: copy from user error.", __func__);
		return -EIO;
	}
	
	sscanf(tmpbuf, "%d", &value);
	FTS_INFO("tp_fw_update:%d", value);
	if (0 == value ) {
		fts_upgrade_oppo(0);
	}
	
	return count;
}

/* oppo_register_info */
static ssize_t fts_oppo_register_info_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	char tmpbuf[64] = {0};
	char *ptr = NULL;

	if (count > 64) {
		ptr = (char *)kzalloc(count, GFP_KERNEL);
		if (NULL == ptr) {
			FTS_ERROR("ptr malloc fail");
			return -ENOMEM;
		}
	} else {
		ptr = tmpbuf;
	}

	if (copy_from_user(ptr, buf, count)) {
		printk("%s: copy from user error.", __func__);
		return -EIO;
	}

	fts_tprwreg_store(NULL, NULL, ptr, count);

	if ((count > 64) && (ptr)) {	
		kfree(ptr);
		ptr = NULL;
	}
	return count;
}

/* baseline_test */
static ssize_t fts_oppo_register_info_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int tmplen = 0;
	char *tmpbuf = NULL;

	if (*ppos != 0) {
		return 0;
	}

	tmpbuf = (char*)kzalloc(count, GFP_KERNEL);
	if (NULL == tmpbuf) {
		FTS_ERROR("tmpbuf zalloc fail");
		return -ENOMEM;
	}

	ret = fts_tprwreg_show(NULL, NULL, tmpbuf);
	if (ret >= 0) {
		tmplen = ret;
		ret = copy_to_user(buf, tmpbuf, tmplen);
		if (ret < 0) {
			FTS_ERROR("copy data to user space fail");
		} else {
			*ppos += tmplen;
			ret = tmplen;
		}
	}
	kfree_safe(tmpbuf);
	return ret;
}


/* double_tap_enable */
static ssize_t fts_oppo_gesture_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int value = 0;
	char tmpbuf[64] = {0};
	

	if (count > 64) {
		FTS_ERROR("count(%d)>64", (int)count);
		return -EINVAL;
	}

	if (copy_from_user(tmpbuf, buf, count)) {
		printk("%s: copy from user error.", __func__);
		return -EIO;
	}
	
	sscanf(tmpbuf, "%d", &value);
	FTS_INFO("gesture write:%d", value);
	if (1 == value) {
		if (fts_data->suspended) {
			FTS_INFO("gesture enable in p-sensor(far away) mode");
			fts_data->gesture->mode = ENABLE;
			g_gesture = 1;
			fts_reset_proc(50);
			fts_tp_state_recovery();
		} else {
			FTS_INFO("gesture enable in normal mode");
			fts_data->gesture->mode = ENABLE;
			g_gesture = 1;
		}
	} else if (0 == value) {
		FTS_INFO("gesture disable in normal mode");
		fts_data->gesture->mode = DISABLE;
		g_gesture = 0;
	} else if (2 == value) {
		if (fts_data->suspended) {
			FTS_INFO("gesture disable in p-sensor(close) mode");
			fts_data->gesture->mode = DISABLE;
			g_gesture = 1;
			ret = fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
			if (ret < 0) {
				FTS_ERROR("tp enter into sleep mode fail");
			}
		}
	}
	
	return count;
}

static ssize_t fts_oppo_gesture_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int tmplen = 0;
	char *tmpbuf = NULL;

	if (*ppos != 0) {
		return 0;
	}

	tmpbuf = (char*)kzalloc(count, GFP_KERNEL);
	if (NULL == tmpbuf) {
		FTS_ERROR("tmpbuf zalloc fail");
		return -ENOMEM;
	}

	tmplen += snprintf(tmpbuf, count, "gesture_enable=%d\n", 
					 fts_data->gesture->mode);
	
	ret = copy_to_user(buf, tmpbuf, tmplen);
	if (ret < 0) {
	  	FTS_ERROR("copy data to user space fail");
	} else {
		*ppos += tmplen;
		ret = tmplen;
	}
	kfree_safe(tmpbuf);
	return ret;
}

/* tp_limit_enable */
static ssize_t fts_oppo_limit_enable_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	int value = 0;
	char tmpbuf[64] = {0};
	struct _tp_limit *tp_limit = &fts_data->oppo_vendor_data->tp_limit;

	if (count > 64) {
		FTS_ERROR("count(%d)>64", (int)count);
		return -EINVAL;
	}

	if (copy_from_user(tmpbuf, buf, count)) {
		printk("%s: copy from user error.", __func__);
		return -EIO;
	}
	
	sscanf(tmpbuf, "%x", &value);
	FTS_INFO("limit enable write:0x%x", value);
	if (value > 0x1F) {
		FTS_ERROR("limit enable write(%x) fail", value);
		return -EINVAL;
	}

	tp_limit->enable.value = value;
	FTS_DEBUG("oppo limit enable:%d,%d,%d,%d,%d", 
			  tp_limit->enable.bit.edge_limit,
			  tp_limit->enable.bit.lu,
			  tp_limit->enable.bit.ru,
			  tp_limit->enable.bit.lb,
			  tp_limit->enable.bit.rb);

	if (!fts_data->suspended) {
		fts_mode_switch(MODE_EDGE_ENABLE, tp_limit->enable.bit.edge_limit);
	}
	
	return count;
}

static ssize_t fts_oppo_limit_enable_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int tmplen = 0;
	char *tmpbuf = NULL;

	if (*ppos != 0) {
		return 0;
	}

	tmpbuf = (char*)kzalloc(count, GFP_KERNEL);
	if (NULL == tmpbuf) {
		FTS_ERROR("tmpbuf zalloc fail");
		return -ENOMEM;
	}

	tmplen += snprintf(tmpbuf, count, "limit enable=0x%x\n", 
					 fts_data->oppo_vendor_data->tp_limit.enable.value);
	
	ret = copy_to_user(buf, tmpbuf, tmplen);
	if (ret < 0) {
	  	FTS_ERROR("copy data to user space fail");
	} else {
		*ppos += tmplen;
		ret = tmplen;
	}
	kfree_safe(tmpbuf);
	return ret;
}

/* tp_limit_area */
static ssize_t fts_oppo_limit_area_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	int value = 0;
	char tmpbuf[64] = {0};
	struct _tp_limit *tp_limit = &fts_data->oppo_vendor_data->tp_limit;

	if (count > 64) {
		FTS_ERROR("count(%d)>64", (int)count);
		return -EINVAL;
	}

	if (copy_from_user(tmpbuf, buf, count)) {
		printk("%s: copy from user error.", __func__);
		return -EIO;
	}
	
	sscanf(tmpbuf, "%x", &value);
	FTS_INFO("limit enable write:0x%x", value);
	if ((value < 0) || (value > 10)) {
		FTS_ERROR("limit area write(%x) fail", value);
		return -EINVAL;
	}

	tp_limit->xlu = (value * 1000) / 100;
	tp_limit->xru = FTS_TOUCH_MAX_WIDTH - tp_limit->xlu;
	tp_limit->xlb = tp_limit->xlu * 2;
	tp_limit->xrb = FTS_TOUCH_MAX_WIDTH - tp_limit->xlb;

	tp_limit->ylu = (value * 1000) / 100;
	tp_limit->yru = tp_limit->ylu;
	tp_limit->ylb = FTS_TOUCH_MAX_HEIGHT - (tp_limit->ylu * 2);
	tp_limit->yrb = FTS_TOUCH_MAX_HEIGHT - (tp_limit->ylu * 2);
	
	return count;
}

static ssize_t fts_oppo_limit_area_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret = 0;
	int tmplen = 0;
	char *tmpbuf = NULL;
	struct _tp_limit *tp_limit = &fts_data->oppo_vendor_data->tp_limit;
	

	if (*ppos != 0) {
		return 0;
	}

	tmpbuf = (char*)kzalloc(count, GFP_KERNEL);
	if (NULL == tmpbuf) {
		FTS_ERROR("tmpbuf zalloc fail");
		return -ENOMEM;
	}

	tmplen += snprintf(tmpbuf + tmplen, count - tmplen, 
					  "limit area left up=(%d,%d)\n", 
					  tp_limit->xlu, tp_limit->ylu);
	tmplen += snprintf(tmpbuf + tmplen, count - tmplen, 
					  "limit area right up=(%d,%d)\n", 
					  tp_limit->xru, tp_limit->yru);
	tmplen += snprintf(tmpbuf + tmplen, count - tmplen, 
					  "limit area left bottom=(%d,%d)\n", 
					  tp_limit->xlb, tp_limit->ylb);
	tmplen += snprintf(tmpbuf + tmplen, count - tmplen, 
					  "limit area right bottom=(%d,%d)\n", 
					  tp_limit->xrb, tp_limit->yrb);
	
	ret = copy_to_user(buf, tmpbuf, tmplen);
	if (ret < 0) {
	  	FTS_ERROR("copy data to user space fail");
	} else {
		*ppos += tmplen;
		ret = tmplen;
	}
	kfree_safe(tmpbuf);
	return ret;
}


static ssize_t oppo_gesture_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	//unsigned int tmp = 0;
	//char cmd[5] = {0};
	//int len = 0;

	return 0;

}

static ssize_t oppo_gesture_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{

	return 0;

}

static const struct file_operations baseline_test_fops =
{
	.read = fts_oppo_baseline_test_read,
	.owner = THIS_MODULE,
};
static const struct file_operations coordinate_fops =
{
	.read = fts_oppo_coordinate_read,
	.owner = THIS_MODULE,
};
static const struct file_operations debug_level_fops =
{
	.write = fts_oppo_debug_level_write,
	.owner = THIS_MODULE,
};
static const struct file_operations double_tap_enable_fops =
{
	.write = fts_oppo_gesture_write,
	.read = fts_oppo_gesture_read,
	.owner = THIS_MODULE,
};
static const struct file_operations irq_depth_fops =
{
	.read = fts_oppo_irq_depth_read,
	.owner = THIS_MODULE,
};
static const struct file_operations oppo_register_info_fops =
{
	.write = fts_oppo_register_info_write,
	.read = fts_oppo_register_info_read,
	.owner = THIS_MODULE,
};
static const struct file_operations oppo_tp_limit_area_fops =
{
	.write = fts_oppo_limit_area_write,
	.read = fts_oppo_limit_area_read,
	.owner = THIS_MODULE,
};
static const struct file_operations oppo_tp_limit_enable_fops =
{
	.write = fts_oppo_limit_enable_write,
	.read = fts_oppo_limit_enable_read,
	.owner = THIS_MODULE,
};
static const struct file_operations tp_fw_update_fops =
{
	.write = fts_oppo_tp_fw_update_write,
	.owner = THIS_MODULE,
};

static const struct file_operations delta_fops =
{
	.read = fts_oppo_delta_read,
	.owner = THIS_MODULE,
};

static const struct file_operations baseline_fops =
{
	.read = fts_oppo_baseline_read,
	.owner = THIS_MODULE,
};

static const struct file_operations main_register_fops =
{
	.read = fts_oppo_main_register_read,
	.owner = THIS_MODULE,
};

static const struct file_operations openshort_fops =
{
	.write = oppo_gesture_write,
	.read = oppo_gesture_read,
	.owner = THIS_MODULE,
};
static const struct file_operations black_screen_test_fops =
{
	.write = fts_oppo_black_screen_test_write,
	.read = fts_oppo_black_screen_test_read,
	.owner = THIS_MODULE,
};

int fts_proc_init(struct fts_ts_data *ts_data)
{

#define FTS_MK_PROC_DIR(name, parent) \
		vendor_data->name##_dir_entry = proc_mkdir(#name, parent); \
		if (IS_ERR(vendor_data->name##_dir_entry)) { \
			FTS_ERROR("Create proc dir entry '%s' failed %ld", \
			#name, PTR_ERR(vendor_data->name##_dir_entry)); \
			vendor_data->name##_dir_entry = NULL; \
			return PTR_ERR(vendor_data->name##_dir_entry); \
		}
	
#define FTS_MK_PROC_ENTRY(name, mode, parent) \
		vendor_data->name##_entry = proc_create(#name, mode, parent, &name##_fops); \
		if (IS_ERR(vendor_data->name##_entry)) { \
			FTS_ERROR("Create proc entry '%s' failed %ld", \
			#name, PTR_ERR(vendor_data->name##_entry)); \
			vendor_data->name##_entry = NULL; \
			return PTR_ERR(vendor_data->name##_entry); \
		}

	struct fts_oppo_data *vendor_data = &_oppo_data;

	FTS_INFO("oppo proc Init start");
	
	memset(vendor_data, 0, sizeof(*vendor_data));
	ts_data->oppo_vendor_data = vendor_data;

	FTS_MK_PROC_DIR(touchpanel, NULL)
	FTS_MK_PROC_ENTRY(baseline_test, 0666, vendor_data->touchpanel_dir_entry)
	FTS_MK_PROC_ENTRY(coordinate, 0444, vendor_data->touchpanel_dir_entry)
	FTS_MK_PROC_ENTRY(debug_level, 0644, vendor_data->touchpanel_dir_entry)
	FTS_MK_PROC_ENTRY(double_tap_enable, 0666, vendor_data->touchpanel_dir_entry)
	FTS_MK_PROC_ENTRY(irq_depth, 0666, vendor_data->touchpanel_dir_entry)
	FTS_MK_PROC_ENTRY(oppo_register_info, 0664, vendor_data->touchpanel_dir_entry)
	FTS_MK_PROC_ENTRY(oppo_tp_limit_area, 0664, vendor_data->touchpanel_dir_entry)
	FTS_MK_PROC_ENTRY(oppo_tp_limit_enable, 0664, vendor_data->touchpanel_dir_entry)
	FTS_MK_PROC_ENTRY(tp_fw_update, 0644, vendor_data->touchpanel_dir_entry)
	FTS_MK_PROC_ENTRY(black_screen_test, 0644, vendor_data->touchpanel_dir_entry)
		

	FTS_MK_PROC_DIR(debug_info, vendor_data->touchpanel_dir_entry)
	FTS_MK_PROC_ENTRY(delta, 0444, vendor_data->debug_info_dir_entry)
	FTS_MK_PROC_ENTRY(baseline, 0444, vendor_data->debug_info_dir_entry)
	FTS_MK_PROC_ENTRY(main_register, 0664, vendor_data->debug_info_dir_entry)

	FTS_MK_PROC_DIR(touchscreen, NULL)
	FTS_MK_PROC_ENTRY(openshort, 0664, vendor_data->touchscreen_dir_entry)
	
#if 0
	int i = 0;
	struct proc_dir_entry *touchpanel_dir_entry;
	struct proc_dir_entry *debug_info_dir_entry;
	struct proc_dir_entry *touchscreen_dir_entry;
#define FTS_MK_PROC_DIR(name, parent) \
			name##_dir_entry = proc_mkdir(#name, parent); \
			if (IS_ERR(name##_dir_entry)) { \
				FTS_ERROR("Create proc dir entry '%s' failed %ld", \
			#name, PTR_ERR(name##_dir_entry)); \
				name##_dir_entry = NULL; \
				return PTR_ERR(name##_dir_entry); \
			}
			
#define FTS_MK_PROC_ENTRY(name, mode, parent) \
					name##_entry = proc_create(#name, mode, parent, &name##_fops); \
					if (IS_ERR(name##_entry)) { \
						FTS_ERROR("Create proc entry '%s' failed %ld", \
			#name, PTR_ERR(name##_entry)); \
						name##_entry = NULL; \
						return PTR_ERR(name##_entry); \
					}
				
	FTS_MK_PROC_DIR(touchscreen,NULL)
	FTS_MK_PROC_DIR(touchpanel,NULL)
	FTS_MK_PROC_DIR(debug_info,touchpanel_dir_entry)

	for(i = 0; i < ARRAY_SIZE(proc_table); i++)
	{
		if( strcmp(proc_table[i].name,"openshort") == 0 ) {

			FTS_MK_PROC_ENTRY(proc_table[i].name,666,touchscreen_dir_entry)	
		} else if ((strcmp(proc_table[i].name,"delta") && strcmp(proc_table[i].name,"delta") && strcmp(proc_table[i].name,"delta")) == 0 ) {
			
			FTS_MK_PROC_ENTRY(proc_table[i].name,666,debug_info_dir_entry)
		} else {
			
			FTS_MK_PROC_ENTRY(proc_table[i].name,666,touchpanel_dir_entry)
		}
	}
#endif
	FTS_INFO("oppo proc Init end");
	return 0;
}
