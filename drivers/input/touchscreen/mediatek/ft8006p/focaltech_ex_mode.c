/*
 *
 * FocalTech ftxxxx TouchScreen driver.
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
 * File Name: focaltech_ex_mode.c
 *
 *    Author: Liu WeiGuang
 *
 *   Created: 2016-08-31
 *
 *  Abstract:
 *
 * Reference:
 *
 *****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

/*****************************************************************************
* Static variables
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*******************************************************************************/
#if FTS_GLOVE_EN
int fts_enter_glove_mode(int mode)
{
    int ret = 0;
    u8 reg_addr = FTS_REG_GLOVE_MODE_EN;
    u8 reg_value = 0;

    if (mode)
        reg_value = 0x01;
    else
        reg_value = 0x00;

    ret = fts_write_reg(reg_addr, reg_value);
    if (ret < 0) {
        FTS_ERROR("write glove mode register fail");
        return ret;
    }

    return 0;
}

static ssize_t fts_touch_glove_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    fts_read_reg(FTS_REG_GLOVE_MODE_EN, &val);
    count = sprintf(buf, "Glove Mode: %s\n", fts_data->glove_mode ? "On" : "Off");
    count += sprintf(buf + count, "Glove Reg(0xC0) = %d\n", val);
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();
    return count;
}

static ssize_t fts_touch_glove_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    FTS_FUNC_ENTER();
    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!ts_data->glove_mode) {
            ret = fts_enter_glove_mode(true);
            if (0 == ret) {
                FTS_INFO("enter glove mode success");
                ts_data->glove_mode = true;
            }
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (ts_data->glove_mode) {
            ret = fts_enter_glove_mode(false);
            if (0 == ret) {
                FTS_INFO("exit glove mode success");
                ts_data->glove_mode = false;
            }
        }
    }

    FTS_INFO("glove mode is %d", ts_data->glove_mode);
    return count;
}

/* read and write glove mode
 *   read example: cat  fts_touch_glove_mode---read glove mode status
 *   write example:echo 0/1 > fts_touch_glove_mode ---enter/exit glove mode
 */
static DEVICE_ATTR (fts_glove_mode, S_IRUGO | S_IWUSR, fts_touch_glove_show, fts_touch_glove_store);
#endif

#if FTS_COVER_EN
int fts_enter_cover_mode(int mode)
{
    int ret = 0;
    u8 reg_addr = FTS_REG_COVER_MODE_EN;
    u8 reg_value = 0;

    if (mode)
        reg_value = 0x01;
    else
        reg_value = 0x00;

    ret = fts_write_reg(reg_addr, reg_value);
    if (ret < 0) {
        FTS_ERROR("write cover mode register fail");
        return ret;
    }

    return 0;
}

static ssize_t fts_touch_cover_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    fts_read_reg(FTS_REG_COVER_MODE_EN, &val);
    count = sprintf(buf, "Cover Mode: %s\n", fts_data->cover_mode ? "On" : "Off");
    count += sprintf(buf + count, "Cover Reg(0xC1) = %d\n", val);
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();
    return count;
}

static ssize_t fts_touch_cover_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!ts_data->cover_mode) {
            ret = fts_enter_cover_mode(true);
            if (0 == ret) {
                FTS_INFO("enter cover mode success");
                ts_data->cover_mode = true;
            }
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (ts_data->cover_mode) {

            ret = fts_enter_cover_mode(false);
            if (0 == ret) {
                FTS_INFO("exit cover mode success");
                ts_data->cover_mode = false;
            }
        }
    }
    FTS_INFO("cover mode is %d", ts_data->cover_mode);
    return count;
}

/* read and write cover mode
 *   read example: cat  fts_touch_cover_mode---read  cover mode
 *   write example:echo 01 > fts_touch_cover_mode ---write cover mode to 01
 */
static DEVICE_ATTR (fts_cover_mode, S_IRUGO | S_IWUSR, fts_touch_cover_show, fts_touch_cover_store);
#endif

#if FTS_CHARGER_EN
int fts_enter_charger_mode(int mode)
{
    int ret = 0;
    u8 reg_addr = FTS_REG_CHARGER_MODE_EN;
    u8 reg_value = 0;

    if (mode)
        reg_value = 0x01;
    else
        reg_value = 0x00;

    ret = fts_write_reg(reg_addr, reg_value);
    if (ret < 0) {
        FTS_ERROR("write charger mode register fail");
        return ret;
    }

    return 0;
}

static ssize_t fts_touch_charger_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    u8 val;
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    fts_read_reg(FTS_REG_CHARGER_MODE_EN, &val);
    count = sprintf(buf, "Charge Mode: %s\n", fts_data->charger_mode ? "On" : "Off");
    count += sprintf(buf + count, "Charge Reg(0x8B) = %d\n", val);
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();
    return count;
}

static ssize_t fts_touch_charger_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!ts_data->charger_mode) {
            ret = fts_enter_charger_mode(true);
            if (0 == ret) {
                FTS_INFO("enter charger mode success");
                ts_data->charger_mode = true;
            }
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (ts_data->charger_mode) {
            ret = fts_enter_charger_mode(false);
            if (0 == ret) {
                FTS_INFO("exit charger mode success");
                ts_data->charger_mode = false;
            }
        }
    }
    FTS_INFO("charger mode is %d", ts_data->charger_mode);
    return count;
}

/* read and write charger mode
 *   read example: cat  fts_touch_charger_mode---read  charger mode
 *   write example:echo 01 > fts_touch_charger_mode ---write charger mode to 01
 */
static DEVICE_ATTR (fts_charger_mode, S_IRUGO | S_IWUSR, fts_touch_charger_show, fts_touch_charger_store);
#endif

int fts_ex_mode_recovery(void)
{
    int ret = 0;
#if FTS_GLOVE_EN
    if (fts_data->glove_mode)
        ret = fts_enter_glove_mode(true);
#endif

#if FTS_COVER_EN
    if (fts_data->cover_mode)
        ret = fts_enter_cover_mode(true);
#endif

#if FTS_CHARGER_EN
    if (fts_data->charger_mode)
        ret = fts_enter_charger_mode(true);
#endif

    return ret;
}

static struct attribute *fts_touch_mode_attrs[] = {
#if FTS_GLOVE_EN
    &dev_attr_fts_glove_mode.attr,
#endif

#if FTS_COVER_EN
    &dev_attr_fts_cover_mode.attr,
#endif

#if FTS_CHARGER_EN
    &dev_attr_fts_charger_mode.attr,
#endif

    NULL,
};

static struct attribute_group fts_touch_mode_group = {
    .attrs = fts_touch_mode_attrs,
};

int fts_ex_mode_init(struct device *dev)
{
    int ret = 0;

    fts_data->glove_mode = false;
    fts_data->cover_mode = false;
    fts_data->charger_mode = false;

    ret = sysfs_create_group(&dev->kobj, &fts_touch_mode_group);
    if (ret) {
        FTS_ERROR("create sysfs fail");
        sysfs_remove_group(&dev->kobj, &fts_touch_mode_group);
        return ret;
    }

    FTS_DEBUG("create ex-mode sys node success");
    return ret;
}

int fts_ex_mode_exit(struct device *dev)
{
    sysfs_remove_group(&dev->kobj, &fts_touch_mode_group);
    return 0;
}
