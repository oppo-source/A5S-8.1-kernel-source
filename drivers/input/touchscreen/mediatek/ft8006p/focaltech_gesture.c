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
* File Name: focaltech_gestrue.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
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
#if FTS_GESTURE_EN
/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define KEY_GESTURE_U                           KEY_U
#define KEY_GESTURE_UP                          KEY_UP
#define KEY_GESTURE_DOWN                        KEY_DOWN
#define KEY_GESTURE_LEFT                        KEY_LEFT
#define KEY_GESTURE_RIGHT                       KEY_RIGHT
#define KEY_GESTURE_O                           KEY_O
#define KEY_GESTURE_E                           KEY_E
#define KEY_GESTURE_M                           KEY_M
#define KEY_GESTURE_L                           KEY_L
#define KEY_GESTURE_W                           KEY_W
#define KEY_GESTURE_S                           KEY_S
#define KEY_GESTURE_V                           KEY_V
#define KEY_GESTURE_C                           KEY_C
#define KEY_GESTURE_Z                           KEY_Z

#define GESTURE_LEFT                            0x20
#define GESTURE_RIGHT                           0x21
#define GESTURE_UP                              0x22
#define GESTURE_DOWN                            0x23
#define GESTURE_DOUBLECLICK                     0x24
#define GESTURE_O                               0x30
#define GESTURE_W                               0x31
#define GESTURE_M                               0x32
#define GESTURE_E                               0x33
#define GESTURE_L                               0x44
#define GESTURE_S                               0x46
#define GESTURE_V                               0x54
#define GESTURE_Z                               0x41
#define GESTURE_C                               0x34

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

/*****************************************************************************
* Static variables
*****************************************************************************/
struct fts_gesture_st fts_gesture_data;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static ssize_t fts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t fts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_buf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

/* sysfs gesture node
 *   read example: cat  fts_gesture_mode        ---read gesture mode
 *   write example:echo 0/1 > fts_gesture_mode   ---disable/enable gesture mode
 *
 */
static DEVICE_ATTR (fts_gesture_mode, S_IRUGO | S_IWUSR, fts_gesture_show, fts_gesture_store);
/*
 *   read example: cat fts_gesture_buf        ---read gesture buf
 */
static DEVICE_ATTR (fts_gesture_buf, S_IRUGO | S_IWUSR, fts_gesture_buf_show, fts_gesture_buf_store);
static struct attribute *fts_gesture_mode_attrs[] = {
    &dev_attr_fts_gesture_mode.attr,
    &dev_attr_fts_gesture_buf.attr,
    NULL,
};

static struct attribute_group fts_gesture_group = {
    .attrs = fts_gesture_mode_attrs,
};

static ssize_t fts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    fts_read_reg(FTS_REG_GESTURE_EN, &val);
    count = snprintf(buf, PAGE_SIZE, "Gesture Mode: %s\n", fts_gesture_data.mode ? "On" : "Off");
    count += snprintf(buf + count, PAGE_SIZE, "Reg(0xD0) = %d\n", val);
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();
    return count;
}

static ssize_t fts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;
    mutex_lock(&input_dev->mutex);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        FTS_INFO("enable gesture");
        fts_gesture_data.mode = ENABLE;
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        FTS_INFO("disable gesture");
        fts_gesture_data.mode = DISABLE;
    }
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    int i = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    count = snprintf(buf, PAGE_SIZE, "Gesture ID: 0x%x\n", fts_gesture_data.header[0]);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture PointNum: %d\n", fts_gesture_data.header[1]);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture Point Buf:\n");
    for (i = 0; i < fts_gesture_data.header[1]; i++) {
        count += snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i, fts_gesture_data.coordinate_x[i], fts_gesture_data.coordinate_y[i]);
        if ((i + 1) % 4 == 0)
            count += snprintf(buf + count, PAGE_SIZE, "\n");
    }
    count += snprintf(buf + count, PAGE_SIZE, "\n");
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();
    return count;
}

static ssize_t fts_gesture_buf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    /* place holder for future use */
    return -EPERM;
}

int fts_create_gesture_sysfs(struct device *dev)
{
    int ret = 0;

    ret = sysfs_create_group(&dev->kobj, &fts_gesture_group);
    if (ret) {
        FTS_ERROR("gesture sys node create fail");
        sysfs_remove_group(&dev->kobj, &fts_gesture_group);
        return ret;
    }

    return 0;
}

static void fts_gesture_report(struct input_dev *input_dev, int gesture_id)
{
    int gesture;

    FTS_FUNC_ENTER();
    FTS_INFO("fts gesture_id==0x%x ", gesture_id);
    switch (gesture_id) {
    case GESTURE_LEFT:
        gesture = KEY_GESTURE_LEFT;
        break;
    case GESTURE_RIGHT:
        gesture = KEY_GESTURE_RIGHT;
        break;
    case GESTURE_UP:
        gesture = KEY_GESTURE_UP;
        break;
    case GESTURE_DOWN:
        gesture = KEY_GESTURE_DOWN;
        break;
    case GESTURE_DOUBLECLICK:
        gesture = KEY_GESTURE_U;
        break;
    case GESTURE_O:
        gesture = KEY_GESTURE_O;
        break;
    case GESTURE_W:
        gesture = KEY_GESTURE_W;
        break;
    case GESTURE_M:
        gesture = KEY_GESTURE_M;
        break;
    case GESTURE_E:
        gesture = KEY_GESTURE_E;
        break;
    case GESTURE_L:
        gesture = KEY_GESTURE_L;
        break;
    case GESTURE_S:
        gesture = KEY_GESTURE_S;
        break;
    case GESTURE_V:
        gesture = KEY_GESTURE_V;
        break;
    case GESTURE_Z:
        gesture = KEY_GESTURE_Z;
        break;
    case  GESTURE_C:
        gesture = KEY_GESTURE_C;
        break;
    default:
        gesture = -1;
        break;
    }
    /* report event key */
    if (gesture != -1) {
        FTS_DEBUG("Gesture Code=%d", gesture);
        input_report_key(input_dev, gesture, 1);
        input_sync(input_dev);
        input_report_key(input_dev, gesture, 0);
        input_sync(input_dev);
    }

    FTS_FUNC_EXIT();
}

int fts_gesture_readdata(struct fts_ts_data *ts_data, u8 *data)
{
    int i = 0;
    int gestrue_id = 0;
    u8 pointnum = 0;
    struct input_dev *input_dev = ts_data->input_dev;
    struct fts_oppo_gesture *oppo_gesture = &ts_data->oppo_vendor_data->gesture_data;

    if (!ts_data->suspended) {
        return -EINVAL;
    }

    /* gesture invalid */
    if (data[0] != 0x01) {
        return 1;
    }

    /* init variable before read gesture point */
    memset(fts_gesture_data.coordinate_x, 0, FTS_GESTURE_POINT_MAX * sizeof(u16));
    memset(fts_gesture_data.coordinate_y, 0, FTS_GESTURE_POINT_MAX * sizeof(u16));

    gestrue_id = data[2];
    pointnum = data[3];
    FTS_DEBUG("[GESTURE]PointNum=%d", pointnum);
    if (pointnum > FTS_GESTURE_POINT_MAX) {
        FTS_ERROR("gesture pointnum(%d) fail", pointnum);
        return -EIO;
    }

    for (i = 0; i < pointnum; i++) {
        fts_gesture_data.coordinate_x[i] = (((s16) data[0 + (4 * i + 4)]) & 0xFF) << 8
                                           | (((s16) data[1 + (4 * i + 4)]) & 0xFF);
        fts_gesture_data.coordinate_y[i] = (((s16) data[2 + (4 * i + 4)]) & 0xFF) << 8
                                           | (((s16) data[3 + (4 * i + 4)]) & 0xFF);
    }

    oppo_gesture->gesture_id = data[2];
    oppo_gesture->clock_wise = data[1];
    oppo_gesture->start_x = fts_gesture_data.coordinate_x[0];
    oppo_gesture->start_y = fts_gesture_data.coordinate_y[0];
    oppo_gesture->end_x = fts_gesture_data.coordinate_x[1];
    oppo_gesture->end_y = fts_gesture_data.coordinate_y[1];
    oppo_gesture->point1_x = fts_gesture_data.coordinate_x[2];
    oppo_gesture->point1_y = fts_gesture_data.coordinate_y[2];
    oppo_gesture->point2_x = fts_gesture_data.coordinate_x[3];
    oppo_gesture->point2_y = fts_gesture_data.coordinate_y[3];
    oppo_gesture->point3_x = fts_gesture_data.coordinate_x[4];
    oppo_gesture->point3_y = fts_gesture_data.coordinate_y[4];
    oppo_gesture->point4_x = fts_gesture_data.coordinate_x[5];
    oppo_gesture->point4_y = fts_gesture_data.coordinate_y[5];
    FTS_DEBUG("oppo gesture:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d",
             oppo_gesture->gesture_id,
             oppo_gesture->start_x, oppo_gesture->start_y,
             oppo_gesture->end_x, oppo_gesture->end_y,
             oppo_gesture->point1_x, oppo_gesture->point1_y,
             oppo_gesture->point2_x, oppo_gesture->point2_y,
             oppo_gesture->point3_x, oppo_gesture->point3_y,
             oppo_gesture->point4_x, oppo_gesture->point4_y,
             oppo_gesture->clock_wise
             );

    fts_gesture_report(input_dev, gestrue_id);

    /* oppo report F4 */
    input_report_key(input_dev, KEY_F4, 1);
    input_sync(input_dev);
    input_report_key(input_dev, KEY_F4, 0);
    input_sync(input_dev);

    return 0;
}

void fts_gesture_recovery(void)
{
    if ((ENABLE == fts_gesture_data.mode) && (ENABLE == fts_gesture_data.active)) {
        FTS_DEBUG("enter fts_gesture_recovery");
        fts_write_reg(0xD1, 0xff);
        fts_write_reg(0xD2, 0xff);
        fts_write_reg(0xD5, 0xff);
        fts_write_reg(0xD6, 0xff);
        fts_write_reg(0xD7, 0xff);
        fts_write_reg(0xD8, 0xff);
        fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
    }
}

int fts_gesture_suspend(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int i = 0;
    u8 state = 0xFF;

    FTS_FUNC_ENTER();
    /* gesture not enable, return immediately */
    if (fts_gesture_data.mode == DISABLE) {
        FTS_DEBUG("gesture is disabled");
        return -EINVAL;
    }

    for (i = 0; i < 5; i++) {
        fts_write_reg(0xd1, 0xFF);
        fts_write_reg(0xd2, 0xFF);
        fts_write_reg(0xd5, 0xFF);
        fts_write_reg(0xd6, 0xFF);
        fts_write_reg(0xd7, 0xFF);
        fts_write_reg(0xd8, 0xFF);
        fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
        msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state == ENABLE)
            break;
    }

    if (i >= 5) {
        FTS_ERROR("Enter into gesture(suspend) fail");
        fts_gesture_data.active = DISABLE;
        return -EIO;
    }

    ret = enable_irq_wake(ts_data->irq);
    if (ret) {
        FTS_DEBUG("enable_irq_wake(irq:%d) fail", ts_data->irq);
    }

    fts_gesture_data.active = ENABLE;
    FTS_INFO("Enter into gesture(suspend) success");
    FTS_FUNC_EXIT();
    return 0;
}

int fts_gesture_resume(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int i = 0;
    u8 state = 0xFF;

    FTS_FUNC_ENTER();
    /* gesture not enable, return immediately */
    if (fts_gesture_data.mode == DISABLE) {
        FTS_DEBUG("gesture is disabled");
        return -EINVAL;
    }

    if (fts_gesture_data.active == DISABLE) {
        FTS_DEBUG("gesture in suspend is failed, no running fts_gesture_resume");
        return -EINVAL;
    }

    fts_gesture_data.active = DISABLE;
    for (i = 0; i < 5; i++) {
        fts_write_reg(FTS_REG_GESTURE_EN, DISABLE);
        msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state == DISABLE)
            break;
    }

    if (i >= 5) {
        FTS_ERROR("exit gesture(resume) fail");
        return -EIO;
    }

    ret = disable_irq_wake(ts_data->irq);
    if (ret) {
        FTS_DEBUG("disable_irq_wake(irq:%d) fail", ts_data->irq);
    }

    FTS_DEBUG("exit gesture(resume) success");
    return 0;
}

int fts_gesture_init(struct fts_ts_data *ts_data)
{
    struct input_dev *input_dev = ts_data->input_dev;

    FTS_FUNC_ENTER();
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(KEY_F4, input_dev->keybit);
    input_set_capability(input_dev, EV_KEY, KEY_F4);
   
    input_set_capability(input_dev, EV_KEY, KEY_POWER);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_U);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);

    __set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
    __set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
    __set_bit(KEY_GESTURE_UP, input_dev->keybit);
    __set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
    __set_bit(KEY_GESTURE_U, input_dev->keybit);
    __set_bit(KEY_GESTURE_O, input_dev->keybit);
    __set_bit(KEY_GESTURE_E, input_dev->keybit);
    __set_bit(KEY_GESTURE_M, input_dev->keybit);
    __set_bit(KEY_GESTURE_W, input_dev->keybit);
    __set_bit(KEY_GESTURE_L, input_dev->keybit);
    __set_bit(KEY_GESTURE_S, input_dev->keybit);
    __set_bit(KEY_GESTURE_V, input_dev->keybit);
    __set_bit(KEY_GESTURE_C, input_dev->keybit);
    __set_bit(KEY_GESTURE_Z, input_dev->keybit);

    fts_create_gesture_sysfs(&ts_data->spi->dev);
    fts_gesture_data.mode = DISABLE;
    ts_data->gesture = &fts_gesture_data;

    FTS_FUNC_EXIT();
    return 0;
}

int fts_gesture_exit(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    sysfs_remove_group(&ts_data->spi->dev.kobj, &fts_gesture_group);
    FTS_FUNC_EXIT();
    return 0;
}
#endif
