/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2018, FocalTech Systems, Ltd., all rights reserved.
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
* File Name: focaltech_core.c
*
* Author: Focaltech Driver Team
*
* Created: 2017-11-06
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1     /* Early-suspend level */
#endif

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define INTERVAL_READ_REG                   100  /* unit:ms */
#define TIMEOUT_READ_REG                    1000 /* unit:ms */

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *fts_data;
u8 fts_oppo_debug_level = 1;


/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);


/*
 * description: wait tp to run fw normal(Timeout: TIMEOUT_READ_REG),
 *              need call when reset/power on/resume...
 *
 * param -
 *
 * return 0 if tp valid, otherwise return error code
 */
int fts_wait_tp_to_valid(void)
{
    int ret = 0;
    int cnt = 0;
    u8 reg_value = 0;
    u8 chip_id = fts_data->ic_info.ids.chip_idh;

    do {
        ret = fts_read_reg(FTS_REG_CHIP_ID, &reg_value);
        if ((ret < 0) || (reg_value != chip_id)) {
            FTS_DEBUG("TP Not Ready, ReadData = 0x%x", reg_value);
        } else if (reg_value == chip_id) {
            FTS_INFO("TP Ready, Device ID = 0x%x", reg_value);
            return 0;
        }
        cnt++;
        msleep(INTERVAL_READ_REG);
    } while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    return -EIO;
}

static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
    int i = 0;
    struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
    int ctype_num = sizeof(ctype) / sizeof(struct ft_chip_t);

    for (i = 0; i < ctype_num; i++) {
        FTS_INFO("%d %d", IC_SERIALS, (int)ctype[i].type);
        if (IC_SERIALS == ctype[i].type)
            break;
    }

    if (i >= ctype_num) {
        FTS_ERROR("get ic type fail, pls check FTS_CHIP_TYPE_MAPPING");
        return -ENODATA;
    }

    ts_data->ic_info.ids = ctype[i];
    FTS_INFO("CHIP TYPE ID in driver = 0x%02x%02x",
             ts_data->ic_info.ids.chip_idh,
             ts_data->ic_info.ids.chip_idl);

    return 0;
}

/*
 * description: recovery tp state: gesture/cover/glove...
 */
void fts_tp_state_recovery(void)
{
    FTS_FUNC_ENTER();
    /* wait tp stable */
    fts_wait_tp_to_valid();
    /* recover TP charger state 0x8B */
    /* recover TP glove state 0xC0 */
    /* recover TP cover state 0xC1 */

    fts_ex_mode_recovery();
    /* recover TP gesture state 0xD0 */

#if FTS_GESTURE_EN
    fts_gesture_recovery();
#endif
    FTS_FUNC_EXIT();
}

/*
 * description: Execute hw reset operation
 */
int fts_reset_proc(int hdelayms)
{
    FTS_DEBUG("tp reset");
    gpio_direction_output(fts_data->pdata->reset_gpio, 0);
    msleep(5);
    gpio_direction_output(fts_data->pdata->reset_gpio, 1);
    if (hdelayms) {
        msleep(hdelayms);
    }

    return 0;
}

void fts_irq_disable(void)
{
    unsigned long irqflags;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (!fts_data->irq_disabled) {
        disable_irq_nosync(fts_data->irq);
        fts_data->irq_disabled = true;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

void fts_irq_enable(void)
{
    unsigned long irqflags = 0;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (fts_data->irq_disabled) {
        enable_irq(fts_data->irq);
        fts_data->irq_disabled = false;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}


static int fts_edge_enable(u8 mode)
{
	int ret = 0;
	u8 reg_addr = 0xCD;

	ret = fts_write_reg(reg_addr, mode);
	if (ret < 0) {
		FTS_ERROR("write edge mode register fail");
		return ret;
	}

	return 0;
}


static int fts_edge_mode_swtich(u8 mode)
{
	int ret = 0;
	u8 reg_addr = 0x8C;

	ret = fts_write_reg(reg_addr, mode);
	if (ret < 0) {
		FTS_ERROR("write edge mode register fail");
		return ret;
	}

	return 0;
}

static int fts_charge_mode_swtich(u8 mode)
{
	int ret = 0;
	u8 reg_addr = FTS_REG_CHARGER_MODE_EN;

	ret = fts_write_reg(reg_addr, mode);
	if (ret < 0) {
		FTS_ERROR("write charge mode register fail");
		return ret;
	}

	return 0;
}

static int fts_jitter_mode_swtich(u8 mode)
{
	int ret = 0;
	u8 reg_addr = 0xCC;

	ret = fts_write_reg(reg_addr, mode);
	if (ret < 0) {
		FTS_ERROR("write jitter mode register fail");
		return ret;
	}

	return 0;
}

int fts_mode_switch(u8 mode, u8 value)
{
	int ret = 0;

	switch(mode) {
	case MODE_EDGE_ENABLE:
		ret = fts_edge_enable(value);
		if (ret < 0) {
			FTS_ERROR("edge enable fail");
			return ret;
		}
		break;
	case MODE_EDGE:
		ret = fts_edge_mode_swtich(value);
		if (ret < 0) {
			FTS_ERROR("edge mode switch fail");
			return ret;
		}
		break;

	case MODE_CHARGE:
		ret = fts_charge_mode_swtich(value);
		if (ret < 0) {
			FTS_ERROR("charge mode switch fail");
			return ret;
		}
		break;

	case MODE_GAME:
		ret = fts_jitter_mode_swtich(value);
		if (ret < 0) {
			FTS_ERROR("game(jitter) mode switch fail");
			return ret;
		}
		break;

	default:
		FTS_ERROR("Wrong mode:%d", mode);
	}

	return ret;
}


/*****************************************************************************
*  Reprot related
*****************************************************************************/
#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
char g_sz_debug[1024] = {0};
static void fts_show_touch_buffer(u8 *buf, int point_num)
{
    int len = point_num * FTS_ONE_TCH_LEN;
    int count = 0;
    int i;

    memset(g_sz_debug, 0, 1024);
    if (len > (fts_data->pnt_buf_size - 3)) {
        len = fts_data->pnt_buf_size - 3;
    } else if (len == 0) {
        len += FTS_ONE_TCH_LEN;
    }
    count += snprintf(g_sz_debug, PAGE_SIZE, "%02X,%02X,%02X", buf[0], buf[1], buf[2]);
    for (i = 0; i < len; i++) {
        count += snprintf(g_sz_debug + count, PAGE_SIZE, ",%02X", buf[i + 3]);
    }
    FTS_DEBUG("buffer: %s", g_sz_debug);
}
#endif

void fts_release_all_finger(void)
{
    struct input_dev *input_dev = fts_data->input_dev;
#if FTS_MT_PROTOCOL_B_EN
    u32 finger_count = 0;
#endif

    FTS_FUNC_ENTER();
    mutex_lock(&fts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
    for (finger_count = 0; finger_count < fts_data->pdata->max_touch_number; finger_count++) {
        input_mt_slot(input_dev, finger_count);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
    }
#else
    input_mt_sync(input_dev);
#endif
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_sync(input_dev);
	fts_data->touchs = 0;
    mutex_unlock(&fts_data->report_mutex);
    FTS_FUNC_EXIT();
}

static int fts_input_report_key(struct fts_ts_data *data, int index)
{
    u32 ik;
    int id = data->events[index].id;
    int x = data->events[index].x;
    int y = data->events[index].y;
    int flag = data->events[index].flag;
    u32 key_num = data->pdata->key_number;

    if (!KEY_EN(data)) {
        return -EINVAL;
    }
    for (ik = 0; ik < key_num; ik++) {
        if (TOUCH_IN_KEY(x, data->pdata->key_x_coords[ik])) {
            if (EVENT_DOWN(flag)) {
                data->key_down = true;
                input_report_key(data->input_dev, data->pdata->keys[ik], 1);
                FTS_DEBUG("Key%d(%d, %d) DOWN!", ik, x, y);
            } else {
                data->key_down = false;
                input_report_key(data->input_dev, data->pdata->keys[ik], 0);
                FTS_DEBUG("Key%d(%d, %d) Up!", ik, x, y);
            }
            return 0;
        }
    }

    FTS_ERROR("invalid touch for key, [%d](%d, %d)", id, x, y);
    return -EINVAL;
}

static int corner_point_process(struct fts_ts_data *data)
{
	int i = 0;
	int points_except_corner = 0;
	struct ts_event *events = data->events;
	struct _tp_limit *tp_limit = &fts_data->oppo_vendor_data->tp_limit;

	for (i = 0; i < data->touch_point; i++) {
		events[i].in_limit_area = 0;
		events[i].need_report = 1;
		if (tp_limit->enable.bit.lu && 
			((events[i].x < tp_limit->xlu) && (events[i].y < tp_limit->ylu))) {
			events[i].in_limit_area = 1;
		} else if (tp_limit->enable.bit.ru && 
			((events[i].x > tp_limit->xru) && (events[i].y < tp_limit->yru))) {
			events[i].in_limit_area = 1;
		} else if (tp_limit->enable.bit.lb && 
			((events[i].x < tp_limit->xlb) && (events[i].y > tp_limit->ylb))) {
			events[i].in_limit_area = 1;
		} else if (tp_limit->enable.bit.rb && 
			((events[i].x > tp_limit->xrb) && (events[i].y > tp_limit->yrb))) {
			events[i].in_limit_area = 1;
		} else {
			points_except_corner++;
		}
	}

	if (points_except_corner) {
		for (i = 0; i < data->touch_point; i++) {
			if (events[i].in_limit_area)
				events[i].need_report = 0;
		}
	}

	return 0;
}

#if FTS_MT_PROTOCOL_B_EN
static int fts_input_report_b(struct fts_ts_data *data)
{
    int i = 0;
    int uppoint = 0;
    int touchs = 0;
    bool va_reported = false;
    u32 max_touch_num = data->pdata->max_touch_number;
    u32 key_y_coor = data->pdata->key_y_coord;
    struct ts_event *events = data->events;

    for (i = 0; i < data->touch_point; i++) {
        if (KEY_EN(data) && TOUCH_IS_KEY(events[i].y, key_y_coor)) {
            fts_input_report_key(data, i);
            continue;
        }

		if (!events[i].need_report)
			continue;

        if (events[i].id >= max_touch_num)
            break;

        va_reported = true;
        input_mt_slot(data->input_dev, events[i].id);

        if (EVENT_DOWN(events[i].flag)) {
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);

#if FTS_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x3f;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            if (events[i].area < 0) {
                events[i].area = 0x05;
            }

            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);
            input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, events[i].area);
            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            touchs |= BIT(events[i].id);
            data->touchs |= BIT(events[i].id);
			
			if ((2 == fts_oppo_debug_level) ||
				((1 == fts_oppo_debug_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
            	FTS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!", events[i].id, events[i].x,
                      	 events[i].y, events[i].p, events[i].area);
			}
        } else {
            uppoint++;
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
            input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 0);
#if FTS_REPORT_PRESSURE_EN
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, 0);
#endif			
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
            data->touchs &= ~BIT(events[i].id);
            if ((2 == fts_oppo_debug_level) || (1 == fts_oppo_debug_level)) {
            	FTS_DEBUG("[B]P%d UP!", events[i].id);
            }
        }
    }

    if (unlikely(data->touchs ^ touchs)) {
        for (i = 0; i < max_touch_num; i++)  {
            if (BIT(i) & (data->touchs ^ touchs)) {
                if ((2 == fts_oppo_debug_level) || (1 == fts_oppo_debug_level)) {
            		FTS_DEBUG("[B]P%d UP!", events[i].id);
            	}
                va_reported = true;
                input_mt_slot(data->input_dev, i);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
            }
        }
    }
    data->touchs = touchs;

    if (va_reported) {
        /* touchs==0, there's no point but key */
        if (EVENT_NO_DOWN(data) || (!touchs)) {
            if ((2 == fts_oppo_debug_level) || (1 == fts_oppo_debug_level)) {
				FTS_DEBUG("[B]Points All Up!");
            }
            input_report_key(data->input_dev, BTN_TOUCH, 0);
        } else {
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }

    input_sync(data->input_dev);
    return 0;
}

#else
static int fts_input_report_a(struct fts_ts_data *data)
{
    int i = 0;
    int touchs = 0;
    bool va_reported = false;
    u32 key_y_coor = data->pdata->key_y_coord;
    struct ts_event *events = data->events;

    for (i = 0; i < data->touch_point; i++) {
        if (KEY_EN(data) && TOUCH_IS_KEY(events[i].y, key_y_coor)) {
            fts_input_report_key(data, i);
            continue;
        }

		if (!events[i].need_report)
			continue;

        va_reported = true;
        if (EVENT_DOWN(events[i].flag)) {
            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, events[i].id);
#if FTS_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x3f;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            if (events[i].area <= 0) {
                events[i].area = 0x05;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, events[i].area);

            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            input_mt_sync(data->input_dev);

            FTS_DEBUG("[A]P%d(%d, %d)[p:%d,tm:%d] DOWN!", events[i].id, events[i].x,
                      events[i].y, events[i].p, events[i].area);
            touchs++;
        }
    }

    /* last point down, current no point but key */
    if (data->touchs && !touchs) {
        va_reported = true;
    }
    data->touchs = touchs;

    if (va_reported) {
        if (EVENT_NO_DOWN(data)) {
            FTS_DEBUG("[A]Points All Up!");
            input_report_key(data->input_dev, BTN_TOUCH, 0);
            input_mt_sync(data->input_dev);
        } else {
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }

    input_sync(data->input_dev);
    return 0;
}
#endif

static int fts_read_touchdata(struct fts_ts_data *data)
{
    int ret = 0;
    int i = 0;
    u8 pointid = 0;
    int base = 0;
    struct ts_event *events = data->events;
    int max_touch_num = data->pdata->max_touch_number;
    u8 *buf = data->point_buf;
    u8 flag = 0;

#if FTS_POINT_REPORT_CHECK_EN
    fts_prc_queue_work(data);
#endif

    data->point_num = 0;
    data->touch_point = 0;
    memset(buf, 0xFF, data->pnt_buf_size);
    buf[0] = 0x01;
    ret = fts_read(buf, 1, buf + 1, data->pnt_buf_size - 1);
    flag = buf[1] & 0xF0;
    if ((ret < 0) && (flag != 0x90)) {
        /* check if need recovery fw */
        fts_fw_recovery();
        return ret;
    } else if ((ret < 0) || (flag != 0x90)) {
        FTS_ERROR("touch data(%x) fail,ret:%d", buf[1], ret);
        return -EIO;
    }

    data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;

#if FTS_GESTURE_EN
    ret = fts_gesture_readdata(data, buf + FTS_GESTURE_OFF);
    if (0 == ret) {
        FTS_INFO("succuss to get gesture data in irq handler");
        return 1;
    }
#endif

    data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
    if ((data->point_num == 0x0F) && (buf[1] == 0xFF) && (buf[2] == 0xFF)
        && (buf[3] == 0xFF) && (buf[4] == 0xFF) && (buf[5] == 0xFF) && (buf[6] == 0xFF)) {
        FTS_DEBUG("touch buff is 0xff, need recovery state");
        fts_tp_state_recovery();
        return -EIO;
    }

    if (data->point_num > max_touch_num) {
        FTS_INFO("invalid point_num(%d)", data->point_num);
        return -EIO;
    }

#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
    fts_show_touch_buffer(buf, data->point_num);
#endif

    for (i = 0; i < max_touch_num; i++) {
        base = FTS_ONE_TCH_LEN * i;

        pointid = (buf[FTS_TOUCH_ID_POS + base]) >> 4;
        if (pointid >= FTS_MAX_ID)
            break;
        else if (pointid >= max_touch_num) {
            FTS_ERROR("ID(%d) beyond max_touch_number", pointid);
            return -EINVAL;
        }

        data->touch_point++;

        events[i].x = ((buf[FTS_TOUCH_X_H_POS + base] & 0x0F) << 8) +
                      (buf[FTS_TOUCH_X_L_POS + base] & 0xFF);
        events[i].y = ((buf[FTS_TOUCH_Y_H_POS + base] & 0x0F) << 8) +
                      (buf[FTS_TOUCH_Y_L_POS + base] & 0xFF);
        events[i].flag = buf[FTS_TOUCH_EVENT_POS + base] >> 6;
        events[i].id = buf[FTS_TOUCH_ID_POS + base] >> 4;
        //events[i].area = buf[FTS_TOUCH_AREA_POS + base] >> 4;
        events[i].area = buf[FTS_TOUCH_AREA_POS + base];
        events[i].p =  buf[FTS_TOUCH_PRE_POS + base];

        if (EVENT_DOWN(events[i].flag) && (data->point_num == 0)) {
            FTS_INFO("abnormal touch data from fw");
            return -EIO;
        }
    }
    if (data->touch_point == 0) {
        FTS_INFO("no touch point information");
        return -EIO;
    }

	corner_point_process(data);
    return 0;
}

#ifdef FTS_MTK_PLATFORM
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag;

static int touch_event_handler(void *unused)
{
    int ret;
    struct fts_ts_data *ts_data = fts_data;
    struct sched_param param = { .sched_priority = 4 };

    sched_setscheduler(current, SCHED_RR, &param);
    do {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, tpd_flag != 0);

        tpd_flag = 0;

        set_current_state(TASK_RUNNING);

#if FTS_POINT_REPORT_CHECK_EN
        fts_prc_queue_work(ts_data);
#endif

#if FTS_ESDCHECK_EN
        fts_esdcheck_set_intr(1);
#endif

        ret = fts_read_touchdata(ts_data);
#if FTS_MT_PROTOCOL_B_EN
        if (ret == 0) {
            mutex_lock(&ts_data->report_mutex);
            fts_input_report_b(ts_data);
            mutex_unlock(&ts_data->report_mutex);
        }
#else
        if (ret == 0) {
            mutex_lock(&ts_data->report_mutex);
            fts_input_report_a(ts_data);
            mutex_unlock(&ts_data->report_mutex);
        }
#endif

#if FTS_ESDCHECK_EN
        fts_esdcheck_set_intr(0);
#endif
    } while (!kthread_should_stop());

    return 0;
}


static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}
#else
static void fts_report_event(struct fts_ts_data *data)
{
#if FTS_MT_PROTOCOL_B_EN
    fts_input_report_b(data);
#else
    fts_input_report_a(data);
#endif
}

static irqreturn_t fts_ts_interrupt(int irq, void *data)
{
    int ret = 0;
    struct fts_ts_data *ts_data = (struct fts_ts_data *)data;

    if (!ts_data) {
        FTS_ERROR("[INTR]: Invalid fts_ts_data");
        return IRQ_HANDLED;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_set_intr(1);
#endif

    ret = fts_read_touchdata(ts_data);
    if (ret == 0) {
        mutex_lock(&ts_data->report_mutex);
        fts_report_event(ts_data);
        mutex_unlock(&ts_data->report_mutex);
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_set_intr(0);
#endif

    return IRQ_HANDLED;
}
#endif

/*
 * description: register irq
 */
static int fts_irq_registration(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct fts_ts_platform_data *pdata = ts_data->pdata;

    ts_data->irq = gpio_to_irq(pdata->irq_gpio);
    FTS_INFO("irq in ts_data:%d irq in client:%d", ts_data->irq, ts_data->spi->irq);
    //if (ts_data->irq != ts_data->spi->irq)
       // FTS_ERROR("IRQs are inconsistent, please check <interrupts> & <focaltech,irq-gpio> in DTS");

//    if (0 == pdata->irq_gpio_flags)
    pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING;
    FTS_INFO("irq flag:%x", pdata->irq_gpio_flags);
#ifdef FTS_MTK_PLATFORM
	ret = request_irq(ts_data->irq, tpd_eint_interrupt_handler,
					  IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
#else
    ret = request_threaded_irq(ts_data->irq, NULL, fts_ts_interrupt,
                               pdata->irq_gpio_flags | IRQF_ONESHOT,
                               FTS_DRIVER_NAME, ts_data);
#endif
    return ret;
}

static int fts_input_init(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int key_num = 0;
    struct fts_ts_platform_data *pdata = ts_data->pdata;
    struct input_dev *input_dev;

    FTS_FUNC_ENTER();

    input_dev = input_allocate_device();
    if (!input_dev) {
        FTS_ERROR("Failed to allocate memory for input device");
        return -ENOMEM;
    }

    /* Init and register Input device */
    input_dev->name = FTS_DRIVER_NAME;
    input_dev->id.bustype = BUS_SPI;
    input_dev->dev.parent = &ts_data->spi->dev;

    input_set_drvdata(input_dev, ts_data);

    __set_bit(EV_SYN, input_dev->evbit);
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

    if (pdata->have_key) {
        FTS_INFO("set key capabilities");
        for (key_num = 0; key_num < pdata->key_number; key_num++)
            input_set_capability(input_dev, EV_KEY, pdata->keys[key_num]);
    }

#if FTS_MT_PROTOCOL_B_EN
    input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
#else
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0F, 0, 0);
#endif
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 0xFF, 0, 0);
#if FTS_REPORT_PRESSURE_EN
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif

    ret = input_register_device(input_dev);
    if (ret) {
        FTS_ERROR("Input device registration failed");
        goto err_input_reg;
    }

    ts_data->input_dev = input_dev;

    FTS_FUNC_EXIT();
    return 0;

err_input_reg:
    input_set_drvdata(input_dev, NULL);
    input_free_device(input_dev);
    input_dev = NULL;

    FTS_FUNC_EXIT();
    return ret;
}
static int fts_malloc_report(struct fts_ts_data *ts_data)
{
    int point_num = 0;
    int ret = 0;

    point_num = FTS_MAX_POINTS_SUPPORT;
    ts_data->pnt_buf_size = point_num * FTS_ONE_TCH_LEN + 3;
#if FTS_GESTURE_EN
    ts_data->pnt_buf_size += FTS_GESTURE_DATA_LEN;
#endif
    ts_data->point_buf = (u8 *)kzalloc(ts_data->pnt_buf_size + 1, GFP_KERNEL);
    if (!ts_data->point_buf) {
        FTS_ERROR("failed to alloc memory for point buf!");
        ret = -ENOMEM;
        goto err_point_buf;
    }

    ts_data->events = (struct ts_event *)kzalloc(point_num * sizeof(struct ts_event), GFP_KERNEL);
    if (!ts_data->events) {
        FTS_ERROR("failed to alloc memory for point events!");
        ret = -ENOMEM;
        goto err_event_buf;
    }

    return 0;

err_event_buf:
    kfree_safe(ts_data->point_buf);

err_point_buf:
    FTS_FUNC_EXIT();
    return ret;
}

/*
 * description: configure reset & irq gpio
 */
static int fts_gpio_configure(struct fts_ts_data *data)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    /* request irq gpio */
    if (gpio_is_valid(data->pdata->irq_gpio)) {
        ret = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
        if (ret) {
            FTS_ERROR("[GPIO]irq gpio request failed");
            goto err_irq_gpio_req;
        }

        ret = gpio_direction_input(data->pdata->irq_gpio);
        if (ret) {
            FTS_ERROR("[GPIO]set_direction for irq gpio failed");
            goto err_irq_gpio_dir;
        }
    }

    /* request reset gpio */
    if (gpio_is_valid(data->pdata->reset_gpio)) {
        ret = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
        if (ret) {
            FTS_ERROR("[GPIO]reset gpio request failed");
            goto err_irq_gpio_dir;
        }

        ret = gpio_direction_output(data->pdata->reset_gpio, 1);
        if (ret) {
            FTS_ERROR("[GPIO]set_direction for reset gpio failed");
            goto err_reset_gpio_dir;
        }
    }

    FTS_FUNC_EXIT();
    return 0;

err_reset_gpio_dir:
    if (gpio_is_valid(data->pdata->reset_gpio))
        gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
    if (gpio_is_valid(data->pdata->irq_gpio))
        gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
    FTS_FUNC_EXIT();
    return ret;
}

static int fts_get_dt_coords(struct device *dev, char *name,
                             struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    u32 coords[FTS_COORDS_ARR_SIZE] = { 0 };
    struct property *prop;
    struct device_node *np = dev->of_node;
    int coords_size;

    prop = of_find_property(np, name, NULL);
    if (!prop)
        return -EINVAL;
    if (!prop->value)
        return -ENODATA;

    coords_size = prop->length / sizeof(u32);
    if (coords_size != FTS_COORDS_ARR_SIZE) {
        FTS_ERROR("invalid:%s, size:%d", name, coords_size);
        return -EINVAL;
    }

    ret = of_property_read_u32_array(np, name, coords, coords_size);
    if (ret && (ret != -EINVAL)) {
        FTS_ERROR("Unable to read %s", name);
        return -ENODATA;
    }

    if (!strcmp(name, "focaltech,display-coords")) {
        pdata->x_min = coords[0];
        pdata->y_min = coords[1];
        pdata->x_max = coords[2];
        pdata->y_max = coords[3];
    } else {
        FTS_ERROR("unsupported property %s", name);
        return -EINVAL;
    }

    FTS_INFO("display x(%d %d) y(%d %d)", pdata->x_min, pdata->x_max,
             pdata->y_min, pdata->y_max);
    return 0;
}

/*
 * description: parse ts data from dts
 */
static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    struct device_node *np = dev->of_node;
    u32 temp_val = 0;

    FTS_FUNC_ENTER();

    ret = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
    if (ret < 0)
        FTS_ERROR("Unable to get display-coords");

    /* key */
    pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
    if (pdata->have_key) {
        ret = of_property_read_u32(np, "focaltech,key-number", &pdata->key_number);
        if (ret)
            FTS_ERROR("Key number undefined!");

        ret = of_property_read_u32_array(np, "focaltech,keys",
                                         pdata->keys, pdata->key_number);
        if (ret)
            FTS_ERROR("Keys undefined!");
        else if (pdata->key_number > FTS_MAX_KEYS)
            pdata->key_number = FTS_MAX_KEYS;

        ret = of_property_read_u32(np, "focaltech,key-y-coord", &pdata->key_y_coord);
        if (ret)
            FTS_ERROR("Key Y Coord undefined!");

        ret = of_property_read_u32_array(np, "focaltech,key-x-coords",
                                         pdata->key_x_coords, pdata->key_number);
        if (ret)
            FTS_ERROR("Key X Coords undefined!");

        FTS_INFO("VK(%d): (%d, %d, %d), [%d, %d, %d][%d]",
                 pdata->key_number, pdata->keys[0], pdata->keys[1], pdata->keys[2],
                 pdata->key_x_coords[0], pdata->key_x_coords[1], pdata->key_x_coords[2],
                 pdata->key_y_coord);
    }

    /* reset, irq gpio info */
    pdata->reset_gpio = of_get_named_gpio_flags(np, "touch,reset-gpio", 0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
        FTS_ERROR("Unable to get reset_gpio");

    pdata->irq_gpio = of_get_named_gpio_flags(np, "touch,irq-gpio", 0, &pdata->irq_gpio_flags);
    if (pdata->irq_gpio < 0)
        FTS_ERROR("Unable to get irq_gpio");

    ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
    if (0 == ret) {
        if (temp_val < 2)
            pdata->max_touch_number = 2;
        else if (temp_val > FTS_MAX_POINTS_SUPPORT)
            pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
        else
            pdata->max_touch_number = temp_val;
    } else {
        FTS_ERROR("Unable to get max-touch-number");
        pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
    }

    FTS_INFO("max touch number:%d, irq gpio:%d, reset gpio:%d",
             pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio);

    FTS_FUNC_EXIT();
    return 0;
}

#if defined(CONFIG_FB)
/* FB notifier callback from LCD driver */
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct fts_ts_data *ts_data =
        container_of(self, struct fts_ts_data, fb_notif);

    if (!(event == FB_EARLY_EVENT_BLANK || event == FB_EVENT_BLANK)) {
        FTS_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    FTS_INFO("FB event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case FB_BLANK_UNBLANK:
        if (FB_EARLY_EVENT_BLANK == event) {
            FTS_INFO("resume: event = %lu, not care\n", event);
        } else if (FB_EVENT_BLANK == event) {
            fts_ts_resume(&ts_data->spi->dev);
        }
        break;
    case FB_BLANK_POWERDOWN:
        if (FB_EARLY_EVENT_BLANK == event) {
            fts_ts_suspend(&ts_data->spi->dev);
        } else if (FB_EVENT_BLANK == event) {
            FTS_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        FTS_INFO("FB BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* early_suspend/resume */
static void fts_ts_early_suspend(struct early_suspend *handler)
{
    struct fts_ts_data *data = container_of(handler,
                                            struct fts_ts_data,
                                            early_suspend);

    fts_ts_suspend(&data->spi->dev);
}

static void fts_ts_late_resume(struct early_suspend *handler)
{
    struct fts_ts_data *data = container_of(handler,
                                            struct fts_ts_data,
                                            early_suspend);

    fts_ts_resume(&data->spi->dev);
}
#endif

static int __maybe_unused tid_get_version(struct device *dev, int *major, int *minor)
{
	*major = 0;
	*minor = 1;
	return 0;
}
static int __maybe_unused tid_get_lockdown_info(struct device *dev, char *out_values)
{
	out_values[LOCKDOWN_INFO_PANEL_MAKER_INDEX] = 0x54;
	return 0;
}

/*
 * description: spi driver probe
 */
static int fts_ts_probe(struct spi_device *spi)
{
    int ret = 0;
    struct fts_ts_platform_data *pdata;
    struct fts_ts_data *ts_data;

    char * cmdline_tp = NULL;

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp,2018/10/11,Add touch-info file function
	struct touch_info_dev *tid;
	struct touch_info_dev_operations *tid_ops;
	struct device *dev= &spi->dev;
#endif

    FTS_FUNC_ENTER();
    if (spi->dev.of_node) {
        FTS_INFO("focaltech dts confige...");
        pdata = devm_kzalloc(&spi->dev, sizeof(*pdata), GFP_KERNEL);
        if (!pdata) {
            FTS_ERROR("allocate memory for platform data fail");
            return -ENOMEM;
        }
//Bin.Su@ODM_WT.BSP.Tp.Init.2018/10/15,Add for compatible HLT and TXD module
        cmdline_tp = strstr(saved_command_line,"ft8006p_lide_hdp_dsi_vdo_lcm_drv");
        printk("%s cmdline_tp = %s\n",__func__,cmdline_tp);
        if ( cmdline_tp == NULL ){
            printk("%s get ft8006p_lide_hdp_dsi_vdo_lcm_drv fail ",__func__);
            return -1;
        }
        ret = fts_parse_dt(&spi->dev, pdata);
        if (ret)
            FTS_ERROR("[DTS]DT parsing failed");
    } else {
        pdata = spi->dev.platform_data;
    }

    if (!pdata) {
        FTS_ERROR("no ts platform data found");
        return -EINVAL;
    }

    spi->mode = SPI_MODE_1;
    spi->bits_per_word = 8;
//    if (!spi->max_speed_hz)
        spi->max_speed_hz = FTS_SPI_CLK_MAX;

    ret = spi_setup(spi);
    if (ret) {
        FTS_ERROR("spi setup fail");
        return ret;
    }

    ts_data = devm_kzalloc(&spi->dev, sizeof(*ts_data), GFP_KERNEL);
    if (!ts_data) {
        FTS_ERROR("allocate memory for fts_data fail");
        return -ENOMEM;
    }

    ts_data->spibuf = kzalloc(SPI_BUF_LENGTH, GFP_KERNEL);
    if (NULL == ts_data->spibuf) {
        FTS_ERROR("failed to allocate memory for spibuf");
        ret = -ENOMEM;
        goto err_spibuf;
    }

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp,2018/11/21,Add touch-info file function
	tid = devm_tid_and_ops_allocate(dev);
	if (unlikely(!tid))
		return -ENOMEM;
	ts_data->tid = tid;
	tid_ops = tid->tid_ops;
	tid_ops->get_version		   = tid_get_version;
	tid_ops->get_lockdown_info 	   = tid_get_lockdown_info;
#endif

    fts_data = ts_data;
    ts_data->spi = spi;
    ts_data->client = spi;
    ts_data->pdata = pdata;
    spi_set_drvdata(spi, ts_data);

    ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
    if (NULL == ts_data->ts_workqueue) {
        FTS_ERROR("failed to create fts workqueue");
    }

    spin_lock_init(&ts_data->irq_lock);
    mutex_init(&ts_data->report_mutex);
    mutex_init(&ts_data->spilock);

    ret = fts_input_init(ts_data);
    if (ret) {
        FTS_ERROR("input initialize fail");
        goto err_input_init;
    }

    ret = fts_malloc_report(ts_data);
    if (ret) {
        FTS_ERROR("malloc report fail");
        goto err_input_init;
    }

    ret = fts_gpio_configure(ts_data);
    if (ret) {
        FTS_ERROR("configure the gpios fail");
        goto err_gpio_config;
    }

    ret = fts_get_ic_information(ts_data);
    if (ret) {
        FTS_ERROR("ic type fail, please check driver setting");
        goto err_irq_req;
    }

#if FTS_APK_NODE_EN
    ret = fts_create_apk_debug_channel(ts_data);
    if (ret) {
        FTS_ERROR("create apk debug node fail");
    }
#endif

#if FTS_SYSFS_NODE_EN
    ret = fts_create_sysfs(&spi->dev);
    if (ret) {
        FTS_ERROR("create sysfs node fail");
    }
#endif
	ret = fts_proc_init(ts_data);
	if (ret != 0) {
		FTS_ERROR("proc init failed\n");
	}

#if FTS_POINT_REPORT_CHECK_EN
    ret = fts_point_report_check_init(ts_data);
    if (ret) {
        FTS_ERROR("init point report check fail");
    }
#endif

    ret = fts_ex_mode_init(&spi->dev);
    if (ret) {
        FTS_ERROR("init glove/cover/charger fail");
    }

#if FTS_GESTURE_EN
    ret = fts_gesture_init(ts_data);
    if (ret) {
        FTS_ERROR("init gesture fail");
    }
#endif

#if FTS_TEST_EN
    ret = fts_test_init(&spi->dev);
    if (ret) {
        FTS_ERROR("init production test fail");
    }
#endif

#if FTS_ESDCHECK_EN
    ret = fts_esdcheck_init(ts_data);
    if (ret) {
        FTS_ERROR("init esd check fail");
    }
#endif

#ifdef FTS_MTK_PLATFORM
	ts_data->thread_tpd = kthread_run(touch_event_handler, 0, "mtk-tpd");
    if (IS_ERR(ts_data->thread_tpd)) {
        ret = PTR_ERR(ts_data->thread_tpd);
        FTS_ERROR("[TPD]Failed to create kernel thread_tpd,ret:%d", ret);
        ts_data->thread_tpd = NULL;
        goto err_irq_req;
    }
#endif

    ret = fts_irq_registration(ts_data);
    if (ret) {
        FTS_ERROR("request irq failed");
        goto err_irq_req;
    }

    ret = fts_fw_init();
    if (ret) {
        FTS_ERROR("init fw fail, tp fw not run");
    }

#if defined(CONFIG_FB)
    ts_data->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts_data->fb_notif);
    if (ret) {
        FTS_ERROR("[FB]Unable to register fb_notifier: %d", ret);
    }
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    ts_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
    ts_data->early_suspend.suspend = fts_ts_early_suspend;
    ts_data->early_suspend.resume = fts_ts_late_resume;
    register_early_suspend(&ts_data->early_suspend);
#endif

#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp,2018/11/21,Add touch-info file function
	ret = devm_tid_register(dev, tid);
	if (unlikely(ret))
		return ret;
#endif
    FTS_FUNC_EXIT();
    return 0;

err_irq_req:
    if (gpio_is_valid(pdata->reset_gpio))
        gpio_free(pdata->reset_gpio);
    if (gpio_is_valid(pdata->irq_gpio))
        gpio_free(pdata->irq_gpio);
err_gpio_config:
    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);
    input_unregister_device(ts_data->input_dev);
err_input_init:
    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);
    kfree_safe(ts_data->spibuf);
err_spibuf:
    devm_kfree(&spi->dev, ts_data);

    FTS_FUNC_EXIT();
    return ret;
}

static int fts_ts_remove(struct spi_device *spi)
{
    struct fts_ts_data *ts_data = spi_get_drvdata(spi);

    FTS_FUNC_ENTER();

#if FTS_POINT_REPORT_CHECK_EN
    fts_point_report_check_exit(ts_data);
#endif

#if FTS_APK_NODE_EN
    fts_release_apk_debug_channel(ts_data);
#endif

#if FTS_SYSFS_NODE_EN
    fts_remove_sysfs(&spi->dev);
#endif

#if FTS_GESTURE_EN
    fts_gesture_exit(ts_data);
#endif

    fts_ex_mode_exit(&spi->dev);

#if FTS_TEST_EN
    fts_test_exit(&spi->dev);
#endif

#if FTS_ESDCHECK_EN
    fts_esdcheck_exit(ts_data);
#endif

    fts_fw_exit();

#if defined(CONFIG_FB)
    if (fb_unregister_client(&ts_data->fb_notif))
        FTS_ERROR("Error occurred while unregistering fb_notifier.");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&ts_data->early_suspend);
#endif

    free_irq(ts_data->irq, ts_data);
    input_unregister_device(ts_data->input_dev);

    if (gpio_is_valid(ts_data->pdata->reset_gpio))
        gpio_free(ts_data->pdata->reset_gpio);

    if (gpio_is_valid(ts_data->pdata->irq_gpio))
        gpio_free(ts_data->pdata->irq_gpio);

    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);

    kfree_safe(ts_data->spibuf);
    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);

    devm_kfree(&spi->dev, ts_data);

    FTS_FUNC_EXIT();
    return 0;
}

static int fts_ts_suspend(struct device *dev)
{
    int ret = 0;
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

    FTS_FUNC_ENTER();
    if (ts_data->suspended) {
        FTS_INFO("Already in suspend state");
        return 0;
    }

    if (ts_data->fw_loading) {
        FTS_INFO("fw upgrade in process, can't suspend");
        return 0;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_suspend();
#endif

#if FTS_GESTURE_EN
    if (fts_gesture_suspend(ts_data) == 0) {
        ts_data->suspended = true;
        return 0;
    }
#endif

    /* TP enter sleep mode */
    ret = fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
    if (ret < 0)
        FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);

    ts_data->suspended = true;
    FTS_FUNC_EXIT();
    return 0;
}

static int fts_ts_resume(struct device *dev)
{
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

    FTS_FUNC_ENTER();
    if (!ts_data->suspended) {
        FTS_DEBUG("Already in awake state");
        return 0;
    }

    fts_release_all_finger();
    fts_tp_state_recovery();

#if FTS_ESDCHECK_EN
    fts_esdcheck_resume();
#endif

#if FTS_GESTURE_EN
    if (fts_gesture_resume(ts_data) == 0) {
        ts_data->suspended = false;
        return 0;
    }
#endif

    ts_data->suspended = false;

    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
* SPI Driver
*****************************************************************************/
static const struct spi_device_id fts_ts_id[] = {
    {FTS_DRIVER_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(spi, fts_ts_id);

static struct of_device_id fts_match_table[] = {
    { .compatible = "focaltech,fts", },
    { },
};

static struct spi_driver fts_ts_driver = {
    .probe = fts_ts_probe,
    .remove = fts_ts_remove,
    .driver = {
        .name = FTS_DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = fts_match_table,
    },
    .id_table = fts_ts_id,
};

static int __init fts_ts_init(void)
{
    int ret = 0;
#if 0	
if (0) {    
	char *temp = NULL;
	char * cmdline_tp = NULL;
	int ctpmodule = -1;
	cmdline_tp = strstr(saved_command_line,"qcom,mdss_dsi_ft8006p_");
	FTS_INFO("cmdline_tp = %s\n",cmdline_tp);
	if ( cmdline_tp == NULL ){
		FTS_INFO("get qcom,mdss_dsi_ft8006p_ fail ");
		return -1;
	}
	temp = cmdline_tp + strlen("qcom,mdss_dsi_ft8006p_");
	FTS_INFO("temp = %s\n",temp);
	ctpmodule = strncmp(temp,"truly",strlen("truly"));
	if ( ctpmodule == 0 ){
		FTS_INFO("this is truly touchscreen\n");
	}
	else {
		FTS_INFO("No ft8006p panel\n");
	}
}
#endif
    FTS_FUNC_ENTER();
    ret = spi_register_driver(&fts_ts_driver);
    if ( ret != 0 ) {
        FTS_ERROR("Focaltech touch screen driver init failed!");
    }
    FTS_FUNC_EXIT();
    return ret;
}

static void __exit fts_ts_exit(void)
{
    FTS_FUNC_ENTER();
    spi_unregister_driver(&fts_ts_driver);
    FTS_FUNC_EXIT();
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
