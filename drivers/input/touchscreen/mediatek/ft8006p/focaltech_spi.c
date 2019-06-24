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

/************************************************************************
*
* File Name: focaltech_spi.c
*
*    Author: FocalTech Driver Team
*
*   Created: 2017-11-06
*
*  Abstract: spi communication with TP
*
*   Version: v1.0
*
* Revision History:
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define STATUS_PACKAGE              (0x05)
#define COMMAND_PACKAGE             (0xC0)
#define DATA_PACKAGE                (0x3F)
#define BUSY_QUERY_TIMEOUT          (100)
#define BUSY_QUERY_DELAY            (30) /* unit: us */
#define CS_HIGH_DELAY               (30) /* unit: us */

#define DATA_CRC_EN                 (0x20)
#define WRITE_CMD                   (0x00)
#define READ_CMD                    ((0x80) + DATA_CRC_EN)

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct special_cmd {
    u8 cmd;
    u16 cmd_len;
};

/*****************************************************************************
* Static variables
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static struct special_cmd special_cmd_list[] = {
    {0xAE, 6},
    {0x85, 6},
    {0xCC, 7},
    {0xF2, 6},
};

/*****************************************************************************
* functions body
*****************************************************************************/
static void crckermit(u8 *data, u16 len, u16 *crc_out)
{
    u16 i = 0;
    u16 j = 0;
    u16 crc = 0xFFFF;

    for ( i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc = (crc >> 1);
        }
    }

    *crc_out = crc;
}

static int rdata_check(u8 *rdata, u32 rlen)
{
    u16 crc_calc = 0;
    u16 crc_read = 0;

    crckermit(rdata, rlen - 2, &crc_calc);
    crc_read = (u16)(rdata[rlen - 1] << 8) + rdata[rlen - 2];
    if (crc_calc != crc_read) {
        FTS_INFO("spi crc check fail,calc:%04x read:%04x", crc_calc, crc_read);
        FTS_INFO("buf:%x %x %x %x", rdata[0], rdata[1], rdata[2], rdata[3]);
        return -EIO;
    }

    return 0;
}
/*
 * description: check cmd is boot command or not, if fw is running, no need
 *
 * param - buf : write data buffer
 *         cmdlen: special command length,output
 *
 */
static void fts_get_specail_cmdlen(u8 *buf, u32 *cmdlen)
{
    int i = 0;
    int list_len = sizeof(special_cmd_list) / sizeof(special_cmd_list[0]);

    /* default cmdlen is 1 */
    *cmdlen = 1;
    for (i = 0; i < list_len; i++) {
        if (buf[0] == special_cmd_list[i].cmd) {
            *cmdlen = special_cmd_list[i].cmd_len;
        }
    }
}

static int fts_spi_write(u8 *buf, u32 len)
{
    int ret = 0;
    struct spi_device *spi = fts_data->spi;
    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = buf,
        .len = len,
    };

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(spi, &msg);
    if (ret < 0) {
        FTS_ERROR("spi_sync(write) msg fail,ret:%d", ret);
        return ret;
    }

    return 0;
}

static int fts_spi_read(u8 *buf, u32 len)
{
    int ret = 0;
    struct spi_device *spi = fts_data->spi;
    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = buf,
        .rx_buf = buf,
        .len = len,
    };

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(spi, &msg);
    if (ret < 0) {
        FTS_ERROR("spi_sync(read) msg fail,ret:%d", ret);
        return ret;
    }

    return 0;
}

int fts_read_status(u8 *status)
{
    int ret = 0;
    u8 status_cmd[2] = {STATUS_PACKAGE, 0xFF};
    struct spi_device *spi = fts_data->spi;
    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = status_cmd,
        .rx_buf = status_cmd,
        .len = 2,
    };

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(spi, &msg);
    if (ret < 0) {
        FTS_ERROR("spi_sync(status) fail,ret:%d", ret);
        return ret;
    }

    *status = status_cmd[1];
    return ret;
}

/*
 * description: wait IC idle(spi)
 *
 * return status if success, otherwise return error code
 */
static int fts_wait_idle(void)
{
    int ret = 0;
    int i = 0;
    u8 status = 0xFF;

    for (i = 0; i < BUSY_QUERY_TIMEOUT; i++) {
        udelay(BUSY_QUERY_DELAY);
        ret = fts_read_status(&status);
        if ((ret >= 0) && (0x01 == (status & 0x81))) {
            break;
        } else {
            //FTS_DEBUG("spi status:0x%x", status);
        }
    }

    if (i >= BUSY_QUERY_TIMEOUT) {
        FTS_ERROR("spi is busy status(0x%x)", status);
        return -EIO;
    }

    udelay(CS_HIGH_DELAY);
    return 0;
}

/*
 * description: write command package to ic
 *
 * param - ctrl: ctrl byte,control read/wrte...
 *         cmd: detail command buffer
 *         len: length of cmd buffer
 *
 * return 0 if success, otherwise return error code
 */
static int fts_cmd_wirte(u8 ctrl, u8 *cmd, u32 len)
{
    int i = 0;
    int pos = 0;
    u8 buf[SPI_MAX_COMMAND_LENGTH] = { 0 };

    if (len >= SPI_MAX_COMMAND_LENGTH - SPI_HEADER_LENGTH) {
        FTS_ERROR("command length(%d) fail", len);
        return -EINVAL;
    }

    if (NULL == cmd) {
        FTS_ERROR("command is null");
        return -EINVAL;
    }

    buf[pos++] = COMMAND_PACKAGE;
    buf[pos++] = ctrl | (len & 0x0F);
    for (i = 0; i < len; i++) {
        buf[pos++] = cmd[i];
    }

    return fts_spi_write(buf, pos);
}

/*
 * description: write cmd&data to ic, used in romboot environment
 *
 * param - cmd: detail command buffer
 *         len: length of cmd buffer
 *         data: data buffer, maybe NULL
 *         datalen: length of data buffer, maybe 0
 *
 * return 0 if success, otherwise return error code
 */
static int fts_boot_write(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
    int ret = 0;
    u8 ctrl = WRITE_CMD;
    u16 crc = 0;
    u8 *txbuf = NULL;
    u32 txlen = 0;

    if ((!cmd) || (0 == cmdlen)) {
        FTS_ERROR("cmd/cmdlen(%d) is invalid", cmdlen);
        return -EINVAL;
    }

    mutex_lock(&fts_data->spilock);

    /* wait spi idle */
    ret = fts_wait_idle();
    if (ret < 0) {
        FTS_ERROR("wait spi idle fail");
        goto err_boot_write;
    }

    /* write cmd */
    ret = fts_cmd_wirte(ctrl, cmd, cmdlen);
    if (ret < 0) {
        FTS_ERROR("command package wirte fail");
        goto err_boot_write;
    }

    /* have data, transfer data */
    if (data && datalen) {
        /* wait spi idle */
        ret = fts_wait_idle();
        if (ret < 0) {
            FTS_ERROR("wait spi idle from cmd fail");
            goto err_boot_write;
        }

        /* write data */
        if (datalen > SPI_BUF_LENGTH - SPI_HEADER_LENGTH) {
            txbuf = kzalloc(datalen + SPI_HEADER_LENGTH, GFP_KERNEL);
            if (NULL == txbuf) {
                FTS_ERROR("txbuf kzalloc fail");
                ret = -ENOMEM;
                goto err_boot_write;
            }
        } else {
            txbuf = fts_data->spibuf;
        }
        memset(txbuf, 0xFF, datalen + SPI_HEADER_LENGTH);
        txbuf[0] = DATA_PACKAGE;
        memcpy(&txbuf[1], data, datalen);
        txlen = datalen + 1;
        if (ctrl & DATA_CRC_EN) {
            /* calc data crc */
            crckermit(&txbuf[1], datalen, &crc);
            txbuf[txlen++] = crc & 0xFF;
            txbuf[txlen++] = (crc >> 8) & 0xFF;
        }

        ret = fts_spi_write(txbuf, txlen);
        if (ret < 0) {
            FTS_ERROR("data wirte fail");
        }

        if (txbuf && (datalen > SPI_BUF_LENGTH - SPI_HEADER_LENGTH)) {
            kfree(txbuf);
            txbuf = NULL;
        }
    }

err_boot_write:
    mutex_unlock(&fts_data->spilock);
    return ret;
}

/*
 * description: write data to addr, used in fw environment
 *
 * param - addr: register address to write
 *         data: data buffer
 *         datalen: length of data buffer
 *
 * return 0 if success, otherwise return error code
 */
static int fts_fw_write(u8 addr, u8 *data, u32 datalen)
{
    u8 cmd[3] = { 0 };

    cmd[0] = addr;
    cmd[1] = (datalen >> 8) & 0xFF;
    cmd[2] = datalen & 0xFF;

    return fts_boot_write(cmd, 3, data, datalen);
}

/*
 * description: write data to ic
 *
 * param - writebuf: data buffer, must have two data at least if fw is running
 *         writelen: length of data buffer, >= 2 if fw is running
 *
 * return 0 if success, otherwise return error code
 */
int fts_write(u8 *writebuf, u32 writelen)
{
    int ret = 0;
    u32 cmdlen = 0;

    if ((NULL == writebuf) || (0 == writelen)) {
        FTS_ERROR("writebuf is null/writelen is 0");
        return -EINVAL;
    }

    if (1 == writelen) {
        /* writelen is 1, must be boot command */
        ret = fts_boot_write(writebuf, writelen, NULL, 0);
    } else {
        if (true == fts_data->fw_is_running) {
            ret = fts_fw_write(writebuf[0], writebuf + 1, writelen - 1);
        } else {
            fts_get_specail_cmdlen(writebuf, &cmdlen);
            if ((0 == cmdlen) || (cmdlen > writelen)) {
                FTS_ERROR("writelen(%d<%d) is short", writelen, cmdlen);
                return -EINVAL;
            } else if (cmdlen == writelen) {
                ret = fts_boot_write(writebuf, cmdlen, NULL, 0);
            } else {
                ret = fts_boot_write(writebuf, cmdlen, writebuf + cmdlen, writelen - cmdlen);
            }
        }
    }

    return ret;
}

int fts_write_reg(u8 regaddr, u8 regvalue)
{
    return fts_fw_write(regaddr, &regvalue, 1);
}

static int fts_boot_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
    int ret = 0;
    u8 ctrl = READ_CMD;
    u8 *txbuf = NULL;
    u32 txlen = 0;

    if ((!data) || (0 == datalen)) {
        FTS_ERROR("data/datalen is invalid");
        return -EINVAL;
    }

    mutex_lock(&fts_data->spilock);
    if (cmd && cmdlen) {
        /* wait spi idle */
        ret = fts_wait_idle();
        if (ret < 0) {
            FTS_ERROR("wait spi idle fail");
            goto boot_read_err;
        }

        /* write cmd */
        ret = fts_cmd_wirte(ctrl, cmd, cmdlen);
        if (ret < 0) {
            FTS_ERROR("command package wirte fail");
            goto boot_read_err;
        }

        /* wait spi idle */
        ret = fts_wait_idle();
        if (ret < 0) {
            FTS_ERROR("wait spi idle from cmd fail");
            goto boot_read_err;
        }
    }

    /* write data */
    if (datalen > SPI_BUF_LENGTH - SPI_HEADER_LENGTH) {
        txbuf = kzalloc(datalen + SPI_HEADER_LENGTH, GFP_KERNEL);
        if (NULL == txbuf) {
            FTS_ERROR("txbuf kzalloc fail");
            ret = -ENOMEM;
            goto boot_read_err;
        }
    } else {
        txbuf = fts_data->spibuf;
    }
    memset(txbuf, 0xFF, datalen + SPI_HEADER_LENGTH);
    txbuf[0] = DATA_PACKAGE;
    txlen = datalen + 1;
    if (ctrl & DATA_CRC_EN) {
        txlen = txlen + 2;
    }
    ret = fts_spi_read(txbuf, txlen);
    if (ret < 0) {
        FTS_ERROR("data read fail");
        goto boot_read_err;
    }

    memcpy(data, &txbuf[1], datalen);
    /* crc check */
    if (ctrl & DATA_CRC_EN) {
        ret = rdata_check(&txbuf[1], txlen - 1);
        if (ret < 0) {
            FTS_INFO("read data crc check fail");
            goto boot_read_err;
        }
    }

boot_read_err:
    if (txbuf && (datalen > SPI_BUF_LENGTH - SPI_HEADER_LENGTH)) {
        kfree(txbuf);
        txbuf = NULL;
    }
    mutex_unlock(&fts_data->spilock);

    return ret;
}

static int fts_fw_read(u8 addr, u8 *data, u32 datalen)
{
    u8 cmd[3] = { 0 };

    cmd[0] = addr;
    cmd[1] = (datalen >> 8) & 0xFF;
    cmd[2] = datalen & 0xFF;

    return fts_boot_read(cmd, 3, data, datalen);
}

/*
 * description: read data from ic
 *
 * param - writebuf: cmd buffer
 *         writelen: length of cmd buffer
 *         readbuf: read data buffer
 *         readlen: length of read data buffer
 *
 * return 0 if success, otherwise return error code
 */
int fts_read(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen)
{
    int ret = 0;

    if ((NULL == readbuf) || (0 == readlen)) {
        FTS_ERROR("readbuf/readlen is invalid");
        return -EINVAL;
    }

    if ((NULL == writebuf) || (0 == writelen)) {
        ret = fts_boot_read(NULL, 0, readbuf, readlen);
    } else {
        if (true == fts_data->fw_is_running) {
            ret = fts_fw_read(writebuf[0], readbuf, readlen);
        } else {
            ret = fts_boot_read(writebuf, writelen, readbuf, readlen);
        }
    }

    return ret;
}

int fts_read_reg(u8 regaddr, u8 *regvalue)
{
    return fts_fw_read(regaddr, regvalue, 1);
}
