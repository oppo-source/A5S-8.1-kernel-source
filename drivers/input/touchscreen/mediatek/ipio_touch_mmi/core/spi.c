/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - spi.c
** Description : This program is for ili9881 driver spi.h
** Version: 1.0
** Date : 2018/5/17
** Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
**
** -------------------------Revision History:----------------------------------
**  <author>	 <data> 	<version >			<desc>
**
**
*******************************************************************************/

#include "../common.h"
#include "config.h"
#include "spi.h"
#include "finger_report.h"

struct core_spi_data *core_spi;
extern int core_ice_mode_disable_9881H11(void);
int core_ice_mode_enable_9881H11(void);
int core_ice_mode_disable_9881H11(void);
#ifdef ILITEK_ESD_CHECK
int core_config_get_esd_data(void)
{
	uint8_t txbuf[5] = {0},rxbuf[2]= {0};
	int res = 0;
	txbuf[0] = 0x82;
	txbuf[1] = 0x1F;
	txbuf[2] = 0x62;
	txbuf[3] = 0x10;
	txbuf[4] = 0x18;
	core_ice_mode_disable_9881H11();
	if (spi_write_then_read(core_spi->spi, txbuf, 1, rxbuf, 1) < 0) {
		res = -EIO;
		ipio_err("spi Write Error, res = %d\n", res);
		return res;
	}
	//check recover data
	ipio_debug(DEBUG_I2C, "core_config_get_esd_data rxbuf:0x%x\n", rxbuf[0]);
	if(rxbuf[0] != 0xA3)
	{
		ipio_err("rxbuf:0x%x\n", rxbuf[0]);
		return CHECK_RECOVER;
	}
	return SUCCESS;
}
EXPORT_SYMBOL(core_config_get_esd_data);

#endif

int core_Rx_check(uint16_t check)
{
	int size = 0, i, count = 100;
	uint8_t txbuf[5] = { 0 }, rxbuf[4] = {0};
	uint16_t status = 0;
	for(i = 0; i < count; i++) {
		size = core_ice_mode_enable_9881H11();
		if (size < 0) {
			ipio_err("enter ice mode error\n");
			return size;
		}
		txbuf[0] = SPI_WRITE;
		txbuf[1] = 0x25;
		txbuf[2] = 0x94;
		txbuf[3] = 0x0;
		txbuf[4] = 0x2;

		if (spi_write_then_read(core_spi->spi, txbuf, 5, txbuf, 0) < 0) {
			size = -EIO;
			ipio_err("spi Write Error, res = %d\n", size);
		}

		txbuf[0] = SPI_READ;

		if (spi_write_then_read(core_spi->spi, txbuf, 1, rxbuf, 4) < 0) {
			size = -EIO;
			ipio_err("spi Read Error, res = %d\n", size);
		}

		status = (rxbuf[2] << 8) + rxbuf[3];
		size = (rxbuf[0] << 8) + rxbuf[1];

		//ipio_info("status =%d\n",status);
		if(status == check) {
		ipio_debug(DEBUG_FINGER_REPORT, "get rx lock retry count = %d\n", i);
			return size;
		}
		mdelay(1);
		if (core_ice_mode_disable_9881H11() < 0) {
			size = -EIO;
			ipio_err("exit ice mode error\n");
			return size;
		}
	}

	size = -EIO;
	ipio_err("Check lock error,status = %x,rxbuf[0] = %x,rxbuf[1] = %x,rxbuf[2] = %x,rxbuf[3] = %x,size = %x\n",status,rxbuf[0],rxbuf[1],rxbuf[2],rxbuf[3],size);
	return size;
}
int core_Tx_unlock_check(void)
{
	int res = 0, i, count = 100;
	uint8_t txbuf[5] = { 0 }, rxbuf[4] = {0};
	uint16_t unlock = 0;
	for(i = 0; i < count; i++)
	{
		txbuf[0] = SPI_WRITE;
		txbuf[1] = 0x25;
		txbuf[2] = 0x0;
		txbuf[3] = 0x0;
		txbuf[4] = 0x2;
		if (spi_write_then_read(core_spi->spi, txbuf, 5, txbuf, 0) < 0) {
			res = -EIO;
			ipio_err("spi Write Error, res = %d\n", res);
		}
		txbuf[0] = SPI_READ;
		if (spi_write_then_read(core_spi->spi, txbuf, 1, rxbuf, 4) < 0) {
			res = -EIO;
			ipio_err("spi Read Error, res = %d\n", res);
		}
		unlock = (rxbuf[2] << 8) + rxbuf[3];

		if(unlock == 0x9881) {
			ipio_debug(DEBUG_FINGER_REPORT, "get tx unlock retry count = %d\n", i);
			return res;
		}
		mdelay(1);
	}
	res = -ETXTBSY;
	ipio_err("Check unlock error,unlock = %d,rxbuf[0] = %d,rxbuf[1] = %d,rxbuf[2] = %d,rxbuf[3] = %d\n",unlock,rxbuf[0],rxbuf[1],rxbuf[2],rxbuf[3]);
	return res;
}
int core_spi_read_bytes_ree(u32 data_len, u8 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;
	u8 tx_data = 0x83;

	xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);

	if (xfer == NULL) {
		ipio_err("%s,no memory for SPI transfer\n", __func__);
		return -ENOMEM;
	}

	spi_message_init(&msg);
	xfer[0].tx_buf = &tx_data;
	xfer[0].len = 1;
	spi_message_add_tail(&xfer[0], &msg);

	xfer[1].rx_buf = rx_buf;
	xfer[1].len = data_len;
	ipio_debug(1, "spi read len:%d\n", data_len);
	spi_message_add_tail(&xfer[1], &msg);
	spi_sync(core_spi->spi, &msg);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}
int core_ice_mode_read_9881H11(uint8_t *data, uint32_t size)
{
	int res = 0;
	uint8_t txbuf[64] = { 0 };
	//set read address
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x98;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	if (spi_write_then_read(core_spi->spi, txbuf, 5, txbuf, 0) < 0) {
		res = -EIO;
		return res;
	}
	//read data
	txbuf[0] = SPI_READ;
	if(size > 32) {
		if (core_spi_read_bytes_ree(size, data) < 0){
			res = -EIO;
			return res;
		}
	} else{
	if (spi_write_then_read(core_spi->spi, txbuf, 1, data, size) < 0) {
		res = -EIO;
		return res;
	}
	}
	/* write data unlock */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x94;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	txbuf[5] = (size & 0xFF00) >> 8;
	txbuf[6] = size & 0xFF;
	txbuf[7] = (char)0x98;
	txbuf[8] = (char)0x81;
	if (spi_write_then_read(core_spi->spi, txbuf, 9, txbuf, 0) < 0) {
		res = -EIO;
		ipio_err("spi Write data unlock error, res = %d\n", res);
	}
	return res;
}

extern uint8_t cal_fr_checksum(uint8_t *pMsg, uint32_t nLength);

int core_ice_mode_write_9881H11(uint8_t *data, uint32_t size)
{
	int res = 0;
	uint8_t check_sum = 0,wsize = 0;
	uint8_t *txbuf;

    txbuf = (uint8_t*)kmalloc(sizeof(uint8_t)*size+9, GFP_KERNEL);
	if (ERR_ALLOC_MEM(txbuf)) {
		ipio_err("Failed to allocate CSV mem\n");
		res = -ENOMEM;
		goto out;
	}

	/* Write data */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x4;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	check_sum = core_fr_calc_checksum(data, size);
	memcpy(txbuf + 5, data, size);
	txbuf[5 + size] = check_sum;
	//size + checksum
	size++;
	wsize = size;
	if(wsize%4 != 0)
		wsize += 4 - (wsize % 4);

	if (spi_write_then_read(core_spi->spi, txbuf, wsize + 5, txbuf, 0) < 0) {
		res = -EIO;
		ipio_err("spi Write Error, res = %d\n", res);
		goto out;
	}
	//write data lock
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x0;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	txbuf[5] = (size & 0xFF00) >> 8;
	txbuf[6] = size & 0xFF;
	txbuf[7] = (char)0x5A;
	txbuf[8] = (char)0xA5;
	if (spi_write_then_read(core_spi->spi, txbuf, 9, txbuf, 0) < 0) {
		res = -EIO;
		ipio_err("spi Write data lock Error, res = %d\n", res);
	}

out:
	kfree(txbuf);
	return res;
}

int core_ice_mode_disable_9881H11(void)
{
	int res = 0;
	uint8_t txbuf[5] = {0};
	txbuf[0] = 0x82;
	txbuf[1] = 0x1B;
	txbuf[2] = 0x62;
	txbuf[3] = 0x10;
	txbuf[4] = 0x18;

	if (spi_write_then_read(core_spi->spi, txbuf, 5, txbuf, 0) < 0) {
		res = -EIO;
		ipio_err("spi_write_then_read Error, res = %d\n", res);
	}
	return res;
}

int core_ice_mode_enable_9881H11(void)
{
	int res = 0;
	uint8_t txbuf[5] = {0}, rxbuf[2]= {0};
	txbuf[0] = 0x82;
	txbuf[1] = 0x1F;
	txbuf[2] = 0x62;
	txbuf[3] = 0x10;
	txbuf[4] = 0x18;
	if (spi_write_then_read(core_spi->spi, txbuf, 1, rxbuf, 1) < 0) {
		res = -EIO;
		ipio_err("spi Write Error, res = %d\n", res);
		return res;
	}
	//check recover data
	if(rxbuf[0] == 0x82)
	{
		ipio_info("rxbuf:0x%x\n", rxbuf[0]);
		return CHECK_RECOVER;
	}
	if (spi_write_then_read(core_spi->spi, txbuf, 5, rxbuf, 0) < 0) {
		res = -EIO;
		ipio_err("spi Write Error, res = %d\n", res);
	}
	return res;
}
int core_spi_check_read_size(void)
{
	int size = 0;
/*	res = core_ice_mode_enable_9881H11();
	if (res < 0) {
		goto out;
	}*/
	size = core_Rx_check(0x5AA5);
	if (size < 0) {
		ipio_err("spi core_Rx_check(0x5AA5) Error, res = %d\n", size);
	}
	return size;
}

int core_spi_read_data_after_checksize(uint8_t *pBuf, uint16_t nSize)
{
	int res = 0;
	if (core_ice_mode_read_9881H11(pBuf, nSize) < 0) {
		res = -EIO;
		ipio_err("spi read Error, res = %d\n", res);
		goto out;
	}
	if (core_ice_mode_disable_9881H11() < 0) {
		res = -EIO;
		ipio_err("spi core_ice_mode_disable_9881H11 Error, res = %d\n", res);
		goto out;
	}
out:
	return res;
}

int core_spi_read_9881H11(uint8_t *pBuf, uint16_t nSize)
{
	int res = 0, size = 0;
/*	res = core_ice_mode_enable_9881H11();
	if (res < 0) {
		goto out;
	}*/
	size = core_Rx_check(0x5AA5);
	if (size < 0) {
		res = -EIO;
		goto out;
	}
	ipio_err("size =========== %d\n ",size);
	if (size >= nSize)
        size = nSize;
	if (core_ice_mode_read_9881H11(pBuf, size) < 0) {
		res = -EIO;
		goto out;
	}
	if (core_ice_mode_disable_9881H11() < 0) {
		res = -EIO;
		goto out;
	}
	out:
	return res;
}

int core_spi_write_9881H11(uint8_t *pBuf, uint16_t nSize)
{
	int res = 0;
	uint8_t *txbuf;
    txbuf = (uint8_t*)kmalloc(sizeof(uint8_t)*nSize+5, GFP_KERNEL);
	if (ERR_ALLOC_MEM(txbuf)) {
		ipio_err("Failed to alllocate txbuf mem %ld\n", PTR_ERR(txbuf));
		return -ENOMEM;
	}
	res = core_ice_mode_enable_9881H11();
	if (res < 0) {
		goto out;
	}
	if (core_ice_mode_write_9881H11(pBuf, nSize) < 0) {
		res = -EIO;
		goto out;
	}
	if(core_Tx_unlock_check() < 0)
	{
		res = -ETXTBSY;
	}
	out:
	kfree(txbuf);
	return res;
}
int core_spi_write(uint8_t *pBuf, uint16_t nSize)
{
	int res = 0, cnt = 5;
	int i;
	uint8_t *txbuf;
    txbuf = (uint8_t*)kmalloc(sizeof(uint8_t)*nSize+1, GFP_KERNEL);
	if (ERR_ALLOC_MEM(txbuf)) {
		ipio_err("Failed to alllocate txbuf mem %ld\n", PTR_ERR(txbuf));
		return -ENOMEM;
	}
	if(core_config->icemodeenable == false)
	{
		for(i = 0;i < cnt;i++) {
		res = core_spi_write_9881H11(pBuf, nSize);
		core_ice_mode_disable_9881H11();
		if(res != -ETXTBSY )
		    break;
		}
		ipio_debug(DEBUG_FINGER_REPORT, "core_spi_write retry count = %d\n", i);
		kfree(txbuf);
		return res;
	}

	txbuf[0] = SPI_WRITE;
    memcpy(txbuf+1, pBuf, nSize);

	if (spi_write_then_read(core_spi->spi, txbuf, nSize+1, txbuf, 0) < 0) {
		if (core_config->do_ic_reset) {
			/* ignore spi error if doing ic reset */
			res = 0;
		} else {
			res = -EIO;
			ipio_err("spi Write Error, res = %d\n", res);
			goto out;
		}
	}

out:
	kfree(txbuf);
	return res;
}
EXPORT_SYMBOL(core_spi_write);

int core_spi_read(uint8_t *pBuf, uint16_t nSize)
{
	int res = 0;
	uint8_t txbuf[1];
	txbuf[0] = SPI_READ;
	if(core_config->icemodeenable == false)
	{
		return core_spi_read_9881H11(pBuf, nSize);
	}
#ifdef ODM_WT_EDIT
//Bin.Su@ODM_WT.BSP.Tp.Function.1372106,2018/05/18,Modify for tp MP test
	if(nSize > 32) {
		if (core_spi_read_bytes_ree(nSize, pBuf) < 0){
			res = -EIO;
			goto out;
		}
	}
	else{
		if (spi_write_then_read(core_spi->spi, txbuf, 1, pBuf, nSize) < 0) {
			res = -EIO;
			goto out;
		}
	}
/*	if (spi_write_then_read(core_spi->spi, txbuf, 1, pBuf, nSize) < 0) {
		if (core_config->do_ic_reset) {
			res = 0;
		} else {
			res = -EIO;
			ipio_err("spi Read Error, res = %d\n", res);
			goto out;
		}
	}*/
#endif
out:
	return res;
}
EXPORT_SYMBOL(core_spi_read);

int core_spi_init(struct spi_device *spi)
{
	int ret;

	core_spi = kmalloc(sizeof(*core_spi), GFP_KERNEL);
	if (ERR_ALLOC_MEM(core_spi)) {
		ipio_err("Failed to alllocate core_i2c mem %ld\n", PTR_ERR(core_spi));
		core_spi_remove();
		return -ENOMEM;
	}

	core_spi->spi = spi;
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if (ret < 0){
		ipio_err("ERR: fail to setup spi\n");
		return -ENODEV;
	}

	ipio_info("%s:name=%s,bus_num=%d,cs=%d,mode=%d,speed=%d\n",__func__,spi->modalias,
	 spi->master->bus_num, spi->chip_select, spi->mode, spi->max_speed_hz);

	return 0;
}
EXPORT_SYMBOL(core_spi_init);

void core_spi_remove(void)
{
	ipio_info("Remove core-spi members\n");
	ipio_kfree((void **)&core_spi);
}
EXPORT_SYMBOL(core_spi_remove);
