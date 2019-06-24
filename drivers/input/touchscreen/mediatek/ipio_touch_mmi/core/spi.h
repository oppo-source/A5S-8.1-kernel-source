/*******************************************************************************
** Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** FILE: - spi.h
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

#ifndef __SPI_H
#define __SPI_H
#define SPI_WRITE 		0X82
#define SPI_READ 		0X83
struct core_spi_data {
	struct spi_device *spi;
};

extern struct core_spi_data *core_spi;
extern int core_spi_write(uint8_t *pBuf, uint16_t nSize);
extern int core_spi_read(uint8_t *pBuf, uint16_t nSize);
extern int core_spi_init(struct spi_device *spi);
extern void core_spi_remove(void);
extern int core_spi_check_read_size(void);
extern int core_spi_read_data_after_checksize(uint8_t *pBuf, uint16_t nSize);


#ifdef ILITEK_ESD_CHECK
extern int core_config_get_esd_data(void);
#endif

#endif
