#*******************************************************************************
# Copyright (C), 2008-2018, OPPO Mobile Comm Corp., Ltd.
# ODM_WT_EDIT
# FILE: - Kconfig
# Description : This program is for ili9881 driver Makefile
# Version: 1.0
# Date : 2018/5/14
# Author: Zhonghua.Hu@ODM_WT.BSP.TP-FP.TP
#
# -------------------------Revision History:----------------------------------
#  <author>	 <data> 	<version >			<desc>
#
#
#*****************************************************************************/
# include header files for MTK
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ipio_touch_driver/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

ccflags-y += -Wall

obj-y += config.o \
		 i2c.o \
		 firmware.o \
		 finger_report.o \
		 mp_test.o \
		 flash.o \
		 protocol.o \
		 parser.o \
		 gesture.o \
		 spi.o
