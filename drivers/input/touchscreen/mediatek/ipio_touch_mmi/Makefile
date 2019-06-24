#
# Makefile for the ipio touchscreen drivers.
#
# Each configuration option enables a list of files.
# include header files for MTK
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ipio_touch_driver/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -Wall

obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  platform.o
obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  userspace.o
obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  core/config.o
obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  core/finger_report.o
obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  core/firmware.o
obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  core/flash.o
obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  core/i2c.o
obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  core/spi.o
obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  core/mp_test.o
obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  core/protocol.o
obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  core/parser.o
obj-$(CONFIG_TOUCHSCREEN_ILI9881H) +=  core/gesture.o

