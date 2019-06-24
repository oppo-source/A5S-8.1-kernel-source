#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

obj-$(CONFIG_NFC_CHIP_SUPPORT) += st21nfc/

ifeq ($(ODM_WT_EDIT), yes)
#Nan.Zhang@ODM_WT.WCN.NFC.Basic, 2018/10/17, Modify for NFC bringup
obj-$(CONFIG_NFC_PN553_DEVICES) += pn553/
endif

ifeq ($(ODM_WT_EDIT), yes)
#Nan.Zhang@ODM_WT.WCN.NFC.Basic, 2018/10/30, Modify for NFC SPI OMA
obj-$(CONFIG_NXP_P73_DEVICES) += p61/
endif