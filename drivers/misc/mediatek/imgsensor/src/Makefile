# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2020 MediaTek Inc.

# MTK_PLATFORM := isp4 isp6s

subdir-ccflags-y += \
	-I$(srctree)/drivers/misc/mediatek/include/ \
	-I$(srctree)/drivers/i2c/busses/ \
	-I$(srctree)/drivers/misc/mediatek/ccu/inc \
	-I$(srctree)/drivers/misc/mediatek/imgsensor/inc \
	-I$(srctree)/drivers/misc/mediatek/cam_cal/inc \

obj-$(CONFIG_MTK_IMGSENSOR_ISP4) += isp4/
obj-$(CONFIG_MTK_IMGSENSOR) += isp6s/

subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/imgsensor/src/OplusImgsensor/
