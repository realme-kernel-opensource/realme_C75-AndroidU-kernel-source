ifeq ($(subst ",,$(CONFIG_MTK_SENSOR_ARCHITECTURE)),1.0)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/accelerometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/sensorHub/inc_v1
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/$(CONFIG_MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/cm4
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/cm4
else
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/mtk_nanohub
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/mtk_nanohub/nanohub
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/core
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/$(CONFIG_MTK_PLATFORM)
subdir-ccflags-y += -D CONFIG_OPLUS_SENSOR_MTK68XX
ifeq ($(findstring k419, $(TARGET_PRODUCT)), k419)
subdir-ccflags-y += -D LINUX_KERNEL_VERSION_419
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/include
endif
ifeq ($(CONFIG_MTK_TINYSYS_SCP_RV_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/rv
else ifeq ($(CONFIG_MTK_TINYSYS_SCP_CM4_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/cm4
endif
endif
obj-$(CONFIG_SYLVIA_FEATURE_SENSOR_TEST)	+= sensor_devinfo_v2.o
obj-$(CONFIG_OPLUS_SENSOR_FB_MTK) += oplus_sensor_feedback/
