ifeq ($(subst ",,$(CONFIG_MTK_SENSOR_ARCHITECTURE)),1.0)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/oplus_virtual_sensor
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/oplus_virtual_sensor/platform/sensor_cmd
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/oplus_virtual_sensor/platform/sensor_device
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/oplus_virtual_sensor/platform/sensor_event
else
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/oplus_virtual_sensor
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/oplus_virtual_sensor/platform/sensor_cmd
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/oplus_virtual_sensor/platform/sensor_device
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/oplus_virtual_sensor/platform/sensor_event
endif

ifeq ($(CONFIG_OPLUS_FEATURE_LUX_AOD),m)
obj-$(CONFIG_OPLUS_FEATURE_LUX_AOD)	:=  lux_aodhub.o
else
obj-$(CONFIG_SYLVIA_FEATURE_SENSOR_TEST)	:=  lux_aodhub.o
endif

