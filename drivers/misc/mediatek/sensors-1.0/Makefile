$(warning Sylvia get def MTK $(CONFIG_MTK_SENSOR_SUPPORT))
$(warning Sylvia get def SENSOR $(OPLUS_FEATURE_SENSOR))
$(warning Sylvia get def SYLVIA1 $(CONFIG_SYLVIA_FEATURE_SENSOR_TEST))
$(warning Sylvia get def SYLVIA2 $(CONFIG_SYLVIA_REPLACE_ORIGING))
$(warning Sylvia get def SYLVIA3 $(CONFIG_SYLVIA_REPLACE_ORIGING_ALGO))
ifneq ($(CONFIG_MTK_SENSOR_SUPPORT), n)
obj-$(CONFIG_CUSTOM_KERNEL_SENSORHUB)	+= sensorHub/
obj-$(CONFIG_MTK_HWMON)	    += hwmon/
obj-y += sensor_probe/
obj-$(CONFIG_CUSTOM_KERNEL_ALSPS) += alsps/
ifeq ($(CONFIG_CUSTOM_KERNEL_ACCELGYRO),y)
obj-y += accelgyro/
else
obj-$(CONFIG_CUSTOM_KERNEL_ACCELEROMETER) += accelerometer/
obj-$(CONFIG_CUSTOM_KERNEL_GYROSCOPE) += gyroscope/
endif
obj-$(CONFIG_CUSTOM_KERNEL_MAGNETOMETER) += magnetometer/
obj-$(CONFIG_CUSTOM_KERNEL_BAROMETER) += barometer/
obj-$(CONFIG_CUSTOM_KERNEL_STEP_COUNTER)	+= step_counter/
obj-$(CONFIG_CUSTOM_KERNEL_SIGNIFICANT_MOTION_SENSOR)	+= step_counter/
obj-$(CONFIG_CUSTOM_KERNEL_SITUATION) += situation/
obj-$(CONFIG_CUSTOM_KERNEL_SENSOR_FUSION) += sensorfusion/
obj-$(CONFIG_CUSTOM_KERNEL_REAR_ALS) += rear_als/
obj-$(CONFIG_CUSTOM_KERNEL_REAR_FLICKER) += rear_flicker/
#if(1) //OPLUS_FEATURE_SENSOR
obj-y += oplus_sensor_devinfo/
#endif //OPLUS_FEATURE_SENSOR
#if(1) //OPLUS_FEATURE_SENSORR_ALGORITHM
#ifneq ($(OPLUS_FEATURE_SENSOR_ALGORITHM),)
obj-y += oplus_virtual_sensor/
#endif
#endif //OPLUS_FEATURE_SENSOR_ALGORITHM
else
obj-y += dummy.o
endif
