// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

#include <linux/module.h>

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_ACCELEROMETER)
#include "accel.h"
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_GYROSCOPE)
#include "gyroscope.h"
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_MAGNETOMETER)
#include "mag.h"
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_ALSPS)
#include "alsps.h"
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_BAROMETER)
#include "barometer.h"
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_STEP_COUNTER)
#include "step_counter.h"
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_SITUATION)
#include "situation.h"
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_SENSOR_FUSION)
#include "fusion.h"
#endif

#if IS_ENABLED(CONFIG_SYLVIA_FEATURE_SENSOR_TEST)
#include "rear_als.h"
#endif

#if IS_ENABLED(CONFIG_SYLVIA_FEATURE_SENSOR_TEST)
#include "rear_flicker.h"
#endif
/*Sylvia add*/
#include "virtual_sensor_api.h"
/*Sylvia add end*/

static int __init sensor_init(void)
{
#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_ACCELEROMETER)
	pr_err("%s:Start init acc", __func__);
	if (acc_probe())
		pr_err("failed to register acc driver\n");
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_GYROSCOPE)
	pr_err("%s:Start init gyro", __func__);
	if (gyro_probe())
		pr_err("failed to register gyro driver\n");
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_MAGNETOMETER)
	pr_err("%s:Start init mag", __func__);
	if (mag_probe())
		pr_err("failed to register mag driver\n");
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_ALSPS)
	pr_err("%s:Start init alsps", __func__);
	if (alsps_probe())
		pr_err("failed to register alsps driver\n");
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_BAROMETER)
	pr_err("%s:Start init baro", __func__);
	if (baro_probe())
		pr_err("failed to register baro driver\n");
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_STEP_COUNTER)
	pr_err("%s:Start init step_c", __func__);
	if (step_c_probe())
		pr_err("failed to register step_c driver\n");
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_SITUATION)
	pr_err("%s:Start init situ", __func__);
	if (situation_probe()) {
		pr_err("failed to register situ driver\n");
		return -ENODEV;
	}
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_SENSOR_FUSION)
	pr_err("%s:Start init fusion", __func__);
	if (fusion_probe()) {
		pr_err("failed to register fusion driver\n");
		return -ENODEV;
	}
#endif
/*Sylvia add
	sylvia_control_alshub_probe();
	sylvia_control_flickerhub_probe();
Sylvia add end*/
#if IS_ENABLED(CONFIG_SYLVIA_FEATURE_SENSOR_TEST)
	pr_err("%s:Start init rear als", __func__);
	if (rear_als_probe()) {
		pr_err("failed to register rear als driver\n");
		return -ENODEV;
	}
#endif

#if IS_ENABLED(CONFIG_SYLVIA_FEATURE_SENSOR_TEST)
	pr_err("%s:Start init rear flicker", __func__);
	if (rear_flicker_probe()) {
		pr_err("failed to register rear flicker driver\n");
		return -ENODEV;
	}
#endif

#if IS_ENABLED(CONFIG_SYLVIA_FEATURE_SENSOR_TEST)
	pr_err("%s:Start init virtual sensor", __func__);
	if (virtual_sensor_probe()) {
		pr_err("failed to register virtual sensor driver\n");
		return -ENODEV;
	}
#endif
	return 0;
}

static void __exit sensor_exit(void)
{
#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_ACCELEROMETER)
	acc_remove();
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_GYROSCOPE)
	gyro_remove();
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_MAGNETOMETER)
	mag_remove();
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_ALSPS)
	alsps_remove();
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_BAROMETER)
	baro_remove();
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_STEP_COUNTER)
	step_c_remove();
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_SITUATION)
	situation_remove();
#endif

#if IS_ENABLED(CONFIG_CUSTOM_KERNEL_SENSOR_FUSION)
	fusion_remove();
#endif

#if IS_ENABLED(CONFIG_SYLVIA_FEATURE_SENSOR_TEST)
	rear_als_remove();
#endif

#if IS_ENABLED(CONFIG_SYLVIA_FEATURE_SENSOR_TEST)
	rear_flicker_remove();
#endif

#if IS_ENABLED(CONFIG_SYLVIA_FEATURE_SENSOR_TEST)
	virtual_sensor_remove();
#endif
}

late_initcall(sensor_init);
module_exit(sensor_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SensorProbe driver");
MODULE_AUTHOR("Mediatek");

