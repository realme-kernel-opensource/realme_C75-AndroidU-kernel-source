// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

#define MAX_EEPROM_SIZE_32K 0x8000
#define MAX_EEPROM_SIZE_16K 0x4000
#define MAX_EEPROM_SIZE_8K  0x2000

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
	{HI1339_SENSOR_ID, 0xB0, Common_read_region},
	{OV13B10LZ_SENSOR_ID, 0xB0, Common_read_region},
	{GC5035_SENSOR_ID,  0x7E, Common_read_region},
	{HI1339SUBOFILM_SENSOR_ID, 0xA2, Common_read_region},
	{HI1339SUBTXD_SENSOR_ID, 0xA2, Common_read_region},
	{OV48B_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX766_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX766DUAL_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID, 0xA0, Common_read_region},
	{IMX481_SENSOR_ID, 0xA2, Common_read_region},
	{IMX586_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX576_SENSOR_ID, 0xA2, Common_read_region},
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3M5SX_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
	{IMX709_SENSOR_ID, 0xA8, Common_read_region},
	{IMX766_SENSOR_ID_21641, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX355_SENSOR_ID_21641, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID_21641, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02M1_SENSOR_ID_21641, 0xA4, Common_read_region},
	{IMX766_SENSOR_ID_21861, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX355_SENSOR_ID_21861, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID_21861, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV64B_SENSOR_ID_21143, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID_21143, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID_21143, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02M1_SENSOR_ID_21143, 0xA4, Common_read_region, MAX_EEPROM_SIZE_8K},
	{OV64B_SENSOR_ID_22823, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355_SENSOR_ID_22823, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID_22823, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02M1_SENSOR_ID_22823, 0xA4, Common_read_region, MAX_EEPROM_SIZE_8K},
    {OV64B_SENSOR_ID_22801, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	/*Cam.Drv add for Doki otp BringUp 20220523*/
	{S5KJN1_SENSOR_ID_DOKI, 0xA0, Common_read_region},
	{OV16A1Q_SENSOR_ID_DOKI, 0xA8, Common_read_region},
	{S5KHM6SP_SENSOR_ID_DOKI, 0xA0, Common_read_region},
	{OV16A1Q_SENSOR_ID_ADOKI, 0xA8, Common_read_region},
	/*changzheng*/
	{OV32C_SENSOR_ID_CHANGZHENG, 0xA0, Common_read_region},
	{GC02M1_SENSOR_ID_CHANGZHENG, 0xA4, Common_read_region},
	{OVA0B4_SENSOR_ID_CHANGZHENG, 0xA0, Common_read_region},
	{S5KHM6S_SENSOR_ID_CHANGZHENG, 0xA0, Common_read_region},
	/*sonic start*/
	{HI846_SENSOR_ID_SONIC, 0X46, Hi846_read_region},
	{HI556_SENSOR_ID_SONIC, 0X40, Hi556_read_region},
	/*sonic end*/
	/*bluey*/
	{HI846_SENSOR_ID_MAIN_BLUEY, 0xA0, Common_read_region},
	{GC08A3_SENSOR_ID_BLUEY, 0xB0, Common_read_region},
	{HI846SUB_SENSOR_ID_FRONT_BLUEY, 0xA8, Common_read_region},
	{SC820CS_SENSOR_ID_BLUEY, 0xA0, Common_read_region},
	/*sonic-s start*/
	{HI846_SENSOR_ID_24695, 0X46, Hi846_read_region},
	{HI556_SENSOR_ID_24695, 0X40, Hi556_read_region},
	/*sonic-s end*/
	{OV50D40_SENSOR_ID_ORIS, 0xA0, Common_read_region, MAX_EEPROM_SIZE_8K},
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


