/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include "imgsensor_eeprom.h"
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#define DUMP_EEPROM 0
#define EEPROM_SPEED_400K    400
#define EEPROM_SPEED_1000K   1000
struct CAMERA_DEVICE_INFO gImgEepromInfo;
struct CAMERA_DEVICE_INFO gImgEepromInfoOrisC = {
    .i4SensorNum = 2,
    .pCamModuleInfo = {
        {OV50D40_SENSOR_ID_ORIS,  0xA0, {0x00, 0x06}, 0x50, 1, {0x44, 0x44, 0x46, 0x46}, "Cam_r0", "ov50d40_mipi_raw_oris"},
        {HI846_SENSOR_ID_ORIS, 0x40, {0x00, 0x06}, 0x22B, 0, {0xFF,0xFF,0xFF,0xFF}, "Cam_f",  "hi846_mipi_raw_oris"},
    },
    .i4MWDataIdx = 0xFF,
    .i4MTDataIdx = 0xFF,
    .i4FrontDataIdx = 0xFF,
    .i4NormDataLen = 40,
    .i4MWDataLen = 3102,
    .i4MWStereoAddr = {0xFFFF, 0xFFFF},
    .i4MTStereoAddr = {0xFFFF, 0xFFFF},
    .i4FrontStereoAddr = {0xFFFF, 0xFFFF},
};

kal_uint16 Eeprom_1ByteDataRead(kal_uint16 addr, kal_uint16 slaveaddr)
{
    kal_uint16 get_byte = 0;
    kal_uint16 i2c_speed = EEPROM_SPEED_400K; //eeprom transmission rate default is 400k.
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;

    //ov64b sensor eeprom support 1000k transmission rate.
    if (is_project(23035) || is_project(23321)) {
        if (!strcmp(pCamDeviceObj->pCamModuleInfo[pCamDeviceObj->i4CurSensorIdx].version, "ov64b")) {
            i2c_speed = EEPROM_SPEED_1000K;
        }
    }
    iReadRegI2CTiming(pusendcmd , 2, (u8*)&get_byte, 1, slaveaddr, i2c_speed);
    return get_byte;
}

void Eeprom_DataRead(kal_uint8 *uData, kal_uint16 dataAddr,
                            kal_uint16 dataLens, kal_uint16 slaveAddr)
{
    int dataCnt = 0;
    for (dataCnt = 0; dataCnt < dataLens; dataCnt++) {
        uData[dataCnt] = (u8)Eeprom_1ByteDataRead(dataAddr+dataCnt, slaveAddr);
    }
}

enum IMGSENSOR_RETURN Eeprom_SensorInfoValid(
            enum IMGSENSOR_SENSOR_IDX sensor_idx,
            kal_uint32 sensorID)
{
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    if (is_project(24700) || is_project(24701) || is_project(24702)
        || is_project(24709) || is_project(24713) || is_project(24714)){
        pCamDeviceObj = &gImgEepromInfoOrisC;
    }

    if (sensor_idx > pCamDeviceObj->i4SensorNum - 1) {
        pr_info("[%s] sensor_idx:%d > i4SensorNum: %d", __func__, sensor_idx, pCamDeviceObj->i4SensorNum);
        return IMGSENSOR_RETURN_ERROR;
    }

    if ((is_project(0x2260B) || is_project(22669) || is_project(0x2266A) || is_project(22609))
        && (sensorID == pCamDeviceObj->pCamModuleInfo[sensor_idx + 1].i4Sensorid)) {
        return IMGSENSOR_RETURN_SUCCESS;
    }

    if (sensorID != pCamDeviceObj->pCamModuleInfo[sensor_idx].i4Sensorid) {
        pr_info("[%s] sensorID:%d mismatch i4Sensorid: %d",
                __func__, sensorID, pCamDeviceObj->pCamModuleInfo[sensor_idx].i4Sensorid);
        return IMGSENSOR_RETURN_ERROR;
    }

    return IMGSENSOR_RETURN_SUCCESS;
}

void Eeprom_CamSNDataRead(enum IMGSENSOR_SENSOR_IDX sensor_idx, kal_uint16 slaveAddr)
{
    kal_uint16 dataAddr = 0xFFFF, i = 0;
    kal_uint16 snLength = OPLUS_CAMERASN_LENS_23;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
    pr_info("[%s]meng  read  SN lens id %d",
                                __func__, snLength);
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    if (is_project(24700) || is_project(24701) || is_project(24702)
        || is_project(24709) || is_project(24713) || is_project(24714)){
        pCamDeviceObj = &gImgEepromInfoOrisC;
    }

    /*Read camera SN-23bytes*/
    dataAddr = pCamDeviceObj->pCamModuleInfo[sensor_idx].i4CamSNAddr;
    for (i = 0; i < snLength; i++) {
        pCamDeviceObj->camNormdata[sensor_idx][OPLUS_CAMMODULEINFO_LENS+i] =
            (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
    }
}

void Eeprom_CamAFCodeDataRead(enum IMGSENSOR_SENSOR_IDX sensor_idx, kal_uint16 slaveAddr)
{
    kal_uint16 dataAddr = 0xFFFF, dataCnt = 0;
    kal_uint16 i = 0;
    kal_uint16 snLength = OPLUS_CAMERASN_LENS_23;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    if (is_project(24700) || is_project(24701) || is_project(24702)
        || is_project(24709) || is_project(24713) || is_project(24714)){
        pCamDeviceObj = &gImgEepromInfoOrisC;
    }

    /*Read camera SN-23bytes*/
    /*Read AF MAC+50+100+Inf+StereoDac*/
    if (pCamDeviceObj->pCamModuleInfo[sensor_idx].i4AfSupport) {
        dataCnt = OPLUS_CAMMODULEINFO_LENS + snLength;
        for (i = 0; i < 4; i ++) {
            dataAddr = pCamDeviceObj->pCamModuleInfo[sensor_idx].i4AFCodeAddr[i];
            pCamDeviceObj->camNormdata[sensor_idx][dataCnt] =
                (kal_uint8)Eeprom_1ByteDataRead(dataAddr, slaveAddr);
            pCamDeviceObj->camNormdata[sensor_idx][dataCnt+1] =
                (kal_uint8)Eeprom_1ByteDataRead(dataAddr+1, slaveAddr);
            dataCnt += 2;
        }
    }

    /*Set stereoFlag*/
    if ((pCamDeviceObj->i4MWDataIdx == sensor_idx)
         || (pCamDeviceObj->i4MTDataIdx == sensor_idx)
         || (pCamDeviceObj->i4FrontDataIdx == sensor_idx)) {
        dataCnt = OPLUS_CAMCOMDATA_LENS - 1;
        pCamDeviceObj->camNormdata[sensor_idx][dataCnt] = (kal_uint8)(sensor_idx);
    }
}

void Eeprom_StereoDataRead(enum IMGSENSOR_SENSOR_IDX sensor_idx, kal_uint16 slaveAddr)
{
    kal_uint16 dataLens = CALI_DATA_MASTER_LENGTH, dataAddr = 0xFFFF, dataCnt = 0;
    kal_uint16 i = 0;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    if (is_project(24700) || is_project(24701) || is_project(24702)
        || is_project(24709) || is_project(24713) || is_project(24714)){
        pCamDeviceObj = &gImgEepromInfoOrisC;
    }

    /*Read Single StereoParamsData and joint stereoData*/
    if (pCamDeviceObj->i4MWDataIdx == IMGSENSOR_SENSOR_IDX_MAIN2) {
        if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN) {
            dataLens = CALI_DATA_MASTER_LENGTH;
            dataAddr = pCamDeviceObj->i4MWStereoAddr[0];
        } else if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2) {
            dataLens = CALI_DATA_SLAVE_LENGTH;
            dataAddr = pCamDeviceObj->i4MWStereoAddr[1];
            dataCnt = CALI_DATA_MASTER_LENGTH;
        }
        if(sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN || sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2) {
            for (i = 0; i < dataLens; i++) {
                pCamDeviceObj->stereoMWdata[dataCnt+i] =
                            (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
            }
        }
    } else if (pCamDeviceObj->i4MWDataIdx == IMGSENSOR_SENSOR_IDX_SUB2) {
        if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN) {
            dataLens = CALI_DATA_MASTER_LENGTH;
            dataAddr = pCamDeviceObj->i4MTStereoAddr[0];
        } else if (sensor_idx == IMGSENSOR_SENSOR_IDX_SUB2) {
            dataLens = CALI_DATA_SLAVE_LENGTH;
            dataAddr = pCamDeviceObj->i4MTStereoAddr[1];
            dataCnt = CALI_DATA_MASTER_LENGTH;
        }
        for (i = 0; i < dataLens; i++) {
            pCamDeviceObj->stereoMTdata[dataCnt+i] =
                        (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
        }
    } else if (pCamDeviceObj->i4MWDataIdx == IMGSENSOR_SENSOR_IDX_MAIN4) {
        if (sensor_idx == IMGSENSOR_SENSOR_IDX_SUB) {
            dataLens = CALI_DATA_MASTER_LENGTH;
            dataAddr = pCamDeviceObj->i4FrontStereoAddr[0];
        } else if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN4) {
            dataLens = CALI_DATA_SLAVE_LENGTH;
            dataAddr = pCamDeviceObj->i4FrontStereoAddr[1];
            dataCnt = CALI_DATA_MASTER_LENGTH;
        }
        for (i = 0; i < dataLens; i++) {
            pCamDeviceObj->stereoFrontdata[dataCnt+i] =
                        (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
        }
    }

    if (pCamDeviceObj->i4MTDataIdx == IMGSENSOR_SENSOR_IDX_SUB2) {
        if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN) {
            dataLens = CALI_DATA_MASTER_LENGTH;
            dataAddr = pCamDeviceObj->i4MTStereoAddr[0];
        } else if (sensor_idx == IMGSENSOR_SENSOR_IDX_SUB2) {
            dataLens = CALI_DATA_SLAVE_LENGTH;
            dataAddr = pCamDeviceObj->i4MTStereoAddr[1];
            dataCnt = CALI_DATA_MASTER_LENGTH ;
        }
        if(sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN || sensor_idx == IMGSENSOR_SENSOR_IDX_SUB2) {
            for (i = 0; i < dataLens; i++) {
                pCamDeviceObj->stereoMTdata[dataCnt+i] =
                            (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
            }
        }
    }
}

void Eeprom_DistortionParamsRead(enum IMGSENSOR_SENSOR_IDX sensor_idx, kal_uint16 slaveAddr)
{
    kal_uint16 dataAddr = 0xFFFF, dataLens = CAMERA_DISTORTIONPARAMS_LENGTH;
    kal_uint16 i = 0;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    if (is_project(24700) || is_project(24701) || is_project(24702)
        || is_project(24709) || is_project(24713) || is_project(24714)){
        pCamDeviceObj = &gImgEepromInfoOrisC;
    }

    if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2 && pCamDeviceObj->i4DistortionAddr) {
        dataAddr = pCamDeviceObj->i4DistortionAddr;
        for (i = 0; i < dataLens; i++) {
            pCamDeviceObj->distortionParams[i] = (kal_uint8)Eeprom_1ByteDataRead(dataAddr+i, slaveAddr);
        }
    }
}


enum CUSTOM_CAMERA_ERROR_CODE_ENUM Eeprom_Control(
            enum IMGSENSOR_SENSOR_IDX sensor_idx,
            MSDK_SENSOR_FEATURE_ENUM feature_id,
            unsigned char *feature_para,
            kal_uint32 sensorID)
{
    kal_uint32 i4SensorID = 0;
    kal_uint32 dataCnt = 0;
    enum IMGSENSOR_SENSOR_IDX i4Sensor_idx = 0;
    struct CAMERA_DEVICE_INFO *pCamDeviceObj = &gImgEepromInfo;
#ifdef SENSOR_PLATFORM_5G_H
    /*2021/12/28, Bringup camera for 20817.*/
    if (is_project(20817) || is_project(20827) || is_project(20831)){
        pCamDeviceObj = &gImgEepromInfo_20817;
    }

    if (is_project(21881) || is_project(21882)){
        pCamDeviceObj = &gImgEepromInfo_21881;
    }
#endif

    if (is_project(24700) || is_project(24701) || is_project(24702)
        || is_project(24709) || is_project(24713) || is_project(24714)){
        pCamDeviceObj = &gImgEepromInfoOrisC;
    }

    i4Sensor_idx = pCamDeviceObj->i4CurSensorIdx;
    i4SensorID = pCamDeviceObj->i4CurSensorId;

    switch (feature_id) {
        case SENSOR_FEATURE_GET_EEPROM_COMDATA:
        {
            if (Eeprom_SensorInfoValid(i4Sensor_idx, i4SensorID) != IMGSENSOR_RETURN_SUCCESS) {
                pr_info("[%s]_COMDATA i4Sensor_idx:%d, i4SensorID:0x%x Invalid",
                                                __func__, i4Sensor_idx, i4SensorID);
                return ERROR_MSDK_IS_ACTIVATED;
            }
            dataCnt = (kal_uint32)(CAMERA_EEPPROM_COMDATA_LENGTH*i4Sensor_idx);
            pr_info("GET_EEPROM_COMDATA i4Sensor_idx %d, i4SensorID:0x%x, dataCnt:%d\n",
                        i4Sensor_idx, i4SensorID, dataCnt);
            memcpy(&feature_para[0], pCamDeviceObj->camNormdata[i4Sensor_idx], CAMERA_EEPPROM_COMDATA_LENGTH);
            break;
        }
        case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
        {
            if (Eeprom_SensorInfoValid(i4Sensor_idx, i4SensorID) != IMGSENSOR_RETURN_SUCCESS) {
                pr_info("[%s]_STEREODATA i4Sensor_idx:%d, i4SensorID:0x%x Invalid",
                                                __func__, i4Sensor_idx, i4SensorID);
                return ERROR_MSDK_IS_ACTIVATED;
            }
            pr_info("GET_EEPROM_STEREODATA i4Sensor_idx %d, stereoDataIdx:%d %d %d\n",
                        i4Sensor_idx,
                        pCamDeviceObj->i4MWDataIdx,
                        pCamDeviceObj->i4MTDataIdx,
                        pCamDeviceObj->i4FrontDataIdx);
            if (i4Sensor_idx == pCamDeviceObj->i4MWDataIdx) {
                if (is_project(20171) || is_project(20353)
                    || is_project(20615) || is_project(20619)
                    || is_project(21015) || is_project(21217)
                    || is_project(21061) || is_project(21218)
                    || is_project(21609) || is_project(20662)) {
                    memcpy(&feature_para[0], pCamDeviceObj->stereoMWdata, DUALCAM_CALI_DATA_LENGTH_TOTAL);
                } else if (is_project(20817) || is_project(20827) || is_project(20831)
                           || is_project(21881) || is_project(21882)) {
                    memcpy(&feature_para[0], pCamDeviceObj->stereoMWdata, DUALCAM_CALI_DATA_LENGTH_TOTAL);
                } else {
                    memcpy(&feature_para[0], pCamDeviceObj->stereoMWdata, DUALCAM_CALI_DATA_LENGTH_TOTAL);
                }
            } else if (i4Sensor_idx == pCamDeviceObj->i4MTDataIdx) {
                memcpy(&feature_para[0], pCamDeviceObj->stereoMTdata, DUALCAM_CALI_DATA_LENGTH_TOTAL_TELE);
            } else if (i4Sensor_idx == pCamDeviceObj->i4FrontDataIdx) {
                memcpy(&feature_para[0], pCamDeviceObj->stereoFrontdata, DUALCAM_CALI_DATA_LENGTH_TOTAL);
            }
            break;
        }
        case SENSOR_FEATURE_GET_DISTORTIONPARAMS:
            pr_debug("SENSOR_FEATURE_GET_DISTORTIONPARAMS: %d 0x%x\n", sensor_idx, sensorID);
            if (pCamDeviceObj->i4DistortionAddr == 0) {
                pr_debug("Invalid distortionParams");
                return ERROR_MSDK_IS_ACTIVATED;
            } else {
                memcpy(&feature_para[0], pCamDeviceObj->distortionParams, CAMERA_DISTORTIONPARAMS_LENGTH);
            }
        break;
        default:
            pr_err("Invalid feature_id: %d", feature_id);
        break;
    }

    return ERROR_NONE;
}
