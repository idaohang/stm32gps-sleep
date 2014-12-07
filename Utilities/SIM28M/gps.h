#ifndef __GPS_H__
#define __GPS_H__

#include <string.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32gps_config.h"

typedef struct
{
    uint8_t UTCDateTime[6]; //YMDHMS
    uint8_t Status; //A/V
    uint8_t Latitude[9]; //ddmm.mmmm
    uint8_t NS; //N/S
    uint8_t Longitude[10]; //dddmm.mmmm
    uint8_t EW; //E/W
    uint8_t Speed[8]; // 速率000.0~999.9节
    uint8_t Course[8]; // 航向000.0~359.9度
} stru_GPSRMC;


typedef struct
{
    union
    {
        unsigned char s[4];
        int i;
    } utc;
    union
    {
        unsigned char s[4];
        int           i;
    } latitude;  // 纬度
    union
    {
        unsigned char s[4];
        int           i;
    } longitude; // 经度
    unsigned char speed;        // 0x00 - 0xFF 0~255km/h
    union
    {
        unsigned char s[2];
        unsigned short i;
    } course; // 0 - 360度 航向
    unsigned char status;  // 定位状态
} stru_GPSDATA;

#define NMEA_NULL    0x00            // GPS语句类型
#define NMEA_GPGGA    0x01
#define NMEA_GPGSA    0x02
#define NMEA_GPGSV    0x04
#define NMEA_GPRMC    0x08

#define GPS_DEVICE_OK 0x00
#define GPS_DEVICE_ERR 0x01

extern volatile uint8_t g_ucRecvOverTimeFlag;

void ShowLongitude(void);
void ShowLatitude(void);
void ShowGPSTime(void);
void ShowSpeed(void);
void ShowCourse(void);

void GPSPowerOn(void);
void GPSPowerOff(void);

uint8_t GetGPSData(void);
void ParseGPSData(stru_GPSDATA *pData);

#endif
