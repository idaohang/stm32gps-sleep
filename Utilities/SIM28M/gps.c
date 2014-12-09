
//-------------------------------------------------------
//GPS 解析模块 By wowbanui
//版本历史:
//       2010/08 v0.1 初始版本
//       2011/03 v0.2 注释掉不需要的字段,部分数据直接处理成数值,
//               减少内存占用.输出部分直接调用LCD命令,移植需更改
//---------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "gps.h"

static uint8_t NMEA_CMD = NMEA_NULL;       // NMEA 语句
static uint8_t NMEA_CMD_Buff[] = "$GPxxx,"; // NMEA 语句类型缓存
static uint8_t NMEA_CMD_Index = 0;         // 读取 CMD字符的个数
static uint8_t NMEA_CMD_Parsered = 0;      // CMD类型解析完毕
static uint8_t NMEA_DAT_Block = 0;         // NMEA 数据字段号 从0开始
static uint8_t NMEA_DAT_BlockIndex = 0;    // NMEA 数据每个字段内字符索引 从0开始
static uint8_t NMEA_CMD_Start = 0;         // NMEA 语句开始. 检测到 $ 时置1
static uint8_t ReciveFlag = 0;             // 数据接收完成. 最后一条 GPRMC 语句发送完毕置1,

static uint8_t ucTempA = 0;                // 存储解析两位数字用的的十位临时变量

volatile uint8_t g_ucRecvOverTimeFlag = 0;
static stru_GPSRMC  g_stGPSRMCData;


static void ParserGPRMC(uint8_t ucData);
static void GPS(uint8_t ucData);
static uint8_t ReciveOK(void);

void ShowGPSTime(void)
{
    int i = 0;
    for(i = 0; i < 6; i++)
    {
        DEBUG("g_stGPSRMCData.UTCDateTime[%d] = %d\r\n", i, g_stGPSRMCData.UTCDateTime[i]);
    }
    DEBUG("Status = %c\r\n", g_stGPSRMCData.Status);
}
// N dd'mm'ss.ssss
void ShowLatitude(void)
{
    int i = 0;
    for(i = 0; i < 9; i++)
    {
        DEBUG("g_stGPSRMCData.Latitude[%d] = %c\r\n", i, g_stGPSRMCData.Latitude[i]);
    }
}


void ShowLongitude(void)
{
    int i = 0;
    for(i = 0; i < 9; i++)
    {
        DEBUG("g_stGPSRMCData.Longitude[%d] = %c\r\n", i, g_stGPSRMCData.Longitude[i]);
    }
}

void ShowSpeed(void)
{
    int i = 0;
    for(i = 0; i < 5; i++)
    {
        DEBUG("g_stGPSRMCData.Speed[%d] = %c\r\n", i, g_stGPSRMCData.Speed[i]);
    }
}

void ShowCourse(void)
{
    int i = 0;
    for(i = 0; i < 5; i++)
    {
        DEBUG("g_stGPSRMCData.Course[%d] = %c\r\n", i, g_stGPSRMCData.Course[i]);
    }
}

/**
  * @brief  Turn on GPS's VCC power.
  * Output LOW to Turn On VCC Power; HIGH to Turn Off VCC Power.
  * @param  None
  * @retval None
  */
void GPSPowerOn(void)
{
    GPIO_ResetBits(GPS_PWR_CTRL_PORT, GPS_PWR_CTRL_PIN);

}

/**
  * @brief  Turn off GPS's VCC power.
  * Output LOW to Turn On VCC Power; HIGH to Turn Off VCC Power.
  * @param  None
  * @retval None
  */
void GPSPowerOff(void)
{
    GPIO_SetBits(GPS_PWR_CTRL_PORT, GPS_PWR_CTRL_PIN);
}

/**
  * @brief  Parse GPS RMC Data
  * @param  ucData - Received Data
  * @retval None
  */
static void ParserGPRMC(uint8_t ucData)
{
    switch(ucData)
    {
    case '*':
        NMEA_CMD_Start = 0;
        ReciveFlag = 1;   // 接收完毕, 可以处理
        break;
    case ',':
        NMEA_DAT_Block++;
        NMEA_DAT_BlockIndex = 0;
        break;
    default:
        switch(NMEA_DAT_Block)
        {
        case 0:         // <1> UTC时间 hhmmss.mmm
            switch(NMEA_DAT_BlockIndex)
            {
            case 0:
            case 2:
            case 4:
                ucTempA = ucData - '0';
                break;
            case 1:
                g_stGPSRMCData.UTCDateTime[3] = ucTempA * 10 + ucData - '0';
                break;
            case 3:
                g_stGPSRMCData.UTCDateTime[4] = ucTempA * 10 + ucData - '0';
                break;
            case 5:
                g_stGPSRMCData.UTCDateTime[5] = ucTempA * 10 + ucData - '0';
                break;
			default:
				break;
            }
            break;
        case 1:         // <2> 定位状态 A=有效定位, V=无效定位
            g_stGPSRMCData.Status = ucData;
            break;
        case 2:         // <3> 纬度 ddmm.mmmm
            g_stGPSRMCData.Latitude[NMEA_DAT_BlockIndex] = ucData;
            break;
        case 3:         // <4> 纬度半球 N/S
            g_stGPSRMCData.NS = ucData;
            break;
        case 4:         // <5> 经度 dddmm.mmmm
            g_stGPSRMCData.Longitude[NMEA_DAT_BlockIndex] = ucData;
            break;
        case 5:         // <6> 经度半球 E/W
            g_stGPSRMCData.EW = ucData;
            break;
        case 6:         // <7> 地面速率 000.0~999.9 节
            g_stGPSRMCData.Speed[NMEA_DAT_BlockIndex] = ucData;
            break;
        case 7:         // <8> 地面航向 000.0~359.9 度, 以真北为参考基准
            g_stGPSRMCData.Course[NMEA_DAT_BlockIndex] = ucData;
            break;
        case 8:         // <9> UTC日期 ddmmyy
            switch(NMEA_DAT_BlockIndex)
            {
            case 0:
            case 2:
            case 4:
                ucTempA = ucData - '0';
                break;
            case 1:
                g_stGPSRMCData.UTCDateTime[2] = ucTempA * 10 + ucData - '0';
                break;
            case 3:
                g_stGPSRMCData.UTCDateTime[1] = ucTempA * 10 + ucData - '0';
                break;
            case 5:
                g_stGPSRMCData.UTCDateTime[0] = ucTempA * 10 + ucData - '0';
                break;
			default:
				break;
            }
            break;
		default:
				break;
        }
        NMEA_DAT_BlockIndex++;
    }
}

/**
  * @brief  Parse GPS Data which Received
  * @param  ucData - Received Data
  * @retval None
  */
void GPS(uint8_t ucData)
{
    if(NMEA_CMD_Start)
    {
        // 解析到以$开始的 NMEA 语句, 进入NMEA 解析流程:
        if(NMEA_CMD_Parsered)
        {
            // CMD语句类型解析完毕, 根据类型条用解析函数
            switch(NMEA_CMD)
            {

            case NMEA_GPRMC:
                ParserGPRMC(ucData);
                break;
            default:    // 无法识别的格式, 复位
                NMEA_CMD = NMEA_NULL;
                NMEA_CMD_Parsered = 0;
                NMEA_CMD_Index = 1;
                NMEA_CMD_Start = 0;
            }
        }
        else
        {
            // 需要解析CMD语句类型
            switch(ucData)
            {
            case ',':     // 第一个字段结束
                if(NMEA_CMD_Buff[4] == 'G' && NMEA_CMD_Buff[5] == 'A') NMEA_CMD = NMEA_GPGGA;
                if(NMEA_CMD_Buff[4] == 'S' && NMEA_CMD_Buff[5] == 'A') NMEA_CMD = NMEA_GPGSA;
                if(NMEA_CMD_Buff[5] == 'V') NMEA_CMD = NMEA_GPGSV;
                if(NMEA_CMD_Buff[5] == 'C') NMEA_CMD = NMEA_GPRMC;
                // 此处如果都不成立, 即语句不被识别, 则NMEA_CMD为NULL或其他,
                // 则转为根据类型解析时会跳转到无法识别的格式, 而后复位
                NMEA_CMD_Parsered = 1;
                NMEA_CMD_Index = 1;
                NMEA_DAT_Block = 0;
                NMEA_DAT_BlockIndex = 0;
                break;
            case '*':
                NMEA_CMD_Start = 0;
                break;
            default:       // 处于第一个字段中, 继续接收
                NMEA_CMD_Buff[NMEA_CMD_Index] = ucData;
                NMEA_CMD_Index++;
                // CMD 超过6个字符, (数组越界, 导致死机)
                // 则判断不是正常的CMD语句, 则略过此句.
                if (NMEA_CMD_Index > 6)
                {
                    NMEA_CMD_Start = 0;
                }
            }
        }
    }
    else
    {
        // 未解析到$, 循环接收并判断 直到 $
        if (ucData == '$')
        {
            // 接收到$, 下一个字符即为类型判断字符, 先进行相关变量初始化
            NMEA_CMD_Buff[0] = ucData;
            NMEA_CMD_Start = 1;   // 下次调用则进入NMEA 解析流程:
            NMEA_CMD_Index = 1;   // 从头存放GPS类型字符到变量
            NMEA_CMD_Parsered = 0;
            NMEA_CMD = NMEA_NULL;
            NMEA_DAT_Block = 0;
            NMEA_DAT_BlockIndex = 0;
        }
    }
    //LED_G=1;
}

/**
  * @brief  Get GPS Receive Result
  * @note   If receive '*', indicate GPS receive ok.
  * @param  None
  * @retval 1 - OK; 0 - FAIL
  */
uint8_t ReciveOK(void)
{
    if (ReciveFlag)
    {
        ReciveFlag = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief  Get GPS Data
  * @note   If receive over time, indicate GPS error.
  * @param  None
  * @retval RST_OK - OK; RST_FAIL - FAIL
  */
uint8_t GetGPSData(void)
{
    uint8_t data = 0;
    while((1 != ReciveOK()) )
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
        {
            if(1 == g_ucRecvOverTimeFlag)
            {
                g_ucRecvOverTimeFlag = 0;
                return RST_FAIL;
            }
        }
        data = USART_ReceiveData(USART1);
        GPS(data);
    }

    return RST_OK;
}

/**
  * @brief  Parse GPS Data
  * @param  pData - struct of GPS Data
  * @retval None
  */
void ParseGPSData(stru_GPSDATA *pData)
{
#ifdef DBG_ENABLE_MACRO
    uint32_t i;
#endif
    struct tm t;
    uint32_t spd = 0;

    t.tm_year = g_stGPSRMCData.UTCDateTime[0] + 100;
    t.tm_mon = g_stGPSRMCData.UTCDateTime[1];
    t.tm_mday = g_stGPSRMCData.UTCDateTime[2];
    t.tm_hour = g_stGPSRMCData.UTCDateTime[3];
    t.tm_min = g_stGPSRMCData.UTCDateTime[4];
    t.tm_sec = g_stGPSRMCData.UTCDateTime[5];
    t.tm_isdst = 0;

    pData->utc.i = mktime(&t);
#ifdef DBG_ENABLE_MACRO
    printf("UTC:");
    for(i = 0; i < 4; i++)
    {
        printf("0x%x-", pData->utc.s[i]);
    }
    printf("\r\n");
#endif

    // 北纬ddmm.mmmm 22 32.7658 (22x60 + 32.7658)*30000 = 40582974 = 0x26B3F3E, 然后转成16进制为: 0x02 0x6B 0x3F 0x3E
    pData->latitude.i = (((g_stGPSRMCData.Latitude[0] * 10 + g_stGPSRMCData.Latitude[1]) * 60
                          + (g_stGPSRMCData.Latitude[2] * 10 + g_stGPSRMCData.Latitude[3])) * 30000
                         + (g_stGPSRMCData.Latitude[5]) * 3000 + (g_stGPSRMCData.Latitude[6]) * 300
                         + (g_stGPSRMCData.Latitude[7]) * 30 + (g_stGPSRMCData.Latitude[8]) * 3);
    // latitude
    if(g_stGPSRMCData.NS == 'S')
    {
        pData->latitude.i *= (-1);
    }

#ifdef DBG_ENABLE_MACRO
    printf("LATI:");
    for(i = 0; i < 4; i++)
    {
        printf("0x%x-", pData->latitude.s[i]);
    }
    printf("\r\n");
#endif

    // 经度 dddmm.mmmm
    pData->longitude.i = (((g_stGPSRMCData.Longitude[0] * 100 + g_stGPSRMCData.Longitude[1] * 10 + g_stGPSRMCData.Longitude[2]) * 60
                           + (g_stGPSRMCData.Longitude[3] * 10 + g_stGPSRMCData.Longitude[4])) * 30000
                          + (g_stGPSRMCData.Longitude[6]) * 3000 + (g_stGPSRMCData.Longitude[7]) * 300
                          + (g_stGPSRMCData.Longitude[8]) * 30 + (g_stGPSRMCData.Longitude[9]) * 3);
    // longitude
    if(g_stGPSRMCData.EW == 'W')
    {
        pData->longitude.i *= (-1);
    }

#ifdef DBG_ENABLE_MACRO
    printf("LONGI:");
    for(i = 0; i < 4; i++)
    {
        printf("0x%x-", pData->longitude.s[i]);
    }
    printf("\r\n");
#endif

    // 1 knot = 1.85km/h
    spd = (uint32_t)((atoi(g_stGPSRMCData.Speed)) * (1.85));
    if(spd > 255)
    {
        pData->speed = 255;
    }
    else
    {
        pData->speed = spd;
    }
    pData->course.i = (unsigned short)(atoi(g_stGPSRMCData.Course));

    if(('A' == g_stGPSRMCData.Status) || ('a' == g_stGPSRMCData.Status))
    {
        pData->status = 1;
    }
    else
    {
        pData->status = 0;
    }

#ifdef DBG_ENABLE_MACRO
    printf("SPEED: %d\r\n", pData->speed);
    printf("COURSE:");
    for(i = 0; i < 2; i++)
    {
        printf("0x%x-", pData->course.s[i]);
    }
    printf("\r\n");
    printf("STATUS: %d\r\n", pData->status);
#endif
}


