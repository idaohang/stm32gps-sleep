
//-------------------------------------------------------
//GPS ����ģ�� By wowbanui
//�汾��ʷ:
//       2010/08 v0.1 ��ʼ�汾
//       2011/03 v0.2 ע�͵�����Ҫ���ֶ�,��������ֱ�Ӵ������ֵ,
//               �����ڴ�ռ��.�������ֱ�ӵ���LCD����,��ֲ�����
//---------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "gps.h"

static uint8_t NMEA_CMD = NMEA_NULL;       // NMEA ���
static uint8_t NMEA_CMD_Buff[] = "$GPxxx,"; // NMEA ������ͻ���
static uint8_t NMEA_CMD_Index = 0;         // ��ȡ CMD�ַ��ĸ���
static uint8_t NMEA_CMD_Parsered = 0;      // CMD���ͽ������
static uint8_t NMEA_DAT_Block = 0;         // NMEA �����ֶκ� ��0��ʼ
static uint8_t NMEA_DAT_BlockIndex = 0;    // NMEA ����ÿ���ֶ����ַ����� ��0��ʼ
static uint8_t NMEA_CMD_Start = 0;         // NMEA ��俪ʼ. ��⵽ $ ʱ��1
static uint8_t ReciveFlag = 0;             // ���ݽ������. ���һ�� GPRMC ��䷢�������1,

static uint8_t ucTempA = 0;                // �洢������λ�����õĵ�ʮλ��ʱ����

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
        ReciveFlag = 1;   // �������, ���Դ���
        break;
    case ',':
        NMEA_DAT_Block++;
        NMEA_DAT_BlockIndex = 0;
        break;
    default:
        switch(NMEA_DAT_Block)
        {
        case 0:         // <1> UTCʱ�� hhmmss.mmm
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
        case 1:         // <2> ��λ״̬ A=��Ч��λ, V=��Ч��λ
            g_stGPSRMCData.Status = ucData;
            break;
        case 2:         // <3> γ�� ddmm.mmmm
            g_stGPSRMCData.Latitude[NMEA_DAT_BlockIndex] = ucData;
            break;
        case 3:         // <4> γ�Ȱ��� N/S
            g_stGPSRMCData.NS = ucData;
            break;
        case 4:         // <5> ���� dddmm.mmmm
            g_stGPSRMCData.Longitude[NMEA_DAT_BlockIndex] = ucData;
            break;
        case 5:         // <6> ���Ȱ��� E/W
            g_stGPSRMCData.EW = ucData;
            break;
        case 6:         // <7> �������� 000.0~999.9 ��
            g_stGPSRMCData.Speed[NMEA_DAT_BlockIndex] = ucData;
            break;
        case 7:         // <8> ���溽�� 000.0~359.9 ��, ���汱Ϊ�ο���׼
            g_stGPSRMCData.Course[NMEA_DAT_BlockIndex] = ucData;
            break;
        case 8:         // <9> UTC���� ddmmyy
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
        // ��������$��ʼ�� NMEA ���, ����NMEA ��������:
        if(NMEA_CMD_Parsered)
        {
            // CMD������ͽ������, �����������ý�������
            switch(NMEA_CMD)
            {

            case NMEA_GPRMC:
                ParserGPRMC(ucData);
                break;
            default:    // �޷�ʶ��ĸ�ʽ, ��λ
                NMEA_CMD = NMEA_NULL;
                NMEA_CMD_Parsered = 0;
                NMEA_CMD_Index = 1;
                NMEA_CMD_Start = 0;
            }
        }
        else
        {
            // ��Ҫ����CMD�������
            switch(ucData)
            {
            case ',':     // ��һ���ֶν���
                if(NMEA_CMD_Buff[4] == 'G' && NMEA_CMD_Buff[5] == 'A') NMEA_CMD = NMEA_GPGGA;
                if(NMEA_CMD_Buff[4] == 'S' && NMEA_CMD_Buff[5] == 'A') NMEA_CMD = NMEA_GPGSA;
                if(NMEA_CMD_Buff[5] == 'V') NMEA_CMD = NMEA_GPGSV;
                if(NMEA_CMD_Buff[5] == 'C') NMEA_CMD = NMEA_GPRMC;
                // �˴������������, ����䲻��ʶ��, ��NMEA_CMDΪNULL������,
                // ��תΪ�������ͽ���ʱ����ת���޷�ʶ��ĸ�ʽ, ����λ
                NMEA_CMD_Parsered = 1;
                NMEA_CMD_Index = 1;
                NMEA_DAT_Block = 0;
                NMEA_DAT_BlockIndex = 0;
                break;
            case '*':
                NMEA_CMD_Start = 0;
                break;
            default:       // ���ڵ�һ���ֶ���, ��������
                NMEA_CMD_Buff[NMEA_CMD_Index] = ucData;
                NMEA_CMD_Index++;
                // CMD ����6���ַ�, (����Խ��, ��������)
                // ���жϲ���������CMD���, ���Թ��˾�.
                if (NMEA_CMD_Index > 6)
                {
                    NMEA_CMD_Start = 0;
                }
            }
        }
    }
    else
    {
        // δ������$, ѭ�����ղ��ж� ֱ�� $
        if (ucData == '$')
        {
            // ���յ�$, ��һ���ַ���Ϊ�����ж��ַ�, �Ƚ�����ر�����ʼ��
            NMEA_CMD_Buff[0] = ucData;
            NMEA_CMD_Start = 1;   // �´ε��������NMEA ��������:
            NMEA_CMD_Index = 1;   // ��ͷ���GPS�����ַ�������
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

    // ��γddmm.mmmm 22 32.7658 (22x60 + 32.7658)*30000 = 40582974 = 0x26B3F3E, Ȼ��ת��16����Ϊ: 0x02 0x6B 0x3F 0x3E
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

    // ���� dddmm.mmmm
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


