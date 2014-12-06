/**
 ******************************************************************************
 * @file    USART/Printf/main.c
 * @author  MCD Application Team
 * @version V3.5.0
 * @date    08-April-2011
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_it_api.h"
#include "stm32gps_board.h"
#include "stm32gps_config.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "GSM_App.h"
//#include "GPS_App.h"
#include "eelink.h"
#include "gpio.h"
#include "gps.h"


/* Private typedef -----------------------------------------------------------*/

typedef struct
{
    unsigned char status[2];    // dveice status
    unsigned char analog1[2]; 	// analog input 1
    unsigned char analog2[2];		// analog input 2
} ST_DEVICEDATA, *pST_DEVICEDATA;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define BKP_DR_NUMBER              42
/* Private variables ---------------------------------------------------------*/

// imei information buffer
//uint8_t imeiBuf[IMEI_BUF_LEN];
uint8_t g_IMSIBuf[IMSI_INFO_LEN];
uint8_t g_phoneNum[PHONE_NUMBER_LEN];

// packet message
EELINK_SIM_PACKET_LOGIN stLoginMsg;
static char LoginBuf[PROTO_LOGIN_BUF_LEN];
static char gpsBuf[PROTO_GPS_BUF_LEN];
static char stationBuf[PROTO_STATION_BUF_LEN];


ST_DEVICEDATA g_deviceData;

ST_BATVOLTAGESTATUS stBATInfo;
ST_CREGINFO stCREGInfo;
ST_IMSIINFO stIMSIInfo;
stru_GPSDATA stGPSData;
ST_PACKET_BASESTATION stStationInfo;

uint8_t ucIMEIBuf[IMEI_BUF_LEN];
uint8_t ucSignalQuality;

uint16_t g_usSequenceNum;  // GPRS packet's sequence number

uint8_t ucGPSStatus;    // GPS status 0x55 - error; 0xaa - ok

uint32_t g_uiSetSleepSec;  // Server setting sleep seconds

FlagStatus g_uiAlarmFlag;  // 报警标志 SET-valid; RESET-invalid
FlagStatus g_uiAlarmPacketFlag; // 报警数据包标志 SET - alarm packet; RESET - not alarm packet


/* Private function prototypes -----------------------------------------------*/
uint8_t ProcessIMEI(uint8_t *pImei, uint8_t *pRst, int32_t imeilen, int32_t rstlen);
void loadLoginMsg(uint8_t *imei, uint16_t sequence);
void PackLoginMsg(void);
void PackGpsMsg(void);
void PackAlarmMsg(void);
void PackStationMsg(void);
void PackFactoryMsg(void);
/* Private functions ---------------------------------------------------------*/

#ifdef DBG_ENABLE_MACRO
void ShowLoginMsg(void)
{
    uint32_t i;
    // imei
    DEBUG("IMEI:");
    for(i = 0; i < IMEI_BUF_LEN; i++)
    {
        DEBUG("%c-", ucIMEIBuf[i]);
    }
    DEBUG("\r\n");
    // login msg
    DEBUG("\r\nLOGIN MSG:");
    for(i = 0; i < PROTO_LOGIN_BUF_LEN; i++)
    {
        DEBUG("0x%x-", LoginBuf[i]);
    }
    DEBUG("\r\n");
}

void ShowGpsMsg(void)
{
    uint32_t i;
    // gps msg
    DEBUG("\r\nGPS MSG:");
    for(i = 0; i < PROTO_GPS_BUF_LEN; i++)
    {
        DEBUG("0x%x-", gpsBuf[i]);
    }
    DEBUG("\r\n");
}

void ShowStationMsg(void)
{
    uint32_t i;
    // gps msg
    DEBUG("\r\nSTATION MSG:");
    for(i = 0; i < (8 + stStationInfo.num * 11); i++)
    {
        DEBUG("0x%x-", stationBuf[i]);
    }
    DEBUG("\r\n");
}

void ShowFactoryMsg(void)
{
    uint32_t i;
    // gps msg
    DEBUG("\r\nFACTORY MSG:");
    for(i = 0; i < FACTORY_REPORT_MSGLEN; i++)
    {
        DEBUG("0x%x-", stationBuf[i]);
    }
    DEBUG("\r\n");
}
#endif // DBG_ENABLE_MACRO

/**
  * @brief  Init Global Variables
  * @param  None
  * @retval None
  */
void InitVariables(void)
{
    uint32_t i;

    g_usSequenceNum = 1;

    ucGPSStatus = GPS_DEVICE_ERR;

    g_uiSetSleepSec = SLEEP_NORMAL_SEC;
    g_uiAlarmFlag = RESET;
    g_uiAlarmPacketFlag = RESET;

    memset(g_IMSIBuf, 0, IMSI_INFO_LEN);
    for(i = 0; i < PROTO_LOGIN_BUF_LEN; i++)
    {
        LoginBuf[i] = 0;
    }
    for(i = 0; i < PROTO_GPS_BUF_LEN; i++)
    {
        gpsBuf[i] = 0;
    }
    for(i = 0; i < PROTO_STATION_BUF_LEN; i++)
    {
        stationBuf[i] = 0;
    }

    memset(ucIMEIBuf, 0, IMEI_BUF_LEN);

    for(i = 0; i < PHONE_NUMBER_LEN; i++)
    {
        g_phoneNum[i] = 0x30;
    }

    memset(&stLoginMsg, 0, sizeof(stLoginMsg));

    memset(&g_deviceData, 0, sizeof(g_deviceData));
    memset(&stStationInfo, 0, sizeof(stStationInfo));

}



/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
    //    int i = 0;
    uint32_t errNum = 0;
    uint8_t gpsRecvTimes = 0;  // GPS Received Times
    uint8_t gsmRetyTimes = 0;  // GSM Retry Times
    uint8_t gprsRetyTimes = 0; // GPRS Retry Times

    uint8_t gpsRtn = RST_FAIL;    // GPS function return result
    uint8_t gsmRtn = RST_FAIL;
    uint8_t gprsRtn = RST_FAIL;
    uint8_t gprsSendFlag = RST_FAIL; // GPRS Send Status Flag

    uint16_t sendLen = 0;      // GPRS send length (used for GPS and ALARM Msg)
    uint32_t sleepSec = 0;
    uint32_t alarmStickFlag = 0;    // 是否有移开的动作 1- 有 0-无

    // Used for parse GPRS Received Data Analysis
    char *pRecvBuf = NULL;     // GPRS Received buffer
    uint32_t recvLen = 0;      // GPRS Received length
    char *pfeed = NULL;        // Used for parse

    // Used for CheckLinkStatus
    uint8_t status = 0;

    /////////////////////////////////////////////////////////////////
    // Configure the GPIO ports and Power OFF GPS and GSM
    /////////////////////////////////////////////////////////////////
    MX_GPIO_Init();
    GPSPowerOff();
    GSM_PowerOff();
    /////////////////////////////////////////////////////////////////
    // Configure the SysTick
    /////////////////////////////////////////////////////////////////
    stm32gps_sys_tick_cfg();

    /////////////////////////////////////////////////////////////////
    // Configure PWR and BKP
    /////////////////////////////////////////////////////////////////
    /* Enable PWR and BKP clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    /* Enable WKUP pin */
    //    PWR_WakeUpPinCmd(ENABLE);


    /////////////////////////////////////////////////////////////////
    // Configure RCC
    /////////////////////////////////////////////////////////////////
    //RCC_Configuration();

    /////////////////////////////////////////////////////////////////
    // Configure EXTI
    /////////////////////////////////////////////////////////////////
    EXTI_Configuration();

    /////////////////////////////////////////////////////////////////
    // Configure RTC
    /////////////////////////////////////////////////////////////////
    RTC_Configuration();
    RTC_NVIC_Configuration();

    /////////////////////////////////////////////////////////////////
    // Configure TIMER
    /////////////////////////////////////////////////////////////////
    TIM2_Configuration();
    TIM2_NVIC_Configuration();

    /////////////////////////////////////////////////////////////////
    // Configure LED, BUTTON and USART(GPS + GSM + DEBUG)
    /////////////////////////////////////////////////////////////////

    stm32gps_led_cfg();
    STM_EVAL_LEDOff(LED1);
#ifdef DBG_ENABLE_MACRO
    stm32gps_com_debug_cfg();
#endif // DBG_ENABLE_MACRO

    STM_EVAL_PBInit(BUTTON_KEY, BUTTON_MODE_EXTI);

    //usart_init(STM32_SIM908_GPS_COM);
    stm32gps_com_gps_cfg();

    usart_init(STM32_SIM908_GSM_COM);
    stm32gps_com_gsm_cfg();
    /* Enable the EVAL_COM2 Receive interrupt: this interrupt is generated when the
     EVAL_COM1 receive data register is not empty */
    USART_ITConfig(EVAL_COM2, USART_IT_RXNE, ENABLE);

    USART_NVIC_Configuration();

    /////////////////////////////////////////////////////////////////
    // Turn On TIM2
    /////////////////////////////////////////////////////////////////
    //TIM2_Start();

    /////////////////////////////////////////////////////////////////
    // Init Variables
    /////////////////////////////////////////////////////////////////
    InitVariables();

    /////////////////////////////////////////////////////////////////
    // Main LOOP
    /////////////////////////////////////////////////////////////////
    while(1)
    {
        delay_10ms(STICK_ON_SEC);
        gpsRtn = RST_FAIL;
        gsmRtn = RST_FAIL;
        gprsRtn = RST_FAIL;
        gprsSendFlag = RST_FAIL;
        gsmRetyTimes = 0;
        gpsRecvTimes = 0;


        // If Stick On Car or sth
        if(((uint32_t)Bit_RESET == STM_EVAL_PBGetState(BUTTON_KEY))
                || (SET == g_uiAlarmFlag))
        {
            /////////////////////////////////////////////////////////////////
            // Check Stick Action and set flag
            /////////////////////////////////////////////////////////////////
            if((uint32_t)Bit_SET == STM_EVAL_PBGetState(BUTTON_KEY))
            {
                alarmStickFlag = 1;  // Not Stick
            }
            else
            {
                alarmStickFlag = 0;
            }

            /////////////////////////////////////////////////////////////////
            // First, Power ON GSM
            /////////////////////////////////////////////////////////////////
            GSM_PowerOn();
            delay_10ms(200);

            while(gsmRetyTimes < GSM_RETERY_TIMES)
            {
                gsmRetyTimes++;
                gsmRtn = GSM_Init();
                if(RST_FAIL == gsmRtn)
                {
                    continue;
                }
                gsmRtn = GSM_CheckSIMCard();
                if(RST_FAIL == gsmRtn)
                {
                    continue;
                }

                gsmRtn = GSM_QuerySignal(&ucSignalQuality);
                if(RST_FAIL == gsmRtn)
                {
                    continue;
                }


                gsmRtn = GSM_CheckNetworkReg();
                if(RST_FAIL == gsmRtn)
                {
                    continue;
                }

                GSM_SetNetworkReg(2);
                if(USART_FAIL == GSM_QueryCreg(&stCREGInfo))
                {
                    // clear creg info
                }

                gsmRtn = GSM_QueryImei(ucIMEIBuf);
                if(RST_FAIL == gsmRtn)
                {
                    // clear imei buffer
                }

                gsmRtn = GSM_QueryImsi(&stIMSIInfo);
                if(RST_FAIL == gsmRtn)
                {
                    // clear imsi info
                }

                GSM_SetCIPMode(0);

                gsmRtn = GSM_CheckGPRSService();
                if(RST_FAIL == gsmRtn)
                {
                    continue;
                }

                gsmRtn = GSM_QueryBatVoltage(&stBATInfo);
                if(RST_FAIL == gsmRtn)
                {
                    // clear battery info
                }

                gsmRtn = GSM_ceng(&stStationInfo);
                if(RST_FAIL == gsmRtn)
                {
                    // clear battery info
                }

                while(gprsRetyTimes < GPRS_RETRY_TIMES)
                {
                    gprsRetyTimes++;
                    GPRS_CIPShut();
                    GPRS_CheckLinkStatus(&status);
                    gsmRtn = GSM_StartTaskAndSetAPN();
                    if(RST_FAIL == gsmRtn)
                    {
                        continue;
                    }
                    GPRS_CheckLinkStatus(&status);
                    gsmRtn = GSM_BringUpConnect();
                    if(RST_FAIL == gsmRtn)
                    {
                        //continue;
                    }
                    GPRS_CheckLinkStatus(&status);
                    gsmRtn = GSM_GetLocalIP();
                    if(RST_FAIL == gsmRtn)
                    {
                        //continue;
                    }
                    GPRS_CheckLinkStatus(&status);
                    gsmRtn = GSM_StartUpConnect();
                    if(RST_FAIL == gsmRtn)
                    {
                        continue;
                    }
                    GPRS_CheckLinkStatus(&status);
                    g_usSequenceNum = 1; // Init packet sequence to 1
                    errNum = 0;
                    loadLoginMsg(ucIMEIBuf, g_usSequenceNum);
                    PackLoginMsg();
#ifdef DBG_ENABLE_MACRO
                    ShowLoginMsg();
#endif
                    while(errNum < 5)
                    {
                        errNum++;
                        gprsRtn = GPRS_SendData_rsp(LoginBuf, EELINK_LOGIN_MSGLEN, &pRecvBuf, &recvLen);
                        if(USART_SUCESS == gprsRtn)
                        {
                            // parse response data, pp is 0x70 0x70
                            pfeed = strstr_len(pRecvBuf, "pp", recvLen);
                            if(pfeed != NULL)
                            {
                                sleepSec = (uint32_t)(((*(pfeed + 7)) << 24)
                                                      + ((*(pfeed + 8)) << 16)
                                                      + ((*(pfeed + 9)) << 8)
                                                      + (*(pfeed + 10)));
                                DEBUG("sleepSec = %d\n", sleepSec);
                                // Check sleep time setting value
                                if((sleepSec > SLEEP_TIME_MIN) && (sleepSec < SLEEP_TIME_MAX))
                                {
                                    g_uiSetSleepSec = sleepSec;

                                }
                                DEBUG("g_uiSetSleepSec = %d\n", g_uiSetSleepSec);
                            }
                            else
                            {
                                DEBUG("Server Set Sleep Timer ERROR\n");
                            }
                            break;
                        }
                        else
                        {

#ifdef DBG_ENABLE_MACRO
                            STM_EVAL_LEDToggle(LED1);
                            DEBUG("GPRS_SendData LOGIN MSG Fail\n");
#endif // DBG_ENABLE_MACRO

                        }
                    }

                    errNum = 0;
                    g_usSequenceNum++;

                    // Factory Test
                    if(0)
                    {
                        PackFactoryMsg();
                        sendLen = FACTORY_REPORT_MSGLEN;
#ifdef DBG_ENABLE_MACRO
                        ShowFactoryMsg();
#endif
                    }
                    else  // NOT Factory Test
                    {
                        PackStationMsg();
                        sendLen = 8 + stStationInfo.num * 11;
#ifdef DBG_ENABLE_MACRO
                        ShowStationMsg();
#endif
                    }
                    while(errNum < 5)
                    {
                        errNum++;
                        gprsRtn = GPRS_SendData(stationBuf, sendLen);
                        if(USART_SUCESS == gprsRtn)
                        {
                            gprsSendFlag = RST_OK;
                            break;
                        }
                    }

                    // break gprs send process
                    if(RST_OK == gprsSendFlag)
                    {
                        break;
                    }

                }
                gprsRetyTimes = 0;
                // break gprs send process
                if(RST_OK == gprsSendFlag)
                {
                    break;
                }

            }
            gsmRetyTimes = 0;
            gprsSendFlag = RST_FAIL;
            errNum = 0;
            /////////////////////////////////////////////////////////////////
            // Set CFUN Min
            /////////////////////////////////////////////////////////////////
            GSM_SetCFunMin();

            /////////////////////////////////////////////////////////////////
            // Then, Power ON GPS
            /////////////////////////////////////////////////////////////////
            GPSPowerOn();
            DEBUG("\r\n GPSPowerOn \r\n");
            delay_10ms(200);
            /////////////////////////////////////////////////////////////////
            // Receive GPS Data and Parse, If Recv Success then break
            /////////////////////////////////////////////////////////////////
            while(gpsRecvTimes < GPS_RETERY_TIMES)
            {
                gpsRecvTimes++;
                TIM2_Start();
                gpsRtn = GetGPSData();
                TIM2_Stop();

                if(RST_OK == gpsRtn)
                {
                    ucGPSStatus = GPS_DEVICE_OK;
                    ParseGPSData(&stGPSData);
                    if(1 == stGPSData.status)
                    {
                        GPSPowerOff();
                        DEBUG("GPSData Valid and TurnOFF GPS\r\n");
                        break;
                    }
                }

                /////////////////////////////////////////////////////////////////
                // Set RTC Alarm to wake from STOP mode
                /////////////////////////////////////////////////////////////////
                /* Wait till RTC Second event occurs */
                RTC_ClearFlag(RTC_FLAG_SEC);
                while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

                /* Alarm in 10 second */
                RTC_SetAlarm(RTC_GetCounter() + GPS_STOPMODE_SEC);
                /* Wait until last write operation on RTC registers has finished */
                RTC_WaitForLastTask();

                /////////////////////////////////////////////////////////////////
                // Go Into STOP Mode
                /////////////////////////////////////////////////////////////////
                /* Request to enter STOP mode with regulator in low power mode*/
                PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

                /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
                	PLL as system clock source (HSE and PLL are disabled in STOP mode) */
                SYSCLKConfig_STOP();

                //delay_10ms(500);
            }

            /////////////////////////////////////////////////////////////////
            // Set CFUN Max
            /////////////////////////////////////////////////////////////////
            GSM_SetCFunFull();

            gsmRetyTimes = 0;
            gprsRetyTimes = 0;
            gsmRtn = RST_FAIL;
            gprsRtn = RST_FAIL;

            while(gsmRetyTimes < GSM_RETERY_TIMES)
            {
                gsmRetyTimes++;
                gsmRtn = GSM_Init();
                if(RST_FAIL == gsmRtn)
                {
                    continue;
                }
                gsmRtn = GSM_CheckSIMCard();
                if(RST_FAIL == gsmRtn)
                {
                    continue;
                }

                gsmRtn = GSM_QuerySignal(&ucSignalQuality);
                if(RST_FAIL == gsmRtn)
                {
                    continue;
                }


                gsmRtn = GSM_CheckNetworkReg();
                if(RST_FAIL == gsmRtn)
                {
                    continue;
                }

                GSM_SetNetworkReg(2);
                if(USART_FAIL == GSM_QueryCreg(&stCREGInfo))
                {
                    // clear creg info
                }

                gsmRtn = GSM_QueryImei(ucIMEIBuf);
                if(RST_FAIL == gsmRtn)
                {
                    // clear imei buffer
                }

                gsmRtn = GSM_QueryImsi(&stIMSIInfo);
                if(RST_FAIL == gsmRtn)
                {
                    // clear imsi info
                }

                GSM_SetCIPMode(0);

                gsmRtn = GSM_CheckGPRSService();
                if(RST_FAIL == gsmRtn)
                {
                    continue;
                }

                gsmRtn = GSM_QueryBatVoltage(&stBATInfo);
                if(RST_FAIL == gsmRtn)
                {
                    // clear battery info
                }
#if 0
                gsmRtn = GSM_ceng(&stStationInfo);
                if(RST_FAIL == gsmRtn)
                {
                    // clear battery info
                }
#endif
                while(gprsRetyTimes < GPRS_RETRY_TIMES)
                {
                    gprsRetyTimes++;
                    GPRS_CIPShut();
                    GPRS_CheckLinkStatus(&status);
                    gsmRtn = GSM_StartTaskAndSetAPN();
                    if(RST_FAIL == gsmRtn)
                    {
                        continue;
                    }
                    GPRS_CheckLinkStatus(&status);
                    gsmRtn = GSM_BringUpConnect();
                    if(RST_FAIL == gsmRtn)
                    {
                        //continue;
                    }
                    GPRS_CheckLinkStatus(&status);
                    gsmRtn = GSM_GetLocalIP();
                    if(RST_FAIL == gsmRtn)
                    {
                        //continue;
                    }
                    GPRS_CheckLinkStatus(&status);
                    gsmRtn = GSM_StartUpConnect();
                    if(RST_FAIL == gsmRtn)
                    {
                        continue;
                    }
                    GPRS_CheckLinkStatus(&status);

                    errNum = 0;
                    g_usSequenceNum++;
                    // detect remove action and alarm flag is setted then send alarm msg
                    if((1 == alarmStickFlag)
                            && (SET == g_uiAlarmFlag))
                    {
                        PackAlarmMsg();
                        sendLen = EELINK_ALARM_MSGLEN;
                        g_uiAlarmPacketFlag = SET;
                        DEBUG("PackAlarmMsg\r\n");
                    }
                    else
                    {
                        PackGpsMsg();
                        sendLen = EELINK_GPS_MSGLEN;
                        g_uiAlarmPacketFlag = RESET;
                        DEBUG("PackGpsMsg\r\n");
                    }
#ifdef DBG_ENABLE_MACRO
                    ShowGpsMsg();
#endif
                    while(errNum < 5)
                    {
                        errNum++;
                        gprsRtn = GPRS_SendData(gpsBuf, sendLen);
                        if(USART_SUCESS == gprsRtn)
                        {
                            gprsSendFlag = RST_OK;
                            DEBUG("send_ok alarmflag = %d; g_alarmPacketFlag = %d\n", g_uiAlarmFlag, g_uiAlarmPacketFlag);
                            // Toggle alarm flag
                            if((SET == g_uiAlarmFlag) && (SET == g_uiAlarmPacketFlag))
                            {
                                g_uiAlarmFlag = RESET;
                            }
                            else
                            {
                                g_uiAlarmFlag = SET;
                            }
                            DEBUG("send_ok alarmflag = %d\n", g_uiAlarmFlag);
                            break;
                        }
                    }
                    errNum = 0;

                    // break gprs send process
                    if(RST_OK == gprsSendFlag)
                    {
                        break;
                    }

                }
                gprsRetyTimes = 0;
                // break gprs send process
                if(RST_OK == gprsSendFlag)
                {
                    break;
                }

            }
        }

        /////////////////////////////////////////////////////////////////
        // This Process is Finished, Then goto sleep
        /////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////
        // Power OFF GPS and GSM before going into sleep mode
        /////////////////////////////////////////////////////////////////
        STM_EVAL_LEDOff(LED1);
        GPRS_CPOwd();
        GPSPowerOff();
        GSM_PowerOff();

        /* Wait till RTC Second event occurs */
        RTC_ClearFlag(RTC_FLAG_SEC);
        while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

        // NOT Stick and Alarm Flag Valid, Then Sleep Less
        if(((uint32_t)Bit_SET == STM_EVAL_PBGetState(BUTTON_KEY))
                && (SET == g_uiAlarmFlag))
        {
            RTC_SetAlarm(RTC_GetCounter() + SLEEP_ALARM_SEC);
            DEBUG("alarmmode sleep\n");
        }
        else
        {
#ifdef DBG_ENABLE_MACRO
            RTC_SetAlarm(RTC_GetCounter() + 120); // 2 min
#else
            RTC_SetAlarm(RTC_GetCounter() + g_uiSetSleepSec);
            DEBUG("normalmode sleep %d\n", g_uiSetSleepSec);
#endif

        }

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();

        /* Request to enter STOP mode with regulator in low power mode*/
        PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

        /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
           PLL as system clock source (HSE and PLL are disabled in STOP mode) */
        SYSCLKConfig_STOP();
    }

    // SHOULD NOT GO HAERE
    while(1)
    {
        STM_EVAL_LEDToggle(LED1);
        delay_10ms(50);
    }
}

/*********************************************************************************************************
 ** Function name:       ProcessIMEI()
 ** Descriptions:        将15个字节的IMEI字符串处理成8个字节的EELINK协议格式
 ** input parameters:    pImei 需要处理的15个字节的IMEI字符串
 **                      pRst 返回的字符串指针
 **                      imeilen IMEI字符串的长度
 **                      rstlen 返回的字符串的长度
 ** output parameters:
 ** Returned value:      成功RST_OK 失败RST_FAIL
 *********************************************************************************************************/
uint8_t ProcessIMEI(uint8_t *pImei, uint8_t *pRst, int32_t imeilen, int32_t rstlen)
{
    uint32_t i;
    if(((imeilen / 2 + 1) < rstlen) || (imeilen != 15))
    {
        return RST_FAIL;
    }

    *(pRst + 0) = (*(pImei + 0)) & 0x0F;
    for(i = (imeilen / 2); i > 0 ; i--)
    {
        *(pRst + i) = (((*(pImei + i * 2 - 1)) << 4) & 0xF0) + ((*(pImei + i * 2)) & 0x0F);
    }
    return RST_OK;
}

/**
  * @brief  Load IMEI to stLoginMsg structure
  * @param  None
  * @retval None
  */
void loadLoginMsg(uint8_t *imei, uint16_t sequence)
{
    stLoginMsg.hdr.header[0] = PROTO_EELINK_HEADER;
    stLoginMsg.hdr.header[1] = PROTO_EELINK_HEADER;
    stLoginMsg.hdr.type = PACKET_EELINK_LOGIN;
    stLoginMsg.hdr.len[0] = 0x00;
    stLoginMsg.hdr.len[1] = 0x0C;
    stLoginMsg.hdr.seq[0] = (uint8_t)((sequence >> 8) & 0x00FF);
    stLoginMsg.hdr.seq[1] = (uint8_t)((sequence) & 0x00FF);
    if(RST_FAIL == ProcessIMEI(imei, stLoginMsg.imei, IMEI_BUF_LEN, 8))
    {
        // re-init imei buffer
        memset(stLoginMsg.imei, 0, sizeof(stLoginMsg.imei));
        DEBUG("IMEI process ERROR!\n");
    }
    stLoginMsg.lang = EELINK_LANG; // english
    stLoginMsg.zone = EELINK_ZONE; // east 8
}

/**
  * @brief  Load stLoginMsg structure to send buffer
  * @param  None
  * @retval None
  */
void PackLoginMsg(void)
{
    uint32_t i;
    uint32_t offset = 0;
    offset = 0;
    for(i = 0; i < 2; i++)
    {
        LoginBuf[offset] = stLoginMsg.hdr.header[i];
        offset++;
    }
    LoginBuf[offset] = stLoginMsg.hdr.type;
    offset++;
    for(i = 0; i < 2; i++)
    {
        LoginBuf[offset] = stLoginMsg.hdr.len[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        LoginBuf[offset] = stLoginMsg.hdr.seq[i];
        offset++;
    }
    for(i = 0; i < 8; i++)
    {
        LoginBuf[offset] = stLoginMsg.imei[i];
        offset++;
    }
    LoginBuf[offset] = stLoginMsg.lang;
    offset++;
    LoginBuf[offset] = stLoginMsg.zone;
    offset++;
    if(offset != (EELINK_LOGIN_MSGLEN))
    {
        DEBUG("PackLoginMsg ERROR!\n");
    }
}


/**
  * @brief  Load gpsMsg structure to send buffer
  * @param  None
  * @retval None
  */
void PackGpsMsg(void)
{
    uint32_t i;
    uint32_t offset = 0;
    offset = 0;
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = PROTO_EELINK_HEADER;
        offset++;
    }
    gpsBuf[offset] = PACKET_EELINK_GPS;
    offset++;
    gpsBuf[offset] = 0x00;
    offset++;
    gpsBuf[offset] = (EELINK_GPS_MSGLEN - 5); // 37
    offset++;
    gpsBuf[offset] = (uint8_t)((g_usSequenceNum >> 8) & 0x00FF);
    offset++;
    gpsBuf[offset] = (uint8_t)((g_usSequenceNum) & 0x00FF);
    offset++;
    // UTC日期时间
    for(i = 0; i < 4; i++)
    {
        gpsBuf[offset] = stGPSData.utc.s[3 - i];
        offset++;
    }
    // 纬度
    for(i = 0; i < 4; i++)
    {
        gpsBuf[offset] = stGPSData.latitude.s[3 - i];
        offset++;
    }
    // 经度
    for(i = 0; i < 4; i++)
    {
        gpsBuf[offset] = stGPSData.longitude.s[3 - i];
        offset++;
    }
    // 速度
    gpsBuf[offset] = stGPSData.speed;
    offset++;
    // 航向
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = stGPSData.course.s[1 - i];
        offset++;
    }
    // 基站 9 bytes
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = stIMSIInfo.Mcc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = stIMSIInfo.Mnc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = stCREGInfo.Lac[i];
        offset++;
    }
    gpsBuf[offset] = 0;  // 补零
    offset++;
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = stCREGInfo.Ci[i];
        offset++;
    }
    // 定位状态
    gpsBuf[offset] = stGPSData.status;
    offset++;
    // 设备状态
    gpsBuf[offset] = 0x00;  // 补零
    offset++;
    gpsBuf[offset] = stGPSData.status;
    offset++;
    // 电池电压
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = stBATInfo.BatVoltage.s[1 - i];
        offset++;
    }
    // 信号强度
    gpsBuf[offset] = 0;  // 补零
    offset++;
    gpsBuf[offset] = ucSignalQuality;
    offset++;
    // 模拟输入1
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_deviceData.analog1[i];
        offset++;
    }
    // 模拟输入2
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_deviceData.analog2[i];
        offset++;
    }

    if(offset != (EELINK_GPS_MSGLEN))
    {
        DEBUG("PackGpsMsg ERROR!\n");
    }
}

/**
  * @brief  Load gpsMsg structure to alarm send buffer
  * @param  None
  * @retval None
  */
void PackAlarmMsg(void)
{
    uint32_t i;
    uint32_t offset;
    offset = 0;
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = PROTO_EELINK_HEADER;
        offset++;
    }
    gpsBuf[offset] = PACKET_EELINK_WARNING; //gpsMsg.hdr.type;
    offset++;
    gpsBuf[offset] = 0; // header len
    offset++;
    gpsBuf[offset] = (EELINK_ALARM_MSGLEN - 5); // header len
    offset++;
    gpsBuf[offset] = (uint8_t)((g_usSequenceNum >> 8) & 0x00FF);
    offset++;
    gpsBuf[offset] = (uint8_t)((g_usSequenceNum) & 0x00FF);
    offset++;

    for(i = 0; i < 4; i++)
    {
        gpsBuf[offset] = stGPSData.utc.s[3 - i];
        offset++;
    }
    for(i = 0; i < 4; i++)
    {
        gpsBuf[offset] = stGPSData.latitude.s[3 - i];
        offset++;
    }
    for(i = 0; i < 4; i++)
    {
        gpsBuf[offset] = stGPSData.longitude.s[3 - i];
        offset++;
    }
    gpsBuf[offset] = stGPSData.speed;
    offset++;
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = stGPSData.course.s[1 - i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = stIMSIInfo.Mcc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = stIMSIInfo.Mnc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = stCREGInfo.Lac[i];
        offset++;
    }
    gpsBuf[offset] = 0;  // 补零
    offset++;
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = stCREGInfo.Ci[i];
        offset++;
    }
    gpsBuf[offset] = stGPSData.status;
    offset++;
    gpsBuf[offset] = 0x71;  // remove alarm flag
    offset++;

    if(offset != (EELINK_ALARM_MSGLEN))
    {
        DEBUG("PackAlarmMsg ERROR!\n");
    }
}

/**
  * @brief  Load station structure to station buffer
  * @param  None
  * @retval None
  */
void PackStationMsg(void)
{
    uint32_t i;
    uint32_t j = 0;
    uint32_t offset;
    offset = 0;
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = PROTO_EELINK_HEADER;
        offset++;
    }
    stationBuf[offset] = PACKET_EELINK_STATION; //gpsMsg.hdr.type;
    offset++;
    stationBuf[offset] = 0; // header len
    offset++;
    stationBuf[offset] = (3 + (stStationInfo.num * 11)); // header len = 7+1+num*11-5
    offset++;
    stationBuf[offset] = (uint8_t)((g_usSequenceNum >> 8) & 0x00FF);
    offset++;
    stationBuf[offset] = (uint8_t)((g_usSequenceNum) & 0x00FF);
    offset++;

    stationBuf[offset] = stStationInfo.num;
    offset++;

    for(i = 0; i < stStationInfo.num; i++)
    {
        for(j = 0; j < 2; j++)
        {
            stationBuf[offset] = stStationInfo.stStation[i].mcc.s[1 - j];
            offset++;
        }
        for(j = 0; j < 2; j++)
        {
            stationBuf[offset] = stStationInfo.stStation[i].mnc.s[1 - j];
            offset++;
        }
        for(j = 0; j < 2; j++)
        {
            stationBuf[offset] = stStationInfo.stStation[i].lac.s[1 - j];
            offset++;
        }
        for(j = 0; j < 3; j++)
        {
            stationBuf[offset] = stStationInfo.stStation[i].ci.s[2 - j];
            offset++;
        }
        stationBuf[offset] = stStationInfo.stStation[i].rxl;
        offset++;
        stationBuf[offset] = stStationInfo.stStation[i].rxq;
        offset++;
    }

    if(offset != (8 + stStationInfo.num * 11))
    {
        DEBUG("PackStationMsg ERROR!\n");
    }
}

/**
  * @brief  Load gpsMsg and stLoginMsg structure to factory send buffer
  * @param  None
  * @retval None
  */
void PackFactoryMsg(void)
{
    uint32_t i;
    uint32_t offset;
    offset = 0;
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = PROTO_FACTORY_HEADER;
        offset++;
    }
    stationBuf[offset] = PACKET_FACTORY_REPORT;  // header type
    offset++;
    stationBuf[offset] = 0; // header len
    offset++;
    stationBuf[offset] = (FACTORY_REPORT_MSGLEN - 5); // header len
    offset++;
    stationBuf[offset] = (uint8_t)((g_usSequenceNum >> 8) & 0x00FF);
    offset++;
    stationBuf[offset] = (uint8_t)((g_usSequenceNum) & 0x00FF);
    offset++;
    for(i = 0; i < 4; i++)
    {
        stationBuf[offset] = stGPSData.utc.s[3 - i];
        offset++;
    }
    for(i = 0; i < 4; i++)
    {
        stationBuf[offset] = stGPSData.latitude.s[3 - i];
        offset++;
    }
    for(i = 0; i < 4; i++)
    {
        stationBuf[offset] = stGPSData.longitude.s[3 - i];
        offset++;
    }
    stationBuf[offset] = stGPSData.speed;
    offset++;


    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = stGPSData.course.s[1 - i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = stIMSIInfo.Mcc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = stIMSIInfo.Mnc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = stCREGInfo.Lac[i];
        offset++;
    }
    stationBuf[offset] = 0;  // 补零
    offset++;
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = stCREGInfo.Ci[i];
        offset++;
    }
    stationBuf[offset] = stGPSData.status;
    offset++;
    stationBuf[offset] = 0;  // device status
    offset++;
    stationBuf[offset] = STM_EVAL_PBGetState(BUTTON_KEY);  // device status
    offset++;
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = stBATInfo.BatVoltage.s[1 - i];
        offset++;
    }

    stationBuf[offset] = 0;  // 补零
    offset++;
    stationBuf[offset] = ucSignalQuality;
    offset++;
    for(i = 0; i < IMEI_BUF_LEN; i++)
    {
        stationBuf[offset] = ucIMEIBuf[i];
        offset++;
    }

    for(i = 0; i < IMSI_INFO_LEN; i++)
    {
        stationBuf[offset] = g_IMSIBuf[i];
        offset++;
    }
    for(i = 0; i < PHONE_NUMBER_LEN; i++)
    {
        stationBuf[offset] = g_phoneNum[i];
        offset++;
    }
    stationBuf[offset] = ucGPSStatus;  // gps status
    offset++;
    stationBuf[offset] = 0x02;  // software version v2.0
    offset++;
    stationBuf[offset] = 0x00;
    offset++;

    if(offset != (FACTORY_REPORT_MSGLEN))
    {
        DEBUG("PackFactoryMsg ERROR!\n");
    }
}

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
