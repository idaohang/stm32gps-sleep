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
#include "device.h"
#include "eelink.h"
#include "gpio.h"
#include "gps.h"


/* Private typedef -----------------------------------------------------------*/



/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint16_t g_usSequenceNum;  // GPRS packet's sequence number
static uint8_t g_ucGPSStatus;    // GPS status 0x55 - error; 0xaa - ok
static uint32_t g_uiSetSleepSec;  // Server setting sleep seconds
static FlagStatus g_uiAlarmFlag;  // 报警标志 SET-valid; RESET-invalid
static FlagStatus g_uiAlarmPacketFlag; // 报警数据包标志 SET - alarm packet; RESET - not alarm packet

static uint8_t g_ucSignalQuality;

static ST_DEVICEDATA g_stDeviceData;
static ST_BATVOLTAGESTATUS g_stBATInfo;
static ST_CREGINFO g_stCREGInfo;
static ST_IMSIINFO g_stIMSIInfo;
static stru_GPSDATA g_stGPSData;
static ST_PACKET_BASESTATION g_stStationInfo;

static uint8_t g_ucIMEIBuf[IMEI_BUF_LEN];
static uint8_t g_ucIMSIBuf[IMSI_BUF_LEN];
static uint8_t g_ucPhoneNumBuf[PHONE_NUM_BUF_LEN];

static char LoginBuf[PROTO_LOGIN_BUF_LEN];
static char gpsBuf[PROTO_GPS_BUF_LEN];
static char stationBuf[PROTO_STATION_BUF_LEN];


/* Private function prototypes -----------------------------------------------*/
static uint8_t ProcessIMEI(uint8_t *pImei, uint8_t *pRst, int32_t imeilen, int32_t rstlen);
static void PackLoginMsg(void);
static void PackGpsMsg(void);
static void PackAlarmMsg(void);
static void PackStationMsg(void);
static void PackFactoryMsg(void);
/* Private functions ---------------------------------------------------------*/

#ifdef DBG_ENABLE_MACRO
void ShowLoginMsg(void)
{
    uint32_t i;
    // imei
    DEBUG("IMEI:");
    for(i = 0; i < IMEI_BUF_LEN; i++)
    {
        DEBUG("%c-", g_ucIMEIBuf[i]);
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

void ShowGpsMsg(uint32_t len)
{
    uint32_t i;
    // gps msg
    DEBUG("\r\nGPS or ALARM MSG:");
    for(i = 0; i < len; i++)
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
    for(i = 0; i < (8 + g_stStationInfo.num * 11); i++)
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
    g_usSequenceNum = 1;
    g_ucGPSStatus = GPS_DEVICE_ERR;
    g_uiSetSleepSec = SLEEP_NORMAL_SEC;
    g_uiAlarmFlag = RESET;
    g_uiAlarmPacketFlag = RESET;
	g_ucSignalQuality = 0;

	memset(&g_stDeviceData, 0, sizeof(g_stDeviceData));
	memset(&g_stBATInfo, 0, sizeof(g_stBATInfo));
	memset(&g_stCREGInfo, 0, sizeof(g_stCREGInfo));
	memset(&g_stIMSIInfo, 0, sizeof(g_stIMSIInfo));
	memset(&g_stGPSData, 0, sizeof(g_stGPSData));
	memset(&g_stStationInfo, 0, sizeof(g_stStationInfo));

	memset(g_ucIMEIBuf, 0, IMEI_BUF_LEN);
	memset(g_ucIMSIBuf, 0, IMSI_BUF_LEN);
	memset(g_ucPhoneNumBuf, 0, PHONE_NUM_BUF_LEN);
	
    memset(LoginBuf, 0, PROTO_LOGIN_BUF_LEN);
	memset(gpsBuf, 0, PROTO_GPS_BUF_LEN);
	memset(stationBuf, 0, PROTO_STATION_BUF_LEN);
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
    uint8_t gprsRtn = USART_FAIL;
    uint8_t gprsSendFlag = RST_FAIL; // GPRS Send Status Flag

    uint16_t sendLen = 0;      // GPRS send length (used for GPS and ALARM Msg)
    uint32_t sleepSec = 0;
    uint32_t alarmStickFlag = 0;    // 是否有移开的动作 1- 有 0-无

    // Used for parse GPRS Received Data Analysis
    char *pRecvBuf = NULL;     // GPRS Received buffer
    uint32_t recvLen = 0;      // GPRS Received length
    char *pfeed = NULL;        // Used for parse

    // Used for CheckLinkStatus
#ifdef DBG_ENABLE_MACRO
    uint8_t status = 0;
#endif

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
    // Init Global Variables
    /////////////////////////////////////////////////////////////////
    InitVariables();

    /////////////////////////////////////////////////////////////////
    // Main LOOP
    /////////////////////////////////////////////////////////////////
    while(1)
    {
		/////////////////////////////////////////////////////////////////
	    // Init Local Variables
	    /////////////////////////////////////////////////////////////////
		errNum = 0;
		gpsRecvTimes = 0;
		gsmRetyTimes = 0;
		gprsRetyTimes = 0;
		
        gpsRtn = RST_FAIL;
        gprsSendFlag = RST_FAIL;

		sendLen = 0;
		sleepSec = 0;
		alarmStickFlag = 0;

		/////////////////////////////////////////////////////////////////
	    // Detect Stick Status
	    /////////////////////////////////////////////////////////////////
		delay_10ms(STICK_ON_SEC);
DEBUG("\r\n  g_uiAlarmFlag = %d; button = %d\r\n",g_uiAlarmFlag, STM_EVAL_PBGetState(BUTTON_KEY));
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
            // First, Power ON GPS
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
                    g_ucGPSStatus = GPS_DEVICE_OK;
                    ParseGPSData(&g_stGPSData);
                    if(1 == g_stGPSData.status)
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
            // Second, Power ON GSM
            /////////////////////////////////////////////////////////////////
            GSM_PowerOn();
            delay_10ms(200);

            gsmRetyTimes = 0;
            gsmRtn = RST_FAIL;

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

                gsmRtn = GSM_QuerySignal(&g_ucSignalQuality);
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
                if(USART_FAIL == GSM_QueryCreg(&g_stCREGInfo))
                {
                    // clear creg info
                }

                gsmRtn = GSM_QueryImei(g_ucIMEIBuf);
                if(RST_FAIL == gsmRtn)
                {
                    // clear imei buffer
                }

                gsmRtn = GSM_QueryImsi(&g_stIMSIInfo);
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

                gsmRtn = GSM_QueryBatVoltage(&g_stBATInfo);
                if(RST_FAIL == gsmRtn)
                {
                    // clear battery info
                }

                gsmRtn = GSM_ceng(&g_stStationInfo);
                if(RST_FAIL == gsmRtn)
                {
                    // clear station info
                }
				
				gprsRetyTimes = 0;
				gsmRtn = RST_FAIL;
                while(gprsRetyTimes < GPRS_RETRY_TIMES)
                {
                    gprsRetyTimes++;
                    GPRS_CIPShut();
#ifdef DBG_ENABLE_MACRO
                    GPRS_CheckLinkStatus(&status);
#endif
                    gsmRtn = GSM_StartTaskAndSetAPN();
                    if(RST_FAIL == gsmRtn)
                    {
                        continue;
                    }
#ifdef DBG_ENABLE_MACRO
                    GPRS_CheckLinkStatus(&status);
#endif
                    gsmRtn = GSM_BringUpConnect();
                    if(RST_FAIL == gsmRtn)
                    {
                        //continue;
                    }
#ifdef DBG_ENABLE_MACRO
                    GPRS_CheckLinkStatus(&status);
#endif
                    gsmRtn = GSM_GetLocalIP();
                    if(RST_FAIL == gsmRtn)
                    {
                        //continue;
                    }
#ifdef DBG_ENABLE_MACRO
                    GPRS_CheckLinkStatus(&status);
#endif
                    gsmRtn = GSM_StartUpConnect();
                    if(RST_FAIL == gsmRtn)
                    {
                        continue;
                    }
#ifdef DBG_ENABLE_MACRO
                    GPRS_CheckLinkStatus(&status);
#endif
					/////////////////////////////////////////////////////////////////
		            // Send LOGIN Msg
		            /////////////////////////////////////////////////////////////////
					g_usSequenceNum = 1; // Init packet sequence to 1
                    errNum = 0;
					gprsRtn = USART_FAIL;

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
					
					/////////////////////////////////////////////////////////////////
		            // Send GPS/STATION or ALARM Msg
		            /////////////////////////////////////////////////////////////////
                    g_usSequenceNum++;
					errNum = 0;
					gprsRtn = USART_FAIL;
                    // detect remove action and alarm flag is setted then send alarm msg
DEBUG("alarmStickFlag = %d; g_uiAlarmFlag = %d\n",alarmStickFlag, g_uiAlarmFlag);
                    if((1 == alarmStickFlag)
                            && (SET == g_uiAlarmFlag))
                    {
                        PackAlarmMsg();
                        sendLen = EELINK_ALARM_MSGLEN;
                        g_uiAlarmPacketFlag = SET;
                        DEBUG("PackAlarmMsg\r\n");
#ifdef DBG_ENABLE_MACRO
ShowGpsMsg(sendLen);
#endif
                    }
                    else
                    {
                    	if(1 == g_stGPSData.status)
                    	{
	                        PackGpsMsg();
	                        sendLen = EELINK_GPS_MSGLEN;
	                        DEBUG("PackGpsMsg\r\n");
#ifdef DBG_ENABLE_MACRO
ShowGpsMsg(sendLen);
#endif
                    	}
                    	else 
                    	{
                    		PackStationMsg();
                        	sendLen = (8 + (g_stStationInfo.num * 11));
#ifdef DBG_ENABLE_MACRO
                        	ShowStationMsg();
#endif
                    	}
                    	
                    	g_uiAlarmPacketFlag = RESET;
                    }

#if 0
PackFactoryMsg();
sendLen = FACTORY_REPORT_MSGLEN;
#ifdef DBG_ENABLE_MACRO
                        ShowFactoryMsg();
#endif
#endif
                    while(errNum < 5)
                    {
                        errNum++;
                        gprsRtn = GPRS_SendData(gpsBuf, sendLen);
                        if(USART_SUCESS == gprsRtn)
                        {
                            gprsSendFlag = RST_OK;
                            DEBUG("send_ok before g_uiAlarmFlag = %d; g_uiAlarmPacketFlag = %d\n", g_uiAlarmFlag, g_uiAlarmPacketFlag);
                            // Toggle alarm flag
                            if((SET == g_uiAlarmFlag) && (SET == g_uiAlarmPacketFlag))
                            {
                                g_uiAlarmFlag = RESET;
                            }
                            else
                            {
                                g_uiAlarmFlag = SET;
                            }
                            DEBUG("send_ok after g_uiAlarmFlag = %d\n", g_uiAlarmFlag);
                            break;
                        }
                    }

                    // break gprs send process
                    if(RST_OK == gprsSendFlag)
                    {
                        break;
                    }

                }
                
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
STM_EVAL_LEDToggle(LED1);
        /////////////////////////////////////////////////////////////////
        // Power OFF GPS and GSM before going into sleep mode
        /////////////////////////////////////////////////////////////////
        //STM_EVAL_LEDOff(LED1);
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
            DEBUG("alarmmode sleep %d\n", SLEEP_ALARM_SEC);
        }
        else
        {
#ifdef DBG_ENABLE_MACRO
            RTC_SetAlarm(RTC_GetCounter() + 120); // 2 min
            DEBUG("DEBUG normalmode sleep %d\n", 120);
#else
RTC_SetAlarm(RTC_GetCounter() + 120); // 2 min
            //RTC_SetAlarm(RTC_GetCounter() + g_uiSetSleepSec);
            //DEBUG("normalmode sleep %d\n", g_uiSetSleepSec);
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
  * @brief  Load stLoginMsg structure to send buffer
  * @param  None
  * @retval None
  */
void PackLoginMsg(void)
{
    uint32_t i;
    uint32_t offset;
	uint8_t ucIMEIBuf[IMEI_BUF_LEN];
	
    offset = 0;
	// header
    for(i = 0; i < 2; i++)
    {
        LoginBuf[offset] = PROTO_EELINK_HEADER;
        offset++;
    }
	// type
    LoginBuf[offset] = PACKET_EELINK_LOGIN;
    offset++;
	// length
	LoginBuf[offset] = 0x00;
    offset++;
	LoginBuf[offset] = 0x0C;
    offset++;
	// sequence
	LoginBuf[offset] = (uint8_t)((g_usSequenceNum >> 8) & 0x00FF);
    offset++;
	LoginBuf[offset] = (uint8_t)((g_usSequenceNum) & 0x00FF);
    offset++;
    // imei
    if(RST_FAIL == ProcessIMEI(g_ucIMEIBuf, ucIMEIBuf, IMEI_BUF_LEN, 8))
    {
        // re-init imei buffer
        memset(ucIMEIBuf, 0, IMEI_BUF_LEN);
        DEBUG("IMEI process ERROR!\n");
    }
    for(i = 0; i < 8; i++)
    {
        LoginBuf[offset] = ucIMEIBuf[i];
        offset++;
    }
	// lang
    LoginBuf[offset] = EELINK_LANG;
    offset++;
	// zone
    LoginBuf[offset] = EELINK_ZONE;
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
        gpsBuf[offset] = g_stGPSData.utc.s[3 - i];
        offset++;
    }
    // 纬度
    for(i = 0; i < 4; i++)
    {
        gpsBuf[offset] = g_stGPSData.latitude.s[3 - i];
        offset++;
    }
    // 经度
    for(i = 0; i < 4; i++)
    {
        gpsBuf[offset] = g_stGPSData.longitude.s[3 - i];
        offset++;
    }
    // 速度
    gpsBuf[offset] = g_stGPSData.speed;
    offset++;
    // 航向
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stGPSData.course.s[1 - i];
        offset++;
    }
    // 基站 9 bytes
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stIMSIInfo.Mcc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stIMSIInfo.Mnc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stCREGInfo.Lac[i];
        offset++;
    }
    gpsBuf[offset] = 0;  // 补零
    offset++;
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stCREGInfo.Ci[i];
        offset++;
    }
    // 定位状态
    gpsBuf[offset] = g_stGPSData.status;
    offset++;
    // 设备状态
    gpsBuf[offset] = 0x00;  // 补零
    offset++;
    gpsBuf[offset] = g_stGPSData.status;
    offset++;
    // 电池电压
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stBATInfo.BatVoltage.s[1 - i];
        offset++;
    }
    // 信号强度
    gpsBuf[offset] = 0;  // 补零
    offset++;
    gpsBuf[offset] = g_ucSignalQuality;
    offset++;
    // 模拟输入1
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stDeviceData.analog1[i];
        offset++;
    }
    // 模拟输入2
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stDeviceData.analog2[i];
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
        gpsBuf[offset] = g_stGPSData.utc.s[3 - i];
        offset++;
    }
    for(i = 0; i < 4; i++)
    {
        gpsBuf[offset] = g_stGPSData.latitude.s[3 - i];
        offset++;
    }
    for(i = 0; i < 4; i++)
    {
        gpsBuf[offset] = g_stGPSData.longitude.s[3 - i];
        offset++;
    }
    gpsBuf[offset] = g_stGPSData.speed;
    offset++;
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stGPSData.course.s[1 - i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stIMSIInfo.Mcc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stIMSIInfo.Mnc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stCREGInfo.Lac[i];
        offset++;
    }
    gpsBuf[offset] = 0;  // 补零
    offset++;
    for(i = 0; i < 2; i++)
    {
        gpsBuf[offset] = g_stCREGInfo.Ci[i];
        offset++;
    }
    gpsBuf[offset] = g_stGPSData.status;
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
    stationBuf[offset] = (3 + (g_stStationInfo.num * 11)); // header len = 7+1+num*11-5
    offset++;
    stationBuf[offset] = (uint8_t)((g_usSequenceNum >> 8) & 0x00FF);
    offset++;
    stationBuf[offset] = (uint8_t)((g_usSequenceNum) & 0x00FF);
    offset++;

    stationBuf[offset] = g_stStationInfo.num;
    offset++;

    for(i = 0; i < g_stStationInfo.num; i++)
    {
        for(j = 0; j < 2; j++)
        {
            stationBuf[offset] = g_stStationInfo.stStation[i].mcc.s[1 - j];
            offset++;
        }
        for(j = 0; j < 2; j++)
        {
            stationBuf[offset] = g_stStationInfo.stStation[i].mnc.s[1 - j];
            offset++;
        }
        for(j = 0; j < 2; j++)
        {
            stationBuf[offset] = g_stStationInfo.stStation[i].lac.s[1 - j];
            offset++;
        }
        for(j = 0; j < 3; j++)
        {
            stationBuf[offset] = g_stStationInfo.stStation[i].ci.s[2 - j];
            offset++;
        }
        stationBuf[offset] = g_stStationInfo.stStation[i].rxl;
        offset++;
        stationBuf[offset] = g_stStationInfo.stStation[i].rxq;
        offset++;
    }

    if(offset != (8 + g_stStationInfo.num * 11))
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
        stationBuf[offset] = g_stGPSData.utc.s[3 - i];
        offset++;
    }
    for(i = 0; i < 4; i++)
    {
        stationBuf[offset] = g_stGPSData.latitude.s[3 - i];
        offset++;
    }
    for(i = 0; i < 4; i++)
    {
        stationBuf[offset] = g_stGPSData.longitude.s[3 - i];
        offset++;
    }
    stationBuf[offset] = g_stGPSData.speed;
    offset++;


    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = g_stGPSData.course.s[1 - i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = g_stIMSIInfo.Mcc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = g_stIMSIInfo.Mnc[i];
        offset++;
    }
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = g_stCREGInfo.Lac[i];
        offset++;
    }
    stationBuf[offset] = 0;  // 补零
    offset++;
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = g_stCREGInfo.Ci[i];
        offset++;
    }
    stationBuf[offset] = g_stGPSData.status;
    offset++;
    stationBuf[offset] = 0;  // device status
    offset++;
    stationBuf[offset] = STM_EVAL_PBGetState(BUTTON_KEY);  // device status
    offset++;
    for(i = 0; i < 2; i++)
    {
        stationBuf[offset] = g_stBATInfo.BatVoltage.s[1 - i];
        offset++;
    }

    stationBuf[offset] = 0;  // 补零
    offset++;
    stationBuf[offset] = g_ucSignalQuality;
    offset++;
    for(i = 0; i < IMEI_BUF_LEN; i++)
    {
        stationBuf[offset] = g_ucIMEIBuf[i];
        offset++;
    }

    for(i = 0; i < IMSI_BUF_LEN; i++)
    {
        stationBuf[offset] = g_ucIMSIBuf[i];
        offset++;
    }
    for(i = 0; i < PHONE_NUM_BUF_LEN; i++)
    {
        stationBuf[offset] = g_ucPhoneNumBuf[i];
        offset++;
    }
    stationBuf[offset] = g_ucGPSStatus;  // gps status
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
