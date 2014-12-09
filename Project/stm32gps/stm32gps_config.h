/**
  ******************************************************************************
  * @file    USART/Printf/stm32f10x_it.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_SIM908_CFG_H
#define __STM32_SIM908_CFG_H

#include "stm32f10x.h"
#include "stm32gps_board.h"



#define SYS_TICK_PER_SEC 100u

#ifdef DBG_ENABLE_MACRO
#define GPS_RETERY_TIMES    18  // GPS 重试次数  Should Set 1
#else
#define GPS_RETERY_TIMES    18  // GPS 重试次数 
#endif

#define GSM_RETERY_TIMES    4 // GSM 错误重发次数
#define GPRS_RETRY_TIMES    4 // GPRS 连接重连次数
#define SEND_RETRY_TIMES    4 // GPRS SEND 重发次数

#define STICK_ON_SEC      300  // 3sec stick on times

// RTC Alarm Second [1 - 2^32]
#define SLEEP_NOTLOC_SCALE  5  // GPS Not location sleep time scale 
#define SLEEP_ALARM_SEC    60   // 1min
#define SLEEP_NORMAL_SEC 	86400 // 24hour SHOULD SET 86400 = 24hour
#define GPS_STOPMODE_SEC    10  // GPS STOP mode = 10 seconds

#define SLEEP_TIME_MIN    60     // 1min 最小睡眠间隔
#define SLEEP_TIME_MAX   172800  // 48hour 最大睡眠间隔

// #define GSM_SERVER_IP   "121.40.200.84"  	// server ip address
#define GSM_SERVER_IP   "lkf.broadnetwork.net"  // server name
#define GSM_SERVER_PORT "9999"           		// server port


#define EELINK_LOGIN_MSGLEN  17		// login length
#define EELINK_GPS_MSGLEN    42     // GPS length
#define EELINK_ALARM_MSGLEN  33     // alarm length
#define EELINK_STATION_MSGLEN_MAX 85 // station max length
#define FACTORY_REPORT_MSGLEN 86    // factory test length

#define EELINK_LANG  0x01  			// English
#define EELINK_ZONE  0x20  			// east 8

// 每次进入中断服务程序间隔时间为：
// ((1+TIM_Prescaler )/72M)*(1+TIM_Period )=((1+35999)/72M)*(1+9999)=5秒
#define TIM2_PRESCALER_TIMER 35999 // TIM2 Prescaler
#define TIM2_PRESCALER_HZ 1000    // TIM2 HZ, Not used
#define TIM2_PERIOD_TIMER  9999  // 5s  根据需要可修改该定时器的值

#define TIM4_PRESCALER_TIMER 65535
#define TIM4_PRESCALER_HZ 1000
#define TIM4_PERIOD_TIMER  35534  // 3min

#define USART_GSM_BUFSIZE_SEND 256
#define USART_GSM_BUFSIZE_RECEIVE 1024
#define USART_GSM_BUFSIZE    1024
#define USART_GPS_BUFSIZE    1024
#define GSM_USART_TIMEOUT_MS 20
#define GPS_USART_TIMEOUT_MS 20

#define STM32_SIM908_GPS_COM COM1
#define STM32_SIM908_GSM_COM COM2

// usart baud rate
#define USART_GSM_BAUD 9600
#define USART_GPS_BAUD 9600
#define USART_DBG_BAUD 9600

#define NULL 0
#define RST_OK   1
#define RST_FAIL 0

// used for output debug information
#ifdef DBG_ENABLE_MACRO
#define DEBUG(msg, args...) \
        (void)printf(msg , ##args)
#define DBG_INFO(msg, args...) \
        (void)printf("[INFO]---" msg "\n", ##args)
#define DBG_WARN(msg, args...) \
        (void)printf("[WARN]---" msg "\n", ##args)
#define DBG_ERRO(msg, args...) \
        (void)printf("[ERRO]---" msg "\n", ##args)
#else
#define DEBUG
#define DBG_INFO
#define DBG_WARN
#define DBG_ERRO
#endif // USE_DEBUG


/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);

void delay_10ms(__IO uint32_t nTime);
void delay_ms(uint32_t Timer);
void stm32gps_sys_tick_cfg(void);
void RCC_Configuration(void);
void SYSCLKConfig_STOP(void);
void RTC_Configuration(void);
void EXTI_Configuration(void);
void TIM2_Configuration(void);
void RTC_NVIC_Configuration(void);
void TIM2_NVIC_Configuration(void);
void USART_NVIC_Configuration(void);
void TIM2_Start(void);
void TIM2_Stop(void);
void IWDG_Configuration(void);
void stm32gps_led_cfg(void);
void stm32gps_com_debug_cfg(void);
void stm32gps_com_gps_cfg(void);
void stm32gps_com_gsm_cfg(void);

#endif /* __STM32_SIM908_CFG_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
