/**
  ******************************************************************************
  * @file    stm32_gps_board.h
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file contains definitions for STM3210C_EVAL's Leds, push-buttons
  *          COM ports, SD Card on SPI and sEE on I2C hardware resources.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_GPS_BOARD_H
#define __STM32_GPS_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32gps_board.h"

#define GPS_PWR_CTRL_PORT	GPIOC
#define GPS_PWR_CTRL_PIN	GPIO_Pin_11

#define GSM_PWR_CTRL_PORT	GPIOC
#define GSM_PWR_CTRL_PIN	GPIO_Pin_9

#define GSM_PWRKEY_PORT		GPIOA
#define GSM_PWRKEY_PIN		GPIO_Pin_1

#define LED1_PIN                         GPIO_Pin_12
#define LED1_GPIO_PORT                   GPIOC

// Key Button
#define KEY_BUTTON_PIN                   GPIO_Pin_15
#define KEY_BUTTON_GPIO_PORT             GPIOA
#define KEY_BUTTON_EXTI_LINE             EXTI_Line15
#define KEY_BUTTON_EXTI_PORT_SOURCE      GPIO_PortSourceGPIOA
#define KEY_BUTTON_EXTI_PIN_SOURCE       GPIO_PinSource15
#define KEY_BUTTON_EXTI_IRQn             EXTI15_10_IRQn

/**
 * @brief Definition for COM port
 */
#define EVAL_COM1                        USART1
#define EVAL_COM2                        USART2
#define EVAL_COM3                        USART3

void STM_EVAL_LEDOn(void);
void STM_EVAL_LEDOff(void);
void STM_EVAL_LEDToggle(void);
uint32_t STM_EVAL_PBGetState(void);


#ifdef __cplusplus
}
#endif

#endif /* __STM32_GPS_BOARD_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
