/**
 ******************************************************************************
 * @file    stm32_gps_board.c
 * @author  MCD Application Team
 * @version V4.5.0
 * @date    07-March-2011
 * @brief   This file provides
 *            - set of firmware functions to manage Leds, push-button and COM ports
 *            - low level initialization functions for SD card (on SPI) and I2C
 *              serial EEPROM (sEE)
 *          available on STM3210C-EVAL evaluation board from STMicroelectronics.
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
#include "stm32gps_board.h"
#include "stm32f10x_it_api.h"
#include "usart.h"

/**
 * @brief  Turns User LED On.
 * @param  None
 * @retval None
 */
void STM_EVAL_LEDOn(void)
{
    LED1_GPIO_PORT->BRR = LED1_PIN;
}

/**
 * @brief  Turns User LED Off.
 * @param  None
 * @retval None
 */
void STM_EVAL_LEDOff(void)
{
    LED1_GPIO_PORT->BSRR = LED1_PIN;
}

/**
 * @brief  Toggles the User LED.
 * @param  None
 * @retval None
 */
void STM_EVAL_LEDToggle(void)
{
    LED1_GPIO_PORT->ODR ^= LED1_PIN;
}


/**
  * @brief  Returns the Button state.
  * @param  None.
  * @retval The Button GPIO pin value.
  */
uint32_t STM_EVAL_PBGetState(void)
{
    return GPIO_ReadInputDataBit(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN);
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
