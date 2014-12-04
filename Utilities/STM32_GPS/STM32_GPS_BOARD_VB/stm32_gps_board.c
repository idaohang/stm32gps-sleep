/**
 ******************************************************************************
 * @file    stm3210c_eval.c
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

/** @addtogroup Utilities
 * @{
 */

/** @addtogroup STM32_EVAL
 * @{
 */

/** @addtogroup STM3210C_EVAL
 * @{
 */

/** @defgroup STM3210C_EVAL_LOW_LEVEL
 * @brief This file provides firmware functions to manage Leds, push-buttons,
 *        COM ports, SD card on SPI and EEPROM (sEE) available on STM3210C-EVAL
 *        evaluation board from STMicroelectronics.
 * @{
 */

/** @defgroup STM3210C_EVAL_LOW_LEVEL_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup STM3210C_EVAL_LOW_LEVEL_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup STM3210C_EVAL_LOW_LEVEL_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup STM3210C_EVAL_LOW_LEVEL_Private_Variables
 * @{
 */
GPIO_TypeDef *GPIO_PORT[LEDn] = { LED1_GPIO_PORT,
                                  LED2_GPIO_PORT,
                                  LED3_GPIO_PORT,
                                  LED4_GPIO_PORT
                                };
const uint16_t GPIO_PIN[LEDn] = { LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN };
const uint32_t GPIO_CLK[LEDn] = { LED1_GPIO_CLK,
                                  LED2_GPIO_CLK,
                                  LED3_GPIO_CLK,
                                  LED4_GPIO_CLK
                                };

GPIO_TypeDef *BUTTON_PORT[BUTTONn] = {KEY_BUTTON_GPIO_PORT};

const uint16_t BUTTON_PIN[BUTTONn] = {KEY_BUTTON_PIN};
const uint32_t BUTTON_CLK[BUTTONn] = {KEY_BUTTON_GPIO_CLK};

const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {KEY_BUTTON_EXTI_LINE};

const uint16_t BUTTON_PORT_SOURCE[BUTTONn] = {KEY_BUTTON_EXTI_PORT_SOURCE};

const uint16_t BUTTON_PIN_SOURCE[BUTTONn] = {KEY_BUTTON_EXTI_PIN_SOURCE};

const uint16_t BUTTON_IRQn[BUTTONn] = {KEY_BUTTON_EXTI_IRQn};


USART_TypeDef *COM_USART[COMn] = { EVAL_COM1, EVAL_COM2, EVAL_COM3 };

GPIO_TypeDef *COM_TX_PORT[COMn] = { EVAL_COM1_TX_GPIO_PORT,
                                    EVAL_COM2_TX_GPIO_PORT,
                                    EVAL_COM3_TX_GPIO_PORT
                                  };

GPIO_TypeDef *COM_RX_PORT[COMn] = { EVAL_COM1_RX_GPIO_PORT,
                                    EVAL_COM2_RX_GPIO_PORT,
                                    EVAL_COM3_RX_GPIO_PORT
                                  };

const uint32_t COM_USART_CLK[COMn] = { EVAL_COM1_CLK,
                                       EVAL_COM2_CLK,
                                       EVAL_COM3_CLK
                                     };

const uint32_t COM_TX_PORT_CLK[COMn] = { EVAL_COM1_TX_GPIO_CLK,
                                       EVAL_COM2_TX_GPIO_CLK,
                                       EVAL_COM3_TX_GPIO_CLK
                                       };

const uint32_t COM_RX_PORT_CLK[COMn] = { EVAL_COM1_RX_GPIO_CLK,
                                       EVAL_COM2_RX_GPIO_CLK,
                                       EVAL_COM3_RX_GPIO_CLK
                                       };

const uint16_t COM_TX_PIN[COMn] = { EVAL_COM1_TX_PIN,
                                    EVAL_COM2_TX_PIN,
                                    EVAL_COM3_TX_PIN
                                  };

const uint16_t COM_RX_PIN[COMn] = { EVAL_COM1_RX_PIN,
                                    EVAL_COM2_RX_PIN,
                                    EVAL_COM3_RX_PIN
                                  };

/**
 * @}
 */

/** @defgroup STM3210C_EVAL_LOW_LEVEL_Private_FunctionPrototypes
 * @{
 */
/**
 * @}
 */

/** @defgroup STM3210C_EVAL_LOW_LEVEL_Private_Functions
 * @{
 */

/**
 * @brief  Configures LED GPIO.
 * @param  Led: Specifies the Led to be configured.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 *     @arg LED3
 *     @arg LED4
 * @retval None
 */
void STM_EVAL_LEDInit(Led_TypeDef Led)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_APB2PeriphClockCmd(GPIO_CLK[Led], ENABLE);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
}


/**
 * @brief  Turns selected LED On.
 * @param  Led: Specifies the Led to be set on.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 *     @arg LED3
 *     @arg LED4
 * @retval None
 */
void STM_EVAL_LEDOn(Led_TypeDef Led)
{
    GPIO_PORT[Led]->BRR = GPIO_PIN[Led];
}

/**
 * @brief  Turns selected LED Off.
 * @param  Led: Specifies the Led to be set off.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 *     @arg LED3
 *     @arg LED4
 * @retval None
 */
void STM_EVAL_LEDOff(Led_TypeDef Led)
{
    GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];
}

/**
 * @brief  Toggles the selected LED.
 * @param  Led: Specifies the Led to be toggled.
 *   This parameter can be one of following parameters:
 *     @arg LED1
 *     @arg LED2
 *     @arg LED3
 *     @arg LED4
 * @retval None
 */
void STM_EVAL_LEDToggle(Led_TypeDef Led)
{
    GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_WAKEUP: Wakeup Push Button
  *     @arg BUTTON_TAMPER: Tamper Push Button
  *     @arg BUTTON_KEY: Key Push Button
  *     @arg BUTTON_RIGHT: Joystick Right Push Button
  *     @arg BUTTON_LEFT: Joystick Left Push Button
  *     @arg BUTTON_UP: Joystick Up Push Button
  *     @arg BUTTON_DOWN: Joystick Down Push Button
  *     @arg BUTTON_SEL: Joystick Sel Push Button
  * @param  Button_Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                     generation capability
  * @retval None
  */
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the BUTTON Clock */
    RCC_APB2PeriphClockCmd(BUTTON_CLK[Button], ENABLE);

    /* Configure Button pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
    GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);


    if (Button_Mode == BUTTON_MODE_EXTI)
    {
        /* Connect Button EXTI Line to Button GPIO Pin */
        GPIO_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);

        /* Configure Button EXTI line */
        EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;

        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);

        /* Enable and set Button EXTI Interrupt to the lowest priority */
        NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

        NVIC_Init(&NVIC_InitStructure);
    }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_WAKEUP: Wakeup Push Button
  *     @arg BUTTON_TAMPER: Tamper Push Button
  *     @arg BUTTON_KEY: Key Push Button
  *     @arg BUTTON_RIGHT: Joystick Right Push Button
  *     @arg BUTTON_LEFT: Joystick Left Push Button
  *     @arg BUTTON_UP: Joystick Up Push Button
  *     @arg BUTTON_DOWN: Joystick Down Push Button
  *     @arg BUTTON_SEL: Joystick Sel Push Button
  * @retval The Button GPIO pin value.
  */
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button)
{
    return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:
  *     @arg COM1
  *     @arg COM2
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef *USART_InitStruct)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM] | RCC_APB2Periph_AFIO, ENABLE);

    /* Enable UART clock */
    if (COM == COM1)
    {
        RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
    }

    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
    GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

    /* USART configuration */
    USART_Init(COM_USART[COM], USART_InitStruct);

    /* Enable USART */
    USART_Cmd(COM_USART[COM], ENABLE);
}

/**
 * @brief  Configures COM port.
 * @param  COM: Specifies the COM port to be configured.
 *   This parameter can be one of following parameters:
 *     @arg COM1
 *     @arg COM2
 * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
 *   contains the configuration information for the specified USART peripheral.
 * @retval None
 */
void uart3_int_handler(uint32_t com, uint32_t arg)
{
    uint16_t data;

    //STM_EVAL_LEDToggle(LED4);

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        /* Read one byte from the receive data register */
        data = USART_ReceiveData(USART3);
    }
}


/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
