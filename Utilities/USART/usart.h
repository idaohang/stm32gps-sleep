#ifndef USART_H
#define USART_H

#include "stm32f10x.h"

//#define USART_GSM_BUFSIZE    1024
//#define USART_GPS_BUFSIZE    1024

#define     USART_SUCESS              0   // 成功
#define     USART_DATAERROR           1   // 数据错误
#define     USART_FAIL                1   //失败
#define     USART_NEEDWAIT            2   // 需要等待
#define     USART_ENPROCESS           3   // 可以处理


typedef struct
{
    unsigned char   *RStartpt;      // 接收缓冲起始地址
    uint32_t        uiRMaxLen;      // 接收缓冲最大长度
    unsigned char   *RBuffWpt;      // 接收缓冲指针
    unsigned char   *RBuffRpt;      // 接收处理指针
    uint32_t        ucRLen;         // 接收数据长度
    uint32_t        ucRTime;        // 接收超时时间
    uint32_t        ucRTick;
} USART_ST;

uint16_t USART_Send(USART_TypeDef *USARTx, uint8_t *Data, uint16_t nBytes);
void usart_gsm_init(void);
void usart_gsm_sendbuffer(char *byData, uint32_t *pReqLen);
uint8_t usart_gsm_readbuffer(char *byData, uint32_t *pReqLen);
void usart_gsm_irq(uint8_t data);
void usart_gsm_timeout(USART_ST *pusart);

#endif /* USART_H */

