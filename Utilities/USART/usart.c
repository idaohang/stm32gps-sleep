#include "usart.h"
#include "stm32gps_config.h"
#include "stm32f10x_it_api.h"
#include "string.h"
#include "stdio.h"

USART_ST st_SerialGsm;

unsigned char USART2_RBuffer[USART_GSM_BUFSIZE] = { 0 };

/**
  * @brief  USART Send Data
  * @param  *USARTx, Data, nBytes
  * @retval Send Number
  */
uint16_t USART_Send(USART_TypeDef *USARTx, uint8_t *Data, uint16_t nBytes)
{
    uint16_t i;
    for (i = 0; i < nBytes; i++)
    {
        USART_SendData(USARTx, *(Data + i));
        while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET )
        {
            ;
        }
    }
    return i;
}

/**
  * @brief  Init GSM Usart
  * @param  None
  * @retval None
  */
void usart_gsm_init(void)
{
  
    st_SerialGsm.RStartpt = (unsigned char *) USART2_RBuffer;   // 上行接收缓冲起始地址
    st_SerialGsm.uiRMaxLen = USART_GSM_BUFSIZE;                 // 上行接收缓冲长度
    st_SerialGsm.ucRTick = (sysTickPerSec() * GSM_USART_TIMEOUT_MS) / 1000 + 1;

    st_SerialGsm.RBuffWpt = st_SerialGsm.RStartpt;    // 上行接收缓冲指针，+2留2字节保存包程度
    st_SerialGsm.RBuffRpt = st_SerialGsm.RStartpt;    // 上行接收处理指针
    st_SerialGsm.ucRTime = 0;

    if (st_SerialGsm.ucRTick <= 1)
    {
        st_SerialGsm.ucRTick = 2;
    }

    //DEBUG("idx %d ucRTick %d", idx, st_SerialGsm[idx].ucRTick);
}

/**
  * @brief  Gsm Send Buffer
  * @param  *byData, *pReqLen
  * @retval None
  */
void usart_gsm_sendbuffer(char *byData, uint32_t *pReqLen)
{
    USART_TypeDef *usart_hw_base = USART2;
    uint32_t reqLen = *pReqLen;

    while (reqLen > 0)
    {
        //DEBUG("%d-%c\n", *pReqLen - reqLen, byData[*pReqLen - reqLen]);
        USART_SendData(usart_hw_base, byData[*pReqLen - reqLen]);

        while (USART_GetFlagStatus(usart_hw_base, USART_FLAG_TXE) == RESET)
        {
            ;
        }

        USART_ClearFlag(usart_hw_base, USART_FLAG_TXE);

        reqLen--;
    }
}

#undef DBG_usart_readbuffer

/**
  * @brief  Gsm Read Buffer
  * @param  *byData, *pReqLen
  * @retval Read Len
  */
uint8_t usart_gsm_readbuffer(char *byData, unsigned int *pReqLen)
{
    unsigned char dFlag = USART_DATAERROR;
    uint32_t curLen = 0;
    uint32_t ReqLen = *pReqLen;

    curLen = st_SerialGsm.ucRLen;

    if (curLen != 0)                   // 缓存区有数据
    {
        if (st_SerialGsm.ucRTime != 0)     // 正在接收数据
        {
            dFlag = USART_NEEDWAIT;
            *pReqLen = 0;
        }
        else
        {
            dFlag = USART_ENPROCESS;
            curLen = st_SerialGsm.ucRLen;
            if (curLen > ReqLen)
            {
                // 接收数据量大于请求数据量，按照请求数据量接收
                curLen = ReqLen;
            }
            else
            {
                ;
            }

#ifdef DBG_usart_readbuffer
            DEBUG("curLen %d ", curLen);
            DEBUG("reqLen %d\n", ReqLen);
#endif

            if (st_SerialGsm.RBuffWpt > st_SerialGsm.RBuffRpt)
            {
                // 读指针在写指针的前面
                //DEBUG("rd before wr\n");

                memcpy(byData, st_SerialGsm.RBuffRpt, curLen);
                st_SerialGsm.RBuffRpt += curLen;
                st_SerialGsm.ucRLen -= curLen;
                *pReqLen = curLen;
            }
            else
            {
                // 读指针在写指针的后面
                unsigned int backLen = (st_SerialGsm.RStartpt
                                        + st_SerialGsm.uiRMaxLen
                                        - st_SerialGsm.RBuffRpt); // 缓冲区后半部数据长度
                unsigned int saveLen = curLen;

                if (saveLen <= backLen) // 只需读后半部就可以完成
                {
                    //DEBUG("only end half\n");
                    memcpy(byData, st_SerialGsm.RBuffRpt, saveLen);
                    st_SerialGsm.RBuffRpt += saveLen;

                    // 环形缓冲区到尾部回退
                    if (st_SerialGsm.RBuffRpt >= st_SerialGsm.RStartpt + st_SerialGsm.uiRMaxLen)
                    {
                        st_SerialGsm.RBuffRpt = st_SerialGsm.RStartpt;
                    }

                    st_SerialGsm.ucRLen -= saveLen;
                    *pReqLen = curLen;
                }
                else // 要取两部分数据
                {

                    //DEBUG("both front and end half\n");
                    // 先取后半部分数据
                    memcpy(byData, st_SerialGsm.RBuffRpt, backLen);
                    st_SerialGsm.RBuffRpt = st_SerialGsm.RStartpt;  // 指到前部
                    st_SerialGsm.ucRLen -= backLen;

                    // 取前半部分数据
                    saveLen -= backLen;                     // 前部要取的数据长度
                    memcpy(byData + backLen, st_SerialGsm.RBuffRpt, saveLen);
                    st_SerialGsm.RBuffRpt += saveLen;
                    st_SerialGsm.ucRLen -= saveLen;
                    *pReqLen = saveLen + backLen;
                }
            }
        }
#ifdef DBG_usart_readbuffer
        DEBUG("usart_gsm_readbuffer,st_reqLen %d, reqLen %d\r\n",
              st_SerialGsm.ucRLen, *pReqLen);
#endif
    }
    else
    {
        //没有数据接收
        ReqLen = 0;
        *pReqLen = 0;
        dFlag = USART_FAIL;
    }

    return dFlag;
}

/**
  * @brief  Gsm Usart IRQ
  * @param  data
  * @retval None
  */
void usart_gsm_irq(uint8_t data)
{
    if ((st_SerialGsm.RBuffRpt == (st_SerialGsm.RBuffWpt + 1)) ||  // 写指针即将覆盖读指针
            ((st_SerialGsm.RBuffRpt == st_SerialGsm.RStartpt) && // 读指针指向缓存第一个位置,写指针指向缓存最后一个位置
             (st_SerialGsm.RBuffWpt == (st_SerialGsm.RStartpt + st_SerialGsm.uiRMaxLen - 1)))) // 接收缓存满
    {
        return;
    }

    *(st_SerialGsm.RBuffWpt++) = data;
    st_SerialGsm.ucRLen++;

    // 接收指针到最缓存底部
    if (st_SerialGsm.RBuffWpt >= (st_SerialGsm.RStartpt + st_SerialGsm.uiRMaxLen))
    {
        st_SerialGsm.RBuffWpt = st_SerialGsm.RStartpt;
    }

    st_SerialGsm.ucRTime = st_SerialGsm.ucRTick; // 20ms接收超时

}


/**
  * @brief  Gsm Usart timeout
  * @param  *pusart
  * @retval None
  */
void usart_gsm_timeout(USART_ST *pusart)
{

    if (st_SerialGsm.ucRTime != 0)
    {
        st_SerialGsm.ucRTime--;

#if 0
        if (st_SerialGsm.ucRTime == 0)
        {
            STM_EVAL_LEDToggle(LED3);
        }
#endif
    }
}


