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
  
    st_SerialGsm.RStartpt = (unsigned char *) USART2_RBuffer;   // ���н��ջ�����ʼ��ַ
    st_SerialGsm.uiRMaxLen = USART_GSM_BUFSIZE;                 // ���н��ջ��峤��
    st_SerialGsm.ucRTick = (sysTickPerSec() * GSM_USART_TIMEOUT_MS) / 1000 + 1;

    st_SerialGsm.RBuffWpt = st_SerialGsm.RStartpt;    // ���н��ջ���ָ�룬+2��2�ֽڱ�����̶�
    st_SerialGsm.RBuffRpt = st_SerialGsm.RStartpt;    // ���н��մ���ָ��
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

    if (curLen != 0)                   // ������������
    {
        if (st_SerialGsm.ucRTime != 0)     // ���ڽ�������
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
                // ��������������������������������������������
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
                // ��ָ����дָ���ǰ��
                //DEBUG("rd before wr\n");

                memcpy(byData, st_SerialGsm.RBuffRpt, curLen);
                st_SerialGsm.RBuffRpt += curLen;
                st_SerialGsm.ucRLen -= curLen;
                *pReqLen = curLen;
            }
            else
            {
                // ��ָ����дָ��ĺ���
                unsigned int backLen = (st_SerialGsm.RStartpt
                                        + st_SerialGsm.uiRMaxLen
                                        - st_SerialGsm.RBuffRpt); // ��������벿���ݳ���
                unsigned int saveLen = curLen;

                if (saveLen <= backLen) // ֻ�����벿�Ϳ������
                {
                    //DEBUG("only end half\n");
                    memcpy(byData, st_SerialGsm.RBuffRpt, saveLen);
                    st_SerialGsm.RBuffRpt += saveLen;

                    // ���λ�������β������
                    if (st_SerialGsm.RBuffRpt >= st_SerialGsm.RStartpt + st_SerialGsm.uiRMaxLen)
                    {
                        st_SerialGsm.RBuffRpt = st_SerialGsm.RStartpt;
                    }

                    st_SerialGsm.ucRLen -= saveLen;
                    *pReqLen = curLen;
                }
                else // Ҫȡ����������
                {

                    //DEBUG("both front and end half\n");
                    // ��ȡ��벿������
                    memcpy(byData, st_SerialGsm.RBuffRpt, backLen);
                    st_SerialGsm.RBuffRpt = st_SerialGsm.RStartpt;  // ָ��ǰ��
                    st_SerialGsm.ucRLen -= backLen;

                    // ȡǰ�벿������
                    saveLen -= backLen;                     // ǰ��Ҫȡ�����ݳ���
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
        //û�����ݽ���
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
    if ((st_SerialGsm.RBuffRpt == (st_SerialGsm.RBuffWpt + 1)) ||  // дָ�뼴�����Ƕ�ָ��
            ((st_SerialGsm.RBuffRpt == st_SerialGsm.RStartpt) && // ��ָ��ָ�򻺴��һ��λ��,дָ��ָ�򻺴����һ��λ��
             (st_SerialGsm.RBuffWpt == (st_SerialGsm.RStartpt + st_SerialGsm.uiRMaxLen - 1)))) // ���ջ�����
    {
        return;
    }

    *(st_SerialGsm.RBuffWpt++) = data;
    st_SerialGsm.ucRLen++;

    // ����ָ�뵽���ײ�
    if (st_SerialGsm.RBuffWpt >= (st_SerialGsm.RStartpt + st_SerialGsm.uiRMaxLen))
    {
        st_SerialGsm.RBuffWpt = st_SerialGsm.RStartpt;
    }

    st_SerialGsm.ucRTime = st_SerialGsm.ucRTick; // 20ms���ճ�ʱ

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


