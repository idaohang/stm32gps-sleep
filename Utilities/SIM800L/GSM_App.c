/****************************************Copyright (c)****************************************************
 **                                  		 广州星海科技
 **                                  http://simhi.taobao.com
 **																		 TEL:15018750559
 **																			 QQ:86817447
 **------------------------File Info----------------------------------------------------------------------
 ** File Name:             GSM_App.c
 ** Last modified Date:   	2012.5.15
 ** Last Version:          V1.0
 ** Description:           GPS管理解析
 **
 **--------------------------------------------------------------------------------------------------------
 ** Created By:            Chenht
 ** Created date:          2012-5-3
 ** Version:               v1.0
 ** Descriptions:          The original version 初始版本
 **
 **--------------------------------------------------------------------------------------------------------
 ** Modified by:
 ** Modified date:
 ** Version:
 ** Description:
 **
 *********************************************************************************************************/
#include "at_sim800.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "usart.h"
#include "GSM_App.h"



static char BackBuf[USART_GSM_BUFSIZE];
static char sendBuf[USART_GSM_BUFSIZE_SEND];
static char receiveBuf[USART_GSM_BUFSIZE_RECEIVE];


/******************************************************************************
 ** Function name:       strnchr()
 ** Descriptions:        查找字符串中第n个字符的位置
 ** input parameters:    S 需要查找的目标字符串
 **                      C 需要查找的字符
 **                      n 第几个位置的C
 ** output parameters:   需要查找的字符位置指针
 ** Returned value:
 *******************************************************************************/
char *strnchr(char *S, int C, int n)
{
    char *pnchr = NULL;
    char *pStr = S;

    if (n > strlen(S))
    {
        return NULL;
    }
    while (n--)
    {
        pnchr = strchr((const char *) pStr, C);
        if (NULL == pnchr)
        {
            break;
        }
        else
        {
            pStr = pnchr + 1;
        }
    }
    return pnchr;
}

/*********************************************************************************
 ** Function name:       strstr_len()
 ** Descriptions:        查找字符串中子字符串的位置，返回子字符串的位置
 ** input parameters:    str 需要查找的目标字符串
 **                      subStr 需要查找的子字符串
 **                      strlenth 目标字符串的长度
 ** output parameters:
 ** Returned value:      子字符串的位置指针
 ** Example:  strstr_len("this,are,my,pointer", "my", 19) 返回的指针指向“m“的位置
 ***********************************************************************************/
char *strstr_len(char *str, char *subStr, uint32_t strlenth)
{
    uint32_t subStrLen = strlen(subStr);
    int32_t cmpCnt = strlenth - strlen(subStr) + 1;

    char *retPtr = NULL;
    int32_t i;

    if (cmpCnt > 0)
    {
        for (i = 0; i < cmpCnt; i ++)
        {
            if (memcmp(str + i, subStr, subStrLen) == 0)
            {
                break;
            }
        }

        if (i < cmpCnt)
        {
            retPtr = str + i;
        }
    }

    return retPtr;
}

/**************************************************************************************
 ** Function name:       strnchr_len()
 ** Descriptions:        查找字符串中第几个字符值的位置，返回C的位置
 ** input parameters:    S 需要查找的目标字符串
 **                      C 需要查找的字符值
 **                      n 查找几个字符
 **                      len 目标字符串的长度
 ** output parameters:
 ** Returned value:      C的位置指针
 ** Example: strnchr_len("this,are,my,pointer", ',', 2, 19); 返回指针指向are后面的','的位置
 *******************************************************************************************/
char *strnchr_len(char *S, int C, uint32_t n, uint32_t len)
{
    char *pnchr = NULL;
    char *pStr = S;
    uint32_t i;

    if (n > len)
    {
        return NULL;
    }
    while (n--)
    {
        for (i = 0; i < len; i ++)
        {
            if (*(pStr + i) == C)
            {
                break;
            }
        }

        if (i < len)
        {
            pnchr = pStr + i;
            len -= i + 1;
        }
        else
        {
            pnchr = NULL;
        }

        if (NULL == pnchr)
        {
            break;
        }
        else
        {
            pStr = pnchr + 1;
        }
    }
    return pnchr;
}

/***************************************************************************************
 ** Function name:       strdig_len()
 ** Descriptions:        查找字符串中是否有n个数字，如果有则返回第一个数字位置的指针，
 **						 如果没有则返回NULL。
 **                      字符串的长度必须大于需要查找的数字个数。
 ** input parameters:    str 需要查找的目标字符串
 **                      strlen 需要查找的字符串的长度
 **                      diglen 需要查找的数字的个数
 ** output parameters:
 ** Returned value:      第一个数字的字符位置指针 or NULL
 **************************************************************************************/
char *strdig_len(char *str, uint32_t strlen, uint32_t diglen)
{

    char *retPtr = NULL;
    int32_t i;
    int32_t sumlen = 0;


    if (strlen > 0 && strlen > diglen)
    {
        for (i = 0; i < strlen; i ++)
        {
            if((*(str + i) >= '0') && (*(str + i) <= '9'))
            {
                sumlen++;
            }
            else
            {
                sumlen = 0;
            }
            if (sumlen >= diglen)
            {
                break;
            }
        }

        if (sumlen >= diglen)
        {
            retPtr = str + i - diglen + 1;
        }
    }

    return retPtr;
}

/**************************************************************************************
 ** Function name:       strhex_len()
 ** Descriptions:        将十六进制字符串转换为十六进制数
 ** input parameters:    str 需要查找的目标字符串
 **                      len 目标字符串的长度
 ** output parameters:
 ** Returned value:      转换后的十六进制数
 ** Example: 比如，字符串是"04"，转换后的值是0x04， 字符串是"2F"，转换后的值是0x2F
 ****************************************************************************************/
uint32_t strhex_len(char *str, uint32_t len)
{
    uint32_t i;
    uint32_t sum = 0;
    uint32_t tmp = 0;
    for(i = 0; i < len; i++)
    {

        if (str[i] >= 'A' && str[i] <= 'F')
        {
            tmp = str[i] - 55; //a-f之间的ascii与对应数值相差55如'A'为65,65-55即为A
        }
        else if(str[i] >= 'a' && str[i] <= 'f')
        {
            tmp = str[i] - 87;
        }
        else if(str[i] >= '0' && str[i] <= '9')
        {
            tmp = str[i] - 48;
        }
        sum <<= 4;
        sum |= tmp;
    }

    return sum;
}


/**
  * @brief  Turn on GSM's VCC power.
  * Output LOW to Turn Off VCC Power; HIGH to Turn On VCC Power.
  * @param  None
  * @retval None
  */
void GSM_PowerOn(void)
{
    GPIO_SetBits(GSM_PWR_CTRL_PORT, GSM_PWR_CTRL_PIN);

}

/**
  * @brief  Turn off GSM's VCC power.
  * Output LOW to Turn Off VCC Power; HIGH to Turn On VCC Power.
  * @param  None
  * @retval None
  */
void GSM_PowerOff(void)
{
    GPIO_ResetBits(GSM_PWR_CTRL_PORT, GSM_PWR_CTRL_PIN);
}

/*********************************************************************************************************
 ** Function name:       GSM_TurnOnOff()
 ** Descriptions:        启动或关闭模块
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:
 *********************************************************************************************************/
void GSM_TurnOnOff(void)
{
    GPIO_ResetBits(GSM_PWRKEY_PORT, GSM_PWRKEY_PIN);
    delay_10ms(300);
    GPIO_SetBits(GSM_PWRKEY_PORT, GSM_PWRKEY_PIN);
    delay_10ms(300);
}


/*********************************************************************************************************
 ** Function name:       GSM_ClearBuffer()
 ** Descriptions:        清空接收缓存
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:
 *********************************************************************************************************/
void GSM_ClearBuffer(void)
{
    memset(BackBuf, 0, USART_GSM_BUFSIZE);
}

void GSM_ClearSendBuffer(void)
{
    memset(sendBuf, 0, USART_GSM_BUFSIZE_SEND);
}

/**
  * @brief  发送一条AT指令
  * Output LOW to Turn Off VCC Power; HIGH to Turn On VCC Power.
  * @param  pCMD：指令指针
  *			pCMDBack： 指令返回参考指针
  *			CMDLen:  指令长度
  *			Count: 查询次数，每次500ms
  * @retval 发送指令返回状态
  */
unsigned char GSM_SendAT(char *pCMD, char *pCMDBack, uint32_t CMDLen, uint8_t Count)
{
    uint8_t i = Count;  // 查询次数
    uint32_t len;
    char *pBackBuf = BackBuf;
    unsigned char retFlag = 0;

    len = CMDLen;

#ifdef DBG_ENABLE_MACRO
    {
        uint32_t tmpIdx;

        if (len > 0)
        {
            DEBUG("GSM_SendAT send\r\n");
            for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
            {
                //DEBUG("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                DEBUG("%c", pCMD[tmpIdx]);
            }
            DEBUG("\r\n");
            DEBUG("GSM_SendAT done\r\n");
        }
    }
#endif

    usart_sendbuffer(STM32_SIM908_GSM_COM, pCMD, (unsigned int *) &len);

    if (NULL != pCMDBack)
    {
        while (i > 0)
        {
            i--;
            len = USART_GSM_BUFSIZE;
            delay_10ms(50);

            //DEBUG("GSM_SendAT before usart_readbuffer len %d\n", len);

            retFlag = usart_readbuffer(STM32_SIM908_GSM_COM, pBackBuf, &len);
            //DEBUG("GSM_SendAT after usart_readbuffer len %d\n", len);

#ifdef DBG_ENABLE_MACRO
            {
                uint32_t tmpIdx;

                if (len > 0 && retFlag == USART_ENPROCESS)
                {
                    DEBUG("GSM_SendAT recv %d bytes\r\n", len);
                    for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
                    {
                        //DEBUG("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                        DEBUG("%c", pBackBuf[tmpIdx]);
                    }
                    DEBUG("\r\n");

                    DEBUG("GSM_SendAT recv done\r\n");
                }
            }
#endif
            if ((len > 0) && (retFlag == USART_ENPROCESS))
            {
                if (NULL != strstr_len(pBackBuf, pCMDBack, len))
                {
                    break;
                }
            }
        }

        // 没有得到返回命令
        if (i == 0)
        {
            return USART_FAIL;
        }
    }

    return USART_SUCESS;
}


unsigned char GSM_SendAT_rsp(char *pCMD, char *pCMDBack,
                             uint32_t CMDLen, char **ppRecvBuf, uint32_t *pRecvLen, uint8_t Count)
{
    uint8_t i = Count;
    unsigned int len;
    char *pBackBuf = BackBuf;
    unsigned char retFlag = 0;

    len = CMDLen;

#ifdef DBG_ENABLE_MACRO
    {
        uint32_t tmpIdx;

        if (len > 0)
        {
            DEBUG("GSM_SendAT send\r\n");
            for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
            {
                //DEBUG("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                DEBUG("%c", pCMD[tmpIdx]);
            }
            DEBUG("\r\n");
            DEBUG("GSM_SendAT done\r\n");
        }
    }
#endif

    usart_sendbuffer(STM32_SIM908_GSM_COM, pCMD, (unsigned int *) &len);

    while (i > 0)
    {
        i--;
        len = USART_GSM_BUFSIZE;
        delay_10ms(50);

        //DEBUG("GSM_SendAT before usart_readbuffer len %d\n", len);
        retFlag = usart_readbuffer(STM32_SIM908_GSM_COM, pBackBuf, &len);
        //DEBUG("GSM_SendAT after usart_readbuffer len %d\n", len);

#ifdef DBG_ENABLE_MACRO
        {
            uint32_t tmpIdx;

            if (len > 0 && retFlag == USART_ENPROCESS)
            {
                DEBUG("GSM_SendAT recv\r\n");
                for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
                {
                    //DEBUG("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                    DEBUG("%c", pBackBuf[tmpIdx]);
                }
                DEBUG("\r\n");
                DEBUG("GSM_SendAT done\r\n");
            }
        }
#endif

        if (len > 0 && retFlag == USART_ENPROCESS)
        {
            if (ppRecvBuf != NULL)
            {
                *ppRecvBuf = pBackBuf;
            }

            if (pRecvLen != NULL)
            {
                *pRecvLen = len;
            }

            if (pCMDBack != NULL)
            {
                if (NULL != strstr((const char *) pBackBuf, (const char *) pCMDBack))
                {
                    break;
                }
            }
            else
            {
                break;
            }
        }


        if (i == 0)
        {
            return USART_FAIL;
        }
    }

    return USART_SUCESS;
}


/*********************************************************************************************************
 ** Function name:       GSM_QueryImsiBuf
 ** Descriptions:        查询IMSI
 ** input parameters:    NONE
 ** output parameters:   pImsiInfo
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GSM_QueryImsiBuf(uint8_t *pImsi)
{
    unsigned int cmdLen;
    static char *pfeed = NULL;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    cmdLen = strlen(AT_CIMI);
    GSM_ClearBuffer();
    if (USART_SUCESS == GSM_SendAT_rsp((char *)AT_CIMI, (char *)AT_OK, cmdLen, &pRecvBuf, &recvLen, MAX_RESP_CMD_CIMI))
    {

        pfeed = strdig_len(pRecvBuf, recvLen, 15);
        if (pfeed == NULL)
        {
            return USART_FAIL;
        }

        if (pImsi != NULL)
        {
            memcpy(pImsi, pfeed, 15);
        }

        return USART_SUCESS;
    }
    return USART_FAIL;
}



/**********************************************************************
 ** Function name:       GSM_QueryCREG
 ** Descriptions:        Query Location information elements
 ** input parameters:    NONE
 ** output parameters:   pCregInfo
 ** Returned value:      返回状态结果
 ***********************************************************************/
unsigned char GSM_QueryCreg(pST_CREGINFO pCregInfo)
{
    uint32_t cmdLen = 0;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;
    char *pStr = NULL;
    char tmpbuf[2];

    cmdLen = strlen(AT_CREG);
    GSM_ClearBuffer();
    if(USART_SUCESS == GSM_SendAT_rsp((char *)AT_CREG, (char *) "CREG",
                                      cmdLen, &pRecvBuf, &recvLen, MAX_RESP_CMD_CREG))
    {

        // lac
        pStr = strnchr_len(pRecvBuf, '"', 1, recvLen);
        if(pStr == NULL)
        {
            return USART_FAIL;
        }
        if (pStr != NULL)
        {
            tmpbuf[0] = *(pStr + 1);
            tmpbuf[1] = *(pStr + 2);
            pCregInfo->Lac[0] = strhex_len(tmpbuf, strlen(tmpbuf));
            tmpbuf[0] = *(pStr + 3);
            tmpbuf[1] = *(pStr + 4);
            pCregInfo->Lac[1] = strhex_len(tmpbuf, strlen(tmpbuf));
        }

        // ci
        pStr = strnchr_len(pRecvBuf, '"', 3, recvLen);
        if(pStr == NULL)
        {
            return USART_FAIL;
        }
        if (pStr != NULL)
        {
            tmpbuf[0] = *(pStr + 1);
            tmpbuf[1] = *(pStr + 2);
            pCregInfo->Ci[0] = strhex_len(tmpbuf, strlen(tmpbuf));
            tmpbuf[0] = *(pStr + 3);
            tmpbuf[1] = *(pStr + 4);
            pCregInfo->Ci[1] = strhex_len(tmpbuf, strlen(tmpbuf));
        }

        // stat
        pStr = strnchr_len(pRecvBuf, ',', 2, recvLen);
        if(pStr == NULL)
        {
            return USART_FAIL;
        }
        if (pStr != NULL)
        {
            pCregInfo->Stat = *(pStr - 1);
        }

        // n
        pStr = strnchr_len(pRecvBuf, ',', 1, recvLen);
        if(pStr == NULL)
        {
            return USART_FAIL;
        }
        if (pStr != NULL)
        {
            pCregInfo->n = *(pStr - 1);
        }

        return USART_SUCESS;
    }

    return USART_FAIL;
}


/**
  * @brief  Get TCPIP Application Mode
  * 0 Normal mode; 1 Transparent mode
  * @param  None
  * @retval None
  */
unsigned char GSM_QueryCIPMode(unsigned char *pMode)
{
    unsigned int cmdLen;
    static char *pfeed = NULL;

    cmdLen = strlen(AT_CIPMODE_GET);
    GSM_ClearBuffer();
    if (USART_SUCESS == GSM_SendAT((char *) AT_CIPMODE_GET, (char *) AT_OK, cmdLen, MAX_RESP_CMD_DEFAULT))
    {
        pfeed = strnchr(BackBuf, ':', 1);
        if (pfeed == NULL)
            return USART_FAIL;
        pfeed++;
        *pMode = atoi(pfeed);
        return USART_SUCESS;
    }
    return USART_FAIL;
}

/**
  * @brief  Select TCPIP Application Mode
  * 0 Normal mode; 1 Transparent mode
  * @param  None
  * @retval None
  */
void GSM_SetCIPMode(unsigned char mode)
{
    static char *pcmdbuf = NULL;
    unsigned int cmdLen = 0;
    unsigned char preMode = 0;
    uint32_t errNum = 0;

    GSM_ClearSendBuffer();
    // get cpimode
    GSM_QueryCIPMode(&preMode);
    // Select TCPIP application mode is normal mode
    if(preMode != mode)
    {
        pcmdbuf = sendBuf;
        sprintf(pcmdbuf, AT_CIPMODE_SET, mode);
        cmdLen = strlen(pcmdbuf);
        while(USART_SUCESS != GSM_SendAT((char *) pcmdbuf, (char *) AT_OK, cmdLen, MAX_RESP_CMD_DEFAULT))
        {
            if(errNum > 5)
            {
                break;
            }
        }
    }
}


/**
  * @brief  Init GSM Module and Set ATE = 0
  * @param  None
  * @retval None
  */
uint8_t GSM_Init(void)
{
    uint8_t i = 0;
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;

    // AT握手
    len = strlen(AT_Cmd);
    GSM_ClearBuffer();
    while(errNum < RETRY_TIMES_CMD_AT)
    {
        errNum++;
        rst = GSM_SendAT((char *) AT_Cmd, (char *) AT_OK, len, MAX_RESP_CMD_AT);
        if(USART_SUCESS == rst)
        {
            rtn = RST_OK;
            break;
        }
        i++;
        //if(i > 2)
        {
            GSM_TurnOnOff();
            i = 0;
        }

        delay_10ms(30);
    }
    errNum = 0;
    rst = USART_FAIL;

    //关闭回显
    len = strlen(ATE0_Cmd);
    GSM_ClearBuffer();
    while(errNum < RETRY_TIMES_CMD_ATE0)
    {
        errNum++;
        rst = GSM_SendAT((char *) ATE0_Cmd, (char *) AT_OK, len, MAX_RESP_CMD_ATE0);
        if(USART_SUCESS == rst)
        {
            break;
        }

        delay_10ms(30);
    }

    return rtn;
}

/**
  * @brief  Check SIM Card (AT+CPIN?)
  * @param  None
  * @retval RST_OK - ; RST_FAIL -
  */
uint8_t GSM_CheckSIMCard(void)
{
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;

    // 查询卡状态
    len = strlen(AT_CPIN);
    GSM_ClearBuffer();
    while(errNum < RETRY_TIMES_CMD_CPIN)
    {
        errNum++;
        rst = GSM_SendAT((char *) AT_CPIN, (char *) AT_READY, len, MAX_RESP_CMD_CPIN);
        if(USART_SUCESS == rst)
        {
            rtn = RST_OK;
            break;
        }

        delay_10ms(20);
    }

    return rtn;
}

/**
  * @brief  查询信号强度
  * @param  pSig: 信号强度
  * @retval RST_OK - ; RST_FAIL -
  */
uint8_t GSM_QuerySignal(uint8_t *pSig)
{
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;
    static char *pfeed = NULL;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    len = strlen(AT_CSQ);
    GSM_ClearBuffer();
    while(errNum < RETRY_TIMES_CMD_CSQ)
    {
        errNum++;
        rst = GSM_SendAT_rsp((char *) AT_CSQ, (char *) AT_OK, len, &pRecvBuf, &recvLen, MAX_RESP_CMD_CSQ);
        if(USART_SUCESS == rst)
        {
            pfeed = strnchr(pRecvBuf, ',', 1);
            if (pfeed == NULL)
            {
                rtn = RST_FAIL;
                continue;
            }
            pfeed -= 2;
            if (pfeed[0] < 0x30 || pfeed[0] > 0x39)
            {
                *pSig = pfeed[1] - 0x30;
            }
            else
            {
                *pSig = (pfeed[0] - 0x30) * 10 + (pfeed[1] - 0x30);
            }

            // 0,1 - Less Signal; 2~31 - Normal Signal; 99 - No Signal
            if((*pSig >= 2) && (*pSig <= 31))
            {
                rtn = RST_OK;
                break;
            }
        }

        delay_10ms(20);
    }

    return rtn;
}

/**
  * @brief  Read GSM's ADC
  * @param  pSig
  * @retval Status
  */
uint8_t GSM_QueryBatVoltage(pST_BATVOLTAGESTATUS pSig)
{
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;
    static char *pfeed = NULL;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    len = strlen(AT_CADC);
    GSM_ClearBuffer();
    while(errNum < RETRY_TIMES_CMD_CADC)
    {
        errNum++;
        rst = GSM_SendAT_rsp((char *) AT_CADC, (char *) AT_OK, len, &pRecvBuf, &recvLen, MAX_RESP_CMD_CADC);
        if(USART_SUCESS == rst)
        {
            pfeed = strnchr(pRecvBuf, ',', 1);	//提取状态
            if(NULL != pfeed)
            {
                pSig->Status = (*(pfeed - 1) - '0');

                // 分压电阻是100K和200K
                pfeed++;
                pSig->BatVoltage.i = ((atoi(pfeed) * 3) / 2);

                rtn = RST_OK;
                break;
            }
        }
    }

    return rtn;
}

/**
  * @brief  Set Network Registration
  * 2 Enable network registration unsolicited result code with
  * location information +CREG: <stat>[,<lac>,<ci>]
  * @param  None
  * @retval None
  */
void GSM_SetNetworkReg(uint8_t data)
{
    static char *pcmdbuf = NULL;
    uint32_t cmdLen = 0;
    uint32_t errNum = 0;

    GSM_ClearSendBuffer();
    pcmdbuf = sendBuf;
    sprintf(pcmdbuf, AT_CREG_SET, data);
    cmdLen = strlen(pcmdbuf);
    while (USART_SUCESS != GSM_SendAT((char *) pcmdbuf, (char *) AT_OK, cmdLen, MAX_RESP_CMD_CREG))
    {
        errNum++;
        if(errNum > 5)
        {
            break;
        }
    }
}

/**
  * @brief  Check Network Registration
  * @param  None
  * @retval None
  */
uint8_t GSM_CheckNetworkReg(void)
{
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;
    static char *pfeed = NULL;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    len = strlen(AT_CREG);
    GSM_ClearBuffer();
    while(errNum < RETRY_TIMES_CMD_CREG)
    {
        errNum++;
        rst = GSM_SendAT_rsp((char *) AT_CREG, (char *) "CREG", len, &pRecvBuf, &recvLen, MAX_RESP_CMD_CREG);
        if(USART_SUCESS == rst)
        {
            pfeed = strnchr_len(pRecvBuf, ',', 1, recvLen);
            if (pfeed != NULL)
            {
                // 1 Registered, home network; 5 - roaming
                if ((*(pfeed + 1) == '1') || (*(pfeed + 1) == '5'))
                {
                    rtn = RST_OK;
                    break;
                }
            }
        }

        delay_10ms(20);
    }

    return rtn;
}

/**
  * @brief  Request TA Serial Number Identification (IMEI)
  * @param  pImei: pointer of IMEI
  * @retval STATUS
  */
uint8_t GSM_QueryImei(uint8_t *pImei)
{
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;
    static char *pfeed = NULL;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    len = strlen(AT_GSN);
    GSM_ClearBuffer();
    while(errNum < RETRY_TIMES_CMD_GSN)
    {
        errNum++;
        rst = GSM_SendAT_rsp((char *)AT_GSN, (char *)AT_OK, len, &pRecvBuf, &recvLen, MAX_RESP_CMD_GSN);
        if(USART_SUCESS == rst)
        {
            pfeed = strdig_len(pRecvBuf, recvLen, 15);
            if (pfeed != NULL)
            {
                memcpy(pImei, pfeed, 15);
                rtn = RST_OK;
                break;
            }
        }

        delay_10ms(20);
    }

    return rtn;
}

/**
  * @brief  查询IMSI
  * @param  pImsiInfo: pointer of IMSI INFO
  * @retval STATUS
  */
uint8_t GSM_QueryImsi(pST_IMSIINFO pImsiInfo)
{
    uint32_t i = 0;
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;
    static char *pfeed = NULL;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;
    char tmpMcc[4];
    char tmpMnc[3];
    uint32_t tmpValue = 0;

    len = strlen(AT_CIMI);
    GSM_ClearBuffer();
    while(errNum < RETRY_TIMES_CMD_CIMI)
    {
        errNum++;
        rst = GSM_SendAT_rsp((char *)AT_CIMI, (char *)AT_OK, len, &pRecvBuf, &recvLen, MAX_RESP_CMD_CIMI);
        if(USART_SUCESS == rst)
        {
            pfeed = strdig_len(pRecvBuf, recvLen, 15);
            if(NULL != pfeed)
            {
                for(i = 0; i < 3; i++)
                {
                    tmpMcc[i] = *(pfeed + i);
                }
                tmpMcc[3] = '\0';

                for(i = 0; i < 2; i++)
                {
                    tmpMnc[i] = *(pfeed + 3 + i);
                }
                tmpMnc[2] = '\0';

                tmpValue = atoi(tmpMcc);
                pImsiInfo->Mcc[0] = (tmpValue / 255);
                if(tmpValue > 255)
                {
                    pImsiInfo->Mcc[1] = (tmpValue % 255 - 1);
                }
                else
                {
                    pImsiInfo->Mcc[1] = (tmpValue % 255);
                }

                tmpValue = atoi(tmpMnc);
                pImsiInfo->Mnc[0] = (tmpValue / 255);
                if(tmpValue > 255)
                {
                    pImsiInfo->Mnc[1] = (tmpValue % 255 - 1);
                }
                else
                {
                    pImsiInfo->Mnc[1] = (tmpValue % 255);
                }

                rtn = RST_OK;
                break;
            }
        }

        delay_10ms(20);
    }

    return rtn;
}

/**
  * @brief  Check Attach or Detach from GPRS Service Status
  * @param  None
  * @retval None
  */
uint8_t GSM_CheckGPRSService(void)
{
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;
    static char *pfeed = NULL;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    len = strlen(AT_CGATT);
    GSM_ClearBuffer();
    while (errNum < RETRY_TIMES_CMD_CGATT)
    {
        errNum++;
        rst = GSM_SendAT_rsp((char *) AT_CGATT, (char *) "CGATT",
                             len, &pRecvBuf, &recvLen, MAX_RESP_CMD_CGATT);
        if (USART_SUCESS == rst)
        {

            // analyze CGATT rsp 1 Attached; 0 Detached
            pfeed = strnchr_len(pRecvBuf, ':', 1, recvLen);
            if (pfeed != NULL)
            {
                do
                {
                    pfeed++;
                }
                while (*pfeed == ' ');

                if (*pfeed == '1')
                {
                    rtn = RST_OK;
                    break;
                }
            }
        }

        delay_10ms(20);
    }

    return rtn;
}

/**
  * @brief  Start Task and Set APN, USER NAME, PASSWORD
  * @param  None
  * @retval None
  */
uint8_t GSM_StartTaskAndSetAPN(void)
{
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;
    static char *pcmdbuf = NULL;

    GSM_ClearSendBuffer();
    pcmdbuf = sendBuf;
    sprintf(pcmdbuf, AT_CSTT_SET, "\"CMNET\"");
    len = strlen(pcmdbuf);
    GSM_ClearBuffer();
    while (errNum < RETRY_TIMES_CMD_CGATT)
    {
        errNum++;
        rst = GSM_SendAT((char *) pcmdbuf, (char *) AT_OK, len, MAX_RESP_CMD_CSTT);
        if(USART_SUCESS == rst)
        {
            rtn = RST_OK;
            break;
        }

        delay_10ms(20);
    }

    return rtn;
}



/**
  * @brief  Bring Up Wireless Connection with GPRS or CSD
  * @param  None
  * @retval None
  */
uint8_t GSM_BringUpConnect(void)
{
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;

    len = strlen(AT_CIICR);
    GSM_ClearBuffer();
    while (errNum < RETRY_TIMES_CMD_CIICR)
    {
        errNum++;
        rst = GSM_SendAT((char *) AT_CIICR, (char *) AT_OK, len, MAX_RESP_CMD_CIICR);
        if(USART_SUCESS == rst)
        {
            rtn = RST_OK;
            break;
        }

        delay_10ms(20);
    }

    return rtn;
}

/**
  * @brief  Get Local IP Address
  * @param  None
  * @retval None
  */
uint8_t GSM_GetLocalIP(void)
{
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;

    len = strlen(AT_CIFSR);
    GSM_ClearBuffer();
    while (errNum < RETRY_TIMES_CMD_CIFSR)
    {
        errNum++;
        rst = GSM_SendAT((char *) AT_CIFSR, (char *)".", len, MAX_RESP_CMD_CIFSR);
        if(USART_SUCESS == rst)
        {
            rtn = RST_OK;
            break;
        }

        delay_10ms(20);
    }

    return rtn;
}

/**
  * @brief  Start Up TCP or UDP Connection
  * @param  None
  * @retval None
  */
uint8_t GSM_StartUpConnect(void)
{
    ST_NETWORKCONFIG stNetCfg;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;

	sprintf(stNetCfg.TransferMode, "%s", "TCP");

    sprintf(stNetCfg.RemoteIP, "%s", GSM_SERVER_IP);
    sprintf(stNetCfg.RemotePort, "%s", GSM_SERVER_PORT);


    while (errNum < RETRY_TIMES_CMD_CIPSTART)
    {
        errNum++;
        rst = GPRS_LinkServer(&stNetCfg);
        if(USART_SUCESS == rst)
        {
            rtn = RST_OK;
            break;
        }

        delay_10ms(20);
    }

    return rtn;
}


/***************************************************************
 ** Function name:       GPRS_LinkServer
 ** Descriptions:      	建立一个链接
 ** input parameters:    pnetconfig: 网络配置信息
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *****************************************************************/
unsigned char GPRS_LinkServer(pST_NETWORKCONFIG pnetconfig)
{
    ST_NETWORKCONFIG netcfg = *pnetconfig;
    static char *pcmdbuf = NULL;
    uint32_t cmdLen = 0;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    GSM_ClearSendBuffer();
    pcmdbuf = sendBuf;
    if (pcmdbuf == NULL)
    {
        return USART_FAIL;
    }
    sprintf(pcmdbuf, AT_CIPSTART, netcfg.TransferMode, netcfg.RemoteIP,
            netcfg.RemotePort);
    cmdLen = strlen(pcmdbuf);
    if (USART_SUCESS == GSM_SendAT_rsp((char *) pcmdbuf, (char *) AT_CONNECTOK, cmdLen, &pRecvBuf, &recvLen, MAX_RESP_CMD_CIPSTART))
    {
        if(NULL != strstr_len(pRecvBuf, (char *)AT_CONNECTOK, recvLen))
        {
            return USART_SUCESS;
        }
        else if(NULL != strstr_len(pRecvBuf, "ALREADY CONNECT", recvLen))
        {
            return USART_SUCESS;
        }
    }

    return USART_FAIL;
}

/***************************************************************************
 ** Function name:       GPRS_SendData
 ** Descriptions:      	GPRS非透传模式下发送数据
 ** input parameters:    pString: 需要发送的数据指针
 **											len:	需要发送的数据长度
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 ***************************************************************************/
unsigned char GPRS_SendData(char *pString, unsigned int len)
{
    static char *pcmdbuf = NULL;
    char *pBackBuf = BackBuf;
    unsigned int cmdLen;

    GSM_ClearSendBuffer();
    pcmdbuf = sendBuf;
    sprintf(pcmdbuf, AT_CIPSEND_SET, len);
    cmdLen = strlen(pcmdbuf);
    //cmdLen = strlen(AT_CIPSEND);
    if (USART_SUCESS == GSM_SendAT((char *) pcmdbuf, (char *) '>', cmdLen, MAX_RESP_CMD_DEFAULT))
    {
        //cmdLen = sizeof(pString);
        usart_sendbuffer(STM32_SIM908_GSM_COM, pString, &len);
        //cmdLen = 1;
        //usart_sendbuffer(STM32_SIM908_GSM_COM, "\x1A", &cmdLen);
        //return USART_SUCESS;
    }

    {
        uint32_t i = 8;
        uint32_t len;
        unsigned char retFlag;

        while (--i)
        {
            len = USART_GSM_BUFSIZE;
            delay_10ms(20);

            //DEBUG("GSM_SendAT before usart_readbuffer len %d\n", len);

            retFlag =
                usart_readbuffer(STM32_SIM908_GSM_COM, BackBuf, &len);
            //DEBUG("GSM_SendAT after usart_readbuffer len %d\n", len);

#ifdef DBG_ENABLE_MACRO
            {
                uint32_t tmpIdx;
                uint32_t tmpStart;

                if (len > 0 && retFlag == USART_ENPROCESS)
                {
                    DEBUG("GPRS_SendData recv\r\n");
                    for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
                    {
                        //DEBUG("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                        DEBUG("%c", BackBuf[tmpIdx]);

                    }
                    DEBUG("\r\n");
                    DEBUG("GPRS_SendData done\r\n");
                }
            }
            DEBUG("%d retFlag = %d len = %d\n\n", i, retFlag, len);
#endif
            if (len > 0 && retFlag == USART_ENPROCESS)
            {
                if (NULL != strstr_len(pBackBuf, "SEND OK", len))
                {
                    break;
                }
            }

        }
        if (i == 0)
        {
            return USART_FAIL;
        }
    }
    return USART_SUCESS;
}

/**
  * @brief  Send GPRS Data
  * @param  None
  * @retval None
  */
unsigned char GPRS_SendData_rsp(char *pString, unsigned int len, char **ppRecvBuf, uint32_t *pRecvLen)
{
    static char *pcmdbuf = NULL;
    char *pBackBuf = BackBuf;
    unsigned int cmdLen;

    GSM_ClearSendBuffer();
    pcmdbuf = sendBuf;
    sprintf(pcmdbuf, AT_CIPSEND_SET, len);
    cmdLen = strlen(pcmdbuf);
    //cmdLen = strlen(AT_CIPSEND);
    if (USART_SUCESS == GSM_SendAT((char *) pcmdbuf, (char *) '>', cmdLen, MAX_RESP_CMD_DEFAULT))
    {

        //cmdLen = sizeof(pString);
        usart_sendbuffer(STM32_SIM908_GSM_COM, pString, &len);
        //cmdLen = 1;
        //usart_sendbuffer(STM32_SIM908_GSM_COM, "\x1A", &cmdLen);
        //return USART_SUCESS;
    }

    {
        uint32_t i = 10;
        uint32_t len;
        unsigned char retFlag;

        while (--i)
        {
            len = USART_GSM_BUFSIZE;
            delay_10ms(20);

            //DEBUG("GSM_SendAT before usart_readbuffer len %d\n", len);

            retFlag =
                usart_readbuffer(STM32_SIM908_GSM_COM, pBackBuf, &len);
            //DEBUG("GSM_SendAT after usart_readbuffer len %d\n", len);

#ifdef DBG_ENABLE_MACRO
            {
                uint32_t tmpIdx;
                uint32_t tmpStart;

                if (len > 0 && retFlag == USART_ENPROCESS)
                {
                    DEBUG("GPRS_SendData_rsp recv\r\n");
                    for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
                    {
                        DEBUG("%c-", pBackBuf[tmpIdx]);
                        //DEBUG(" [%d] 0x%x-",tmpIdx, pBackBuf[tmpIdx]);

                    }
                    DEBUG("\r\n");
                    DEBUG("GPRS_SendData_rsp done\r\n");
                }
            }
            //DEBUG("%d retFlag = %d len = %d\n\n", i, retFlag, len);
#endif
            if (len > 0 && retFlag == USART_ENPROCESS)
            {
                if (ppRecvBuf != NULL)
                {
                    *ppRecvBuf = pBackBuf;
                }

                if (pRecvLen != NULL)
                {
                    *pRecvLen = len;
                }

                if (NULL != strstr_len(pBackBuf, "SEND OK", len))
                {
                    break;
                }
            }

        }
        if (i == 0)
        {
            return USART_FAIL;
        }
    }
    return USART_SUCESS;
}

/*********************************************************************************************************
 ** Function name:       GPRS_ReceiveData
 ** Descriptions:      	GPRS接收数据,SIM908直接串口接收,不需要发送指令读取
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GPRS_ReceiveData(char *pString)
{
    uint32_t i = 5;
    uint32_t len;
    unsigned char retFlag;
    char *pReceiveBuf = receiveBuf;

    while (--i)
    {
        len = USART_GSM_BUFSIZE_RECEIVE;
        delay_10ms(20);

        //DEBUG("GSM_SendAT before usart_readbuffer len %d\n", len);

        retFlag =
            usart_readbuffer(STM32_SIM908_GSM_COM, receiveBuf, &len);
        //DEBUG("GSM_SendAT after usart_readbuffer len %d\n", len);

#ifdef DBG_ENABLE_MACRO
        {
            uint32_t tmpIdx;


            if (len > 0 && retFlag == USART_ENPROCESS)
            {
                DEBUG("GPRS_ReceiveData recv\r\n");
                for (tmpIdx = 0; tmpIdx < len; tmpIdx ++)
                {
                    //DEBUG("%d-%c\r\n", tmpIdx, pBackBuf[tmpIdx]);
                    DEBUG("%c", receiveBuf[tmpIdx]);

                }
                DEBUG("\r\n");
                DEBUG("GPRS_ReceiveData done\r\n");
            }
        }
        DEBUG("%d retFlag = %d len = %d\n\n", i, retFlag, len);

        if (len > 0 && retFlag == USART_ENPROCESS)
        {
            if (NULL != strstr_len(pReceiveBuf, pString, len))
            {
                DEBUG("Receive New Message.\n");
                break;
            }
        }

#endif
    }

    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GPRS_CheckLinkStatus
 ** Descriptions:      	查询GPRS链接状态
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      返回GPRS链接状态结果
 *********************************************************************************************************/
unsigned char GPRS_CheckLinkStatus(unsigned char *pStatus)
{
    unsigned int cmdLen;
    static char *pfeed = NULL;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    cmdLen = strlen(AT_CIPSTATUS);
    GSM_ClearBuffer();
    if (USART_SUCESS == GSM_SendAT_rsp((char *) AT_CIPSTATUS, (char *) AT_OK, cmdLen, &pRecvBuf, &recvLen, MAX_RESP_CMD_DEFAULT))
    {
        pfeed = strstr_len(pRecvBuf, "CONNECT OK", recvLen);
        if (NULL != strstr_len(pRecvBuf, "CONNECT OK", recvLen))
        {
            *pStatus = 6;
            return USART_SUCESS;
        }
        else if(NULL != strstr_len(pRecvBuf, "IP INITIAL", recvLen))
        {
            *pStatus = 0;
            return USART_SUCESS;
        }
        else if(NULL != strstr_len(pRecvBuf, "IP START", recvLen))
        {
            *pStatus = 1;
            return USART_SUCESS;
        }
        else
        {

            ;
        }
    }

    *pStatus = 0xFF;
    DEBUG("GPRS_QueryLinkStatus = %d\n", *pStatus);
    return USART_FAIL;
}

/**
  * @brief  Close TCP or UDP Connection
  * 1 Quick close
  * @param  None
  * @retval Status
  */
unsigned char GPRS_CloseLink(void)
{
    unsigned int cmdLen;

    cmdLen = strlen(AT_CIPCLOSE);
    if (USART_SUCESS
            == GSM_SendAT((char *) AT_CIPCLOSE, (char *) AT_OK, cmdLen, MAX_RESP_CMD_DEFAULT))
    {
        return USART_SUCESS;
    }
    return USART_FAIL;
}
/*********************************************************************************************************
 ** Function name:       GPRS_CIPShut
 ** Descriptions:      	退出GPRS链接
 ** input parameters:    NONE
 ** output parameters:   NONE
 ** Returned value:      返回状态结果
 *********************************************************************************************************/
unsigned char GPRS_CIPShut(void)
{
    unsigned int cmdLen;

    cmdLen = strlen(AT_CIPSHUT);
    if (USART_SUCESS
            == GSM_SendAT((char *) AT_CIPSHUT, (char *) AT_OK, cmdLen, MAX_RESP_CMD_DEFAULT))
    {
        return USART_SUCESS;
    }
    return USART_FAIL;
}

unsigned char GPRS_CPOwd(void)
{
    char *pcmdbuf = NULL;
    unsigned int cmdLen;

    //sprintf(pcmdbuf, AT_CPOWD, 1);
    cmdLen = strlen(AT_CPOWD);
    if (USART_SUCESS
            == GSM_SendAT((char *) AT_CPOWD, (char *) "POWER", cmdLen, MAX_RESP_CMD_DEFAULT))
    {
        return USART_SUCESS;
    }
    return USART_FAIL;
}

/**
  * @brief  Set CFUN
  * @param  None
  * @retval Status
  */
uint8_t GSM_SetCFunMin(void)
{
	uint32_t len = 0;
	uint32_t errNum = 0;
	uint8_t rst = USART_FAIL;
	uint8_t rtn = RST_FAIL;
	
	len = strlen(AT_CFUN_MIN);
	GSM_ClearBuffer();
	while (errNum < RETRY_TIMES_CMD_CFUN)
    {
		errNum++;
		rst = GSM_SendAT((char *) AT_CFUN_MIN, (char *) AT_OK, len, MAX_RESP_CMD_CFUN);
		if(USART_SUCESS == rst)
		{
			rtn = RST_OK;
			break;
		}

		delay_10ms(20);
	}

	return rtn;
}

/**
  * @brief  Set CFUN
  * @param  None
  * @retval Status
  */
uint8_t GSM_SetCFunFull(void)
{
	uint32_t len = 0;
	uint32_t errNum = 0;
	uint8_t rst = USART_FAIL;
	uint8_t rtn = RST_FAIL;
	
	len = strlen(AT_CFUN_FULL);
	GSM_ClearBuffer();
	while (errNum < RETRY_TIMES_CMD_CFUN)
    {
		errNum++;
		rst = GSM_SendAT((char *) AT_CFUN_FULL, (char *) AT_OK, len, MAX_RESP_CMD_CFUN);
		if(USART_SUCESS == rst)
		{
			rtn = RST_OK;
			break;
		}

		delay_10ms(20);
	}

	return rtn;
}

unsigned char GSM_creg(void)
{
    static char *pcmdbuf = NULL;
    unsigned int cmdLen = 0;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;

    char *pStr;

    GSM_ClearBuffer();
    GSM_ClearSendBuffer();
    pcmdbuf = sendBuf;
    sprintf(pcmdbuf, AT_CREG_SET, 2);
    cmdLen = strlen(pcmdbuf);
    while (USART_SUCESS != GSM_SendAT((char *) pcmdbuf, (char *) AT_OK, cmdLen, MAX_RESP_CMD_DEFAULT));

    while (1)
    {
        while (USART_SUCESS != GSM_SendAT_rsp((char *)AT_CREG, (char *) "CREG",
                                              sizeof(AT_CREG), &pRecvBuf, &recvLen, MAX_RESP_CMD_DEFAULT));

        pStr = strnchr_len(pRecvBuf, ':', 1, recvLen);
        if (pStr != NULL)
        {
            do
            {
                pStr ++;
            }
            while (*pStr == ' ');

            if (*pStr == '2')
            {
                break;
            }
        }
    }

    // analyze AT_CREG rsp


    return USART_SUCESS;
}



uint8_t GSM_ceng(pST_PACKET_BASESTATION pStation)
{
    uint32_t len = 0;
    uint32_t errNum = 0;
    uint8_t rst = USART_FAIL;
    uint8_t rtn = RST_FAIL;
    static char *pfeed = NULL;
    char *pRecvBuf = NULL;
    uint32_t recvLen = 0;
    static char *pcmdbuf = NULL;

    int idx = 0;
    int result = 0;
    char *ptmp = NULL;
    char *ptmp2 = NULL;

    char *endptr;

    GSM_ClearSendBuffer();
    pcmdbuf = sendBuf;
    if (pcmdbuf == NULL)
    {
        return RST_FAIL;
    }

    pcmdbuf = sendBuf;
    sprintf(pcmdbuf, AT_CENG_SET, 1, 1);
    len = strlen(pcmdbuf);

	GSM_ClearBuffer();
	while(errNum < RETRY_TIMES_CMD_DEFAULT)
	{
		errNum++;
		rst = GSM_SendAT((char *) pcmdbuf, (char *) AT_OK, len, MAX_RESP_CMD_DEFAULT);
		if(USART_SUCESS == rst)
		{
			break;
		}
		delay_10ms(20);
	}
	errNum = 0;
	len = strlen(AT_CENG);
	GSM_ClearBuffer();
	while(errNum < RETRY_TIMES_CMD_DEFAULT)
	{
		pStation->num = 0;
		rst = GSM_SendAT_rsp((char *) AT_CENG, (char *) AT_OK, len, &pRecvBuf, &recvLen, MAX_RESP_CMD_DEFAULT);
		if(USART_SUCESS == rst)
		{
			// analyze CENG rsp
			// 检查是否有CENG字段
			ptmp = strstr_len(pRecvBuf, "+CENG", recvLen);
			if(NULL == ptmp)
			{
				continue;
			}
			
			// The serving Cell
			ptmp = strnchr_len(pRecvBuf, '"', 1 , recvLen);
			if((NULL != ptmp) && (NULL != strnchr_len(pRecvBuf, '+', 2 , recvLen)))
			{
				ptmp2 = strnchr_len(ptmp, ',', 1 , 40);
				if(NULL != ptmp2)
				{
					result = strtol((ptmp2+1), &endptr, 16);
					DEBUG("0: rxl = 0x%x\n", result);
					pStation->stStation[0].rxl = result;
				}
				ptmp2 = strnchr_len(ptmp, ',', 2 , 40);
				if(NULL != ptmp2)
				{
					result = strtol((ptmp2+1), &endptr, 16);
					DEBUG("0: rxq = 0x%x\n", result);
					pStation->stStation[0].rxq = result;
				}
				ptmp2 = strnchr_len(ptmp, ',', 3 , 40);
				if(NULL != ptmp2)
				{
					result = strtol((ptmp2+1), &endptr, 16);
					DEBUG("0: mcc = 0x%x\n", result);
					pStation->stStation[0].mcc.i = result;
				}
				ptmp2 = strnchr_len(ptmp, ',', 4 , 40);
				if(NULL != ptmp2)
				{
					result = strtol((ptmp2+1), &endptr, 16);
					DEBUG("0: mnc = 0x%x\n", result);
					pStation->stStation[0].mnc.i = result;
				}
				ptmp2 = strnchr_len(ptmp, ',', 6 , 40);
				if(NULL != ptmp2)
				{
					result = strtol((ptmp2+1), &endptr, 16);
					DEBUG("0: ci = 0x%x\n", result);
					pStation->stStation[0].ci.i = result;
				}
				ptmp2 = strnchr_len(ptmp, ',', 9 , 40);
				if(NULL != ptmp2)
				{
					result = strtol((ptmp2+1), &endptr, 16);
					DEBUG("0: lac = 0x%x\n", result);
					pStation->stStation[0].lac.i = result;
				}
				pStation->num = 1;
			}
			
			// Neighboring Cell
			for(idx = 1; idx < 7; idx++)
			{
				ptmp = strnchr_len(pRecvBuf, '"', (idx*2+1) , recvLen);
				if((NULL != ptmp) && (NULL != strnchr_len(pRecvBuf, '+', (idx+2) , recvLen)))
				{
					ptmp2 = strnchr_len(ptmp, ',', 1 , 29);
					if(NULL != ptmp2)
					{
						result = strtol((ptmp2+1), &endptr, 16);
						DEBUG("%d: rxl = 0x%x\n", idx, result);
						if(result == 0)
						{
							break;
						}
						pStation->stStation[idx].rxl = result;
					}
					ptmp2 = strnchr_len(ptmp, ',', 3 , 29);
					if(NULL != ptmp2)
					{
						result = strtol((ptmp2+1), &endptr, 16);
						DEBUG("%d: ci = 0x%x\n", idx, result);
						pStation->stStation[idx].rxq = result;
					}
					ptmp2 = strnchr_len(ptmp, ',', 4 , 29);
					if(NULL != ptmp2)
					{
						result = strtol((ptmp2+1), &endptr, 16);
						DEBUG("%d: mcc = 0x%x\n", idx, result);
						pStation->stStation[idx].mcc.i = result;
					}
					ptmp2 = strnchr_len(ptmp, ',', 5 , 29);
					if(NULL != ptmp2)
					{
						result = strtol((ptmp2+1), &endptr, 16);
						DEBUG("%d: mnc = 0x%x\n", idx, result);
						pStation->stStation[idx].mnc.i = result;
					}
					ptmp2 = strnchr_len(ptmp, ',', 6 , 29);
					if(NULL != ptmp2)
					{
						result = strtol((ptmp2+1), &endptr, 16);
						DEBUG("%d: lac = 0x%x\n", idx, result);
						pStation->stStation[idx].lac.i = result;
					}
					
				}
			}
			
			pStation->num = idx;
			rtn = RST_OK;
			break;
		}
	}

	DEBUG("number = %d\r\n", idx);
    return rtn;
}


