#ifndef __GSM_APP_H_
#define __GSM_APP_H_

#include "stm32gps_config.h"

#define BASE_STATION_NUM_MAX	7	// Base Station Max Number

typedef	struct
{
    char CMDType;
    void *pCMDData;
} ST_GSMCMD, *pST_GSMCMD;

typedef struct
{
    char PhoneNumber[12];
    char Name[6];
} ST_PHONEBOOKS, *pST_PHONEBOOKS;

typedef struct
{
    char RemoteIP[32];
    char RemotePort[5];
    char TransferMode[4];
    char APN[10];
} ST_NETWORKCONFIG, *pST_NETWORKCONFIG;

typedef struct
{
    unsigned char	BatStatus;	//电池状态，0没充电，1充电中，2已充满
    unsigned char BatPower;		//电池电量，0-100
    union
    {
        unsigned char s[2];
        unsigned short  i;     // 电池电压mV
    } BatVoltage;
} ST_BATTERYSTATUS, *pST_BATTERYSTATUS;

typedef struct
{
    unsigned char	Status;		//是否有效 0无效，1有效
    union
    {
        unsigned char s[2];
        unsigned short  i;     	// 电池电压mV
    } BatVoltage;
} ST_BATVOLTAGESTATUS, *pST_BATVOLTAGESTATUS;

typedef struct
{
    char Mcc[2];		// 国家代码
    char Mnc[2];		// 网络代码
} ST_IMSIINFO, *pST_IMSIINFO;

typedef struct
{
    unsigned char n;    // 0 - disable network; 1 - ; 2 - with location
    unsigned char Stat; // 1 - registered, home network
    char Lac[2];		// location area code
    char Ci[2];			// cell ID
} ST_CREGINFO, *pST_CREGINFO;

typedef     struct
{
    /* date and time components */
    signed char     Sec;
    signed char     Min;
    signed char     Hour;
    signed char     Day;
    signed char     Month;
    signed char     Year;
    signed char     Week;
    signed char     Timezone;
} ST_RTCTIME, *pST_RTCTIME;

typedef struct
{
    char i;
} stru_GSMDATA;

typedef struct
{
    union
    {
        char s[2];
        uint16_t i;
    } mcc;
    union
    {
        char s[2];
        uint16_t i;
    } mnc;
    union
    {
        char s[2];
        uint16_t i;
    } lac;
    union
    {
        char s[3];
        uint16_t i;
    } ci;
    char rxl;
    char rxq;
} ST_BASESTATION;

typedef struct
{
    uint8_t num;
    ST_BASESTATION stStation[BASE_STATION_NUM_MAX];
} ST_PACKET_BASESTATION, *pST_PACKET_BASESTATION;


char *strstr_len(char *str, char *subStr, uint32_t strlenth);
char *strstrlen_len(char *str, char *subStr, uint32_t strlenth, uint32_t *len);
char *strnchr(char *S, int C, int n);

void GSM_PowerOn(void);
void GSM_PowerOff(void);
void GSM_TurnOnOff(void);
void GSM_ClearBuffer(void);
void GSM_ClearSendBuffer(void);
unsigned char GSM_SendAT(char *pCMD, char *pCMDBack, uint32_t CMDLen, uint8_t Count);
unsigned char GSM_SendAT_rsp(char *pCMD, char *pCMDBack,
                             uint32_t CMDLen, char **ppRecvBuf, uint32_t *pRecvLen, uint8_t Count);
unsigned char GSM_QueryNetType(void);
unsigned char GSM_QuerySignal(unsigned char *pSig);
uint8_t GSM_QueryImei(uint8_t *pImei);
unsigned char GSM_QueryImsiBuf(uint8_t *pImsi);
uint8_t GSM_QueryImsi(pST_IMSIINFO pImsiInfo);
unsigned char GSM_QueryCreg(pST_CREGINFO pCregInfo);
unsigned char GSM_QueryNumber(uint8_t *pNum);

unsigned char GSM_CallNumber(char *pNumber);
unsigned char GSM_ReadNumberFromSIM(unsigned char No, ST_PHONEBOOKS *pNumber);
unsigned char GSM_CallSIMNumber(unsigned char numberNo);
unsigned char GSM_QueryCallStatus(void);
unsigned char GSM_HangCall(void);
unsigned char GSM_AnswerCall(void);

unsigned char GSM_GetRTCTime(pST_RTCTIME prtctime);
unsigned char GSM_SetRTCTime(ST_RTCTIME rtctime);
unsigned char GSM_SendSMS(char *pNumb, char *pSMS, unsigned char type);

uint8_t GSM_Init(void);
uint8_t GSM_CheckSIMCard(void);
uint8_t GSM_CheckNetworkReg(void);
void GSM_SetNetworkReg(uint8_t data);
unsigned char GSM_QueryCIPMode(unsigned char *pMode);
void GSM_SetCIPMode(unsigned char mode);
uint8_t GSM_CheckGPRSService(void);
uint8_t GSM_StartTaskAndSetAPN(void);
uint8_t GSM_BringUpConnect(void);
uint8_t GSM_GetLocalIP(void);
uint8_t GSM_StartUpConnect(void);
unsigned char GPRS_LinkServer(pST_NETWORKCONFIG pnetconfig);
unsigned char GPRS_CloseLink(void);
unsigned char GPRS_CIPShut(void);
unsigned char GPRS_CPOwd(void);
uint8_t GSM_SetCFunMin(void);
uint8_t GSM_SetCFunFull(void);
unsigned char GPRS_SendData(char *pString, unsigned int len);
unsigned char GPRS_SendData_rsp(char *pString, unsigned int len, char **ppRecvBuf, uint32_t *pRecvLen);
uint8_t GSM_QueryBatVoltage(pST_BATVOLTAGESTATUS pSig);

unsigned char GPRS_CheckLinkStatus(unsigned char *pStatus);
uint8_t GSM_ceng(pST_PACKET_BASESTATION pStation);


#endif
