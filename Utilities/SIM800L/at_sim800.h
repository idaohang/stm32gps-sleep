#ifndef __AT_SIM908_H_
#define __AT_SIM908_H_


#define RETRY_TIMES_CMD_AT     15
#define RETRY_TIMES_CMD_ATE0   5
#define RETRY_TIMES_CMD_CPIN   6
#define RETRY_TIMES_CMD_CSQ    10
#define RETRY_TIMES_CMD_CADC   4
#define RETRY_TIMES_CMD_CREG   10
#define RETRY_TIMES_CMD_GSN    4
#define RETRY_TIMES_CMD_CIMI   4
#define RETRY_TIMES_CMD_CGATT  5
#define RETRY_TIMES_CMD_CSTT   5
#define RETRY_TIMES_CMD_CIICR  5
#define RETRY_TIMES_CMD_CIFSR  5
#define RETRY_TIMES_CMD_CIPSTART 4

// 500ms
#define MAX_RESP_CMD_DEFAULT 4
#define MAX_RESP_CMD_AT   4  // 2 sec
#define MAX_RESP_CMD_ATE0 4
#define MAX_RESP_CMD_CPIN 10 // 5 sec
#define MAX_RESP_CMD_CSQ  4
#define MAX_RESP_CMD_CADC  4
#define MAX_RESP_CMD_CREG 40 // 20 sec
#define MAX_RESP_CMD_GSN  10
#define MAX_RESP_CMD_CIMI 40 // 20 sec, CIMI max is 20 sec
#define MAX_RESP_CMD_CGATT 40 // 20 sec, CGATT max is 20 sec
#define MAX_RESP_CMD_CSTT  10
#define MAX_RESP_CMD_CIICR 120 // 60 sec, CIICR max is 85 sec
#define MAX_RESP_CMD_CIFSR 20  // 10 sec
#define MAX_RESP_CMD_CIPSTART 20



#define     AT_ACTIVE              '0'
#define     AT_HELD                '1'
#define     AT_DIALING             '2'
#define     AT_ALERTING            '3'
#define     AT_INCOMING            '4'
#define     AT_WAITING             '5'

const char AT_Cmd[]     =   "AT\r";
const char AT_OK[]      =   "OK";
const char AT_READY[]   =   "READY";
const char AT_CONNECTOK[]   =   "CONNECT OK";
const char ATI_Cmd[]    =   "ATI\r";
const char ATE0_Cmd[]   =   "ATE0\r";
const char AT_CSQ[]     =   "AT+CSQ\r";
const char AT_GSN_TEST[]     =   "AT+GSN=?\r";
const char AT_GSN[]     =   "AT+GSN\r";
const char AT_CPIN[]    =   "AT+CPIN?\r";
const char AT_COPS[]    =   "AT+COPS?\r";
const char ATD_Cmd[]    =   "ATD%s;\r"; 									 //���֧��20λ�绰����
const char AT_CPBS[]    =   "AT+CPBS=\"SM\"\r";            //����ѡ��SIMͨѶ¼
const char AT_CPBR[]    =   "AT+CPBR=%d\r";                //��ȡSIM���еĺ���
const char ATD_n[]      =   "ATD>%d;\r";
const char AT_CLCC[]    =   "AT+CLCC\r";
const char ATH_Cmd[]    =   "ATH\r";
const char ATA_Cmd[]    =   "ATA\r";
const char AT_CIPHEAD[]	=   "AT+CIPHEAD\r";
const char AT_CIPSRIP[] =   "AT+CIPSRIP=1\r";
const char AT_CIPQSEND[]=   "AT+CIPQSEND=0\r";
const char AT_CIPSPRT[] =   "AT+CIPSPRT=1\r";
const char AT_CIICR[]		=		"AT+CIICR\r";
const char AT_CIPMODE[] = 	"AT+CIPMODE=1\r"; 
const char AT_CIPMODE_SET[] = 	"AT+CIPMODE=%d\r";
const char AT_CIPMODE_GET[] = 	"AT+CIPMODE?\r";
const char AT_CIPMODE_0[] = 	"AT+CIPMODE=0\r"; 
const char AT_CIPSTART[]=		"AT+CIPSTART=\"%s\",\"%s\",%s\r";
const char AT_CIPSEND[]=   	"AT+CIPSEND\r";
const char AT_CIPSEND_SET[]=   	"AT+CIPSEND=%d\r";
const char AT_CIPSTATUS[]= 	"AT+CIPSTATUS\r";
const char AT_CIPCLOSE[]=		"AT+CIPCLOSE=1\r";
const char AT_CIPSHUT[] =		"AT+CIPSHUT\r";
const char AT_CCLK[]		=		"AT+CCLK?\r";
const char AT_CCLKSet[]	=		"AT+CCLK=\"%d%d/%d%d/%d%d,%d%d:%d%d:%d%d+%d%d\"\r";
const char AT_CMGF[] 		= 	"AT+CMGF=%d\r";
const char AT_CMGS[]    = 	"AT+CMGS=\"%s\"\r";
const char AT_CMGSPDU[] = 	"AT+CMGS=%d\r";
const char AT_CBC[] 	=		"AT+CBC\r";
const char AT_CADC[] 	=		"AT+CADC?\r";
const char AT_CIMI[]    = "AT+CIMI\r";
const char AT_CSCLK_SET[]   = "AT+CSCLK=%d\r";
const char AT_CENG_SET[] = "AT+CENG=%d\r";
const char AT_CENG[] = "AT+CENG?\r";
const char AT_CNUM[]     =   "AT+CNUM\r";
const char AT_CPOWD[]     =   "AT+CPOWD=1\r";

// gps command
const char AT_CGPSPWR[] =       "AT+CGPSPWR=1\r";
const char AT_CGPSRST[] =       "AT+CGPSRST=1\r";
const char AT_CGPSSTATUS[] = "AT+CGPSSTATUS?\r";

// tcpip command
const char AT_CREG[] = "AT+CREG?\r";
const char AT_CREG_SET[] = "AT+CREG=%d\r";
const char AT_CSTT[] = "AT+CSTT?\r";
const char AT_CSTT_SET[] = "AT+CSTT=%s\r";
const char AT_CGATT[] = "AT+CGATT?\r";
const char AT_CIFSR[] = "AT+CIFSR\r";

#endif /* __AT_SIM908_H_ */

