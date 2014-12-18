
#ifndef __EELINK_H
#define __EELINK_H

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_EELINK_HEADER 	0x70    // normal protocal header
#define PROTO_FACTORY_HEADER 	0x72 	// factory test header

#define PROTO_LOGIN_BUF_LEN 	32		// login packet buffer length
#define PROTO_GPS_BUF_LEN  		128		// gps/station/alarm packet buffer length

#define IMEI_BUF_LEN 			15      // imei buffer length
#define IMSI_BUF_LEN 			15      // imsi buffer length
#define PHONE_NUM_BUF_LEN 		15      // phone number buffer length

    typedef enum protoEelinkPacketType
    {
        PACKET_EELINK_MIN 				= 0,
                  PACKET_EELINK_MAX 				= 0xFF,
                            PACKET_EELINK_LOGIN 			= 0x1,
                                     PACKET_EELINK_GPS 				= 0x2,
                                               PACKET_EELINK_HEARTBEAT 		= 0x3,
                                                     PACKET_EELINK_WARNING 			= 0x4,
                                                            PACKET_EELINK_TERMSTATUS 		= 0x5,
                                                                 PACKET_EELINK_MESSAGEUPLOAD 	= 0x6,
                                                                    PACKET_EELINK_INTERACTIVE 		= 0x80,
                                                                        PACKET_EELINK_MESSAGEDOWNLOAD 	= 0x81,
                                                                         PACKET_EELINK_PHOTOINFO 		= 0x0E,
                                                                               PACKET_EELINK_PHOTODATA 		= 0x0F,
                                                                                     PACKET_EELINK_OBDDATA 			= 0x07,
                                                                                            PACKET_EELINK_OBDERR 			= 0x08,
                                                                                                    PACKET_FACTORY_REPORT 			= 0x77,
                                                                                                           PACKET_EELINK_STATION 			= 0xA0
    }
                                                                                                               PROTO_EELINK_PACKET_TYPE;

#ifdef __cplusplus
}
#endif

#endif // __EELINK_H

