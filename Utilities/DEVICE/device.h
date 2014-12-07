
#ifndef __DEVICE_H
#define __DEVICE_H

typedef struct
{
    unsigned char status[2];    // dveice status
    unsigned char analog1[2]; 	// analog input 1
    unsigned char analog2[2];		// analog input 2
} ST_DEVICEDATA, *pST_DEVICEDATA;

#endif // __DEVICE_H

