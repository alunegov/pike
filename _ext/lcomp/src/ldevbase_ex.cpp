#include "../include/stubs.h"
#include "../include/ioctl.h"
#include "../include/salayer.h"
#include "../include/ifc_ldev.h"
#include "../include/ldevbase.h"
#include <stdio.h>
// extended interface for ADC/DAC start-stop

FDF(ULONG) LDaqBoard::InitStartLDeviceEx(ULONG StreamId)
{
ULONG cbRet, OutBuf, status = L_ERROR;
UCHAR InBuf[16];

   return sal->LDeviceIoControl(hVxd,sal->CTL(DIOC_INIT_SYNC),
                           &InBuf,sizeof(ULONG)+StreamId,
                           &OutBuf,sizeof(ULONG),
                           &cbRet,L_STREAM_NULL
                         );
}


FDF(ULONG) LDaqBoard::StartLDeviceEx(ULONG StreamId)
{
ULONG cbRet, status =  L_ERROR;
UCHAR InBuf[16];

   return status = sal->LDeviceIoControl(hVxd,sal->CTL(DIOC_START),
                         &InBuf,sizeof(ULONG)+StreamId,
                         NULL, 0, // here we send data buffer parameters to lock in driver
                         &cbRet,StreamId
                        );
}

FDF(ULONG) LDaqBoard::StopLDeviceEx(ULONG StreamId)
{
ULONG cbRet, OutBuf, status = L_ERROR;
UCHAR InBuf[16];

   status = sal->LDeviceIoControl(hVxd,sal->CTL(DIOC_STOP),
                              &InBuf,sizeof(ULONG)+StreamId,
                              &OutBuf,sizeof(ULONG),
                              &cbRet,L_STREAM_NULL
                             );
   sal->LWaitOverlapped(hVxd,StreamId);

   return status;
};
