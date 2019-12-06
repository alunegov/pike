#include <stdio.h>
#include <string.h>

#include "../include/stubs.h"
#include "../include/ioctl.h"
#include "../include/salayer.h"
#include "../include/e2010cmd.h"

#include "../include/ifc_ldev.h"
#include "../include/ldevbase.h"
#include "../include/e2010.h"

// размер не лезет.....
// универсально для 2010 и 2010b, но размер прошивок важен!!!!
FDF(ULONG) DaqE2010::LoadBios(char *FileName)
{
CHAR  FName[255];
FILE  *BiosFile;
LONG    PLDSize,NBytes;
PUCHAR  BiosCode=0;
PUCHAR BC=0;
ULONG cbRet;
int status = L_ERROR;
   do
   {
      strcpy(FName,FileName);
      strcat(FName,".pld");

      BiosFile=fopen(FName, "rb");
      if(!BiosFile) break;

      fseek(BiosFile,0,SEEK_END);
      PLDSize=NBytes = ftell(BiosFile);
      rewind(BiosFile);

      BC=BiosCode=new UCHAR[NBytes+2];
      if(BiosCode==NULL) break;
      if(fread(BiosCode,1,NBytes,BiosFile) != NBytes) break;

      // transfer bios to driver
      int len = 4096;
      // first call alloc memory, other simple add data
      do
      {
         if(NBytes<len) len=NBytes;
         if(sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_SEND_BIOS),
                           &NBytes,sizeof(LONG),
                           BiosCode, len*sizeof(CHAR),
                           &cbRet, L_STREAM_NULL)) break;
         BiosCode+=len;
         NBytes-=len;
      } while(NBytes);
      if(NBytes) break; // not all transfered to driver

      // load prev transf. bios and free memory //////////////////////////////////////////////////////////////
      //NBytes = EP1K10_SIZE;
      if(sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_LOAD_BIOS),
                           &PLDSize,sizeof(LONG),
                           NULL, 0,
                           &cbRet, L_STREAM_NULL)) break;
      status = L_SUCCESS; // all ok
   } while (status==L_ERROR);

   if(BC) delete[] BC;
   if(BiosFile) fclose(BiosFile);
   return status;//E;
}
