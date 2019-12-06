#define INITGUID 

#include "../include/stubs.h"
#include "../include/ioctl.h"
#include "../include/salayer.h"
#include "../include/ifc_ldev.h"
#include "../include/ldevbase.h"
#include <string.h>


HRESULT __stdcall LDaqBoard::QueryInterface(const IID& iid, void** ppv)
{
   if(iid==IID_IUnknown)
   {
      *ppv = static_cast<IDaqLDevice*>(this);
   }
   else
   {
      if(iid==IID_ILDEV)
      {
         *ppv = static_cast<IDaqLDevice*>(this);
      }
      else 
      {
         if(iid==IID_ILDEV2)
         {
            *ppv = static_cast<IDaqLDevice2*>(this);
         }
         else
         {
            *ppv=NULL;
            return ((HRESULT)0x80004002L); // E_NOINTERFACE;
         }
      }
   }
   reinterpret_cast<LUnknown*>(*ppv)->AddRef();
   return ((HRESULT)0x00000000L); // S_OK;
}

ULONG __stdcall LDaqBoard::AddRef()
{
   sal->atomic_inc(&m_cRef);
   return m_cRef.counter;
}

ULONG __stdcall LDaqBoard::Release()
{
   sal->atomic_dec(&m_cRef);
   if(m_cRef.counter==0) {
      delete this;
      return 0;
   }
   return m_cRef.counter;
}

// Working with I/O ports
// data - data len - in bytes(sizeof)
FDF(ULONG) LDaqBoard::inbyte(ULONG offset, PUCHAR data, ULONG len, ULONG key )
{
   ULONG cbRet;
   PORT_PAR par; par.datatype=0+key; par.port=offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_INP),
                          &par, sizeof(PORT_PAR),
                          data,len,
                          &cbRet, L_STREAM_NULL
                         );
}

FDF(ULONG) LDaqBoard::inword(ULONG offset, PUSHORT data, ULONG len, ULONG key )
{
   ULONG cbRet;
   PORT_PAR par; par.datatype=1+key; par.port=offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_INP),
                          &par, sizeof(PORT_PAR),
                          data,len,
                          &cbRet, L_STREAM_NULL
                         );
}

FDF(ULONG) LDaqBoard::indword(ULONG offset, PULONG data, ULONG len, ULONG key )
{
   ULONG cbRet;
   PORT_PAR par; par.datatype=2+key; par.port=offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_INP),
                          &par, sizeof(PORT_PAR),
                          data,len,
                          &cbRet, L_STREAM_NULL
                           );
}

FDF(ULONG) LDaqBoard::outbyte(ULONG offset, PUCHAR data, ULONG len, ULONG key )
{
   ULONG cbRet;
   PORT_PAR par; par.datatype=0+key; par.port=offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_OUTP),
                          &par, sizeof(PORT_PAR),
                          data,len,
                          &cbRet, L_STREAM_NULL
                         );
}

FDF(ULONG) LDaqBoard::outword(ULONG offset, PUSHORT data, ULONG len, ULONG key )
{
   ULONG cbRet;
   PORT_PAR par; par.datatype=1+key; par.port=offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_OUTP),
                          &par, sizeof(PORT_PAR),
                          data,len,
                          &cbRet, L_STREAM_NULL
                         );
}

FDF(ULONG) LDaqBoard::outdword(ULONG offset, PULONG data, ULONG len, ULONG key )
{
   ULONG cbRet;
   PORT_PAR par; par.datatype=2+key; par.port=offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_OUTP),
                          &par, sizeof(PORT_PAR),
                          data,len,
                          &cbRet, L_STREAM_NULL
                         );
}

// Working with mem I/O ports
FDF(ULONG) LDaqBoard::inmbyte(ULONG Offset, PUCHAR Data, ULONG Length, ULONG Key)
{
   ULONG cbRet;
   PORT_PAR Par; Par.datatype=0+Key; Par.port=Offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_INM),
                          &Par, sizeof(PORT_PAR),
                          Data, Length,
                          &cbRet, L_STREAM_NULL
                         );
}

FDF(ULONG) LDaqBoard::inmword(ULONG Offset, PUSHORT Data, ULONG Length, ULONG Key)
{
   ULONG cbRet;
   PORT_PAR Par; Par.datatype=1+Key; Par.port=Offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_INM),
                          &Par, sizeof(PORT_PAR),
                          Data, Length,
                          &cbRet, L_STREAM_NULL
                         );
}

FDF(ULONG) LDaqBoard::inmdword(ULONG Offset, PULONG Data, ULONG Length, ULONG Key)
{
   ULONG cbRet;
   PORT_PAR Par; Par.datatype=2+Key; Par.port=Offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_INM),
                          &Par, sizeof(PORT_PAR),
                          Data, Length,
                          &cbRet, L_STREAM_NULL
                         );
}

FDF(ULONG) LDaqBoard::outmbyte(ULONG Offset, PUCHAR Data, ULONG Length, ULONG Key)
{
   ULONG cbRet;
   PORT_PAR Par; Par.datatype=0+Key; Par.port=Offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_OUTM),
                          &Par, sizeof(PORT_PAR),
                          Data, Length,
                          &cbRet, L_STREAM_NULL
                         );
}

FDF(ULONG) LDaqBoard::outmword(ULONG Offset, PUSHORT Data, ULONG Length, ULONG Key)
{
   ULONG cbRet;
   PORT_PAR Par; Par.datatype=1+Key; Par.port=Offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_OUTM),
                          &Par, sizeof(PORT_PAR),
                          Data, Length,
                          &cbRet, L_STREAM_NULL
                         );
}

FDF(ULONG) LDaqBoard::outmdword(ULONG Offset, PULONG Data, ULONG Length, ULONG Key)
{
   ULONG cbRet;
   PORT_PAR Par; Par.datatype=2+Key; Par.port=Offset;
   return sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_OUTM),
                          &Par, sizeof(PORT_PAR),
                          Data, Length,
                          &cbRet, L_STREAM_NULL
                         );
}

// COMMON FUNCTIONS //////////////////////////////////////
FDF(ULONG) LDaqBoard::GetSlotParam(PSLOT_PAR slPar)
{
   memcpy(slPar,&sl,sizeof(SLOT_PAR));
   return 0;
}


FDF(HANDLE) LDaqBoard::OpenLDevice()
{
   ULONG status = L_ERROR;
   ULONG cbRet;

   do
   {
      hVxd = sal->LCreateFile(m_Slot);
      if(hVxd == INVALID_HANDLE_VALUE) break;

      status = sal->LDeviceIoControl(hVxd,sal->CTL(DIOC_GET_PARAMS),
                                     NULL,0,
                                     &sl,sizeof(SLOT_PAR),
                                     &cbRet, L_STREAM_NULL);

      if(status == L_ERROR) { sal->LCloseHandle(hVxd); hVxd = INVALID_HANDLE_VALUE; }
   } while(0);
   sal->LSetLastError(status);
   return hVxd;//csOpenLDevice();
}

FDF(ULONG) LDaqBoard::CloseLDevice()
{
   ULONG status = L_ERROR;

   do
   {
      if(hVxd==INVALID_HANDLE_VALUE) break;

      status = sal->LCloseHandle(hVxd);
      hVxd=INVALID_HANDLE_VALUE;    ////////////////      !!!!!!!!!!!!!!!!!! close before open

      if(map_inBuffer)
      {
         sal->LUnmapMemory(map_inBuffer, map_inSize);
         ULONG sts = sal->LVirtualFree(map_inBuffer); 
         map_inBuffer = NULL;
         if(!sts) status = L_ERROR;
      }

      if(map_outBuffer)
      {
         sal->LUnmapMemory(map_outBuffer, map_inSize);
         ULONG sts = sal->LVirtualFree(map_outBuffer); 
         map_outBuffer = NULL;
         if(!sts) status = L_ERROR;
      }
   } while(0);

   return status;//csCloseLDevice(status);
}


// uni stream interface
FDF(ULONG) LDaqBoard::RequestBufferStream(ULONG *Size, ULONG StreamId) //in 16 bit words
{
ULONG cbRet;
ULONG status = L_ERROR;
ULONG pb = 0;
ULONG RealPages;

void *buffer;

   switch(StreamId)
   {
      case L_STREAM_DDS: { *Size = 0; status = L_SUCCESS; } break; // для DDS E-310 не нужна память

      case L_STREAM_ADC :
         {
            if(map_inBuffer!=NULL)
            {                                                //0
               status = sal->LDeviceIoControl(hVxd,sal->CTL(DIOC_SETBUFFER), &pb,sizeof(ULONG), NULL,0,&cbRet,L_STREAM_NULL);
               if(status == L_ERROR) return L_ERROR;

               sal->LWaitOverlapped(hVxd, L_MEM_ALLOC+StreamId);

               sal->LUnmapMemory(map_inBuffer, map_inSize);
               sal->LVirtualFree(map_inBuffer); 
               map_inBuffer = NULL;
            }

            RealPages= (*Size)/2048+((*Size)%2048 ? 1:0);
            pb = (RealPages+1)*2048;

            buffer = sal->LVirtualAlloc(pb*sizeof(USHORT));
            if(buffer == NULL) return L_ERROR;

            status = sal->LDeviceIoControl(hVxd, sal->CTL(DIOC_SETBUFFER), &pb, sizeof(ULONG), buffer, pb*sizeof(USHORT), &cbRet, L_MEM_ALLOC+StreamId);

            if(status == L_SUCCESS) { *Size = RealPages*2048; } 
            else 
              { sal->LVirtualFree(buffer); buffer = NULL; *Size = 0; }

            map_inSize = *Size * sizeof(short);
            map_inBuffer = sal->LMapMemory(hVxd, buffer, map_inSize, 0x1000);
            if(map_inBuffer == NULL) status = L_ERROR;

         } break;

      case L_STREAM_DAC :
         {
            if(map_outBuffer!=NULL)
            {                                                  //0
               status= sal->LDeviceIoControl(hVxd,sal->CTL(DIOC_SETBUFFER_1), &pb,sizeof(ULONG), NULL,0,&cbRet, L_STREAM_NULL); // запрос на завершение блокировки выделения
               if(status == L_ERROR) return L_ERROR;

               sal->LWaitOverlapped(hVxd, L_MEM_ALLOC+StreamId);

               sal->LUnmapMemory(map_outBuffer, map_outSize);
               sal->LVirtualFree(map_outBuffer);
               map_outBuffer = NULL;
            }

            RealPages= (*Size)/2048+((*Size)%2048 ? 1:0);
            pb = (RealPages+1)*2048;

            buffer = sal->LVirtualAlloc(pb*sizeof(USHORT));
            if(buffer == NULL) return L_ERROR;

            status = sal->LDeviceIoControl(hVxd,sal->CTL(DIOC_SETBUFFER_1), &pb,sizeof(ULONG), buffer, pb*sizeof(USHORT), &cbRet, L_MEM_ALLOC+StreamId);

            if(status == L_SUCCESS) { *Size = RealPages*2048; } 
            else 
              { sal->LVirtualFree(buffer); buffer = NULL; *Size = 0; }

            map_outSize = *Size * sizeof(short);
            map_outBuffer = sal->LMapMemory(hVxd, buffer, map_outSize, 0x2000);
            if(map_outBuffer == NULL) status = L_ERROR;

         } break;

      default: status = L_ERROR;
   }

   return status;//csRequestBufferStream(Size,StreamId,status); // call to class specific function (if no - simple return status)//
}


LPVOID LDaqBoard::GetDAQ_PtrSz(ULONG StreamId, ULONG *sz)
{
   if(sal->LIsWin32()) // windows c++ style
   {
      switch(StreamId)
      {
      case L_STREAM_ADC: { *sz = sizeof(ADC_PAR); return (LPVOID)&adc_par; }
      case L_STREAM_DAC: { *sz = sizeof(DAC_PAR); return (LPVOID)&dac_par; }
      case L_STREAM_DDS: { *sz = sizeof(DAC_PAR); return (LPVOID)&dac_par; }
      }
   }
   else // linux, с style
   {
      switch(StreamId)
      {
      case L_STREAM_ADC: { *sz = sizeof(WDAQ_PAR); return (LPVOID)&wadc_par; }
      case L_STREAM_DAC: { *sz = sizeof(WDAQ_PAR); return (LPVOID)&wdac_par; }
      case L_STREAM_DDS: { *sz = sizeof(WDAQ_PAR); return (LPVOID)&wdac_par; }
      }
   }
   return NULL;
}

FDF(ULONG) LDaqBoard::SetParametersStream(PDAQ_PAR sp, PULONG UsedSize, void** Data, void** Sync, ULONG StreamId)
{
   ULONG cbRet;

   LPVOID OutBuf[4]; // x64 i32 8 or 4 байт

   ULONG status = L_ERROR;
   USHORT *d1;
   ULONG DiocCode;

   LPVOID dp;
   ULONG sz;
   void *ptr;
   USHORT tPages, tFIFO, tIrqStep;

   switch(StreamId)
   {
      case L_STREAM_ADC: { DiocCode = DIOC_SETUP;     } break;
      case L_STREAM_DAC: { DiocCode = DIOC_SETUP_DAC; } break;
      case L_STREAM_DDS: { DiocCode = DIOC_SETUP_DDS; } break;
      default: return status;
   };

   dp = GetDAQ_PtrSz(StreamId, &sz); // get pointer and stuct size

   if(dp == NULL) return status;

   status=sal->LDeviceIoControl(hVxd,sal->CTL(DiocCode),
                          dp,sz,
                          OutBuf,sizeof(OutBuf), // PVOID platform dependent
                          &cbRet,L_STREAM_NULL);

   if(StreamId == L_STREAM_DDS) return L_SUCCESS; // если настраиваем DDS E310 то ничего больше не нужно

   tPages   = (USHORT)OutBuf[0];
   tFIFO    = (USHORT)OutBuf[1];
   tIrqStep = (USHORT)OutBuf[2];

//   ret_val(Pages)   = tPages;            /////////////// FIX IT !!!!!!!!!!!!!!!! нафиг не нужно
//   ret_val(FIFO)    = tFIFO;
//   ret_val(IrqStep) = tIrqStep;

   *UsedSize        = tPages*tIrqStep;

   switch(StreamId)
   {
      case L_STREAM_ADC: { ptr = map_inBuffer; } break;
      case L_STREAM_DAC: { ptr = map_outBuffer;} break;
      default: return L_ERROR;
   }

   if(ptr == NULL) return L_ERROR;

   *Sync = (PULONG)ptr; d1 = (PUSHORT)ptr; *Data = &d1[2048];

   if(sp!=NULL) // update properties to new real values;
   {
      sp->Pages = tPages;   
      sp->FIFO = tFIFO;
      sp->IrqStep = tIrqStep;
   }

//   status = csSetParametersStream(sp, UsedSize, Data, Sync, StreamId, status);

   switch(StreamId) // for GetParameter
   {
      case L_STREAM_ADC: { sync_addr =(PULONG)*Sync;  data_addr =  (PULONG)*Data;} break;
      case L_STREAM_DAC: { sync_addr1 =(PULONG)*Sync; data_addr1 = (PULONG)*Data;} break;
   }

   return status;
}


FDF(ULONG) LDaqBoard::FillDAQparameters(PDAQ_PAR sp)
{
   if(sp == NULL) return L_ERROR;

   switch(sp->s_Type)
   {
      case L_ADC_PARAM:    return FillADCparameters(sp);
      case L_DAC_PARAM:    return FillDACparameters(sp);
      case L_DDS_FM_PARAM: return FillDDSparameters(sp);
      default : return L_ERROR;
   }
}


FDF(ULONG) LDaqBoard::IoAsync(PDAQ_PAR sp)
{
   if(sp == NULL) return L_ERROR;

   switch(sp->s_Type)
   {
      case L_ASYNC_ADC_CFG: return ConfigADC(sp);
      case L_ASYNC_TTL_CFG: return ConfigTTL(sp);
      case L_ASYNC_DAC_CFG: return ConfigDAC(sp);

      case L_ASYNC_ADC_INP: return InputADC(sp);
      case L_ASYNC_TTL_INP: return InputTTL(sp);

      case L_ASYNC_TTL_OUT: return OutputTTL(sp);
      case L_ASYNC_DAC_OUT: return OutputDAC(sp);

      default : return L_ERROR;
   }
}

// end of uni stream interface

// нужно проверять LastError
FDF(ULONG) LDaqBoard::InitStartLDevice()
{
   ULONG Err=0;
   if(InitStartLDeviceEx(L_STREAM_ADC)) SETBIT(Err,L_STREAM_ADC_ERROR);
   if(InitStartLDeviceEx(L_STREAM_DAC)) SETBIT(Err,L_STREAM_DAC_ERROR);
   //sal->LSetLastError(0x20000000+Err);
   //if(Err==0x3) return L_ERROR;
   return L_SUCCESS;
}

// нужно проверять LastError
FDF(ULONG) LDaqBoard::StartLDevice()
{
   ULONG Err=0;
   if(StartLDeviceEx(L_STREAM_ADC)) SETBIT(Err,L_STREAM_ADC_ERROR);
   if(StartLDeviceEx(L_STREAM_DAC)) SETBIT(Err,L_STREAM_DAC_ERROR);
   sal->LSetLastError(0x20000000+Err);
   if(Err==0x3) return L_ERROR;
   return L_SUCCESS;

}

// нужно проверять LastError
FDF(ULONG) LDaqBoard::StopLDevice()
{
   ULONG Err=0;
   if(StopLDeviceEx(L_STREAM_ADC)) SETBIT(Err,L_STREAM_ADC_ERROR);
   if(StopLDeviceEx(L_STREAM_DAC)) SETBIT(Err,L_STREAM_DAC_ERROR);
   sal->LSetLastError(0x20000000+Err);
   if(Err==0x3) return L_ERROR;
   return L_SUCCESS;

};


FDF(ULONG) LDaqBoard::GetParameter(ULONG name, PULONG param)
{
   ULONG status = L_SUCCESS;
   if((name>=10100) && (name<10228)) 
   {
      *param = user_prop[name-10100];
      return status;
   }

   switch(name)
   {
      case L_BOARD_TYPE:     *param = sl.BoardType; break;
      case L_SYNC_ADDR_LO:   *param = ((unsigned _int64)sync_addr)&0xFFFFFFFF; break;
      case L_SYNC_ADDR_HI:   *param = (((unsigned _int64)sync_addr)>>32)&0xFFFFFFFF; break;
      case L_DATA_ADDR_LO:   *param = ((unsigned _int64)data_addr)&0xFFFFFFFF; break;
      case L_DATA_ADDR_HI:   *param = (((unsigned _int64)data_addr)>>32)&0xFFFFFFFF; break;

      case L_SYNC1_ADDR_LO:  *param = ((unsigned _int64)sync_addr1)&0xFFFFFFFF; break;
      case L_SYNC1_ADDR_HI:  *param = (((unsigned _int64)sync_addr1)>>32)&0xFFFFFFFF; break;
      case L_DATA1_ADDR_LO:  *param = ((unsigned _int64)data_addr1)&0xFFFFFFFF; break;
      case L_DATA1_ADDR_HI:  *param = (((unsigned _int64)data_addr1)>>32)&0xFFFFFFFF; break;

      default: return csGetParameter(name, param, status);
   }
   return status;
}

FDF(ULONG) LDaqBoard::SetParameter(ULONG name, PULONG param)
{
   ULONG status = L_SUCCESS;
   if((name>=10100) && (name<10228))
   {
      user_prop[name-10100] = *param;
      return status;
   }

   return csSetParameter(name, param, status);
}




// work with event
// DIOC_SETEVENT - adc stop event; DIOC_SETEVENT_DAC - dac stop event
FDF(ULONG) LDaqBoard::SetLDeviceEvent(HANDLE hEvnt,ULONG EventId)
{

      PVOID InBuf[1];
      ULONG OutBuf,cbRet;
      InBuf[0]=hEvnt;
      ULONG DiocCode;

      if(sal->LIsWin32()) // windows c++ style
      {
         switch(EventId)
         {
            //case L_EVENT_ADC_BUF:
            case L_STREAM_ADC : DiocCode = DIOC_SETEVENT; break;
            //case L_EVENT_DAC_BUF:
            case L_STREAM_DAC : DiocCode = DIOC_SETEVENT_DAC; break;

            default: return L_ERROR;
         }

         return sal->LDeviceIoControl(hVxd,sal->CTL(DiocCode),
                                   &InBuf,sizeof(PVOID),
                                   &OutBuf,sizeof(OutBuf),
                                   &cbRet,L_STREAM_NULL
                                  );
      } else return L_NOTSUPPORTED;
}

void LDaqBoard::CopyDAQtoWDAQ(PDAQ_PAR dp, LPVOID ss, int sp_type)
{
//DAC_PAR_0 d0_sp;  //0
//DAC_PAR_1 d1_sp;  //1
//ADC_PAR_0 a0_sp;  //2
//ADC_PAR_1 a1_sp;  //3
PDAC_PAR dac = (PDAC_PAR)dp;
PADC_PAR adc = (PADC_PAR)dp;
PWDAQ_PAR sp = (PWDAQ_PAR)ss;
      // декодируем тип структуры
   switch (sp_type) {
      case 0:
         {
            sp->t1.s_Type    = dac->t1.s_Type;
            sp->t1.FIFO      = dac->t1.FIFO;
            sp->t1.IrqStep   = dac->t1.IrqStep;
            sp->t1.Pages     = dac->t1.Pages;
            sp->t1.AutoInit  = dac->t1.AutoInit;
            sp->t1.dRate     = dac->t1.dRate;
            sp->t1.Rate      = dac->t1.Rate;
            sp->t1.DacNumber = dac->t1.DacNumber;
            sp->t1.DacEna    = dac->t1.DacEna;
            sp->t1.IrqEna    = dac->t1.IrqEna;
         } break;
      case 1:
         {
            sp->t2.s_Type   = dac->t2.s_Type;
            sp->t2.FIFO     = dac->t2.FIFO;
            sp->t2.IrqStep  = dac->t2.IrqStep;
            sp->t2.Pages    = dac->t2.Pages;
            sp->t2.AutoInit = dac->t2.AutoInit;
            sp->t2.dRate    = dac->t2.dRate;
            sp->t2.Rate     = dac->t2.Rate;
            sp->t2.DacEna   = dac->t2.DacEna;
            sp->t2.IrqEna   = dac->t2.IrqEna;
         } break;
      case 2:
         {
            sp->t3.s_Type   = adc->t1.s_Type;
            sp->t3.FIFO     = adc->t1.FIFO;
            sp->t3.IrqStep  = adc->t1.IrqStep;
            sp->t3.Pages    = adc->t1.Pages;
            sp->t3.AutoInit = adc->t1.AutoInit;
            sp->t3.dRate    = adc->t1.dRate;
            sp->t3.dKadr    = adc->t1.dKadr;
            sp->t3.dScale   = adc->t1.dScale;
            sp->t3.Rate     = adc->t1.Rate;
            sp->t3.Kadr     = adc->t1.Kadr;
            sp->t3.Scale    = adc->t1.Scale;
            sp->t3.FPDelay  = adc->t1.FPDelay;

            sp->t3.SynchroType        = adc->t1.SynchroType;
            sp->t3.SynchroSensitivity = adc->t1.SynchroSensitivity;
            sp->t3.SynchroMode        = adc->t1.SynchroMode;
            sp->t3.AdChannel          = adc->t1.AdChannel;
            sp->t3.AdPorog            = adc->t1.AdPorog;
            sp->t3.NCh                = adc->t1.NCh;
            for(int i=0;i<128;i++) sp->t3.Chn[i] = adc->t1.Chn[i];
            sp->t3.AdcEna = adc->t1.AdcEna;
            sp->t3.IrqEna = adc->t1.IrqEna;
         } break;
      case 3:
         {
            sp->t4.s_Type    = adc->t2.s_Type;
            sp->t4.FIFO      = adc->t2.FIFO;
            sp->t4.IrqStep   = adc->t2.IrqStep;
            sp->t4.Pages     = adc->t2.Pages;
            sp->t4.AutoInit  = adc->t2.AutoInit;
            sp->t4.dRate     = adc->t2.dRate;
            sp->t4.dKadr     = adc->t2.dKadr;
            sp->t4.Reserved1 = adc->t2.Reserved1;
            sp->t4.DM_Ena    = adc->t2.DM_Ena;
            sp->t4.Rate      = adc->t2.Rate;
            sp->t4.Kadr      = adc->t2.Kadr;
            sp->t4.StartCnt  = adc->t2.StartCnt;
            sp->t4.StopCnt   = adc->t2.StopCnt;


            sp->t4.SynchroType = adc->t2.SynchroType;
            sp->t4.SynchroMode = adc->t2.SynchroMode;
            sp->t4.AdPorog     = adc->t2.AdPorog;
            sp->t4.SynchroSrc  = adc->t2.SynchroSrc;
            sp->t4.AdcIMask    = adc->t2.AdcIMask;
            sp->t4.NCh         = adc->t2.NCh;
            for(int i=0;i<128;i++) sp->t4.Chn[i] = adc->t2.Chn[i];
            sp->t4.AdcEna = adc->t2.AdcEna;
            sp->t4.IrqEna = adc->t2.IrqEna;
         } break;
      }
}
