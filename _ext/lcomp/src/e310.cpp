#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/stubs.h"
#include "../include/ioctl.h"
#include "../include/salayer.h"
#include "../include/e310cmd.h"

#include "../include/ifc_ldev.h"
#include "../include/ldevbase.h"
#include "../include/e310.h"


// descr async i/o routines for adc,dac & ttl
/*
typedef struct _ASYNC_PARAM_
 : public DAQ_PAR
{
   double dRate;
   ULONG Rate;
   ULONG NCh;
   ULONG Chn[128];
   ULONG Data[128];
   ULONG Mode;
} ASYNC_PAR, *PASYNC_PAR;
*/

// ttl out
//NCh - pattern линий TTL
// Mode none 0 ,config 1 , in 2, out 3
// Data[0] данные


// 4 канала ацп
// Mode -  режим запуска АЦП iNTERNAL 00000000  , EXT 00010000
// NCh -   маска активных каналов АЦП E310_ADC_CH0 00000010 00000020 00000040 00000080
// Data[] результат
ULONG DaqE310::ConfigADC(PDAQ_PAR sp)
{
   PASYNC_PAR ap = (PASYNC_PAR)sp;
   ULONG par;
   ULONG cbRet;

   par = (ap->Mode & 0x00010000) | (ap->NCh & 0x000000F0);

   ULONG status = sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_ADC_CFG),
                           &par, sizeof(ULONG),
                           NULL, 0,
                           &cbRet, L_STREAM_NULL
                         );
   return status;
}

ULONG DaqE310::InputADC(PDAQ_PAR sp)
{
   PASYNC_PAR ap = (PASYNC_PAR)sp;
   ULONG Buf[4];
   ULONG cbRet;

   ULONG status = sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_ADCSAMPLE),
                           NULL, 0,
                           Buf, 4*sizeof(ULONG),
                           &cbRet, L_STREAM_NULL
                         );
   ap->Data[0] = Buf[0];
   ap->Data[1] = Buf[1];
   ap->Data[2] = Buf[2];
   ap->Data[3] = Buf[3];
   return status;
}

//NCh  - pattern TTL
//Mode - add ttl lines 1 ena 0 dis
ULONG DaqE310::ConfigTTL(PDAQ_PAR sp)
{
   PASYNC_PAR ap = (PASYNC_PAR)sp;
   ULONG cbRet;
   ULONG pattern;

   if(ap->Mode) pattern =  ap->NCh & 0x07FF;
   else pattern =  ap->NCh & 0x00FF;

   ULONG status = sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_TTL_CFG),
                           &pattern, sizeof(ULONG),
                           NULL, 0,
                           &cbRet, L_STREAM_NULL
                         );
   return status;
}

// Mode Nch from config call
// Data[0] данные
ULONG DaqE310::InputTTL(PDAQ_PAR sp)
{
   PASYNC_PAR ap = (PASYNC_PAR)sp;
   ULONG TtlParam;
   ULONG pattern;
   ULONG cbRet;
/*
   if(ap->Mode) pattern =  (~ap->NCh) & 0x07FF;
   else pattern =  (~ap->NCh) & 0x00FF;
*/
   ULONG status = sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_TTL_IN),
                           NULL, 0,
                           &TtlParam, sizeof(ULONG),
                           &cbRet, L_STREAM_NULL
                         );
   ap->Data[0] = TtlParam;// & pattern;
   return status;
}

// Mode Nch from config call
// Data[0] данные
ULONG DaqE310::OutputTTL(PDAQ_PAR sp)
{
   PASYNC_PAR ap = (PASYNC_PAR)sp;
   ULONG TtlParam = ap->Data[0];// & ap->NCh;
   ULONG cbRet;
   ULONG status = sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_TTL_OUT),
                           &TtlParam, sizeof(ULONG),
                           NULL, 0,
                           &cbRet, L_STREAM_NULL
                         );
   return status;
}


FDF(ULONG) DaqE310::PlataTest()  // stub function
{
   return L_SUCCESS;
}


FDF(ULONG) DaqE310::ReadPlataDescr(LPVOID pd)
{
   ULONG cbRet;
   USHORT Addr=0;
   ULONG status = sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_READ_FLASH_WORD),
                           &Addr, sizeof(USHORT),
                           &pdu, sizeof(PLATA_DESCR_U2),
                           &cbRet, L_STREAM_NULL
                         );

   memcpy(pd,&pdu,sizeof(PLATA_DESCR_U2));
   return status;
}


// пишет все независимо от Ena
FDF(ULONG) DaqE310::WritePlataDescr(LPVOID pd, USHORT Ena)
{
   memcpy(&pdu,pd,sizeof(PLATA_DESCR_U2));
   pdu.t8.CRC  = CRC16CALC(&pdu.bi256.data[0], 254);
   if(!Ena) return L_ERROR;
   ULONG cbRet;
   USHORT Addr=0;

   ULONG status = sal->LDeviceIoControl( hVxd, sal->CTL(DIOC_WRITE_FLASH_WORD),
                           &Addr, sizeof(USHORT),
                           &pdu, sizeof(PLATA_DESCR_U2),
                           &cbRet, L_STREAM_NULL
                         );
   return status;
}


ULONG DaqE310::csGetParameter(ULONG name, PULONG param, ULONG status)
{
   switch(name)
   {
      case L_POINT_SIZE: *param = 2; break;
      default: return L_NOTSUPPORTED;
   }
   return status;
}

ULONG DaqE310::csSetParameter(ULONG name, PULONG param, ULONG status)
{
   switch(name)
   {
      case 9999:
      default: return L_NOTSUPPORTED;
   }
   return status;
}

//FDF(ULONG)
ULONG DaqE310::FillDDSparameters(PDAQ_PAR sp)
{
   PDAC_PAR ap = (PDAC_PAR)sp;
   memset(&dac_par.t3.dds_fm,0,sizeof(dac_par.t3.dds_fm));

   dac_par.t3.dds_fm.DDS_CtrlReg = DDS_CTRL_PRESET;

   if((ap->t3.MasterClk < 0.001) || (ap->t3.MasterClk > INTERNAL_DDS_MASTER_CLOCK)) ap->t3.MasterClk = INTERNAL_DDS_MASTER_CLOCK;
   if(ap->t3.MasterSrc == INT_MCLK_LINE) ap->t3.MasterClk = INTERNAL_DDS_MASTER_CLOCK;
   dac_par.t3.dds_fm.MasterClock = ap->t3.MasterClk; // 50000
   if(ap->t3.MasterSrc == EXT_MCLK_LINE) SETBIT(dac_par.t3.dds_fm.DDS_CtrlReg, BIT_MCLK_LINE);

   ap->t3.ScanSteps &= 0xFFF; // return
   dac_par.t3.dds_fm.IncStepsReg = ap->t3.ScanSteps | NUMBER_OF_INCREMENTS_ADDR_REG;

   USHORT Sign = (ap->t3.ScanFreqInc >= 0) ? 0:1;
   ULONG SFICode = (ULONG)((l_fabs(ap->t3.ScanFreqInc)/dac_par.t3.dds_fm.MasterClock) * MAX_FREQ_DDS_CODE + 0.5);
   if(SFICode > MAX_FREQ_DDS_CODE) SFICode = MAX_FREQ_DDS_CODE;
   ap->t3.ScanFreqInc = (Sign ? (-1.0):(1.0)) * (SFICode*dac_par.t3.dds_fm.MasterClock)/MAX_FREQ_DDS_CODE; // return
   dac_par.t3.dds_fm.LoDeltaFreqReg = LO_DELTA_FREQUENCY_ADDR_REG | (USHORT)(SFICode&0x0FFF);
   dac_par.t3.dds_fm.HiDeltaFreqReg = HI_DELTA_FREQUENCY_ADDR_REG | (Sign<<11) | (USHORT)((SFICode>>12)&0x07FF);

   ap->t3.StartFreq = l_fabs(ap->t3.StartFreq);
   ULONG SFCode = (ULONG)((ap->t3.StartFreq/dac_par.t3.dds_fm.MasterClock) * MAX_FREQ_DDS_CODE + 0.5);
   if(SFCode > MAX_FREQ_DDS_CODE) SFCode = MAX_FREQ_DDS_CODE;
   ap->t3.StartFreq = (SFCode*dac_par.t3.dds_fm.MasterClock)/MAX_FREQ_DDS_CODE; // return
   dac_par.t3.dds_fm.LoStartFreqReg = LO_START_FREQUENCY_ADDR_REG | (USHORT)(SFCode&0x0FFF);
   dac_par.t3.dds_fm.HiStartFreqReg = HI_START_FREQUENCY_ADDR_REG | (USHORT)((SFCode>>12)&0x0FFF);

   ap->t3.StopFreq = ap->t3.StartFreq + ap->t3.ScanSteps*ap->t3.ScanFreqInc;
   if((ap->t3.StopFreq < 0) || (ap->t3.StopFreq > ap->t3.MasterClk/2.)) return L_ERROR;

   ULONG EFCode = (ULONG)((ap->t3.StopFreq/dac_par.t3.dds_fm.MasterClock) * MAX_FREQ_DDS_CODE + 0.5);
   if(EFCode > MAX_FREQ_DDS_CODE) return L_ERROR;

   ap->t3.StopFreq = (EFCode*dac_par.t3.dds_fm.MasterClock)/MAX_FREQ_DDS_CODE; // return
   dac_par.t3.dds_fm.LoStopFreqReg = LO_START_FREQUENCY_ADDR_REG | (USHORT)(EFCode&0x0FFF);
   dac_par.t3.dds_fm.HiStopFreqReg = HI_START_FREQUENCY_ADDR_REG | (USHORT)((EFCode>>12)&0x0FFF);

   ap->t3.ScanPeriodQnt = (ap->t3.ScanPeriodQnt < 2) ? 2: (ap->t3.ScanPeriodQnt&0x7FF);
   dac_par.t3.dds_fm.IncIntervalReg = ap->t3.ScanPeriodBase | ap->t3.ScanPeriodMult | ap->t3.ScanPeriodQnt;

   if(ap->t3.TTLWaveEna) SETBIT(dac_par.t3.dds_fm.DDS_CtrlReg, BIT_MSBOUTEN);

   if(ap->t3.SyncOutType==SYNC_EOS) SETBIT(dac_par.t3.dds_fm.DDS_CtrlReg, BIT_SYNCSEL);
   if(ap->t3.SyncOutEna) SETBIT(dac_par.t3.dds_fm.DDS_CtrlReg, BIT_SYNCOUTEN);

   if(ap->t3.IRQLineType == EXT_IRQ_LINE) SETBIT(dac_par.t3.dds_fm.CtrlReg, BIT_INTR_LINE);

   dac_par.t3.dds_fm.AutoScanType = ap->t3.CyclicScan;

   if(ap->t3.ScanIncType == CTRL_INCREMENT) SETBIT(dac_par.t3.dds_fm.DDS_CtrlReg,BIT_INT_OR_EXT_INCR); //auto
   if(ap->t3.ScanCtrlType==EXT_CTRL_LINE) SETBIT(dac_par.t3.dds_fm.CtrlReg,BIT_CTRL_LINE); //if ext set bit

   if(ap->t3.CyclicScan)
   {
      CLEARBIT(dac_par.t3.dds_fm.DDS_CtrlReg,BIT_INT_OR_EXT_INCR); //auto inc
      CLEARBIT(dac_par.t3.dds_fm.CtrlReg,BIT_CTRL_LINE); //internal
      SETBIT(dac_par.t3.dds_fm.DDS_CtrlReg, BIT_SYNCSEL);
      SETBIT(dac_par.t3.dds_fm.DDS_CtrlReg, BIT_SYNCOUTEN);
   }

   if(ap->t3.SignalForm == TRI_WAVE) SETBIT(dac_par.t3.dds_fm.DDS_CtrlReg, BIT_SIN_OR_TRI);

   if(ap->t3.Out10_offsrc == EXT_10OHM_OFFSET) SETBIT(dac_par.t3.dds_fm.CtrlReg, BIT_10OHM_OFFSET);

   // == величина внутреннего смещение на выходе 10 Ом в В   коду 000 --> -4B   коду 255 --> +4B
   if((ap->t3.Out10_offset < -4.0) || (ap->t3.Out10_offset > 4.0)) ap->t3.Out10_offset = 0.0;
   dac_par.t3.dds_fm.OUT_10_OFFSET = (UCHAR)(255.0*(ap->t3.Out10_offset + 4.0)/8.0 + 0.5); // addr 0 по умолчанию
   ap->t3.Out10_offset = 8.0*dac_par.t3.dds_fm.OUT_10_OFFSET/255.0 - 4.0; // return

   dac_par.t3.dds_fm.CtrlReg |= ap->t3.OutGain;

   return L_SUCCESS;
}