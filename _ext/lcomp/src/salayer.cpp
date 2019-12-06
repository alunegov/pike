#include <windows.h>
#include <stdio.h>
#include <string.h>

#include "../include/ioctl.h"
#include "../include/salayer.h"


class SAL::sySAL
{
public:

	sySAL()
   { 
      for(int i=0;i<20;i++) memset(&ov[i], 0, sizeof(OVERLAPPED));
      InitializeCriticalSection (&cs_ref);
   }

	~sySAL() 
	{
      for(int i=0;i<20;i++) //if(hEvent[i]) {CloseHandle(hEvent[i]); hEvent[i] = 0;}
         if(ov[i].hEvent) { CloseHandle(ov[i].hEvent); ov[i].hEvent = 0;}
      DeleteCriticalSection (&cs_ref);
	};

	HANDLE syCreateFile(int slot)
	{
      char szDrvName[18], szSlot[4];
      sprintf(szSlot,"%d",slot);

      //#ifdef LCOMP_LINUX
//         strcpy(szDrvName,"/dev/ldev");
  //    #else
      strcpy(szDrvName,"\\\\.\\LDEV");
    //  #endif

      strncat(szDrvName,szSlot,strlen(szSlot));

      ULONG dwCreation = OPEN_EXISTING;
      ULONG dwFlags = FILE_FLAG_OVERLAPPED;
      HANDLE hnd = CreateFile(szDrvName, GENERIC_WRITE | GENERIC_READ, 0, // single use only no sharing
                        NULL, dwCreation, dwFlags, NULL);
      if(hnd==INVALID_HANDLE_VALUE)
      {
         if(GetLastError()==ERROR_FILE_NOT_FOUND) SetLastError(L_ERROR_NOBOARD);
         if(GetLastError()==ERROR_ACCESS_DENIED) SetLastError(L_ERROR_INUSE);
      }
      return hnd;
	}
	
	BOOL syCloseHandle(HANDLE hDevice) { return !CloseHandle(hDevice); } 
	
	ULONG syDeviceIoControl(HANDLE hDevice,
                         ULONG dwIoControlCode,
                         LPVOID lpInBuffer,
                         ULONG nInBufferSize,
                         LPVOID lpOutBuffer,
                         ULONG nOutBufferSize,
                         PULONG lpBytesReturned,
                         ULONG StreamId)
	{
      // для простых запросов локальный overlapped
      OVERLAPPED ot;
      OVERLAPPED *o;
      if(StreamId == L_STREAM_NULL)
      {
      	memset(&ot, 0, sizeof(OVERLAPPED));
	      ot.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
         o =  &ot;
      }
      else
      {
   	   memset(&ov[StreamId], 0, sizeof(OVERLAPPED));
	      ov[StreamId].hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
         o = &ov[StreamId];
      }
	   
   	ULONG status = DeviceIoControl( hDevice, dwIoControlCode,
                           lpInBuffer, nInBufferSize,
                           lpOutBuffer, nOutBufferSize,
                           lpBytesReturned, o//v[StreamId]
                         );

		if(status != 0)
      {
         CloseHandle(o/*v[StreamId].*/->hEvent);
         o->hEvent = 0;
			return L_SUCCESS;
      }
      // иначе
	  	if(GetLastError() == ERROR_IO_PENDING)
   	{
 			if(StreamId == L_STREAM_NULL)
			{
     			GetOverlappedResult(hDevice, o/*v[StreamId]*/, lpBytesReturned, TRUE); // ждем завершения
     			CloseHandle(o/*v[StreamId]*/->hEvent);
            o->hEvent = 0;
   		}
			return L_SUCCESS;
		}
   	else
		{
         CloseHandle(o/*v[StreamId].*/->hEvent);
         o->hEvent = 0;
			return L_ERROR;
		}
	}

   ULONG syWaitOverlapped(HANDLE hDevice, ULONG StreamId) // ждет завершения overlapped операции
   {
      ULONG cbRet;
      if(ov[StreamId].hEvent)
      {
         GetOverlappedResult(hDevice, &ov[StreamId], &cbRet, TRUE); // ждем завершения
         CloseHandle(ov[StreamId].hEvent); ov[StreamId].hEvent = 0;
         return L_SUCCESS;
      } else return L_ERROR;
   }

   void sySetLastError(unsigned long Err) { SetLastError(Err); }
   unsigned long syGetLastError(void) { return GetLastError(); }

   void sy_atomic_inc(atomic_t *v)
   {
      EnterCriticalSection (&cs_ref);
         v->counter++;
      LeaveCriticalSection (&cs_ref);
   }

   void sy_atomic_dec(atomic_t *v)
   {
      EnterCriticalSection (&cs_ref);
         v->counter--;
      LeaveCriticalSection (&cs_ref);
   }

   void syUnmapMemory(LPVOID lpAddr, long Size ) { return; }
   LPVOID syMapMemory(HANDLE hDevice, LPVOID lpAddr, long Size, long off) { return lpAddr; }

   LPVOID syVirtualAlloc(size_t size)
   {
      return VirtualAlloc(NULL, size, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
   }

   BOOL syVirtualFree(LPVOID addr)
   {
      return VirtualFree(addr,0,MEM_RELEASE);
   }

   BOOL syIsWin32() { return TRUE; }

   
   ULONG syDIOC(ULONG cmd)
   {
      switch(cmd)
      {
         // METHOD_OUT_DIRECT
      case  DIOC_SETUP:
      case  DIOC_START:
      case  DIOC_STOP:
      case  DIOC_INP:
      case  DIOC_INM:
      case  DIOC_SETBUFFER:
      case  DIOC_ADD_BOARDS:
      case  DIOC_CLEAR_BOARDS:
      case  DIOC_INIT_SYNC:
      //case DIOC_SETBUFFER_DAC:
      case DIOC_SETEVENT_DAC:
      case DIOC_SEND_COMMAND:
      case DIOC_COMMAND_PLX:
      case DIOC_GET_DATA_A:
      case DIOC_GET_DM_A:
      case DIOC_GET_PM_A:
      case DIOC_GET_PARAMS:
      case DIOC_SET_DSP_TYPE:
      case DIOC_SETBUFFER_1:
      case DIOC_SETUP_DAC:
      case DIOC_READ_FLASH_WORD:
      case DIOC_ADCSAMPLE:
      case DIOC_TTL_IN:
      case DIOC_ENABLE_CORRECTION:
      case DIOC_START_DAC:

      //case DIOC_STOP_DAC:
      //case DIOC_INIT_SYNC_ADC:
      //case DIOC_INIT_SYNC_DAC:
      case DIOC_SETUP_DDS:
      //case        //DIOC_START:
      //case        //DIOC_STOP:
      //case        //DIOC_LOAD_BIOS_USB:
      //case        //DIOC_SEND_COMMAND:
      case DIOC_RESET_PIPE1:
      case DIOC_RESET_PIPE3:
      case DIOC_ABORT_PIPE1:
      case DIOC_ABORT_PIPE3 : return CTL_CODE(FILE_DEVICE_UNKNOWN, cmd, METHOD_OUT_DIRECT, FILE_ANY_ACCESS);

         // METHOD_IN_DIRECT
      case DIOC_SETEVENT :
      case DIOC_OUTP :
      case DIOC_OUTM :
      case DIOC_PUT_DATA_A :
      case DIOC_PUT_DM_A :
      case DIOC_PUT_PM_A :
      case DIOC_WRITE_FLASH_WORD :
      case DIOC_ENABLE_FLASH_WRITE :
      case DIOC_SETEVENT_1 :
      case DIOC_SETEVENT_2 :
      case DIOC_SETEVENT_3 :
      case DIOC_SETEVENT_4 :
      case DIOC_SETEVENT_5 :
      case DIOC_LOAD_BIOS :
      case DIOC_TTL_OUT :
      case DIOC_TTL_CFG :
      case DIOC_DAC_OUT :
      case DIOC_RESET_PLX :
      case DIOC_SEND_BIOS :
      case DIOC_ADC_CFG :
      case DIOC_DAC_CFG : return CTL_CODE(FILE_DEVICE_UNKNOWN, cmd, METHOD_IN_DIRECT, FILE_ANY_ACCESS);

      default : return (ULONG)-1;
      }
   }
   
private:
	OVERLAPPED ov[20];
   CRITICAL_SECTION cs_ref;
};


SAL::SAL() : d(new sySAL) {}

SAL::~SAL() { delete d; }

void SAL::LSetLastError(uint32_t Err) { d->sySetLastError(Err); }
uint32_t SAL::LGetLastError(void) { return d->syGetLastError(); }

void * SAL::LCreateFile(int slot) { return d->syCreateFile(slot); }
int    SAL::LCloseHandle(HANDLE hDevice) { return d->syCloseHandle(hDevice); }
uint32_t  SAL::LDeviceIoControl(void * hDevice,
                                uint32_t dwIoControlCode,
                                void * lpInBuffer,
                                uint32_t nInBufferSize,
                                void * lpOutBuffer,
                                uint32_t nOutBufferSize,
                                uint32_t * lpBytesReturned,
                                uint32_t StreamId)
{
	return d->syDeviceIoControl(hDevice, dwIoControlCode,lpInBuffer,nInBufferSize,
                               lpOutBuffer,nOutBufferSize,(PULONG)lpBytesReturned,StreamId);
}

uint32_t SAL::LWaitOverlapped(void * hDevice, uint32_t StreamId) // ждет завершения overlapped операции
{
   return d->syWaitOverlapped(hDevice, StreamId); // ждет завершения overlapped операции
}

void SAL::atomic_inc(atomic_t *v) { d->sy_atomic_inc(v); };
void SAL::atomic_dec(atomic_t *v) { d->sy_atomic_dec(v); };

void SAL::LUnmapMemory(void * lpAddr, int32_t Size) { d->syUnmapMemory(lpAddr, Size); }
void * SAL::LMapMemory(void * hDevice, void * lpAddr, int32_t Size, int32_t off) { return d->syMapMemory(hDevice, lpAddr, Size, off); }

void * SAL::LVirtualAlloc(size_t size) { return d->syVirtualAlloc(size); }
int SAL::LVirtualFree(void * addr) { return d->syVirtualFree(addr); }
int SAL::LIsWin32() { return d->syIsWin32(); }
uint32_t SAL::CTL(uint32_t cmd) { return d->syDIOC(cmd); }

