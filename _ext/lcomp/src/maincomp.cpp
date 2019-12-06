
#include "../include/stubs.h"
#include "../include/salayer.h"

#include "../include/ioctl.h"
#include "../include/ifc_ldev.h"
#include "../include/ldevbase.h"

#include "../include/plx.h"
#include "../include/791.h"
#include "../include/e440.h"
#include "../include/e140.h"
#include "../include/e2010.h"
#include "../include/e154.h"
#include "../include/e310.h"

#include <stdio.h>

// add class for new board here.
// main function of dll
extern "C" LUnknown* CreateInstance(ULONG Slot)
{
   LUnknown* pI;
//   char buf[128];

   SLOT_PAR sl;
   LDaqBoard *pL = new LDaqBoard(Slot);
   if(pL==NULL) return NULL;

   void * hVxd = pL->OpenLDevice();
   if(hVxd==INVALID_HANDLE_VALUE) { delete pL; return NULL; }

   pL->GetSlotParam(&sl);

   switch(sl.BoardType)
   {
   case PCIA:
   case PCIB:
   case PCIC:
      {
         pI = static_cast<IDaqLDevice*>(new DaqL780(Slot));
         pI->AddRef();
      } break;

   case L791:
      {
         pI = static_cast<IDaqLDevice*>(new DaqL791(Slot));
         pI->AddRef();
      } break;

   case E440:
      {
         pI = static_cast<IDaqLDevice*>(new DaqE440(Slot));
         pI->AddRef();
      } break;

   case E140:
	case 90:
      {
         pI = static_cast<IDaqLDevice*>(new DaqE140(Slot));
         pI->AddRef();
      } break;

   case E2010B:
   case E2010:
      {
         pI = static_cast<IDaqLDevice*>(new DaqE2010(Slot));
         pI->AddRef();
      } break;

   case E154:
      {
         pI = static_cast<IDaqLDevice*>(new DaqE154(Slot));
         pI->AddRef();
      } break;

   case E310:
      {
         pI = static_cast<IDaqLDevice*>(new DaqE310(Slot));
         pI->AddRef();
      } break;

   default: { pI=NULL; pL->sal->LSetLastError(L_NOTSUPPORTED); }

   }

   pL->CloseLDevice();
   delete pL;

   return pI;
}

