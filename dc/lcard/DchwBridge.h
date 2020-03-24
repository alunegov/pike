#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// ��������� ��������� � ��������� UsbE_dll_v2.dll (��. uLCardUSBv2SDK.pas)

#ifdef DC_LCARD_DLL
#   ifdef DC_LCARD_EXPORTS
#       define DC_LCARD_API __declspec(dllexport)
#   else
#       define DC_LCARD_API __declspec(dllimport)
#   endif
#else
#   define DC_LCARD_API
#endif

#define DllProtoVer 4

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)

struct ReadModulData
{
    // ����� ������ ���
    uint32_t dThreadA;

    // ������������� ������ ���
    void* hThreadA;

    // ���� ����������������� ���������� �����
    bool AbortBool;
};

struct UsbLModul
{
    /// <summary>
    /// ��� ������: 1 - 440, 2 - 140(-�), 3 - 2010
    /// </summary>
    uint8_t ModuleType;

    /// <summary>
    /// ����: ������������� ������ ��� ������ ���
    /// </summary>
    bool ModulState;

    /// <summary>
    /// ��������� ��� ������ � ������� (���� �� ILE440, ILE140 ��� ILE2010)
    /// </summary>
    void* Modul;

    /// <summary>
    /// �������� ������ ������ �� ���� USB: 0 - USB 1.1, 1 - USB 2.0 
    /// </summary>
    /// <remarks>
    /// �������� ��������� � ����������� USB11_LUSBAPI � USB20_LUSBAPI ������ Lusbapi
    /// </remarks>
    uint8_t UsbSpeed;

    /// <summary>
    ///
    /// </summary>
    ReadModulData RMD;

    /// <summary>
    /// ������� ������ (��������� ������)
    /// </summary>
    /// <summary>
    /// �������� - MODULE_DESCRIPTION_E���.Module.Revision
    /// </summary>
    uint8_t Revision;
};

struct OptionsRead
{
    void* BreakEvent;
    void* DacFinishWrite;
    void* AdcFinishRead;
    void* cbBuffFilledEvent;
    uint32_t cbBuffCount;

    /// <summary>
    /// ����� ������� �������� �� ������� ������ �� �����
    /// </summary>
    uint8_t ReadRequestsCount;
};

struct ParamRead
{
    // ����� ������, ������������ � ������
    uint8_t State;

    // ������� ������ � ���
    double_t RealChRate;

    // ���������� �������
    uint8_t NumbCh;

    // ����������� ������� �������
    uint16_t ChTab[128];

    // ���������� ����������� ������� ��� ������
    uint32_t NumbChStep;

    // ��������� �� ����� ������
    void* pBuffer;

    // ����� ����������� ������
    uint32_t BufferCount;

    // ������������ ����������� �����
    bool CicleBuffer;

    // ���� ������������� �����
    bool UseFile;

    // ��� ����� ��� �������� ������
    char* FileName;

    // ���������� ���� � ������ �������
    uint8_t LengthR;

    // ��������� �� ����� ��������� ������
    void* pReserv;
    
    // FIFO ������� ���������� ������
    void* Queue;
};

struct Point
{
    uint32_t X;
    uint32_t Y;
};

#pragma pack(pop)
  
DC_LCARD_API
uint16_t __stdcall GetDllProtoVer(void);

//DC_LCARD_API
//void __stdcall SetLang(const char* lang);

DC_LCARD_API
uint8_t __stdcall InitDevice(uint8_t ModulType, uint8_t NumberSlot, struct UsbLModul** UsbLModul);

DC_LCARD_API
uint8_t __stdcall DestroyDevice(struct UsbLModul** UsbLModul);

DC_LCARD_API
const char* __stdcall GetErrorText2(uint8_t Number);

DC_LCARD_API
struct Point __stdcall GetRate(const struct UsbLModul* aModule, double_t aChannelRate, uint8_t* aChannelCount);

DC_LCARD_API
uint8_t __stdcall Break_Adc(struct UsbLModul* pULM, struct OptionsRead* pOR);

DC_LCARD_API
uint8_t __stdcall Slow_Adc(struct UsbLModul* pULM, struct ParamRead* AdcParam, struct OptionsRead* pOR);

DC_LCARD_API
uint8_t __stdcall Fast_Adc(struct UsbLModul* pULM, struct ParamRead* AdcParam, struct OptionsRead* pOR);

DC_LCARD_API
uint8_t __stdcall SetTTL_Line(struct UsbLModul* pULM, uint16_t Line);

DC_LCARD_API
uint8_t __stdcall GetTTL_Line(struct UsbLModul* pULM, uint16_t* TTLLine);

DC_LCARD_API
void __stdcall ActiveTTL_Line(struct UsbLModul* pULM, bool Active);

DC_LCARD_API
bool __stdcall DAC_Exists(struct UsbLModul* aULM);

DC_LCARD_API
bool __stdcall DAC_Sample(struct UsbLModul* aULM, uint8_t aDACChannel, int16_t aValue);

#ifdef __cplusplus
}
#endif
