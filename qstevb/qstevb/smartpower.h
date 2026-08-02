//V1.2
#ifndef _SMART_POWER_H_
#define _SMART_POWER_H_

#define SMART_POWER_API __declspec(dllimport)

#pragma comment(lib,"SmartPower.lib")

#ifdef __cplusplus
extern "C" {
#endif

SMART_POWER_API int SmartPower_DeviceNums();
SMART_POWER_API int SmartPower_DeviceNums_Ext(wchar_t wSN_Buf2[50][30]);
SMART_POWER_API bool SmartPower_DeviceOpen(int iIndex);
SMART_POWER_API bool SmartPower_DeviceOpen_Ext(int iIndex,wchar_t wSN_Buf[30]);
SMART_POWER_API bool SmartPower_DeviceClose();

SMART_POWER_API bool SmartPower_OutputOn(double fVoltage,double fCurrent);
SMART_POWER_API bool SmartPower_OutputOn_Ext(double fVoltage,double fCurrent, double fOC_Time,double fOV_AddVoltage,double fOV_Time);

SMART_POWER_API bool SmartPower_OutputUpdate(double fVoltage,double fCurrent);
SMART_POWER_API bool SmartPower_OutputUpdate_Ext(double fVoltage,double fCurrent, double fOC_Time,double fOV_AddVoltage,double fOV_Time);

SMART_POWER_API bool SmartPower_GetStatus(double fBuff_ValueRead[4]);
SMART_POWER_API bool SmartPower_GetStatus_Ext(double fBuff_ValueRead[4],int bBuff_StatusRead[4]);

SMART_POWER_API bool SmartPower_OutputOff(int iOutput_OffRes);

#ifdef __cplusplus
}
#endif

//------------------------------------------------------------------------------
#endif //_SMART_POWER_H_