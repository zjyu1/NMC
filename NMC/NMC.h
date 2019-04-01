#ifndef NMC_NI
#define NMC_NI

#include<iostream>
//#include "stdafx.h"
//#include "NmcHardware.h"

#define LX_DLL_CLASS_EXPORTS //export

#ifndef LX_DLL_CLASS_EXPORTS
    #define LX_DLL_CLASS __declspec(dllexport)
#else
    #define LX_DLL_CLASS __declspec(dllimport)
#endif

using namespace std;

extern "C" LX_DLL_CLASS int HardwareOpen();

extern "C" LX_DLL_CLASS int HardwareClose();

extern "C" LX_DLL_CLASS bool HardwareIsOpen();

extern "C" LX_DLL_CLASS bool HardwareGoZero();

extern "C" LX_DLL_CLASS bool HardwareGo(int axis, double range, double speed);

extern "C" LX_DLL_CLASS bool HardwareBGo(int axis, double range, double speed);


extern "C" LX_DLL_CLASS bool HardwareContinue();

extern "C" LX_DLL_CLASS bool HardwareGoThree(int axis1, double range1, double speed1, int axis2, double range2, double speed2, int axis3, double range3, double speed3);

extern "C" LX_DLL_CLASS bool HardwareGoPlanar(int axis1, double range1, int axis2, double range2, double speed, double mmStep);

extern "C" LX_DLL_CLASS bool HardwareBGoPlanar(int axis1, double range1, int axis2, double range2, double speed, double mmStep);

extern "C" LX_DLL_CLASS int HardwareGetCurPos(int axis, double* pos);

extern "C" LX_DLL_CLASS int HardwareCurVeloc(int axis, double* v);

extern "C" LX_DLL_CLASS bool HardwareIsComplete();

extern "C" LX_DLL_CLASS bool HardwareWaitComplete(int ms);

extern "C" LX_DLL_CLASS void HardwareEHalt();

extern "C" LX_DLL_CLASS void HardwareEKill();

extern "C" LX_DLL_CLASS void HardwareEStop();

extern "C" LX_DLL_CLASS void HardwareSigStart2Hardware();

extern "C" LX_DLL_CLASS void HardwareSigStop2Hardware();

extern "C" LX_DLL_CLASS void HardwareReset(int axis1, int axis2, int axis3, int axis4);

extern "C" LX_DLL_CLASS void HardwareReset(int axis1, int axis2, int axis3, int axis4); 

extern "C" LX_DLL_CLASS int HardwareChklimit();

extern "C" LX_DLL_CLASS HANDLE OpenDriver(TCHAR * port);

extern "C" LX_DLL_CLASS bool SendData(HANDLE fd, BYTE *data, DWORD dwDataLen);

extern "C" LX_DLL_CLASS bool ReceiveData(HANDLE fd, BYTE *data);

extern "C" LX_DLL_CLASS void ClosePort(HANDLE fd);

#endif