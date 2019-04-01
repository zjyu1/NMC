#include "NMC.h"
#include "stdafx.h"
#include "NmcHardware.h"

static NmcHardareRes hardware;

int HardwareOpen()
{
	int err = 0;
	err = hardware.Open();
	return err;
}

int HardwareClose()
{
	int err = 0;
	err = hardware.Close();
	return err;
}

bool HardwareIsOpen()
{
	bool flag = false;
	flag = hardware.IsOpen();
	return flag;
}

//内置编码器，因此需要重写
bool HardwareGoZero()
{
	bool flag = false;
	flag = hardware.GoZero();
	return flag;
}

bool HardwareGo(int axis, double range, double speed)
{
	bool flag = false;
	flag = hardware.Go(axis, range, speed);
	return flag;
}

bool HardwareBGo(int axis, double range, double speed)
{
	bool flag = false;
	flag = hardware.BGo(axis, range, speed);
	return flag;
}

bool HardwareContinue()
{
	bool flag = false;
	flag = hardware.Continue();
	return flag;
}

//R轴重写
int HardwareGetCurPos(int axis, double* pos)
{
	int err = 0;
	double val = 0;
	err = hardware.GetCurPos(axis, &val);
	*pos = val;
	return err;
}

int HardwareCurVeloc(int axis, double* v)
{
	int err = 0;
	double val = 0;
	err = hardware.GetCurVeloc(axis, &val);
	*v = val;
	return err;
}
//检测是否触碰限位开关
int HardwareChklimit()
{
	int err = 0;
	err = hardware.Chklimit();
	return err;
}

bool HardwareIsComplete()
{
	bool flag = false;
	flag = hardware.IsComplete();
	return flag;
}

bool HardwareWaitComplete(int ms)
{
	bool flag = false;
	flag = hardware.WaitComplete(ms);
	return flag;
}

void HardwareEHalt()
{
	hardware.EHalt();
}

void HardwareEKill()
{
	hardware.EKill();
}

void HardwareEStop()
{
	hardware.EStop();
}

void HardwareReset(int axis1, int axis2 , int axis3 , int axis4 )
{
	hardware.reset(axis1,axis2,axis3,axis4);

}

//实际上没有用
void HardwareSigStart2Hardware()
{
//	hardware.SigStart2Hardware();
}

//实际上没有用
void HardwareSigStop2Hardware()
{
//	hardware.SigStop2Hardware();
}

//实际上没有用
bool HardwareBGoPlanar(int axis1, double range1, int axis2, double range2, double speed, double mmStep)
{
	bool flag = false;
	//	flag = hardware.BGoPlanar(axis1, range1, axis2, range2, speed, mmStep);
	return flag;
}

//实际上没有用
bool HardwareGoThree(int axis1, double range1, double speed1, int axis2, double range2, double speed2, int axis3, double range3, double speed3)
{
	bool flag = false;
	//flag = hardware.Go(axis1, range1, speed1, axis2, range2, speed2, axis3, range3, speed3);  //原本便被注释掉
	return flag;
}

//实际上没有用
bool HardwareGoPlanar(int axis1, double range1, int axis2, double range2, double speed, double mmStep)
{
	bool flag = false;
	//	flag = hardware.GoPlanar(axis1, range1, axis2, range2, speed, mmStep);
	return flag;
}