
#ifndef NMC_HAERDWARE_CPLUSPLUS
#define NMC_HAERDWARE_CPLUSPLUS

using namespace std;

#include <string>
#include <fstream>
#include "NiDataType.h"
#include <thread>
#include "BlockingMotionQueue.h"
#include <windows.h>
#include<mutex>


class NmcHardareRes
{

private:
	//index
	unsigned char boardId;

	string boardName;


private:
	enum Mode
	{
		Line = 0,
		Planar = 1,
		Space=2
	}MoveMode;

private:
	int flag;
	std::mutex mut;

private:	
	bool isOpen;

	bool isPause;     //flag of Pause;

	bool isMonitor;   //monitor the limit;

	bool isTrigger;   //flag of trigger;

	unsigned int accRate; //acceleration rate

	double curPos[5];  //4 axis, [0] is reserved
	
	double curVeloc[5]; //current velocity mm/s,[0] is reserved
	
	double pltMotionRange[5]; //scan platform's motion range

	double outputModlus[5]; //output modulus, unit is mm

	double relativeZeroPt[5]; //relative zero point pos

	int maxWaitTime; //ms

	bool isQuit;   // thread is over or not

	bool isrunning; //thread is running or not

	bool iscomplete; //motions are completed or not

private: //LOG
	//log file name
	string logFileName;

	//log out file output stream
	ofstream  errLog;

private:
	MotionQueue<Motion*> Queue;
	MotionQueue<Motion*> tmpQueue;

private:
	int tmpaxis;
	double tmprange;
	double tmpspeed;
	double startPos;

private:
	int limitflag;

public:
	static DWORD WINAPI thread_func(LPVOID LPARAM);			//the motion thread;
	static DWORD WINAPI thread_Monitor(LPVOID LPARAM);		//the Monitor thread;

public:
	NmcHardareRes();
	~NmcHardareRes();

public:
	//open close
	bool Open();
	bool Close();
	bool IsOpen();

	
	//motion control
	bool GoZero();   //go zero point
	//void _nimcDisplayError(i32 errorCode, u16 commandID, u16 resourceID);  //Error log

	bool Go(int axis, double range, double speed);  //one axis go

	bool BGo(int axis, double range, double speed);
	//bool BGoLine(int axis, double range, double speed);
	//bool Go(int axis1, double range1, double speed1, int axis2, double range2, double speed2, int axis3, double range3, double speed3); //three axis motion

	bool Continue();

	//bool BScanUsingModulusBreakPoint(int axis,double targetPos,double velocity,double modulus);

	bool GoPlanar(int axis1, double range1, int axis2, double range2, double speed, double mmStep); //plane scan

	bool BGoPlanar(int axis1, double range1, int axis2, double range2, double speed, double mmStep); //plane scan

	//get current state
	bool     GetCurPos(int axis, double* pos); //mm
	bool     GetCurVeloc(int axis, double* v); //mm/s


	//check limit switch
	bool Chklimit();

	//Query is moving complete?
	bool IsComplete();
	bool WaitComplete(int ms);

	bool IsOver();

	void  SetMaxWaitTime(int ms);

	void EHalt();   //emergy stop
	void EKill();
	void EStop();

	//
	bool SigStart2Hardware();   //signal start sigs to testing board through RTSI line
	bool SigStop2Hardware();    //stop signals

private:   //NI 7334 interface	
	int  NI_7334Init(); //init NI-7334 stepper controller

	bool NI_7334IsComplete(); //see if NI-7334 si motion over
	bool NI_WaitZero(int ms);
	
	bool ChkCsrIsOk();  //check csr register of  NI-7334 is Ok?
	
	void Stop(unsigned short type);  //stop motion
	void Halt();  //halt move
	void Kill();  //kill move
	void Stop();  //stop move

	bool Start(int axis); //start motion
	bool Start(int axis1, int axis2, int axis3); //start motion two axis
	bool SetGo(int axis, double range, double speed);
	
	

private:   //tools function
	int		ChkAxis(int axis);
	int     LoadMotionPara(int axis);

	double	GetMaxVeloc(int axis);
	double	GetMinVeloc(int axis);
	bool	ChkVelocIsValid(int axis, double veloc); 

	//velocity, unit is mm/s
	double Steps2mm(int axis, int steps); //change steps/s to mm/s
	int Mm2Steps(int axis, double mmVeloc); //change mm/s to steps/s for speed & range
	double	CalVeloc(int axis, int steps);  //cal steps/s to mm/s, used by Steps2mm()

	//
	int ChkRangeIsValid(int axis, double range);
	bool ChkHomeStatus();


private:  //windows function
	//time
	void WaitTime(int ms); //unit is micros second

	
//ERR PRINTF
	//log printf to logFileName according stationary format
    void LogPrintf(long line, char* strfunc,char* strErr);

public:
	//reset
	void reset(int axis1 , int axis2 , int axis3, int axis4 );

};

#endif














