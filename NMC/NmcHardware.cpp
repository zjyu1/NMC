#include "stdafx.h"
#include "NmcHardware.h"
#include "NiDataType.h"
#include "flexmotn.h" //Ni motion driver
#include "Windows.h"

#ifndef USCOMM_ERR_PRINTF
#define USCOMM_ERR_PRINTF
#define ErrPrintf(str_err_info) LogPrintf(__LINE__, __FUNCTION__, #str_err_info)
#endif 

#pragma comment(lib, "FlexMS32.lib")

DWORD WINAPI NmcHardareRes::thread_func(LPVOID LPARAM)
{
	//def
	Motion* item;
	int axis;
	double range;
	double speed;
	bool isOk;

	//init
	NmcHardareRes* pNmc;
	pNmc = (NmcHardareRes*)LPARAM;

	while (!pNmc->isQuit)
	{	
			while (pNmc->Queue.Take(item))    //pop one element
			{
				pNmc->tmpaxis = item->getAxis();
				pNmc->tmprange = item->getRange();
				pNmc->tmpspeed = item->getSpeed();

				axis = item->getAxis();
				range = item->getRange();
				speed = item->getSpeed();

				pNmc->GetCurPos(axis, &(pNmc->startPos));

				while(!pNmc->Go(axis, range, speed)) ;	//until the moving is completed

				isOk = pNmc->IsComplete();
				
				//下面这段程序需要更改，我觉得可以删除
				if (pNmc->Queue.isEmpty() && isOk)       
				{
					pNmc->iscomplete = true;
				}
				else
				{
					//报错，打印错误信息
				}
			}
	}
	return 0;
		
}

NmcHardareRes::NmcHardareRes()
{
	this->boardId = 1;
	this->boardName = "NI 7330 Step Controller.";

	this->isOpen = false;
	this->isrunning = false;
	this->iscomplete = false;
	this->isPause = false;
	this->isMonitor = true;
	this->isTrigger = false;

	this->flag = 0;
	this->accRate = SPL_DFT_ACC; //about 4r/s, 10000 steps

								 //pos
	this->curPos[0] = 0; //[0] is reserved
	this->curPos[SPL_X_AXIS] = 0;
	this->curPos[SPL_Y_AXIS] = 0;
	this->curPos[SPL_Z_AXIS] = 0;
	this->curPos[SPL_R_AXIS] = 0;

	//velocity
	this->curVeloc[0] = 0.0;
	this->curVeloc[SPL_X_AXIS] = SPL_DFT_VELOC;
	this->curVeloc[SPL_Y_AXIS] = SPL_DFT_VELOC;
	this->curVeloc[SPL_Z_AXIS] = SPL_DFT_VELOC;
	this->curVeloc[SPL_R_AXIS] = SPL_DFT_VELOC;

	//platform motion range
	//行程需要改改
	this->pltMotionRange[0] = 0.0;
	this->pltMotionRange[SPL_X_AXIS] = SPL_X_MOTION_RANGE - SPL_RELATIVE_X_OFFSET - SPL_RANGE_RSV;   //265mm
	this->pltMotionRange[SPL_Y_AXIS] = SPL_Y_MOTION_RANGE - SPL_RELATIVE_Y_OFFSET - SPL_RANGE_RSV;   //265mm
	this->pltMotionRange[SPL_Z_AXIS] = SPL_Z_MOTION_RANGE - SPL_RELATIVE_Z_OFFSET - SPL_RANGE_RSV;   //235 mm
	this->pltMotionRange[SPL_R_AXIS] = SPL_R_MOTION_RANGE - SPL_RELATIVE_R_OFFSET - SPL_RANGE_RSV;   //应该为1圈

	//default 
	this->outputModlus[0] = 0.0;
	this->outputModlus[SPL_X_AXIS] = SPL_DFT_OUTPUT_MODLUS;
	this->outputModlus[SPL_Y_AXIS] = SPL_DFT_OUTPUT_MODLUS;
	this->outputModlus[SPL_Z_AXIS] = SPL_DFT_OUTPUT_MODLUS;
	this->outputModlus[SPL_R_AXIS] = SPL_DFT_OUTPUT_MODLUS;

	//relative zero point, because the zero switch effect
	//由于限位开关的影响，所以有个初始偏置
	this->relativeZeroPt[0] = 0;
	this->relativeZeroPt[SPL_X_AXIS] = SPL_RELATIVE_X_OFFSET;
	this->relativeZeroPt[SPL_Y_AXIS] = SPL_RELATIVE_Y_OFFSET;
	this->relativeZeroPt[SPL_Z_AXIS] = SPL_RELATIVE_Z_OFFSET;
	this->relativeZeroPt[SPL_R_AXIS] = SPL_RELATIVE_R_OFFSET;

	this->maxWaitTime = SPL_1_SECOND * 60; //one minute
										   //log file name
	this->logFileName = "NmcHardareResErrorLog.txt";

	//open log file, has bug?
	this->errLog.open("D:\\logFileName.c_str()", ios::trunc | ios::out); //if exsist, delete and new one file		
	
} //end of NmcHardareRes()

NmcHardareRes::~NmcHardareRes()
{
	// if open close the device
	if (this->isOpen == true)
	{
		this->Close();
	}
	//close logfile
	this->errLog.close();

} //end of ~NmcHardareRes(...)

bool NmcHardareRes::Open()
{
	ErrPrintf("NMC procedure start!");
	int err;
	err = SPL_OK;
	err = this->NI_7334Init();
	if (err == SPL_OK)
	{
		this->isOpen = true;
		isQuit = false;		
		while(!this->Queue.Empty());   //until the queue is empty
		while (!this->tmpQueue.Empty()); //until the tmpQueue is empty

		HANDLE hThread = CreateThread(
			NULL,              // default security
			0,                 // default stack size
			this->thread_func,  // name of the thread function
			(LPVOID) this,              // no thread parameters
			0,                 // default startup flags
			NULL);
		return true;
	}
	else
	{
		return false;
	}
}

//这个函数用于停止系统运动，当系统依旧在运动时，超过最大等待时间后强制停止
bool NmcHardareRes::Close()
{
	bool isOver = false;

	if (this->isOpen == false)
	{
		isQuit = true;
		return true;
	}

	this->isOpen = false;

	isOver = isOver = WaitComplete(this->maxWaitTime);

	if (isOver == false)
	{
		this->Kill();
	}
	while (!this->Queue.Empty());
	isQuit = true;
	return true;
}

bool NmcHardareRes::IsOpen()
{
	return this->isOpen;
}

void NmcHardareRes::LogPrintf(long line, char* strfunc, char* strErr)
{
	char s[256];

	sprintf_s(s, "%d", line);

	this->errLog << "Date: " << __DATE__ << ".\n";

	this->errLog << "Time: " << __TIME__ << ".\n";

	this->errLog << "Line: " << s << ".\n";

	this->errLog << strfunc << "(...),  " << strErr << ",  pls chk!.\n";

	this->errLog << "\n";

	this->errLog << flush;

} //end of LogPrintf(...

void NmcHardareRes::WaitTime(int ms)
{
	Sleep(ms);
}

//return max velocity, unit is mm/s
double NmcHardareRes::GetMaxVeloc(int axis)
{
	int err;
	double veloc;

	err = ChkAxis(axis);

	if (err != SPL_OK)
	{
		ErrPrintf("Axis is not Valid, Pls chk!");
		return SPL_FAULT_VELOCITY;
	}
	//send max HZ,return max velocity
	veloc = this->CalVeloc(axis, SPL_MOTOR_MAX_HZ);

	return veloc;
}

//return min velocity, unit is mm/s
double NmcHardareRes::GetMinVeloc(int axis)
{
	int err;
	double veloc;

	err = 0;
	veloc = 0;

	err = ChkAxis(axis);
	if (err != SPL_OK)
	{
		return SPL_FAULT_VELOCITY;
	}

	veloc = this->CalVeloc(axis, 1);

	return veloc;
}

//chk if veloc between min and max, unit is mm/s
//判断轴的速度是否在有效范围内判断轴速度的有效性
bool NmcHardareRes::ChkVelocIsValid(int axis, double veloc)
{
	int err;
	double min;
	double max;

	err = ChkAxis(axis);

	if (err != SPL_OK)
	{
		return false;
	}

	min = GetMinVeloc(axis);
	max = GetMaxVeloc(axis);

	if (veloc >= min && veloc <= max)
	{
		return true;
	}
	else
	{
		return false;
	}

}

//cal steps/s to mm/s 
double NmcHardareRes::CalVeloc(int axis, int steps)
{
	//defination
	int err;
	double veloc;
	double rps; //r/s

	//init
	err = 0;
	veloc = 0;
	rps = 0;

	//check
	err = ChkAxis(axis);
	if (err != SPL_OK)
	{
		return SPL_FAULT_VELOCITY;
	}

	/*
	if (steps < 0)
	{
		return SPL_FAULT_VELOCITY;
	}
	*/

	//main
	rps = ((double)steps) / SPL_ENCODER_RESLUTION;

	switch (axis)
	{
	case SPL_X_AXIS:
	{
		veloc = (double)SPL_X_AXIS_ROTOR_PITCH*rps;
	}
	break;

	case SPL_Y_AXIS:
	{
		veloc = (double)SPL_Y_AXIS_ROTOR_PITCH*rps;
	}
	break;

	case SPL_Z_AXIS:
	{
		veloc = (double)SPL_Z_AXIS_ROTOR_PITCH*rps;
	}
	break;

	case SPL_R_AXIS:
	{
		veloc = (double)SPL_R_AXIS_ROTOR_PITCH*rps;
	}
	break;

	default:
		break;
	}
	return veloc;
}

//chk axis nume is valid
//通过判断轴的取值范围是否为1-4判断有效性
int  NmcHardareRes::ChkAxis(int axis)
{
	int err = SPL_OK;

	if (axis < SPL_X_AXIS || axis > SPL_R_AXIS)
	{
		err = SPL_ERR;
		ErrPrintf("Axis is not Valid!");
	}
	return err;
}

//chk motion para is loaded
int  NmcHardareRes::LoadMotionPara(int axis)
{
	int err = SPL_OK;
	// Set the acceleration for the move (in counts/sec^2)
	err = flex_load_acceleration(this->boardId, axis, NIMC_ACCELERATION, this->accRate, SPL_DIRECT);

	// Set the deceleration for the move (in counts/sec^2)
	err |= flex_load_acceleration(this->boardId, axis, NIMC_DECELERATION, this->accRate, SPL_DIRECT);

	// Set the jerk - scurve time (in sample periods)
	err |= flex_load_scurve_time(this->boardId, axis, this->accRate / 10, 0xFF);

	// Set the operation mode
	err = flex_set_op_mode(this->boardId, axis, NIMC_RELATIVE_POSITION);
	if (err != 0)
	{
		err = SPL_ERR;
		ErrPrintf("Para is not Valid!");
	}

	return err;
}

//change steps/s to mm/s for idividual axis
double NmcHardareRes::Steps2mm(int axis, int steps)
{
	//Defination
	int err;
	double veloc;

	//Init
	err = 0;
	veloc = 0;

	//Check Inputs
	err = ChkAxis(axis);
	if (err != SPL_OK)
	{
		err = SPL_ERR;
		ErrPrintf("axis is not valid!");
		return SPL_FAULT_VELOCITY;
	}
	//main
	veloc = this->CalVeloc(axis, steps);
	//return
	return veloc;
}


int NmcHardareRes::Mm2Steps(int axis, double mmVeloc)
{
	//defination
	int err;

	double r; //round
	int steps;

	//Init
	r = 0.0;
	steps = 0;

	//Check inputs
	err = ChkAxis(axis);
	if (err != SPL_OK)
	{
		err = SPL_ERR;
		ErrPrintf("axis is not valid!");
		return err;
	}

	switch (axis)
	{
	case SPL_X_AXIS:
	{
		r = (double)mmVeloc / SPL_X_AXIS_ROTOR_PITCH;
	}
	break;

	case SPL_Y_AXIS:
	{
		r = (double)mmVeloc / SPL_Y_AXIS_ROTOR_PITCH;
	}
	break;

	case SPL_Z_AXIS:
	{
		r = (double)mmVeloc / SPL_Z_AXIS_ROTOR_PITCH;
	}
	break;

	case SPL_R_AXIS:
	{
		r = (double)mmVeloc / SPL_R_AXIS_ROTOR_PITCH;
	}
	break;

	default:
		break;
	}

	steps = (int)(r*SPL_ENCODER_RESLUTION); //round * lines of encoder/r --->steps
											//return
	return steps;
}

//chk the range is beyond the scan platform's limit, range can be + or -, unit is mm
//通过检查轴当前位置与目标位置的和是否在 规定范围内，判断有效性
int NmcHardareRes::ChkRangeIsValid(int axis, double range)
{
	//defination
	int err;
	double cur;

	//init
	err = 0;
	cur = 0;

	//check inputs
	err = ChkAxis(axis);
	if (err != SPL_OK)
	{
		return false;
	}

	//main
	err = GetCurPos(axis, &cur);
	if (range >= (double)0.0)
	{
		if ((cur + range) > this->pltMotionRange[axis])
		{
			return false;
		}
	}
	else
	{
		if ((cur - range) < -this->pltMotionRange[axis])
		{
			return false;
		}
	}


	//return
	return true;

}

//不知道有什么用
bool NmcHardareRes::ChkHomeStatus()
{
	//defination
	int err;
	int remainder;
	u16 homeStatus;
	//init
	err = 0;
	remainder = -1;
	err = flex_read_home_input_status_rtn(this->boardId, &homeStatus);
	remainder = homeStatus % 2;
	switch (remainder)
	{
	case 0:
		Stop();
		break;
	case 1:
		Stop();
		break;
	}
	return true;
}

// Read the communication status register and check the modal errors
bool NmcHardareRes::ChkCsrIsOk()
{
	int err;
	bool isOk;
	unsigned short csr;

	isOk = true;
	err = 0;
	csr = 0;

	err = flex_read_csr_rtn((unsigned char)this->boardId, &csr); // Read the communication status register and check the modal errors
	if (err != SPL_OK || SPL_CSR_ERR(csr) != SPL_OK)
	{
		ErrPrintf("ChkCsr() error occured, pls chk!");
		isOk = false;
	}
	return isOk;
}

//Init the ni motion controller
int NmcHardareRes::NI_7334Init()
{
	//Defination
	int err;
	int i;

	unsigned char brdId;
	int axis;
	u16 axisStatus;			// Axis status  
	u16 moveComplete;
	u16 csr = 0;				// Communication status register

	//Init
	err = SPL_NOK;

	//Main
	brdId = this->boardId;

	if (this->isOpen == true)
	{
		ErrPrintf("NI_7334Init(), controller is open, pls chk close firstly!");
		return SPL_NOK;
	}

	flex_initialize_controller(brdId, "Default 7330 Settings"); //init

																//set acceration paras
	for (i = SPL_X_AXIS; i< SPL_R_AXIS; i++)
	{
		axis = (unsigned char)i;
		err = flex_load_acceleration(brdId, (u8)axis, NIMC_ACCELERATION, this->accRate, SPL_DIRECT);
		err |= flex_load_acceleration(brdId, (u8)axis, NIMC_DECELERATION, this->accRate, SPL_DIRECT);

		if (err != SPL_OK)
		{
			ErrPrintf("flex_load_acceleration() set acc rate error, pls chk!");
			return err;
		}
	}

	//set op mode

	for (i = SPL_X_AXIS; i <= SPL_R_AXIS; i++)
	{
		axis = (unsigned char)i;
		// Set the jerk - scurve time (in sample periods)
		err = flex_load_scurve_time(brdId, (u8)axis, 1000, SPL_DIRECT);
		//设置为相对模式*
		err |= flex_set_op_mode(brdId, (u8)axis, NIMC_RELATIVE_POSITION);

		if (err != SPL_OK)
		{
			ErrPrintf("flex_load_scurve_time() or flex_set_op_mode(), pls chk!");
			return err;
		}
	}

	do
	{
		axisStatus = 0;
		// Check the move complete status
		err = flex_check_move_complete_status(brdId, axis, 0, &moveComplete);
	 
		// Check the following error/axis off status for the axis
		err |= flex_read_axis_status_rtn(brdId, axis, &axisStatus);

		// Read the communication status register and check the modal errors
		err |= flex_read_csr_rtn(brdId, &csr);
		if (err != SPL_OK)
		{
			return err;
		}

		// Check the modal errors
		if (csr & NIMC_MODAL_ERROR_MSG)
		{
			err = csr & NIMC_MODAL_ERROR_MSG;
			if (err != SPL_OK)
			{
				return err;
			}
		}
	} while (!moveComplete && !(axisStatus & NIMC_FOLLOWING_ERROR_BIT) && !(axisStatus & NIMC_AXIS_OFF_BIT)); //Exit on move complete/following error/axis off
	
	return err;		// Exit the Application
}

bool NmcHardareRes::NI_7334IsComplete()
{
	//defination
	int i;
	unsigned char brdId;
	unsigned short axis;

	bool cmpt;
	int err;
	unsigned short moveComplete;
	unsigned short axisStatus;

	//init
	cmpt = true;
	err = SPL_OK;
	moveComplete = 0;

	if (this->isOpen == false)
	{
		return true;
	}

	brdId = this->boardId;

	//main
	for (i = SPL_X_AXIS; i <= SPL_R_AXIS; i++)
	{
		axis = (unsigned char)i;
		err = flex_check_move_complete_status(brdId, (u8)axis, 0, &moveComplete);  // Check the move complete status

		err |= flex_read_axis_status_rtn(brdId, (u8)axis, &axisStatus);  //Check the following error/axis off status for the axis	

		if (!moveComplete && !(axisStatus & NIMC_FOLLOWING_ERROR_BIT) && !(axisStatus & NIMC_AXIS_OFF_BIT)) //Exit on move complete/following error/axis off   
		{
			cmpt = false;
			break;
		}

		if (err != SPL_OK)
		{
			cmpt = false;
			ErrPrintf("flex_check_move_complete_status() or (), pls chk!");
			break;
		}
	}

	if (this->ChkCsrIsOk() != true)
	{
		ErrPrintf("ChkCsrIsOk() error occured, pls chk!");
		cmpt = false;
	}

	//return
	return cmpt;
}

//以下三个程序应该都是一样的
bool NmcHardareRes::IsComplete()
{
	return this->NI_7334IsComplete();
}

bool NmcHardareRes::IsOver()
{
	return this->iscomplete;
}

bool NmcHardareRes::WaitComplete(int ms)
{
	int i; //every 100ms check 
	bool isOver = false;
	int time = 0;

	if (this->IsComplete() == true)
	{
		return true;
	}
	else
	{
		if (ms == 0)
		{
			time = this->maxWaitTime;
		}
		else
		{
			time = ms;
		}
		for (i = 0; i<time / 100; i++)
		{
			this->WaitTime(100);
			if (this->IsComplete() == true)
			{
				isOver = true;
				break;
			}
		}
	}
	return isOver;
}

void  NmcHardareRes::SetMaxWaitTime(int ms)
{
	this->maxWaitTime = ms;
}

//stop motion with decellration
//以下三个程序是三种不同的停止方式
void NmcHardareRes::Stop()
{
	this->Queue.Empty();
	this->tmpQueue.Empty();

	this->isQuit = true;

	//this->isrunning = false;
	this->Stop(NIMC_DECEL_STOP);
}

//kill motion immeditely
void NmcHardareRes::Kill()
{
	this->Stop(NIMC_KILL_STOP);
}

//halt motion immeditely
void NmcHardareRes::Halt()
{
	//this->Queue.Empty();
	this->Stop(NIMC_HALT_STOP);
}

//inner function for Halt(), Stop(), and Kill()
void NmcHardareRes::Stop(unsigned short type)
{
	int err;

	err = 0;

	if (this->isOpen == false)
	{
		return;
	}
	err = flex_stop_motion(this->boardId, NIMC_AXIS_CTRL, type, SPL_3AXIS_VECTOR_SPACE);
	if (err != SPL_OK)
	{
		ErrPrintf("flex_stop_motion() error occured, can not stop,  pls chk!");
	}
}

//start motion

//start move one axis
bool NmcHardareRes::Start(int axis)
{
	int status = 0;
	bool isOver = false;
	unsigned short  bitMap = 0;

	if (this->isOpen == false)
	{
		ErrPrintf("fatal error, board is not open");
		return false;
	}

	//start move
	status = flex_start(this->boardId, axis, 0);
	if (status != SPL_OK)
	{
		ErrPrintf("flex_start_motion() error occured, can not start,  pls chk!");
		return false;
	}
	//wait motion over
	isOver = this->WaitComplete(this->maxWaitTime);
	if (isOver == false)
	{
		ErrPrintf("fatal error, can not wait motion over before start muti axis motion!");
		return false;
	}

	return true;
}

//start move two axis
bool NmcHardareRes::Start(int axis1, int axis2, int axis3)
{
	//defintion
	u8 axis;
	int err = 0;
	int status = 0;
	bool isOver = false;
	unsigned short  bitMap = 0;

	//init
	if (this->isOpen == false)
	{
		ErrPrintf("fatal error, board is not open");
		return false;
	}

	//check inputs
	err = ChkAxis(axis1);
	if (err != SPL_OK)
	{
		return false;
	}

	err = ChkAxis(axis2);
	if (err != SPL_OK)
	{
		return false;
	}

	if (axis1 == axis2)
	{
		ErrPrintf("fatal error, axis1 ==  axis2!");
		return false;
	}

	if (axis2 == axis3)
	{
		ErrPrintf("fatal error, axis2 ==  axis2!");
		return false;
	}

	//start move
	for (axis = 1; axis < 4; axis++)
	{
		status = flex_start(this->boardId, axis, 0);
		if (status != SPL_OK)
		{
			ErrPrintf("flex_start_motion() error occured, can not start,  pls chk!");
			return false;
		}
	}

	return true;
}
//move to zero positions, and wait until is moved done, if error, return false
bool NmcHardareRes::GoZero()
{
	//defination
	bool isOk;
	int err;
	double speed;
	int axis1;
	int axis2;
	int axis3;
	int axis4;
	double curpos1;
	double curpos2;
	double curpos3;
	double curpos4;

	//init
	isOk = true;
	err = 0;
	speed = 20;
	axis1 = 1;
	axis2 = 2;
	axis3 = 3;
	axis4 = 4;

	//get current position
	GetCurPos(axis1, &curpos1);
	GetCurPos(axis2, &curpos2);
	GetCurPos(axis3, &curpos3);
	GetCurPos(axis4, &curpos4);


		//check inputs
		if (this->IsOpen() == false)
		{
			ErrPrintf("fatal error, board is not open!");
			return false;
		}

		//main
		if (this->ChkCsrIsOk() == false)
		{
			ErrPrintf("fatal error, csr chk err,pls chk!");
			return false;
		}

		this->accRate = SPL_DFT_ACC;
		while (!this->Queue.Empty());  //empty the queue
		//this->isrunning = false;

		for (int i = SPL_X_AXIS; i < SPL_R_AXIS; i++)
		{
			err = this->LoadMotionPara(i);
			if (err != SPL_OK)
			{
				ErrPrintf("fatal error,load motion para err,pls chk!");
				return false;
			}
		}

		if (curpos1 != 0 || curpos2 != 0 && curpos3 != 0 && curpos4 != 0)
		{
			//find home 从此处更改
			isOk = BGo(axis1, -curpos1, speed);
			isOk &= BGo(axis2, -curpos2, speed);
			isOk &= BGo(axis3, -curpos3, speed);
			isOk &= BGo(axis4, -curpos4, speed);

			//get current position
			GetCurPos(axis1, &curpos1);
			GetCurPos(axis2, &curpos2);
			GetCurPos(axis3, &curpos3);
			GetCurPos(axis4, &curpos4);
		}
	

	return isOk;
}

bool  NmcHardareRes::NI_WaitZero(int ms)
{
	//defination
	bool isOk;
	int err;
	u16 isFinding;
	u16 isFind;

	//init
	isOk = true;
	err = SPL_OK;
	isFind = SPL_FALSE;
	isFinding = SPL_FALSE;

	//check inputs
	//none

	//main
	err = flex_wait_reference(this->boardId, NIMC_AXIS_CTRL, SPL_3AXIS_VECTOR_SPACE, (u32)ms, SPL_1_SECOND, &isFind);
	if (err != SPL_OK) //|| isFind != SPL_TRUE
	{
		ErrPrintf("fatal error, wait index error,pls chk!");
		return false;
	}

	err = flex_check_reference(this->boardId, NIMC_AXIS_CTRL, SPL_3AXIS_VECTOR_SPACE, &isFind, &isFinding);

	if (err != SPL_OK || isFind != SPL_TRUE || isFinding == SPL_TRUE)
	{
		ErrPrintf("fatal error, chk find home or index error,pls chk!");
		return false;
	}

	return true;
}

//bool NmcHardareRes::GoZero()
//{
//
//	u8 axis1 = 1;
//	u8 axis2 = 2; 
//	u8 axis3 = 3;
//	u8 boardID;							//Board Identification number
//							//Second Axis Number
//	u16 axisBitmap;					//The Bitmap of Axes
//	i32 err=-1;								//Return Status of functions
//	u16 csr;								//Communication Status Register
//	//u32 scanVar;						//scanf wants a u16 to put numeric values in.
//	i32 position;						//Current position of axis
//	u16 homeFound;						//If home was found on both axes
//	u16 indexFound;					//If index was found on both axes
//	u16 referenceFound;				//If the reference we are searching for is found
//	u8 i;									//Loop Variable
//	u16 axisIsFinding;				//If the axis is currently searching 
//
//	//Variables for modal error handling
//	u16 commandID;						//The commandID of the function
//	u16 resourceID;					//The resource ID
//	i32 errorCode;						//The error generated
//	bool isOver;
//
//	//Check if the board is at power up reset condition
//	err = ChkCsrIsOk();
//	//err = flex_read_csr_rtn((unsigned char)this->boardId, &csr);
//	if(err == SPL_NOK)
//	{
//		return false;
//	}
//			
//	axisBitmap = (1 << axis1) | (1 << axis2) | (1 << axis3);
//
//	for (i = 0; i < 2; i++) {		
//			//First Find Home
//			if (i == 0) {			
//				//Find Reference on the axis selected
//				err = flex_find_reference(this->boardId, NIMC_AXIS_CTRL, axisBitmap, NIMC_FIND_HOME_REFERENCE);
//				if(err != SPL_OK)
//				{
//					return false;
//				}
//			//Then Find Index
//			} else if (i == 1) {			
//				//Find Reference on the axis selected
//				err = flex_find_reference(this->boardId, NIMC_AXIS_CTRL, axisBitmap, NIMC_FIND_INDEX_REFERENCE);
//				if(err != SPL_OK)
//				{
//					return false;
//				}
//			}
//			
//			err = flex_check_reference(this->boardId, NIMC_AXIS_CTRL, axisBitmap, &referenceFound, &axisIsFinding);
//			if(err != SPL_OK)
//				{
//					return false;
//				}
//			if (i == 0)
//				homeFound = referenceFound;
//			else
//				indexFound = referenceFound;
//
//		}
//		referenceFound = homeFound & indexFound; 
//	
//	if(referenceFound != SPL_OK)
//	{
//		ErrPrintf("GoZero error occured, can not start,  pls chk!");
//		return false;		
//	}
//	else
//		/*isOver = this->WaitComplete(SPL_1_SECOND*10);
//		if(isOver == false)
//		{
//			ErrPrintf("fatal error, can not wait motion over before start muti axis motion!");
//			return false;
//		}*/
//		WaitTime(2000);
//		//reset encoders
//		err = flex_reset_pos(this->boardId,NIMC_AXIS1,0,0,0xFF); 
//			
//		err |= flex_reset_pos(this->boardId,NIMC_AXIS2,0,0,0xFF); 
//			
//		err |= flex_reset_pos(this->boardId,NIMC_AXIS3,0,0,0xFF); 
//		if(err != SPL_OK)
//			{
//				return false;
//			}
//	return true;
//
//
//}

//emergency halt
//用于与Bgo配合
//这里面目前用不到
void NmcHardareRes::EHalt()
{
	Motion* item;

	if (this->isOpen == false)
	{
		return;
	}

	this->isPause = true;
	this->Halt();

	mut.lock();
	while (!Queue.isEmpty())
	{
		Queue.Take(item);
		tmpQueue.Put(item);
	}

	mut.unlock();
	
}

void NmcHardareRes::EKill()
{
	if (this->isOpen == false)
	{
		return;
	}

	this->Kill();
}

void NmcHardareRes::EStop()
{
	if (this->isOpen == false)
	{
		return;
	}

	this->Stop();
}

//one axis moving
bool NmcHardareRes::Go(int axis, double range, double speed)
{
	//defination
	bool isOk;
	double time;

	//init
	isOk = false;
	time = 0;

	//set 
	isOk = this->SetGo(axis, range, speed);
	if (isOk != true)
	{
		ErrPrintf("SetGo() return error, pls chk!");
		return false;
	}

	//go
	isOk = this->Start(axis);
	if (isOk == false)
	{
		ErrPrintf("Start() return error, pls chk!");
		return false;
	}

	//return
	return isOk;
}

//inner fucntion for set one axis velocity and range, unit is mm/s and mm(+ is posive direction, - negtive direction)
bool NmcHardareRes::SetGo(int axis, double range, double speed)
{
	//defination
	bool isOk;
	int err;
	int status;
	int stepV;
	int stepPos;
	double time;
	double maxVeloc;
	double minVeloc;

	//init
	isOk = false;
	err = 0;
	status = 0;
	stepV = 0;
	stepPos = 0;
	time = 0;

	if (this->isOpen == false)
	{
		ErrPrintf("pls open the device firstly!");
		return false;
	}

	//check inputs
	err = ChkAxis(axis);

	this->accRate = SPL_RUNNING_ACC;

	//与7334init()一样，再次针对单个轴进行一次参数配置
	err |= LoadMotionPara(axis);
	if (err != SPL_OK)
	{
		return false;
	}

	if(this->ChkRangeIsValid(axis,range) != true)
	{
	ErrPrintf("range is exceed the max or min, pls chk!");
	return false;
	}

	maxVeloc = this->GetMaxVeloc(axis);
	minVeloc = this->GetMinVeloc(axis);
	if (speed > maxVeloc || speed < minVeloc)
	{
		err = SPL_ERR;
		ErrPrintf("mmVeloc vecocity is too large, pls chk!");
		return false;
	}
	//change  mm to the correspond pulse num 
	stepV = this->Mm2Steps(axis, speed);

	//set velocity
	status = flex_load_velocity(this->boardId, (u8)axis, (i32)stepV, SPL_DIRECT);
	if (status != SPL_OK)
	{
		ErrPrintf("flex_load_velocity() return error, pls chk!");
		return false;
	}

	//set target pos

	stepPos = this->Mm2Steps(axis, range);

	status = flex_load_target_pos(this->boardId, (u8)axis, (i32)stepPos, SPL_DIRECT);
	if (status != SPL_OK)
	{
		ErrPrintf("flex_load_target_pos() return error, pls chk!");
		return false;
	}
	return true;
}

//
//bool NmcHardareRes::Go(int axis1, double range1, double speed1, int axis2, double range2, double speed2, int axis3, double range3, double speed3)
//{
//	
//	//defination
//	bool isOk;
//	double time;
//	
//	//init
//	isOk = false;
//	time = 0;
//
//	
//	//set 
//	isOk = this->SetGo(axis1, range1, speed1);
//	isOk |= this->SetGo(axis2, range2, speed2);
//	isOk |= this->SetGo(axis3, range3, speed3);
//	if(isOk != true)
//	{
//		ErrPrintf("SetGo() return error, pls chk!");
//		return false;
//	}
//	//go
//	isOk = this->Start(axis1, axis2, axis3);
//	if(isOk == false)
//	{
//		ErrPrintf("Start() return error, pls chk!");
//		return false;
//	}
//	
//	return isOk;
//}

//bool NmcHardareRes::BScanUsingModulusBreakPoint(int axis,double targetPos,double velocity,double modulus)
//{
//	//Variables for motion 
//	int targetPosTemp;
//	int modulusTemp;
//	int velocityTemp;
//
//	//Variables for hardware 
//	u8  boardID;
//	u16 csr	= 0;					// Communication Status Register
//	u8	profileStatus;				// Profile Complete Status
//	u8  bpStatus;					// Break Point found Status
//	u16 status;						// Axis status   
//	i32 bpPos=-500;					// Break Point Position
//	u32 bpMod;						// Break Point Modulus
//	//i32 targetPos;				// Target Position
//	i32 currentPos;					// Current Position
//	u16 axisStatus;					// Status of the axis
//	i32 scanVar;					// Scan variable to read in values not supported bu
//									//		by the scanf function
//
//	//Variables for modal error handling
//	u16 commandID;					// The commandID of the function
//	u16 resourceID;					// The resource ID
//	i32 errorCode;
//	i32 err = -1;
//	//Get the Break Point Modulus
//	bpMod =  modulus;
//
//	boardID = this->boardId;
//	velocityTemp = (i32) velocity*2500/5;
//	
//	if(axis==3)
//		targetPosTemp = (int)targetPos*2500;
//	else if(axis==1)
//		targetPosTemp = -(int)targetPos*2500/5; 
//	else
//		targetPosTemp = (int)targetPos*2500/5; 
//			
//	modulusTemp = (int) (modulus*2500/5); 
//	
//   //Set Operation Mode
//	err  = flex_set_op_mode(boardID, axis, NIMC_RELATIVE_POSITION);
//    err |= flex_load_velocity(boardID, axis, velocityTemp, 0xFF); 
//   //Load the break point position
//	err |= flex_load_pos_bp(boardID,axis,bpPos,0xFF);
//	//Route breakpoint 1 to RTSI line 2
//	err |= flex_select_signal(boardID,NIMC_RTSI2,NIMC_BREAKPOINT1);
//	//Configure the break point to be modulus
//	err |= flex_configure_breakpoint(boardID,axis,NIMC_MODULO_BREAKPOINT,NIMC_NO_CHANGE,0);
//	//Reset the position
//	err |= flex_reset_pos(boardID,axis,0,0,0xFF);
//	//Enable the breakpoint
//	err |= flex_enable_breakpoint(boardID,axis,NIMC_TRUE); 
//	//Load a target position
//	err |= flex_load_target_pos(boardID,axis,targetPosTemp,0xFF);
//	//Start the motion
//	err |= flex_start(boardID,axis,0);
//	
//	//route the signal 
//	err |= flex_select_signal(boardID,NIMC_RTSI0,NIMC_RTSI_SOFTWARE_PORT); 
//	err |= flex_select_signal(boardID,NIMC_RTSI1,NIMC_RTSI_SOFTWARE_PORT);
//	err |= flex_select_signal(boardID,NIMC_RTSI2,NIMC_RTSI_SOFTWARE_PORT);
//	err |= flex_select_signal(boardID,NIMC_RTSI4,NIMC_RTSI_SOFTWARE_PORT);
//	err |= flex_select_signal(boardID,NIMC_RTSI6,NIMC_RTSI_SOFTWARE_PORT);
//	if(err != SPL_OK)
//	{
//		return false;
//	}
//
//	status = flex_set_port(boardID,NIMC_RTSI_PORT,0x02,0x01,0xFF);  
//	if(status != SPL_OK)
//	{
//		ErrPrintf("flex_set_port return error, pls chk!");
//		return false;
//	}
//
//	switch(axis){
//		case 1:
//				do
//				{
//					//Read the axis status
//					err = flex_read_axis_status_rtn(boardID,axis,&axisStatus);
//					err |= flex_read_pos_rtn(boardID,axis,&currentPos);
//					if(err != SPL_OK)
//					{
//						return false;
//					}
//					//Check the breakpoint bit
//					bpStatus = !((axisStatus & NIMC_POS_BREAKPOINT_BIT)==0);
//					//Check the profile complete bit
//					profileStatus = !((axisStatus & NIMC_PROFILE_COMPLETE_BIT)==0);
//
//					//printf("Current Position=%10d Breakpoint Status=%d Profile Complete=%d\r",currentPos,bpStatus,profileStatus);
//
//					//Re-enable the breakpoint
//					if (bpStatus)
//					{
//						//printf("Breakpoint Occured\n");
//						//Enable the breakpoint on axis 1
//						err = flex_enable_breakpoint(boardID,axis,NIMC_TRUE);
//						if(err != SPL_OK)
//						{
//							return false;
//						}
//						status = flex_set_port(boardID,NIMC_RTSI_PORT,0x0A,0x01,0xFF); 
//						if(status != SPL_OK)
//						{
//							ErrPrintf("flex_set_port return error, pls chk!");
//							return false;
//						}
//					}
//
//					//Check for modal errors
//					err = flex_read_csr_rtn(boardID,&csr);
//					if(err != SPL_OK)
//					{
//						return false;
//					}
//
//					//Check the modal errors
//					if (csr & NIMC_MODAL_ERROR_MSG)
//					{
//						flex_stop_motion(boardID,NIMC_VECTOR_SPACE1, NIMC_DECEL_STOP, 0);//Stop the Motion
//						err = csr & NIMC_MODAL_ERROR_MSG;
//						if(err != SPL_OK)
//						{
//							return false;
//						}
//					}
//					status = flex_set_port(boardID,NIMC_RTSI_PORT,0x02,0x01,0xFF); 
//					if(status != SPL_OK)
//					{
//						ErrPrintf("flex_set_port return error, pls chk!");
//						return false;
//					}
//
//				}while(!profileStatus);
//				break;
//		case 2:
//				do
//				{
//					//Read the axis status
//					err = flex_read_axis_status_rtn(boardID,axis,&axisStatus);
//					err |= flex_read_pos_rtn(boardID,axis,&currentPos);
//					if(err != SPL_OK)
//					{
//						return false;
//					}
//					//Check the breakpoint bit
//					bpStatus = !((axisStatus & NIMC_POS_BREAKPOINT_BIT)==0);
//					//Check the profile complete bit
//					profileStatus = !((axisStatus & NIMC_PROFILE_COMPLETE_BIT)==0);
//
//					//printf("Current Position=%10d Breakpoint Status=%d Profile Complete=%d\r",currentPos,bpStatus,profileStatus);
//
//					//Re-enable the breakpoint
//					if (bpStatus)
//					{
//						//printf("Breakpoint Occured\n");
//						//Enable the breakpoint on axis 1
//						err = flex_enable_breakpoint(boardID,axis,NIMC_TRUE);
//						if(err != SPL_OK)
//						{
//							return false;
//						}
//						status = flex_set_port(boardID,NIMC_RTSI_PORT,0x22,0x01,0xFF); 
//						if(status != SPL_OK)
//						{
//							ErrPrintf("flex_set_port return error, pls chk!");
//							return false;
//						}
//					}
//
//					//Check for modal errors
//					err = flex_read_csr_rtn(boardID,&csr);
//					if(err != SPL_OK)
//					{
//						return false;
//					}
//
//					//Check the modal errors
//					if (csr & NIMC_MODAL_ERROR_MSG)
//					{
//						flex_stop_motion(boardID,NIMC_VECTOR_SPACE1, NIMC_DECEL_STOP, 0);//Stop the Motion
//						err = csr & NIMC_MODAL_ERROR_MSG;
//						if(err != SPL_OK)
//						{
//							return false;
//						}
//					}
//					status = flex_set_port(boardID,NIMC_RTSI_PORT,0x02,0x01,0xFF); 
//					if(status != SPL_OK)
//					{
//						ErrPrintf("flex_set_port return error, pls chk!");
//						return false;
//					}
//
//				}while(!profileStatus);
//				break;
//		case 3:
//				do
//				{
//					//Read the axis status
//					err = flex_read_axis_status_rtn(boardID,axis,&axisStatus);
//					err |= flex_read_pos_rtn(boardID,axis,&currentPos);
//					if(err != SPL_OK)
//					{
//						return false;
//					}
//					//Check the breakpoint bit
//					bpStatus = !((axisStatus & NIMC_POS_BREAKPOINT_BIT)==0);
//					//Check the profile complete bit
//					profileStatus = !((axisStatus & NIMC_PROFILE_COMPLETE_BIT)==0);
//
//					//printf("Current Position=%10d Breakpoint Status=%d Profile Complete=%d\r",currentPos,bpStatus,profileStatus);
//
//					//Re-enable the breakpoint
//					if (bpStatus)
//					{
//						//printf("Breakpoint Occured\n");
//						//Enable the breakpoint on axis 1
//						err = flex_enable_breakpoint(boardID,axis,NIMC_TRUE);
//						if(err != SPL_OK)
//						{
//							return false;
//						}
//						status = flex_set_port(boardID,NIMC_RTSI_PORT,0x082,0x01,0xFF); 
//						if(status != SPL_OK)
//						{
//							ErrPrintf("flex_set_port return error, pls chk!");
//							return false;
//						}
//					}
//
//					//Check for modal errors
//					err = flex_read_csr_rtn(boardID,&csr);
//					if(err != SPL_OK)
//					{
//						return false;
//					}
//
//					//Check the modal errors
//					if (csr & NIMC_MODAL_ERROR_MSG)
//					{
//						flex_stop_motion(boardID,NIMC_VECTOR_SPACE1, NIMC_DECEL_STOP, 0);//Stop the Motion
//						err = csr & NIMC_MODAL_ERROR_MSG;
//						if(err != SPL_OK)
//						{
//							return false;
//						}
//					}
//					status = flex_set_port(boardID,NIMC_RTSI_PORT,0x02,0x01,0xFF); 
//					if(status != SPL_OK)
//					{
//						ErrPrintf("flex_set_port return error, pls chk!");
//						return false;
//					}
//
//				}while(!profileStatus);
//				break;
//	}
//	
//	status = flex_set_port(boardID,NIMC_RTSI_PORT,0x01,0x02,0xFF);
//	//printf("\nFinished.\n");
//
//	return true;		// Exit the Application
//
//}


bool NmcHardareRes::GoPlanar(int axis1, double range1, int axis2, double range2, double speed, double mmStep)
{

	//defination
	bool isOk;
	double time;
	int num;
	int i;

	//init
	isOk = false;
	time = 0;
	num = 0;

	//check inputs
	if (mmStep < this->CalVeloc(axis2, 1))
	{
		ErrPrintf("mmStep is too little, pls chk!");
		return false;
	}

	num = (int)(abs(range2) / mmStep + 1);

	for (i = 0; i <= num / 2; i++)
	{
		//axis 1 go
		isOk = Go(axis1, range1, speed);
		if (isOk != true)
		{
			ErrPrintf("Go() axis1 return error, pls chk!");
			return false;
		}

		//axis2 go step
		isOk = Go(axis2, mmStep, speed);
		if (isOk != true)
		{
			ErrPrintf("Go() axis2 return error, pls chk!");
			return false;
		}

		//axis 1 go back
		isOk = Go(axis1, -range1, speed);
		if (isOk != true)
		{
			ErrPrintf("Go() axis1 back return error, pls chk!");
			return false;
		}

		//axis2 go step again
		isOk = Go(axis2, mmStep, speed);
		if (isOk != true)
		{
			ErrPrintf("Go() axis2 again return error, pls chk!");
			return false;
		}
	}

	//return
	return isOk;
}

bool NmcHardareRes::BGoPlanar(int axis1, double range1, int axis2, double range2, double speed, double mmStep)
{

	//defination
	bool isOk;
	double time;
	int num;
	int i;

	//init
	isOk = false;
	time = 0;
	num = 0;

	//check inputs
	if (mmStep < this->CalVeloc(axis2, 1))
	{
		ErrPrintf("mmStep is too little, pls chk!");
		return false;
	}

	num = (int)(abs(range2) / mmStep + 1);

	while (!this->Queue.Empty());

	for (i = 0; i < num / 2; i++)
	{
		//axis 1 go
		isOk = BGo(axis1, range1, speed);
		if (isOk != true)
		{
			ErrPrintf("Go() axis1 return error, pls chk!");
			return false;
		}

		//axis2 go step
		isOk = BGo(axis2, mmStep, speed);
		if (isOk != true)
		{
			ErrPrintf("Go() axis2 return error, pls chk!");
			return false;
		}

		//axis 1 go back
		isOk = BGo(axis1, -range1, speed);
		if (isOk != true)
		{
			ErrPrintf("Go() axis1 back return error, pls chk!");
			return false;
		}

		//axis2 go step again
		isOk = BGo(axis2, mmStep, speed);
		if (isOk != true)
		{
			ErrPrintf("Go() axis2 again return error, pls chk!");
			return false;
		}
	}

	//return
	return isOk;
}

//check the limit switch
bool NmcHardareRes::Chklimit()
{
	int err;
	u16 forLimitStatus;
	u16 resLimitStatus;
	err = flex_read_axis_limit_status_rtn(this->boardId,NIMC_LIMIT_INPUTS, &forLimitStatus, &resLimitStatus);
	if (err == SPL_OK)
	{
		ErrPrintf("failed to check limit switch, pls chk!");
	}
	if (forLimitStatus == 0)
	{
		this->limitflag = 0;
	}
	if (forLimitStatus == 0x0002 && this->limitflag == 0)
	{
		EStop();
	}
	return true;
}


//return cur axis's pos the unit is mm						
bool NmcHardareRes::GetCurPos(int axis, double* pos)
{
	//defination
	int err;
	int status;
	i32 stepPos;
	double rpos; //relative pos

	//init
	err = 0;
	stepPos = 0;
	rpos = 0;

	//check inputs
	err = ChkAxis(axis);
	if (err != SPL_OK)
	{
		return false;
	}

	if (pos == 0)
	{
		ErrPrintf("input pos ptr is Null, pls chk!");
		return false;
	}

	*pos = 0;
	//main
	status = flex_read_pos_rtn(this->boardId, (unsigned char)axis, &stepPos);
	if (status != SPL_OK)
	{
		ErrPrintf("flex_read_pos_rtn(), return false, pls chk!");
		return false;
	}

	/*
	if (stepPos < 0)   //maybe has bug
	{
		stepPos = -stepPos;
	}
	*/

	rpos = this->Steps2mm((unsigned char)axis, (int)stepPos); //maybe has bug?

	//这段整体删掉，感觉没有意义
	/*
															  //update pos state
	switch (axis)
	{
	case SPL_X_AXIS:
	{
		rpos = rpos - SPL_RELATIVE_X_OFFSET - 0.001999999999998;
	}
	break;
	case SPL_Y_AXIS:
	{
		rpos = rpos - SPL_RELATIVE_Y_OFFSET;
	}
	break;
	case SPL_Z_AXIS:
	{
		rpos = rpos - SPL_RELATIVE_Z_OFFSET - 0.0012;
	}
	break;

	default:
		rpos = 0;
		break;
	}
	*/

	if (rpos < (double)-SPL_LOC_TOLERANCE)
	{
		ErrPrintf("relative pos chk error, pls chk!");
	}
	this->curPos[axis] = rpos;
	*pos = rpos;
	//return
	return true;
}

//return cur axis's velocity, unit is mm/s
bool NmcHardareRes::GetCurVeloc(int axis, double* v)
{
	//defination
	int err;
	int status;
	i32 stepV;

	//init
	err = 0;
	stepV = 0;

	//check inputs
	err = ChkAxis(axis);
	if (err != SPL_OK)
	{
		return false;
	}

	if (v == 0)
	{
		ErrPrintf("input pos ptr is Null, pls chk!");
		return false;
	}

	*v = 0;

	//main
	status = flex_read_velocity_rtn(this->boardId, (unsigned char)axis, &stepV);
	if (status != SPL_OK)
	{
		ErrPrintf("flex_read_velocity_rtn(), return false, pls chk!");
		return false;
	}

	*v = this->Steps2mm((unsigned char)axis, (int)stepV); //maybe has bug?
	
    //update pos state
	this->curVeloc[axis] = *v;

	//return
	return true;
}

//下面的程序可以用来同步
//下面两段程序应该可以删除
//sigal utsp board to start testing in automode
bool NmcHardareRes::SigStart2Hardware()
{
	unsigned char mustOn;
	unsigned char mustOff;
	int status;

	status = 0;
	mustOn = 0x02; //start is high
	mustOff = 0x01; //stop is high

	if (this->isOpen == false)
	{
		ErrPrintf("board has not opended, pls open firstly!");
		return false;
	}

	status = flex_select_signal(this->boardId, NIMC_RTSI0, NIMC_RTSI_SOFTWARE_PORT);
	status |= flex_select_signal(this->boardId, NIMC_RTSI1, NIMC_RTSI_SOFTWARE_PORT);
	if (status != SPL_OK)
	{
		ErrPrintf("port error occured, pls chk!");
		return false;
	}

	status = flex_set_port(this->boardId, NIMC_RTSI_PORT, mustOn, mustOff, SPL_DIRECT);
	if (status != SPL_OK)
	{
		ErrPrintf("flex_set_port() error occured, pls chk!");
		return false;
	}
	return true;
}

//用RISI总线实现各个板卡的同步
bool NmcHardareRes::SigStop2Hardware()
{
	//definition
	unsigned char mustOn;
	unsigned char mustOff;
	int status;

	//init 
	status = 0;
	mustOn = 0x01; //start is high
	mustOff = 0x02; //stop is high

	if (this->isOpen == false)
	{
		ErrPrintf("board has not opended, pls open firstly!");
		return false;
	}

	status = flex_set_port(this->boardId, NIMC_RTSI_PORT, mustOn, mustOff, SPL_DIRECT);
	if (status != SPL_OK)
	{
		ErrPrintf("flex_set_port() error occured, pls chk!");
		return false;
	}
	return true;
}


bool NmcHardareRes::BGo(int axis, double range, double speed)
{
	int m_axis;
	double m_range;
	double m_speed;
	Motion* item;
	bool result;

	m_axis = axis;
	m_range = range;
	m_speed = speed;
	item = new Motion(axis, range, speed);
	//this->isrunning = true;
	result = Queue.Put(item);
	return result;
}

//与Bgo等函数搭配
bool NmcHardareRes::Continue()
{
	double curpos;
	double range;
	bool result;
	Motion* item;

	this->isQuit = false;

	this->isPause = false;
	
	GetCurPos(tmpaxis, &curpos);

	//注意这里用的tmpaxis 是与 Bgo配合的，在这里可以不用
	if ((tmprange - (curpos - startPos)) != 0)
	{
		range = (tmprange - (curpos - startPos));
		result = BGo(tmpaxis, range, tmpspeed);
		GetCurPos(tmpaxis, &curpos);

	}
	mut.lock();
	while (!tmpQueue.isEmpty())
	{
		tmpQueue.Take(item);
		Queue.Put(item);
	}
	
	mut.unlock();

	return result;
}

void NmcHardareRes::reset(int axis1 ,int axis2 , int axis3 , int axis4 )
{
	int status;
	status=flex_reset_pos(this->boardId, axis1, 0, 0, SPL_DIRECT);
	status = flex_reset_pos(this->boardId, axis2, 0, 0, SPL_DIRECT);
	status = flex_reset_pos(this->boardId, axis3, 0, 0, SPL_DIRECT);
	status = flex_reset_pos(this->boardId, axis4, 0, 0, SPL_DIRECT);
}