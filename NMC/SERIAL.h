#ifndef SERIAL_PORT
#define SERIAL_PORT

#include<tchar.h>
#include<fstream>
#include<string>
using namespace std;

class SerialPort
{
private:
	OVERLAPPED wrOverlapped;
	string logFileName;
	ofstream errLog;
	HANDLE hcom;

private:
	void LogPrintf(long line, char *strfunc, char * strErr);

public:
	SerialPort();
	~SerialPort();

public:
	HANDLE Open_driver_Y(TCHAR *name);

	bool Send_driver_Y(HANDLE fd, BYTE *data, DWORD dwDataLen);

	bool Receive_driver_Y(HANDLE fd, BYTE *data);

	void Close_driver(void *fd);

};



#endif