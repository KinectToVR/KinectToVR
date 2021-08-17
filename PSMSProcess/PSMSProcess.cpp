// PSMSProcess.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <KinectToVR.h>
#include <openvr.h>
#include <Windows.h>

//This process is intended to be run in situations without a kinect
// i.e. at time of writing, the SteamVRBridge driver for PSMoves is broke
// and I just want to use the moves as trackers, so the process will be run without the kinect

class FakeKinect : public KinectHandlerBase
{
public:
	FakeKinect()
	{
		initialised = true;
		isPSMS = true; //Notify Parent that we're using PSMS
	}

	~FakeKinect()
	{
	}
};

int main(int argc, char* argv[])
{
	initLogging();
	HWND hWnd = GetConsoleWindow();
	ShowWindow(hWnd, SW_SHOW);
#ifndef _DEBUG
	ShowWindow(hWnd, SW_HIDE);
#endif
	FakeKinect kinect;

	processLoop(kinect);

	return 0;
}
