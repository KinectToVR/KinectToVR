// KinectV2Process.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "KinectV2Handler.h"
#include <KinectToVR.h>
#include <sstream>
#include <string>
#include <iostream>

using namespace std;

int main(int argc, char* argv[])
{
    // Set up the 'crash handler'
    std::thread([] {
        auto _pid = GetCurrentProcessId();
        system(std::string("KV2CrashHandler.exe " + std::to_string(_pid)).c_str());
        }).detach();
	
	initLogging();
	HWND hWnd = GetConsoleWindow();
	ShowWindow(hWnd, SW_SHOW);
#ifndef _DEBUG
	ShowWindow(hWnd, SW_HIDE);
#endif
	KinectV2Handler kinect;
	KinectSettings::leftFootJointWithRotation = KVR::KinectJointType::AnkleLeft;
	KinectSettings::rightFootJointWithRotation = KVR::KinectJointType::AnkleRight;
	KinectSettings::leftFootJointWithoutRotation = KVR::KinectJointType::FootLeft;
	KinectSettings::rightFootJointWithoutRotation = KVR::KinectJointType::FootRight;

	processLoop(kinect);
	return 0;
}

/*
#ifdef _WIN32
// This disables the console window from appearing on windows only if the Project Settings->Linker->System->SubSystem is set to Windows (rather than Console).
int WinMain(HINSTANCE hinstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCMDShow)
{
    KinectV2Handler kinect;

    processLoop(kinect);

    return 0;
}
#endif
*/
