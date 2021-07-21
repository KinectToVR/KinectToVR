#pragma once
#include "stdafx.h"

#include <string>
#include <vector>
#include <SFML/Graphics/RenderWindow.hpp>
#include <gl/glew.h>
#include <openvr.h>

#include <opencv2/opencv.hpp>


#include <windows.h>

#include "KinectSettings.h"

class IKinectHandler
{
	// Interface base for Kinectv1 and v2
public:
	virtual ~IKinectHandler()
	{
	}

	virtual void initOpenGL() = 0;
	virtual void initialise() = 0;

	virtual HRESULT getStatusResult() = 0;
	virtual std::string statusResultString(HRESULT stat) = 0;

	virtual void update() = 0;

	virtual void drawKinectData(sf::RenderWindow& win) = 0; // Houses the below draw functions with a check
	virtual void drawKinectImageData(sf::RenderWindow& win) = 0;
	virtual void drawTrackedSkeletons(sf::RenderWindow& win) = 0;
	
	bool isInitialised() { return initialised; }
	bool isZeroed() { return zeroed; }

	KinectVersion kVersion = KinectVersion::INVALID;
	std::unique_ptr<GLubyte[]> kinectImageData; // array containing the texture data

	bool zeroed = false, isPSMS = false;
	vr::HmdVector3d_t trackedPositionVROffset = {0, 0, 0};
protected:
	bool initialised = false;

	class FailedKinectInitialisation : public std::exception
	{
		const char* what() const throw() override
		{
			return "Failure to initialise the kinect sensor. Is it plugged in and supplied with power?";
		}
	} FailedKinectInitialisation;

private:
};
