#pragma once
#include "stdafx.h"
#include "KinectV2Handler.h"
#include <sfLine.h>
#include <iostream>
#include <VRHelper.h>
#include "KinectJointFilter.h"
#include <Eigen/Geometry>
#include <ppl.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/transform2.hpp>
#include <glm/mat4x4.hpp>
#include <glm/detail/type_vec3.hpp>
#include <glm/detail/type_vec4.hpp>
#include <glm/detail/type_vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <thread>
#include <chrono>
#include <../KTVR/KinectToVR/LowPassFilter.h>

LowPassFilter lowPassFilter[3][4] = {
	{LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005)},
	{LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005)},
	{LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005)}
};

HRESULT KinectV2Handler::getStatusResult()
{
	BOOLEAN avail;
	kinectSensor->get_IsAvailable(&avail);
	if (avail)
		return S_OK;
	return S_FALSE;
	// Hresult only actually determines whether the function worked, the bool is the true value....
}

std::string KinectV2Handler::statusResultString(HRESULT stat)
{
	switch (stat)
	{
	case S_OK: return "S_OK";
	case S_FALSE: return "Sensor Unavailable! Check if it's plugged in to your USB and power plugs";
	default: return "Uh Oh undefined kinect error! " + std::to_string(stat);
	}
}

void KinectV2Handler::initialise()
{
	try
	{
		kVersion = KinectVersion::Version2;
		kinectImageData = std::make_unique<GLubyte[]>(
			KinectSettings::kinectV2Width * KinectSettings::kinectV2Height * 4); //RGBA
		initialised = initKinect();
		// initialiseColor();
		// Commented both image frames out, as most people use the kinect for skeletal data
		// Updating all of the arrays uses a shit ton of CPU, but then again, it's still WIP
		// initialiseDepth();
		initialiseSkeleton();
		if (!initialised) throw FailedKinectInitialisation;
	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what() << std::endl;
	}
}

void KinectV2Handler::initialiseSkeleton()
{
	if (bodyFrameReader)
		bodyFrameReader->Release();
	IBodyFrameSource* bodyFrameSource;
	kinectSensor->get_BodyFrameSource(&bodyFrameSource);
	bodyFrameSource->OpenReader(&bodyFrameReader);

	// Newfangled event based frame capture
	// https://github.com/StevenHickson/PCL_Kinect2SDK/blob/master/src/Microsoft_grabber2.cpp
	h_bodyFrameEvent = (WAITABLE_HANDLE)CreateEvent(nullptr, FALSE, FALSE, nullptr);
	HRESULT hr = bodyFrameReader->SubscribeFrameArrived(&h_bodyFrameEvent);
	if (bodyFrameSource) bodyFrameSource->Release();
	if (FAILED(hr))
	{
		//throw std::exception("Couldn't subscribe frame");
		LOG(ERROR) << "ERROR: Could not subscribe to skeleton frame event! HRESULT " << hr;
	}
	else
	{
		LOG(INFO) << "Kinect Skeleton Reader subscribed to event, initialised successfully.";
	}
}

void KinectV2Handler::initialiseColor()
{
https: //github.com/UnaNancyOwen/Kinect2Sample/blob/master/sample/CoordinateMapper/app.h
	if (colorFrameReader)
		colorFrameReader->Release();
	// Open Color Reader
	IColorFrameSource* colorFrameSource;
	kinectSensor->get_ColorFrameSource(&colorFrameSource);
	HRESULT hr = colorFrameSource->OpenReader(&colorFrameReader);

	// Retrieve Color Description
	IFrameDescription* colorFrameDescription;
	colorFrameSource->CreateFrameDescription(ColorImageFormat_Bgra, &colorFrameDescription);
	colorFrameDescription->get_Width(&colorWidth); // 1920
	colorFrameDescription->get_Height(&colorHeight); // 1080
	colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel); // 4

	// Allocation Color Buffer
	colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
	if (colorFrameSource) colorFrameSource->Release();
	if (colorFrameDescription) colorFrameDescription->Release();

	if (FAILED(hr))
	{
		LOG(ERROR) << "Kinect Color Reader could not be opened! HRESULT " << hr;
	}
	else
		LOG(INFO) << "Kinect Color Reader initialised successfully.";
}

void KinectV2Handler::initialiseDepth()
{
https: //github.com/UnaNancyOwen/Kinect2Sample/blob/master/sample/CoordinateMapper/app.h
	if (depthFrameReader) depthFrameReader->Release();

	// Open Depth Reader
	IDepthFrameSource* depthFrameSource;
	kinectSensor->get_DepthFrameSource(&depthFrameSource);
	HRESULT hr = depthFrameSource->OpenReader(&depthFrameReader);

	// Retrieve Depth Description
	IFrameDescription* depthFrameDescription;
	depthFrameSource->get_FrameDescription(&depthFrameDescription);
	depthFrameDescription->get_Width(&depthWidth); // 512
	depthFrameDescription->get_Height(&depthHeight); // 424
	depthFrameDescription->get_BytesPerPixel(&depthBytesPerPixel); // 2
	// Allocation Depth Buffer
	depthBuffer.resize(depthWidth * depthHeight);

	if (depthFrameSource) depthFrameSource->Release();
	if (depthFrameDescription) depthFrameDescription->Release();

	if (FAILED(hr))
	{
		LOG(ERROR) << "Kinect Depth Reader could not be opened! HRESULT " << hr;
	}
	else
		LOG(INFO) << "Kinect Depth Reader initialised successfully.";
}

void KinectV2Handler::terminateSkeleton()
{
	if (bodyFrameReader)
	{
		HRESULT hr = bodyFrameReader->UnsubscribeFrameArrived(h_bodyFrameEvent);
		if (FAILED(hr))
		{
			LOG(ERROR) << "Couldn't unsubscribe skeleton frame! HRESULT " << hr;
			throw std::exception("Couldn't unsubscribe frame!");
		}
		CloseHandle((HANDLE)h_bodyFrameEvent);
		h_bodyFrameEvent = NULL;

		bodyFrameReader->Release();
		bodyFrameReader = nullptr;
		LOG(INFO) << "Skeleton Reader closed successfully";
	}
	else
		LOG(WARNING) << "Skeleton Reader was asked to terminate, but was already closed!";
}

void KinectV2Handler::terminateColor()
{
	if (colorFrameReader)
	{
		colorFrameReader->Release();
		colorFrameReader = nullptr;
		colorMat.release();
		LOG(INFO) << "Color Reader closed successfully";
	}
	else
		LOG(WARNING) << "Color Reader was asked to terminate, but was already closed!";
}

void KinectV2Handler::terminateDepth()
{
	if (depthFrameReader)
	{
		depthFrameReader->Release();
		depthFrameReader = nullptr;
		depthMat.release();
		LOG(INFO) << "Depth Reader closed successfully";
	}
	else
		LOG(WARNING) << "Depth Reader was asked to terminate, but was already closed!";
}

void KinectV2Handler::initOpenGL()
{
	LOG(INFO) << "Attempted to initialise OpenGL";
	int width = 0, height = 0;
	if (kVersion == KinectVersion::Version1)
	{
		width = KinectSettings::kinectWidth;
		height = KinectSettings::kinectHeight;
	}
	else if (kVersion == KinectVersion::Version2)
	{
		width = KinectSettings::kinectV2Width;
		height = KinectSettings::kinectV2Height;
	} // REMOVE THIS INTO KINECT V2 IMPL
	// Initialize textures
	glGenTextures(1, &kinectTextureId);
	glBindTexture(GL_TEXTURE_2D, kinectTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height,
	             0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, static_cast<GLvoid*>(kinectImageData.get()));
	glBindTexture(GL_TEXTURE_2D, 0);

	// OpenGL setup
	glClearColor(1, 0, 0, 0);
	glClearDepth(1.0f);
	glEnable(GL_TEXTURE_2D);

	// Camera setup
	glViewport(0, 0, SFMLsettings::m_window_width, SFMLsettings::m_window_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, SFMLsettings::m_window_width, SFMLsettings::m_window_height, 0, 1, -1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void KinectV2Handler::update()
{
	if (isInitialised())
	{
		BOOLEAN isAvailable = false;
		HRESULT kinectStatus = kinectSensor->get_IsAvailable(&isAvailable);
		if (kinectStatus == S_OK)
		{
			// NEW ARRIVED FRAMES ------------------------
			MSG msg;
			while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) // Unneccesary?
			{
				DispatchMessage(&msg);
			}

			if (h_bodyFrameEvent)
			{
				//printf("Kinect Event ID: %d\n" ,(int)h_bodyFrameEvent);

				//now check for IR Events
				HANDLE handles[] = {reinterpret_cast<HANDLE>(h_bodyFrameEvent)};
				// , reinterpret_cast<HANDLE>(ke.hMSEvent)		};

				switch (MsgWaitForMultipleObjects(_countof(handles), handles, false, 0, QS_ALLINPUT))
				{
				case WAIT_OBJECT_0:
					{
						IBodyFrameArrivedEventArgs* pArgs = nullptr;
						//printf("Body Frame Event Signaled.\n");

						if (bodyFrameReader)
						{
							HRESULT hr = bodyFrameReader->GetFrameArrivedEventData(h_bodyFrameEvent, &pArgs);
							//printf("Retreive Frame Arrive Event Data -HR: %d\n", hr);

							if (SUCCEEDED(hr))
							{
								//printf("Retreived Frame Arrived Event Data\n");
								onBodyFrameArrived(*bodyFrameReader, *pArgs);
								pArgs->Release();
								//printf("Frame Arrived Event Data Released\n");
							}
						}
					}
					break;
				}
			}
			// ------------------------------
			updateKinectData();
		}
	}
}

template <typename T>
void updateBufferWithSmoothedMat(cv::Mat& in, cv::Mat& out, std::vector<T>& buffer)
{
	int filterIntensity = 5; //MUST be odd
	medianBlur(in, out, filterIntensity);

	if (out.isContinuous()) {
		buffer.assign((T*)out.datastart, (T*)out.dataend);
	}
	else {
		for (int i = 0; i < out.rows; ++i) {
			buffer.insert(buffer.end(), out.ptr<T>(i), out.ptr<T>(i) + out.cols);
		}
	}
}

void KinectV2Handler::updateColorData()
{
	if (colorFrameReader)
	{
		IColorFrame* colorFrame;
		const HRESULT retrieveFrame = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if (FAILED(retrieveFrame))
		{
			LOG(ERROR) << "Could not retrieve color frame! HRESULT " << retrieveFrame;
		}
		else
		{
			// Necessary instead of instant return to prevent memory leak of colorFrame
			//Convert from YUY2 -> BGRA
			colorFrame->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()), &colorBuffer[0],
			                                          ColorImageFormat_Bgra);
			colorMat = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);

			updateBufferWithSmoothedMat(colorMat, colorMat, colorBuffer);
		}
		if (colorFrame) colorFrame->Release();
	}
}

void KinectV2Handler::updateDepthData()
{
	if (depthFrameReader)
	{
		// Retrieve Depth Frame
		IDepthFrame* depthFrame;
		const HRESULT retrieveFrame = depthFrameReader->AcquireLatestFrame(&depthFrame);
		if (FAILED(retrieveFrame))
		{
			LOG(ERROR) << "Could not retrieve depth frame! HRESULT " << retrieveFrame;
		}
		else
		{
			// Necessary instead of instant return to prevent memory leak of depthFrame
			// Retrieve Depth Data
			depthFrame->CopyFrameDataToArray(static_cast<UINT>(depthBuffer.size()), &depthBuffer[0]);

			// Create cv::Mat from Depth Buffer
			auto unsmoothedDepthMat = cv::Mat(depthHeight, depthWidth, CV_16U, &depthBuffer[0]);
			updateBufferWithSmoothedMat(unsmoothedDepthMat, depthMat, depthBuffer);
		}
		if (depthFrame) depthFrame->Release();
	}
}

void KinectV2Handler::drawKinectData(sf::RenderWindow& win)
{
	if (KinectSettings::isKinectDrawn)
	{
		drawKinectImageData(win);
	}
	if (KinectSettings::isSkeletonDrawn)
	{
		drawTrackedSkeletons(win);
	}
}

void KinectV2Handler::drawKinectImageData(sf::RenderWindow& win)
{
	glBindTexture(GL_TEXTURE_2D, kinectTextureId);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, SFMLsettings::m_window_width, SFMLsettings::m_window_height, GL_BGRA,
	                GL_UNSIGNED_BYTE, static_cast<GLvoid*>(kinectImageData.get()));
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(0, 0, 0);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(SFMLsettings::m_window_width, 0, 0);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(SFMLsettings::m_window_width, SFMLsettings::m_window_height, 0.0f);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(0, SFMLsettings::m_window_height, 0.0f);
	glEnd();
}

Joint backup[JointType_Count];
sf::Vector2f vbackup[JointType_Count];
HandState lbackup = HandState_Unknown, rbackup = HandState_Unknown;

void KinectV2Handler::drawTrackedSkeletons(sf::RenderWindow& win)
{
	for (int i = 0; i < BODY_COUNT; ++i)
	{
		IBody* pBody = kinectBodies[i];
		if (pBody)
		{
			BOOLEAN bTracked = false;
			HRESULT thisBodyTracked = pBody->get_IsTracked(&bTracked);
			if (SUCCEEDED(thisBodyTracked) && bTracked)
			{
				lbackup = HandState_Unknown;
				rbackup = HandState_Unknown;

				pBody->get_HandLeftState(&lbackup);
				pBody->get_HandRightState(&rbackup);

				HRESULT jointsFound = pBody->GetJoints(_countof(backup), backup);
				if (SUCCEEDED(jointsFound))
				{
					for (int j = 0; j < _countof(backup); ++j)
					{
						vbackup[j] = BodyToScreen(backup[j].Position, SFMLsettings::m_window_width,
						                          SFMLsettings::m_window_height);
					}
				}
			}
		}
	}
	if (KinectSettings::isSkeletonDrawn)
	{
		win.pushGLStates();
		win.resetGLStates();

		drawBody(backup, vbackup, win);
		drawHand(lbackup, vbackup[JointType_HandLeft], win);
		drawHand(rbackup, vbackup[JointType_HandRight], win);

		win.popGLStates();
	}
}

void KinectV2Handler::drawHand(HandState handState, const sf::Vector2f& handPosition, sf::RenderWindow& win)
{
	sf::CircleShape circle{};
	circle.setPosition(handPosition);
	static const float c_HandSize = 30.0f;
	circle.setRadius(c_HandSize);

	switch (handState)
	{
	case HandState_Closed:
		circle.setFillColor(sf::Color::Red);
		win.draw(circle);
		break;

	case HandState_Open:
		circle.setFillColor(sf::Color::Green);
		win.draw(circle);
		break;

	case HandState_Lasso:
		circle.setFillColor(sf::Color::Blue);
		win.draw(circle);
		break;
	}
}

void KinectV2Handler::onBodyFrameArrived(IBodyFrameReader& sender, IBodyFrameArrivedEventArgs& eventArgs)
{
	updateSkeletalData();
}

void KinectV2Handler::updateSkeletalData()
{
	if (bodyFrameReader)
	{
		IBodyFrame* bodyFrame = nullptr;
		HRESULT frameReceived = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
		if (FAILED(frameReceived))
		{
			if (frameReceived == E_PENDING)
			{
				// LOG(INFO) << "Could not retrieve skeleton frame, stuck pending...";
				// Harmless
			}
			else
				LOG(ERROR) << "Could not retrieve skeleton frame! HRESULT " << frameReceived;
		}

		//IBodyFrameReference* frameRef = nullptr;
		//multiFrame->get_BodyFrameReference(&frameRef);
		//frameRef->AcquireFrame(&bodyFrame);
		//if (frameRef) frameRef->Release();
		//if (bodyFrameReader) bodyFrameReader->Release();
		if (!bodyFrame) return;

		bodyFrame->GetAndRefreshBodyData(BODY_COUNT, kinectBodies);
		newBodyFrameArrived = true;
		if (bodyFrame) bodyFrame->Release();

		updateSkeletalFilters();
	}
}

static bool flip = false;

void KinectV2Handler::updateSkeletalFilters()
{

	// Just for testing, should be removed //
	// TODO: TODO: TODO: TODO: TODO: //
	for (int i = 0; i < BODY_COUNT; i++)
	{
		if (kinectBodies[i])
			kinectBodies[i]->get_IsTracked(&isTracking);
		if (isTracking)
		{
			kinectBodies[i]->GetJoints(JointType_Count, joints);
			kinectBodies[i]->GetJointOrientations(JointType_Count, jointOrientations);

			//Smooth
			filter.update(joints, newBodyFrameArrived);
			rotationFilter.UpdateFilter(kinectBodies[i], jointOrientations);

			newBodyFrameArrived = false;

			break;
		}
	}
	// TODO: TODO: TODO: TODO: TODO: //
	// Just for testing, should be removed //
	
	KinectSettings::head_position = glm::vec3(
		joints[JointType_Head].Position.X,
		joints[JointType_Head].Position.Y,
		joints[JointType_Head].Position.Z
	);
	KinectSettings::left_hand_pose = glm::vec3(
		joints[JointType_HandLeft].Position.X,
		joints[JointType_HandLeft].Position.Y,
		joints[JointType_HandLeft].Position.Z
	);
	KinectSettings::mHandPose = glm::vec3(
		joints[JointType_HandRight].Position.X,
		joints[JointType_HandRight].Position.Y,
		joints[JointType_HandRight].Position.Z
	);
	KinectSettings::hElPose = glm::vec3(
		joints[JointType_ElbowLeft].Position.X,
		joints[JointType_ElbowLeft].Position.Y,
		joints[JointType_ElbowLeft].Position.Z
	);
	KinectSettings::mElPose = glm::vec3(
		joints[JointType_ElbowRight].Position.X,
		joints[JointType_ElbowRight].Position.Y,
		joints[JointType_ElbowRight].Position.Z
	);
	KinectSettings::left_foot_raw_pose = glm::vec3(
		joints[JointType_AnkleLeft].Position.X,
		joints[JointType_AnkleLeft].Position.Y,
		joints[JointType_AnkleLeft].Position.Z
	);
	KinectSettings::right_foot_raw_pose = glm::vec3(
		joints[JointType_AnkleRight].Position.X,
		joints[JointType_AnkleRight].Position.Y,
		joints[JointType_AnkleRight].Position.Z
	);
	KinectSettings::waist_raw_pose = glm::vec3(
		joints[JointType_SpineBase].Position.X,
		joints[JointType_SpineBase].Position.Y,
		joints[JointType_SpineBase].Position.Z
	);


	JointOrientation jointOrientationsF[3];

	// we're using direction vectors for feet so comment it out
	/*
	jointOrientationsF[0].Orientation.w = lowPassFilter[0][0].update(jointOrientations[JointType_AnkleLeft].Orientation.w);
	jointOrientationsF[0].Orientation.x = lowPassFilter[0][1].update(jointOrientations[JointType_AnkleLeft].Orientation.x);
	jointOrientationsF[0].Orientation.y = lowPassFilter[0][2].update(jointOrientations[JointType_AnkleLeft].Orientation.y);
	jointOrientationsF[0].Orientation.z = lowPassFilter[0][3].update(jointOrientations[JointType_AnkleLeft].Orientation.z);

	jointOrientationsF[1].Orientation.w = lowPassFilter[1][0].update(jointOrientations[JointType_AnkleRight].Orientation.w);
	jointOrientationsF[1].Orientation.x = lowPassFilter[1][1].update(jointOrientations[JointType_AnkleRight].Orientation.x);
	jointOrientationsF[1].Orientation.y = lowPassFilter[1][2].update(jointOrientations[JointType_AnkleRight].Orientation.y);
	jointOrientationsF[1].Orientation.z = lowPassFilter[1][3].update(jointOrientations[JointType_AnkleRight].Orientation.z);
	*/

	//hips are ok
	jointOrientationsF[2].Orientation.w = lowPassFilter[2][0].update(
		jointOrientations[JointType_SpineBase].Orientation.w);
	jointOrientationsF[2].Orientation.x = lowPassFilter[2][1].update(
		jointOrientations[JointType_SpineBase].Orientation.x);
	jointOrientationsF[2].Orientation.y = lowPassFilter[2][2].update(
		jointOrientations[JointType_SpineBase].Orientation.y);
	jointOrientationsF[2].Orientation.z = lowPassFilter[2][3].update(
		jointOrientations[JointType_SpineBase].Orientation.z);


	/***********************************************************************************************/
	glm::quat hFootRotF, mFootRotF;

	//calculate direction vectors
	glm::vec3 up(0, 1, 0),
		ankleLeftPose(
			joints[JointType_AnkleLeft].Position.X,
			joints[JointType_AnkleLeft].Position.Y,
			joints[JointType_AnkleLeft].Position.Z),

		ankleRightPose(
			joints[JointType_AnkleRight].Position.X,
			joints[JointType_AnkleRight].Position.Y,
			joints[JointType_AnkleRight].Position.Z),

		footLeftPose(
			joints[JointType_FootLeft].Position.X,
			joints[JointType_FootLeft].Position.Y,
			joints[JointType_FootLeft].Position.Z),

		footRightPose(
			joints[JointType_FootRight].Position.X,
			joints[JointType_FootRight].Position.Y,
			joints[JointType_FootRight].Position.Z),

		kneeLeftPose(
			joints[JointType_KneeLeft].Position.X,
			joints[JointType_KneeLeft].Position.Y,
			joints[JointType_KneeLeft].Position.Z),

		kneeRightPose(
			joints[JointType_KneeRight].Position.X,
			joints[JointType_KneeRight].Position.Y,
			joints[JointType_KneeRight].Position.Z);

	glm::vec3 feetRot[2] = {
		eulerAngles(glm::quat(lookAt(footLeftPose, ankleLeftPose, up))),
		eulerAngles(glm::quat(lookAt(footRightPose, ankleRightPose, up)))
	};

	glm::vec3 tibiaRotZ[2] = {
		eulerAngles(glm::quat(lookAt(
			glm::vec3(ankleLeftPose.x, ankleLeftPose.y, 1),
			glm::vec3(kneeLeftPose.x, kneeLeftPose.y, 0), up))),
		eulerAngles(glm::quat(lookAt(
			glm::vec3(ankleRightPose.x, ankleRightPose.y, 1),
			glm::vec3(kneeRightPose.x, kneeRightPose.y, 0), up)))
	};

	glm::vec3 tibiaRotX[2] = {
		eulerAngles(glm::quat(lookAt(
			glm::vec3(1, ankleLeftPose.y, ankleLeftPose.z),
			glm::vec3(0, kneeLeftPose.y, kneeLeftPose.z), up))),
		eulerAngles(glm::quat(lookAt(
			glm::vec3(1, ankleRightPose.y, ankleRightPose.z),
			glm::vec3(0, kneeRightPose.y, kneeRightPose.z), up)))
	};

	hFootRotF = glm::vec3(
		-tibiaRotX[0].x - M_PI / 3,
		-feetRot[0].y + /*2 */ KinectSettings::calibration_trackers_yaw / 180 * M_PI,
		-tibiaRotZ[0].z * 15 + M_PI);

	mFootRotF = glm::vec3(
		-tibiaRotX[1].x - M_PI / 3,
		-feetRot[1].y + /*2 */ KinectSettings::calibration_trackers_yaw / 180 * M_PI,
		-tibiaRotZ[1].z * 15 + M_PI);

	normalize(hFootRotF);
	normalize(mFootRotF);

	//smooth with lowpass filter
	/*hFootRotF.w = lowPassFilter[0][0].update(hFootRotF.w);
	hFootRotF.x = lowPassFilter[0][1].update(hFootRotF.x);
	hFootRotF.y = lowPassFilter[0][2].update(hFootRotF.y);
	hFootRotF.z = lowPassFilter[0][3].update(hFootRotF.z);

	mFootRotF.w = lowPassFilter[1][0].update(mFootRotF.w);
	mFootRotF.x = lowPassFilter[1][1].update(mFootRotF.x);
	mFootRotF.y = lowPassFilter[1][2].update(mFootRotF.y);
	mFootRotF.z = lowPassFilter[1][3].update(mFootRotF.z);*/

	//K/inectSettings::trackerSoftRot[0] = hFootRotF;
	//K/inectSettings::trackerSoftRot[1] = mFootRotF;

	/***********************************************************************************************/


	/* KINECT V2 ONLY: filter quaternion to be less jittery at end */
	/* glm::quat hFootRotF = glm::quat(
		jointOrientationsF[0].Orientation.w,
		jointOrientationsF[0].Orientation.x,
		jointOrientationsF[0].Orientation.y,
		jointOrientationsF[0].Orientation.z
	);
	glm::quat mFootRotF = glm::quat(
		jointOrientationsF[1].Orientation.w,
		jointOrientationsF[1].Orientation.x,
		jointOrientationsF[1].Orientation.y,
		jointOrientationsF[1].Orientation.z
	); */

	//hips are ok
	Eigen::Quaternionf kinect_waist_raw_ori = Eigen::Quaternionf(
		jointOrientationsF[2].Orientation.w,
		jointOrientationsF[2].Orientation.x,
		jointOrientationsF[2].Orientation.y,
		jointOrientationsF[2].Orientation.z
	);
	
	// Check for identity / equality to quat_zero

	/*if (hFootRotF != glm::quat(1, 0, 0, 0) &&
		hFootRotF != glm::inverse(glm::quat(1, 0, 0, 0)))*/
	//K/inectSettings::left_foot_raw_ori = hFootRotF;

	/*if (mFootRotF != glm::quat(1, 0, 0, 0) &&
		mFootRotF != glm::inverse(glm::quat(1, 0, 0, 0)))*/
	//K/inectSettings::right_foot_raw_ori = mFootRotF;

	//hips are ok
	if (kinect_waist_raw_ori.isApprox(Eigen::Quaternionf(1, 0, 0, 0)) &&
		kinect_waist_raw_ori.isApprox(Eigen::Quaternionf(1, 0, 0, 0).inverse()))
		KinectSettings::waist_raw_ori = kinect_waist_raw_ori;
}

sf::Vector3f KinectV2Handler::zeroKinectPosition(int trackedSkeletonIndex)
{
	return sf::Vector3f(
		joints[JointType_Head].Position.X,
		joints[JointType_Head].Position.Y,
		joints[JointType_Head].Position.Z);
}

void KinectV2Handler::setKinectToVRMultiplier(int skeletonIndex)
{
	/*
	KinectSettings::kinectToVRScale = KinectSettings::hmdZero.v[1]
		/ (joints[JointType_Head].Position.Y
			+
			-joints[JointType_AnkleLeft].Position.Y);
	std::cerr << "HMD zero: " << KinectSettings::hmdZero.v[1] << '\n';
	std::cerr << "head pos: " << joints[JointType_Head].Position.Y << '\n';
	std::cerr << "foot pos: " << joints[JointType_AnkleLeft].Position.Y << '\n';
	*/
}

bool KinectV2Handler::initKinect()
{
	if (FAILED(GetDefaultKinectSensor(&kinectSensor)))
	{
		LOG(ERROR) << "Could not get default Kinect Sensor!";
		return false;
	}
	if (kinectSensor)
	{
		kinectSensor->get_CoordinateMapper(&coordMapper);

		HRESULT hr_open = kinectSensor->Open();
		//kinectSensor->OpenMultiSourceFrameReader( FrameSourceTypes::FrameSourceTypes_Body| FrameSourceTypes::FrameSourceTypes_Depth
		//    | FrameSourceTypes::FrameSourceTypes_Color,
		//   &frameReader);
		//return frameReader;
		std::this_thread::sleep_for(std::chrono::seconds(2));
		// Necessary to allow kinect to become available behind the scenes

		BOOLEAN available = false;
		kinectSensor->get_IsAvailable(&available);

		if (FAILED(hr_open) || !available)
		{
			LOG(ERROR) << "Kinect sensor failed to open!";
			return false;
		}
		LOG(INFO) << "Kinect sensor opened successfully.";
		return true;
	}
	return false;
}

void KinectV2Handler::updateKinectData()
{
	updateDepthData();
	updateColorData();
}

void KinectV2Handler::drawBody(const Joint* pJoints, const sf::Vector2f* pJointPoints, sf::RenderWindow& window)
{
	// Draw the bones
	window.clear();

	// Torso
	drawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck, window);
	drawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder, window);
	drawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid, window);
	drawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase, window);
	drawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight, window);
	drawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft, window);
	drawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight, window);
	drawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft, window);

	// Right Arm
	drawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight, window);
	drawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight, window);
	drawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight, window);
	drawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight, window);
	drawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight, window);

	// Left Arm
	drawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft, window);
	drawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft, window);
	drawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft, window);
	drawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft, window);
	drawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft, window);

	// Right Leg
	drawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight, window);
	drawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight, window);
	drawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight, window);
	// Left Leg
	drawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft, window);
	drawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft, window);
	drawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft, window);

	// Draw the joints
	for (int i = 0; i < JointType_Count; ++i)
	{
		sf::CircleShape circle{};
		circle.setRadius(KinectSettings::g_JointThickness);
		circle.setPosition(pJointPoints[i]);

		if (pJoints[i].TrackingState == TrackingState_Inferred)
		{
			circle.setFillColor(sf::Color::Red);
			window.draw(circle);
		}
		else if (pJoints[i].TrackingState == TrackingState_Tracked)
		{
			circle.setFillColor(sf::Color::Yellow);
			window.draw(circle);
		}
	}
}

void KinectV2Handler::drawBone(const Joint* pJoints, const sf::Vector2f* pJointPoints, JointType joint0,
                               JointType joint1, sf::RenderWindow& window)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		drawLine(pJointPoints[joint0], pJointPoints[joint1], sf::Color::Green, KinectSettings::g_TrackedBoneThickness,
		         window);
	}
	else
	{
		drawLine(pJointPoints[joint0], pJointPoints[joint1], sf::Color::Green, KinectSettings::g_TrackedBoneThickness,
		         window);
	}
}

void KinectV2Handler::drawLine(sf::Vector2f start, sf::Vector2f end, sf::Color colour, float lineThickness,
                               sf::RenderWindow& window)
{
	sfLine line(start, end);
	line.setColor(colour);
	line.setThickness(lineThickness);
	window.draw(line);
}

JointType KinectV2Handler::convertJoint(KVR::KinectJoint kJoint)
{
	// Currently, the v2 SDK id's match my jointtype class 1:1
	return static_cast<JointType>(kJoint.joint);
}
