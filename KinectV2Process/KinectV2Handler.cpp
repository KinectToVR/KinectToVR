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

		// Init the second, slower filter
		SmoothingParameters bakparameters
		{
		.05f, // In meters? MS docs...
		.8f,
		.1f,
		.08f, // In meters? MS docs...
		.08f
		};
		bakfilter.init(bakparameters);
		
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
		win.popGLStates();
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

Eigen::Quaternionf yawFilteringQuaternion[2] = { Eigen::Quaternionf(1,0,0,0) }; // L, R

void KinectV2Handler::updateSkeletalFilters()
{

	/*
	 * I'm sending fast data to poses and creating a copy 'bak'
	 * of it, then smoothing it AND additionally applying a filter.
	 * Then I'm gonna create orientations from the filtered one.
	 */

	 // Just for testing, may be zeroed //
	 // https://docs.microsoft.com/en-us/previous-versions/windows/kinect-1.8/hh855623(v=ieb.10)

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

	/***********************************************************************************************/
	/*  Software/Math based feet trackers' orientation is being calculated here, from base poses.  */
	/***********************************************************************************************/

	// Additional objects for copy-ing
	Joint bakjoints[JointType_Count];
	
	for (int i = 0; i < BODY_COUNT; i++)
	{
		if (kinectBodies[i])
			kinectBodies[i]->get_IsTracked(&isTracking);
		if (isTracking)
		{
			kinectBodies[i]->GetJoints(JointType_Count, bakjoints);

			//Smooth
			bakfilter.update(bakjoints, newBodyFrameArrived);
			break;
		}
	}

	/***********************************************************************************************/
	// Setup base types we'll need
	/***********************************************************************************************/

	// Final orientations and backups
	Eigen::Quaternionf calculatedLeftFootOrientation, calculatedRightFootOrientation;

	// Position vectors for needed points
	// 10+ to make all positives, just in case
	Eigen::Vector3f up(0, 1, 0), forward(0, 0, 1), backward(0, 0, -1), // forward cuz zero quat in steamvr means just forward
		ankleLeftPose(
			bakjoints[JointType_AnkleLeft].Position.X,
			bakjoints[JointType_AnkleLeft].Position.Y,
			bakjoints[JointType_AnkleLeft].Position.Z),

		ankleRightPose(
			bakjoints[JointType_AnkleRight].Position.X,
			bakjoints[JointType_AnkleRight].Position.Y,
			bakjoints[JointType_AnkleRight].Position.Z),

		footLeftPose(
			bakjoints[JointType_FootLeft].Position.X,
			bakjoints[JointType_FootLeft].Position.Y,
			bakjoints[JointType_FootLeft].Position.Z),

		footRightPose(
			bakjoints[JointType_FootRight].Position.X,
			bakjoints[JointType_FootRight].Position.Y,
			bakjoints[JointType_FootRight].Position.Z),

		kneeLeftPose(
			bakjoints[JointType_KneeLeft].Position.X,
			bakjoints[JointType_KneeLeft].Position.Y,
			bakjoints[JointType_KneeLeft].Position.Z),

		kneeRightPose(
			bakjoints[JointType_KneeRight].Position.X,
			bakjoints[JointType_KneeRight].Position.Y,
			bakjoints[JointType_KneeRight].Position.Z);

	/***********************************************************************************************/
	// Setup base types we'll need
	/***********************************************************************************************/

	/***********************************************************************************************/
	// Calculate orientations with lookAt
	/***********************************************************************************************/

	// Calculate euler yaw foot orientation, we'll need it later
	Eigen::Vector3f
		footLeftRawOrientation = EigenUtils::DirectionQuat(
			Eigen::Vector3f(ankleLeftPose.x(), 0.f, ankleLeftPose.z()),
			Eigen::Vector3f(footLeftPose.x(), 0.f, footLeftPose.z()),
			forward).toRotationMatrix().eulerAngles(0, 1, 2),

		footRightRawOrientation = EigenUtils::DirectionQuat(
			Eigen::Vector3f(ankleRightPose.x(), 0.f, ankleRightPose.z()),
			Eigen::Vector3f(footRightPose.x(), 0.f, footRightPose.z()),
			forward).toRotationMatrix().eulerAngles(0, 1, 2);

	// Flip the yaw around, without reversing it -> we need it basing to 0
	// (what an irony that we actually need to reverse it...)
	footLeftRawOrientation.y() *= -1.f; footLeftRawOrientation.y() += M_PI;
	footRightRawOrientation.y() *= -1.f; footRightRawOrientation.y() += M_PI;

	// Make the yaw less sensitive
	// Decided to go for radians for the read-ability
	// (Although my code is shit anyway, and there'll be none in the end)
	float lsFixedYaw[2] = {
		glm::degrees(footLeftRawOrientation.y()),
		glm::degrees(footRightRawOrientation.y())
	}; // L, R

	// Left
	if (lsFixedYaw[0] > 180.f && lsFixedYaw[0] < 360.f)
		lsFixedYaw[0] = 360.f - abs(lsFixedYaw[0] - 360.f) * .5f;
	else if (lsFixedYaw[0] < 180.f && lsFixedYaw[0] > 0.f)
		lsFixedYaw[0] *= .5f;

	// Right
	if (lsFixedYaw[1] > 180.f && lsFixedYaw[1] < 360.f)
		lsFixedYaw[1] = 360.f - abs(lsFixedYaw[1] - 360.f) * .5f;
	else if (lsFixedYaw[1] < 180.f && lsFixedYaw[1] > 0.f)
		lsFixedYaw[1] *= .5f;

	// Apply to the base
	footLeftRawOrientation.y() = glm::radians(lsFixedYaw[0]); // Back to the RAD format
	footRightRawOrientation.y() = glm::radians(lsFixedYaw[1]);

	// Construct a helpful offsetting quaternion from the stuff we got
	// It's made like Quat->Eulers->Quat because we may need to adjust some things on-to-go
	Eigen::Quaternionf
		leftFootYawOffsetQuaternion = EigenUtils::EulersToQuat(footLeftRawOrientation), // There is no X and Z anyway
		rightFootYawOffsetQuaternion = EigenUtils::EulersToQuat(footRightRawOrientation);

	// Smooth a bit with a slerp
	yawFilteringQuaternion[0] = yawFilteringQuaternion[0].slerp(.25f, leftFootYawOffsetQuaternion);
	yawFilteringQuaternion[1] = yawFilteringQuaternion[1].slerp(.25f, rightFootYawOffsetQuaternion);

	// Apply to the base
	leftFootYawOffsetQuaternion = yawFilteringQuaternion[0];
	rightFootYawOffsetQuaternion = yawFilteringQuaternion[1];

	/***********************************************************************************************/
	// Calculate orientations with lookAt
	/***********************************************************************************************/

	// Calculate the knee-ankle orientation, aka "Tibia"
	// We aren't disabling look-thorough yaw, since it'll be 0
	Eigen::Quaternionf
		knee_ankleLeftOrientationQuaternion = EigenUtils::DirectionQuat(kneeLeftPose, ankleLeftPose, forward),
		knee_ankleRightOrientationQuaternion = EigenUtils::DirectionQuat(kneeRightPose, ankleRightPose, forward);

	/***********************************************************************************************/
	// Calculate orientations with lookAt
	/***********************************************************************************************/

	/***********************************************************************************************/
	// Add an offset
	/***********************************************************************************************/

	// The tuning quat
	Eigen::Quaternionf
		tuneQuaternion_first = Eigen::Quaternionf(1, 0, 0, 0);

	// Now adjust some values like playspace yaw and pitch, additional rotations
	// -> they're facing purely down and Z / Y are flipped
	tuneQuaternion_first =
		EigenUtils::EulersToQuat(
			Eigen::Vector3f(
				M_PI / 5.f,
				0.f,
				0.f
			));

	// Apply the fine-tuning to global variable
	knee_ankleLeftOrientationQuaternion = tuneQuaternion_first * knee_ankleLeftOrientationQuaternion;
	knee_ankleRightOrientationQuaternion = tuneQuaternion_first * knee_ankleRightOrientationQuaternion;

	/***********************************************************************************************/
	// Mirror 2 missed axes
	/***********************************************************************************************/

	// Grab original orientations and make them euler angles
	Eigen::Vector3f left_knee_ori_full = EigenUtils::QuatToEulers(knee_ankleLeftOrientationQuaternion);
	Eigen::Vector3f right_knee_ori_full = EigenUtils::QuatToEulers(knee_ankleRightOrientationQuaternion);

	// Try to fix yaw and roll mismatch, caused by XYZ XZY mismatch
	knee_ankleLeftOrientationQuaternion = EigenUtils::EulersToQuat(
		Eigen::Vector3f(
			left_knee_ori_full.x() - M_PI / 1.6f,
			0.0, // left_knee_ori_full.z(), // actually 0.0 but okay
			-left_knee_ori_full.y()));

	knee_ankleRightOrientationQuaternion = EigenUtils::EulersToQuat(
		Eigen::Vector3f(
			right_knee_ori_full.x() - M_PI / 1.6f,
			0.0, // right_knee_ori_full.z(), // actually 0.0 but okay
			-right_knee_ori_full.y()));

	/***********************************************************************************************/
	// Add the results
	/***********************************************************************************************/

	if (bakjoints[JointType_AnkleLeft].TrackingState == TrackingState_Tracked) {
		// All the rotations
		calculatedLeftFootOrientation = leftFootYawOffsetQuaternion * knee_ankleLeftOrientationQuaternion;
		calculatedRightFootOrientation = rightFootYawOffsetQuaternion * knee_ankleRightOrientationQuaternion;
	}
	else {
		// Without the foot's yaw
		calculatedLeftFootOrientation = knee_ankleLeftOrientationQuaternion;
		calculatedRightFootOrientation = knee_ankleRightOrientationQuaternion;
	}

	//calculatedLeftFootOrientation = leftFootYawOffsetQuaternion;
	//calculatedRightFootOrientation = rightFootYawOffsetQuaternion;

	/***********************************************************************************************/
	// Add the results / Push to global / Apply fine-tuning
	/***********************************************************************************************/

	// The tuning quat
	Eigen::Quaternionf
		leftFootFineTuneQuaternion = Eigen::Quaternionf(1, 0, 0, 0),
		rightFootFineTuneQuaternion = Eigen::Quaternionf(1, 0, 0, 0);

	// Now adjust some values like playspace yaw and pitch, additional rotations

	leftFootFineTuneQuaternion =
		EigenUtils::EulersToQuat(
			Eigen::Vector3f(
				KinectSettings::calibration_kinect_pitch, // this one's in radians alr
				0.f, //glm::radians(KinectSettings::calibration_trackers_yaw),
				0.f
			));

	rightFootFineTuneQuaternion =
		EigenUtils::EulersToQuat(
			Eigen::Vector3f(
				KinectSettings::calibration_kinect_pitch, // this one's in radians alr
				0.f, //glm::radians(KinectSettings::calibration_trackers_yaw),
				0.f
			));

	// Apply the fine-tuning to global variable
	calculatedLeftFootOrientation = leftFootFineTuneQuaternion * calculatedLeftFootOrientation;
	calculatedRightFootOrientation = rightFootFineTuneQuaternion * calculatedRightFootOrientation;

	/***********************************************************************************************/
	// Add the results / Push to global
	/***********************************************************************************************/

	// Additionally slerp for smoother orientation,
	// @see https://eigen.tuxfamily.org/dox/classEigen_1_1QuaternionBase.html
	KinectSettings::trackerSoftRot[0] = KinectSettings::trackerSoftRot[0].slerp(0.37, calculatedLeftFootOrientation);
	KinectSettings::trackerSoftRot[1] = KinectSettings::trackerSoftRot[1].slerp(0.37, calculatedRightFootOrientation);

	/***********************************************************************************************/
	// Add the results / Push to global
	/***********************************************************************************************/
	
	// hips are ok
	Eigen::Quaternionf kinect_waist_raw_ori = Eigen::Quaternionf(
		jointOrientations[JointType_SpineBase].Orientation.w,
		jointOrientations[JointType_SpineBase].Orientation.x,
		jointOrientations[JointType_SpineBase].Orientation.y,
		jointOrientations[JointType_SpineBase].Orientation.z
	);
	
	// Check for identity / equality to quat_zero

	/*if (hFootRotF != glm::quat(1, 0, 0, 0) &&
		hFootRotF != glm::inverse(glm::quat(1, 0, 0, 0)))*/
	//K/inectSettings::left_foot_raw_ori = hFootRotF;

	/*if (mFootRotF != glm::quat(1, 0, 0, 0) &&
		mFootRotF != glm::inverse(glm::quat(1, 0, 0, 0)))*/
	//K/inectSettings::right_foot_raw_ori = mFootRotF;

	// hips are ok, so check them
	if (!kinect_waist_raw_ori.isApprox(Eigen::Quaternionf(1, 0, 0, 0)) &&
		!kinect_waist_raw_ori.isApprox(Eigen::Quaternionf(1, 0, 0, 0).inverse()))
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
		circle.setPosition(
			pJointPoints[i].x - KinectSettings::g_JointThickness/2.,
			pJointPoints[i].y - KinectSettings::g_JointThickness/2.);

		if (pJoints[i].TrackingState == TrackingState_Inferred)
		{
			circle.setFillColor(sf::Color(176, 0, 0));
			window.draw(circle);
		}
		else if (pJoints[i].TrackingState == TrackingState_Tracked)
		{
			circle.setFillColor(sf::Color(255, 255, 255));
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
		drawLine(pJointPoints[joint0], pJointPoints[joint1], sf::Color(184, 184, 184), KinectSettings::g_TrackedBoneThickness,
		         window);
	}
	else
	{
		drawLine(pJointPoints[joint0], pJointPoints[joint1], sf::Color(255, 189, 0), KinectSettings::g_TrackedBoneThickness,
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
