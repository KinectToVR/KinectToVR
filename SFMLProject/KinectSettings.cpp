#include "stdafx.h"
#include "KinectSettings.h"
#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <MathEigen.h>
#include <iostream>
#include <fstream>
#include "wtypes.h"

#include <Windows.h>
#include <codecvt>
#include <openvr_types.h>

namespace KinectSettings
{
	const std::wstring CFG_NAME(L"KinectToVR.cfg");
	std::string KVRversion = "0.8.1 EX";
	vr::TrackedDeviceIndex_t trackerIndex[3] = { vr::k_unTrackedDeviceIndexInvalid }; // Trackers' indexes
	bool latencyTestPending = false, doingLatencyTest = false;
	long long latencyTestMillis = -1;
	std::chrono::steady_clock::time_point latencyTestStart, latencyTestEnd;

	/* Interfacing with the k2api */
	long long pingTime = 0, parsingTime = 0,
		lastLoopTime = 0;
	int pingCheckingThreadsNumber = 0;
	const int maxPingCheckingThreads = 3;

	/*All trackers which should be added : W, L, R*/
	std::vector<K2STracker> trackerVector{
		K2STracker(), K2STracker(), K2STracker()
	}; // This way we're gonna auto-init filters
	int trackerID[3] = { -1, -1, -1 }; // W, L, R
	std::string trackerSerial[3] = { "", "", "" }; // W, L, R

	int kinectVersion = -1;
	bool psmbuttons[5][10];
	bool isKinectDrawn = false, isServerFailure = false;
	bool isSkeletonDrawn = false;
	bool isDriverPresent = false;
	float svrhmdyaw = 0, calibration_kinect_pitch = 0;
	int K2Drivercode = -1; //unknown
	std::vector<int> psmindexidpsm[2];
	int flashnow[2];
	bool conActivated = false;
	bool ignoreInferredPositions = false;
	bool ignoreRotationSmoothing = false;
	float ardroffset = 0.f;
	int positional_tracking_option = 1, headtrackingoption = 1;
	// The joints which actually have rotation change based on the kinect
	// Each kinect type should set these in their process beginning
	// These would be the defaults for the V1
	KVR::KinectJointType leftFootJointWithRotation = KVR::KinectJointType::FootLeft;
	KVR::KinectJointType rightFootJointWithRotation = KVR::KinectJointType::FootRight;
	KVR::KinectJointType leftFootJointWithoutRotation = KVR::KinectJointType::AnkleLeft;
	KVR::KinectJointType rightFootJointWithoutRotation = KVR::KinectJointType::AnkleRight;
	bool isCalibrating = false;

	PSMPSMove right_move_controller, left_move_controller, left_foot_psmove, right_foot_psmove, waist_psmove;
	bool initialised = false, initialised_bak = false, isKinectPSMS = false;
	bool userChangingZero = false;
	bool legacy = false;
	float g_TrackedBoneThickness = 6.0f;
	float g_InferredBoneThickness = 1.5f;
	float g_JointThickness = 4.0f;
	std::string opt = "";
	const int kinectHeight = 640;
	const int kinectWidth = 480;
	bool expcalib = true;
	bool frame1 = true;
	int feet_rotation_option, hips_rotation_option, posOption = 2, conOption;

	float map(float value, float start1, float stop1, float start2, float stop2)
	{
		return (start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1)));
	}

	glm::vec3 joybk[2];

	template <class T>
	const T& constrain(const T& x, const T& a, const T& b)
	{
		if (x < a)
		{
			return a;
		}
		if (b < x)
		{
			return b;
		}
		return x;
	}

	glm::vec3 head_position, left_hand_pose, mHandPose, left_foot_raw_pose, right_foot_raw_pose, waist_raw_pose, hElPose
		, mElPose,
		lastPose[3][2];
	Eigen::Quaternionf left_foot_raw_ori, right_foot_raw_ori, waist_raw_ori;
	Eigen::Quaternionf trackerSoftRot[2];
	vr::HmdQuaternion_t hmdRot;

	const int kinectV2Height = 1920;
	const int kinectV2Width = 1080;
	bool rtconcalib = false;
	double kinectToVRScale = 1;
	double hipRoleHeightAdjust = 0.0; // in metres up - applied post-scale
	//Need to delete later (Merge should sort it)
	int leftHandPlayspaceMovementButton = 0;
	int rightHandPlayspaceMovementButton = 0;
	int leftFootPlayspaceMovementButton = 0;
	int rightFootPlayspaceMovementButton = 0;
	int psm_right_id, psm_left_id, psm_waist_id;
	float hmdYaw = 0;
	float conID[2] = { 0, 1 };

	// Default: all on, all enabled
	bool OnTrackersSave[3] = { true, true, true };
	bool EnabledTrackersSave[3] = { true, true, true };

	vr::HmdVector3d_t hmdPosition = { 0, 0, 0 };
	vr::HmdQuaternion_t hmdRotation = { 0, 0, 0, 0 };
	vr::HmdMatrix34_t hmdAbsoluteTracking = {};
	extern vr::HmdMatrix34_t trackingOrigin = {};
	extern vr::HmdVector3d_t trackingOriginPosition = { 0, 0, 0 };
	vr::HmdVector3d_t secondaryTrackingOriginOffset = { 0 };
	vr::HmdQuaternion_t kinectRepRotation{ 0, 0, 0, 0 }; //TEMP
	vr::HmdVector3d_t kinectRadRotation{ 0, 0, 0 };
	vr::HmdVector3d_t kinectRepPosition{ 0, 0, 0 };

	// R, L, H
	vr::HmdVector3d_t manual_offsets[2][3] = { {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}} };
	vr::HmdVector3d_t hoffsets{ 0, 0, 0 };
	vr::HmdVector3d_t huoffsets{ 0, 0, 0 };
	vr::HmdVector3d_t kinect_tracker_offsets{ 0, 0, 0 };
	float hroffset = 0;
	float troffset = 0;
	Eigen::Vector3f calibration_origin;
	vr::TrackedDevicePose_t controllersPose[2];
	vr::HmdVector3d_t hauoffset{ 0, 0, 0 }, mauoffset{ 0, 0, 0 };
	Eigen::Matrix<float, 3, 3> calibration_rotation;
	Eigen::Matrix<float, 3, 1> calibration_translation;
	bool ismatrixcalibrated = false;
	bool matrixes_calibrated = false;

	float calibration_trackers_yaw = 0.0;
	bool jcalib;
	int cpoints = 3;
	int p_loops = 0; // Loops passed since last status update

	vr::HmdQuaternion_t hmdquat{ 1, 0, 0, 0 };

	vr::HmdVector3d_t kinect_m_positions[3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };

	bool headtracked = false;
	bool sensorConfigChanged = true; // First time used, it's config has changed internally

	bool adjustingKinectRepresentationRot = false;
	bool adjustingKinectRepresentationPos = false;

	void updateKinectQuaternion()
	{
		kinectRepRotation = vrmath::quaternionFromYawPitchRoll(kinectRadRotation.v[1], kinectRadRotation.v[0],
			kinectRadRotation.v[2]);
	}

	//first is cutoff in hz (multiplied per 2PI) and second is our framerate about 100 fps
	bool flip;
	PSMQuatf offset[2];
	Eigen::Quaternionf quatf[2];
	glm::vec3 joy[2] = { glm::vec3(0, 0, 0), glm::vec3(0, 0, 0) };

	Eigen::Quaternionf move_ori_offset[3] = {
						   Eigen::Quaternionf(1, 0, 0, 0),
						   Eigen::Quaternionf(1, 0, 0, 0),
						   Eigen::Quaternionf(1, 0, 0, 0)
	},
		left_tracker_rot, right_tracker_rot, waist_tracker_rot;

	void sendipc()
	{

		while (true)
		{
			auto loop_start_time = std::chrono::high_resolution_clock::now();

			/*****************************************************************************************/
			// Compute poses and update trackers if we're running okay
			/*****************************************************************************************/
			
			if (// If the server is not crashed
				!isServerFailure
				 // If trackers are AFTER spawn
				&& spawned) {

				// RAW poses/oris are ones provided by kinect,
				// without anything modified

				// If we're using PSMS, let's just replace everything right away
				if (positional_tracking_option == k_PSMoveFullTracking)
				{
					left_foot_raw_pose = .01f * glm::vec3(left_foot_psmove.Pose.Position.x,
						left_foot_psmove.Pose.Position.y,
						left_foot_psmove.Pose.Position.z);
					right_foot_raw_pose = .01f * glm::vec3(right_foot_psmove.Pose.Position.x,
						right_foot_psmove.Pose.Position.y,
						right_foot_psmove.Pose.Position.z);
					waist_raw_pose = .01f * glm::vec3(waist_psmove.Pose.Position.x, waist_psmove.Pose.Position.y,
						waist_psmove.Pose.Position.z);

					left_foot_raw_ori = Eigen::Quaternionf(left_foot_psmove.Pose.Orientation.w,
						left_foot_psmove.Pose.Orientation.x,
						left_foot_psmove.Pose.Orientation.y,
						left_foot_psmove.Pose.Orientation.z);
					right_foot_raw_ori = Eigen::Quaternionf(right_foot_psmove.Pose.Orientation.w,
						right_foot_psmove.Pose.Orientation.x,
						right_foot_psmove.Pose.Orientation.y,
						right_foot_psmove.Pose.Orientation.z);
					waist_raw_ori = Eigen::Quaternionf(waist_psmove.Pose.Orientation.w, waist_psmove.Pose.Orientation.x,
						waist_psmove.Pose.Orientation.y, waist_psmove.Pose.Orientation.z);
				}

				// Update poses for interfacing
				kinect_m_positions[2].v[0] = waist_raw_pose.x;
				kinect_m_positions[2].v[1] = waist_raw_pose.y;
				kinect_m_positions[2].v[2] = waist_raw_pose.z;
				kinect_m_positions[1].v[0] = left_hand_pose.x;
				kinect_m_positions[1].v[1] = left_hand_pose.y;
				kinect_m_positions[1].v[2] = left_hand_pose.z;
				kinect_m_positions[0].v[0] = head_position.x;
				kinect_m_positions[0].v[1] = head_position.y;
				kinect_m_positions[0].v[2] = head_position.z;

				/*****************************************************************************************/
				// Resetting PSMoves orientations
				/*****************************************************************************************/
				const PSMPSMove left_psmove = left_move_controller, right_psmove = right_move_controller;

				if (right_psmove.SelectButton == PSMButtonState_DOWN) //we are recentering right psmove with select button
					offset[0] = right_psmove.Pose.Orientation; //quaterion for further offset maths

				if (left_psmove.SelectButton == PSMButtonState_DOWN)
					//we are recentering right psmove with select button
					offset[1] = left_psmove.Pose.Orientation; //quaterion for further offset maths


				if (left_foot_psmove.SelectButton == PSMButtonState_DOWN) //recenter left foot move with select button
					move_ori_offset[0] = Eigen::Quaternionf(left_foot_psmove.Pose.Orientation.w,
						left_foot_psmove.Pose.Orientation.x,
						left_foot_psmove.Pose.Orientation.y,
						left_foot_psmove.Pose.Orientation.z);

				if (right_foot_psmove.SelectButton == PSMButtonState_DOWN) //recenter right foot move with select button
					move_ori_offset[1] = Eigen::Quaternionf(right_foot_psmove.Pose.Orientation.w,
						right_foot_psmove.Pose.Orientation.x,
						right_foot_psmove.Pose.Orientation.y,
						right_foot_psmove.Pose.Orientation.z);

				if (waist_psmove.SelectButton == PSMButtonState_DOWN) //recenter waist move with select button
					move_ori_offset[2] = Eigen::Quaternionf(waist_psmove.Pose.Orientation.w,
						waist_psmove.Pose.Orientation.x,
						waist_psmove.Pose.Orientation.y,
						waist_psmove.Pose.Orientation.z);

				/*****************************************************************************************/
				// Resetting PSMoves orientations
				/*****************************************************************************************/

				using PointSet = Eigen::Matrix<float, 3, Eigen::Dynamic>; //create pointset for korejan's transform algo
				const float yaw = glm::degrees(hmdYaw); //get current headset yaw (RAD->DEG) format: 0+360
				const float facing = yaw - calibration_trackers_yaw; //get facing to kinect;
				const bool autocalib = (calibration_origin == Eigen::Vector3f(0, 0, 0)); // Zeros in auto

				// we're subtracting looking at the kinect degree from actual yaw to get offset angle:
				//       
				//             FRONT                 Front is at 0deg
				//              / \     KINECT       Kinect is at 30deg
				//               |       /           
				//               |      /            Assuming we're looking at front, we have facing -30
				//               |     /             because: front:0deg, kinect:30deg -> 0-30 = -30deg        
				//               |    /                             
				//               |   /               flip activates itself if facing is between -155 and -205deg
				//               |  /                and deactivates if facing is between 25 and -25deg
				//              ---                  AND we're not using psms for tracking
				//             CENTER                          
				//              ---                         
				//               |                          
				//               |

				// NOTE! I'm using:
				// Pitch for rotation around +x
				// Yaw for rotation around +y (yes, +)
				// Roll for rotation around +z
				// Just get used to it

				// Disable flipping when we're in PSMS mode
				// TODO: Add disabling flip via settings
				if (positional_tracking_option == k_PSMoveFullTracking
					|| !matrixes_calibrated) // If not calibrated yet, set flip to false too
					flip = false;
				else
				{
					// GUIHandler.h #3040, angle is 0-360deg
					if ( //facing <= 25 && facing >= -25 || //if we use -180+180
						(facing <= 25 && facing >= 0 || facing >= 345 && facing <= 360)) //if we use 0+360
						flip = false;
					if ( //facing <= -155 && facing >= -205 || //if we use -180+180
						facing >= 155 && facing <= 205) //if we use 0+360
						flip = true;
				}

				// One-way change to false if disabled
				if (!FlipEnabled)
					flip = false; // Disable flip whatever reason the user wants to be bugged away //

				/*****************************************************************************************/
				// Compose the string to send to driver
				/*****************************************************************************************/

					/*****************************************************************************************/
					// Modify the orientation, depending on the currently applied option - WAIST
					/*****************************************************************************************/

					// Look at #220; orientations are already composed in psms mode
				if (hips_rotation_option == k_EnableHipsOrientationFilter)
					waist_tracker_rot = waist_raw_ori;

				// If we don't want hip tracker to move, let it be
				else if (hips_rotation_option == k_DisableHipsOrientationFilter)
					waist_tracker_rot = Eigen::Quaternionf(1, 0, 0, 0);

				// If we want to use head yaw for tracker, let it use
				else if (hips_rotation_option == k_EnableOrientationFilter_HeadOrientation)
				{
					Eigen::Quaternionf hmdYawQuaternion = EigenUtils::EulersToQuat(
						Eigen::Vector3f(0.f, hmdYaw, 0.f));

					waist_tracker_rot = hmdYawQuaternion;
				}

				/*****************************************************************************************/
				// Modify the orientation, depending on the currently applied option
				/*****************************************************************************************/


				/*****************************************************************************************/
				// Modify the orientation, depending on the currently applied option - FEET
				/*****************************************************************************************/

				// If we're fully supporting the standard orientation option,
				// to remove blocking, we'll check if the kinect version is v2 / is PSMS
				if (feet_rotation_option == k_EnableOrientationFilter &&
					(kinectVersion == 1 || positional_tracking_option == k_PSMoveFullTracking))
				{
					if (positional_tracking_option == k_KinectFullTracking)
					{
						// Base case is false one, just because it looks better
						// have default case first
						if (!flip)
						{
							left_tracker_rot = left_foot_raw_ori;
							right_tracker_rot = right_foot_raw_ori;
						}
						else
						{
							// Since we're flipped, inverse the rotations
							// and swap the feet - they're recognized differently
							right_tracker_rot = left_foot_raw_ori.inverse();
							left_tracker_rot = right_foot_raw_ori.inverse();
						}
					}
					// positional since mixed is not supported - maybe in next major release
					else if (positional_tracking_option == k_PSMoveFullTracking)
					{
						// If we're using psmoves, apply the psmoves' orientations
						// Look at #220; orientations are already composed in psms mode
						left_tracker_rot = left_foot_raw_ori;
						right_tracker_rot = right_foot_raw_ori;
					}
				}

				// If we want to totally stop trackers from rotating
				// (Probably except the flip)
				else if (feet_rotation_option == k_DisableOrientationFilter)
				{
					left_tracker_rot = Eigen::Quaternionf(1, 0, 0, 0);
					right_tracker_rot = Eigen::Quaternionf(1, 0, 0, 0);
				}

				// If we're discarding yaw from our results, PSMS supported too
				// to remove blocking, we'll check if the kinect version is v2 / is PSMS
				else if (feet_rotation_option == k_EnableOrientationFilter_WithoutYaw &&
					(kinectVersion == 1 || positional_tracking_option == k_PSMoveFullTracking))
				{
					if (positional_tracking_option == k_KinectFullTracking)
					{
						// Grab original orientations and make them euler angles
						Eigen::Vector3f left_ori_with_yaw = EigenUtils::QuatToEulers(left_foot_raw_ori);
						Eigen::Vector3f right_ori_with_yaw = EigenUtils::QuatToEulers(right_foot_raw_ori);

						// Remove yaw from eulers
						auto
							left_tracker_rot_wyaw_vector =
							Eigen::Vector3f(
								left_ori_with_yaw.x(),
								0.f, // Disable the yaw
								left_ori_with_yaw.z()),

							right_tracker_rot_wyaw_vector =
							Eigen::Vector3f(
								right_ori_with_yaw.x(),
								0.f, // Disable the yaw
								right_ori_with_yaw.z());


						// Kind of a solution for flipping at too big X.
						// Found out during testing,
						// no other known mathematical reason (maybe except gimbal lock)

						/****************************************************/

						if (left_tracker_rot_wyaw_vector.x() <= 1.f
							&& left_tracker_rot_wyaw_vector.x() >= 0
							&& (left_tracker_rot_wyaw_vector.z() >= 1.f
								|| left_tracker_rot_wyaw_vector.z() <= -1.f))

							left_tracker_rot_wyaw_vector.y() += M_PI;

						/****************************************************/

						if (right_tracker_rot_wyaw_vector.x() <= 1.f
							&& right_tracker_rot_wyaw_vector.x() >= 0

							&& (right_tracker_rot_wyaw_vector.z() >= 1.f
								|| right_tracker_rot_wyaw_vector.z() <= -1.f))

							right_tracker_rot_wyaw_vector.y() += M_PI;

						/****************************************************/

						// Apply to the base
						Eigen::Quaternionf
							left_tracker_rot_wyaw = EigenUtils::EulersToQuat(
								left_tracker_rot_wyaw_vector),

							right_tracker_rot_wyaw = EigenUtils::EulersToQuat(
								right_tracker_rot_wyaw_vector);


						// If we're in flip mode, reverse and swap additionally
						if (!flip)
						{
							left_tracker_rot = left_tracker_rot_wyaw;
							right_tracker_rot = right_tracker_rot_wyaw;
						}
						else
						{
							right_tracker_rot = left_tracker_rot_wyaw.inverse();
							left_tracker_rot = right_tracker_rot_wyaw.inverse();
						}
					}
					else if (positional_tracking_option == k_PSMoveFullTracking)
					{
						// Grab original orientations and make them euler angles
						Eigen::Vector3f left_ori_with_yaw = EigenUtils::QuatToEulers(left_foot_raw_ori);
						Eigen::Vector3f right_ori_with_yaw = EigenUtils::QuatToEulers(right_foot_raw_ori);

						// Remove yaw from eulers
						left_tracker_rot = EigenUtils::EulersToQuat(
							Eigen::Vector3f(
								left_ori_with_yaw.x(),
								0.f, // Disable the yaw
								left_ori_with_yaw.z()));

						right_tracker_rot = EigenUtils::EulersToQuat(
							Eigen::Vector3f(
								right_ori_with_yaw.x(),
								0.f, // Disable the yaw
								right_ori_with_yaw.z()));
					}
				}

				// If we want to use head yaw for tracker, let it use
				else if (feet_rotation_option == k_EnableOrientationFilter_HeadOrientation)
				{
					Eigen::Quaternionf hmdYawQuaternion = EigenUtils::EulersToQuat(
						Eigen::Vector3f(0.f, hmdYaw, 0.f));

					left_tracker_rot = hmdYawQuaternion;
					right_tracker_rot = hmdYawQuaternion;
				}

				// If we have calculated the orientation by ourselves, aka math-based
				// in addition, kinect v2 uses it too by default, so let's make it use this too
				else if (feet_rotation_option == k_EnableOrientationFilter_Software ||
					feet_rotation_option == k_EnableOrientationFilter && kinectVersion == 2)
				{
					// Should actually work same as default option,
					// though without later adjustments

					// Base case is false one, just because it looks better
					// have default case first
					if (!flip)
					{
						left_tracker_rot = trackerSoftRot[0];
						right_tracker_rot = trackerSoftRot[1];
					}
					else
					{
						// Since we're flipped, inverse the rotations
						// and swap the feet - they're recognized differently
						left_tracker_rot = trackerSoftRot[1].inverse();
						right_tracker_rot = trackerSoftRot[0].inverse();
					}
				}

				// If we want yaw disabled on v2, we'll actually need to run math-based
				// and additionally disable yaw on it. this is only for the v2
				else if (feet_rotation_option == k_EnableOrientationFilter_WithoutYaw && kinectVersion == 2)
				{
					// same as upper but without yaw
					// Grab original orientations and make them euler angles
					Eigen::Vector3f left_ori_with_yaw = EigenUtils::QuatToEulers(trackerSoftRot[0]);
					Eigen::Vector3f right_ori_with_yaw = EigenUtils::QuatToEulers(trackerSoftRot[1]);

					// Remove yaw from eulers
					auto left_tracker_rot_wyaw_vector =
						Eigen::Vector3f(
							left_ori_with_yaw.x(),
							0.f, // Disable the yaw
							left_ori_with_yaw.z()),

						right_tracker_rot_wyaw_vector =
						Eigen::Vector3f(
							right_ori_with_yaw.x(),
							0.f, // Disable the yaw
							right_ori_with_yaw.z());


					// Kind of a solution for flipping at too big X.
					// Found out during testing,
					// no other known mathematical reason (maybe except gimbal lock)

					/****************************************************/

					if (left_tracker_rot_wyaw_vector.x() <= 1.f
						&& left_tracker_rot_wyaw_vector.x() >= 0
						&& (left_tracker_rot_wyaw_vector.z() >= 1.f
							|| left_tracker_rot_wyaw_vector.z() <= -1.f))

						left_tracker_rot_wyaw_vector.y() += M_PI;

					/****************************************************/

					if (right_tracker_rot_wyaw_vector.x() <= 1.f
						&& right_tracker_rot_wyaw_vector.x() >= 0

						&& (right_tracker_rot_wyaw_vector.z() >= 1.f
							|| right_tracker_rot_wyaw_vector.z() <= -1.f))

						right_tracker_rot_wyaw_vector.y() += M_PI;

					/****************************************************/

					// Apply to the base
					Eigen::Quaternionf
						left_tracker_rot_wyaw = EigenUtils::EulersToQuat(
							left_tracker_rot_wyaw_vector),

						right_tracker_rot_wyaw = EigenUtils::EulersToQuat(
							right_tracker_rot_wyaw_vector);


					// If we're in flip mode, reverse and swap additionally
					if (!flip)
					{
						left_tracker_rot = left_tracker_rot_wyaw;
						right_tracker_rot = right_tracker_rot_wyaw;
					}
					else
					{
						right_tracker_rot = left_tracker_rot_wyaw.inverse();
						left_tracker_rot = right_tracker_rot_wyaw.inverse();
					}
				}

				/*****************************************************************************************/
				// Modify the orientation, depending on the currently applied option
				/*****************************************************************************************/


				/*****************************************************************************************/
				// Modify the orientation, add the manually-applied offset
				/*****************************************************************************************/

				// Create 3 offset quats and multiply the original ones by them
				// Remember! left is the offset and right is the base

				Eigen::Vector3f offset_eulers[3] = {
					Eigen::Vector3f(
						glm::radians(manual_offsets[1][1].v[0]),
						glm::radians(manual_offsets[1][1].v[1]),
						glm::radians(manual_offsets[1][1].v[2])),
					Eigen::Vector3f(
						glm::radians(manual_offsets[1][0].v[0]),
						glm::radians(manual_offsets[1][0].v[1]),
						glm::radians(manual_offsets[1][0].v[2])),
					Eigen::Vector3f(
						glm::radians(manual_offsets[1][2].v[0]),
						glm::radians(manual_offsets[1][2].v[1]),
						glm::radians(manual_offsets[1][2].v[2]))
				};

				left_tracker_rot = EigenUtils::EulersToQuat(offset_eulers[0]) * left_tracker_rot;
				right_tracker_rot = EigenUtils::EulersToQuat(offset_eulers[1]) * right_tracker_rot;
				waist_tracker_rot = EigenUtils::EulersToQuat(offset_eulers[2]) * waist_tracker_rot;

				/*****************************************************************************************/
				// Modify the orientation, add the manually-applied offset
				/*****************************************************************************************/

				/*****************************************************************************************/
				// Modify the orientation, add the calibration yaw value (Look At Kinect, even if artificial)
				/*****************************************************************************************/

				if (positional_tracking_option == k_KinectFullTracking)
				{
					// Construct an offset quaternion with the calibration yaw
					Eigen::Quaternionf
						yawOffsetQuaternion =
						EigenUtils::EulersToQuat(Eigen::Vector3f(0.f, glm::radians(calibration_trackers_yaw), 0.f)),

						yawFlipQuaternion =
						EigenUtils::EulersToQuat(Eigen::Vector3f(0.f, M_PI, 0.f)), // Just turn around the yaw

						waistYawFlipQuaternion =
						EigenUtils::EulersToQuat(Eigen::Vector3f(
							M_PI / 11.0, 
							(autocalib ? M_PI : 0.f),
							(autocalib ? M_PI : 0.f))); // Turn around Y and Z + pitchShift

				//TODO:
				//// Construct an offset quaternion with the pitch offset
				//Eigen::Quaternionf rollOffsetQuaternion =
				//	EigenUtils::EulersToQuat(Eigen::Vector3f(0.f, 0.f, M_PI)); // Whole PI will flip them around (180deg)
				//TODO:

				// Temporary holder for the quaternion to begin
					Eigen::Quaternionf temp_orientation[3] = {
						left_tracker_rot,
						right_tracker_rot,
						waist_tracker_rot
					}; // L, R, W

					/*
					 * Apply additional things here that have to be in eulers
					 * AND be the last to be applied.
					 * For example there are: flip yaw, calibration pitch...
					 * AND are connected with joints' orientations as they are,
					 * not with the headori ones.
					 */

					if (flip)
					{
						// Optionally disable pitch in flip mode,
						// if you want not to, just set it to true
						bool pitchOn = true;
						float pitchOffOffset = 0.0, // May be applied when pitch is off
							pitchShift = 0.f;

						if (feet_rotation_option == k_EnableOrientationFilter ||
							feet_rotation_option == k_EnableOrientationFilter_WithoutYaw)
							pitchShift = M_PI / 3.0; // Normal offset

						if (feet_rotation_option == k_EnableOrientationFilter_Software ||
							(feet_rotation_option == k_EnableOrientationFilter && kinectVersion == 2))
							pitchShift = M_PI / 12.0; // Special offset (if any), 0.f to nullify

						if (feet_rotation_option != k_EnableOrientationFilter_HeadOrientation)
						{
							// Remove the pitch angle
							// Grab original orientations and make them euler angles
							Eigen::Vector3f left_ori_with_yaw = EigenUtils::QuatToEulers(temp_orientation[0]);
							Eigen::Vector3f right_ori_with_yaw = EigenUtils::QuatToEulers(temp_orientation[1]);

							//TODO:
							//if (feet_rotation_option == k_EnableOrientationFilter_Software)
							//	pitchOffOffset = M_PI / 2.0;
							//TODO:

							// Remove pitch from eulers and apply to the parent
							left_tracker_rot = EigenUtils::EulersToQuat(
								Eigen::Vector3f(
									pitchOn ? left_ori_with_yaw.x() - pitchShift : pitchOffOffset, // Disable the pitch
									left_ori_with_yaw.y(),
									(autocalib ? -1.f : 1.f) * left_ori_with_yaw.z()));

							right_tracker_rot = EigenUtils::EulersToQuat(
								Eigen::Vector3f(
									pitchOn ? right_ori_with_yaw.x() - pitchShift : pitchOffOffset, // Disable the pitch
									right_ori_with_yaw.y(),
									(autocalib ? -1.f : 1.f) * right_ori_with_yaw.z()));

							// Apply the turn-around flip quaternion
							right_tracker_rot = yawFlipQuaternion * right_tracker_rot;
							left_tracker_rot = yawFlipQuaternion * left_tracker_rot;
						}
						if (hips_rotation_option != k_EnableHipsOrientationFilter_HeadOrientation)
						{
							// Remove the pitch angle
							// Grab original orientations and make them euler angles
							Eigen::Vector3f waist_ori_with_yaw = EigenUtils::QuatToEulers(temp_orientation[0]);

							// Remove pitch from eulers and apply to the parent
							waist_tracker_rot = EigenUtils::EulersToQuat(
								Eigen::Vector3f(
									pitchOn ? waist_ori_with_yaw.x() : pitchOffOffset, // Disable the pitch
									-waist_ori_with_yaw.y(),
									-waist_ori_with_yaw.z()));

							// Apply the turn-around flip quaternion
							waist_tracker_rot = waistYawFlipQuaternion * waist_tracker_rot;
						}

						// Apply to everything, even to the one without yaw
						// it'll make the tracker face the kinect
						if (feet_rotation_option != k_EnableOrientationFilter_HeadOrientation)
						{
							temp_orientation[0] = yawOffsetQuaternion * left_tracker_rot;
							temp_orientation[1] = yawOffsetQuaternion * right_tracker_rot;
						}
						// Apply to hip tracker too
						if (hips_rotation_option != k_EnableHipsOrientationFilter_HeadOrientation)
							temp_orientation[2] = yawOffsetQuaternion * waist_tracker_rot;

						// Apply to the base
						left_tracker_rot = temp_orientation[0];
						right_tracker_rot = temp_orientation[1];
						waist_tracker_rot = temp_orientation[2];
					}
					else
					{
						// Apply to everything, even to one without yaw
						// it'll make the tracker face the kinect
						// Duplicated because it may be modified later
						if (feet_rotation_option != k_EnableOrientationFilter_HeadOrientation)
						{
							temp_orientation[0] = yawOffsetQuaternion * left_tracker_rot;
							temp_orientation[1] = yawOffsetQuaternion * right_tracker_rot;
						}
						// Apply to hip tracker too
						if (hips_rotation_option != k_EnableHipsOrientationFilter_HeadOrientation)
							temp_orientation[2] = yawOffsetQuaternion * waist_tracker_rot;

						left_tracker_rot = temp_orientation[0];
						right_tracker_rot = temp_orientation[1];
						waist_tracker_rot = temp_orientation[2];
					}
				}
				// If we're using PSMoves, apply the manual offset
				else if (positional_tracking_option == k_PSMoveFullTracking)
				{
					left_tracker_rot = move_ori_offset[0].inverse() * left_tracker_rot;
					right_tracker_rot = move_ori_offset[1].inverse() * right_tracker_rot;
					waist_tracker_rot = move_ori_offset[2].inverse() * waist_tracker_rot;
				}

				/*****************************************************************************************/
				// Modify the orientation, add the calibration yaw value (Look At Kinect, even if artificial)
				/*****************************************************************************************/

				/*****************************************************************************************/
				// Modify the orientation, add the calibration pitch value (Kinect perspective, even if artificial)
				/*****************************************************************************************/

				//// Apply only if calibrated and only if using kinect for everything
				//if (matrixes_calibrated && positional_tracking_option == k_KinectFullTracking && flip)
				//{
				//	// Construct an offset quaternion with the calibration pitch (Note: already in radians)
				//	Eigen::Quaternionf tunePitchQuaternion =
				//		EigenUtils::EulersToQuat(Eigen::Vector3f(calibration_kinect_pitch, 0.f, 0.f));

				//	// Only these two, math-based should do it on its own
				//	if (feet_rotation_option == k_EnableOrientationFilter ||
				//		feet_rotation_option == k_EnableOrientationFilter_WithoutYaw) {

				//		// Don't run on v2, it's using the math-based
				//		if (kinectVersion == 1) {
				//			left_tracker_rot = tunePitchQuaternion * left_tracker_rot;
				//			right_tracker_rot = tunePitchQuaternion * right_tracker_rot;
				//		}
				//	}
				//	// Do the same for waist tracker if wanted
				//	if (hips_rotation_option == k_EnableHipsOrientationFilter)
				//		waist_tracker_rot = tunePitchQuaternion * waist_tracker_rot;
				//}

				/*****************************************************************************************/
				// Modify the orientation, add the calibration pitch value (Kinect perspective, even if artificial)
				/*****************************************************************************************/

				/*****************************************************************************************/
				// Swap poses for flip if needed and construct the message string, check if we're calibrated
				/*****************************************************************************************/

				/*****************************************************************************************/
				// Push RAW poses to trackers
				/*****************************************************************************************/

				trackerVector.at(0).pose.position = waist_raw_pose;
				trackerVector.at(1).pose.position = left_foot_raw_pose;
				trackerVector.at(2).pose.position = right_foot_raw_pose;

				/*****************************************************************************************/
				// Push offset poses to trackers
				/*****************************************************************************************/

				// Waist
				trackerVector.at(0).positionOffset.x = manual_offsets[0][2].v[0];
				trackerVector.at(0).positionOffset.y = manual_offsets[0][2].v[1];
				trackerVector.at(0).positionOffset.z = manual_offsets[0][2].v[2];

				// Left
				trackerVector.at(1).positionOffset.x = manual_offsets[0][1].v[0];
				trackerVector.at(1).positionOffset.y = manual_offsets[0][1].v[1];
				trackerVector.at(1).positionOffset.z = manual_offsets[0][1].v[2];

				// Right
				trackerVector.at(2).positionOffset.x = manual_offsets[0][0].v[0];
				trackerVector.at(2).positionOffset.y = manual_offsets[0][0].v[1];
				trackerVector.at(2).positionOffset.z = manual_offsets[0][0].v[2];

				/*****************************************************************************************/
				// Push RAW poses to trackers
				/*****************************************************************************************/

				/*****************************************************************************************/
				// Flip
				/*****************************************************************************************/

				// Flipping trackers
				// To flip trackers without interfering their internal filters,
				// we need to 'fool' the server that it updates the same tracker,
				// with pose rapidly changed. To achieve this, we'll change particular
				// trackers' ids while updating them via k2api's longer function w/ id

				// Also, we won't update trackers via full update or updatedata,
				// this would create A GODDAMN LOAD of logs in the driver

				// Now why am I replacing rotations one more time?
				// soooo in the code, they are being replaced peacefully
				// and we're replacing the whole pose applied to trackers.
				// Since we're gonna replace the whole pose,
				// this will be replace one more time too //>_<//

				// flip ? id1 : id2 --->
				// if flip: pass id1
				// if not flip: pass id2

				/*****************************************************************************************/
				// Push RAW orientations to trackers / with Flip
				/*****************************************************************************************/

				// If poses & rots aren't frozen
				if (!trackingPaused) {

					trackerVector.at(0).pose.orientation = p_cast_type<glm::quat>(waist_tracker_rot);
					trackerVector.at(flip ? 2 : 1).pose.orientation = p_cast_type<glm::quat>(left_tracker_rot);
					trackerVector.at(flip ? 1 : 2).pose.orientation = p_cast_type<glm::quat>(right_tracker_rot);

					// Update orientation filters
					trackerVector.at(0).updateOrientationFilters();
					trackerVector.at(1).updateOrientationFilters();
					trackerVector.at(2).updateOrientationFilters();

					// If we're in flip, slow down the rotation a bit
					trackerVector.at(0).pose.orientation = trackerVector.at(0).SLERPOrientation;
					trackerVector.at(1).pose.orientation = trackerVector.at(1).SLERPOrientation;
					trackerVector.at(2).pose.orientation = trackerVector.at(2).SLERPOrientation;

					/*****************************************************************************************/
					// Filters & update
					/*****************************************************************************************/

					trackerVector.at(0).updatePositionFilters();
					trackerVector.at(1).updatePositionFilters();
					trackerVector.at(2).updatePositionFilters();

					// Update pose w/ filtering
					// WAIST TRACKER (0)
					if (EnabledTrackersSave[0]) {
						if (matrixes_calibrated)
							ktvr::update_tracker_pose<false>(
								trackerVector.at(0).getTrackerBase
								(
									calibration_rotation,
									calibration_translation,
									calibration_origin,
									posOption, t_NoOrientationTrackingFilter
								));

						else ktvr::update_tracker_pose<false>(
							trackerVector.at(0).getTrackerBase(
								posOption, t_NoOrientationTrackingFilter));
					}

					// Update pose w/ filtering
					// LEFT TRACKER (1)
					if (EnabledTrackersSave[1]) {
						if (matrixes_calibrated)
							ktvr::update_tracker_pose<false>(
								trackerVector.at(1).id,
								ktvr::K2PosePacket(
									trackerVector.at(flip ? 2 : 1).getTrackerBase
									(
										calibration_rotation,
										calibration_translation,
										calibration_origin,
										posOption, t_NoOrientationTrackingFilter
									).pose));

						else ktvr::update_tracker_pose<false>(
							trackerVector.at(1).id,
							ktvr::K2PosePacket(
								trackerVector.at(flip ? 2 : 1).getTrackerBase(
									posOption, t_NoOrientationTrackingFilter).pose));
					}

					// Update pose w/ filtering
					// RIGHT TRACKER (2)
					if (EnabledTrackersSave[2]) {
						if (matrixes_calibrated)
							ktvr::update_tracker_pose<false>(
								trackerVector.at(2).id,
								ktvr::K2PosePacket(
									trackerVector.at(flip ? 1 : 2).getTrackerBase
									(
										calibration_rotation,
										calibration_translation,
										calibration_origin,
										posOption, t_NoOrientationTrackingFilter
									).pose));

						else ktvr::update_tracker_pose<false>(
							trackerVector.at(2).id,
							ktvr::K2PosePacket(
								trackerVector.at(flip ? 1 : 2).getTrackerBase(
									posOption, t_NoOrientationTrackingFilter).pose));
					}

				}

				// If poses & rots are frozen
				else
				{
					// Refresh the tracker
					// WAIST TRACKER (0)
					if (EnabledTrackersSave[0])
						ktvr::refresh_tracker_pose<false>(
							trackerVector.at(0).id);

					// Refresh the tracker
					// LEFT TRACKER (1)
					if (EnabledTrackersSave[1])
						ktvr::refresh_tracker_pose<false>(
							trackerVector.at(1).id);

					// Refresh the tracker
					// RIGHT TRACKER (2)
					if (EnabledTrackersSave[2])
						ktvr::refresh_tracker_pose<false>(
							trackerVector.at(2).id);
				}

				// Update status 1/1000 loops / ~8s
				// or right after any change
				for (int i = 0; i < 3; i++) { // try 3 times
					if (p_loops >= 1000 ||
						(initialised_bak != initialised)) {
						// Update status in server

						// Would be in a loop but somehow bugs everything if it is
						if (EnabledTrackersSave[0])
							ktvr::set_tracker_state<false>(trackerVector.at(0).id, OnTrackersSave[0] && initialised);
						if (EnabledTrackersSave[1])
							ktvr::set_tracker_state<false>(trackerVector.at(1).id, OnTrackersSave[1] && initialised);
						if (EnabledTrackersSave[2])
							ktvr::set_tracker_state<false>(trackerVector.at(2).id, OnTrackersSave[2] && initialised);

						// Update internal status
						initialised_bak = initialised;

						// Reset
						p_loops = 0;
					}
					else p_loops++;
				}

				/*****************************************************************************************/
				// Filters & update
				/*****************************************************************************************/

				/*****************************************************************************************/
				// Swap poses for flip if needed and construct the message string, check if we're calibrated
				/*****************************************************************************************/

			}
			
			/*****************************************************************************************/
			// Compute poses and update trackers if we're running okay
			/*****************************************************************************************/
			
			// Wait until certain loop time has passed
			if (auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::high_resolution_clock::now() - loop_start_time).count();
				duration <= 12222222.f) // If we were too fast, sleep peacefully @80hz
				std::this_thread::sleep_for(std::chrono::nanoseconds(12222222 - duration));
		}
	}

	void serializeKinectSettings()
	{
		std::ifstream is(KVR::fileToDirPath(CFG_NAME));
		LOG(INFO) << "Attempted CFG load: " << KVR::fileToDirPath(CFG_NAME) << '\n';
		//CHECK IF VALID
		if (is.fail())
		{
			//FAIL!!!!
			LOG(ERROR) << "ERROR: COULD NOT OPEN CONFIG FILE, GENERATING NEW ONE...";
			writeKinectSettings();
		}
		else
		{
			LOG(INFO) << "CFG Loaded Attempted!";

			using namespace KinectSettings;
			using namespace SFMLsettings;
			float rot[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
			float pos[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
			double hipHeight = 0;
			float fontScale = 12.f;

			try
			{
				cereal::JSONInputArchive archive(is);
				archive(rot);
				archive(pos);
				archive(hipHeight);
				archive(globalFontSize);
				archive(secondaryTrackingOriginOffset);
			}
			catch (cereal::RapidJSONException& e)
			{
				LOG(ERROR) << "CONFIG FILE LOAD JSON ERROR: " << e.what();
			}

			manual_offsets[0][0] = { pos[0][0], pos[0][1], pos[0][2] };
			manual_offsets[0][1] = { pos[1][0], pos[1][1], pos[1][2] };
			manual_offsets[0][2] = { pos[2][0], pos[2][1], pos[2][2] };

			manual_offsets[1][0] = { rot[0][0], rot[0][1], rot[0][2] };
			manual_offsets[1][1] = { rot[1][0], rot[1][1], rot[1][2] };
			manual_offsets[1][2] = { rot[2][0], rot[2][1], rot[2][2] };

			hipRoleHeightAdjust = hipHeight;
			sensorConfigChanged = true;
		}
	}

	void writeKinectSettings()
	{
		std::ofstream os(KVR::fileToDirPath(CFG_NAME));
		if (os.fail())
		{
			//FAIL!!!
			LOG(ERROR) << "ERROR: COULD NOT WRITE TO CONFIG FILE\n";
		}
		else
		{
			using namespace KinectSettings;
			using namespace SFMLsettings;
			float kRotation[3][3] = {
				{manual_offsets[1][0].v[0], manual_offsets[1][0].v[1], manual_offsets[1][0].v[2]},
				{manual_offsets[1][1].v[0], manual_offsets[1][1].v[1], manual_offsets[1][1].v[2]},
				{manual_offsets[1][2].v[0], manual_offsets[1][2].v[1], manual_offsets[1][2].v[2]},
			};

			float kPosition[3][3] = {
				{manual_offsets[0][0].v[0], manual_offsets[0][0].v[1], manual_offsets[0][0].v[2]},
				{manual_offsets[0][1].v[0], manual_offsets[0][1].v[1], manual_offsets[0][1].v[2]},
				{manual_offsets[0][2].v[0], manual_offsets[0][2].v[1], manual_offsets[0][2].v[2]},
			};

			cereal::JSONOutputArchive archive(os);
			LOG(INFO) << "Attempted to save config settings to file";
			try
			{
				archive(
					CEREAL_NVP(kRotation),
					CEREAL_NVP(kPosition),
					CEREAL_NVP(hipRoleHeightAdjust),
					CEREAL_NVP(globalFontSize),
					CEREAL_NVP(secondaryTrackingOriginOffset)
				);
			}
			catch (cereal::RapidJSONException& e)
			{
				LOG(ERROR) << "CONFIG FILE SAVE JSON ERROR: " << e.what();
			}
		}
	}
}

namespace SFMLsettings
{
	int m_window_width = 800;
	int m_window_height = 600;
	float windowScale = .4f;
	bool keepRunning = true;

	float globalFontSize = 12.f;

	std::wstring fileDirectoryPath;

	bool usingGamepad = false;
	std::stringstream debugDisplayTextStream;
}

namespace vr
{
	bool operator==(const HmdVector3d_t& lhs, const HmdVector3d_t& rhs)
	{
		return lhs.v[0] == rhs.v[0]
			&& lhs.v[1] == rhs.v[1]
			&& lhs.v[2] == rhs.v[2];
	}

	bool operator==(const HmdQuaternion_t& lhs, const HmdQuaternion_t& rhs)
	{
		return lhs.w == rhs.w
			&& lhs.x == rhs.x
			&& lhs.y == rhs.y
			&& lhs.z == rhs.z;
	}
}

namespace KVR
{
	std::wstring trackerConfig = L"lastTrackers.cfg";

	std::wstring fileToDirPath(const std::wstring& relativeFilePath)
	{
		CreateDirectory(std::wstring(std::wstring(_wgetenv(L"APPDATA")) + std::wstring(L"\\KinectToVR\\")).c_str(),
			nullptr);
		return std::wstring(_wgetenv(L"APPDATA")) + L"\\KinectToVR\\" + relativeFilePath;
	}

	std::wstring ToUTF16(const std::string& data)
	{
		return std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(data);
	}

	std::string ToUTF8(const std::wstring& data)
	{
		return std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(data);
	}

	std::string inputDirForOpenVR(const std::string& file)
	{
		std::string pathStr = ToUTF8(SFMLsettings::fileDirectoryPath) + "Input\\" + file;
		std::cout << file << " PATH: " << pathStr << '\n';
		return pathStr;
	}

	TrackingSystemCalibration retrieveSystemCalibration(const std::string& systemName)
	{
		std::wstring trackingSystemConfig = ToUTF16(systemName) + L".tracking";
		std::ifstream is(fileToDirPath(trackingSystemConfig));
		LOG(INFO) << "Attempted tracking system load: " << fileToDirPath(trackingSystemConfig) << '\n';

		TrackingSystemCalibration calibration;

		//CHECK IF VALID
		if (is.fail())
		{
			//FAIL!!!!
			LOG(ERROR) << "ERROR: COULD NOT OPEN " << systemName << " TRACKING FILE, GENERATING NEW ONE...";
			saveSystemCalibration(systemName, calibration);
		}
		else
		{
			LOG(INFO) << systemName << "Tracking Load Attempted!";

			vr::HmdQuaternion_t driverFromWorldRotation = { 1, 0, 0, 0 };
			vr::HmdVector3d_t driverFromWorldPosition = { 0, 0, 0 };

			try
			{
				cereal::JSONInputArchive archive(is);
				archive(CEREAL_NVP(driverFromWorldRotation));
				archive(CEREAL_NVP(driverFromWorldPosition));
			}
			catch (cereal::Exception& e)
			{
				LOG(ERROR) << systemName << "TRACKING FILE LOAD JSON ERROR: " << e.what();
			}

			calibration.systemName = systemName;
			calibration.driverFromWorldRotation = driverFromWorldRotation;
			calibration.driverFromWorldPosition = driverFromWorldPosition;
		}
		return calibration;
	}

	void saveSystemCalibration(const std::string& systemName, TrackingSystemCalibration calibration)
	{
		std::wstring trackingSystemConfig = ToUTF16(systemName) + L".tracking";
		std::ofstream os(fileToDirPath(trackingSystemConfig));
		if (os.fail())
		{
			//FAIL!!!
			LOG(ERROR) << "ERROR: COULD NOT WRITE TO TRACKING SYSTEM FILE\n";
		}
		else
		{
			cereal::JSONOutputArchive archive(os);
			LOG(INFO) << "Attempted to save " << systemName << " tracking system to file";

			vr::HmdQuaternion_t driverFromWorldRotation = calibration.driverFromWorldRotation;
			vr::HmdVector3d_t driverFromWorldPosition = calibration.driverFromWorldPosition;

			try
			{
				archive(CEREAL_NVP(driverFromWorldRotation));
				archive(CEREAL_NVP(driverFromWorldPosition));
			}
			catch (cereal::RapidJSONException& e)
			{
				LOG(ERROR) << systemName << "TRACKING FILE SAVE JSON ERROR: " << e.what();
			}
		}
	}
}

# define M_PI           3.14159265358979323846

namespace VRInput
{
	bool legacyInputModeEnabled;

	// Action Handles
	vr::VRActionHandle_t moveHorizontallyHandle;
	vr::VRActionHandle_t moveVerticallyHandle;
	vr::VRActionHandle_t confirmCalibrationHandle;

	// Calibration Sets
	vr::VRActionSetHandle_t calibrationSetHandle;

	// Action Sets
	vr::VRActiveActionSet_t activeActionSet;

	// Digital Action Data
	vr::InputDigitalActionData_t confirmCalibrationData{};

	// Analog Action Data
	vr::InputAnalogActionData_t moveHorizontallyData{};
	vr::InputAnalogActionData_t moveVerticallyData{};

	vr::InputAnalogActionData_t trackpadpose[2]{};
	vr::InputDigitalActionData_t confirmdatapose{};
}

bool VRInput::initialiseVRInput()
{
	std::string path = KVR::inputDirForOpenVR("action-manifest.json");
	const char* c_path = path.c_str();
	vr::EVRInputError iError = vr::VRInput()->SetActionManifestPath(c_path);
	if (iError == vr::EVRInputError::VRInputError_None)
	{
		LOG(INFO) << "Action manifest path set correctly!";
	}
	else
	{
		LOG(ERROR) << "Action manifest path Error, EVRInputError Code: " << static_cast<int>(iError);
		return false;
	}
	// Obtain handles
	iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/MoveHorizontally", &moveHorizontallyHandle);
	iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/MoveVertically", &moveVerticallyHandle);
	iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/ConfirmCalibration", &confirmCalibrationHandle);

	iError = vr::VRInput()->GetActionSetHandle("/actions/calibration", &calibrationSetHandle);

	// Set Actionset Settings
	activeActionSet.ulActionSet = calibrationSetHandle;
	activeActionSet.ulRestrictedToDevice = vr::k_ulInvalidInputValueHandle;
	activeActionSet.unPadding; // Ignored
	activeActionSet.nPriority = 0;

	if (iError == vr::EVRInputError::VRInputError_None)
	{
		LOG(INFO) << "Input Handles set correctly!";
	}
	else
	{
		LOG(ERROR) << "Input Handle Error, EVRInputError Code: " << static_cast<int>(iError);
		return false;
	}
	return true;
}

void VRInput::updateVRInput()
{
	vr::EVRInputError iError = vr::VRInput()->UpdateActionState(&activeActionSet, sizeof(activeActionSet), 1);
	if (iError != vr::EVRInputError::VRInputError_None)
	{
		LOG(ERROR) << "Error when updating input action state, EVRInputError Code: " << static_cast<int>(iError);
		return;
	}
	vr::InputAnalogActionData_t moveHorizontallyData{};
	iError = vr::VRInput()->GetAnalogActionData(
		moveHorizontallyHandle,
		&moveHorizontallyData,
		sizeof(moveHorizontallyData),
		vr::k_ulInvalidInputValueHandle);

	vr::InputAnalogActionData_t moveVerticallyData{};
	iError = vr::VRInput()->GetAnalogActionData(
		moveVerticallyHandle,
		&moveVerticallyData,
		sizeof(moveVerticallyData),
		vr::k_ulInvalidInputValueHandle);

	vr::InputDigitalActionData_t confirmPosData{};
	iError = vr::VRInput()->GetDigitalActionData(
		confirmCalibrationHandle,
		&confirmPosData,
		sizeof(confirmPosData),
		vr::k_ulInvalidInputValueHandle);

	// Ugly Hack until Valve fixes this behaviour ---------
	if (iError == vr::EVRInputError::VRInputError_InvalidHandle)
	{
		// SteamVR's latest wonderful bug/feature:
		// Switches to Legacy mode on any application it doesn't recognize
		// Meaning that the new system isn't used at all...
		// Why god. Why do you taunt me so?
		legacyInputModeEnabled = true;
	}
	else
	{
		legacyInputModeEnabled = false;
	}
	// -----------------------------------------------------
}
