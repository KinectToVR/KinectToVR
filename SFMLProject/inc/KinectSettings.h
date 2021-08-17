#pragma once
#include "stdafx.h"
#include <openvr.h>
#include <SFML/Graphics/Text.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/detail/type_vec3.hpp>
#include <glm/detail/type_vec4.hpp>
#include <glm/detail/type_vec2.hpp>
#include <string>
#include <sstream>
#include <Eigen/Geometry>
#include "KinectJoint.h"
#include <PSMoveClient_CAPI.h>
#include <EigenGLHelpers.h>

#include <Windows.h>
#include <mmsystem.h>
#include <filesystem>

// Hook into KTVR and use its K2STracker object
// (for easier tracker management)
// This also gives us some built-in filters
#include <../KTVR/KinectToVR/K2STracker.h>

// Casting Eigen<->GLM<->OpenVR
#include <codecvt>
#include <TypeCast.h>

enum KinectVersion
{
	Version1 = 1,
	//AKA Xbox 360/ Windows v1
	Version2 = 2,
	//AKA Xbox One/ Windows v2
	INVALID = 404
};

enum footRotationFilterOption
{
	k_EnableOrientationFilter,
	///enable all rotation filter dimensions
	k_DisableOrientationFilter,
	///disable joints rotation
	k_EnableOrientationFilter_WithoutYaw,
	///don't rotate foots in +y
	k_EnableOrientationFilter_HeadOrientation,
	///use headset orientation for feet
	
	/*
	 * WTF IS GOING ON HERE, IT'S COMMENTED OUT
	*/

	//k_HipTrackerOrientation,
	///use the kinect/vive/owotrack hip yaw instead of head
	//k_HipTrackerOrientationMixed,
	///same as previous but keep orientation filter for pitch/roll

	/*
	 * WTF IS GOING ON HERE, IT'S COMMENTED OUT
	*/
	
	k_EnableOrientationFilter_Software
	///use calculated orientation for feet
};

enum hipsRotationFilterOption
{
	k_EnableHipsOrientationFilter,
	///enable all rotation filter dimensions
	k_DisableHipsOrientationFilter,
	///disable joint rotation
	k_EnableHipsOrientationFilter_HeadOrientation,
	///use headset orientation for hip tracker
};

static struct footRotFilter
{
	footRotationFilterOption filterOption;
} footOrientationFilterOption;

static struct hipsRotFilter
{
	hipsRotationFilterOption filterOption;
} hipsOrientationFilterOption;

enum positionalFilterOption
{
	k_EnablePositionFilter_Kalman,
	///use EKF filtering for position
	k_EnablePositionFilter_LowPass,
	///use LowPass filtering for position
	k_EnablePositionFilter_LERP,
	///use Interpolation filtering for position
	k_DisablePositionFilter,
	///disable filtering for position
};

// Map positional filters from K2EX to KTVR



static struct posFilter
{
	positionalFilterOption filterOption;
} positionFilterOption;

// Ancient things left from the ArduVR
// BELOW

enum controllersTrackingOption
{
	k_PSMoveFull,
	k_PSMoveRot_KinectPose
};

static struct trackingOpt
{
	controllersTrackingOption trackingOption;
} controllersTrackingOption_s;

// UP
// Ancient things left from the ArduVR

enum bodyTrackingOption
{
	k_PSMoveFullTracking,
	k_KinectFullTracking
};

static struct bodyTrackingOpt
{
	bodyTrackingOption trackingOption;
} bodyTrackingOption_s;

inline std::wstring s2ws(const std::string& utf8Str)
{
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> conv;
	return conv.from_bytes(utf8Str);
}

inline std::string ws2s(const std::wstring& utf16Str)
{
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> conv;
	return conv.to_bytes(utf16Str);
}

namespace KinectSettings
{
	static struct K2VR_PSMoveData
	{
		PSMPSMove PSMoveData;
		bool isValidController = false;
	} KVRPSMoveData[11];

	/* Interfacing with the k2api */
	extern long long pingTime, parsingTime,
		lastLoopTime;
	extern int pingCheckingThreadsNumber;
	extern const int maxPingCheckingThreads;

	/*All trackers which should be added : W, L, R*/
	extern std::vector<K2STracker> trackerVector;
	extern int trackerID[3]; // W, L, R
	extern std::string trackerSerial[3]; // W, L, R

	extern vr::TrackedDeviceIndex_t trackerIndex[3]; // Trackers' indexes : L, R, W
	extern bool latencyTestPending, doingLatencyTest;
	extern long long latencyTestMillis;
	
	extern std::chrono::steady_clock::time_point latencyTestStart, latencyTestEnd;
	
	static std::vector<K2VR_PSMoveData> KVR_PSMoves;
	extern bool isCalibrating, isKinectPSMS, isServerFailure;
	extern int K2Drivercode, kinectVersion;
	extern PSMPSMove right_move_controller, left_move_controller, left_foot_psmove, right_foot_psmove, waist_psmove;
	extern Eigen::Quaternionf left_tracker_rot, right_tracker_rot, waist_tracker_rot;
	extern Eigen::Quaternionf trackerSoftRot[2]; //Software-calculated
	extern bool isDriverPresent;
	extern bool isKinectDrawn;
	
	extern bool OnTrackersSave[3];
	extern bool EnabledTrackersSave[3];
	
	extern bool isSkeletonDrawn;
	extern bool ignoreInferredPositions;
	extern bool ignoreRotationSmoothing;
	extern std::string opt;
	extern KVR::KinectJointType leftFootJointWithRotation;
	extern KVR::KinectJointType rightFootJointWithRotation;
	extern KVR::KinectJointType leftFootJointWithoutRotation;
	extern KVR::KinectJointType rightFootJointWithoutRotation;
	extern float svrhmdyaw;
	extern bool userChangingZero;
	extern bool headtracked;
	extern float g_TrackedBoneThickness;
	extern float g_InferredBoneThickness;
	extern float g_JointThickness;
	extern float hmdYaw, calibration_kinect_pitch;
	extern int flashnow[2];
	extern bool conActivated;
	extern std::vector<int> psmindexidpsm[2];
	extern float ardroffset;
	extern const int kinectHeight;
	extern const int kinectWidth;
	extern bool legacy;
	extern const int kinectV2Height;
	extern const int kinectV2Width;
	extern bool rtconcalib;
	extern double kinectToVRScale;
	extern bool initialised, initialised_bak;
	extern bool psmbuttons[5][10];
	extern float conID[2];
	extern double hipRoleHeightAdjust;
	extern float calibration_trackers_yaw;
	extern int feet_rotation_option, hips_rotation_option, posOption, conOption;
	extern int positional_tracking_option, headtrackingoption;
	//Need to delete later (Merge should sort it)
	extern int leftHandPlayspaceMovementButton;
	extern int rightHandPlayspaceMovementButton;
	extern int leftFootPlayspaceMovementButton;
	extern int rightFootPlayspaceMovementButton;
	extern vr::TrackedDevicePose_t controllersPose[2];
	extern bool frame1;
	extern vr::HmdVector3d_t hmdPosition;
	extern vr::HmdQuaternion_t hmdRotation, hmdRot;
	extern vr::HmdMatrix34_t hmdAbsoluteTracking;
	extern vr::HmdMatrix34_t trackingOrigin;
	extern vr::HmdVector3d_t trackingOriginPosition;
	// Input Emulator is by default offset from this - so 0,0,0 in IE is really these coords
	extern vr::HmdVector3d_t secondaryTrackingOriginOffset;
	// Demonic offset, actual origin unknown. Probably evil and trying to destroy everything I love.
	extern vr::HmdVector3d_t hauoffset, mauoffset;
	extern vr::HmdQuaternion_t kinectRepRotation;
	extern vr::HmdVector3d_t kinectRadRotation;
	extern vr::HmdVector3d_t kinectRepPosition;
	extern vr::HmdVector3d_t manual_offsets[2][3];
	extern vr::HmdVector3d_t hoffsets;
	extern vr::HmdVector3d_t huoffsets;
	extern vr::HmdVector3d_t kinect_tracker_offsets;
	extern float hroffset;
	extern float troffset;
	extern vr::HmdQuaternion_t hmdquat;
	extern bool expcalib;
	extern bool jcalib;
	extern Eigen::Matrix<float, 3, 3> calibration_rotation;
	extern Eigen::Matrix<float, 3, 1> calibration_translation;
	extern bool ismatrixcalibrated;
	extern Eigen::Vector3f calibration_origin;
	extern int cpoints;
	extern bool matrixes_calibrated;
	extern int psm_right_id, psm_left_id, psm_waist_id, psmatama;

	extern float hmdegree;
	extern bool sensorConfigChanged;

	extern vr::HmdVector3d_t kinect_m_positions[3];

	extern bool adjustingKinectRepresentationRot;
	extern bool adjustingKinectRepresentationPos;
	void updateKinectQuaternion();

	extern std::string KVRversion;
	extern glm::vec3 head_position, left_hand_pose, mHandPose, left_foot_raw_pose, right_foot_raw_pose, waist_raw_pose, hElPose, mElPose,
	                 lastPose[3][2];
	extern Eigen::Quaternionf left_foot_raw_ori, right_foot_raw_ori, waist_raw_ori;

	void sendipc();

	void serializeKinectSettings();
	void writeKinectSettings();

	// When a reconnect is pending
	inline bool reconnecting = false;
	// If the tracing is paused
	inline bool trackingPaused = false;
	// If the flip is enabled
	inline bool FlipEnabled = true;
	// If trackers are spawned
	inline bool spawned = false;
	// MCalibration derivatives
	inline bool calibration_confirm = false,
		calibration_modeSwap = false,
		calibration_fineTune = false;
	inline float calibration_leftJoystick[2] = { 0.f,0.f },
		calibration_rightJoystick[2] = { 0.f,0.f };

	// https://stackoverflow.com/questions/8498300/allow-for-range-based-for-with-enum-classes
	enum class IK2EXSoundType
	{
		k2ex_sound_invalid, // always first, not mapped
		k2ex_sound_startup,
		k2ex_sound_trackers_spawned,
		k2ex_sound_trackers_destroyed,
		k2ex_sound_calibration_start,
		k2ex_sound_calibration_complete,
		k2ex_sound_calibration_aborted,
		k2ex_sound_calibration_tick_move,
		k2ex_sound_calibration_tick_stand,
		k2ex_sound_calibration_point_captured,
		k2ex_sound_tracking_freeze_toggle,
		k2ex_sound_flip_toggle,
		k2ex_sound_server_error,
		k2ex_sound_kinect_error,
		k2ex_sound_count // always last, not mapped
	};

	inline bool k2ex_SoundsEnabled = true;
	inline const boost::unordered_map<IK2EXSoundType, const char*>
		IK2EXSoundType_String = boost::assign::map_list_of
		(IK2EXSoundType::k2ex_sound_startup, "k2ex_startup")
		(IK2EXSoundType::k2ex_sound_trackers_spawned, "k2ex_trackers_spawned")
		(IK2EXSoundType::k2ex_sound_trackers_destroyed, "k2ex_trackers_destroyed")
		(IK2EXSoundType::k2ex_sound_calibration_start, "k2ex_calibration_start")
		(IK2EXSoundType::k2ex_sound_calibration_complete, "k2ex_calibration_complete")
		(IK2EXSoundType::k2ex_sound_calibration_aborted, "k2ex_calibration_aborted")
		(IK2EXSoundType::k2ex_sound_calibration_tick_move, "k2ex_calibration_tick_move")
		(IK2EXSoundType::k2ex_sound_calibration_tick_stand, "k2ex_calibration_tick_stand")
		(IK2EXSoundType::k2ex_sound_calibration_point_captured, "k2ex_calibration_point_captured")
		(IK2EXSoundType::k2ex_sound_tracking_freeze_toggle, "k2ex_tracking_freeze_toggle")
		(IK2EXSoundType::k2ex_sound_flip_toggle, "k2ex_flip_toggle")
		(IK2EXSoundType::k2ex_sound_server_error, "k2ex_server_error")
		(IK2EXSoundType::k2ex_sound_kinect_error, "k2ex_kinect_error");

	// Sounds
	inline void k2ex_LoadSounds()
	{
		if (k2ex_SoundsEnabled)
			for (int i = (int)IK2EXSoundType::k2ex_sound_invalid + 1; i < (int)IK2EXSoundType::k2ex_sound_count; i++) {
				if (!std::filesystem::exists((std::string("sounds\\") + IK2EXSoundType_String.at(static_cast<IK2EXSoundType>(i)) + ".wav").c_str()))
					LOG(ERROR) << std::string("Sound file with name [") + IK2EXSoundType_String.at(static_cast<IK2EXSoundType>(i)) + ".wav" + "] was not found inside sounds/ folder.";
			}
	}

	// Play sound
	inline void k2ex_PlaySound(IK2EXSoundType sound)
	{
		if (k2ex_SoundsEnabled) {
			if (std::filesystem::exists((std::string("sounds\\") + IK2EXSoundType_String.at(sound) + ".wav").c_str())) {
				if (!PlaySoundA((std::string("sounds\\") + IK2EXSoundType_String.at(sound) + ".wav").c_str(), NULL, SND_FILENAME | SND_ASYNC))
					LOG(ERROR) << std::string("Sound file with name [") + IK2EXSoundType_String.at(sound) + ".wav" + "] could not be played.";
			}
			else
				LOG(ERROR) << std::string("Sound file with name [") + IK2EXSoundType_String.at(sound) + ".wav" + "] was not found inside sounds/ folder.";
		}
	}
}

namespace SFMLsettings
{
	extern int m_window_width;
	extern int m_window_height;
	extern float windowScale;

	extern float globalFontSize;

	extern bool keepRunning;

	extern bool usingGamepad;

	extern std::wstring fileDirectoryPath;

	extern std::stringstream debugDisplayTextStream;
}

namespace vr
{
	// For Cereal to actually use the serialization methods, they have to
	// be in the same namespace as the declared object...
	template <class Archive>
	void serialize(Archive& archive,
	               HmdQuaternion_t& q)
	{
		archive(q.w, q.x, q.y, q.z);
	}

	template <class Archive>
	void serialize(Archive& archive,
	               HmdVector3d_t& v)
	{
		archive(v.v[0], v.v[1], v.v[2]);
	}

	bool operator==(const HmdVector3d_t& lhs, const HmdVector3d_t& rhs);
	bool operator==(const HmdQuaternion_t& lhs, const HmdQuaternion_t& rhs);
}

namespace KVR
{
	extern std::wstring trackerConfig;

	std::wstring fileToDirPath(const std::wstring& relativeFilePath);
	extern std::wstring ToUTF16(const std::string& data);

	extern std::string ToUTF8(const std::wstring& data);
	extern std::string inputDirForOpenVR(const std::string& file);

	// Each tracking system has it's global adjustments here, in the form
	// of their driver-from-world offsets, so that they can be reapplied at startup
	struct TrackingSystemCalibration
	{
		std::string systemName = "INVALID";
		vr::HmdQuaternion_t driverFromWorldRotation = {1, 0, 0, 0};
		vr::HmdVector3d_t driverFromWorldPosition = {0, 0, 0};
	};

	TrackingSystemCalibration retrieveSystemCalibration(const std::string& systemName);
	void saveSystemCalibration(const std::string& systemName, TrackingSystemCalibration calibration);
}

# define M_PI           3.14159265358979323846
