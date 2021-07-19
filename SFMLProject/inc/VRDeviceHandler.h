#pragma once


#include <openvr.h>

#include "DeviceHandler.h"
#include "VRHelper.h"

#include <openvr_types.h>
#include <Eigen/Geometry>

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <KinectSettings.h>

#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/split_free.hpp>

enum class VirtualHipMode
{
	Standing,
	Sitting,
	Lying
};

struct VirtualHipSettings
{
	// Seconds. How far behind the tracked point should the hips be so that they don't instantly follow every tiny movement
	bool rtcalib = false;

	bool astartt = false;
	float tryawst, kinpitchst;
	int footOption, hipsOption, posOption = 3;
	int bodyTrackingOption = 1;
	// --- Standing Settings ---
	bool positionFollowsHMDLean = false;
	// Determines whether the virtual hips in standing mode will stay above the foot trackers, or interpolate between the HMD and foot trackers on a direct slant
	float hmdegree = 0.0;
	float tdegree = 3;
	double heightFromHMD = 0.00;
	// Meters. Hips are by default projected downwards from the HMD, by 72cm (adjustable by user)
	bool positionAccountsForFootTrackers = false;
	// If false, Hip tracker always stays bolted to directly under the HMD with no horizontal shift

	Eigen::Matrix<float, 3, 3> rcR_matT;
	Eigen::Matrix<float, 3, 1> rcT_matT;
	Eigen::Matrix<float, 3, 1> hauoffset_s;
	Eigen::Matrix<float, 3, 1> mauoffset_s;
	Eigen::Vector3f caliborigin;

	float rcR_matT_S[3][3];
	float rcT_matT_S[3];
	float hauoffset_s_S[3];
	float mauoffset_s_S[3];
	float caliborigin_S[3];

	template <class Archive>
	void serialize(Archive& archive)
	{
		archive(
			CEREAL_NVP(positionFollowsHMDLean),
			CEREAL_NVP(hmdegree),
			CEREAL_NVP(tdegree),
			CEREAL_NVP(footOption),
			CEREAL_NVP(hipsOption),
			CEREAL_NVP(posOption),
			CEREAL_NVP(bodyTrackingOption),
			CEREAL_NVP(astartt),
			CEREAL_NVP(tryawst),
			CEREAL_NVP(kinpitchst),

			CEREAL_NVP(rtcalib),
			CEREAL_NVP(heightFromHMD),
			CEREAL_NVP(positionAccountsForFootTrackers),

			CEREAL_NVP(caliborigin_S),
			CEREAL_NVP(hauoffset_s_S),
			CEREAL_NVP(mauoffset_s_S),
			CEREAL_NVP(rcR_matT_S),
			CEREAL_NVP(rcT_matT_S)
		);
	}
};

void decomposeEigen(VirtualHipSettings& settings)
{
	for (int i = 0; i < 3; i++)
	{
		settings.rcT_matT_S[i] = settings.rcT_matT[i];
		settings.hauoffset_s_S[i] = settings.hauoffset_s[i];
		settings.mauoffset_s_S[i] = settings.mauoffset_s[i];
		settings.caliborigin_S[i] = settings.caliborigin[i];
		for (int j = 0; j < 3; j++)
			settings.rcR_matT_S[i][j] = settings.rcR_matT.coeff(i, j);
	}
}

void recomposeEigen(VirtualHipSettings& settings)
{
	for (int i = 0; i < 3; i++)
	{
		settings.rcT_matT[i] = settings.rcT_matT_S[i];
		settings.hauoffset_s[i] = settings.hauoffset_s_S[i];
		settings.mauoffset_s[i] = settings.mauoffset_s_S[i];
		settings.caliborigin[i] = settings.caliborigin_S[i];
		for (int j = 0; j < 3; j++)
			settings.rcR_matT.coeffRef(i, j) = settings.rcR_matT_S[i][j];
	}
}

namespace VirtualHips
{
	VirtualHipSettings settings;
	static const std::wstring settingsConfig = L"ConfigSettings.cfg";

	void saveSettings()
	{
		decomposeEigen(settings);

		std::ofstream os(KVR::fileToDirPath(settingsConfig));
		if (os.fail())
		{
			//FAIL!!!
			LOG(ERROR) << "ERROR: COULD NOT WRITE TO SETTINGS FILE\n";
		}
		else
		{
			cereal::JSONOutputArchive archive(os);
			LOG(INFO) << "Attempted to save settings to file";
			try
			{
				archive(
					CEREAL_NVP(settings)
				);
			}
			catch (cereal::RapidJSONException e)
			{
				LOG(ERROR) << "CONFIG FILE SAVE JSON ERROR: " << e.what();
			}
		}
	}

	void retrieveSettings()
	{
		std::ifstream is(KVR::fileToDirPath(settingsConfig));
		LOG(INFO) << "Attempted to load settings at " << KVR::fileToDirPath(settingsConfig);

		if (is.fail())
		{
			LOG(ERROR) << "Settings file could not be found, generating a new one...";
			saveSettings();
		}
		else
		{
			LOG(INFO) << settingsConfig << " load attempted!";
			try
			{
				cereal::JSONInputArchive archive(is);
				archive(CEREAL_NVP(settings));

				recomposeEigen(settings);

				KinectSettings::hroffset = settings.hmdegree;
				KinectSettings::cpoints = settings.tdegree;

				KinectSettings::calibration_rotation = settings.rcR_matT;
				KinectSettings::calibration_translation = settings.rcT_matT;

				KinectSettings::huoffsets.v[0] = settings.heightFromHMD;

				KinectSettings::matrixes_calibrated = settings.rtcalib;
				KinectSettings::calibration_trackers_yaw = settings.tryawst;
				KinectSettings::calibration_kinect_pitch = settings.kinpitchst;

				KinectSettings::hauoffset.v[0] = settings.hauoffset_s(0);
				KinectSettings::hauoffset.v[1] = settings.hauoffset_s(1);
				KinectSettings::hauoffset.v[2] = settings.hauoffset_s(2);

				KinectSettings::mauoffset.v[0] = settings.mauoffset_s(0);
				KinectSettings::mauoffset.v[1] = settings.mauoffset_s(1);
				KinectSettings::mauoffset.v[2] = settings.mauoffset_s(2);

				footOrientationFilterOption.filterOption = static_cast<footRotationFilterOption>(settings.footOption);
				hipsOrientationFilterOption.filterOption = static_cast<hipsRotationFilterOption>(settings.hipsOption);
				positionFilterOption.filterOption = static_cast<positionalFilterOption>(settings.posOption);
				bodyTrackingOption_s.trackingOption = static_cast<bodyTrackingOption>(settings.bodyTrackingOption);

				KinectSettings::calibration_origin = settings.caliborigin;

				LOG(INFO) << settings.tryawst << '\n' << settings.rcR_matT << '\n' <<
					KinectSettings::calibration_trackers_yaw << '\n' <<
					KinectSettings::calibration_rotation << '\n';
			}
			catch (cereal::Exception e)
			{
				LOG(ERROR) << settingsConfig << "SETTINGS FILE LOAD JSON ERROR: " << e.what();
			}
		}
	}
}

class VRDeviceHandler : public DeviceHandler
{
	// Updates the tracking pool with data from the 
	// non-IE SteamVR devices - e.g. head position/rotation
public:
	VRDeviceHandler(vr::IVRSystem* & g_VRSystem)
		: m_VRSystem(g_VRSystem)
	{
	}

	~VRDeviceHandler() override
	{
	}

	int initialise() override
	{
		// Add all devices that aren't sensors or virtual
		LOG(INFO) << "Initialising VR Device Handler...";

		initVirtualHips();
		active = true;
		return 0;
	}

	int run() override
	{
		return 0;
	}

	void updateVirtualDeviceList()
	{
		//int virtualDeviceCount = m_inputEmulator.getVirtualDeviceCount();
		//std::fill(virtualDevices, virtualDevices + vr::k_unMaxTrackedDeviceCount, false);
		//for (int i = 0; i < virtualDeviceCount; ++i) {
		//    vrinputemulator::VirtualDeviceInfo info = m_inputEmulator.getVirtualDeviceInfo(i);
		//    virtualDevices[info.openvrDeviceId] = true;
		//}
	}

private:
	vr::IVRSystem* & m_VRSystem;
	//vrinputemulator::VRInputEmulator & m_inputEmulator;

	int virtualDeviceCount = 0;
	bool virtualDevices[vr::k_unMaxTrackedDeviceCount]{false};
	TrackerIDs vrDeviceToPoolIds[vr::k_unMaxTrackedDeviceCount]{};
	TrackerIDs virtualHipsIds{};
	uint32_t virtualHipsLocalId = 420;

	void initVirtualHips()
	{
		LOG(INFO) << "Reading Strings...";
		VirtualHips::retrieveSettings();
	}

	void getVRStringProperty(const uint32_t& openvrID, vr::ETrackedDeviceProperty strProperty, std::string& string)
	{
		vr::ETrackedPropertyError peError;
		uint32_t unRequiredBufferLen = m_VRSystem->GetStringTrackedDeviceProperty(
			openvrID, strProperty, nullptr, 0, &peError);
		if (unRequiredBufferLen == 0)
		{
			string = "";
		}
		else
		{
			auto pchBuffer = new char[unRequiredBufferLen];
			unRequiredBufferLen = m_VRSystem->GetStringTrackedDeviceProperty(
				openvrID, strProperty, pchBuffer, unRequiredBufferLen, &peError);
			string = pchBuffer;
			delete[] pchBuffer;
		}
	}
};
