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
	bool AreMatricesCalibrated = false;

	bool AutoStartTrackers = false;
	float CalibrationTrackersYawOffset, CalibrationKinectCalculatedPitch;
	int SelectedFootTrackingOption, SelectedWaistTrackingOption, SelectedPositionalTrackingOption = 3;
	int SelectedBodyTrackingOption = 1;
	// --- Standing Settings ---
	bool positionFollowsHMDLean = false;
	// Determines whether the virtual hips in standing mode will stay above the foot trackers, or interpolate between the HMD and foot trackers on a direct slant
	float HMDOrientationOffset = 0.0;
	float CalibrationPointsNumber = 3;
	double HeightFromHMD = 0.00;
	bool FlipEnabled = true;
	// Meters. Hips are by default projected downwards from the HMD, by 72cm (adjustable by user)
	bool positionAccountsForFootTrackers = false;
	// If false, Hip tracker always stays bolted to directly under the HMD with no horizontal shift

	Eigen::Matrix<float, 3, 3> rcR_matT;
	Eigen::Matrix<float, 3, 1> rcT_matT;
	Eigen::Matrix<float, 3, 1> hauoffset_s;
	Eigen::Matrix<float, 3, 1> mauoffset_s;
	Eigen::Vector3f caliborigin;

	float RecoveredRotationMatrixSave[3][3];
	float RecoveredTranslationVectorSave[3];
	float ManualOffsetsSave[3];
	float CalibrationOriginSave[3];
	
	bool OnTrackersSave[3] = { true, true, true };
	bool EnabledTrackersSave[3] = { true, true, true };

	template <class Archive>
	void serialize(Archive& archive)
	{
		archive(
			CEREAL_NVP(positionFollowsHMDLean),
			CEREAL_NVP(FlipEnabled),
			CEREAL_NVP(HMDOrientationOffset),
			CEREAL_NVP(CalibrationPointsNumber),
			CEREAL_NVP(SelectedFootTrackingOption),
			CEREAL_NVP(SelectedWaistTrackingOption),
			CEREAL_NVP(SelectedPositionalTrackingOption),
			CEREAL_NVP(SelectedBodyTrackingOption),
			CEREAL_NVP(AutoStartTrackers),
			CEREAL_NVP(CalibrationTrackersYawOffset),
			CEREAL_NVP(CalibrationKinectCalculatedPitch),

			CEREAL_NVP(AreMatricesCalibrated),
			CEREAL_NVP(HeightFromHMD),

			CEREAL_NVP(OnTrackersSave),
			CEREAL_NVP(EnabledTrackersSave),

			CEREAL_NVP(CalibrationOriginSave),
			CEREAL_NVP(ManualOffsetsSave),
			CEREAL_NVP(RecoveredRotationMatrixSave),
			CEREAL_NVP(RecoveredTranslationVectorSave)
		);
	}
};

void decomposeEigen(VirtualHipSettings& settings)
{
	for (int i = 0; i < 3; i++)
	{
		settings.RecoveredTranslationVectorSave[i] = settings.rcT_matT[i];
		settings.ManualOffsetsSave[i] = settings.mauoffset_s[i];
		settings.CalibrationOriginSave[i] = settings.caliborigin[i];
		for (int j = 0; j < 3; j++)
			settings.RecoveredRotationMatrixSave[i][j] = settings.rcR_matT.coeff(i, j);
	}
}

void recomposeEigen(VirtualHipSettings& settings)
{
	for (int i = 0; i < 3; i++)
	{
		settings.rcT_matT[i] = settings.RecoveredTranslationVectorSave[i];
		settings.mauoffset_s[i] = settings.ManualOffsetsSave[i];
		settings.caliborigin[i] = settings.CalibrationOriginSave[i];
		for (int j = 0; j < 3; j++)
			settings.rcR_matT.coeffRef(i, j) = settings.RecoveredRotationMatrixSave[i][j];
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

				KinectSettings::hroffset = settings.HMDOrientationOffset;
				KinectSettings::cpoints = settings.CalibrationPointsNumber;

				KinectSettings::calibration_rotation = settings.rcR_matT;
				KinectSettings::calibration_translation = settings.rcT_matT;

				KinectSettings::huoffsets.v[0] = settings.HeightFromHMD;

				KinectSettings::matrixes_calibrated = settings.AreMatricesCalibrated;
				KinectSettings::calibration_trackers_yaw = settings.CalibrationTrackersYawOffset;
				KinectSettings::calibration_kinect_pitch = settings.CalibrationKinectCalculatedPitch;
				
				KinectSettings::mauoffset.v[0] = settings.mauoffset_s(0);
				KinectSettings::mauoffset.v[1] = settings.mauoffset_s(1);
				KinectSettings::mauoffset.v[2] = settings.mauoffset_s(2);

				KinectSettings::OnTrackersSave[0] = settings.OnTrackersSave[0];
				KinectSettings::OnTrackersSave[1] = settings.OnTrackersSave[1];
				KinectSettings::OnTrackersSave[2] = settings.OnTrackersSave[2];

				KinectSettings::EnabledTrackersSave[0] = settings.EnabledTrackersSave[0];
				KinectSettings::EnabledTrackersSave[1] = settings.EnabledTrackersSave[1];
				KinectSettings::EnabledTrackersSave[2] = settings.EnabledTrackersSave[2];

				footOrientationFilterOption.filterOption = static_cast<footRotationFilterOption>(settings.SelectedFootTrackingOption);
				hipsOrientationFilterOption.filterOption = static_cast<hipsRotationFilterOption>(settings.SelectedWaistTrackingOption);
				positionFilterOption.filterOption = static_cast<positionalFilterOption>(settings.SelectedPositionalTrackingOption);
				bodyTrackingOption_s.trackingOption = static_cast<bodyTrackingOption>(settings.SelectedBodyTrackingOption);

				KinectSettings::calibration_origin = settings.caliborigin;

				LOG(INFO) << settings.CalibrationTrackersYawOffset << '\n' << settings.rcR_matT << '\n' <<
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
