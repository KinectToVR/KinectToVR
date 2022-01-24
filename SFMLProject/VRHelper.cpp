#include "stdafx.h"

#include "VRHelper.h"
#include <openvr_types.h>
#include "KinectSettings.h"

namespace vrmath
{
	double length_sq(vr::HmdVector3d_t v)
	{
		return
			v.v[0] * v.v[0] +
			v.v[1] * v.v[1] +
			v.v[2] * v.v[2];
	}

	double length(vr::HmdVector3d_t v)
	{
		return sqrt(length_sq(v));
	}

	double length(vr::HmdQuaternion_t q)
	{
		return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	}

	vr::HmdQuaternion_t normalized(vr::HmdQuaternion_t a)
	{
		vr::HmdQuaternion_t q = a;
		float magnitude = length(q);
		q.w /= magnitude;
		q.x /= magnitude;
		q.y /= magnitude;
		q.z /= magnitude;
		return q;
	}

	float norm_squared(const vr::HmdQuaternion_t x)
	{
		return x.w * x.w + x.x * x.x + x.y * x.y + x.z * x.z;
	}

	vr::HmdQuaternion_t divide(const vr::HmdQuaternion_t& x, float k)
	{
		vr::HmdQuaternion_t q;
		q.w = x.w / k;
		q.x = x.x / k;
		q.y = x.y / k;
		q.z = x.z / k;
		return q;
	}

	vr::HmdQuaternion_t inverse(const vr::HmdQuaternion_t x)
	{
		// Might need to take in reference
		auto sq = norm_squared(x);
		if (sq == 0.0f)
			return {1, 0, 0, 0};
		return divide(quaternionConjugate(x), sq);
	}

	vr::HmdVector3d_t cross(vr::HmdVector3d_t v1, vr::HmdVector3d_t v2)
	{
		float x = (v1.v[1] * v2.v[2]) - (v1.v[2] * v2.v[1]);
		float y = (v1.v[2] * v2.v[0]) - (v1.v[0] * v2.v[2]);
		float z = (v1.v[0] * v2.v[1]) - (v1.v[1] * v2.v[0]);
		return {x, y, z};
	}

	double dot(vr::HmdVector3d_t v1, vr::HmdVector3d_t v2)
	{
		return v1.v[0] * v2.v[0] + v1.v[1] * v2.v[1] + v1.v[2] * v2.v[2];
	}

	vr::HmdQuaternion_t get_rotation_between(vr::HmdVector3d_t u, vr::HmdVector3d_t v)
	{
		double k_cos_theta = dot(u, v);
		float k = sqrt(length_sq(u) * length_sq(v));

		if (k_cos_theta / k == -1)
		{
			// 180 degree rotation around any orthogonal vector
			return vr::HmdQuaternion_t{1, 0, 0, 0};
		}
		auto vec = cross(u, v);
		return normalized(vr::HmdQuaternion_t{k_cos_theta + k, vec.v[0], vec.v[1], vec.v[2]});
	}
}

void toEulerAngle(vr::HmdQuaternion_t q, double& pitch, double& yaw, double& roll)
{
	vr::HmdVector3d_t v;
	double test = q.x * q.y + q.z * q.w;
	if (test > 0.499)
	{
		// singularity at north pole
		v.v[0] = 2 * atan2(q.x, q.w); // heading
		v.v[1] = M_PI / 2; // attitude
		v.v[2] = 0; // bank
		return;
	}
	if (test < -0.499)
	{
		// singularity at south pole
		v.v[0] = -2 * atan2(q.x, q.w); // headingq
		v.v[1] = -M_PI / 2; // attitude
		v.v[2] = 0; // bank
		return;
	}
	double sqx = q.x * q.x;
	double sqy = q.y * q.y;
	double sqz = q.z * q.z;
	v.v[0] = asin(2 * test);
	v.v[1] = atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz);
	v.v[2] = atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * sqx - 2 * sqz);
	roll = v.v[0];
	yaw = v.v[1];
	pitch = v.v[2];
}

vr::DriverPose_t defaultReadyDriverPose()
{
	vr::DriverPose_t pose{};
	pose.deviceIsConnected = true;

	vr::HmdVector3d_t pos = {0};
	vr::HmdQuaternion_t rot = {1, 0, 0, 0};

	pose.qRotation = rot;

	pose.qWorldFromDriverRotation = {1, 0, 0, 0};
	// need these else nothing rotates/moves visually, but if the tracking system requires adjustments, these must be set by the device handler
	pose.vecWorldFromDriverTranslation[0] = 0;
	pose.vecWorldFromDriverTranslation[1] = 0;
	pose.vecWorldFromDriverTranslation[2] = 0;

	pose.qDriverFromHeadRotation = {1, 0, 0, 0};
	pose.vecDriverFromHeadTranslation[0] = 0;
	pose.vecDriverFromHeadTranslation[1] = 0;
	pose.vecDriverFromHeadTranslation[2] = 0;

	//Final Position Adjustment
	pose.vecPosition[0] = pos.v[0];
	pose.vecPosition[1] = pos.v[1];
	pose.vecPosition[2] = pos.v[2];

	pose.poseIsValid = true;

	pose.result = vr::TrackingResult_Running_OK;

	return pose;
}

vr::DriverPose_t trackedDeviceToDriverPose(vr::TrackedDevicePose_t tPose)
{
	vr::DriverPose_t pose = defaultReadyDriverPose();
	pose.deviceIsConnected = true;

	vr::HmdVector3d_t pos = GetVRPositionFromMatrix(tPose.mDeviceToAbsoluteTracking);
	vr::HmdQuaternion_t rot = GetVRRotationFromMatrix(tPose.mDeviceToAbsoluteTracking);


	pose.vecAngularVelocity[0] = tPose.vAngularVelocity.v[0];
	pose.vecAngularVelocity[1] = tPose.vAngularVelocity.v[1];
	pose.vecAngularVelocity[2] = tPose.vAngularVelocity.v[2];

	pose.vecVelocity[0] = tPose.vVelocity.v[0];
	pose.vecVelocity[1] = tPose.vVelocity.v[1];
	pose.vecVelocity[2] = tPose.vVelocity.v[2];

	pose.poseTimeOffset = 0.f;

	return pose;
}

vr::HmdVector3d_t getWorldPositionFromDriverPose(vr::DriverPose_t pose)
{
	vr::HmdVector3d_t position{0};

	position.v[0] = pose.vecPosition[0] + pose.vecWorldFromDriverTranslation[0];
	position.v[1] = pose.vecPosition[1] + pose.vecWorldFromDriverTranslation[1];
	position.v[2] = pose.vecPosition[2] + pose.vecWorldFromDriverTranslation[2];

	return position;
}

vr::HmdVector3d_t updateHMDPosAndRot(vr::IVRSystem* & m_system)
{
	//Gets the HMD location for relative position setting
	// Use the head joint for the zero location!
	vr::HmdVector3d_t position{};
	const int HMD_INDEX = 0;

	vr::TrackedDevicePose_t hmdPose;
	vr::TrackedDevicePose_t devicePose[vr::k_unMaxTrackedDeviceCount];
	m_system->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding, 0, devicePose,
	                                          vr::k_unMaxTrackedDeviceCount);
	if (devicePose[HMD_INDEX].bPoseIsValid)
	{
		if (vr::VRSystem()->GetTrackedDeviceClass(HMD_INDEX) == vr::TrackedDeviceClass_HMD)
		{
			hmdPose = devicePose[HMD_INDEX];
			KinectSettings::hmdAbsoluteTracking = hmdPose.mDeviceToAbsoluteTracking;
			position = GetVRPositionFromMatrix(hmdPose.mDeviceToAbsoluteTracking);
			vr::HmdQuaternion_t quaternion = GetVRRotationFromMatrix(hmdPose.mDeviceToAbsoluteTracking);
			KinectSettings::hmdPosition = position;
			KinectSettings::hmdRot = GetVRRotationFromMatrix(hmdPose.mDeviceToAbsoluteTracking);

			/*
			// DEBUG -------------------------------------------------------------------
			// FIND THE SECONDARY ORIGIN
			// Has to have vecPosition at 
			auto vrRelativePSMovePos = GetVRPositionFromMatrix(devicePose[5].mDeviceToAbsoluteTracking);
			LOG(INFO) << "SPAWNED: VR : " << vrRelativePSMovePos.v[0] << ", " << vrRelativePSMovePos.v[1] << ", " << vrRelativePSMovePos.v[2];


			LOG(INFO) << "HMD: " << position.v[0] << ", " << position.v[1] << ", " << position.v[2];

			

			auto originMatrix = m_system->GetRawZeroPoseToStandingAbsoluteTrackingPose();
			auto vrRelativeOriginPos = GetVRPositionFromMatrix(originMatrix);
			auto vrRelativeOriginRot = GetVRRotationFromMatrix(KinectSettings::trackingOrigin);
			LOG(INFO) << "ORIGIN: VR : " << vrRelativeOriginPos.v[0] << ", " << vrRelativeOriginPos.v[1] << ", " << vrRelativeOriginPos.v[2];
			LOG(INFO) << "ORIGIN ROT: VR : " << vrRelativeOriginRot.w << ", " << vrRelativeOriginRot.x << ", " << vrRelativeOriginRot.y << ", " << vrRelativeOriginRot.z;



			// -------------------------------------------------------------------------
			*/

			double yaw = std::atan2(hmdPose.mDeviceToAbsoluteTracking.m[0][2],
			                        hmdPose.mDeviceToAbsoluteTracking.m[2][2]),
			       yawRaw = std::atan2(hmdPose.mDeviceToAbsoluteTracking.m[0][2],
			                           hmdPose.mDeviceToAbsoluteTracking.m[2][2]);

			if (yawRaw < 0.0f)
			{
				yawRaw += 2 * M_PI;
			}
			if (yaw < 0.0)
			{
				yaw = 2 * M_PI + yaw;
			}

			KinectSettings::hmdYaw = yaw;
			KinectSettings::hmdRotation = quaternion;
		}
	}
	return position;
}

// Get the quaternion representing the rotation
vr::HmdQuaternion_t GetVRRotationFromMatrix(vr::HmdMatrix34_t matrix)
{
	// Credit to Omnifinity https://github.com/Omnifinity/OpenVR-Tracking-Example/
	vr::HmdQuaternion_t q;

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
	return q;
}

// Get the vector representing the position
vr::HmdVector3d_t GetVRPositionFromMatrix(vr::HmdMatrix34_t matrix)
{
	// Credit to Omnifinity https://github.com/Omnifinity/OpenVR-Tracking-Example/
	vr::HmdVector3d_t vector;

	vector.v[0] = matrix.m[0][3];
	vector.v[1] = matrix.m[1][3];
	vector.v[2] = matrix.m[2][3];

	return vector;
}
