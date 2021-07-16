#pragma once
#include "stdafx.h"
#include "openvr.h"
#include <openvr_types.h>
#include <SFML/System/Vector3.hpp>

namespace vrmath
{
	double length_sq(vr::HmdVector3d_t v);
	double length(vr::HmdVector3d_t v);
	double length(vr::HmdQuaternion_t q);
	vr::HmdQuaternion_t normalized(vr::HmdQuaternion_t a);
	float norm_squared(vr::HmdQuaternion_t x);
	vr::HmdQuaternion_t divide(const vr::HmdQuaternion_t& x, float k);
	vr::HmdQuaternion_t inverse(vr::HmdQuaternion_t x);

	vr::HmdVector3d_t cross(vr::HmdVector3d_t v1, vr::HmdVector3d_t v2);
	double dot(vr::HmdVector3d_t v1, vr::HmdVector3d_t v2);
	vr::HmdQuaternion_t get_rotation_between(vr::HmdVector3d_t u, vr::HmdVector3d_t v);
}

void setTrackerRolesInVRSettings();
void removeTrackerRolesInVRSettings();

void toEulerAngle(vr::HmdQuaternion_t q, double& pitch, double& yaw, double& roll);

vr::DriverPose_t defaultReadyDriverPose();
vr::DriverPose_t trackedDeviceToDriverPose(vr::TrackedDevicePose_t tPose);
vr::HmdVector3d_t getWorldPositionFromDriverPose(vr::DriverPose_t pose);

vr::HmdVector3d_t updateHMDPosAndRot(vr::IVRSystem* & m_sys);

// Get the quaternion representing the rotation
vr::HmdQuaternion_t GetVRRotationFromMatrix(vr::HmdMatrix34_t matrix);
// Get the vector representing the position
vr::HmdVector3d_t GetVRPositionFromMatrix(vr::HmdMatrix34_t matrix);
