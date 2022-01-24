#pragma once

#include <cmath>
#include <openvr.h>

inline vr::HmdQuaternion_t operator+(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		lhs.w + rhs.w,
		lhs.x + rhs.x,
		lhs.y + rhs.y,
		lhs.z + rhs.z
	};
}


inline vr::HmdQuaternion_t operator-(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return{
		lhs.w - rhs.w,
		lhs.x - rhs.x,
		lhs.y - rhs.y,
		lhs.z - rhs.z
	};
}


inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}


inline vr::HmdVector3d_t operator+(const vr::HmdVector3d_t& lhs, const vr::HmdVector3d_t& rhs) {
	return {
		lhs.v[0] + rhs.v[0],
		lhs.v[1] + rhs.v[1],
		lhs.v[2] + rhs.v[2]
	};
}

inline vr::HmdVector3d_t operator+(const vr::HmdVector3d_t& lhs, const double(&rhs)[3]) {
	return{
		lhs.v[0] + rhs[0],
		lhs.v[1] + rhs[1],
		lhs.v[2] + rhs[2]
	};
}

inline vr::HmdVector3d_t operator-(const vr::HmdVector3d_t& lhs, const vr::HmdVector3d_t& rhs) {
	return{
		lhs.v[0] - rhs.v[0],
		lhs.v[1] - rhs.v[1],
		lhs.v[2] - rhs.v[2]
	};
}

inline vr::HmdVector3d_t operator-(const vr::HmdVector3d_t& lhs, const double (&rhs)[3]) {
	return{
		lhs.v[0] - rhs[0],
		lhs.v[1] - rhs[1],
		lhs.v[2] - rhs[2]
	};
}


inline vr::HmdVector3d_t operator*(const vr::HmdVector3d_t& lhs, const double rhs) {
	return{
		lhs.v[0] * rhs,
		lhs.v[1] * rhs,
		lhs.v[2] * rhs
	};
}


inline vr::HmdVector3d_t operator/(const vr::HmdVector3d_t& lhs, const double rhs) {
	return{
		lhs.v[0] / rhs,
		lhs.v[1] / rhs,
		lhs.v[2] / rhs
	};
}


namespace vrmath {

	template<typename T> int signum(T v) {
		return (v > (T)0) ? 1 : ((v < (T)0) ? -1 : 0);
	}

	inline vr::HmdQuaternion_t quaternionFromRotationAxis(double rot, double ux, double uy, double uz) {
		auto ha = rot / 2;
		return{
			std::cos(ha),
			ux * std::sin(ha),
			uy * std::sin(ha),
			uz * std::sin(ha)
		};
	}

	inline vr::HmdQuaternion_t quaternionFromRotationX(double rot) {
		auto ha = rot / 2;
		return{
			std::cos(ha),
			std::sin(ha),
			0.0f,
			0.0f
		};
	}

	inline vr::HmdQuaternion_t quaternionFromRotationY(double rot) {
		auto ha = rot / 2;
		return{
			std::cos(ha),
			0.0f,
			std::sin(ha),
			0.0f
		};
	}

	inline vr::HmdQuaternion_t quaternionFromRotationZ(double rot) {
		auto ha = rot / 2;
		return{
			std::cos(ha),
			0.0f,
			0.0f,
			std::sin(ha)
		};
	}

	inline vr::HmdQuaternion_t quaternionFromYawPitchRoll(double yaw, double pitch, double roll) {
		return quaternionFromRotationY(yaw) * quaternionFromRotationX(pitch) * quaternionFromRotationZ(roll);
	}

	inline vr::HmdQuaternion_t quaternionFromRotationMatrix(const vr::HmdMatrix34_t& mat) {
		auto a = mat.m;
		vr::HmdQuaternion_t q;
		double trace = a[0][0] + a[1][1] + a[2][2];
		if (trace > 0) {
			double s = 0.5 / sqrt(trace + 1.0);
			q.w = 0.25 / s;
			q.x = (a[1][2] - a[2][1]) * s;
			q.y = (a[2][0] - a[0][2]) * s;
			q.z = (a[0][1] - a[1][0]) * s;
		} else {
			if (a[0][0] > a[1][1] && a[0][0] > a[2][2]) {
				double s = 2.0 * sqrt(1.0 + a[0][0] - a[1][1] - a[2][2]);
				q.w = (a[1][2] - a[2][1]) / s;
				q.x = 0.25 * s;
				q.y = (a[1][0] + a[0][1]) / s;
				q.z = (a[2][0] + a[0][2]) / s;
			} else if (a[1][1] > a[2][2]) {
				double s = 2.0 * sqrt(1.0 + a[1][1] - a[0][0] - a[2][2]);
				q.w = (a[2][0] - a[0][2]) / s;
				q.x = (a[1][0] + a[0][1]) / s;
				q.y = 0.25 * s;
				q.z = (a[2][1] + a[1][2]) / s;
			} else {
				double s = 2.0 * sqrt(1.0 + a[2][2] - a[0][0] - a[1][1]);
				q.w = (a[0][1] - a[1][0]) / s;
				q.x = (a[2][0] + a[0][2]) / s;
				q.y = (a[2][1] + a[1][2]) / s;
				q.z = 0.25 * s;
			}
		}
		q.x = -q.x;
		q.y = -q.y;
		q.z = -q.z;
		return q;
	}

	inline vr::HmdQuaternion_t quaternionConjugate(const vr::HmdQuaternion_t& quat) {
		return {
			quat.w,
			-quat.x,
			-quat.y,
			-quat.z,
		};
	}

	inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const vr::HmdVector3d_t& vector, bool reverse = false) {
		if (reverse) {
			vr::HmdQuaternion_t pin = { 0.0, vector.v[0], vector.v[1] , vector.v[2] };
			auto pout = vrmath::quaternionConjugate(quat) * pin * quat;
			return {pout.x, pout.y, pout.z};
		} else {
			vr::HmdQuaternion_t pin = { 0.0, vector.v[0], vector.v[1] , vector.v[2] };
			auto pout = quat * pin * vrmath::quaternionConjugate(quat);
			return { pout.x, pout.y, pout.z };
		}
	}

	inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const vr::HmdQuaternion_t& quatInv, const vr::HmdVector3d_t& vector, bool reverse = false) {
		if (reverse) {
			vr::HmdQuaternion_t pin = { 0.0, vector.v[0], vector.v[1] , vector.v[2] };
			auto pout = quatInv * pin * quat;
			return{ pout.x, pout.y, pout.z };
		} else {
			vr::HmdQuaternion_t pin = { 0.0, vector.v[0], vector.v[1] , vector.v[2] };
			auto pout = quat * pin * quatInv;
			return{ pout.x, pout.y, pout.z };
		}
	}

	inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double (&vector)[3], bool reverse = false) {
		if (reverse) {
			vr::HmdQuaternion_t pin = { 0.0, vector[0], vector[1] , vector[2] };
			auto pout = vrmath::quaternionConjugate(quat) * pin * quat;
			return{ pout.x, pout.y, pout.z };
		} else {
			vr::HmdQuaternion_t pin = { 0.0, vector[0], vector[1] , vector[2] };
			auto pout = quat * pin * vrmath::quaternionConjugate(quat);
			return{ pout.x, pout.y, pout.z };
		}
	}

	inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const vr::HmdQuaternion_t& quatInv, const double(&vector)[3], bool reverse = false) {
		if (reverse) {
			vr::HmdQuaternion_t pin = { 0.0, vector[0], vector[1] , vector[2] };
			auto pout = quatInv * pin * quat;
			return{ pout.x, pout.y, pout.z };
		} else {
			vr::HmdQuaternion_t pin = { 0.0, vector[0], vector[1] , vector[2] };
			auto pout = quat * pin * quatInv;
			return{ pout.x, pout.y, pout.z };
		}
	}

	inline vr::HmdMatrix34_t matMul33(const vr::HmdMatrix34_t& a, const vr::HmdMatrix34_t& b) {
		vr::HmdMatrix34_t result;
		for (unsigned i = 0; i < 3; i++) {
			for (unsigned j = 0; j < 3; j++) {
				result.m[i][j] = 0.0f;
				for (unsigned k = 0; k < 3; k++) {
					result.m[i][j] += a.m[i][k] * b.m[k][j];
				}
			}
		}
		return result;
	}

	inline vr::HmdVector3_t matMul33(const vr::HmdMatrix34_t& a, const vr::HmdVector3_t& b) {
		vr::HmdVector3_t result;
		for (unsigned i = 0; i < 3; i++) {
			result.v[i] = 0.0f;
			for (unsigned k = 0; k < 3; k++) {
				result.v[i] += a.m[i][k] * b.v[k];
			};
		}
		return result;
	}

	inline vr::HmdVector3d_t matMul33(const vr::HmdMatrix34_t& a, const vr::HmdVector3d_t& b) {
		vr::HmdVector3d_t result;
		for (unsigned i = 0; i < 3; i++) {
			result.v[i] = 0.0f;
			for (unsigned k = 0; k < 3; k++) {
				result.v[i] += a.m[i][k] * b.v[k];
			};
		}
		return result;
	}

	inline vr::HmdVector3_t matMul33(const vr::HmdVector3_t& a, const vr::HmdMatrix34_t& b) {
		vr::HmdVector3_t result;
		for (unsigned i = 0; i < 3; i++) {
			result.v[i] = 0.0f;
			for (unsigned k = 0; k < 3; k++) {
				result.v[i] += a.v[k] * b.m[k][i];
			};
		}
		return result;
	}

	inline vr::HmdVector3d_t matMul33(const vr::HmdVector3d_t& a, const vr::HmdMatrix34_t& b) {
		vr::HmdVector3d_t result;
		for (unsigned i = 0; i < 3; i++) {
			result.v[i] = 0.0f;
			for (unsigned k = 0; k < 3; k++) {
				result.v[i] += a.v[k] * b.m[k][i];
			};
		}
		return result;
	}

	inline vr::HmdMatrix34_t transposeMul33(const vr::HmdMatrix34_t& a) {
		vr::HmdMatrix34_t result;
		for (unsigned i = 0; i < 3; i++) {
			for (unsigned k = 0; k < 3; k++) {
				result.m[i][k] = a.m[k][i];
			}
		}
		result.m[0][3] = a.m[0][3];
		result.m[1][3] = a.m[1][3];
		result.m[2][3] = a.m[2][3];
		return result;
	}
}

namespace vr {
	// Copied from openvr_driver.h
	struct DriverPose_t {
		/* Time offset of this pose, in seconds from the actual time of the pose,
		* relative to the time of the PoseUpdated() call made by the driver.
		*/
		double poseTimeOffset;

		/* Generally, the pose maintained by a driver
		* is in an inertial coordinate system different
		* from the world system of x+ right, y+ up, z+ back.
		* Also, the driver is not usually tracking the "head" position,
		* but instead an internal IMU or another reference point in the HMD.
		* The following two transforms transform positions and orientations
		* to app world space from driver world space,
		* and to HMD head space from driver local body space.
		*
		* We maintain the driver pose state in its internal coordinate system,
		* so we can do the pose prediction math without having to
		* use angular acceleration.  A driver's angular acceleration is generally not measured,
		* and is instead calculated from successive samples of angular velocity.
		* This leads to a noisy angular acceleration values, which are also
		* lagged due to the filtering required to reduce noise to an acceptable level.
		*/
		vr::HmdQuaternion_t qWorldFromDriverRotation;
		double vecWorldFromDriverTranslation[3];

		vr::HmdQuaternion_t qDriverFromHeadRotation;
		double vecDriverFromHeadTranslation[3];

		/* State of driver pose, in meters and radians. */
		/* Position of the driver tracking reference in driver world space
		* +[0] (x) is right
		* +[1] (y) is up
		* -[2] (z) is forward
		*/
		double vecPosition[3];

		/* Velocity of the pose in meters/second */
		double vecVelocity[3];

		/* Acceleration of the pose in meters/second */
		double vecAcceleration[3];

		/* Orientation of the tracker, represented as a quaternion */
		vr::HmdQuaternion_t qRotation;

		/* Angular velocity of the pose in axis-angle
		* representation. The direction is the angle of
		* rotation and the magnitude is the angle around
		* that axis in radians/second. */
		double vecAngularVelocity[3];

		/* Angular acceleration of the pose in axis-angle
		* representation. The direction is the angle of
		* rotation and the magnitude is the angle around
		* that axis in radians/second^2. */
		double vecAngularAcceleration[3];

		ETrackingResult result;

		bool poseIsValid;
		bool willDriftInYaw;
		bool shouldApplyHeadModel;
		bool deviceIsConnected;
	};
} // end namespace vr
