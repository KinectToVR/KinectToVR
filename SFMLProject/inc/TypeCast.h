#pragma once

#include <Eigen/Dense>
#include <glm/glm.hpp>
#include <glm/detail/type_quat.hpp>
#include <openvr_types.h>

// From KTVR / runtimeConfig.h

/**
 * \brief This template will let us convert between different types
 * \tparam ret What type should be returned from function, passed in template
 * \tparam T Class of the parameter 'in', deduced
 * \param in Parameter, object which will be 'converted'
 * \return Returns 'in' converted to 'ret' return type
 */
template <typename ret, class T> auto p_cast_type(const T& in)
{
	/* If somehow same */
	if constexpr (std::is_same<ret, T>::value) return in;

	/* To glm Quaternion */
	else if constexpr (std::is_same<ret, glm::quat>::value && std::is_same<T, Eigen::Quaternionf>::value)
		return glm::quat(in.w(), in.x(), in.y(), in.z());

	else if constexpr (std::is_same<ret, glm::quat>::value && std::is_same<T, vr::HmdQuaternion_t>::value)
		return glm::quat(in.w, in.x, in.y, in.z);

	/* To Eigen Quaternion */
	else if constexpr (std::is_same<ret, Eigen::Quaternionf>::value && std::is_same<T, glm::quat>::value ||
		std::is_same<ret, Eigen::Quaternionf>::value && std::is_same<T, vr::HmdQuaternion_t>::value)
		return Eigen::Quaternionf(in.w, in.x, in.y, in.z);

	/* To OpenVR Quaternion */
	else if constexpr (std::is_same<ret, vr::HmdQuaternion_t>::value && std::is_same<T, glm::quat>::value)
		return vr::HmdQuaternion_t{ in.w, in.x, in.y, in.z };

	else if constexpr (std::is_same<ret, vr::HmdQuaternion_t>::value && std::is_same<T, Eigen::Quaternionf>::value)
		return vr::HmdQuaternion_t{ in.w(), in.x(), in.y(), in.z() };

	/* To glm vec3 */
	else if constexpr (std::is_same<ret, glm::vec3>::value && std::is_same<T, Eigen::Vector3f>::value)
		return glm::vec3(in.x(), in.y(), in.z());

	else if constexpr (std::is_same<ret, glm::vec3>::value && std::is_same<T, vr::HmdVector3d_t>::value)
		return glm::vec3(in.x, in.y, in.z);

	else if constexpr (std::is_same<ret, glm::vec3>::value && std::is_same<T, vr::HmdMatrix34_t>::value)
		return glm::vec3(in.m[0][3], in.m[1][3], in.m[2][3]);

	/* To Eigen Vector3f */
	else if constexpr (std::is_same<ret, Eigen::Vector3f>::value && std::is_same<T, glm::vec3>::value ||
		std::is_same<ret, Eigen::Vector3f>::value && std::is_same<T, vr::HmdVector3d_t>::value)
		return Eigen::Vector3f(in.x, in.y, in.z);

	else if constexpr (std::is_same<ret, Eigen::Vector3f>::value && std::is_same<T, vr::HmdMatrix34_t>::value)
		return Eigen::Vector3f(in.m[0][3], in.m[1][3], in.m[2][3]);

	/* To OpenVR HmdVector3d_t */
	else if constexpr (std::is_same<ret, vr::HmdVector3d_t>::value && std::is_same<T, glm::vec3>::value)
		return vr::HmdVector3d_t{ in.x, in.y, in.z };

	else if constexpr (std::is_same<ret, vr::HmdVector3d_t>::value && std::is_same<T, Eigen::Vector3f>::value)
		return vr::HmdVector3d_t{ in.x(), in.y(), in.z() };

	else if constexpr (std::is_same<ret, vr::HmdVector3d_t>::value && std::is_same<T, vr::HmdMatrix34_t>::value)
		return vr::HmdVector3d_t{ in.m[0][3], in.m[1][3], in.m[2][3] };

	/* From OpenVR Matrix to OpenVR Quaternion */
	else if constexpr (std::is_same<ret, vr::HmdQuaternion_t>::value && std::is_same<T, vr::HmdMatrix34_t>::value)
	{
		auto q = vr::HmdQuaternion_t{ 1.,0.,0.,0. };
		q.w = sqrt(fmax(0, 1 + in.m[0][0] + in.m[1][1] + in.m[2][2])) / 2;
		q.x = sqrt(fmax(0, 1 + in.m[0][0] - in.m[1][1] - in.m[2][2])) / 2;
		q.y = sqrt(fmax(0, 1 - in.m[0][0] + in.m[1][1] - in.m[2][2])) / 2;
		q.z = sqrt(fmax(0, 1 - in.m[0][0] - in.m[1][1] + in.m[2][2])) / 2;
		q.x = copysign(q.x, in.m[2][1] - in.m[1][2]);
		q.y = copysign(q.y, in.m[0][2] - in.m[2][0]);
		q.z = copysign(q.z, in.m[1][0] - in.m[0][1]);
		return q;
	}

	/* From OpenVR Matrix to glm Quaternion */
	else if constexpr (std::is_same<ret, glm::quat>::value && std::is_same<T, vr::HmdMatrix34_t>::value)
		return p_cast_type<glm::quat>(p_cast_type<vr::HmdQuaternion_t>(in));

	/* From OpenVR Matrix to Eigen Quaternion */
	else if constexpr (std::is_same<ret, Eigen::Quaternionf>::value && std::is_same<T, vr::HmdMatrix34_t>::value)
		return p_cast_type<Eigen::Quaternionf>(p_cast_type<vr::HmdQuaternion_t>(in));
}