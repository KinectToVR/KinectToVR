#pragma once
#include "stdafx.h"
#include <logging.h>
#include <openvr.h>
#include <boost/filesystem.hpp>

// Error strings, **assuming they can't change**
inline std::string EVRInputErrorString[] = {
	"VRInputError_None",
	"VRInputError_NameNotFound",
	"VRInputError_WrongType",
	"VRInputError_InvalidHandle",
	"VRInputError_InvalidParam",
	"VRInputError_NoSteam",
	"VRInputError_MaxCapacityReached",
	"VRInputError_IPCError",
	"VRInputError_NoActiveActionSet",
	"VRInputError_InvalidDevice",
	"VRInputError_InvalidSkeleton",
	"VRInputError_InvalidBoneCount",
	"VRInputError_InvalidCompressedData",
	"VRInputError_NoData",
	"VRInputError_BufferTooSmall",
	"VRInputError_MismatchedActionManifest",
	"VRInputError_MissingSkeletonData",
	"VRInputError_InvalidBoneIndex",
	"VRInputError_InvalidPriority",
	"VRInputError_PermissionDenied",
	"VRInputError_InvalidRenderModel",
};

// Action strings set in action_manifest.json

// Required
constexpr auto k_actionSetDefault = "/actions/default"; // Default

constexpr auto k_actionLeftJoystick = "/actions/default/in/LeftJoystick"; // Left-hand Move/Rotate Controls
constexpr auto k_actionRightJoystick = "/actions/default/in/RightJoystick"; // Right-hand Move/Rotate Controls

constexpr auto k_actionConfirmAndSave = "/actions/default/in/ConfirmAndSave"; // Confirm and Save
constexpr auto k_actionModeSwap = "/actions/default/in/ModeSwap"; // Swap Move/Rotate Modes
constexpr auto k_actionFineTune = "/actions/default/in/FineTune"; // Fine-tuning

// Optional
constexpr auto k_actionTrackerFreeze = "/actions/default/in/TrackerFreeze"; // Freeze Trackers
constexpr auto k_actionFlipToggle = "/actions/default/in/FlipToggle"; // Toggle Flip

// Main SteamEVRInput class
class SteamEVRInput {
public:

	// Note: SteamVR must be initialized beforehand.
	// Preferred type is (vr::VRApplication_Scene)
	bool InitInputActions()
	{
		// Find the absolute path of manifest
		boost::filesystem::path absoluteManifestPath;
		try {
			absoluteManifestPath = boost::filesystem::canonical(m_actionManifestPath);
		}
		catch(...) {
			LOG(ERROR) << "Action manifest was not found in the current directory.";
			return false;
		}
		
		// Set the action manifest. This should be in the executable directory.
		// Defined by m_actionManifestPath.
		vr::EVRInputError error = vr::VRInput()->SetActionManifestPath(absoluteManifestPath.string().c_str());
		if (error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "Action manifest error: " << EVRInputErrorString[error];
			return false;
		}

		/**********************************************/
		// Here, setup every action with its handler
		/**********************************************/
		
		// Get action handle for Left Joystick
		error = vr::VRInput()->GetActionHandle(k_actionLeftJoystick,
			&m_LeftJoystickHandler);
		if (error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "Action handle error: " << EVRInputErrorString[error];
			return false;
		}

		// Get action handle for Right Joystick
		error = vr::VRInput()->GetActionHandle(k_actionRightJoystick,
			&m_RightJoystickHandler);
		if (error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "Action handle error: " << EVRInputErrorString[error];
			return false;
		}

		// Get action handle for Confirm And Save
		error = vr::VRInput()->GetActionHandle(k_actionConfirmAndSave,
			&m_ConfirmAndSaveHandler);
		if (error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "Action handle error: " << EVRInputErrorString[error];
			return false;
		}

		// Get action handle for Mode Swap
		error = vr::VRInput()->GetActionHandle(k_actionModeSwap,
			&m_ModeSwapHandler);
		if (error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "Action handle error: " << EVRInputErrorString[error];
			return false;
		}

		// Get action handle for Fine-tuning
		error = vr::VRInput()->GetActionHandle(k_actionFineTune,
			&m_FineTuneHandler);
		if (error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "Action handle error: " << EVRInputErrorString[error];
			return false;
		}
		
		// Get action handle for Tracker Freeze
		error = vr::VRInput()->GetActionHandle(k_actionTrackerFreeze,
			&m_TrackerFreezeHandler);
		if (error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "Action handle error: " << EVRInputErrorString[error];
			return false;
		}

		// Get action handle for Flip Toggle
		error = vr::VRInput()->GetActionHandle(k_actionFlipToggle,
			&m_FlipToggleHandler);
		if (error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "Action handle error: " << EVRInputErrorString[error];
			return false;
		}
		
		/**********************************************/
		// Here, setup every action set handle
		/**********************************************/

		// Get set handle Default Set
		error = vr::VRInput()->GetActionSetHandle(k_actionSetDefault,
			&m_DefaultSetHandler);
		if (error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "ActionSet handle error: " << EVRInputErrorString[error];
			return false;
		}
		
		/**********************************************/
		// Here, setup action-set handler
		/**********************************************/

		// Default Set
		m_defaultActionSet.ulActionSet = m_DefaultSetHandler;
		m_defaultActionSet.ulRestrictedToDevice = vr::k_ulInvalidInputValueHandle;
		m_defaultActionSet.nPriority = 0;
		
		// Return OK
		LOG(INFO) << "EVR Input Actions initialized OK";
		return true;
	}
	
	bool UpdateActionStates()
	{
		/**********************************************/
		// Here, update main action sets' handles
		/**********************************************/

		// Update Default ActionSet states
		if (const auto error = vr::VRInput()->UpdateActionState(
			&m_defaultActionSet, sizeof m_defaultActionSet, 1);
			error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "ActionSet (Default) state update error: " << EVRInputErrorString[error];
			return false;
		}
		
		/**********************************************/
		// Here, update the actions and grab data-s
		/**********************************************/

		// Update the left joystick
		if(!GetLeftJoystickState())
		{
			LOG(INFO) << "Left Joystick Action is not active, can't update!";
			return false;
		}

		// Update the right joystick
		if (!GetRightJoystickState())
		{
			LOG(INFO) << "Right Joystick Action is not active, can't update!";
			return false;
		}
		
		// Update the confirm and save
		if (!GetConfirmAndSaveState())
		{
			LOG(INFO) << "Confirm And Save Action is not active, can't update!";
			return false;
		}

		// Update the mode swap
		if (!GetModeSwapState())
		{
			LOG(INFO) << "Mode Swap Action is not active, can't update!";
			return false;
		}

		// Update the fine tune
		if (!GetFineTuneState())
		{
			LOG(INFO) << "Fine-tuning Action is not active, can't update!";
			return false;
		}

		// Update the freeze
		// This time without checks, since this one is optional
		GetTrackerFreezeState();
		
		// Return OK
		return true;
	}

	// Analog data poll
	[[nodiscard]] vr::InputAnalogActionData_t
		leftJoystickActionData() const
	{
		return m_LeftJoystickHandlerData;
	}
	
	[[nodiscard]] vr::InputAnalogActionData_t
		rightJoystickActionData() const
	{
		return m_RightJoystickHandlerData;
	}

	// Digital data poll
	[[nodiscard]] vr::InputDigitalActionData_t
		confirmAndSaveActionData() const
	{
		return m_ConfirmAndSaveData;
	}
	
	[[nodiscard]] vr::InputDigitalActionData_t
		modeSwapActionData() const
	{
		return m_ModeSwapData;
	}

	[[nodiscard]] vr::InputDigitalActionData_t
		fineTuneActionData() const
	{
		return m_FineTuneData;
	}
	
	[[nodiscard]] vr::InputDigitalActionData_t
		trackerFreezeActionData() const
	{
		return m_TrackerFreezeData;
	}

	[[nodiscard]] vr::InputDigitalActionData_t
		trackerFlipToggleData() const
	{
		return m_FlipToggleData;
	}

private:
	// Action manifest path
	const std::string m_actionManifestPath = "action_manifest.json";

	// Calibration actions
	vr::VRActionHandle_t
		m_LeftJoystickHandler = {},
		m_RightJoystickHandler = {},
		m_ConfirmAndSaveHandler = {},
		m_ModeSwapHandler = {},
		m_FineTuneHandler = {};

	// Tracking freeze actions
	vr::VRActionHandle_t
		m_TrackerFreezeHandler = {},
		m_FlipToggleHandler = {};

	// Tracking Default set
	vr::VRActionSetHandle_t m_DefaultSetHandler = {};
	
	// The action sets
	vr::VRActiveActionSet_t m_defaultActionSet = {};

	// Buttons data
	vr::InputDigitalActionData_t
		m_ConfirmAndSaveData = {},
		m_ModeSwapData = {},
		m_FineTuneData = {},
		m_TrackerFreezeData = {},
		m_FlipToggleData = {};

	// Analogs data
	vr::InputAnalogActionData_t
		m_LeftJoystickHandlerData = {},
		m_RightJoystickHandlerData = {};

	// Update Left Joystick Action
	bool GetLeftJoystickState()
	{
		// Update the action and grab data
		if (const auto error = vr::VRInput()->
			GetAnalogActionData(
				m_LeftJoystickHandler,
				&m_LeftJoystickHandlerData,
				sizeof m_LeftJoystickHandlerData,
				vr::k_ulInvalidInputValueHandle);
			error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "GetAnalogActionData call error: " << EVRInputErrorString[error];
			return false;
		}

		// Return OK
		return true;
	}

	// Update Right Joystick Action
	bool GetRightJoystickState()
	{
		// Update the action and grab data
		if (const auto error = vr::VRInput()->
			GetAnalogActionData(
				m_RightJoystickHandler,
				&m_RightJoystickHandlerData,
				sizeof m_RightJoystickHandlerData,
				vr::k_ulInvalidInputValueHandle);
			error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "GetAnalogActionData call error: " << EVRInputErrorString[error];
			return false;
		}

		// Return OK
		return true;
	}

	// Update Confirm And Save Action
	bool GetConfirmAndSaveState()
	{
		// Update the action and grab data
		if (const auto error = vr::VRInput()->
			GetDigitalActionData(
				m_ConfirmAndSaveHandler,
				&m_ConfirmAndSaveData,
				sizeof m_ConfirmAndSaveData,
				vr::k_ulInvalidInputValueHandle);
			error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "GetDigitalActionData call error: " << EVRInputErrorString[error];
			return false;
		}

		// Return OK
		return true;
	}

	// Update Mode Swap Action
	bool GetModeSwapState()
	{
		// Update the action and grab data
		if (const auto error = vr::VRInput()->
			GetDigitalActionData(
				m_ModeSwapHandler,
				&m_ModeSwapData,
				sizeof m_ModeSwapData,
				vr::k_ulInvalidInputValueHandle);
			error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "GetDigitalActionData call error: " << EVRInputErrorString[error];
			return false;
		}

		// Return OK
		return true;
	}

	// Update Mode Swap Action
	bool GetFineTuneState()
	{
		// Update the action and grab data
		if (const auto error = vr::VRInput()->
			GetDigitalActionData(
				m_FineTuneHandler,
				&m_FineTuneData,
				sizeof m_FineTuneData,
				vr::k_ulInvalidInputValueHandle);
			error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "GetDigitalActionData call error: " << EVRInputErrorString[error];
			return false;
		}

		// Return OK
		return true;
	}

	// Update Tracker Freeze Action
	bool GetTrackerFreezeState()
	{
		// Update the action and grab data
		if (const auto error = vr::VRInput()->
			GetDigitalActionData(
				m_TrackerFreezeHandler,
				&m_TrackerFreezeData,
				sizeof m_TrackerFreezeData,
				vr::k_ulInvalidInputValueHandle);
			error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "GetDigitalActionData call error: " << EVRInputErrorString[error];
			return false;
		}

		// Return OK
		return true;
	}

	// Update Tracker Freeze Action
	bool GetFlipToggleState()
	{
		// Update the action and grab data
		if (const auto error = vr::VRInput()->
			GetDigitalActionData(
				m_FlipToggleHandler,
				&m_FlipToggleData,
				sizeof m_FlipToggleData,
				vr::k_ulInvalidInputValueHandle);
			error != vr::EVRInputError::VRInputError_None)
		{
			LOG(ERROR) << "GetDigitalActionData call error: " << EVRInputErrorString[error];
			return false;
		}

		// Return OK
		return true;
	}
};
