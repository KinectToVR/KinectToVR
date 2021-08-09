#include <boost/asio.hpp>
#include "stdafx.h"
#include "KinectToVR.h"
#include "VRHelper.h"

#include "KinectSettings.h"
#include "VRController.h"
#include "GUIHandler.h"
#include "VRDeviceHandler.h"
#include "PSMoveHandler.h"
#include "DeviceHandler.h"
#include <boost/thread.hpp>
#include <SFML/Audio.hpp>

#include <locale>
#include <codecvt>
#include <iostream>
#include <string>
#include <thread>
#include <SFGUI/Widgets.hpp>
#include <SteamIVRInput.h>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/iostreams/stream.hpp>
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

// KinectToVR API include, from K2APP
#include <KinectToVR_API.h>

#pragma comment(lib, "Ws2_32.lib")

using namespace KVR;

std::string log_get_timestamp_prefix()
{
	// From PSMoveService ServerLog.cpp
	auto now = std::chrono::system_clock::now();
	auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
	auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds);
	time_t in_time_t = std::chrono::system_clock::to_time_t(now);

	std::stringstream ss;
	ss << "[" << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S") << "." << milliseconds.count() << "]: ";

	return ss.str();
}

void processKeyEvents(sf::Event event)
{
	switch (event.key.code)
	{
	case sf::Keyboard::A:
		toggle(KinectSettings::isKinectDrawn);
		break;
	default:
		break;
	}
}

void toggle(bool& b)
{
	b = !b;
}

// Get the horizontal and vertical screen sizes in pixel
//  https://stackoverflow.com/questions/8690619/how-to-get-screen-resolution-in-c
void getDesktopResolution(int& horizontal, int& vertical)
{
	RECT desktop;
	// Get a handle to the desktop window
	const HWND hDesktop = GetDesktopWindow();
	// Get the size of screen to the variable desktop
	GetWindowRect(hDesktop, &desktop);
	// The top left corner will have coordinates (0,0)
	// and the bottom right corner will have coordinates
	// (horizontal, vertical)
	horizontal = desktop.right;
	vertical = desktop.bottom;
}

sf::VideoMode getScaledWindowResolution()
{
	int h;
	int v;
	getDesktopResolution(h, v);

	auto mode = sf::VideoMode(SFMLsettings::windowScale * static_cast<float>(h),
	                          SFMLsettings::windowScale * static_cast<float>(v));
	//std::cerr << "desktop: " << h << ", " << v << '\n';
	//std::cerr << "scaled: " << mode.width << ", " << mode.height << '\n';
	return mode;
}

void updateKinectWindowRes(const sf::RenderWindow& window)
{
	SFMLsettings::m_window_width = window.getSize().x;
	SFMLsettings::m_window_height = window.getSize().y;
	LOG(INFO) << "Stored window size: " << SFMLsettings::m_window_width << " x " << SFMLsettings::m_window_height;
	//std::cerr << "w: " << SFMLsettings::m_window_width << " h: " << SFMLsettings::m_window_height << "\n";
}

bool filePathIsNonASCII(const std::wstring& filePath)
{
	for (auto c : filePath)
	{
		if (static_cast<unsigned char>(c) > 127)
		{
			return true;
		}
	}
	return false;
}

void verifyDefaultFilePath()
{
	// Warn about non-english file path, as openvr can only take ASCII chars
	// If this isn't checked, unfortunately, most of the bindings won't load
	// Unless OpenVR's C API adds support for non-english filepaths, K2VR can't either
	bool filePathInvalid = filePathIsNonASCII(SFMLsettings::fileDirectoryPath);
	if (filePathInvalid)
	{
		LOG(ERROR) << "K2VR File Path NONASCII (Invalid)!";
		auto message = L"WARNING: NON-ENGLISH FILEPATH DETECTED: "
			+ SFMLsettings::fileDirectoryPath
			+ L"\n It's possible that OpenVR bindings won't work correctly"
			+ L"\n Please move the K2VR directory to a location with ASCII only"
			+ L"\n e.g. C:/KinectToVR will be fine";
		auto result = MessageBox(nullptr, message.c_str(), L"WARNING!!!", MB_ABORTRETRYIGNORE + MB_ICONWARNING);
		if (result = IDABORT)
		{
			SFMLsettings::keepRunning = false;
		}
	}
	else
		LOG(INFO) << "K2VR File Path ASCII (Valid)";
}

void updateFilePath()
{
	HMODULE module = GetModuleHandleW(nullptr);
	WCHAR exeFilePath[MAX_PATH];
	GetModuleFileNameW(module, exeFilePath, MAX_PATH);

	//Get rid of exe from name
	WCHAR directory[MAX_PATH];
	WCHAR drive[_MAX_DRIVE];
	WCHAR dir[_MAX_DIR];
	WCHAR fname[_MAX_FNAME];
	WCHAR ext[_MAX_EXT];
	_wsplitpath_s(exeFilePath, drive, _MAX_DRIVE, dir, _MAX_DIR, fname,
	              _MAX_FNAME, ext, _MAX_EXT);

	WCHAR filename[_MAX_FNAME]{};
	WCHAR extension[_MAX_EXT]{};
	WCHAR directoryFilePath[MAX_PATH];
	_wmakepath_s(directoryFilePath, _MAX_PATH, drive, dir, filename, extension);
	std::wstring filePathString(directoryFilePath);
	SFMLsettings::fileDirectoryPath = filePathString;

	LOG(INFO) << "File Directory Path Set to " << filePathString;
}

vr::HmdQuaternion_t kinectQuaternionFromRads()
{
	return vrmath::quaternionFromYawPitchRoll(KinectSettings::kinectRadRotation.v[1],
	                                          KinectSettings::kinectRadRotation.v[0],
	                                          KinectSettings::kinectRadRotation.v[2]);
}

void updateTrackerInitGuiSignals(GUIHandler& guiRef,
	vr::IVRSystem*& m_VRsystem)
{
	guiRef.setTrackerButtonSignals(m_VRsystem);
}

void limitVRFramerate(double& endFrameMilliseconds)
{
	static unsigned int FPS = 90;

	double deltaDeviationMilliseconds;
	int maxMillisecondsToCompensate = 30;

	deltaDeviationMilliseconds = 1000.0 / FPS - endFrameMilliseconds;

	if (floor(deltaDeviationMilliseconds) > 0) // TODO: Handle -ve (slow) frames
		Sleep(deltaDeviationMilliseconds);
	if (deltaDeviationMilliseconds < -maxMillisecondsToCompensate)
	{
		endFrameMilliseconds -= maxMillisecondsToCompensate;
	}
	else
	{
		endFrameMilliseconds += deltaDeviationMilliseconds;
	}
}

bool testConnection(const bool log)
{
	// Do not spawn 1000 voids, check how many do we have
	if (KinectSettings::pingCheckingThreadsNumber <= KinectSettings::maxPingCheckingThreads)
	{
		// Add a new worker
		KinectSettings::pingCheckingThreadsNumber += 1; // May be ++ too

		try
		{
			// Send a ping message and capture the data
			const auto [test_response, send_time, full_time] = ktvr::test_connection();

			// Dump data to variables
			KinectSettings::pingTime = full_time;
			KinectSettings::parsingTime = std::clamp( // Subtract message creation (got) time and send time
				test_response.messageTimestamp - test_response.messageManualTimestamp,
				static_cast<long long>(1), LLONG_MAX);

			// Log ?success
			LOG(INFO) <<
				"Connection test has ended, [result: " <<
				(test_response.success ? "success" : "fail") <<
				"], response code: " << test_response.result;

			// Log some data if needed
			LOG_IF(log, INFO) <<
				"\nTested ping time: " << full_time << " [micros], " <<

				"call time: " <<
				std::clamp( // Subtract message creation (got) time and send time
					send_time - test_response.messageManualTimestamp,
					static_cast<long long>(0), LLONG_MAX) <<
				" [micros], " <<

				"\nparsing time: " <<
				KinectSettings::parsingTime << // Just look at the k2api
				" [micros], "

				"flight-back time: " <<
				std::clamp( // Subtract message creation (got) time and send time
					K2API_GET_TIMESTAMP_NOW - test_response.messageManualTimestamp,
					static_cast<long long>(1), LLONG_MAX) <<
				" [micros]";

			// Release
			KinectSettings::pingCheckingThreadsNumber -= 1; // May be -- too

			// Return the result
			return test_response.success;
		}
		catch (const std::exception& e)
		{
			// Log ?success
			LOG(INFO) <<
				"Connection test has ended, [result: fail], got an exception";

			// Release
			KinectSettings::pingCheckingThreadsNumber -= 1; // May be -- too
			return false;
		}
	}

	// else
	LOG(ERROR) << "Connection checking threads exceeds 3, aborting...";
	return false;
}

// Note: blocking!
int checkK2Server()
{
	if (!KinectSettings::isDriverPresent)
	{
		try
		{
			/* Initialize the port */
			LOG(INFO) << "Initializing the server IPC...";
			const auto init_code = ktvr::init_k2api();
			bool server_connected = false;

			LOG(INFO) << "Server IPC initialization " <<
				(init_code == 0 ? "succeed" : "failed") << ", exit code: " << init_code;

			/* Connection test and display ping */
			// We may wait
			LOG(INFO) << "Testing the connection...";

			for (int i = 0; i < 3; i++)
			{
				LOG(INFO) << "Starting the test no " << i + 1 << "...";
				server_connected = testConnection(true);
				// Not direct assignment since it's only a one-way check
				if (server_connected)KinectSettings::isDriverPresent = true;
			}

			return init_code == 0
				       ? (server_connected ? 1 : -10)
				       : -1;
		}
		catch (const std::exception& e) { return -1; }
	}

	/*
	 * codes:
	 * -1: check fail
	 * -10: server fail
	 * 10: driver failure
	 * -11: API fail
	 * 1: result ok
	 */
	return 1; //don't check if it was already working
}

void updateServerStatus(GUIHandler& guiRef)
{
	std::thread([&]()
	{
		if (!KinectSettings::isServerFailure)
		{
			KinectSettings::K2Drivercode = checkK2Server();
			switch (KinectSettings::K2Drivercode)
			{
			case -1:
				guiRef.DriverStatusLabel->SetText("SteamVR Driver Status: EXCEPTION WHILE CHECKING (Code: -1)");
				break;
			case -10:
				guiRef.DriverStatusLabel->SetText(
					"SteamVR Driver Status: SERVER CONNECTION ERROR (Code: -10)\nCheck SteamVR add-ons (NOT overlays) and enable KinectToVR.");
				break;
			case 10:
				guiRef.DriverStatusLabel->SetText(
					"SteamVR Driver Status: FATAL SERVER FAILURE (Code: 10)\nCheck logs and write to us on Discord.");
				break;
			case 1:
				guiRef.DriverStatusLabel->SetText("SteamVR Driver Status: Success!");
				KinectSettings::isDriverPresent = true;
				break;
			default:
				guiRef.DriverStatusLabel->SetText("SteamVR Driver Status: COULD NOT CONNECT TO K2API (Code: -11)");
				break;
			}

			guiRef.TrackerInitButton->SetState(KinectSettings::isDriverPresent
				                                   ? sfg::Widget::State::NORMAL
				                                   : sfg::Widget::State::INSENSITIVE);
			guiRef.ping_InitTrackers();
		}
	}).detach();
}

void processLoop(KinectHandlerBase& kinect)
{
	LOG(INFO) << "~~~New logging session for main process begins here!~~~";
	LOG(INFO) << "Kinect version is V" << static_cast<int>(kinect.kVersion);
	KinectSettings::kinectVersion = kinect.kVersion; //Set kinect version

	// Connect to OpenVR at the very beginning
	LOG(INFO) << "Attempting connection to vrsystem... ";
	vr::EVRInitError eError = vr::VRInitError_None;
	vr::IVRSystem* m_VRSystem = VR_Init(&eError, vr::VRApplication_Scene);

	if (eError != vr::VRInitError_None) {
		LOG(ERROR) << "IVRSystem could not be initialised: EVRInitError Code " << static_cast<int>(eError);
		MessageBoxA(nullptr,
			std::string(
				"Couldn't initialise VR system. (Code " + std::to_string(eError) + ")\n\nPlease check if SteamVR is installed (or running) and try again."
			).c_str(),
			"IVRSystem Init Failure!",
			MB_OK);
		raise(SIGINT); // Forcefully exit after OK
	}

	// Setup OpenVR actions
	LOG(INFO) << "Attempting to set up IVR Input Actions...";
	SteamIVRInput ivr_input;
	if (!ivr_input.InitInputActions())
	{
		LOG(ERROR) << "Could not set up Input Actions. Please check the upper log for further information.";
		MessageBoxA(nullptr,
			std::string(
				"Couldn't set up Input Actions.\n\nPlease check the log file for further information."
			).c_str(),
			"IVR Input Actions Init Failure!",
			MB_OK);
	}
	else LOG(INFO) << "IVR Input Actions set up OK";
	
	updateFilePath();
	//sf::RenderWindow renderWindow(getScaledWindowResolution(), "KinectToVR: " + KinectSettings::KVRversion, sf::Style::Titlebar | sf::Style::Close);
	sf::RenderWindow renderWindow(sf::VideoMode(1280, 768, 32), "KinectToVR: " + KinectSettings::KVRversion,
	                              sf::Style::Titlebar | sf::Style::Close | sf::Style::Resize);
	auto mGUIView = sf::View(renderWindow.getDefaultView());
	auto mGridView = sf::View(sf::FloatRect(0, 0, 1280, 768));

	updateKinectWindowRes(renderWindow);
	int windowFrameLimit = 90;
	renderWindow.setFramerateLimit(windowFrameLimit);
	//Prevents ridiculous overupdating and high CPU usage - plus 90Hz is the recommended refresh rate for most VR panels 
	//renderWindow.setVerticalSyncEnabled(true);

	sf::Clock frameClock;
	sf::Clock timingClock;

	sf::Time time_lastKinectStatusUpdate = timingClock.getElapsedTime();
	sf::Time time_lastGuiDesktopUpdate = timingClock.getElapsedTime();

	//Initialise Settings
	KinectSettings::serializeKinectSettings();
	sf::Font font;
	sf::Text debugText;
	// Global Debug Font

	//Initialise Kinect
	KinectSettings::kinectRepRotation = kinectQuaternionFromRads();
	kinect.update();

	KinectSettings::isKinectPSMS = kinect.isPSMS;
	KinectSettings::expcalib = !kinect.isPSMS; //Enable default manual calibration if PSMS

	// Check in the background
	std::thread([&]
	{
		/* Check for updates */
		LOG(INFO) << "Searching for updates...";
		try
		{
			curlpp::Cleanup myCleanup;
			std::ostringstream os;
			os << curlpp::options::Url("https://raw.githubusercontent.com/KinectToVR/KinectToVR/experiments/version");

			if (std::string read_buffer = os.str(); !read_buffer.empty())
			{
				LOG(INFO) << "Update-check successful, string:\n" << read_buffer;

				// Get rest strings
				std::stringstream s;
				s << read_buffer;

				// Read the JSON
				boost::property_tree::ptree s_ptree;
				read_json(s, s_ptree);

				// Get results
				auto release = s_ptree.get<std::string>("release"),
				     build = s_ptree.get<std::string>("build"),
				     changes = s_ptree.get<std::string>("changes");

				// Split strin into lines
				std::vector<std::string> _major, _minor;

				// Split version strings into ints
				split(_major, release, boost::is_any_of(","));
				split(_minor, build, boost::is_any_of(","));

				// Replace ";" with "\n" to make the changelog multi-line
				std::string _changelog = changes;
				std::replace(_changelog.begin(), _changelog.end(), ';', '\n');

				// Compare to the current version
				bool updateFound = false;
				for (int i = 0; i < 3; i++)
				{
					// Check major
					int _ver = boost::lexical_cast<int>(_major.at(i));
					if (_ver > k2vr_version_major[i]) updateFound = true;

						// Not to false-alarm in situations like 0.9.1 (local) vs 0.8.2 (remote)
					else if (_ver < k2vr_version_major[i]) break;
				}

				// Don't check the build number
				//for (int i = 0; i < 4; i++)
				//{
				//	// Check minor
				//	int _ver = boost::lexical_cast<int>(_minor.at(i));
				//	if (_ver > k2vr_version_minor[i]) updateFound = true;

				//	// Not to false-alarm in situations like 0.9.1 (local) vs 0.8.2 (remote)
				//	else if (_ver < k2vr_version_minor[i]) break;
				//}

				// Show the message box if update was found
				if (updateFound)
				{
					if (MessageBoxA(nullptr,
					                std::string(
						                "KinectToVR EX "

						                + _major.at(0) + "."
						                + _major.at(1) + "."
						                + _major.at(2) +

						                "\nBuild number: "

						                + _minor.at(0) + "."
						                + _minor.at(1) + "."
						                + _minor.at(2) + "."
						                + _minor.at(3) +

						                "\n\nChanges:\n\n"

						                + _changelog +

						                "\n\nDo you wish to update KinectToVR now?"
					                ).c_str(),
					                "KinectToVR Update Found!",
					                MB_YESNO) == IDYES)
					{
						/*
						 * Here, do all the stuff that needs to be done after pressing 'Yes'
						 */

						// Turn off trackers
						KinectSettings::initialised = false;

						// Find the installer
						// Clear the stream for reuse
						os.str(std::string());

						// Setup the User-Agent
						curlpp::Easy request;
						std::list<std::string> headers;
						headers.emplace_back("User-Agent: curl/7.77.7");

						using namespace curlpp::Options;

						request.setOpt(new Verbose(true));
						request.setOpt(new HttpHeader(headers));
						request.setOpt(
							new Url("https://api.github.com/repos/KinectToVR/k2vr-installer-gui/releases/latest"));
						request.perform();

						// Dump the result
						os << request;

						if (std::string installer_url, installer_releases = os.str(); !installer_releases.empty())
						{
							// Init the stream for boost
							std::stringstream ss;
							ss << installer_releases;

							// Read the JSON
							boost::property_tree::ptree ptree;
							read_json(ss, ptree);

							// Iterate over all results
							for (boost::property_tree::ptree::value_type& result : ptree.get_child("assets."))
								for (boost::property_tree::ptree::value_type& field : result.second)
									if (field.first == "browser_download_url") installer_url = field.second.data();

							// Notify about download
							std::thread([]
							{
								MessageBoxA(nullptr,
								            std::string(
									            "Update will begin soon!\n\nK2EX will close automatically."
								            ).c_str(),
								            "KinectToVR Update Found!",
								            MB_OK);
							}).detach();

							// Download the installer
							// Setup the User-Agent
							request.reset();

							using namespace curlpp::Options;
							request.setOpt(new Verbose(true));
							request.setOpt(new HttpHeader(headers));
							request.setOpt(new FollowLocation(true));
							request.setOpt(new Url(installer_url));

							// Perform the request
							FILE* file;
							fopen_s(&file, "k2ex-installer.exe", "wb");

							request.setOpt(curlpp::options::WriteFile(file));
							request.perform();

							// Close the file
							fclose(file);

							// Run the file and close K2EX
							system("start k2ex-installer.exe");

							SFMLsettings::keepRunning = false;
							KinectSettings::initialised = false;
							renderWindow.close();
						}
					}
				}
			}
			else
				LOG(INFO) << "Update failed, string was empty.";
		}
		catch (const std::exception& e)
		{
			LOG(INFO) << "Update failed, error:\n" << e.what();
		}
		/* Check for updates */
	}).detach();

	// Generate the default trackers here
	/* update 3 default trackers <KinectSettings.h> */
	for (int i = 0; i < 3; i++)
	{
		// We don't let the user overwrite serial here
		KinectSettings::trackerVector.at(i).data.serial = "LHR-CB9AD1T" + std::to_string(i);

		// Switch the rest
		switch (i)
		{
		case 0: // Waist
			KinectSettings::trackerVector.at(i).data.role = ktvr::Tracker_Waist;
			break;
		case 1: // LFoot
			KinectSettings::trackerVector.at(i).data.role = ktvr::Tracker_LeftFoot;
			break;
		case 2: // RFoot
			KinectSettings::trackerVector.at(i).data.role = ktvr::Tracker_RightFoot;
			break;
		}
	}

	// Initialize trackers' filters
	for (auto& tracker : KinectSettings::trackerVector)
		tracker.initAllFilters();

	//SFGUI Handling -------------------------------------- 
	GUIHandler guiRef;
	// ----------------------------------------------------

	// Update kinect status
	guiRef.updateKinectStatusLabel(kinect);
	// Reconnect Kinect Event Signal
	guiRef.setKinectButtonSignal(kinect);

	VRcontroller rightController(vr::TrackedControllerRole_RightHand);
	VRcontroller leftController(vr::TrackedControllerRole_LeftHand);
	
	// Initialise the VR Device Handler (For settings)
	VRDeviceHandler vrDeviceHandler(m_VRSystem);
	if (eError == vr::VRInitError_None)
		vrDeviceHandler.initialise();

	//Update driver status
	/************************************************/
	LOG(INFO) << "KinectToVR will try to connect the Driver via API on K2API's default addresses.";

	updateServerStatus(guiRef);
	/************************************************/

	// INPUT BINDING TEMPORARY --------------------------------
	// Warn about non-english file path, as openvr can only take ASCII chars
	verifyDefaultFilePath();

	if (eError == vr::VRInitError_None)
	{
		// Set origins so that proper offsets for each coordinate system can be found
		KinectSettings::trackingOrigin = m_VRSystem->GetRawZeroPoseToStandingAbsoluteTrackingPose();
		KinectSettings::trackingOriginPosition = GetVRPositionFromMatrix(KinectSettings::trackingOrigin);
		double yaw = std::atan2(KinectSettings::trackingOrigin.m[0][2], KinectSettings::trackingOrigin.m[2][2]);
		if (yaw < 0.0)
		{
			yaw = 2 * M_PI + yaw;
		}
		KinectSettings::svrhmdyaw = yaw;

		LOG(INFO) << "SteamVR Tracking Origin for Driver Relative: " << std::fixed <<
			KinectSettings::trackingOriginPosition.v[0] << ", " <<
			KinectSettings::trackingOriginPosition.v[1] << ", " <<
			KinectSettings::trackingOriginPosition.v[2] << ", " <<
			KinectSettings::svrhmdyaw << "RAD";

		guiRef.setVRSceneChangeButtonSignal(m_VRSystem);
		updateTrackerInitGuiSignals(guiRef, m_VRSystem);
		//VRInput::initialiseVRInput();

		leftController.Connect(m_VRSystem);
		rightController.Connect(m_VRSystem);

		// Todo: implement binding system
		// guiRef.loadK2VRIntoBindingsMenu(m_VRSystem);
	}

	guiRef.updateVRStatusLabel(eError);

	//Update driver status
	/************************************************/
	updateServerStatus(guiRef);
	/************************************************/

	/************************************************/
	// Add trackers (Try to download via serial / add)
	/************************************************/

	// Generate the default trackers here
	/* update 3 default trackers <KinectSettings.h> */
	for (int i = 0; i < 3; i++)
	{
		// Check if the tracker is enabled
		if (!KinectSettings::EnabledTrackersSave[i]) continue;
		
		// We don't let the user overwrite serial here
		auto tracker_downloaded = ktvr::download_tracker("LHR-CB9AD1T" + std::to_string(i));

		// Check the role too
		int _role = -1;
		switch (i)
		{
		case 0: // Waist
			_role = ktvr::Tracker_Waist;
			break;
		case 1: // LFoot
			_role = ktvr::Tracker_LeftFoot;
			break;
		case 2: // RFoot
			_role = ktvr::Tracker_RightFoot;
			break;
		}

		// If tracker's been found
		if (tracker_downloaded.result == ktvr::K2ResponseMessageCode_OK &&
			tracker_downloaded.messageType == ktvr::K2ResponseMessage_Tracker &&
			tracker_downloaded.tracker_base.data.role == _role)
		{
			KinectSettings::trackerID[i] = tracker_downloaded.id;
			KinectSettings::trackerSerial[i] = tracker_downloaded.tracker_base.data.serial;

			KinectSettings::trackerVector.at(i).id = tracker_downloaded.id;
			KinectSettings::trackerVector.at(i).data = tracker_downloaded.tracker_base.data;
			KinectSettings::trackerVector.at(i).pose = tracker_downloaded.tracker_base.pose;

			LOG(INFO) << "Tracker with serial " +
				("LHR-CB9AD1T" + std::to_string(i)) +
				" has been found with id " + std::to_string(tracker_downloaded.id) + " and will be used from now on.";
		}
		else
		{
			LOG(INFO) << "Tracker with serial " +
				("LHR-CB9AD1T" + std::to_string(i)) +
				" and suitable role has not been found and will be added separately.";

			// Add the tracker and overwrite id
			auto tracker_base = KinectSettings::trackerVector.at(i).getTrackerBase();
			auto add_tracker_response =
				add_tracker(tracker_base);
			KinectSettings::trackerVector.at(i).id = add_tracker_response.id;

			if (add_tracker_response.result == ktvr::K2ResponseMessageCode_OK)
			{
				LOG(INFO) << "Tracker with serial " +
					("LHR-CB9AD1T" + std::to_string(i)) +
					" has been added successfully.";

				KinectSettings::trackerID[i] = add_tracker_response.id;
				KinectSettings::trackerSerial[i] = add_tracker_response.tracker_base.data.serial;
			}
			else if (add_tracker_response.result == ktvr::K2ResponseMessageCode_AlreadyPresent)
			{
				LOG(INFO) << "Tracker with serial " +
					("LHR-CB9AD1T" + std::to_string(i)) +
					" is already present. Changing the last serial digit by +3...";

				// Change the serial
				KinectSettings::trackerVector.at(i).data.serial = "LHR-CB9AD1T" + std::to_string(i + 3);

				// Compose the response
				auto tracker_base_ns = KinectSettings::trackerVector.at(i).getTrackerBase();
				if (auto add_tracker_response_ns =
					add_tracker(tracker_base_ns); 
					add_tracker_response_ns.result == ktvr::K2ResponseMessageCode_OK)
				{
					LOG(INFO) << "Tracker with serial " +
						("LHR-CB9AD1T" + std::to_string(i + 3)) +
						" has been added successfully.";

					KinectSettings::trackerVector.at(i).id = add_tracker_response_ns.id;
					KinectSettings::trackerID[i] = add_tracker_response_ns.id;
					KinectSettings::trackerSerial[i] = add_tracker_response_ns.tracker_base.data.serial;
				}
				else
				{
					LOG(INFO) << "Tracker with serial " +
						("LHR-CB9AD1T" + std::to_string(i + 3)) +
						" could not be added. Giving up...";

					// Cause not checking anymore
					KinectSettings::isServerFailure = true;
					guiRef.DriverStatusLabel->SetText(
						"SteamVR Driver Status: FATAL SERVER FAILURE (Code: 10)\nCheck logs and write to us on Discord.");
				}
			}
		}
	}

	/************************************************/
	// Add trackers (Try to download via serial / add)
	/************************************************/

	KinectSettings::userChangingZero = true;
	if (kinect.kVersion != INVALID)
		kinect.initialiseSkeleton();

	// Since settings are read now, initialize the rest of gui
	guiRef.setVirtualHipsBoxSignals();

	guiRef.initialisePSMoveHandlerIntoGUI(); // Needs the deviceHandlerRef to be set

	// Select backed up or first (we may switch from KV1 to KV2, keeping config)
	guiRef.coptbox->SelectItem(
		VirtualHips::settings.SelectedFootTrackingOption < guiRef.coptbox->GetItemCount() ? VirtualHips::settings.SelectedFootTrackingOption : 0);

	guiRef.coptbox1->SelectItem(VirtualHips::settings.SelectedWaistTrackingOption);
	guiRef.foptbox->SelectItem(VirtualHips::settings.SelectedPositionalTrackingOption);

	// Select automatically
	guiRef.bodytrackingselectbox->SelectItem(0);
	guiRef.refreshpsms();

	// Select tracking option automatically
	VirtualHips::settings.SelectedBodyTrackingOption = KinectSettings::isKinectPSMS
		                                           ? k_PSMoveFullTracking
		                                           : k_KinectFullTracking;
	bodyTrackingOption_s.trackingOption = static_cast<bodyTrackingOption>(VirtualHips::settings.SelectedBodyTrackingOption);
	KinectSettings::positional_tracking_option = VirtualHips::settings.SelectedBodyTrackingOption;

	auto ipcThread = new boost::thread(KinectSettings::sendipc);
	ipcThread->detach();

	while (renderWindow.isOpen() && SFMLsettings::keepRunning)
	{
		if (!KinectSettings::isDriverPresent)
		{
			//Update driver status
			/************************************************/
			updateServerStatus(guiRef);
			/************************************************/
		}

		if (KinectSettings::isServerFailure)
			guiRef.DriverStatusLabel->SetText(
				"SteamVR Driver Status: FATAL SERVER FAILURE (Code: 10)\nCheck logs and write to us on Discord.");

		//Clear the debug text display
		SFMLsettings::debugDisplayTextStream.str(std::string());
		SFMLsettings::debugDisplayTextStream.clear();

		double currentTime = frameClock.restart().asSeconds();
		double deltaT = currentTime;
		SFMLsettings::debugDisplayTextStream << "FPS Start = " << 1.0 / deltaT << '\n';
		//std::cout << SFMLsettings::debugDisplayTextStream.str() << std::endl;

		if (timingClock.getElapsedTime() > time_lastGuiDesktopUpdate + sf::milliseconds(33))
		{
			sf::Event event;

			while (renderWindow.pollEvent(event))
			{
				guiRef.desktopHandleEvents(event);
				if (event.type == sf::Event::Closed)
				{
					SFMLsettings::keepRunning = false;
					KinectSettings::initialised = false;
					renderWindow.close();
					break;
				}
				if (event.type == sf::Event::KeyPressed)
				{
					processKeyEvents(event);
				}
				if (event.type == sf::Event::Resized)
				{
					std::cerr << "HELP I AM RESIZING!\n";
					//sf::Vector2f size = static_cast<sf::Vector2f>(renderWindow.getSize());
					auto size = sf::Vector2f(event.size.width, event.size.height);
					// Minimum size
					if (size.x < 800)
						size.x = 800;
					if (size.y < 600)
						size.y = 600;

					// Apply possible size changes
					renderWindow.setSize(static_cast<sf::Vector2u>(size));

					// Reset grid view
					mGridView.setCenter(size / 2.f);
					mGridView.setSize(size);
					// = sf::View(sf::FloatRect(mGridView.getCenter().x, mGridView.getCenter().y, mGridView.getSize().x+(mGridView.getSize().x - size.x), mGridView.getSize().y+(mGridView.getSize().y - size.y)));

					// Reset  GUI view
					mGUIView = sf::View(sf::FloatRect(0.f, 0.f, size.x, size.y));
					//mGUIView.setCenter(size / 2.f);
					renderWindow.setView(mGUIView);

					// Resize widgets
					updateKinectWindowRes(renderWindow);
					guiRef.updateWithNewWindowSize(size);
				}
			}
			if (!(renderWindow.isOpen() && SFMLsettings::keepRunning))
			{
				// Possible for window to be closed mid-loop, in which case, instead of using goto's
				// this is used to avoid glErrors that crash the program, and prevent proper
				// destruction and cleaning up
				break;
			}

			//Clear ---------------------------------------
			renderWindow.clear(); /////////////////////////////////////////////////////
			renderWindow.setView(mGridView); //////////////////////////////////////////
			renderWindow.setView(mGUIView); ///////////////////////////////////////////

			//Process -------------------------------------
			//Update GUI

			guiRef.updateDesktop(deltaT);
			time_lastGuiDesktopUpdate = timingClock.getElapsedTime();
		}

		//Update VR Components
		if (eError == vr::VRInitError_None)
		{

			/**********************************************/
			// Here, update IVR Input actions
			/**********************************************/

			// Backup the current ( OLD ) data
			bool bak_confirm_state = ivr_input.confirmAndSaveActionData().bState,
				bak_mode_swap_state = ivr_input.modeSwapActionData().bState,
				bak_freeze_state = ivr_input.trackerFreezeActionData().bState;

			// Update all input actions
			if (!ivr_input.UpdateActionStates())
				LOG(ERROR) << "Could not update IVR Input Actions. Please check logs for further information.";

			// Update the Tracking Freeze : flip-switch
			if(ivr_input.trackerFreezeActionData().bState
				!= bak_freeze_state) // Only if the state has changed
			{
				KinectSettings::trackingPaused = !KinectSettings::trackingPaused;
				guiRef.pauseTrackingButton->SetLabel(
					std::string(KinectSettings::trackingPaused ? "Resume" : "Freeze") + std::string(" Body Tracking in SteamVR"));
			}







			
			/**********************************************/
			// Here, update IVR Input actions
			/**********************************************/
			
			rightController.update(deltaT);
			leftController.update(deltaT);

			updateHMDPosAndRot(m_VRSystem);

			VRInput::trackpadpose[0].x = rightController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).x;
			VRInput::trackpadpose[0].y = rightController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).y;
			VRInput::trackpadpose[1].x = leftController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).x;
			VRInput::trackpadpose[1].y = leftController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).y;
			VRInput::confirmdatapose.bState = leftController.GetTriggerDown() || rightController.GetTriggerDown();

			KinectSettings::isGripPressed[0] = rightController.GetGripDown();
			KinectSettings::isGripPressed[1] = leftController.GetGrip();
			KinectSettings::isTriggerPressed[0] = rightController.GetTrigger();
			KinectSettings::isTriggerPressed[1] = leftController.GetTrigger();

			// EWWWWWWWWW -------------
			if (VRInput::legacyInputModeEnabled)
			{
				using namespace VRInput;
				moveHorizontallyData.bActive = true;
				auto leftStickValues = leftController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad);
				moveHorizontallyData.x = leftStickValues.x;
				moveHorizontallyData.y = leftStickValues.y;

				moveVerticallyData.bActive = true;
				auto rightStickValues = rightController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad);
				moveVerticallyData.x = rightStickValues.x;
				moveVerticallyData.y = rightStickValues.y;

				confirmCalibrationData.bActive = true;
				auto triggerDown = leftController.GetTriggerDown() || rightController.GetTriggerDown();
				confirmCalibrationData.bState = triggerDown;
			}
			// -------------------------
		}

		renderWindow.clear(); //////////////////////////////////////////////////////

		// Update Kinect Status
		// Only needs to be updated sparingly
		if (timingClock.getElapsedTime() > time_lastKinectStatusUpdate + sf::seconds(2.0))
		{
			guiRef.updateKinectStatusLabel(kinect);
			time_lastKinectStatusUpdate = timingClock.getElapsedTime();
		}

		if (kinect.isInitialised())
		{
			kinect.update();

			//renderWindow.clear();
			kinect.drawKinectData(renderWindow);
		}

		if (footOrientationFilterOption.filterOption != static_cast<footRotationFilterOption>(guiRef.coptbox->
			GetSelectedItem()))
		{
			footOrientationFilterOption.filterOption = static_cast<footRotationFilterOption>(guiRef.coptbox->
				GetSelectedItem());
			VirtualHips::settings.SelectedFootTrackingOption = guiRef.coptbox->GetSelectedItem();

			VirtualHips::saveSettings();
		}
		if (hipsOrientationFilterOption.filterOption != static_cast<hipsRotationFilterOption>(guiRef.coptbox1->
			GetSelectedItem()))
		{
			hipsOrientationFilterOption.filterOption = static_cast<hipsRotationFilterOption>(guiRef.coptbox1->
				GetSelectedItem());
			VirtualHips::settings.SelectedWaistTrackingOption = guiRef.coptbox1->GetSelectedItem();

			VirtualHips::saveSettings();
		}
		if (positionFilterOption.filterOption != static_cast<positionalFilterOption>(guiRef.foptbox->GetSelectedItem()))
		{
			positionFilterOption.filterOption = static_cast<positionalFilterOption>(guiRef.foptbox->GetSelectedItem());
			VirtualHips::settings.SelectedPositionalTrackingOption = guiRef.foptbox->GetSelectedItem();

			VirtualHips::saveSettings();
		}
		// We're not updating it, it would mess everything
		/*if (bodyTrackingOption_s.trackingOption != static_cast<bodyTrackingOption>(guiRef.bodytrackingselectbox->
			GetSelectedItem()))
		{
			bodyTrackingOption_s.trackingOption = static_cast<bodyTrackingOption>(guiRef.bodytrackingselectbox->
				GetSelectedItem());
			VirtualHips::settings.SelectedBodyTrackingOption = guiRef.bodytrackingselectbox->GetSelectedItem();

			VirtualHips::saveSettings();
		}*/

		if (VirtualHips::settings.SelectedBodyTrackingOption == k_PSMoveFullTracking)
			guiRef.psmidbox->Show(true);
		else
			guiRef.psmidbox->Show(false);

		KinectSettings::feet_rotation_option = VirtualHips::settings.SelectedFootTrackingOption;
		KinectSettings::hips_rotation_option = VirtualHips::settings.SelectedWaistTrackingOption;
		KinectSettings::posOption = VirtualHips::settings.SelectedPositionalTrackingOption;
		KinectSettings::positional_tracking_option = VirtualHips::settings.SelectedBodyTrackingOption;

		//KinectSettings::footRotationFilterOption::k_EnableOrientationFilter;

		// Draw GUI
		updateHMDPosAndRot(m_VRSystem);

		//renderWindow.clear(); //////////////////////////////////////////////////////

		renderWindow.setActive(true);
		renderWindow.setView(mGUIView);
		guiRef.display(renderWindow);

		//Draw debug font
		double endTimeMilliseconds = frameClock.getElapsedTime().asMilliseconds();
		SFMLsettings::debugDisplayTextStream << "endTimeMilli: " << endTimeMilliseconds << '\n';

		//limitVRFramerate(endTimeMilliseconds);
		debugText.setString(SFMLsettings::debugDisplayTextStream.str());
		renderWindow.draw(debugText);

		renderWindow.resetGLStates();
		//End Frame
		renderWindow.display();
	}

	KinectSettings::writeKinectSettings();
	VirtualHips::saveSettings();

	kinect.terminateColor();
	kinect.terminateDepth();
	kinect.terminateSkeleton();

	if (eError == vr::EVRInitError::VRInitError_None)
		vr::VR_Shutdown();
}

void spawnDefaultLowerBodyTrackers()
{
	std::thread([&]
	{
		bool spawned[3] = {false};
		
		// Try 3 times
		for (int i = 0; i < 3; i++)
		{
			// Add only default trackers from the vector (0-2)
			for (int t = 0; t < 3; t++)
			{
				if (KinectSettings::EnabledTrackersSave[t])
				{
					if (const auto& m_result =
						ktvr::set_tracker_state(KinectSettings::trackerVector.at(t).id, true); // We WANT a reply
						m_result.id == KinectSettings::trackerVector.at(t).id && m_result.success)
					{
						LOG(INFO) << "Tracker with serial " + KinectSettings::trackerVector.at(t).data.serial + " and id " + std::to_string(
								KinectSettings::trackerVector.at(t).id) +
							" was successfully updated with status [active]";
						spawned[t] = true;
					}

					else if (m_result.id != KinectSettings::trackerVector.at(t).id && m_result.success)
						LOG(ERROR) << "Tracker with serial " + KinectSettings::trackerVector.at(t).data.serial + " and id " + std::to_string(
								KinectSettings::trackerVector.at(t).id) +
							" could not be spawned due to ID mismatch.";

					else
					{
						LOG(ERROR) << "Tracker with serial " + KinectSettings::trackerVector.at(t).data.serial + " and id " + std::to_string(
								KinectSettings::trackerVector.at(t).id) +
							" could not be spawned due to internal server error.";
						if (!ktvr::GetLastError().empty())
							LOG(ERROR) << "Last K2API error: " + ktvr::GetLastError();
					}
				}
				else
				{
					spawned[t] = true; // Hacky hack
					LOG(INFO) << "Not spawning tracker with serial " + KinectSettings::trackerVector.at(t).data.serial +
						" because it is disabled in settings.";
				}
			}
		}

		if (!spawned[0] || !spawned[1] || !spawned[2])
		{
			LOG(INFO) << "One or more trackers couldn't be spawned after 3 tries. Giving up...";

			// Cause not checking anymore
			KinectSettings::isServerFailure = true;
			KinectSettings::spawned = false;
		}
		else // Notify that we're good now
			KinectSettings::spawned = true;
	}).detach();
}
