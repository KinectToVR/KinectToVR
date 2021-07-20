﻿#include <boost/asio.hpp>
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

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/iostreams/stream.hpp>
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

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
	if constexpr (true)
	{
		guiRef.setTrackerButtonSignals(m_VRsystem);
	}
	else
	{
		guiRef.updateTrackerInitButtonLabelFail();
	}
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

int checkK2Server()
{
	if (!KinectSettings::isDriverPresent)
	{
		try
		{
			HANDLE ServerStatusPipe = CreateNamedPipe(
				TEXT("\\\\.\\pipe\\K2ServerStatusPipe"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND,
				PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE, 1, 1024, 1024, 120 * 1000, nullptr);

			char ServerData[1024];
			DWORD Init = DWORD();

			ConnectNamedPipe(ServerStatusPipe, nullptr);
			ReadFile(ServerStatusPipe, ServerData, 1024, &Init, nullptr);
			CloseHandle(ServerStatusPipe);

			std::string ServerString = ServerData;

			if (ServerString.find("10") != std::string::npos)
			{
				return 10;
			}
			if (ServerString.find("1") != std::string::npos)
			{
				return 1;
			}
			return -10;
		}
		catch (const std::exception& e) { return -10; }
	}

	/*
	 * codes:
	 * -1: check fail
	 * -10: sever fail
	 * 1: result ok
	 * 10: init fail
	 */
	return 1; //don't check if it was already working
}

void updateServerStatus(GUIHandler& guiRef)
{
	std::thread([&]()
	{
		KinectSettings::K2Drivercode = checkK2Server();
		switch (KinectSettings::K2Drivercode)
		{
		case -1:
			guiRef.DriverStatusLabel->SetText("SteamVR Driver Status: UNKNOWN (Code: -1)");
			break;
		case -10:
			guiRef.DriverStatusLabel->SetText(
				"SteamVR Driver Status: SERVER ERROR (Code: -10)\nCheck SteamVR add-ons (NOT overlays) and enable KinectToVR.");
			break;
		case 1:
			guiRef.DriverStatusLabel->SetText("SteamVR Driver Status: Success!");
			KinectSettings::isDriverPresent = true;
			break;
		case 10:
			guiRef.DriverStatusLabel->SetText("SteamVR Driver Status: ERROR NOT INITIALIZED (Code: 10)");
			break;
		default:
			guiRef.DriverStatusLabel->SetText("SteamVR Driver Status: UNKNOWN (Code: -1)");
			break;
		}

		guiRef.TrackerInitButton->SetState(KinectSettings::isDriverPresent
			                                   ? sfg::Widget::State::NORMAL
			                                   : sfg::Widget::State::INSENSITIVE);
		guiRef.ping_InitTrackers();
	}).detach();
}

void processLoop(KinectHandlerBase& kinect)
{
	LOG(INFO) << "~~~New logging session for main process begins here!~~~";
	LOG(INFO) << "Kinect version is V" << static_cast<int>(kinect.kVersion);
	KinectSettings::kinectVersion = kinect.kVersion; //Set kinect version

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
			os << curlpp::options::Url("https://raw.githubusercontent.com/KimihikoAkayasaki/update-dummy/main/version");

			std::string read_buffer = os.str();
			read_buffer.erase(read_buffer.find_last_of("\n"), read_buffer.length()); // Erase the ending "\n"
			
			if (!read_buffer.empty())
			{
				if (read_buffer.front() == '{' &&
					read_buffer.back() == '}')
				{
					LOG(INFO) << "Update-check successful, string:\n" << read_buffer;

					// Remove the {} from string
					read_buffer.erase(0, read_buffer.find("\n") + 1);
					read_buffer.erase(read_buffer.find_last_of("\n"), read_buffer.length());

					// Split strin into lines
					std::vector<std::string> _lines, _major, _minor;
					split(_lines, read_buffer, boost::is_any_of("\n"));

					// Split version strings into ints
					split(_major, _lines.at(0), boost::is_any_of(","));
					split(_minor, _lines.at(1), boost::is_any_of(","));

					// Replace ";" with "\n" to make the changelog multi-line
					std::string _changelog = _lines.at(2);
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

					for (int i = 0; i < 4; i++)
					{
						// Check minor
						int _ver = boost::lexical_cast<int>(_minor.at(i));
						if (_ver > k2vr_version_minor[i]) updateFound = true;

							// Not to false-alarm in situations like 0.9.1 (local) vs 0.8.2 (remote)
						else if (_ver < k2vr_version_minor[i]) break;
					}

					// Show the message box if update was found
					if (updateFound)
					{
						if (MessageBoxA(nullptr,
						                std::string(
							                "KinectToVR EX "

							                + _major.at(0) + "."
							                + _major.at(1) + "."
							                + _major.at(2) +

							                "\nVersion: "

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
							request.setOpt(new Url("https://api.github.com/repos/KinectToVR/k2vr-installer-gui/releases/latest"));
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
								boost::property_tree::read_json(ss, ptree);
								
								// Iterate over all results
								for (boost::property_tree::ptree::value_type& result : ptree.get_child("assets."))
									for (boost::property_tree::ptree::value_type& field : result.second)
										if (field.first == "browser_download_url") installer_url = field.second.data();

								// Notify about download
								std::thread([] {
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
					LOG(INFO) << "Update failed, string was corrupted.";
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

	//SFGUI Handling -------------------------------------- 
	GUIHandler guiRef;
	// ----------------------------------------------------

	// Update kinect status
	guiRef.updateKinectStatusLabel(kinect);
	// Reconnect Kinect Event Signal
	guiRef.setKinectButtonSignal(kinect);

	//Clear driver memory
	boost::interprocess::shared_memory_object::remove("K2ServerDriverSHM");

	VRcontroller rightController(vr::TrackedControllerRole_RightHand);
	VRcontroller leftController(vr::TrackedControllerRole_LeftHand);

	LOG(INFO) << "Attempting connection to vrsystem.... "; // DEBUG
	vr::EVRInitError eError = vr::VRInitError_None;
	vr::IVRSystem* m_VRSystem = VR_Init(&eError, vr::VRApplication_Overlay);

	LOG_IF(eError != vr::VRInitError_None, ERROR) << "IVRSystem could not be initialised: EVRInitError Code " <<
 static_cast<int>(eError);

	// Initialise the VR Device Handler (For settings)
	VRDeviceHandler vrDeviceHandler(m_VRSystem);
	if (eError == vr::VRInitError_None)
		vrDeviceHandler.initialise();

	//Update driver status
	/************************************************/
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
		setTrackerRolesInVRSettings();
		//VRInput::initialiseVRInput();

		leftController.Connect(m_VRSystem);
		rightController.Connect(m_VRSystem);

		// Todo: implement binding system
		guiRef.loadK2VRIntoBindingsMenu(m_VRSystem);
	}

	guiRef.updateVRStatusLabel(eError);

	//Update driver status
	/************************************************/
	updateServerStatus(guiRef);
	/************************************************/

	KinectSettings::userChangingZero = true;
	if (kinect.kVersion != INVALID)
		kinect.initialiseSkeleton();

	// Since settings are read now, initialize the rest of gui
	guiRef.setVirtualHipsBoxSignals();

	guiRef.initialisePSMoveHandlerIntoGUI(); // Needs the deviceHandlerRef to be set

	// Select backed up or first (we may switch from KV1 to KV2, keeping config)
	guiRef.coptbox->SelectItem(
		VirtualHips::settings.footOption < guiRef.coptbox->GetItemCount() ? VirtualHips::settings.footOption : 0);

	guiRef.coptbox1->SelectItem(VirtualHips::settings.hipsOption);
	guiRef.foptbox->SelectItem(VirtualHips::settings.posOption);

	// Select automatically
	guiRef.bodytrackingselectbox->SelectItem(0);
	guiRef.refreshpsms();

	// Select tracking option automatically
	VirtualHips::settings.bodyTrackingOption = KinectSettings::isKinectPSMS
		                                           ? k_PSMoveFullTracking
		                                           : k_KinectFullTracking;
	bodyTrackingOption_s.trackingOption = static_cast<bodyTrackingOption>(VirtualHips::settings.bodyTrackingOption);
	KinectSettings::positional_tracking_option = VirtualHips::settings.bodyTrackingOption;

	auto ipcThread = new boost::thread(KinectSettings::sendipc);
	ipcThread->detach();

	// Start a new test
	KinectSettings::latencyTestPending = true;
	LOG(INFO) << "Starting a latency test...";
	LOG(INFO) << "Trackers may jump or go somewhere for a while, please don't panic.";
	int framenumber = 3500, checks = 0; // Run a new test "right away"

	while (renderWindow.isOpen() && SFMLsettings::keepRunning)
	{
		if (framenumber >= 500 && checks < 5)
		{
			framenumber = 0;
			checks++;

			// Start a new test
			KinectSettings::latencyTestPending = true;
			LOG(INFO) << "+500 frames have passed since the last check.";
			LOG(INFO) << "Starting a latency test...";
		}
		else framenumber++;

		if (!KinectSettings::isDriverPresent)
		{
			//Update driver status
			/************************************************/
			updateServerStatus(guiRef);
			/************************************************/
		}

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
			rightController.update(deltaT);
			leftController.update(deltaT);

			updateHMDPosAndRot(m_VRSystem);

			/*std::cout << "X: " << float(int(rightController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).x*10))/10.f <<
				" Y: " << float(int(rightController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).y*10))/10.f << 
				" T: " << rightController.GetTrigger() << '\n';*/

			/***********************************************************************************************
			std::cout <<
				"Left: " << KinectSettings::left_foot_psmove.Pose.Position.x << ' ' << KinectSettings::left_foot_psmove.Pose
				.Position.y << ' ' << KinectSettings::left_foot_psmove.Pose.Position.z << '\n' <<
				"Right: " << KinectSettings::right_foot_psmove.Pose.Position.x << ' ' << KinectSettings::right_foot_psmove.Pose.
				Position.y << ' ' << KinectSettings::right_foot_psmove.Pose.Position.z << '\n' <<
				"Waist: " << KinectSettings::waist_psmove.Pose.Position.x << ' ' << KinectSettings::waist_psmove.Pose.Position.y
				<< ' ' << KinectSettings::waist_psmove.Pose.Position.z << "\n\n";
			***********************************************************************************************/

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


			///**********************************************************/

			//vr::TrackedDevicePose_t devicePose[vr::k_unMaxTrackedDeviceCount];
			//
			//for (vr::TrackedDeviceIndex_t index = 0; index < vr::k_unMaxTrackedDeviceCount; index++) {
			//	
			//	if (index != vr::k_unTrackedDeviceIndexInvalid && index != 0) {
			//		
			//		m_VRSystem->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding, 
			//			0, devicePose, vr::k_unMaxTrackedDeviceCount);

			//		if (devicePose[index].bPoseIsValid)
			//		{
			//			if (m_VRSystem->GetTrackedDeviceClass(index) == vr::TrackedDeviceClass_GenericTracker)
			//			{
			//				m_VRSystem->GetTrackedDeviceClass(index);

			//				char pch_value[1024] = { 0 };
			//				m_VRSystem->GetStringTrackedDeviceProperty(index, vr::Prop_ControllerType_String, pch_value, sizeof pch_value);

			//				// L, R, W
			//				if (std::string(pch_value) == "vive_tracker_left_foot")
			//					KinectSettings::trackerIndex[0] = index;
			//				if (std::string(pch_value) == "vive_tracker_right_foot")
			//					KinectSettings::trackerIndex[1] = index;
			//				if (std::string(pch_value) == "vive_tracker_waist")
			//					KinectSettings::trackerIndex[2] = index;

			//				LOG(INFO) <<
			//					devicePose[KinectSettings::trackerIndex[1]].mDeviceToAbsoluteTracking.m[0][3] << ' ' <<
			//					devicePose[KinectSettings::trackerIndex[1]].mDeviceToAbsoluteTracking.m[1][3] << ' ' <<
			//					devicePose[KinectSettings::trackerIndex[1]].mDeviceToAbsoluteTracking.m[2][3] << ' ';
			//			}
			//		}
			//	}
			//}

			///**********************************************************/
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
			VirtualHips::settings.footOption = guiRef.coptbox->GetSelectedItem();

			VirtualHips::saveSettings();
		}
		if (hipsOrientationFilterOption.filterOption != static_cast<hipsRotationFilterOption>(guiRef.coptbox1->
			GetSelectedItem()))
		{
			hipsOrientationFilterOption.filterOption = static_cast<hipsRotationFilterOption>(guiRef.coptbox1->
				GetSelectedItem());
			VirtualHips::settings.hipsOption = guiRef.coptbox1->GetSelectedItem();

			VirtualHips::saveSettings();
		}
		if (positionFilterOption.filterOption != static_cast<positionalFilterOption>(guiRef.foptbox->GetSelectedItem()))
		{
			positionFilterOption.filterOption = static_cast<positionalFilterOption>(guiRef.foptbox->GetSelectedItem());
			VirtualHips::settings.posOption = guiRef.foptbox->GetSelectedItem();

			VirtualHips::saveSettings();
		}
		// We're not updating it, it would mess everything
		/*if (bodyTrackingOption_s.trackingOption != static_cast<bodyTrackingOption>(guiRef.bodytrackingselectbox->
			GetSelectedItem()))
		{
			bodyTrackingOption_s.trackingOption = static_cast<bodyTrackingOption>(guiRef.bodytrackingselectbox->
				GetSelectedItem());
			VirtualHips::settings.bodyTrackingOption = guiRef.bodytrackingselectbox->GetSelectedItem();

			VirtualHips::saveSettings();
		}*/

		if (VirtualHips::settings.bodyTrackingOption == k_PSMoveFullTracking)
			guiRef.psmidbox->Show(true);
		else
			guiRef.psmidbox->Show(false);

		KinectSettings::feet_rotation_option = VirtualHips::settings.footOption;
		KinectSettings::hips_rotation_option = VirtualHips::settings.hipsOption;
		KinectSettings::posOption = VirtualHips::settings.posOption;
		KinectSettings::positional_tracking_option = VirtualHips::settings.bodyTrackingOption;

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
	{
		removeTrackerRolesInVRSettings();
		vr::VR_Shutdown();
	}
}

void spawnDefaultLowerBodyTrackers()
{
	auto activate = new std::thread([]
	{
		// create chrono for limiting loop timing
		using clock = std::chrono::steady_clock;
		auto next_frame = clock::now();

		while (true)
		{
			// measure loop time, let's run at 140/s
			next_frame += std::chrono::milliseconds(1000 / 30);

			HANDLE pingPipe = CreateFile(
				TEXT("\\\\.\\pipe\\TrackersInitPipe"), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0,
				nullptr);
			DWORD Written;

			std::string InitS = "Initialize Trackers!";

			char InitD[1024];
			strcpy_s(InitD, InitS.c_str());

			WriteFile(pingPipe, InitD, sizeof(InitD), &Written, nullptr);
			CloseHandle(pingPipe);

			//Sleep until next frame, if time haven't passed yet
			std::this_thread::sleep_until(next_frame);
		}
	});
}
