#pragma once
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include "VRController.h"
#include <atlbase.h>
#include "KinectSettings.h"
#include "KinectHandlerBase.h"
#include "KinectJoint.h"
#include "VRHelper.h"
#include <TlHelp32.h>
#include "DeviceHandler.h"
#include "PSMoveHandler.h"
#include "VRDeviceHandler.h"
#include <ShellAPI.h>

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <Eigen/Geometry>
#include <codecvt>
#include <string>

#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
//GUI
#include <SFGUI/SFGUI.hpp>
#include <SFGUI/Widgets.hpp>
#include <string>

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/access.hpp>
#include "EigenGLHelpers.h"

class GUIHandler
{
public:
	sfg::ComboBox::Ptr coptbox = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr coptbox1 = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr foptbox = sfg::ComboBox::Create();

	sfg::ComboBox::Ptr psmovebox = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr psmovebox1 = sfg::ComboBox::Create();

	sfg::ComboBox::Ptr bodytrackingselectbox = sfg::ComboBox::Create();

	sfg::ComboBox::Ptr psmovebox_left = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr psmovebox_right = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr psmovebox_waist = sfg::ComboBox::Create();
	sfg::Box::Ptr psmidbox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
	sfg::Button::Ptr TrackerInitButton = sfg::Button::Create("Spawn Trackers");
	sfg::Label::Ptr DriverStatusLabel = sfg::Label::Create("Driver Status: UNKNOWN (Code: -1)");

	GUIHandler()
	{
		guiWindow->SetTitle("KinectToVR EX 0.8.1");
		if (KinectSettings::kinectVersion == 1)
		{
			guiWindow->SetTitle("KinectToVR EX 0.8.1 (Xbox 360/V1)");
		}
		if (KinectSettings::kinectVersion == 2)
		{
			guiWindow->SetTitle("KinectToVR EX 0.8.1 (Xbox One/V2)");
		}

		setDefaultSignals();

		setLineWrapping();
		packElementsIntoMainBox();
		packElementsIntoAdvTrackerBox();
		packElementsIntoCalibrationBox();
		setRequisitions();

		mainNotebook->AppendPage(mainGUIBox, sfg::Label::Create(" Body Trackers "));
		mainNotebook->AppendPage(calibrationBox, sfg::Label::Create(" Offsets "));
		mainNotebook->AppendPage(advancedTrackerBox, sfg::Label::Create(" Options "));

		guiWindow->Add(mainNotebook);
		guiDesktop.Add(guiWindow);

		setScale();

		bool b = guiDesktop.LoadThemeFromFile("main_theme.theme");

		guiDesktop.Update(0.f);
	}

	~GUIHandler()
	{
	}

	void display(sf::RenderWindow& window)
	{
		sfguiRef.Display(window);
	}

	void desktopHandleEvents(sf::Event event)
	{
		guiDesktop.HandleEvent(event);
	}

	void updateDesktop(float d)
	{
		guiDesktop.Update(d);
	}

	void setRequisitions()
	{
		CalibrationEntryrPosX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryrPosY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryrPosZ->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryrRotX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryrRotY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryrRotZ->SetRequisition(sf::Vector2f(40.f, 0.f));

		CalibrationEntrylPosX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntrylPosY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntrylPosZ->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntrylRotX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntrylRotY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntrylRotZ->SetRequisition(sf::Vector2f(40.f, 0.f));

		CalibrationEntryhPosX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryhPosY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryhPosZ->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryhRotX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryhRotY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryhRotZ->SetRequisition(sf::Vector2f(40.f, 0.f));
	}

	void setScale()
	{
		guiWindow->SetAllocation(sf::FloatRect(0.f, 0.f, .8f * SFMLsettings::m_window_width,
		                                       .7f * SFMLsettings::m_window_height));
		guiWindow->SetRequisition(sf::Vector2f(.2f * SFMLsettings::m_window_width,
		                                       .2f * SFMLsettings::m_window_height));
		//Text scaling
		/*
		Window > * > * > Label{
			FontSize : 18;
		/*FontName: data/linden_hill.otf;*/
		/*
		float defaultFontSize = 10.f / 1920.f; // Percentage relative to 1080p
		float scaledFontSize = defaultFontSize * (SFMLsettings::m_window_width / SFMLsettings::windowScale);
		*/
		float scaledFontSize = SFMLsettings::globalFontSize;
		guiDesktop.SetProperty(
			"Window Label, Box, Button, Notebook, CheckButton, ToggleButton, Label, RadioButton, ComboBox, SpinButton",
			"FontSize", scaledFontSize);
	}

	void toggleRotButton()
	{
		KinectRotButton->SetActive(KinectSettings::adjustingKinectRepresentationRot);
	}

	void togglePosButton()
	{
		KinectPosButton->SetActive(KinectSettings::adjustingKinectRepresentationPos);
	}

	void connectPSMoveHandlerGUIEvents()
	{
		if (psMoveHandler.active)
		{
			PSMoveHandlerLabel->SetText("Status: Connected!");
		}
		else
		{
			PSMoveHandlerLabel->SetText("Status: Disconnected!");
		}

		StartPSMoveHandler->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			initialisePSMoveHandlerIntoGUI();
		});
		StopPSMoveHandler->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			if (!psMoveHandler.active)
				return;
			psMoveHandler.shutdown();
			PSMoveHandlerLabel->SetText("Status: Disconnected!");
		});
	}

	void initialisePSMoveHandlerIntoGUI()
	{
		if (psMoveHandler.active)
		{
			LOG(INFO) << "Tried to initialise PSMoveHandler in the GUI, but it was already active";
			return;
		}

		auto errorCode = psMoveHandler.initialise();

		if (psMoveHandler.active)
		{
			static bool addedToVector = false;
			if (!addedToVector)
			{
				v_deviceHandlersRef->push_back(std::make_unique<PSMoveHandler>(psMoveHandler));
				addedToVector = true;
				PSMoveHandlerLabel->SetText("Status: Connected!");
			}
		}
		else
		{
			PSMoveHandlerLabel->SetText(psMoveHandler.connectionMessages[errorCode]);
		}
	}

	void setDefaultSignals()
	{
		//Post VR Tracker Initialisation
		hidePostTrackerInitUI();

		{
			// Font Size Scaling
			FontSizeScale->SetDigits(3);
			FontSizeScale->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				// This checking is required due to some weird anomaly in Sfgui.
				// Without it, it will constantly reupdate the SpinButton event,
				// effectively lagging this for 2-10x as long as it should
				lastFontSizeValue = SFMLsettings::globalFontSize;
				SFMLsettings::globalFontSize = FontSizeScale->GetValue();
				if (lastFontSizeValue != SFMLsettings::globalFontSize)
				{
					setScale();
				}
			});
		}

		ShowSkeletonButton->GetSignal(sfg::Widget::OnLeftClick).Connect([]
		{
			KinectSettings::isSkeletonDrawn = !KinectSettings::isSkeletonDrawn;
		});

		psmovebox->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			KinectSettings::psmh = psmovebox->GetSelectedItem();
			KinectSettings::flashnow[0] = psmovebox->GetSelectedItem();
			KinectSettings::flashnow[1] = true;
		});

		psmovebox1->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			KinectSettings::psmm = psmovebox1->GetSelectedItem();
			KinectSettings::flashnow[0] = psmovebox1->GetSelectedItem();
			KinectSettings::flashnow[1] = true;
		});

		psmovebox_left->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			KinectSettings::psmhidari = psmovebox_left->GetSelectedItem();
			KinectSettings::flashnow[0] = psmovebox_left->GetSelectedItem();
			KinectSettings::flashnow[1] = true;
		});
		psmovebox_right->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			KinectSettings::psmmigi = psmovebox_right->GetSelectedItem();
			KinectSettings::flashnow[0] = psmovebox_right->GetSelectedItem();
			KinectSettings::flashnow[1] = true;
		});
		psmovebox_waist->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			KinectSettings::psmyobu = psmovebox_waist->GetSelectedItem();
			KinectSettings::flashnow[0] = psmovebox_waist->GetSelectedItem();
			KinectSettings::flashnow[1] = true;
		});

		connectPSMoveHandlerGUIEvents();
		setCalibrationSignal();
	}

	void refreshCalibrationMenuValues()
	{
		using namespace KinectSettings;
		CalibrationEntryrPosX->SetValue(manual_offsets[0][0].v[0]);
		CalibrationEntryrPosY->SetValue(manual_offsets[0][0].v[1]);
		CalibrationEntryrPosZ->SetValue(manual_offsets[0][0].v[2]);

		CalibrationEntryrRotX->SetValue(manual_offsets[1][0].v[0]);
		CalibrationEntryrRotY->SetValue(manual_offsets[1][0].v[1]);
		CalibrationEntryrRotZ->SetValue(manual_offsets[1][0].v[2]);

		CalibrationEntrylPosX->SetValue(manual_offsets[0][1].v[0]);
		CalibrationEntrylPosY->SetValue(manual_offsets[0][1].v[1]);
		CalibrationEntrylPosZ->SetValue(manual_offsets[0][1].v[2]);

		CalibrationEntrylRotX->SetValue(manual_offsets[1][1].v[0]);
		CalibrationEntrylRotY->SetValue(manual_offsets[1][1].v[1]);
		CalibrationEntrylRotZ->SetValue(manual_offsets[1][1].v[2]);

		CalibrationEntryhPosX->SetValue(manual_offsets[0][2].v[0]);
		CalibrationEntryhPosY->SetValue(manual_offsets[0][2].v[1]);
		CalibrationEntryhPosZ->SetValue(manual_offsets[0][2].v[2]);

		CalibrationEntryhRotX->SetValue(manual_offsets[1][2].v[0]);
		CalibrationEntryhRotY->SetValue(manual_offsets[1][2].v[1]);
		CalibrationEntryhRotZ->SetValue(manual_offsets[1][2].v[2]);
	}

	void setCalibrationSignal()
	{
		CalibrationEntryrPosX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[0][0].v[0] = CalibrationEntryrPosX->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryrPosY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[0][0].v[1] = CalibrationEntryrPosY->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryrPosZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[0][0].v[2] = CalibrationEntryrPosZ->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationEntrylPosX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[0][1].v[0] = CalibrationEntrylPosX->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntrylPosY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[0][1].v[1] = CalibrationEntrylPosY->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntrylPosZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[0][1].v[2] = CalibrationEntrylPosZ->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationEntryhPosX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[0][2].v[0] = CalibrationEntryhPosX->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryhPosY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[0][2].v[1] = CalibrationEntryhPosY->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryhPosZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[0][2].v[2] = CalibrationEntryhPosZ->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationEntryrRotX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[1][0].v[0] = CalibrationEntryrRotX->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryrRotY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[1][0].v[1] = CalibrationEntryrRotY->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryrRotZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[1][0].v[2] = CalibrationEntryrRotZ->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationEntrylRotX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[1][1].v[0] = CalibrationEntrylRotX->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntrylRotY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[1][1].v[1] = CalibrationEntrylRotY->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntrylRotZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[1][1].v[2] = CalibrationEntrylRotZ->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationEntryhRotX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[1][2].v[0] = CalibrationEntryhRotX->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryhRotY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[1][2].v[1] = CalibrationEntryhRotY->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryhRotZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::manual_offsets[1][2].v[2] = CalibrationEntryhRotZ->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationSaveButton->GetSignal(sfg::Button::OnLeftClick).Connect([this]
			{
				KinectSettings::updateKinectQuaternion();
				KinectSettings::writeKinectSettings();
			}
		);
	}

	void loadK2VRIntoBindingsMenu(vr::IVRSystem*& m_VRSystem)
	{
		// Only scene apps currently actually load into the Bindings menu
		// So, this momentarily opens the vrsystem as a scene, and closes it
		// Which actually allows the menu to stay open, while still functioning as normal
		do
		{
			vr::EVRInitError eError = vr::VRInitError_None;
			vr::VR_Shutdown();
			LOG(INFO) << "(Workaround/Hack) Loading K2VR into bindings menu...";
			m_VRSystem = VR_Init(&eError, vr::VRApplication_Overlay);
			Sleep(100); // Necessary because of SteamVR timing occasionally being too quick to change the scenes
			vr::VR_Shutdown();
			m_VRSystem = VR_Init(&eError, vr::VRApplication_Background);
			LOG_IF(eError != vr::EVRInitError::VRInitError_None, ERROR) <<
 " (Workaround/Hack) VR System failed to reinitialise, attempting again...";
		}
		while (m_VRSystem == nullptr); // Potential Segfault if not actually initialised and used later on
		LOG(INFO) << "(Workaround/Hack) Successfully loaded K2VR into bindings menu!";
	}

	void setVRSceneChangeButtonSignal(vr::IVRSystem*& m_VRSystem)
	{
		ActivateVRSceneTypeButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &m_VRSystem]
		{
			loadK2VRIntoBindingsMenu(m_VRSystem);
		});
	}

	void refreshpsms()
	{
		psmovebox->Clear();
		psmovebox1->Clear();
		psmovebox_left->Clear();
		psmovebox_right->Clear();
		psmovebox_waist->Clear();

		KinectSettings::psmindexidpsm[0].clear();
		KinectSettings::psmindexidpsm[1].clear();

		for (int i = 0; i < 11; i++)
		{
			if (KinectSettings::KVRPSMoveData[i].isValidController)
			{
				psmovebox->AppendItem("PSMove ID: " + boost::lexical_cast<std::string>(i));
				psmovebox1->AppendItem("PSMove ID: " + boost::lexical_cast<std::string>(i));
				psmovebox_left->AppendItem("PSMove ID: " + boost::lexical_cast<std::string>(i));
				psmovebox_right->AppendItem("PSMove ID: " + boost::lexical_cast<std::string>(i));
				psmovebox_waist->AppendItem("PSMove ID: " + boost::lexical_cast<std::string>(i));

				KinectSettings::psmindexidpsm[0].push_back(psmovebox->GetItemCount() - 1);
				KinectSettings::psmindexidpsm[1].push_back(i);
			}
		}
		if (psmovebox->GetItemCount() >= 1)
		{
			psmovebox->SelectItem(0);
			KinectSettings::psmh = 0;
		}
		if (psmovebox1->GetItemCount() >= 2)
		{
			psmovebox1->SelectItem(1);
			KinectSettings::psmm = 1;
		}
		if (psmovebox_left->GetItemCount() >= 1)
		{
			psmovebox_left->SelectItem(0);
			KinectSettings::psmhidari = 0;
		}
		if (psmovebox_right->GetItemCount() >= 2)
		{
			psmovebox_right->SelectItem(1);
			KinectSettings::psmmigi = 1;
		}
		if (psmovebox_waist->GetItemCount() >= 3)
		{
			psmovebox_waist->SelectItem(2);
			KinectSettings::psmyobu = 2;
		}
	}

	void setKinectButtonSignal(KinectHandlerBase& kinect)
	{
		reconKinectButton->GetSignal(sfg::Widget::OnLeftClick).Connect([&kinect]
		{
			kinect.initialise();
		});

		refreshpsmovesbuton->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			refreshpsms();
		});

		refreshpsmovesbuton1->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			refreshpsms();
		});

		refreshpsmovesbuton11->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			refreshpsms();
		});
	}

	static std::wstring s2ws(const std::string& s)
	{
		int len;
		int slength = static_cast<int>(s.length()) + 1;
		len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, nullptr, 0);
		auto buf = new wchar_t[len];
		MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
		std::wstring r(buf);
		delete[] buf;
		return r;
	}

	void ping_InitTrackers()
	{
		if (!KinectSettings::initialised && // If not done yet
			VirtualHips::settings.astartt && KinectSettings::isDriverPresent)
		{
			auto st = new std::thread([this]
				{
					std::this_thread::sleep_for(std::chrono::seconds(3));
					TrackerInitButton->SetLabel("Trackers Initialised - Destroy Trackers");
					spawnDefaultLowerBodyTrackers();

					showPostTrackerInitUI();

					TrackerLastInitButton->SetState(sfg::Widget::State::INSENSITIVE);

					modeTitleBox110->Show(!KinectSettings::isKinectPSMS);
					TDegreeButton->SetValue(KinectSettings::cpoints);
					TrackersConfigSaveButton->Show(true);
					TrackersCalibButton->Show(true);
					expcalibbutton->Show(!KinectSettings::isKinectPSMS);

					space_label->Show(true);
					AutoStartTrackers->Show(true);
					KinectSettings::initialised = true;
				});
		}
		else if (!KinectSettings::isDriverPresent)
			LOG(INFO) << "Not autospawning trackers as the server is not yet connected.";
		else if (KinectSettings::initialised)
			LOG(INFO) << "Not autospawning trackers as they are already initialised.";
	}

	void setTrackerButtonSignals(vr::IVRSystem*& m_VRSystem)
	{
		TrackerInitButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			if (!KinectSettings::initialised)
			{
				/*
				bool reuseLastTrackers = true;
				if (reuseLastTrackers) {
					LOG(INFO) << "SPAWNING TRACKERS FROM LAST OPEN, MAY BE ISSUES";

					// Load Last Set of trackers used

					// Spawn
				}
				*/
				TrackerInitButton->SetLabel("Trackers Initialised - Destroy Trackers");
				spawnDefaultLowerBodyTrackers();

				showPostTrackerInitUI();

				TrackerLastInitButton->SetState(sfg::Widget::State::INSENSITIVE);

				modeTitleBox110->Show(!KinectSettings::isKinectPSMS);
				TDegreeButton->SetValue(KinectSettings::cpoints);
				TrackersConfigSaveButton->Show(true);
				TrackersCalibButton->Show(true);
				expcalibbutton->Show(!KinectSettings::isKinectPSMS); //Manual only if PSMS

				space_label->Show(true);
				AutoStartTrackers->Show(true);
				KinectSettings::initialised = true;
			}
			else
			{
				KinectSettings::initialised = false;
				TrackerInitButton->SetLabel("Spawn Trackers");
			}
		});
		// Make sure that users don't get confused and hit the spawn last button when they don't need it
		/*bool foundCachedTrackers = trackerConfigExists() ? true : false;
		TrackerLastInitButton->Show(foundCachedTrackers);*/

		//Initialise trackers if $(Conditions)
		ping_InitTrackers();
	}

	void updateTrackerInitButtonLabelFail()
	{
		TrackerInitButton->SetLabel("Input Emulator not connected! Can't init trackers");
	}

	void setReconnectControllerButtonSignal(VRcontroller& left, VRcontroller& right, vr::IVRSystem*& sys
	)
	{
		ReconControllersButton->GetSignal(sfg::Button::OnLeftClick).Connect([&left, &right, &sys, this]
		{
			std::stringstream stream;
			stream <<
				"If controller input isn't working, press this to reconnect them.\n Make sure both are on, and not in standby.\n";
			if (right.Connect(sys))
			{
				stream << "RIGHT: OK!\t";
			}
			else
			{
				stream << "RIGHT: DISCONNECTED!\t";
			}
			if (left.Connect(sys))
			{
				stream << "LEFT: OK!\t";
			}
			else
			{
				stream << "LEFT: DISCONNECTED!\t";
			}
			ReconControllersLabel->SetText(stream.str());
		});
	}

	vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix)
	{
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

	vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix)
	{
		vr::HmdVector3_t vector;

		vector.v[0] = matrix.m[0][3];
		vector.v[1] = matrix.m[1][3];
		vector.v[2] = matrix.m[2][3];

		return vector;
	}

	void getsvrposesnrots(vr::DriverPose_t in_out)
	{
		vr::EVRInitError error;
		vr::IVRSystem* system = VR_Init(&error, vr::VRApplication_Background);

		vr::TrackedDevicePose_t trackedDevicePose;
		vr::TrackedDevicePose_t trackedControllerPose;
		vr::VRControllerState_t controllerState;
		vr::HmdMatrix34_t poseMatrix;
		vr::HmdVector3_t position;
		vr::HmdQuaternion_t quaternion;
		vr::VRControllerState_t state;

		system->GetControllerState(0, &state, sizeof(state));
		vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
		// print positiona data for the HMD.
		poseMatrix = trackedDevicePose.mDeviceToAbsoluteTracking;
		// This matrix contains all positional and rotational data.
		position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
		quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);

		in_out.qRotation = quaternion;

		in_out.vecPosition[0] = position.v[0];
		in_out.vecPosition[1] = position.v[1];
		in_out.vecPosition[2] = position.v[2];

		//// Process SteamVR device states
		//for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
		//{
		//    if (!system->IsTrackedDeviceConnected(unDevice))
		//        continue;

		//    vr::VRControllerState_t state;
		//    if (system->GetControllerState(unDevice, &state, sizeof(state)))
		//    {
		//        vr::TrackedDevicePose_t trackedDevicePose;
		//        vr::TrackedDevicePose_t trackedControllerPose;
		//        vr::VRControllerState_t controllerState;
		//        vr::HmdMatrix34_t poseMatrix;
		//        vr::HmdVector3_t position;
		//        vr::HmdQuaternion_t quaternion;
		//        vr::ETrackedDeviceClass trackedDeviceClass = vr::VRSystem()->GetTrackedDeviceClass(unDevice);

		//        switch (trackedDeviceClass) {
		//        case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:
		//            vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
		//            // print positiona data for the HMD.
		//            poseMatrix = trackedDevicePose.mDeviceToAbsoluteTracking; // This matrix contains all positional and rotational data.
		//            position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
		//            quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);

		//            in_out[0].qRotation = quaternion;

		//            std::cout << glm::vec3(glm::eulerAngles(glm::quat(quaternion.w, quaternion.x, quaternion.y, quaternion.z))).y << std::endl;

		//            in_out[0].vecPosition[0] = position.v[0];
		//            in_out[0].vecPosition[1] = position.v[1];
		//            in_out[0].vecPosition[2] = position.v[2];

		//            break;

		//        case vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker:
		//            vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
		//            // print positiona data for a general vive tracker.
		//            break;

		//        case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller:
		//            vr::VRSystem()->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState,
		//                sizeof(controllerState), &trackedControllerPose);
		//            poseMatrix = trackedControllerPose.mDeviceToAbsoluteTracking; // This matrix contains all positional and rotational data.
		//            position = GetPosition(trackedControllerPose.mDeviceToAbsoluteTracking);
		//            quaternion = GetRotation(trackedControllerPose.mDeviceToAbsoluteTracking);

		//            auto trackedControllerRole = vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(unDevice);
		//            std::string whichHand = "";
		//            if (trackedControllerRole == vr::TrackedControllerRole_LeftHand)
		//            {
		//                whichHand = "LeftHand";
		//            }
		//            else if (trackedControllerRole == vr::TrackedControllerRole_RightHand)
		//            {
		//                whichHand = "RightHand";
		//            }

		//            switch (trackedControllerRole)
		//            {
		//            case vr::TrackedControllerRole_Invalid:
		//                // invalid
		//                break;

		//            case vr::TrackedControllerRole_LeftHand:
		//                in_out[1].qRotation = quaternion;
		//                in_out[1].vecPosition[0] = position.v[0];
		//                in_out[1].vecPosition[1] = position.v[1];
		//                in_out[1].vecPosition[2] = position.v[2];
		//                break;

		//            case vr::TrackedControllerRole_RightHand:
		//                in_out[2].qRotation = quaternion;
		//                in_out[2].vecPosition[0] = position.v[0];
		//                in_out[2].vecPosition[1] = position.v[1];
		//                in_out[2].vecPosition[2] = position.v[2];
		//                break;
		//            }

		//            break;
		//        }

		//    }
		//}
	}

	void setLineWrapping()
	{
		CalibrationSettingsLabel->SetLineWrap(true);
		CalibrationSettingsLabel->SetRequisition(sf::Vector2f(600.f, 20.f));
	}

	void packElementsIntoMainBox()
	{
		//Statuses are at the top
		mainGUIBox->Pack(KinectStatusLabel);
		mainGUIBox->Pack(DriverStatusLabel);
		mainGUIBox->Pack(SteamVRStatusLabel);

		auto fontSizeBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		fontSizeBox->Pack(FontSizeScaleLabel);
		fontSizeBox->Pack(FontSizeScale);
		mainGUIBox->Pack(fontSizeBox);

		auto recBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		recBox->Pack(reconKinectButton);
		recBox->Pack(ReconControllersButton);

		mainGUIBox->Pack(recBox);
		mainGUIBox->Pack(TrackerInitButton);

		//setHipScaleBox();
		mainGUIBox->Pack(ShowSkeletonButton);

		modeTitleBox110->Pack(sfg::Label::Create("Calibration Points (Reccomended: 3)"));
		modeTitleBox110->Pack(TDegreeButton);

		mainGUIBox->Pack(modeTitleBox110);
		modeTitleBox110->Show(false);

		mainGUIBox->Pack(TrackersConfigSaveButton);
		TrackersConfigSaveButton->Show(false);

		mainGUIBox->Pack(sfg::Label::Create(""));
		mainGUIBox->Pack(sfg::Label::Create(""));
		mainGUIBox->Pack(sfg::Label::Create(""));

		mainGUIBox->Pack(TrackersCalibButton);
		TrackersCalibButton->Show(false);

		mainGUIBox->Pack(expcalibbutton);
		expcalibbutton->Show(false);
	}

	void packElementsIntoAdvTrackerBox()
	{
		// Only if we're using psms
		if (KinectSettings::isKinectPSMS)
		{
			sfg::Box::Ptr horizontalPSMBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
			horizontalPSMBox->Pack(StartPSMoveHandler);
			horizontalPSMBox->Pack(StopPSMoveHandler);

			advancedTrackerBox->Pack(sfg::Label::Create("PSMoveSerive handler"));
			advancedTrackerBox->Pack(horizontalPSMBox);
		}

		advancedTrackerBox->Pack(sfg::Label::Create("Body tracking option"));

		sfg::Box::Ptr selectoptionbox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
		selectoptionbox->Pack(sfg::Label::Create("Tracking option for body trackers"));
		selectoptionbox->Pack(bodytrackingselectbox);
		advancedTrackerBox->Pack(selectoptionbox);

		// Append just a needed one
		bodytrackingselectbox->AppendItem(
			KinectSettings::isKinectPSMS ? "PSMove body tracking" : "Kinect body tracking");

		// Select corresponding (only) device
		bodytrackingselectbox->SelectItem(0);

		sfg::Box::Ptr psmleftidbox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
		sfg::Box::Ptr psmrightidbox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
		sfg::Box::Ptr psmhipsidbox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);

		psmleftidbox->Pack(sfg::Label::Create("Left Foot tracker PSMove ID"));
		psmleftidbox->Pack(psmovebox_left);

		psmrightidbox->Pack(sfg::Label::Create("Right Foot tracker PSMove ID"));
		psmrightidbox->Pack(psmovebox_right);

		psmhipsidbox->Pack(sfg::Label::Create("Hips tracker PSMove ID"));
		psmhipsidbox->Pack(psmovebox_waist);

		psmidbox->Pack(psmleftidbox);
		psmidbox->Pack(psmrightidbox);
		psmidbox->Pack(psmhipsidbox);
		psmidbox->Pack(refreshpsmovesbuton1);

		advancedTrackerBox->Pack(psmidbox);
		advancedTrackerBox->Pack(sfg::Label::Create("Trackers orientation filter"));

		auto box1 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		box1->Pack(sfg::Label::Create("Feet trackers orientation tracking"));
		auto box2 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		box2->Pack(sfg::Label::Create("Waist tracker orientation tracking"));

		box1->Pack(coptbox);
		box2->Pack(coptbox1);

		coptbox->AppendItem("Enable Feet Rotation");
		coptbox->AppendItem("Disable Feet Rotation");
		coptbox->AppendItem("Disable Feet Yaw (+Y)");
		coptbox->AppendItem("Use Head Orientation");

		/*
		* UNTIL TRIPING EXPLAINS WTF IS GOING ON HERE, IT'S COMMENTED OUT
		*/

		//coptbox->AppendItem("Use Tracker Orientation");
		//coptbox->AppendItem("Mixed Tracker Orientation");

		/*
		* UNTIL TRIPING EXPLAINS WTF IS GOING ON HERE, IT'S COMMENTED OUT
		*/

		// Only if we're using kinect v1
		if (KinectSettings::kinectVersion == 1)
			coptbox->AppendItem("Math-Based Rotation");

		coptbox1->AppendItem("Enable Waist Rotation");
		coptbox1->AppendItem("Disable Waist Rotation");
		coptbox1->AppendItem("Use Head Orientation");

		advancedTrackerBox->Pack(box1);
		advancedTrackerBox->Pack(box2);

		advancedTrackerBox->Pack(sfg::Label::Create("Positional filtering options"));

		auto box11 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		box11->Pack(sfg::Label::Create("Positional tracking filter"));
		box11->Pack(foptbox);

		/*
		* EKF is really nice if you are planning to stand in place
		* Low pass filter is fot those, who have lit room and kinda bad tracking
		* interpolation predicts kinect results so it makes you faster but
		*    literally teleports trackers to next position, should be connected with lpf
		* "Magic Touch" actually disables filters, but only on k2vr side
		*    idk how it works in normal life, but driver should detect it
		*    and apply it's own filters (driver's source code will stay unpublished
		*    until arduvr is officially announced and promoted)
		*/

		foptbox->AppendItem("Extended Kalman filter - smooths every jitter"); //use ekf in k2vr
		foptbox->AppendItem("Low Pass filter - quite fast, adaptive smoothing"); //use lpf in k2vr
		foptbox->AppendItem("Linear Interpolation - gentle, continuous smoothing"); //use glm::mix in k2vr
		foptbox->AppendItem("No filter - realtime results, no smoothing");

		advancedTrackerBox->Pack(box11);
		advancedTrackerBox->Pack(space_label);

		sfg::Box::Ptr astartbox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		astartbox->Pack(AutoStartTrackers);

		space_label->Show(false);
		AutoStartTrackers->Show(false);
		advancedTrackerBox->Pack(astartbox);
	}

	void packElementsIntoCalibrationBox()
	{
		sfg::Box::Ptr verticalBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL);

		verticalBox->Pack(
			sfg::Label::Create("This tab allows you to move and rotate trackers to fine tune the calibration values."));

		verticalBox->Pack(
			sfg::Label::Create("This is generally used to slighty adjust position and orientation of trackers."));
		verticalBox->Pack(sfg::Label::Create("\nRotation is in degrees and position is declared in meters.\n "));

		auto horizontalrPosBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontalrPosBox->Pack(CalibrationrPosLabel);
		CalibrationEntryrPosX->SetDigits(4);
		horizontalrPosBox->Pack(CalibrationEntryrPosX);
		CalibrationEntryrPosY->SetDigits(4);
		horizontalrPosBox->Pack(CalibrationEntryrPosY);
		CalibrationEntryrPosZ->SetDigits(4);
		horizontalrPosBox->Pack(CalibrationEntryrPosZ);
		verticalBox->Pack(horizontalrPosBox);

		auto horizontalrRotBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontalrRotBox->Pack(CalibrationrRotLabel);
		CalibrationEntryrRotX->SetDigits(4);
		horizontalrRotBox->Pack(CalibrationEntryrRotX);
		CalibrationEntryrRotY->SetDigits(4);
		horizontalrRotBox->Pack(CalibrationEntryrRotY);
		CalibrationEntryrRotZ->SetDigits(4);
		horizontalrRotBox->Pack(CalibrationEntryrRotZ);
		verticalBox->Pack(horizontalrRotBox);

		auto horizontallPosBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontallPosBox->Pack(CalibrationlPosLabel);
		CalibrationEntrylPosX->SetDigits(4);
		horizontallPosBox->Pack(CalibrationEntrylPosX);
		CalibrationEntrylPosY->SetDigits(4);
		horizontallPosBox->Pack(CalibrationEntrylPosY);
		CalibrationEntrylPosZ->SetDigits(4);
		horizontallPosBox->Pack(CalibrationEntrylPosZ);
		verticalBox->Pack(horizontallPosBox);

		auto horizontallRotBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontallRotBox->Pack(CalibrationlRotLabel);
		CalibrationEntrylRotX->SetDigits(4);
		horizontallRotBox->Pack(CalibrationEntrylRotX);
		CalibrationEntrylRotY->SetDigits(4);
		horizontallRotBox->Pack(CalibrationEntrylRotY);
		CalibrationEntrylRotZ->SetDigits(4);
		horizontallRotBox->Pack(CalibrationEntrylRotZ);
		verticalBox->Pack(horizontallRotBox);

		auto horizontalhPosBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontalhPosBox->Pack(CalibrationhPosLabel);
		CalibrationEntryhPosX->SetDigits(4);
		horizontalhPosBox->Pack(CalibrationEntryhPosX);
		CalibrationEntryhPosY->SetDigits(4);
		horizontalhPosBox->Pack(CalibrationEntryhPosY);
		CalibrationEntryhPosZ->SetDigits(4);
		horizontalhPosBox->Pack(CalibrationEntryhPosZ);
		verticalBox->Pack(horizontalhPosBox);

		auto horizontalhRotBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontalhRotBox->Pack(CalibrationhRotLabel);
		CalibrationEntryhRotX->SetDigits(4);
		horizontalhRotBox->Pack(CalibrationEntryhRotX);
		CalibrationEntryhRotY->SetDigits(4);
		horizontalhRotBox->Pack(CalibrationEntryhRotY);
		CalibrationEntryhRotZ->SetDigits(4);
		horizontalhRotBox->Pack(CalibrationEntryhRotZ);
		verticalBox->Pack(horizontalhRotBox);

		verticalBox->Pack(sfg::Label::Create(" "));

		verticalBox->Pack(CalibrationSaveButton);

		verticalBox->Pack(sfg::Label::Create(" "));

		verticalBox->Pack(ActivateVRSceneTypeButton);

		calibrationBox->Pack(verticalBox);
	}

	void updateKinectStatusLabel(KinectHandlerBase& kinect)
	{
		HRESULT status = kinect.getStatusResult();
		if (kinect.isInitialised())
		{
			if (status == lastKinectStatus)
				return; // No need to waste time updating it;
			if (kinect.isPSMS)
			{
				KinectStatusLabel->SetText("PSMoveService Mode!");
			}
			else
			{
				switch (status)
				{
				case S_OK:
					{
						KinectStatusLabel->SetText("Kinect Status: Success!");
						break;
					}
				default:
					KinectStatusLabel->SetText("Kinect Status: ERROR " + kinect.statusResultString(status));
					break;
				}
			}
		}
		else
			updateKinectStatusLabelDisconnected();
		if (status != lastKinectStatus)
		{
			LOG(INFO) << "Kinect Status changed to: " << KinectStatusLabel->GetText().toAnsiString();
			lastKinectStatus = status;
		}
	}

	void updateVRStatusLabel(vr::EVRInitError eError)
	{
		if (eError == vr::VRInitError_None)
			SteamVRStatusLabel->SetText("SteamVR Status: Success!");
		else
			SteamVRStatusLabel->SetText(
				"SteamVR Status: ERROR " + std::to_string(eError) +
				"\nPlease restart K2VR with SteamVR successfully running!");
	}

	void updateWithNewWindowSize(sf::Vector2f size)
	{
		guiWindow->SetAllocation(sf::FloatRect(0.f, 0.f, .4f * size.x, .4f * size.y));
		//setScale();
		//guiWindow->SetAllocation(sf::FloatRect(size.x - width, 0.f, width, size.y));
		//mGUI.SideBar->SetAllocation(sf::FloatRect(0.f, 0.f, width, size.y));
	}

	using PointSet = Eigen::Matrix<float, 3, Eigen::Dynamic>;

	std::tuple<Eigen::Matrix3f, Eigen::Vector3f> rigid_transform_3D(const PointSet& A, const PointSet& B)
	{
		static_assert(PointSet::RowsAtCompileTime == 3);
		assert(A.cols() == B.cols());

		// find mean column wise
		const Eigen::Vector3f centroid_A = A.rowwise().mean();
		const Eigen::Vector3f centroid_B = B.rowwise().mean();

		// subtract mean
		PointSet Am = A.colwise() - centroid_A;
		PointSet Bm = B.colwise() - centroid_B;

		PointSet H = Am * Bm.transpose();

		//
		//# sanity check
		//#if linalg.matrix_rank(H) < 3:
		//	#    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))
		//

		// find rotation
		Eigen::JacobiSVD<Eigen::Matrix3Xf> svd = H.jacobiSvd(
			Eigen::DecompositionOptions::ComputeFullU | Eigen::DecompositionOptions::ComputeFullV);
		const Eigen::Matrix3f& U = svd.matrixU();
		Eigen::MatrixXf V = svd.matrixV();
		Eigen::Matrix3f R = V * U.transpose();

		// special reflection case
		if (R.determinant() < 0.0f)
		{
			V.col(2) *= -1.0f;
			R = V * U.transpose();
		}

		const Eigen::Vector3f t = -R * centroid_A + centroid_B;

		return std::make_tuple(R, t);
	}

	void killProcessByName(const char* filename)
	{
		USES_CONVERSION;
		HANDLE hSnapShot = CreateToolhelp32Snapshot(TH32CS_SNAPALL, NULL);
		PROCESSENTRY32 pEntry;
		pEntry.dwSize = sizeof(pEntry);
		BOOL hRes = Process32First(hSnapShot, &pEntry);
		while (hRes)
		{
			if (strcmp(W2A(pEntry.szExeFile), filename) == 0)
			{
				HANDLE hProcess = OpenProcess(PROCESS_TERMINATE, 0,
				                              pEntry.th32ProcessID);
				if (hProcess != nullptr)
				{
					TerminateProcess(hProcess, 9);
					CloseHandle(hProcess);
				}
			}
			hRes = Process32Next(hSnapShot, &pEntry);
		}
		CloseHandle(hSnapShot);
	}

	void setVirtualHipsBoxSignals()
	{
		using namespace VirtualHips;

		VirtualHipHeightFromHMDButton->SetDigits(2);
		DegreeButton->SetDigits(2);
		TDegreeButton->SetDigits(0);

		VirtualHipLockToHeadButton->GetSignal(sfg::RadioButton::OnToggle).Connect([this]
			{
				settings.positionAccountsForFootTrackers = !VirtualHipLockToHeadButton->IsActive();
			}
		);
		VirtualHipLockToFeetButton->GetSignal(sfg::RadioButton::OnToggle).Connect([this]
			{
				settings.positionAccountsForFootTrackers = VirtualHipLockToFeetButton->IsActive();
			}
		);

		AutoStartTrackers->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			settings.astartt = !settings.astartt;
			if (settings.astartt)
			{
				AutoStartTrackers->SetLabel("Initialise trackers automatically CURRENT: YES");
			}
			else
			{
				AutoStartTrackers->SetLabel("Initialise trackers automatically CURRENT: NO");
			}
		});

		if (settings.astartt)
		{
			AutoStartTrackers->SetLabel("Initialise trackers automatically CURRENT: YES");
		}
		else
		{
			AutoStartTrackers->SetLabel("Initialise trackers automatically CURRENT: NO");
		}

		VirtualHipHeightFromHMDButton->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.heightFromHMD = VirtualHipHeightFromHMDButton->GetValue();
				KinectSettings::huoffsets.v[0] = VirtualHipHeightFromHMDButton->GetValue();
			}
		);

		DegreeButton->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.hmdegree = DegreeButton->GetValue();
				KinectSettings::hroffset = DegreeButton->GetValue();
			}
		);
		TDegreeButton->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.tdegree = TDegreeButton->GetValue();
				KinectSettings::cpoints = TDegreeButton->GetValue();
			}
		);
		VirtualHipFollowHMDLean->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			settings.positionFollowsHMDLean = (VirtualHipFollowHMDLean->IsActive());
		});

		VirtualHipConfigSaveButton->GetSignal(sfg::Button::OnLeftClick).Connect([this]
			{
				saveSettings();
			}
		);

		TrackersConfigSaveButton->GetSignal(sfg::Button::OnLeftClick).Connect([this]
			{
				saveSettings();
			}
		);

		expcalibbutton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			KinectSettings::expcalib = !KinectSettings::expcalib;
		});

		TrackersCalibButton->GetSignal(sfg::Button::OnLeftClick).Connect([this]
		{
			vr::EVRInitError error;
			vr::IVRSystem* csystem = VR_Init(&error, vr::VRApplication_Background);
			vr::HmdMatrix34_t cposeMatrix;
			vr::HmdVector3_t cposition;
			vr::HmdQuaternion_t cquaternion;
			vr::TrackedDevicePose_t ctrackedControllerPose;
			vr::VRControllerState_t ccontrollerState;
			vr::VRControllerState_t cstate;

			if (!KinectSettings::isCalibrating)
			{
				KinectSettings::isCalibrating = true;

				if (!KinectSettings::expcalib)
				{
					auto t1 = new std::thread([this]()
					{
						KinectSettings::matrixes_calibrated = true;
						KinectSettings::jcalib = true;

						Eigen::AngleAxisd rollAngle(0.f, Eigen::Vector3d::UnitZ());
						Eigen::AngleAxisd yawAngle(0.f, Eigen::Vector3d::UnitY());
						Eigen::AngleAxisd pitchAngle(0.f, Eigen::Vector3d::UnitX());
						Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

						Eigen::Matrix3d rotationMatrix = q.matrix();
						KinectSettings::calibration_rotation = rotationMatrix.cast<float>();

						bool firstTime = true;
						double yawtmp = 0, pitchtmp = 0;
						while (!KinectSettings::isTriggerPressed[0] || !KinectSettings::isTriggerPressed[1])
						{
							std::this_thread::sleep_for(std::chrono::milliseconds(300));
							/******************************************************************************/
							TrackersCalibButton->SetLabel(
								std::string(
									"Adjust position with Thumbsticks (LGrip: Fine adjust, RGrip: Switch to Rotation, Triggers: Confirm)")
								.c_str());
							/******************************************************************************/

							while (!KinectSettings::isGripPressed[0] && (!KinectSettings::isTriggerPressed[0] || !
								KinectSettings::isTriggerPressed[1]))
							{
								if (!KinectSettings::isGripPressed[1])
								{
									KinectSettings::calibration_translation(0) += VRInput::trackpadpose[1].x * .01f;
									KinectSettings::calibration_translation(1) += VRInput::trackpadpose[0].y * .01f;
									KinectSettings::calibration_translation(2) += -VRInput::trackpadpose[1].y * .01f;
								}
								else
								{
									KinectSettings::calibration_translation(0) += VRInput::trackpadpose[1].x * .001f;
									KinectSettings::calibration_translation(1) += VRInput::trackpadpose[0].y * .001f;
									KinectSettings::calibration_translation(2) += -VRInput::trackpadpose[1].y * .001f;
								}

								std::this_thread::sleep_for(std::chrono::milliseconds(5));
								if (!KinectSettings::isCalibrating) break;
							}

							if (firstTime)
								KinectSettings::calibration_origin = Eigen::Vector3f(
									KinectSettings::kinect_m_positions[2].v[0],
									KinectSettings::kinect_m_positions[2].v[1],
									KinectSettings::kinect_m_positions[2].v[2]);
							firstTime = false;

							std::this_thread::sleep_for(std::chrono::milliseconds(300));
							/******************************************************************************/
							TrackersCalibButton->SetLabel(
								std::string(
									"Adjust rotation with Thumbsticks (LGrip: Fine adjust, RGrip: Switch to Position, Triggers: Confirm)")
								.c_str());
							/******************************************************************************/

							while (!KinectSettings::isGripPressed[0] && (!KinectSettings::isTriggerPressed[0] || !
								KinectSettings::isTriggerPressed[1]))
							{
								if (!KinectSettings::isGripPressed[1])
								{
									yawtmp += VRInput::trackpadpose[1].x * M_PI / 280.f;
									pitchtmp += VRInput::trackpadpose[0].y * M_PI / 280.f;
								}
								else
								{
									yawtmp += (VRInput::trackpadpose[1].x * M_PI / 280.f) * .1f;
									pitchtmp += (VRInput::trackpadpose[0].y * M_PI / 280.f) * .1f;
								}

								Eigen::AngleAxisd rollAngle(0.f, Eigen::Vector3d::UnitZ());
								Eigen::AngleAxisd yawAngle(yawtmp, Eigen::Vector3d::UnitY());
								Eigen::AngleAxisd pitchAngle(pitchtmp, Eigen::Vector3d::UnitX());
								Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

								Eigen::Matrix3d rotationMatrix = q.matrix();
								KinectSettings::calibration_rotation = rotationMatrix.cast<float>();

								std::this_thread::sleep_for(std::chrono::milliseconds(5));
								KinectSettings::calibration_trackers_yaw = glm::degrees(yawtmp);

								// TODO: Check if it's not the opposite
								KinectSettings::calibration_kinect_pitch = pitchtmp; // This one's in radians

								if (!KinectSettings::isCalibrating) break;
							}

							if (!KinectSettings::isCalibrating)
							{
								KinectSettings::calibration_origin = settings.caliborigin;
								KinectSettings::calibration_rotation = settings.rcR_matT;
								KinectSettings::calibration_translation = settings.rcT_matT;
								KinectSettings::calibration_trackers_yaw = settings.tryawst;

								KinectSettings::calibration_kinect_pitch = settings.kinpitchst;
								break;
							}
						}

						std::this_thread::sleep_for(std::chrono::seconds(1));

						if (!KinectSettings::isCalibrating)
						{
							settings.caliborigin = KinectSettings::calibration_origin;
							settings.rcR_matT = KinectSettings::calibration_rotation;
							settings.rcT_matT = KinectSettings::calibration_translation;
							settings.tryawst = glm::degrees(yawtmp);
							settings.kinpitchst = pitchtmp;
						}

						KinectSettings::matrixes_calibrated = true;
						settings.rtcalib = true;

						TrackersCalibButton->SetLabel(
							std::string(!KinectSettings::isCalibrating
								            ? "Calibration aborted! Hit me 2 times to re-calibrate!"
								            : "Done! Hit me 2 times to re-calibrate!").c_str());
						TrackersCalibButton->SetState(sfg::Widget::State::NORMAL);

						saveSettings();
					});
				}
				else
				{
					auto t1 = new std::thread([this]()
					{
						vr::EVRInitError error;
						vr::IVRSystem* system = VR_Init(&error, vr::VRApplication_Background);
						vr::TrackedDevicePose_t trackedDevicePose;
						vr::TrackedDevicePose_t trackedControllerPose;
						vr::VRControllerState_t controllerState;
						vr::HmdMatrix34_t poseMatrix;
						vr::HmdVector3_t position;
						vr::HmdQuaternion_t quaternion;

						std::vector<vr::DriverPose_t> spose;
						std::vector<vr::HmdVector3d_t> hpose;

						KinectSettings::ismatrixcalibrated = false;
						KinectSettings::matrixes_calibrated = false;
						KinectSettings::calibration_origin = Eigen::Vector3f(0, 0, 0);
						settings.caliborigin = KinectSettings::calibration_origin;

						for (int ipoint = 1; ipoint <= KinectSettings::cpoints; ipoint++)
						{
							if (!KinectSettings::isCalibrating) break;
							vr::DriverPose_t ispose;
							vr::HmdVector3d_t ihpose;

							// Tell the user to move somewhere else, ! means WE WANT IT
							for (auto i = 4; i >= 0; i--)
							{
								TrackersCalibButton->SetLabel(
									std::string(
										"[Time left: " + boost::lexical_cast<std::string>(i) + "s]     "
										"Point " + boost::lexical_cast<std::string>(ipoint) +
										": Move somewhere else! "
										"     [Time left: " + boost::lexical_cast<std::string>(i) + "s]").
									c_str());
								std::this_thread::sleep_for(std::chrono::seconds(1));
								if (!KinectSettings::isCalibrating) break;
							}
							if (!KinectSettings::isCalibrating) break;

							// NEW Stand still for 3 seconds, lil' idiot-proofing
							for (auto i = 3; i >= 0; i--)
							{
								TrackersCalibButton->SetLabel(
									std::string(
										"[Time left: " + boost::lexical_cast<std::string>(i) + "s]     "
										"Point " + boost::lexical_cast<std::string>(ipoint) +
										": Please stand still!"
										"     [Time left: " + boost::lexical_cast<std::string>(i) + "s]").
									c_str());
								std::this_thread::sleep_for(std::chrono::seconds(1));
								if (!KinectSettings::isCalibrating) break;
							}
							if (!KinectSettings::isCalibrating) break;

							vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
								vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
							position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);

							ispose.vecPosition[0] = position.v[0] - KinectSettings::trackingOriginPosition.v[0];
							ispose.vecPosition[1] = position.v[1] - KinectSettings::trackingOriginPosition.v[1];
							ispose.vecPosition[2] = position.v[2] - KinectSettings::trackingOriginPosition.v[2];

							Eigen::AngleAxisd rollAngle(0.f, Eigen::Vector3d::UnitZ());
							Eigen::AngleAxisd yawAngle(-KinectSettings::svrhmdyaw, Eigen::Vector3d::UnitY());
							Eigen::AngleAxisd pitchAngle(0.f, Eigen::Vector3d::UnitX());

							Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

							Eigen::Vector3d in(ispose.vecPosition[0], ispose.vecPosition[1], ispose.vecPosition[2]);
							Eigen::Vector3d out = q * in;

							ispose.vecPosition[0] = out(0);
							ispose.vecPosition[1] = out(1);
							ispose.vecPosition[2] = out(2);

							for (auto i = 0; i < 3; i++)ihpose.v[i] = KinectSettings::kinect_m_positions[0].v[i];

							// Tell we got it? May be removed TODO: TODO:
							TrackersCalibButton->SetLabel(
								std::string(
									"Position captured: Point " + boost::lexical_cast<std::string>(ipoint) + "!")
								.c_str());
							std::this_thread::sleep_for(std::chrono::seconds(1));
							// Tell we got it? May be removed TODO: TODO:

							spose.push_back(ispose);
							hpose.push_back(ihpose);

							if (!KinectSettings::isCalibrating) break;
						}
						if (!KinectSettings::isCalibrating)
						{
							KinectSettings::calibration_origin = settings.caliborigin;
							KinectSettings::calibration_rotation = settings.rcR_matT;
							KinectSettings::calibration_translation = settings.rcT_matT;
							KinectSettings::calibration_trackers_yaw = settings.tryawst;

							KinectSettings::calibration_kinect_pitch = settings.kinpitchst;
						}

						if (KinectSettings::isCalibrating)
						{
							Eigen::Matrix<float, 3, Eigen::Dynamic> spoints(3, KinectSettings::cpoints), hpoints(
								                                        3, KinectSettings::cpoints);

							for (int ipoint = 0; ipoint < KinectSettings::cpoints; ipoint++)
							{
								spoints(0, ipoint) = spose.at(ipoint).vecPosition[0];
								spoints(1, ipoint) = spose.at(ipoint).vecPosition[1];
								spoints(2, ipoint) = spose.at(ipoint).vecPosition[2];

								hpoints(0, ipoint) = hpose.at(ipoint).v[0];
								hpoints(1, ipoint) = hpose.at(ipoint).v[1];
								hpoints(2, ipoint) = hpose.at(ipoint).v[2];
								if (!KinectSettings::isCalibrating) break;
							}

							PointSet A = hpoints, B = spoints;

							const auto [ret_R, ret_t] = rigid_transform_3D(A, B);

							std::cout << "\nHead points\n" << A << "\nSteamvr points\n" << B << "\nTranslation\n" <<
								ret_t << "\nRotation\n" << ret_R << '\n';

							PointSet B2 = (ret_R * A).colwise() + ret_t;

							PointSet err = B2 - B;
							err = err.cwiseProduct(err);
							const float rmse = std::sqrt(err.sum() / static_cast<float>(3));

							// Used in Korejan's algo for checking results
							/*if (rmse < 0.01f)
								std::cout << "\nEverything looks good!\n";
							else
								std::cout << "\nHmm something doesn't look right ...\n";*/

							std::cout << "\nOrginal points\n" << B << "\nMy result\n" << B2 << '\n';

							// Used in Korejan's algo for checking results
							/*Eigen::Matrix<float, 3, 1> xht;
							xht << 0, 0, 3;
							Eigen::Matrix<float, 3, 1> xht2 = (ret_R * xht).colwise() + ret_t;*/

							KinectSettings::calibration_rotation = ret_R;
							KinectSettings::calibration_translation = ret_t;

							settings.rcR_matT = ret_R;
							settings.rcT_matT = ret_t;

							/*
							 * A little explaination what's going on in applying Korejan's transforms...
							 *
							 * So then, what is colwise and what does it do?
							 * https://eigen.tuxfamily.org/dox/group__TutorialReductionsVisitorsBroadcasting -> Broadcasting
							 * Since points are columns and poses are in rows,
							 * we need to "rotate" it to add poses to X,Y,Z
							 * and not X,X,X... ERROR WRONG TYPE
							 *
							 * Lower, transpose is just ^-1 or inverse() in glm.
							 * Anyway, if you don't get something, please read eigen's docs,
							 * and if you still don't, repeat it until you do...
							 */

							/* NEW Try to "compute" lookAt kinect yaw ourselves */

							// Assuming that if a point is at 1,0,3,
							// in relative to the tracking device, then
							// the tracking device should be at -(1,0,3)
							// which would be -1,0,-3, I guess...?

							// Calibration origin is 0 this time,
							// soooo the kinect's pose is just retrived trans

							// Then, what is kinpitch or kinect_pitch?
							// Assuming the kinect is looking at us from the up,
							// it's a bit tilted towards the floor.
							// When it looks up for a human body,
							// you're going to be shorter and tilted too...
							// I'm not fixing being shorter but tilting is going to be fixed
							// inside the calibration itself.
							// The next thing is rotation, trackers are going to break
							// if they're upside-down, sice we're using bad old
							// euler angles and not quats (for some things at least)
							// then we're gonna just flip them a bit...
							// Or actually offset them by a certain angle.

							//Eigen::Vector3f KinectPoseInSVR = ret_t;
							//
							//// calculate direction vectors
							//Eigen::Vector3f up(0, 1, 0),
							//	KinectDirectionVector(
							//		KinectPoseInSVR.x(),
							//		KinectPoseInSVR.y(),
							//		KinectPoseInSVR.z());

							//// Here's the SVR's home position
							//Eigen::Vector3f SVRHomePos(
							//	KinectSettings::trackingOriginPosition.v[0],
							//	KinectSettings::trackingOriginPosition.v[1] - 1.1f, // At knees level, assuming you're about 1.8m
							//	KinectSettings::trackingOriginPosition.v[2]);

							//
							//// Calculate the quaternion
							//auto // Matrix https://stackoverflow.com/questions/21761909/eigen-convert-matrix3d-rotation-to-quaternion
							//	KinectDirectionRotMat(EigenUtils::lookAt(KinectDirectionVector, SVRHomePos, up));


							// calculate direction vectors
							Eigen::Vector3f up(0, 1, 0),
							                KinectDirectionVector(
								                ret_t.x(),
								                0.0,
								                ret_t.z());

							// Here's the SVR's home position
							Eigen::Vector3f SVRHomePos(
								KinectSettings::trackingOriginPosition.v[0],
								0.0,
								KinectSettings::trackingOriginPosition.v[2]);

							// Calculate the quaternion
							auto
								// Matrix https://stackoverflow.com/questions/21761909/eigen-convert-matrix3d-rotation-to-quaternion
								KinectDirectionRotMat(EigenUtils::lookAt(KinectDirectionVector, SVRHomePos, up));


							// https://stackoverflow.com/questions/60758298/eigenmatrixdouble-4-4-to-eigenquaterniond
							Eigen::Vector3f
								KinectDirectionEigenEuler =
									KinectDirectionRotMat.topLeftCorner<3, 3>().eulerAngles(0, 1, 2);

							/*
							 *
							 * OKAY IT KINDA WORKS
							 * although, it seems like: [YAW, DEG]
							 *      true       eigen
							 *		  0			 0 ///
							 *		  90		 90
							 *		  180		 180
							 *		  181		 -180
							 *		  270		 -90
							 *		  359		 -1
							 *		  360		 0 ///
							 *
							 */

							Eigen::Vector3f
								KinectDirectionEigenEulerDegrees = KinectDirectionEigenEuler * 180 / M_PI;

							LOG(INFO) << "GOT ARTIFICIAL LOOKATKINECT ORIENTATION [DEG]: ";
							LOG(INFO) << KinectDirectionEigenEulerDegrees.x();
							LOG(INFO) << KinectDirectionEigenEulerDegrees.y();
							LOG(INFO) << KinectDirectionEigenEulerDegrees.z();

							///////// Make it 0-360

							float RetrievedYaw = KinectDirectionEigenEulerDegrees.y();

							//if (RetrievedYaw < 180.f && RetrievedYaw > 0.f)
							//	RetrievedYaw = abs(RetrievedYaw - 180.f);  // hack, although kinda working

							if (RetrievedYaw < 0.f && RetrievedYaw > -180.f)
								RetrievedYaw = abs(RetrievedYaw + 180.f) + 180.f;

							// abs() part will add PI radians and make it 0 - 180,
							// additional PI radians will make it 180-360
							// abs() because we're adding floats and idk how to make it safer
							// it could have been std::clamp, actually...

							///////// Make it 0-360

							LOG(INFO) << "GOT FIXED ARTIFICIAL LOOKATKINECT ORIENTATION [DEG]: ";
							LOG(INFO) << KinectDirectionEigenEulerDegrees.x();
							LOG(INFO) << RetrievedYaw;
							LOG(INFO) << KinectDirectionEigenEulerDegrees.z();

							// Save it to settings

							// Save our retrieved yaw (this one's in degrees)
							KinectSettings::calibration_trackers_yaw = RetrievedYaw; // Use fixed one
							settings.tryawst = KinectSettings::calibration_trackers_yaw;

							// Pitch may require some tweaks, before it was about 180deg
							KinectSettings::calibration_kinect_pitch =
								glm::radians(KinectDirectionEigenEulerDegrees.x()); // We're using radsians
							settings.kinpitchst = KinectSettings::calibration_kinect_pitch;

							// In maualcalib it's hips position, although here it's gonna be 0
							KinectSettings::calibration_origin = Eigen::Vector3f(0, 0, 0);
							settings.caliborigin = KinectSettings::calibration_origin;

							/* NEW Try to "compute" lookAt kinect yaw ourselves */
						}

						// Grab the yaw (now it's calculated with a lookAt)
						/*TrackersCalibButton->SetLabel(
							std::string("Prepare to calibration: Tracker Orientation").c_str());
						std::this_thread::sleep_for(std::chrono::seconds(1));
						for (auto i = 3; i >= 0; i--)
						{
							if (!KinectSettings::isCalibrating) break;
							TrackersCalibButton->SetLabel(
								std::string(
									"Tracker Orientation: Look at Kinect... Time left: " + boost::lexical_cast<
										std::string>(i) + "s").c_str());
							std::this_thread::sleep_for(std::chrono::seconds(1));
						}*/

						// Just the last thing to go back if we're somehow not sure
						if (!KinectSettings::isCalibrating)
						{
							KinectSettings::calibration_origin = settings.caliborigin;
							KinectSettings::calibration_rotation = settings.rcR_matT;
							KinectSettings::calibration_translation = settings.rcT_matT;
							KinectSettings::calibration_trackers_yaw = settings.tryawst;

							KinectSettings::calibration_kinect_pitch = settings.kinpitchst;
						}

						/**********************************************************************************************/

						//// Grab the yaw (now it's calculated with a lookAt)
						//if (KinectSettings::isCalibrating)
						//{
						//	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
						//		vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
						//	quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
						//	double yaw = std::atan2(trackedDevicePose.mDeviceToAbsoluteTracking.m[0][2],
						//	                        trackedDevicePose.mDeviceToAbsoluteTracking.m[2][2]),
						//	       yawRaw = std::atan2(trackedDevicePose.mDeviceToAbsoluteTracking.m[0][2],
						//	                           trackedDevicePose.mDeviceToAbsoluteTracking.m[2][2]);

						//	if (yawRaw < 0.0f)
						//	{
						//		yawRaw += 2 * M_PI;
						//	}
						//	if (yaw < 0.0)
						//	{
						//		yaw = 2 * M_PI + yaw;
						//	}

						//	/*KinectSettings::calibration_trackers_yaw = glm::degrees(yaw);
						//	settings.tryawst = glm::degrees(yaw);*/

						//	LOG(INFO) << "GOT REAL LOOKATKINECT ORIENTATION YAW: " << glm::degrees(yaw);
						//}

						/**********************************************************************************************/

						KinectSettings::matrixes_calibrated = true;
						settings.rtcalib = true;

						TrackersCalibButton->SetLabel(
							std::string(!KinectSettings::isCalibrating
								            ? "Calibration aborted! Hit me 2 times to re-calibrate!"
								            : "Done! Hit me 2 times to re-calibrate!").c_str());
						TrackersCalibButton->SetState(sfg::Widget::State::NORMAL);

						saveSettings();
					});
				}
			}
			else
			{
				KinectSettings::isCalibrating = false;
			}

			saveSettings();
		});
	}

private:
	sf::Font mainGUIFont;
	sfg::SFGUI sfguiRef;
	sfg::Window::Ptr guiWindow = sfg::Window::Create();
	sfg::Notebook::Ptr mainNotebook = sfg::Notebook::Create();
	std::vector<std::unique_ptr<DeviceHandler>>* v_deviceHandlersRef;

	// All the device handlers
	PSMoveHandler psMoveHandler;

	HRESULT lastKinectStatus = E_FAIL;

	sfg::Desktop guiDesktop;

	sfg::Box::Ptr mainGUIBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
	sfg::Box::Ptr calibrationBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
	sfg::Box::Ptr advancedTrackerBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);

	sfg::Adjustment::Ptr fontSizeAdjustment = sfg::Adjustment::Create();
	sfg::Label::Ptr FontSizeScaleLabel = sfg::Label::Create("(WARNING, LAGS ON CHANGE) Font Size: ");
	sfg::SpinButton::Ptr FontSizeScale = sfg::SpinButton::Create(
		sfg::Adjustment::Create(SFMLsettings::globalFontSize, 5.f, 100.f, .5f));
	float lastFontSizeValue = SFMLsettings::globalFontSize;

	sfg::Box::Ptr modeTitleBox110 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);

	//Statuses
	sfg::Label::Ptr KinectStatusLabel = sfg::Label::Create();
	sfg::Label::Ptr SteamVRStatusLabel = sfg::Label::Create();

	sfg::Button::Ptr reconKinectButton = sfg::Button::Create("Reconnect Kinect");
	sfg::Button::Ptr refreshpsmovesbuton = sfg::Button::Create("Refresh");
	sfg::Button::Ptr refreshpsmovesbuton1 = sfg::Button::Create("Refresh");
	sfg::Button::Ptr refreshpsmovesbuton11 = sfg::Button::Create("Refresh");

	sfg::Button::Ptr TrackerLastInitButton = sfg::Button::Create(
		"**Please be in VR before hitting me!** Spawn same trackers as last session");

	sfg::Button::Ptr ShowSkeletonButton = sfg::CheckButton::Create("Show/Hide Skeleton Tracking");
	sfg::Button::Ptr AutoStartTrackers = sfg::Button::Create("Initialise trackers automatically");
	sfg::Box::Ptr horcombox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);

	//Zeroing
	sfg::Label::Ptr KinectRotLabel = sfg::Label::Create(
		"Calibrate the rotation of the Kinect sensor with the controller thumbsticks. Press the trigger to confirm.");
	sfg::CheckButton::Ptr KinectRotButton = sfg::CheckButton::Create("Enable Kinect Rotation Calibration");

	//Position Adjust
	sfg::Label::Ptr KinectPosLabel = sfg::Label::Create(
		"Calibrate the position of the Kinect sensor with the controller thumbsticks. Press the trigger to confirm.");
	sfg::CheckButton::Ptr KinectPosButton = sfg::CheckButton::Create("Enable Kinect Position Calibration");

	// Controllers
	sfg::Label::Ptr ReconControllersLabel = sfg::Label::Create(
		"If controller input isn't working, press this to reconnect them.\n Make sure both are on, and not in standby.");
	sfg::Button::Ptr ReconControllersButton = sfg::Button::Create("Reconnect VR Controllers");

	sfg::Label::Ptr CalibrationSettingsLabel = sfg::Label::Create(
		"This tab allows you to (manually) offset every one of your trackers. It will aeffect real calibration values, but it will not change them directly.\nYou may need it for example when your right foot will be upper than left, etc.\nRotation is in degrees and position is declared in meters.");
	sfg::Label::Ptr CalibrationrPosLabel = sfg::Label::Create("Right Foot Position x, y, z");
	sfg::SpinButton::Ptr CalibrationEntryrPosX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[0][0].v[0], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryrPosY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[0][0].v[1], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryrPosZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[0][0].v[2], -1000.f, 1000.f, .01f, .2f));

	sfg::Label::Ptr CalibrationrRotLabel = sfg::Label::Create("Right Foot Rotation x, y, z");
	sfg::SpinButton::Ptr CalibrationEntryrRotX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[1][0].v[0], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryrRotY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[1][0].v[1], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryrRotZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[1][0].v[2], -360.f, 360.f, .01f, .2f));

	sfg::Label::Ptr CalibrationlPosLabel = sfg::Label::Create("Left Foot Position x, y, z ");
	sfg::SpinButton::Ptr CalibrationEntrylPosX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[0][1].v[0], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntrylPosY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[0][1].v[1], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntrylPosZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[0][1].v[2], -1000.f, 1000.f, .01f, .2f));

	sfg::Label::Ptr CalibrationlRotLabel = sfg::Label::Create("Left Foot Rotation x, y, z ");
	sfg::SpinButton::Ptr CalibrationEntrylRotX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[1][1].v[0], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntrylRotY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[1][1].v[1], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntrylRotZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[1][1].v[2], -360.f, 360.f, .01f, .2f));

	sfg::Label::Ptr CalibrationhPosLabel = sfg::Label::Create("Hips Position x, y, z        ");
	sfg::SpinButton::Ptr CalibrationEntryhPosX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[0][2].v[0], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryhPosY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[0][2].v[1], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryhPosZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[0][2].v[2], -1000.f, 1000.f, .01f, .2f));

	sfg::Label::Ptr CalibrationhRotLabel = sfg::Label::Create("Hips Rotation x, y, z        ");
	sfg::SpinButton::Ptr CalibrationEntryhRotX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[1][2].v[0], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryhRotY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[1][2].v[1], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryhRotZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::manual_offsets[1][2].v[2], -360.f, 360.f, .01f, .2f));

	sfg::Button::Ptr CalibrationSaveButton = sfg::Button::Create("Save Calibration Values");
	sfg::Button::Ptr ActivateVRSceneTypeButton = sfg::Button::Create("Show K2VR in the VR Bindings Menu!");

	//Adv Trackers
	sfg::Button::Ptr calibrateOffsetButton = sfg::Button::Create("Calibrate VR Offset");
	sfg::Button::Ptr AddHandControllersToList = sfg::Button::Create("Add Hand Controllers");
	sfg::Button::Ptr AddLowerTrackersToList = sfg::Button::Create("Add Lower Body Trackers");
	sfg::Label::Ptr space_label = sfg::Label::Create(" ");

	bool userSelectedDeviceRotIndex = false;
	bool userSelectedDevicePosIndex = false;
	sfg::Box::Ptr TrackerList = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5);
	sfg::Label::Ptr TrackerListLabel = sfg::Label::Create("Trackers to be spawned:");

	sfg::Box::Ptr TrackerListOptionsBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5);
	sfg::SpinButton::Ptr HipScale = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::hipRoleHeightAdjust, -1.f, 1.f, .01f));
	sfg::Box::Ptr HipScaleBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);

	bool kinectJointDevicesHiddenFromList = true;
	sfg::CheckButton::Ptr showJointDevicesButton = sfg::CheckButton::Create("Show joints in devices");
	sfg::Button::Ptr refreshDeviceListButton = sfg::Button::Create("Refresh Devices");
	sfg::ComboBox::Ptr BonesList = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr PositionDeviceList = sfg::ComboBox::Create();
	sfg::CheckButton::Ptr identifyPosDeviceButton = sfg::CheckButton::Create("Boop");
	sfg::ComboBox::Ptr RotationDeviceList = sfg::ComboBox::Create();
	sfg::CheckButton::Ptr identifyRotDeviceButton = sfg::CheckButton::Create("Beep");
	sfg::ComboBox::Ptr RolesList = sfg::ComboBox::Create();
	sfg::CheckButton::Ptr IsControllerButton = sfg::CheckButton::Create("Controller");
	sfg::Button::Ptr AddTrackerToListButton = sfg::Button::Create("Add");
	sfg::Button::Ptr RemoveTrackerFromListButton = sfg::Button::Create("Remove");

	//Tracking Method Box
	sfg::Button::Ptr InitiateColorTrackingButton = sfg::Button::Create("Start Color Tracker");
	sfg::Button::Ptr DestroyColorTrackingButton = sfg::Button::Create("Destroy Color Tracker");
	sfg::Label::Ptr TrackingMethodLabel = sfg::Label::Create(
		"Click the corresponding button for the devices you wish to use, and K2VR will try its best to connect to them. (Go to the 'Adv. Trackers' tab once these are connected.");

	sfg::Button::Ptr StartPSMoveHandler = sfg::Button::Create("Run PS Move Handler");
	sfg::Button::Ptr StopPSMoveHandler = sfg::Button::Create("Stop PS Move Handler");
	sfg::Label::Ptr PSMoveHandlerLabel = sfg::Label::Create("Status: Off");

	// Virtual Hips Box
	sfg::CheckButton::Ptr VirtualHipUseHMDYawButton = sfg::CheckButton::Create("Yaw");
	sfg::CheckButton::Ptr VirtualHipUseHMDPitchButton = sfg::CheckButton::Create("Pitch");
	sfg::CheckButton::Ptr VirtualHipUseHMDRollButton = sfg::CheckButton::Create("Roll");

	sfg::RadioButton::Ptr VirtualHipLockToHeadButton = sfg::RadioButton::Create("Head");
	sfg::RadioButton::Ptr VirtualHipLockToFeetButton = sfg::RadioButton::Create("Feet");

	sfg::SpinButton::Ptr VirtualHipHeightFromHMDButton = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, -1000.f, 1000.f, 0.01f));
	sfg::CheckButton::Ptr VirtualHipFollowHMDLean = sfg::CheckButton::Create("Follow HMD Lean");

	sfg::SpinButton::Ptr DegreeButton = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, -360.f, 360.f, 0.01f));
	sfg::SpinButton::Ptr TDegreeButton = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, 2, 11, 1.f));

	sfg::Button::Ptr VirtualHipConfigSaveButton = sfg::Button::Create("Save Settings");
	sfg::Button::Ptr TrackersConfigSaveButton = sfg::Button::Create("Save Settings");

	sfg::Button::Ptr HeadTrackingStartButton = sfg::Button::Create("Start Head Tracking");
	sfg::Button::Ptr HeadTrackingCalibButton = sfg::Button::Create("Calibration: Look at Kinect and stand still");
	sfg::Button::Ptr TrackersCalibButton = sfg::Button::Create("Begin Calibration");
	sfg::CheckButton::Ptr expcalibbutton = sfg::CheckButton::Create("Enable Manual Calibration");

	void updateKinectStatusLabelDisconnected()
	{
		KinectStatusLabel->SetText("Kinect Status: ERROR KINECT NOT DETECTED");
	}

	void showPostTrackerInitUI(bool show = true)
	{
		KinectRotLabel->Show(show);
		KinectRotButton->Show(show);
		KinectPosLabel->Show(show);
		KinectPosButton->Show(show);
		ReconControllersLabel->Show(show);
		ReconControllersButton->Show(show);
		HipScale->Show(show);
		HipScaleBox->Show(show);
		calibrateOffsetButton->Show(show);

		//calibrationBox->Show(show);
	}

	void hidePostTrackerInitUI()
	{
		showPostTrackerInitUI(false);
	}
};
