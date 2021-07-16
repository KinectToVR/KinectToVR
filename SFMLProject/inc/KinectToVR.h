#pragma once
#include "stdafx.h"
//OpenGl and SFML
#include <SFML/Window/Event.hpp>
#include "KinectHandlerBase.h"
//VR
#include <openvr.h>

//void updateKinectTracker(vrinputemulator::VRInputEmulator &emulator, KinectTrackedDevice device);
void processKeyEvents(sf::Event event);
void toggle(bool& b);


void limitVRFramerate(double& endTimeMilliseconds, std::stringstream& ss);

void processLoop(KinectHandlerBase& kinect);

void updateFilePath();

void spawnDefaultLowerBodyTrackers();
