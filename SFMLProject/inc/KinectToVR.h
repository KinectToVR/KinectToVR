#pragma once
#include "stdafx.h"
//OpenGl and SFML
#include <SFML/Window/Event.hpp>
#include "KinectHandlerBase.h"

//VR
#include <openvr.h>

//cURL
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

// Version number
const int
k2vr_version_major[3] = { 0, 9, 0 },
k2vr_version_minor[4] = { 51, 5, 35, 0 };

void processKeyEvents(sf::Event event);
void toggle(bool& b);

void processLoop(KinectHandlerBase& kinect);

void updateFilePath();

// Property of KinectToVR.cpp
void spawnDefaultLowerBodyTrackers();