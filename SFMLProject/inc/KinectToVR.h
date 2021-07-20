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
k2vr_version_major[4] = { 0, 8, 1 },
k2vr_version_minor[4] = { 51, 4, 30, 99 };

void processKeyEvents(sf::Event event);
void toggle(bool& b);

void limitVRFramerate(double& endTimeMilliseconds, std::stringstream& ss);

void processLoop(KinectHandlerBase& kinect);

void updateFilePath();

void spawnDefaultLowerBodyTrackers();
