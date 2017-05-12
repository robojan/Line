﻿#define _CRT_SECURE_NO_WARNINGS
#include "OptionsManager.h"
#include <stdexcept>
#include <cstring>
#include <cstdio>

OptionsManager::OptionsManager(int argc, char** argv) :
	_debug(false), _name(argv[0]), _resolution(640,480), _tracking(false), _horizon(0.5f)
{
	for(auto i = 1; i < argc; i++)
	{
		if(argv[i][0] == '-')
		{
			if (strcmp(argv[i], "-h") == 0) {
				printf(GetUsage().c_str());
				throw std::runtime_error("Showing Help");
			}
			else if (strcmp(argv[i], "-d") == 0)
				_debug = true;
			else if (strcmp(argv[i], "-t") == 0)
				_tracking = true;
			else if (strcmp(argv[i], "-c") == 0)
			{
				if(argc - 1 == i || argv[i+1][0] == '-')
				{
					throw std::runtime_error("Capture device must be specified after -c option");
				}
				i++;
				_captureDevice = argv[i];
			}
			else if (strcmp(argv[i], "-s") == 0)
			{
				if (argc - 1 == i || argv[i + 1][0] == '-')
				{
					throw std::runtime_error("Capture device must be specified after -c option");
				}
				i++;
				char *p;
				float horizon = strtof(argv[i], &p);
				if (*p || horizon < 0 || horizon > 1)
				{
					throw std::runtime_error("The horizon hight in the range 0-1 must be given after the -s option");
				}
				_horizon = horizon;
			}
			else if (strcmp(argv[i], "-r") == 0)
			{
				if (argc - 1 == i || argv[i + 1][0] == '-')
				{
					throw std::runtime_error("Resolution must be specified after -r option");
				}
				i++;
				int width, height;
				if (sscanf(argv[i], " %dx%d", &width, &height) != 2)
				{
					throw std::runtime_error("Valid resolution must be specified after -r option");
				}
				_resolution.width = width > 0 ? width : 640;
				_resolution.height = height > 0 ? height : 480;
			}
		} else
		{
			_parameters.push_back(std::string(argv[i]));
		}
	}
}

OptionsManager::~OptionsManager()
{
}

bool OptionsManager::IsDebugMode() const
{
	return _debug;
}

const std::string& OptionsManager::GetName() const
{
	return _name;
}

const std::vector<std::string>& OptionsManager::GetParameters() const
{
	return _parameters;
}

const std::string& OptionsManager::GetCaptureDevice() const
{
	return _captureDevice;
}

const cv::Size& OptionsManager::GetProcessingResolution() const
{
	return _resolution;
}

bool OptionsManager::IsTracking() const
{
	return _tracking;
}

float OptionsManager::GetHorizon() const
{
	return _horizon;
}

std::string OptionsManager::GetUsage()
{
	return "-h\t\tShow this help\n"
		"-d\t\tEnter debugging mode\n"
		"-c dev\tUse capture device\n"
		"-r widthxheight\tSet the resolution of the capture device\n"
		"-s horizon\tSet the horizon in the range of 0 - 1\n";
}
