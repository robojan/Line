#define _CRT_SECURE_NO_WARNINGS
#include "OptionsManager.h"
#include <stdexcept>
#include <cstring>
#include <cstdio>

OptionsManager::OptionsManager(int argc, char** argv) :
	_debug(false), _name(argv[0]), _resolution(640, 480), _tracking(false), _horizon(0.5f),
	_acceleration(false), _cameraCorrMat(cv::Mat::eye(3, 3, CV_32F)),
	_cameraCorrDist(cv::Mat::zeros(5, 1, CV_32F)), _iptMat(cv::Mat::eye(3, 3, CV_32F)),
	_baudRate(115200), _mapSize(200, 200), _mapTileSize(100), _cameraCorrTSR(cv::Mat::eye(3, 3, CV_32F)),
	_skylimit(0), _signAreaLimit(100, 5000), _signRatioLimit(0.75f, 1.5f)
{
	_thresholds[FeatureType::BlueSign] = ColorThreshold(cv::Scalar(0, 130, 92), cv::Scalar(255, 140, 105));
	_thresholds[FeatureType::RedSign] = ColorThreshold(cv::Scalar(0, 170, 138), cv::Scalar(255, 180, 155));
	_thresholds[FeatureType::YellowSign] = ColorThreshold(cv::Scalar(0, 145, 190), cv::Scalar(255, 160, 210));
	WriteConfigFile("Default_config.xml");
	for(auto i = 1; i < argc; i++)
	{
		if(argv[i][0] == '-')
		{
			if (strcmp(argv[i], "-h") == 0) {
				printf(GetUsage().c_str());
				throw std::runtime_error("Showing Help");
			}
			else if (strcmp(argv[i], "-a") == 0)
				_acceleration = true;
			else if (strcmp(argv[i], "-d") == 0)
				_debug = true;
			else if (strcmp(argv[i], "-t") == 0)
				_tracking = true;
			else if (strcmp(argv[i], "--serial") == 0)
			{
				if (argc - 1 == i || argv[i + 1][0] == '-')
				{
					throw std::runtime_error("Capture device must be specified after -c option");
				}
				i++;
				_controlDevice = argv[i];
			}
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
			else if(strcmp(argv[i], "-f") == 0)
			{
				if (argc - 1 == i || argv[i + 1][0] == '-')
				{
					throw std::runtime_error("Configuration file must be specified after -f option");
				}
				i++;
				ReadConfigFile(argv[i]);
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

void OptionsManager::ReadConfigFile(const char* path)
{
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if(!fs.isOpened())
	{
		throw std::runtime_error("Could not open configuration file.");
	}
	cv::String captureDev = _captureDevice;
	cv::read(fs["Input"], captureDev, captureDev);
	_captureDevice = captureDev;
	cv::read(fs["Resolution"], _resolution, _resolution);
	cv::read(fs["Horizon"], _horizon, _horizon);
	cv::read(fs["SkyLimit"], _skylimit, _skylimit);
	cv::read(fs["CameraCorrMat"], _cameraCorrMat, _cameraCorrMat);
	cv::read(fs["CameraCorrDist"], _cameraCorrDist, _cameraCorrDist);
	cv::read(fs["CameraCorrTSR"], _cameraCorrTSR, _cameraCorrTSR);
	cv::read(fs["IPT"], _iptMat, _iptMat);
	cv::read(fs["Tracking"], _tracking, _tracking);
	cv::read(fs["Acceleration"], _acceleration, _acceleration);
	cv::read(fs["Debug"], _debug, _debug);
	cv::String controlDevice = _controlDevice;
	cv::read(fs["ControlDevice"], controlDevice, controlDevice);
	_controlDevice = controlDevice;
	cv::read(fs["BaudRate"], _baudRate, _baudRate);
	cv::read(fs["MapSize"], _mapSize, _mapSize);
	cv::read(fs["MapTileSize"], _mapTileSize, _mapTileSize);
	cv::Scalar low, high;
	cv::read(fs["ThresholdBlueLow"], low, _thresholds.at(FeatureType::BlueSign).Low());
	cv::read(fs["ThresholdBlueHigh"], high, _thresholds.at(FeatureType::BlueSign).High());
	_thresholds[FeatureType::BlueSign] = ColorThreshold(low, high);
	cv::read(fs["ThresholdYellowLow"], low, _thresholds.at(FeatureType::YellowSign).Low());
	cv::read(fs["ThresholdYellowHigh"], high, _thresholds.at(FeatureType::YellowSign).High());
	_thresholds[FeatureType::YellowSign] = ColorThreshold(low, high);
	cv::read(fs["ThresholdRedLow"], low, _thresholds.at(FeatureType::RedSign).Low());
	cv::read(fs["ThresholdRedHigh"], high, _thresholds.at(FeatureType::RedSign).High());
	_thresholds[FeatureType::RedSign] = ColorThreshold(low, high);
	cv::read(fs["SignRatioLimit"], _signRatioLimit, _signRatioLimit);
	cv::read(fs["SignAreaLimit"], _signAreaLimit, _signAreaLimit);
	//cv::read(fs["Thresholds"], _thresholds, _thresholds);
}

void OptionsManager::WriteConfigFile(const char* path)
{
	cv::FileStorage fs(path, cv::FileStorage::WRITE);
	if (!fs.isOpened())
	{
		throw std::runtime_error("Could not open configuration file.");
	}
	fs << "Input" << _captureDevice;
	fs << "Resolution" << _resolution;
	fs << "Horizon" << _horizon;
	fs << "SkyLimit" << _skylimit;
	fs << "CameraCorrMat" << _cameraCorrMat;
	fs << "CameraCorrDist" << _cameraCorrDist;
	fs << "CameraCorrTSR" << _cameraCorrTSR;
	fs << "IPT" << _iptMat;
	fs << "Tracking" << false;
	fs << "Acceleration" << false;
	fs << "Debug" << false;
	fs << "ControlDevice" << _controlDevice;
	fs << "BaudRate" << _baudRate;
	fs << "MapSize" << _mapSize;
	fs << "MapTileSize" << _mapTileSize;
	auto it = _thresholds.find(FeatureType::BlueSign);
	if(it != _thresholds.end())
	{
		fs << "ThresholdBlueLow" << it->second.Low();
		fs << "ThresholdBlueHigh" << it->second.High();
	}
	it = _thresholds.find(FeatureType::YellowSign);
	if (it != _thresholds.end())
	{
		fs << "ThresholdYellowLow" << it->second.Low();
		fs << "ThresholdYellowHigh" << it->second.High();
	}
	it = _thresholds.find(FeatureType::RedSign);
	if (it != _thresholds.end())
	{
		fs << "ThresholdRedLow" << it->second.Low();
		fs << "ThresholdRedHigh" << it->second.High();
	}
	fs << "SignAreaLimit" << _signAreaLimit;
	fs << "SignRatioLimit" << _signRatioLimit;
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

float OptionsManager::GetSkyLimit() const
{
	return _skylimit;
}

bool OptionsManager::IsAccelerated() const
{
	return _acceleration;
}

const std::string& OptionsManager::GetControlDevice() const
{
	return _controlDevice;
}

int OptionsManager::GetBaudRate() const
{
	return _baudRate;
}

const cv::Mat& OptionsManager::GetCameraCorrectionMatrix() const
{
	return _cameraCorrMat;
}

const cv::Mat& OptionsManager::GetCameraCorrectionDist() const
{
	return _cameraCorrDist;
}

const cv::Mat & OptionsManager::GetCameraCorrectionTSR() const
{
	return _cameraCorrTSR;
}

const cv::Mat& OptionsManager::GetIPTMatrix() const
{
	return _iptMat;
}

const cv::Size& OptionsManager::GetMapSize() const
{
	return _mapSize;
}

float OptionsManager::GetMapTileSize() const
{
	return _mapTileSize;
}

const ColorThreshold& OptionsManager::GetColorThreshold(FeatureType type) const
{
	return _thresholds.at(type);
}

std::string OptionsManager::GetUsage()
{
	return "-h\t\tShow this help\n"
		"-d\t\tEnter debugging mode\n"
		"-c dev\tUse capture device\n"
		"-r widthxheight\tSet the resolution of the capture device\n"
		"-s horizon\tSet the horizon in the range of 0 - 1\n"
		"-f Read configuration from file\n";
}

const cv::Vec2f& OptionsManager::GetSignAreaLimit() const
{
	return _signAreaLimit;
}

const cv::Vec2f& OptionsManager::GetSignRatioLimit() const
{
	return _signRatioLimit;
}
