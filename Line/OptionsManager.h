#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>

/**
 * \brief 
 */
class OptionsManager
{
public:
	/**
	 * \brief Constructor of the optionsManager class
	 * \param argc The number of arguments
	 * \param argv The arguments, argv[0] is the application name
	 */
	OptionsManager(int argc, char **argv);
	/**
	 * \brief 
	 */
	~OptionsManager();

	bool IsDebugMode() const;
	const std::string &GetName() const;
	const std::vector<std::string> &GetParameters() const;
	const std::string &GetCaptureDevice() const;
	const cv::Size &GetProcessingResolution() const;
	bool IsTracking() const;
	float GetHorizon() const;
	bool IsAccelerated() const;
	const std::string &GetControlDevice() const;

	static std::string GetUsage();

private:
	bool _debug;
	std::string _captureDevice;
	const std::string _name;
	std::vector<std::string> _parameters;
	cv::Size _resolution;
	bool _tracking;
	float _horizon;
	bool _acceleration;
	std::string _controlDevice;
};
