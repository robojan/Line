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

	void ReadConfigFile(const char* path);
	void WriteConfigFile(const char* path);

	bool IsDebugMode() const;
	const std::string &GetName() const;
	const std::vector<std::string> &GetParameters() const;
	const std::string &GetCaptureDevice() const;
	const cv::Size &GetProcessingResolution() const;
	bool IsTracking() const;
	float GetHorizon() const;
	bool IsAccelerated() const;
	const std::string &GetControlDevice() const;
	int GetBaudRate() const;
	const cv::Mat &GetCameraCorrectionMatrix() const;
	const cv::Mat &GetCameraCorrectionDist() const;
	const cv::Mat &GetCameraCorrectionTSR() const;
	const cv::Mat &GetIPTMatrix() const;
	const cv::Size &GetMapSize() const;
	float GetMapTileSize() const;


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
	cv::Mat _cameraCorrMat;
	cv::Mat _cameraCorrDist;
	cv::Mat _cameraCorrTSR;
	cv::Mat _iptMat;
	int _baudRate;
	cv::Size _mapSize;
	float _mapTileSize;
};
