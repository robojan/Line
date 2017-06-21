#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include "FeatureLibrary.h"
#include "GLAccelerator.h"

class ColorThreshold
{
public:
	ColorThreshold();
	ColorThreshold(const cv::Scalar &low, const cv::Scalar &high);

	void SetLow(const cv::Scalar &low);
	void SetHigh(const cv::Scalar &high);

	cv::Scalar Low() const;
	cv::Scalar High() const;
private:
	cv::Scalar _low;
	cv::Scalar _high;
};

typedef struct
{
	cv::Rect roi;
	FeatureType type;
	std::string sign;
	float area;
} track_data_t;

class ImgProcessor
{
public:

	ImgProcessor(cv::Size resolution, FeatureLibrary *featureLibrary, bool accelerated, cv::Size mapSize, float tileSize);
	~ImgProcessor();

	void Process(cv::Mat &frame, cv::Mat &display, cv::Point2f pos, float angle);

	void PrintPerf();

	void SetHorizon(float horizon);
	void SetSkyLimit(float skyLimit);
	void EnableDisplay(bool enabled);
	void SetThreshold(FeatureType type, ColorThreshold thresh);
	void SetSignRatioLimit(float low, float high);
	void SetSignAreaLimit(float low, float high);
	void SetTrackFrames(int frames);
	void SetCameraCorrection(const cv::Mat &matrix, const cv::Mat &dist, const cv::Mat &tsr);
	void SetIPT(const cv::Mat &matrix);
	const ColorThreshold &GetThreshold(FeatureType type);

	void ResetSignCounter();
	void GetSignProbabilities(std::map<std::string, float> &out);
private:
	struct Performance {
		struct {
			int64 scale;
			int64 cam;
			int64 display;
			int64 bgr2lab;
			int64 split;
		} pre;
		struct {
			int64 blur;
			int64 hist;
			int64 mask;
			int64 canny;
			int64 hough;
			int64 drawing;
		} line;
		struct {
			int64 thresh;
			int64 erosion;
			int64 dilation;
			int64 contour;
			int64 detect;
			struct {
				int64 illumCorrection;
				int64 canny;
				int64 match;
			} detectPart;
			int64 tracking;
		} sign;
	};

	std::string GetPerfString(const struct ImgProcessor::Performance &perf);
	void UpdatePerf();
	void ScaleFrame(cv::Mat& in, cv::Mat &out);
	void ProcessLines(cv::Mat &frame, cv::Mat &display, cv::Point2f pos, float angle);
	void ProcessSigns(cv::Mat &frame, int frameGLTex, cv::Mat &display);
	void ProcessSignContour(cv::Mat& frame, cv::Mat& display,
		const std::vector<std::vector<cv::Point>>& contours,
		const std::vector<cv::Vec4i>& hierarchy,
		FeatureType type,
		std::vector<track_data_t>& detected);
	void StartTracking(cv::Mat &frame, std::vector<track_data_t> &detected);
	void ProcessTracking(cv::Mat &frame, std::vector<track_data_t> &detected, cv::Mat& display);
	void UpdateSignCounter();
	void UpdateMap(const std::vector<cv::Vec4f> &lines, const std::vector<cv::Point2f>& visible, const cv::Point2f &pos, float angle);
	void CalculateCameraCorrection();

	cv::Size _resolution;
	float _horizon;
	float _skylimit;
	bool _displayEnabled;
	cv::Mat _cameraCorrectionMat;
	cv::Mat _cameraCorrectionDist;
	cv::Mat _cameraCorrectionTSR;
	cv::Mat _cameraMap1;
	cv::Mat _cameraMap2;
	cv::Mat _iptMat;
	std::map<FeatureType, ColorThreshold> _thresholds;
	struct
	{
		float low;
		float high;
	} _signRatio;
	struct
	{
		float low;
		float high;
	}_signArea;
	int _trackingFrames;
	int _trackedFrames;

	FeatureLibrary *_features;
	cv::Ptr<GLAccelerator> _accelerator;

	cv::Ptr<cv::CLAHE> _clahe;
	std::vector<cv::Ptr<cv::Tracker>> _trackers;
	std::vector<cv::Rect2d> _trackRegion;
	std::vector<track_data_t> _detectedSigns;
	cv::Mat _map;
	float _mapTileSize;

	std::map<std::string, float> _signsDetectedCounter;
	float _totalDetectedCounter;

	struct Performance _perf;
	struct Performance _avgPerf;
};
