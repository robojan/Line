#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

class ImgProcessor
{
public:
	ImgProcessor(cv::Size resolution);
	~ImgProcessor();

	void Process(cv::Mat &frame, cv::Mat &display);

	void PrintPerf();

	void SetHorizon(float horizon);
private:
	struct Performance {
		struct {
			int64 scale;
			int64 display;
			int64 bgr2lab;
			int64 labclone;
			int64 illumCorr;
			int64 lab2bgr;
			int64 split;
		} pre;
		struct {
			int64 split;
			int64 corr;
			int64 blur;
			int64 canny;
			int64 hough;
			int64 drawing;
		} line;
		struct {

		} sign;
	};

	std::string GetPerfString(const struct ImgProcessor::Performance &perf);
	void UpdatePerf();
	void ScaleFrame(cv::Mat &frame);
	void IlluminationCorrection(cv::Mat &in, cv::Mat &out);
	void HeavyBlur(cv::Mat &in, cv::Mat &out);
	void ProcessLines(cv::Mat &frame, cv::Mat &display);
	void ProcessSigns(cv::Mat &frame, cv::Mat &display);

	cv::Size _resolution;
	float _horizon;

	cv::Ptr<cv::CLAHE> _clahe;
	struct Performance _perf;
	struct Performance _avgPerf;
};
