#include "ImgProcessor.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <vector>
#include <sstream>
#include <iostream>

using namespace cv;

ImgProcessor::ImgProcessor(cv::Size resolution) : 
	_resolution(resolution)
{
	_clahe = createCLAHE();
	_clahe->setClipLimit(2);
	memset(&_avgPerf, 0, sizeof(struct Performance));
	memset(&_perf, 0, sizeof(struct Performance));
}

ImgProcessor::~ImgProcessor()
{
}

void ImgProcessor::Process(cv::Mat& frame, cv::Mat& display)
{
	int64 t1, t2;
	t1 = getTickCount();
	ScaleFrame(frame);
	t2 = getTickCount();
	_perf.pre.scale = t2 - t1;
	display = frame.clone();
	t1 = getTickCount();
	_perf.pre.display = t1 - t2;
	
	Mat labFrame;
	cvtColor(frame, labFrame, CV_BGR2Lab);
	t2 = getTickCount();
	_perf.pre.bgr2lab = t2 - t1;

	Mat labFrameCpy = labFrame.clone();
	t1 = getTickCount();
	_perf.pre.labclone = t1 - t2;

	//IlluminationCorrection(labFrame, labFrame);
	t2 = getTickCount();
	_perf.pre.illumCorr = t2 - t1;

	Mat correctedFrame;
	//cvtColor(labFrame, correctedFrame, CV_Lab2BGR);
	t1 = getTickCount();
	_perf.pre.lab2bgr = t1 - t2;

	int horizon = int(_resolution.height * _horizon);
	line(display, Point(0, horizon), Point(_resolution.width, horizon), Scalar(0, 255, 0), 2, LINE_8, 0);
	//Mat skyImg = correctedFrame(Range(0, horizon), Range::all());
	//Mat streetImg = correctedFrame(Range(horizon, _resolution.height), Range::all());
	Mat skyLabImg = labFrameCpy(Range(0, horizon), Range::all());
	Mat streetLabImg = labFrameCpy(Range(horizon, _resolution.height), Range::all());
	Mat skyDisplay = display(Range(0, horizon), Range::all());
	Mat streetDisplay = display(Range(horizon, _resolution.height), Range::all());

	t2 = getTickCount();
	_perf.pre.split = t2 - t1;

	ProcessLines(streetLabImg, streetDisplay);
	ProcessSigns(skyLabImg, skyDisplay);

	UpdatePerf();
}

void ImgProcessor::SetHorizon(float horizon)
{
	_horizon = horizon;
}

void ImgProcessor::ScaleFrame(cv::Mat& frame)
{
	if(frame.cols == _resolution.width && frame.rows == _resolution.height)
	{
		return;
	}
	resize(frame, frame, _resolution, 0, 0, INTER_LINEAR);
}

void ImgProcessor::IlluminationCorrection(cv::Mat& in, cv::Mat& out)
{
	Mat dst;

	std::vector<cv::Mat> planes(3);
	split(in, planes);
	_clahe->apply(planes[0], dst);
	dst.copyTo(planes[0]);
	merge(planes, out);
}

void ImgProcessor::HeavyBlur(cv::Mat& in, cv::Mat& out)
{
	blur(in, out, Size(4, 4));
}

void ImgProcessor::ProcessLines(cv::Mat& frame, cv::Mat& display)
{
	//Mat grayscale;
	//cvtColor(frame, grayscale, CV_BGR2GRAY);
	int64 t1, t2;
	t1 = getTickCount();
	std::vector<Mat> splitFrame;
	split(frame, splitFrame);
	t2 = getTickCount();
	_perf.line.split = t2 - t1;
	t1 = t2;
	Mat corr;
	_clahe->apply(splitFrame[0], corr);
	t2 = getTickCount();
	_perf.line.corr = t2 - t1;

	Mat blurred;
	HeavyBlur(corr, blurred);
	t1 = getTickCount();
	_perf.line.blur = t1 - t2;

	Mat edges;
	Canny(blurred, edges, 200, 250, 3, false);
	t2 = getTickCount();
	_perf.line.canny = t2 - t1;


	std::vector<Vec4i> lines;
	HoughLinesP(edges, lines, 1, CV_PI / 90, 30, 80, 20);
	//std::vector<Vec2f> lines;
	//HoughLines(edges, lines, 1, CV_PI / 180, 250, 0, 0, 0, CV_PI);
	t1 = getTickCount();
	_perf.line.hough = t1 - t2;

	for (auto detectedLine : lines) {
		line(display, Point(detectedLine[0], detectedLine[1]), Point(detectedLine[2], detectedLine[3]), Scalar(255, 0, 0), 3, LINE_8, 0);
	}
	t2 = getTickCount();
	_perf.line.drawing = t2 - t1;

	Mat perspective;
	//warpPerspective(blurred, perspective,)

	//imshow("Lum", corr);
	//imshow("Edges", edges);
}

void ImgProcessor::ProcessSigns(cv::Mat& frame, cv::Mat& display)
{
	Mat colorFrame;
	resize(frame, colorFrame, Size(64,32), 0, 0, INTER_CUBIC);
	//imshow("signColor", colorFrame);
}

float getTimeMs(int64 time)
{
	float freq = (float)getTickFrequency();
	return 1000 * time / freq;
}

void ImgProcessor::PrintPerf()
{	
	std::cout << GetPerfString(_avgPerf) << std::endl;
}

std::string ImgProcessor::GetPerfString(const ImgProcessor::Performance& perf)
{
	std::stringstream ss;

	int64 pre = perf.pre.scale + perf.pre.display + perf.pre.bgr2lab +
		perf.pre.labclone + perf.pre.illumCorr + perf.pre.lab2bgr +
		perf.pre.split;
	int64 line = perf.line.split + perf.line.blur + perf.line.canny +
		perf.line.hough + perf.line.drawing + perf.line.corr;

	int64 frame = pre + line;

	ss << "Performance:" << getTimeMs(frame) << "ms\n"
		"\tPreprocessing: " << getTimeMs(pre) << "ms\n"
		"\t\tscale:     " << getTimeMs(perf.pre.scale) << "ms\n"
		"\t\tdisplay:   " << getTimeMs(perf.pre.display) << "ms\n"
		"\t\tbgr2lab:   " << getTimeMs(perf.pre.bgr2lab) << "ms\n"
		"\t\tlabclone:  " << getTimeMs(perf.pre.labclone) << "ms\n"
		"\t\tillumCorr: " << getTimeMs(perf.pre.illumCorr) << "ms\n"
		"\t\tlab2bgr:   " << getTimeMs(perf.pre.lab2bgr) << "ms\n"
		"\t\tsplit:     " << getTimeMs(perf.pre.split) << "ms\n"
		"\tLine detection: " << getTimeMs(line) << "\n"
		"\t\tsplit:     " << getTimeMs(perf.line.split) << "ms\n"
		"\t\tcorr:      " << getTimeMs(perf.line.corr) << "ms\n"
		"\t\tblur:      " << getTimeMs(perf.line.blur) << "ms\n"
		"\t\tcanny:     " << getTimeMs(perf.line.canny) << "ms\n"
		"\t\though:     " << getTimeMs(perf.line.hough) << "ms\n"
		"\t\tdrawing:   " << getTimeMs(perf.line.drawing) << "ms\n";
	return ss.str();
}

void ImgProcessor::UpdatePerf()
{
	_avgPerf.pre.scale = (_avgPerf.pre.scale * 3 + _perf.pre.scale) / 4;
	_avgPerf.pre.display = (_avgPerf.pre.display * 3 + _perf.pre.display) / 4;
	_avgPerf.pre.bgr2lab = (_avgPerf.pre.bgr2lab * 3 + _perf.pre.bgr2lab) / 4;
	_avgPerf.pre.labclone = (_avgPerf.pre.labclone * 3 + _perf.pre.labclone) / 4;
	_avgPerf.pre.illumCorr = (_avgPerf.pre.illumCorr * 3 + _perf.pre.illumCorr) / 4;
	_avgPerf.pre.lab2bgr = (_avgPerf.pre.lab2bgr * 3 + _perf.pre.lab2bgr) / 4;
	_avgPerf.pre.split = (_avgPerf.pre.split * 3 + _perf.pre.split) / 4;

	_avgPerf.line.split = (_avgPerf.line.split * 3 + _perf.line.split) / 4;
	_avgPerf.line.corr = (_avgPerf.line.corr * 3 + _perf.line.corr) / 4;
	_avgPerf.line.blur = (_avgPerf.line.blur * 3 + _perf.line.blur) / 4;
	_avgPerf.line.canny = (_avgPerf.line.canny * 3 + _perf.line.canny) / 4;
	_avgPerf.line.hough = (_avgPerf.line.hough * 3 + _perf.line.hough) / 4;
	_avgPerf.line.drawing = (_avgPerf.line.drawing * 3 + _perf.line.drawing) / 4;
}