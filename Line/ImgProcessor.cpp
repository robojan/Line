#include "ImgProcessor.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <vector>
#include <sstream>
#include <iostream>
#include <complex.h>

using namespace cv;

ColorThreshold::ColorThreshold() :
	_low(0,0,0), _high(255,255,255)
{
}

ColorThreshold::ColorThreshold(const cv::Scalar& low, const cv::Scalar& high) :
	_low(low), _high(high)
{
}

void ColorThreshold::SetLow(const cv::Scalar& low)
{
	_low = low;
}

void ColorThreshold::SetHigh(const cv::Scalar& high)
{
	_high = high;
}

cv::Scalar ColorThreshold::Low() const
{
	return _low;
}

cv::Scalar ColorThreshold::High() const
{
	return _high;
}

ImgProcessor::ImgProcessor(cv::Size resolution, FeatureLibrary *featureLibrary, bool accelerated, Size mapSize, float tileSize) :
	_resolution(resolution), _displayEnabled(false), _features(featureLibrary),
	_trackingFrames(0), _trackedFrames(0), _horizon(0.5f), 
	_cameraCorrectionMat(Mat::eye(3, 3, CV_32F)), _cameraCorrectionDist(Mat::zeros(5, 1, CV_32F)),
	_iptMat(Mat::eye(3, 3, CV_32F)), _map(Mat::zeros(mapSize, CV_32F)), _mapTileSize(tileSize)
{
	_clahe = createCLAHE();
	_clahe->setClipLimit(2);
	memset(&_avgPerf, 0, sizeof(struct Performance));
	memset(&_perf, 0, sizeof(struct Performance));
	SetSignRatioLimit(0.5f, 2.0f);
	SetSignAreaLimit(1000, 10000);
	ResetSignCounter();
	
	std::vector<Point2f> imPoints;
	std::vector<Point2f> rePoints;
	imPoints.push_back(Point2f(52, 126-112)); rePoints.push_back(Point2f(-1000, 1500));
	imPoints.push_back(Point2f(100, 144-112)); rePoints.push_back(Point2f(-500, 1000));
	imPoints.push_back(Point2f(379, 175-112)); rePoints.push_back(Point2f(500, 500));
	imPoints.push_back(Point2f(274, 135-112)); rePoints.push_back(Point2f(500, 1500));
	imPoints.push_back(Point2f(305, 148-112)); rePoints.push_back(Point2f(500, 1000));

	Mat ipt = cv::findHomography(imPoints, rePoints);
	std::cout << ipt;

	if(accelerated)
	{
		try
		{
			_accelerator = new GLAccelerator("shaders/");
			_accelerator->CreateProgram("cc", _resolution);
			_accelerator->CreateProgram("thresh", Size(_resolution.width, int(_resolution.height * _horizon)));
		} catch(GLAcceleratorException &e)
		{
			std::cerr << e.what() << std::endl;
			throw e;
		}
	}
}

ImgProcessor::~ImgProcessor()
{
}

void ImgProcessor::Process(cv::Mat& frame, cv::Mat& display, cv::Point2f pos, float angle)
{
	int64 t1, t2;

	Mat camCorr;
	Mat labFrame;
	int labGLFrame = 0;
	if(!_accelerator.empty())
	{
		_perf.pre.scale = 0;
		// Clone the frame for output
		t1 = getTickCount();
		if (_displayEnabled)
		{
			ScaleFrame(frame, display);
		}
		t2 = getTickCount();
		_perf.pre.display = t2 - t1;

		t1 = t2;
		_accelerator->ProcessFrame("cc", frame);
		labGLFrame = _accelerator->GetResultTexture("cc");
		_accelerator->GetResult("cc", labFrame);
		t2 = getTickCount();
		_perf.pre.bgr2lab = t2 - t1;
	} else
	{
		// Scaling the image
		t1 = getTickCount();
		ScaleFrame(frame, frame);
		t2 = getTickCount();
		_perf.pre.scale = t2 - t1;

		// Camera correction
		t1 = t2;
		undistort(frame, camCorr, _cameraCorrectionMat, _cameraCorrectionDist);
		t2 = getTickCount();
		_perf.pre.cam = t2 - t1;

		// Clone the frame for output
		t1 = t2;
		if (_displayEnabled)
		{
			display = camCorr.clone();
		}
		t2 = getTickCount();
		_perf.pre.display = t2 - t1;

		// Convert the colors to the LAB color space
		t1 = t2;
		cvtColor(camCorr, labFrame, CV_BGR2Lab);
		t2 = getTickCount();
		_perf.pre.bgr2lab = t2 - t1;
	}

	// Split the frame
	t1 = t2;
	int horizon = int(_resolution.height * _horizon);
	Mat skyDisplay, streetDisplay;
	if(_displayEnabled)
	{
		line(display, Point(0, horizon), Point(_resolution.width, horizon), Scalar(0, 255, 0), 2, LINE_8, 0);
		skyDisplay = display(Range(0, horizon), Range::all());
		streetDisplay = display(Range(horizon, _resolution.height), Range::all());
	}
	Mat streetImg = labFrame(Range(horizon, _resolution.height), Range::all());
	Mat skyImg = labFrame(Range(0, horizon), Range::all());
	t2 = getTickCount();
	_perf.pre.split = t2 - t1;

	// Process the rest
	ProcessLines(streetImg, streetDisplay, pos, angle);
	if(_trackingFrames == _trackedFrames)
	{
		_detectedSigns.clear();
		ProcessSigns(skyImg, labGLFrame, skyDisplay);
		_trackedFrames = 0;
		if (_trackingFrames > 0) // only on first frame
		{
			StartTracking(skyImg, _detectedSigns);
		}
		UpdateSignCounter();
	} else
	{
		_perf.sign.thresh = 0;
		_perf.sign.erosion = 0;
		_perf.sign.dilation = 0;
		_perf.sign.contour = 0;
		_perf.sign.detect = 0;
		ProcessTracking(skyImg, _detectedSigns, skyDisplay);
		_trackedFrames++;
	}

	UpdatePerf();

	if(!_accelerator.empty())
	{
		_accelerator->Update();
	}
}

void ImgProcessor::SetHorizon(float horizon)
{
	_horizon = horizon;
}

void ImgProcessor::EnableDisplay(bool enabled)
{
	_displayEnabled = enabled;
}

const ColorThreshold &ImgProcessor::GetThreshold(FeatureType type) 
{
	auto thresh = _thresholds.find(type);
	if(thresh != _thresholds.end())
	{
		return thresh->second;
	}
	_thresholds[type] = ColorThreshold();
	return _thresholds[type];
}

void ImgProcessor::SetThreshold(FeatureType type, ColorThreshold thresh)
{
	_thresholds[type] = thresh;
}

void ImgProcessor::SetSignRatioLimit(float low, float high)
{
	_signRatio.low = low;
	_signRatio.high = high;
}

void ImgProcessor::SetSignAreaLimit(float low, float high)
{
	_signArea.low = low;
	_signArea.high = high;
}

void ImgProcessor::SetTrackFrames(int frames)
{
	_trackingFrames = frames;
}

void ImgProcessor::SetCameraCorrection(const cv::Mat& matrix, const cv::Mat& dist)
{
	_cameraCorrectionMat = matrix;
	_cameraCorrectionDist = dist;
}

void ImgProcessor::SetIPT(const cv::Mat& matrix)
{
	_iptMat = matrix;
}

void ImgProcessor::ScaleFrame(cv::Mat& in, cv::Mat &out)
{
	if(in.cols == _resolution.width && in.rows == _resolution.height)
	{
		if(out.data != in.data)
		{
			out = in.clone();
		}
		return;
	}
	resize(in, out, _resolution, 0, 0, INTER_LINEAR);
}

cv::Point2f operator*(cv::Mat M, const cv::Point2f& p)
{
	cv::Mat_<float> src(3/*rows*/, 1 /* cols */);

	src(0, 0) = p.x;
	src(1, 0) = p.y;
	src(2, 0) = 1.0;

	cv::Mat_<float> dst = M*src; //USE MATRIX ALGEBRA 
	return cv::Point2f(dst(1, 0), dst(0, 0));
}

void ImgProcessor::ProcessLines(cv::Mat& frame, cv::Mat& display, cv::Point2f pos, float angle)
{
	int64 t1, t2;
	
	// split
	Mat planes[3];
	Mat &LPlane = planes[0];
	split(frame, planes);

	// Blur
	t1 = getTickCount();
	Mat blurred;
	GaussianBlur(LPlane, blurred, Size(3, 3), 5, 5, BORDER_DEFAULT);
	t2 = getTickCount();
	_perf.line.blur = t2 - t1;

	// Background subtraction
	t1 = t2;
	Mat hist;
	int histSize = 8;
	float range[] = { 0, 256 };
	const float *histRange = { range };
	calcHist(&blurred, 1, NULL, Mat(), (OutputArray)hist, 1, &histSize, &histRange, true, false);
	int loBin, hiBin;
	float loCount, hiCount;
	loCount = hiCount = hist.at<float>();
	loBin = hiBin = 0;
	float binSize = range[1] / histSize;
	for(int i = 1; i < histSize; i++)
	{
		float val = hist.at<float>(i);
		if (val < loCount) {
			loCount = val;
			loBin = i;
		}
		if (val > hiCount) {
			hiCount = val;
			hiBin = i;
		}
	}
	
	if (hiBin == 0) hiBin++;
	int loVal = int(((hiBin - 1) * binSize) + 0.5f);
	int hiVal = int(((hiBin + 1) * binSize)  + 0.5f);
	t2 = getTickCount();
	_perf.line.hist = t2 - t1;

	// Masking
	t1 = t2;
	Mat mask;
	//inRange(blurred, loVal, hiVal, mask);
	threshold(blurred, mask, loVal, 255, THRESH_BINARY);
	
	dilate(mask, mask, Mat(), Point(-1, -1), 2);
	//mask = 255 - mask;
	t2 = getTickCount();
	_perf.line.mask = t2 - t1;


	// Canny
	t1 = t2;
	Mat edges;
	Canny(mask, edges, 200, 250, 3, false);
	t2 = getTickCount();
	_perf.line.canny = t2 - t1;

	// Hough
	t1 = t2;
	std::vector<Vec4i> lines;
	HoughLinesP(edges, lines, 1.5, CV_PI / 180, 30, 50, 35);
	//std::vector<Vec2f> lines;
	//HoughLines(edges, lines, 1.5, CV_PI / 180, 50, 0, 0, 0, CV_PI);
	t2 = getTickCount();
	_perf.line.hough = t2 - t1;

	/*
	std::vector<Vec2f> linesParameters(lines.size());
	float avgRho = 0, avgTheta = 0;
	int leftLine=-1, rightLine=-1;
	float leftDist = INFINITY; float rightDist = INFINITY;
	Point center(frame.cols / 2, frame.rows / 2);
	for(int i = 0; i < lines.size(); i++)
	{
		Vec4i &line = lines[i];
		Point pt1(line[0], line[1]);
		Point pt2(line[2], line[3]);
		float dx = line[2] - line[0];
		float dy = line[3] - line[1];
		float rho = abs(pt2.x*pt1.y - pt2.y*pt1.x) / norm(pt2 - pt1);
		float theta = -atan2(dx, dy);
		avgRho += rho;
		avgTheta += theta;
		float dist = abs(dy*center.x - dx*center.y + pt2.x*pt1.y - pt2.y * pt1.x) /
			sqrtf(dy*dy + dx*dx);
		if(pt1.x < center.x && pt2.x < center.x && dist < leftDist)
		{
			leftLine = i;
		}
		if (pt1.x > center.x && pt2.x > center.x && dist < rightDist)
		{
			rightLine = i;
		}
		
		float a = cosf(theta);
		float b = sinf(theta);
		double x0 = a * rho;
		double y0 = b * rho;
		Point dpA(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * a));
		Point dpB(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * a));
		cv::line(display, dpA, dpB, Scalar(255, 255, 0), 1, LINE_8, 0);
		linesParameters[i] = Vec2f(rho, theta);
	}
	avgRho /= lines.size();
	avgTheta /= lines.size();*/

	// Draw the lines
	t1 = t2;
	if(_displayEnabled)
	{
		for (int i = 0; i < lines.size(); i++)
		{
			Vec4i &detectedLine = lines[i];
			Scalar color = Scalar(255, 0, 0);
			/*if(leftLine == i || rightLine == i)
			{
				color = Scalar(0, 255, 0);
			} else
			{
				color = Scalar(255, 0, 0);
			}*/
			line(display, Point(detectedLine[0], detectedLine[1]), Point(detectedLine[2], detectedLine[3]), color, 2, LINE_8, 0);
			/*float theta = lines[i][1];
			float rho = lines[i][0];
			float a = cosf(theta);
			float b = sinf(theta);
			double x0 = a * rho;
			double y0 = b * rho;
			Point avgLineA(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * a));
			Point avgLineB(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * a));
			line(display, avgLineA, avgLineB, Scalar(255, 255, 0), 3, LINE_8, 0);*/
		}
		/*
		float a = cosf(avgTheta);
		float b = sinf(avgTheta);
		double x0 = a * avgRho;
		double y0 = b * avgRho;
		Point avgLineA(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * a));
		Point avgLineB(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * a));
		line(display, avgLineA, avgLineB, Scalar(255, 255, 0), 3, LINE_8, 0);*/
	}
	t2 = getTickCount();
	_perf.line.drawing = t2 - t1;

	//Mat topdown;
	//Mat scale = (Mat_<float>(3, 3) << 0.1, 0, 250, 0, -0.1, 1000, 0, 0, 1);
	Mat robotMat = (Mat_<float>(3, 3) << cos(angle), -sin(angle), pos.x, sin(angle), cos(angle), pos.y, 0, 0, 1);
	Mat ipt = robotMat * _iptMat;
	
	//warpPerspective(display, topdown, ipt, Size(700, 1000));
	std::vector<Vec4f> tdLines;
	for (int i = 0; i < lines.size(); i++)
	{
		Vec4i &detectedLine = lines[i];
		std::vector<Point2f> src(2);
		std::vector<Point2f> dst(2);
		src[0] = Point2f((float)detectedLine[0], (float)detectedLine[1]);
		src[1] = Point2f((float)detectedLine[2], (float)detectedLine[3]);
		perspectiveTransform(src, dst, ipt);
		Scalar color = Scalar(0, 0, 255);
		if (dst[0].x < -20000 || dst[0].x > 20000 || dst[0].y < -20000 || dst[0].y > 20000 ||
			dst[1].x < -20000 || dst[1].x > 20000 || dst[1].y < -20000 || dst[1].y > 20000)
		{
			continue;
		}
		tdLines.push_back(Vec4f(dst[0].x, dst[0].y, dst[1].x, dst[1].y));
		//line(topdown, dst[0], dst[1], color, 2, LINE_8, 0);
	}
	std::vector<cv::Point2f> visibleRect(4);
	visibleRect[0] = Point2f(0, 0);
	visibleRect[1] = Point2f(frame.cols -1, 0);
	visibleRect[2] = Point2f(frame.cols - 1, frame.rows - 1);
	visibleRect[3] = Point2f(0, frame.rows - 1);
	perspectiveTransform(visibleRect, visibleRect, ipt);

	UpdateMap(tdLines, visibleRect);

}

void ImgProcessor::ProcessSigns(cv::Mat& frame, int frameGLTex, cv::Mat& display)
{
	int64 t1, t2;
	
	// Thresholding
	t1 = getTickCount();
	Mat blueMask, redMask, yellowMask;
	const ColorThreshold &blueThresholds = GetThreshold(FeatureType::BlueSign);
	const ColorThreshold &redThresholds = GetThreshold(FeatureType::RedSign);
	const ColorThreshold &yellowThresholds = GetThreshold(FeatureType::YellowSign);
	if(_accelerator.empty())
	{
		inRange(frame, blueThresholds.Low(), blueThresholds.High(), blueMask);
		inRange(frame, redThresholds.Low(), redThresholds.High(), redMask);
		inRange(frame, yellowThresholds.Low(), yellowThresholds.High(), yellowMask);
	} else
	{
		_accelerator->SetProgramUniformColor("thresh", "lowBlue", blueThresholds.Low() / 255);
		_accelerator->SetProgramUniformColor("thresh", "highBlue", blueThresholds.High() / 255);
		_accelerator->SetProgramUniformColor("thresh", "lowRed", redThresholds.Low() / 255);
		_accelerator->SetProgramUniformColor("thresh", "highRed", redThresholds.High() / 255);
		_accelerator->SetProgramUniformColor("thresh", "lowYellow", yellowThresholds.Low() / 255);
		_accelerator->SetProgramUniformColor("thresh", "highYellow", yellowThresholds.High() / 255);
		_accelerator->SetProgramUniform("thresh", "horizon", _horizon);
		_accelerator->ProcessFrame("thresh", frameGLTex);
		_accelerator->ProcessFrame("thresh", frame);
		Mat mask;
		_accelerator->GetResult("thresh", mask);
		Mat maskPlanes[3];
		split(mask, maskPlanes);
		blueMask = maskPlanes[0];
		redMask = maskPlanes[1];
		yellowMask = maskPlanes[2];

	}
	t2 = getTickCount();
	_perf.sign.thresh = t2 - t1;

	// Erosion
	t1 = t2;
	erode(blueMask, blueMask, Mat(), Point(-1, -1), 1);
	erode(redMask, redMask, Mat(), Point(-1, -1), 1);
	erode(yellowMask, yellowMask, Mat(), Point(-1, -1), 1);
	t2 = getTickCount();
	_perf.sign.erosion = t2 - t1;

	// Dilation
	t1 = t2;
	dilate(blueMask, blueMask, Mat(), Point(-1, -1), 3);
	dilate(redMask, redMask, Mat(), Point(-1, -1), 6);
	dilate(yellowMask, yellowMask, Mat(), Point(-1, -1), 3);
	t2 = getTickCount();
	_perf.sign.dilation = t2 - t1;

	// Contours
	t1 = t2;
	std::vector<std::vector<Point>> blueContours, redContours, yellowContours;
	std::vector<Vec4i> blueHierarchy, redHierarchy, yellowHierarchy;
	findContours(blueMask, blueContours, blueHierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	findContours(redMask, redContours, redHierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	findContours(yellowMask, yellowContours, yellowHierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	t2 = getTickCount();
	_perf.sign.contour = t2 - t1;
	
	// Detect signs
	t1 = t2;
	_detectedSigns.clear();
	ProcessSignContour(frame, display, blueContours, blueHierarchy, FeatureType::BlueSign, _detectedSigns);
	ProcessSignContour(frame, display, redContours, redHierarchy, FeatureType::RedSign, _detectedSigns);
	ProcessSignContour(frame, display, yellowContours, yellowHierarchy, FeatureType::YellowSign, _detectedSigns);
	t2 = getTickCount();
	_perf.sign.detect = t2 - t1;
}

void ImgProcessor::ProcessSignContour(cv::Mat& frame, cv::Mat& display, 
	const std::vector<std::vector<cv::Point>>& contours, 
	const std::vector<cv::Vec4i>& hierarchy, 
	FeatureType type, 
	std::vector<track_data_t>& detected)
{
	float minRatio = _signRatio.low;
	float maxRatio = _signRatio.high;
	float minArea = _signArea.low;
	float maxArea = _signArea.high;
	cv::Scalar color = _features->GetOverlayColor(type);

	for (unsigned int i = 0; i < contours.size(); i++)
	{
		float area = (float)contourArea(contours[i]);
		if (area < minArea || area > maxArea)
			continue;
		Rect rect = boundingRect(contours[i]);
		float ratio = float(rect.width) / float(rect.height);
		if (ratio < minRatio || ratio > maxRatio)
			continue;

		Mat signImage = frame(rect);

		// illumination correction
		Mat illumCorrectedPlanes[3];
		Mat &illumCorrectedDebuggerView = illumCorrectedPlanes[0];
		split(signImage, illumCorrectedPlanes);
		_clahe->apply(illumCorrectedPlanes[0], illumCorrectedPlanes[0]);

		Mat signEdges;
		cv::Canny(illumCorrectedPlanes[0], signEdges, 150, 200, 3);
		std::string matchStr = _features->FindMatch(type, signEdges);
		if(!matchStr.empty())
		{
			track_data_t result;
			result.roi = rect;
			result.type = type;
			result.sign = matchStr;
			result.area = area;
			detected.push_back(result);
			if(_displayEnabled)
			{
				putText(display, matchStr, Point(rect.x, rect.y), FONT_HERSHEY_COMPLEX, 0.75, color, 1);
			}
		}
		if(_displayEnabled)
		{
			drawContours(display, contours, i, color, 2, LINE_8, hierarchy, 0, Point());
			rectangle(display, rect, color, 2, LINE_8, 0);
		}
	}
}

void ImgProcessor::StartTracking(cv::Mat& frame, std::vector<track_data_t>& detected)
{
	_trackers.clear();
	_trackRegion.clear();
	for(unsigned int i = 0; i < detected.size(); i++)
	{
		_trackers.push_back(Tracker::create("MEDIANFLOW"));// create tracker and choose which type to use: KCF good tracking can shift off its target will always try to find a target somewhere even if it doesnt exist
														 // MEDIANFLOW loses target if change is too big, can resize area but that can go wrong 
		_trackRegion.push_back(detected[i].roi);

		_trackers[i]->init(frame, _trackRegion[i]);
	}
}


void ImgProcessor::ProcessTracking(cv::Mat& frame, std::vector<track_data_t>& detected, cv::Mat& display)
{
	int64 t1, t2;
	t1 = getTickCount();
	for(unsigned int i = 0; i < _trackers.size(); i++)
	{
		if(!_trackers[i]->update(frame, _trackRegion[i]))
		{
			_trackers.erase(_trackers.begin() + i);
			_trackRegion.erase(_trackRegion.begin() + i);
			_detectedSigns.erase(_detectedSigns.begin() + i);
			i--;
		} else if(_displayEnabled)
		{
			Scalar color = _features->GetOverlayColor(_detectedSigns[i].type);
			putText(display, _detectedSigns[i].sign, Point(int(_trackRegion[i].x), int(_trackRegion[i].y)), FONT_HERSHEY_COMPLEX, 0.75, color, 1);
			rectangle(display, _trackRegion[i], color, 2, LINE_8, 0);
		}
	}
	t2 = getTickCount();
	_perf.sign.tracking = t2 - t1;
}

void ImgProcessor::UpdateSignCounter()
{
	for (auto detected : _detectedSigns)
	{
		// TODO include size of the sign
		float val = 1;
		_signsDetectedCounter[detected.sign] += val;
		_totalDetectedCounter += val;
	}
}

void ImgProcessor::UpdateMap(const std::vector<cv::Vec4f>& lines, const std::vector<cv::Point2f>& visible)
{
	Mat newMap = Mat::zeros(_map.cols, _map.rows, CV_32F);
	for(Vec4f line : lines)
	{
		Point2f a(line[0], line[1]); 
		Point2f b(line[2], line[3]);
		a /= _mapTileSize;
		b /= _mapTileSize;
		cv::line(newMap, a, b, Scalar(1.0f));
	}
	addWeighted(_map, 0.8, newMap, 0.2, 0, _map);
	if(_displayEnabled)
	{
		CV_Assert(visible.size() == 4);
		Mat display;
		cvtColor(_map, display, CV_GRAY2BGR);
		Scalar visibleColor(0, 0, 255);
		line(display, visible[0] / _mapTileSize, visible[1] / _mapTileSize, visibleColor);
		line(display, visible[1] / _mapTileSize, visible[2] / _mapTileSize, visibleColor);
		line(display, visible[2] / _mapTileSize, visible[3] / _mapTileSize, visibleColor);
		line(display, visible[3] / _mapTileSize, visible[0] / _mapTileSize, visibleColor);
		resize(display, display, Size(_map.cols, _map.rows)*3, 0, 0, INTER_NEAREST);
		Mat flipped;
		flip(display, flipped, 0);

		imshow("Map", flipped);
	}
}

void ImgProcessor::ResetSignCounter()
{
	_signsDetectedCounter.clear();
	_totalDetectedCounter = 0;
}

void ImgProcessor::GetSignProbabilities(std::map<std::string, float>& out)
{
	for (auto signs : _signsDetectedCounter) {
		out[signs.first] = signs.second / _totalDetectedCounter;
	}
}

float getTimeMs(int64 time)
{
	float freq = (float)getTickFrequency();
	return 1000 * time / freq;
}

void ImgProcessor::PrintPerf()
{	
	std::cout << GetPerfString(_perf) << std::endl;
}

std::string ImgProcessor::GetPerfString(const ImgProcessor::Performance& perf)
{
	std::stringstream ss;

	int64 pre = perf.pre.scale + perf.pre.display + perf.pre.bgr2lab +
		perf.pre.split + perf.pre.cam;
	int64 line = perf.line.blur + perf.line.canny + perf.line.hist + 
		perf.line.mask + perf.line.hough + perf.line.drawing;
	int64 sign = perf.sign.thresh + perf.sign.erosion + perf.sign.dilation +
		perf.sign.contour + perf.sign.detect + perf.sign.tracking;

	int64 frame = pre + line + sign;

	ss << "Performance:" << getTimeMs(frame) << "ms\n"
		"\tPreprocessing: " << getTimeMs(pre) << "ms\n"
		"\t\tscale:      " << getTimeMs(perf.pre.scale) << "ms\n"
		"\t\tCamera corr:" << getTimeMs(perf.pre.cam) << "ms\n"
		"\t\tdisplay:    " << getTimeMs(perf.pre.display) << "ms\n"
		"\t\tbgr2lab:    " << getTimeMs(perf.pre.bgr2lab) << "ms\n"
		"\t\tsplit:      " << getTimeMs(perf.pre.split) << "ms\n"
		"\tLine detection: " << getTimeMs(line) << "\n"
		"\t\tblur:       " << getTimeMs(perf.line.blur) << "ms\n"
		"\t\thistogram:  " << getTimeMs(perf.line.hist) << "ms\n"
		"\t\tmask:       " << getTimeMs(perf.line.mask) << "ms\n"
		"\t\tcanny:      " << getTimeMs(perf.line.canny) << "ms\n"
		"\t\though:      " << getTimeMs(perf.line.hough) << "ms\n"
		"\t\tdrawing:    " << getTimeMs(perf.line.drawing) << "ms\n"
		"\tSign detection: " << getTimeMs(sign) << "\n"
		"\t\tthresh:     " << getTimeMs(perf.sign.thresh) << "ms\n"
		"\t\terosion:    " << getTimeMs(perf.sign.erosion) << "ms\n"
		"\t\tdilation:   " << getTimeMs(perf.sign.dilation) << "ms\n"
		"\t\tcontouring: " << getTimeMs(perf.sign.contour) << "ms\n"
		"\t\tDetection:  " << getTimeMs(perf.sign.detect) << "ms\n"
		"\t\tTracking:   " << getTimeMs(perf.sign.tracking) << "ms\n"
		;
	return ss.str();
}

void ImgProcessor::UpdatePerf()
{
	// Preprocessing
	_avgPerf.pre.scale = (_avgPerf.pre.scale * 3 + _perf.pre.scale) / 4;
	_avgPerf.pre.cam = (_avgPerf.pre.cam * 3 + _perf.pre.cam) / 4;
	_avgPerf.pre.display = (_avgPerf.pre.display * 3 + _perf.pre.display) / 4;
	_avgPerf.pre.bgr2lab = (_avgPerf.pre.bgr2lab * 3 + _perf.pre.bgr2lab) / 4;
	_avgPerf.pre.split = (_avgPerf.pre.split * 3 + _perf.pre.split) / 4;

	// Line detection
	_avgPerf.line.blur = (_avgPerf.line.blur * 3 + _perf.line.blur) / 4;
	_avgPerf.line.hist = (_avgPerf.line.hist * 3 + _perf.line.hist) / 4;
	_avgPerf.line.mask = (_avgPerf.line.mask * 3 + _perf.line.mask) / 4;
	_avgPerf.line.canny = (_avgPerf.line.canny * 3 + _perf.line.canny) / 4;
	_avgPerf.line.hough = (_avgPerf.line.hough * 3 + _perf.line.hough) / 4;
	_avgPerf.line.drawing = (_avgPerf.line.drawing * 3 + _perf.line.drawing) / 4;

	// Sign detection
	_avgPerf.sign.thresh = (_avgPerf.sign.thresh * 3 + _perf.sign.thresh) / 4;
	_avgPerf.sign.erosion = (_avgPerf.sign.erosion * 3 + _perf.sign.erosion) / 4;
	_avgPerf.sign.dilation = (_avgPerf.sign.dilation * 3 + _perf.sign.dilation) / 4;
	_avgPerf.sign.contour = (_avgPerf.sign.contour * 3 + _perf.sign.contour) / 4;
	_avgPerf.sign.detect = (_avgPerf.sign.detect * 3 + _perf.sign.detect) / 4;
	_avgPerf.sign.tracking = (_avgPerf.sign.tracking * 3 + _perf.sign.tracking) / 4;
}