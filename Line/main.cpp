#include <cstdio>
#include "OptionsManager.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "ImgProcessor.h"
#include <iomanip>
#include <iostream>
#include <string>
#include "control.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using namespace cv;

cv::VideoCapture OpenVideoCapture(std::string device)
{
	char *p;
	long deviceNumber = strtol(device.c_str(), &p, 10);
	if (*p)
	{
		return cv::VideoCapture(device);
	}
	return cv::VideoCapture(deviceNumber);
}

void DrawSignStats(Mat &display, ImgProcessor &processor)
{
	std::map<std::string, float> probs;
	processor.GetSignProbabilities(probs);

	Point pos = Point(5, 40);

	for(auto sign : probs)
	{
		putText(display, sign.first + ": " + std::to_string(sign.second), pos, FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		pos += Point(0, 20);
	}
}

void mainLoopSingleFrame(Mat frame, ImgProcessor &processor)
{
	Mat display;
	int64 lastFrameTime = getTickCount();
	while ((char)waitKey(1) != 'q')
	{
		int64 now = getTickCount();
		double tickFrequency = getTickFrequency();
		double fps = tickFrequency / (now - lastFrameTime);
		lastFrameTime = now;

		processor.Process(frame, display, Point2f(10000,10000), 0);

		std::ostringstream fpsStream;
		fpsStream << "FPS: " << std::setprecision(2) << fps;

		if(!display.empty())
		{
			DrawSignStats(display, processor);

			putText(display, fpsStream.str(), Point(5, 20), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2);

			imshow("Output Display", display);
		}
		std::cout << fpsStream.str() << std::endl;
		processor.PrintPerf();
	}
}

void mainLoopVideo(VideoCapture &video, ImgProcessor &processor)
{
	Mat frame, display;
	int64 lastFrameTime = getTickCount();
	bool freeze = false;
	while (true)
	{
		char key = (char)waitKey(1);
		switch (key) {
		case 'q':
			break;
		case 'f':
			freeze = !freeze;
			break;
		}
		int64 now = getTickCount();
		double tickFrequency = getTickFrequency();
		double fps = tickFrequency / (now - lastFrameTime);
		lastFrameTime = now;

		if(!freeze)
		{
			video >> frame;
		}
		if (frame.empty()) {
			break;
		}
		processor.Process(frame, display, Point2f(10000,10000), 0);
		
		std::ostringstream fpsStream;
		fpsStream << "FPS: " << std::setprecision(2) << fps;
		if (!display.empty())
		{
			DrawSignStats(display, processor);

			putText(display, fpsStream.str(), Point(5, 20), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2);
		
			imshow("Output Display", display);
		}
		std::cout << fpsStream.str() << std::endl;
		processor.PrintPerf();
	}
}

const char *GetCWD()
{
	static char cwd[260];
#ifdef _WIN32
	GetCurrentDirectoryA(260, cwd);
	return cwd;
#else 
	getcwd(cwd, 1024);
	return cwd;
#endif
}

int main(int argc, char **argv)
{
	OptionsManager options(argc, argv);

	auto parameters = options.GetParameters();
	if (options.IsDebugMode())
	{
		printf("Parameters:\n");
		for (auto parameter : parameters)
		{
			printf("\t%s\n", parameter.c_str());
		}

		printf("Current working dir: %s\n", GetCWD());
	}

	printf("Loading feature library\n");
	FeatureLibrary library(400);
	try
	{
		library.Add(FeatureType::BlueSign, "Left", "signs/left_64.jpg");
		library.Add(FeatureType::BlueSign, "Right", "signs/right_64.jpg");
		library.Add(FeatureType::BlueSign, "Forward", "signs/straight_64.jpg");
		library.Add(FeatureType::RedSign, "Stop", "signs/stop_64.jpg");
		library.Add(FeatureType::YellowSign, "UTurn", "signs/uturn_64.jpg");
		library.SetTypeOverlayColor(FeatureType::BlueSign, Scalar(255, 0, 0));
		library.SetTypeOverlayColor(FeatureType::RedSign, Scalar(0, 0, 255));
		library.SetTypeOverlayColor(FeatureType::YellowSign, Scalar(0, 255, 255));
	}
	catch (ImageReadException &e)
	{
		fprintf(stderr, "Error: %s\n", e.what());
		return 3;
	}

	ImgProcessor processor(options.GetProcessingResolution(), &library, options.IsAccelerated(), 
		options.GetMapSize(), options.GetMapTileSize());
	processor.SetHorizon(options.GetHorizon());
	processor.SetSkyLimit(options.GetSkyLimit());
	processor.EnableDisplay(options.IsDebugMode());
	processor.SetSignRatioLimit(options.GetSignRatioLimit()[0], options.GetSignRatioLimit()[1]);
	processor.SetSignAreaLimit(options.GetSignAreaLimit()[0], options.GetSignAreaLimit()[1]);
	processor.SetThreshold(FeatureType::BlueSign, options.GetColorThreshold(FeatureType::BlueSign));
	processor.SetThreshold(FeatureType::RedSign, options.GetColorThreshold(FeatureType::RedSign));
	processor.SetThreshold(FeatureType::YellowSign, options.GetColorThreshold(FeatureType::YellowSign));
	processor.SetCameraCorrection(options.GetCameraCorrectionMatrix(), options.GetCameraCorrectionDist(),
		options.GetCameraCorrectionTSR());
	processor.SetIPT(options.GetIPTMatrix());
	processor.SetTrackFrames(options.IsTracking() ? 30 * 1 : 0);

	if (options.GetControlDevice().empty()) {
		fprintf(stdout, "No serial device given. use --serial dev to specify the device\n");
	}

	Control control(options.GetControlDevice(), options.GetBaudRate());

	std::string captureDevice = options.GetCaptureDevice();
	if (captureDevice.empty())
	{
		if (parameters.size() == 0)
		{
			fprintf(stderr, "Please give an image file\n");
			return 1;
		}

		printf("Reading input image %s\n", parameters[0].c_str());
		Mat image = cv::imread(parameters[0], cv::IMREAD_COLOR);
		if (image.empty())
		{
			fprintf(stderr, "Could not open or find the image\n");
			return 2;
		}
		mainLoopSingleFrame(image, processor);
	}
	else
	{
		printf("Opening capture device %s\n", captureDevice.c_str());
		VideoCapture capture = OpenVideoCapture(captureDevice);
		if (!capture.isOpened())
		{
			fprintf(stderr, "Could not open capture device\n");
			return 5;
		}

		if (options.GetProcessingResolution().height > 0 && options.GetProcessingResolution().width > 0) {
			capture.set(CV_CAP_PROP_FRAME_WIDTH, options.GetProcessingResolution().width);
			capture.set(CV_CAP_PROP_FRAME_HEIGHT, options.GetProcessingResolution().height);
		}

		mainLoopVideo(capture, processor);
	}
    return 0;
}
