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

		processor.Process(frame, display);

		std::ostringstream fpsStream;
		fpsStream << "FPS: " << std::setprecision(2) << fps;
		putText(display, fpsStream.str(), Point(5, 20), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2);

		if(!display.empty())
		{
			imshow("Output Display", display);
		}
		std::cout << fpsStream.str() << std::endl;
		processor.PrintPerf();
	}
}

void mainLoopVideo(VideoCapture video, ImgProcessor &processor)
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
		processor.Process(frame, display);
		
		std::ostringstream fpsStream;
		fpsStream << "FPS: " << std::setprecision(2) << fps;
		putText(display, fpsStream.str(), Point(5, 20), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2);
		if (!display.empty())
		{
			imshow("Output Display", display);
		}
		std::cout << fpsStream.str() << std::endl;
		processor.PrintPerf();
	}
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

	ImgProcessor processor(options.GetProcessingResolution(), &library, options.IsAccelerated());
	processor.SetHorizon(options.GetHorizon());
	processor.EnableDisplay(options.IsDebugMode());
	processor.SetSignRatioLimit(0.5f, 2.0f);
	processor.SetSignAreaLimit(100, 5000);
	processor.SetThreshold(FeatureType::BlueSign, ColorThreshold(Scalar(0, 130, 92), Scalar(255, 140, 105)));
	processor.SetThreshold(FeatureType::RedSign, ColorThreshold(Scalar(0, 170, 138), Scalar(255, 180, 155)));
	processor.SetThreshold(FeatureType::YellowSign, ColorThreshold(Scalar(0, 145, 190), Scalar(255, 160, 210)));
	processor.SetTrackFrames(options.IsTracking() ? 30 * 1 : 0);

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
