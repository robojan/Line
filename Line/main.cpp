#include <cstdio>
#include "OptionsManager.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "ImgProcessor.h"
#include <iomanip>

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

		imshow("Output Display", display);
		processor.PrintPerf();
	}
}

void mainLoopVideo(VideoCapture video, ImgProcessor &processor)
{
	Mat frame, display;
	int64 lastFrameTime = getTickCount();
	while ((char)waitKey(1) != 'q')
	{
		int64 now = getTickCount();
		double tickFrequency = getTickFrequency();
		double fps = tickFrequency / (now - lastFrameTime);
		lastFrameTime = now;

		video >> frame;
		if (frame.empty()) {
			break;
		}
		processor.Process(frame, display);
		
		std::ostringstream fpsStream;
		fpsStream << "FPS: " << std::setprecision(2) << fps;
		putText(display, fpsStream.str(), Point(5, 20), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2);
		imshow("Output Display", display);
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

	ImgProcessor processor(options.GetProcessingResolution());
	processor.SetHorizon(options.GetHorizon());

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
