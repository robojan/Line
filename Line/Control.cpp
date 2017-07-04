#include "Control.h"
#include "ImgProcessor.h"
#include "Communication.h"

#include <opencv2/highgui.hpp>

using namespace cv;

Control::Control(ImgProcessor * imgProcessor, Communication * comm, bool debug) :
	_state(State::Init), _img(imgProcessor), _comm(comm), _old("none"), _filter(0),
	_debug(debug)
{
	assert(imgProcessor != nullptr);
	assert(comm != nullptr);
}

Control::~Control()
{

}

void Control::Update(float deltaTime)
{
	float lineL, lineM, lineR;
	GetMapDistances(lineL, lineM, lineR);
	std::map<std::string, float> detected;
	switch (_state)
	{
	case State::Init: {
		
		
		Control::correctPos();
		//drive forward 
		_comm->Forward(10, 80);
		//if sign is detected

		std::string name;
		float prob;
		//if detected sign right next state is right
		//if detected sign left next state is left
		//if detected sign forward next state is forward
		//if detected sign stop next state is stop
		//if detected sign uturn next state is uturn
		_img->GetMostProbableSign(name, prob);
		if (prob < 0.1) {
			name = "none";
		}
		std::cout << name << std::endl;
		if (name == _old) {
			_filter = _filter + 1;
			if (_filter > 5) {
				if (name == "Left") {
					_state = State::Left;
				}
				if (name == "Right") {
					_state = State::Right;
				}
				if (name == "Uturn") {
					_state = State::Uturn;
				}
				if (name == "Forward") {
					_state = State::Forward;
				}
				if (name == "Stop") {
					_state = State::Stop;
				}

			_filter = 0;
			}
		}
		else {
			_filter = 0;
		}
		_old = name;
		//_img->ResetSignCounter();

		break;
	}
	case State::Left:

		_state = State::Init;
		break;

	case State::Forward:
		
		_state = State::Init;
		break;

	case State::Right:

		_state = State::Init;
		break;

	case State::Uturn:
		_comm->Turn(180, 50);
		//todo wait untill turn is finished
		
		_state = State::Init;
		break;

	case State::Stop:
		_comm->Forward(0, 0);

		break;
	default:
		throw new std::runtime_error("Reached unknown state");
	}
}

void Control::correctPos() {


}

void Control::GetMapDistances(float & l, float & m, float & r)
{
	cv::Mat map;
	_img->GetMap(map);

	//std::vector<cv::Vec4f> lines;
	//HoughLinesP(map, lines, 2, 30 * CV_PI / 180, 5, 4, 2);

	Point2f origin(map.cols / 2, 0);
	const float lAngle = 30 * CV_PI / 180;
	const float rAngle = -lAngle;
	const float dAngle = 5 * CV_PI / 180;
	float m1, m2, m3;
	float l1, l2, l3;
	float r1, r2, r3;
	l1 = GetMapIntersection(map, origin, lAngle - dAngle, 0.25);
	l2 = GetMapIntersection(map, origin, lAngle, 0.25);
	l3 = GetMapIntersection(map, origin, lAngle + dAngle, 0.25);
	m1 = GetMapIntersection(map, origin, 0 - dAngle, 0.25);
	m2 = GetMapIntersection(map, origin, 0, 0.25);
	m3 = GetMapIntersection(map, origin, 0 + dAngle, 0.25);
	r1 = GetMapIntersection(map, origin, rAngle - dAngle, 0.25);
	r2 = GetMapIntersection(map, origin, rAngle, 0.25);
	r3 = GetMapIntersection(map, origin, rAngle + dAngle, 0.25);
	
	l = min(l1, min(l2, l3));
	m = min(m1, min(m2, m3));
	r = min(r1, min(r2, r3));


	if (_debug) {

		Mat display;
		const float scale = 4;
		resize(map, display, Size(map.cols*scale, map.rows*scale), scale, scale, CV_INTER_NN);
		cvtColor(display, display, CV_GRAY2BGR);
		/*
		Scalar lineColor(0, 0, 255);
		for (auto line : lines) {
			Point2f a(line[0], line[1]);
			Point2f b(line[2], line[3]);
			a *= scale;
			b *= scale;
			cv::line(display, a, b, lineColor, 2);
		}*/
		float maxDist = sqrt(map.cols * map.cols + map.rows * map.rows);
		Scalar projectColor(0, 255, 0);
		
		float dist;
		dist = min(maxDist, l);
		Point2f lPoint = origin + Point2f(sin(lAngle)*dist, cos(lAngle)*dist);
		dist = min(maxDist, m);
		Point2f mPoint = origin + Point2f(sin(0)*dist, cos(0)*dist);
		dist = min(maxDist, r);
		Point2f rPoint = origin + Point2f(sin(rAngle)*dist, cos(rAngle)*dist);

		line(display, origin * scale, lPoint * scale, projectColor, 2);
		line(display, origin * scale, mPoint * scale, projectColor, 2);
		line(display, origin * scale, rPoint * scale, projectColor, 2);
		imshow("map", display);
	}
}

float Control::GetMapIntersection(Mat map, Point2f start, float angle, float stepSize)
{
	Rect mapRect(0, 0, map.cols, map.rows);
	float dist = 0;
	while (true)
	{
		Point2f pos = start + Point2f(sin(angle)*dist, cos(angle)*dist);
		Point pixel(pos + Point2f(0.5f, 0.5f));
		if (!mapRect.contains(pixel))
		{
			break;
		}
		if (map.at<uchar>(pixel) > 128) {
			return dist;
		}
		dist += stepSize;
	}
	return INFINITY;
}
