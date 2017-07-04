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

	std::vector<cv::Vec4f> lines;
	HoughLinesP(map, lines, 2, 30 * CV_PI / 180, 5, 4, 2);

	Point2f origin(map.cols / 2, 0);
	m = GetMapIntersection(map, origin, 0, 0.5);


	if (_debug) {

		Mat display;
		const float scale = 4;
		resize(map, display, Size(map.cols*scale, map.rows*scale), scale, scale, CV_INTER_NN);
		cvtColor(display, display, CV_GRAY2BGR);

		Scalar lineColor(0, 0, 255);
		for (auto line : lines) {
			Point2f a(line[0], line[1]);
			Point2f b(line[2], line[3]);
			a *= scale;
			b *= scale;
			cv::line(display, a, b, lineColor, 2);
		}

		imshow("map", display);
	}

	

	l = INFINITY;
	m = INFINITY;
	r = INFINITY;
}

float Control::GetMapIntersection(Mat map, Point2f start, float angle, float stepSize)
{
	Point2f pos = start;
	
	Point pixel = 

	return 0.0f;
}
