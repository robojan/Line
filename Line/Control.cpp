#include "Control.h"
#include "ImgProcessor.h"
#include "Communication.h"

#include <opencv2/highgui.hpp>

using namespace cv;

Control::Control(ImgProcessor * imgProcessor, Communication * comm) :
	_state(State::Init), _img(imgProcessor), _comm(comm), _old("none"), _filter(0)
{
	assert(imgProcessor != nullptr);
	assert(comm != nullptr);
}

Control::~Control()
{

}

void Control::Update(float deltaTime)
{
	cv::Mat map;
	_img->GetMap(map);
	cv::resize(map, map, cv::Size(map.cols, map.rows) * 6, 0, 0, cv::INTER_NEAREST);
	cv::imshow("MapBin", map);
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