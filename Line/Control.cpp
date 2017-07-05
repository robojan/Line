#include "Control.h"
#include "ImgProcessor.h"
#include "Communication.h"

#include <opencv2/highgui.hpp>

using namespace cv;

Control::Control(ImgProcessor * imgProcessor, Communication * comm, bool debug) :
	_state(State::Init), _img(imgProcessor), _comm(comm), _old("none"), _filter(0),
	_debug(debug), _forwardDistance(20), _driveSpeed(40), _turnSpeed(40), _infSeen(0)
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
	int detectedLines = 0;
	detectedLines |= lineR < 8 ? 1 : 0;
	detectedLines |= lineM < 10 ? 2 : 0;
	detectedLines |= lineL < 8 ? 4 : 0;
	std::string sign;
	float signProb;
	_img->GetMostProbableSign(sign, signProb);

	int forwardDist = 100;
	if (!isinf(lineM)) {
		forwardDist = min(forwardDist, int(lineM * 10) -30);
	}
	switch (_state)
	{
	case State::Init: {
		_img->ResetSignCounter();
		_state = State::Drive;
		break;
	}
	case State::Drive: {
		if (signProb >= 20 && sign == "Stop") {
			_state = State::Stop;
		}
		else if (signProb >= 20 && sign == "UTurn") {
			_state = State::UTurn;
		}
		else if (detectedLines == 5 && abs(lineR - lineL) > 1) {
			_checkReadyReceived = _comm->GetReadyReceived();
			_comm->Turn(lineR -lineL > 0 ? 2 : -2, _turnSpeed);
			_state = State::FinishCommand;
		}
		else {
			switch (detectedLines)
			{
			case 1:
			case 4:
			case 5: // Forward
				_state = State::FinishCommand;
				_checkReadyReceived = _comm->GetReadyReceived();
				_comm->Forward(20, _driveSpeed);
				break;
			case 6: // Turn right
				_state = State::DriveForwardTurnRight;
				_checkReadyReceived = _comm->GetReadyReceived();
				_comm->Forward(10, _driveSpeed);
				break;
			case 3: // Turn left
				_state = State::DriveForwardTurnLeft;
				_checkReadyReceived = _comm->GetReadyReceived();
				_comm->Forward(10, _driveSpeed);
				break;
			case 0:
			case 2: // Crossing
				if (sign == "Left" && signProb > 15) {
					_state = State::DriveForwardTurnLeft;
					_checkReadyReceived = _comm->GetReadyReceived();
					_comm->Forward(forwardDist, _driveSpeed);
				}
				else if (sign == "Right" && signProb > 15) {
					_state = State::DriveForwardTurnRight;
					_checkReadyReceived = _comm->GetReadyReceived();
					_comm->Forward(forwardDist, _driveSpeed);
				}
				else {
					_state = State::FinishCommand;
					_checkReadyReceived = _comm->GetReadyReceived();
					_comm->Forward(forwardDist, _driveSpeed);
				}
				_img->ResetSignCounter();
				break;
			case 7: // I Dunno, uturn maybe
				std::cout << "I see all the lines" << std::endl;
				_state = State::FinishCommand;
				_checkReadyReceived = _comm->GetReadyReceived();
				_comm->Forward(-20, _driveSpeed);
				break;
			}
		}
		break;
	}
	case State::FinishCommand: {
		if (_checkReadyReceived != _comm->GetReadyReceived()) {
			_state = State::Drive;
		}
		break;
	}
	case State::SignFinishedCommand: {
		if (_checkReadyReceived != _comm->GetReadyReceived()) {
			_state = State::Drive;
			_img->ResetSignCounter();
		}
		break;
	}
	case State::DriveForwardTurnLeft: {
		if (_checkReadyReceived != _comm->GetReadyReceived()) {
			if (!isinf(lineM) && lineM <= 5) {
				_img->ResetSignCounter();
				_checkReadyReceived = _comm->GetReadyReceived();
				_comm->Turn(-90, _turnSpeed);
				_state = State::SignFinishedCommand;
				_infSeen = 0;
			}
			else if(!isinf(lineM) && lineM > 5) {
				_checkReadyReceived = _comm->GetReadyReceived();
				_comm->Forward(10, _driveSpeed);
				_infSeen = 0;
			}
			else {
				if (_infSeen > 0) {
					_state = State::Drive;
					_infSeen = 0;
				}
				else {
					_infSeen++;
				}
			}
		}
		break;
	}
	case State::DriveForwardTurnRight: {
		if (_checkReadyReceived != _comm->GetReadyReceived()) {
			if (!isinf(lineM) && lineM <= 5) {
				_img->ResetSignCounter();
				_checkReadyReceived = _comm->GetReadyReceived();
				_comm->Turn(90, _turnSpeed);
				_state = State::SignFinishedCommand;
				_infSeen = 0;
			}
			else if (!isinf(lineM) && lineM > 5) {
				_checkReadyReceived = _comm->GetReadyReceived();
				_comm->Forward(10, _driveSpeed);
				_infSeen = 0;
			}
			else {
				if (_infSeen > 0) {
					_state = State::Drive;
					_infSeen = 0;
				}
				else {
					_infSeen++;
				}
			}
		}
		break;
	}
	case State::Stop: {
		_img->ResetSignCounter();
		// WHAT TO DO!?!?!
		break;
	}
	case State::UTurn: {
		_img->ResetSignCounter();
		_checkReadyReceived = _comm->GetReadyReceived();
		int forwardDist = 100;
		if (!isinf(lineM)) {
			forwardDist = min(forwardDist, int(lineM * 10) / 2);
		}
		_comm->Forward(forwardDist, _driveSpeed);
		_state = State::DriveForwardUTurn;
		break;
	}
	case State::DriveForwardUTurn: {
		if (_checkReadyReceived != _comm->GetReadyReceived()) {
			_checkReadyReceived = _comm->GetReadyReceived();
			_img->ResetSignCounter();
			_comm->Turn(180, _turnSpeed);
			_state = State::SignFinishedCommand;
		}
		break;
	}
	default:
		throw new std::runtime_error("Reached unknown state");
	}
	fprintf(stdout, "State: %s, Sign: %s(%g), lines: %g %g %g\n", GetStateName(_state).c_str(), sign.c_str(), signProb, lineL, lineM, lineR);
}

void Control::correctPos() {


}

std::string Control::GetStateName(State state)
{
	switch (state)
	{
	case State::Init: return "Init";
	case State::Drive: return "Drive";
	case State::Stop: return "Stop";
	case State::UTurn: return "UTurn";
	case State::FinishCommand: return "FinishCommand";
	case State::DriveForwardTurnLeft: return "DriveForwardTurnLeft";
	case State::DriveForwardTurnRight: return "DriveForwardTurnRight";
	case State::DriveForwardUTurn: return "DriveForwardUTurn";
	default: return "Unknown";
	}
}

void Control::GetMapDistances(float & l, float & m, float & r)
{
	cv::Mat map;
	_img->GetMap(map);

	//std::vector<cv::Vec4f> lines;
	//HoughLinesP(map, lines, 2, 30 * CV_PI / 180, 5, 4, 2);

	Point2f origin(map.cols / 2, 0);
	const float lAngle = -50 * CV_PI / 180;
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
