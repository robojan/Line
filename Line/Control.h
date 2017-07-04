#ifndef _CONTROL_H_H
#define _CONTROL_H_H

#include <string>
#include <opencv2/core.hpp>

class ImgProcessor;
class Communication;

class Control
{
	enum class State {
		Init,
		Drive,
		Stop,
		UTurn,
		FinishCommand,
		SignFinishedCommand,
		DriveForwardTurnRight,
		DriveForwardTurnLeft,
		DriveForwardUTurn,
	};

public:
	Control(ImgProcessor *imgProcessor, Communication *comm, bool debug);
	~Control();

	void Update(float deltaTime);
	void correctPos();

	static std::string GetStateName(State state);

private:
	void GetMapDistances(float &l, float &m, float &r);
	float GetMapIntersection(cv::Mat map, cv::Point2f start, float angle, float stepSize);

	ImgProcessor *_img;
	Communication *_comm;
	std::string _old;
	int _filter;
	State _state;
	bool _debug;
	int _driveSpeed;
	int _turnSpeed;
	int _forwardDistance;
	int _checkReadyReceived;
};

#endif