#ifndef _CONTROL_H_H
#define _CONTROL_H_H

#include <string>

class ImgProcessor;
class Communication;

class Control
{
	enum class State {
		Init,
		Uturn,
		Stop,
		Left,
		Right,
		Forward


	};

public:
	Control(ImgProcessor *imgProcessor, Communication *comm);
	~Control();

	void Update(float deltaTime);
	void Control::correctPos();

private:
	ImgProcessor *_img;
	Communication *_comm;
	std::string _old;
	int _filter;
	State _state;
};

#endif