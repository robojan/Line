#ifndef _CONTROL_H_H
#define _CONTROL_H_H

class ImgProcessor;
class Communication;

class Control
{
	enum class State {
		Init
	};

public:
	Control(ImgProcessor *imgProcessor, Communication *comm);
	~Control();

	void Update(float deltaTime);

private:
	ImgProcessor *_img;
	Communication *_comm;

	State _state;
};

#endif