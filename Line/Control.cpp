#include "Control.h"
#include "ImgProcessor.h"
#include "Communication.h"

Control::Control(ImgProcessor * imgProcessor, Communication * comm) :
	_state(State::Init), _img(imgProcessor), _comm(comm)
{
	assert(imgProcessor != nullptr);
	assert(comm != nullptr);
}

Control::~Control()
{

}

void Control::Update(float deltaTime)
{
	switch (_state)
	{
	case State::Init:
		break;
	default:
		throw new std::runtime_error("Reached unknown state");
	}
}
