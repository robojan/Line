#include "control.h"

#ifndef _WIN32
#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <vector>
#endif

#ifdef _WIN32

Control::Control(const std::string & dev) :
	_devPath(dev)
{

}

Control::~Control()
{

}

bool Control::IsConnected()
{
	return false;
}

void Control::Forward(float distance)
{

}

void Control::Turn(float angle)
{

}

void Control::Sweep(bool on)
{

}

float Control::GetEnergyIn()
{
	return 0;
}

float Control::GetEnergyOut()
{
	return 0;
}

#else
Control::Control(const std::string & dev) :
	_devPath(dev), _fd(-1), _running(true)
{
	if (!dev.empty()) {
		Connect();
	}
	int status = pthread_create(&_readingThread, NULL, (void *(*)(void*))Control::ReadThread, this);
	if(status)
	{
		throw ControlException("Could not create listening thread: " + std::to_string(status));
	}
}

Control::~Control()
{
	_running = false;
	pthread_join(_readingThread, NULL);

	if (IsConnected()) {
		close(_fd);
	}
}

bool Control::IsConnected()
{
	return _fd >= 0;
}

void Control::Forward(float distance)
{

}

void Control::Turn(float angle)
{

}

void Control::Sweep(bool on)
{

}

float Control::GetEnergyIn()
{
	return 0;
}

float Control::GetEnergyOut()
{
	return 0;
}

void Control::Connect()
{
	struct termios tio;

	int fd = open(_devPath.c_str(), O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		throw ControlException("Could not open serial device " + _devPath + ": " + strerror(errno));
	}

	memset(&tio, 0, sizeof(struct termios));
	tio.c_iflag = IGNCR;
	tio.c_oflag = 0;
	tio.c_cflag = CS8 | CREAD | CLOCAL;
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 1;

	cfsetispeed(&tio, 9600);
	cfsetospeed(&tio, 9600);

	tcsetattr(fd, TCSANOW, &tio);
	_fd = fd;
}
void Control::SendMessage(const std::string & msg)
{
	if (!IsConnected())
	{
		return;
	}
	write(_fd, msg.c_str(), msg.length());
}

bool Control::GetMessage(std::string & msg)
{
	return false;
}

void Control::Tokenize(std::vector<std::string>& tokens, const std::string & msg)
{
	int pos = 0;
	while(true)
	{
		size_t it = msg.find_first_of(',', pos);
		if (it == msg.npos) {
			std::string token = msg.substr(pos);
			tokens.push_back(token);
			break;
		}
		std::string token = msg.substr(pos, it - pos);
		pos = it + 1;
		tokens.push_back(token);
	}
}

void Control::ProcessMessage(const char *str)
{
	if (str[0] == '$') {
		std::string msg(str + 1);
		std::vector<std::string> tokens;
		Tokenize(tokens, msg);
		std::string type = tokens[0];
		if (type == "ENERGY")
		{
			
		}
		else {
			fprintf(stderr, "Unknown msg received: %s\n", type.c_str());
		}
	}
	else {
		printf("Arduino: %s\n", str);
	}
}

void *Control::ReadThread(Control * self)
{
	std::vector<char> buffer(50);
	while(self->_running)
	{
		if(!self->IsConnected())
		{
			sleep(1);
			continue;
		}
		char c;
		while(read(self->_fd, &c, 1) > 0)
		{
			if(c == '\n')
			{
				buffer.push_back('\0');
				self->ProcessMessage(buffer.data());
				buffer.clear();
			} else
			{
				buffer.push_back(c);
			}
		}
	}
	return NULL;
}
#endif