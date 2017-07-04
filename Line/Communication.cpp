#include "Communication.h"

#include <iostream>
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

Communication::Communication(const std::string & dev, int baud) :
	_devPath(dev), _baud(baud), _readyReceived(0)
{

}

Communication::~Communication()
{

}

bool Communication::IsConnected()
{
	return false;
}

void Communication::Forward(int distance, int speed)
{
	std::cout << "Forward " << distance << std::endl;
	_readyReceived++;
}

void Communication::Turn(int angle, int speed)
{
	std::cout << "Turn " << angle << std::endl;
	_readyReceived++;
}

void Communication::Sweep(bool on)
{

}

float Communication::GetEnergyIn()
{
	return 0;
}

float Communication::GetEnergyOut()
{
	return 0;
}

int Communication::GetReadyReceived() const
{
	return _readyReceived;
}

#else
Communication::Communication(const std::string & dev, int baud) :
	_devPath(dev), _baud(baud), _fd(-1), _running(true)
{
	if (!dev.empty()) {
		Connect();
	}
	int status = pthread_create(&_readingThread, NULL, (void *(*)(void*))Communication::ReadThread, this);
	if(status)
	{
		throw CommunicationException("Could not create listening thread: " + std::to_string(status));
	}
}

Communication::~Communication()
{
	_running = false;
	pthread_join(_readingThread, NULL);

	if (IsConnected()) {
		close(_fd);
	}
}

bool Communication::IsConnected()
{
	return _fd >= 0;
}

void Communication::Forward(int distance, int speed)
{
	std::string msg = "$DRIVE," + std::to_string(distance) + "," + std::to_string(speed) + "\n";
	SendMessage(msg);
}

void Communication::Turn(int angle, int speed)
{
	std::string msg = "$TURN," + std::to_string(angle) + "," + std::to_string(speed) + "\n";
	SendMessage(msg);
}

void Communication::Sweep(bool on)
{
	if (on) {
		SendMessage("$SWEEP,1\n");
	}
	else {
		SendMessage("$SWEEP,0\n");
	}
}

float Communication::GetEnergyIn()
{
	return 0;
}

float Communication::GetEnergyOut()
{
	return 0;
}

void Communication::Connect()
{
	struct termios tio;

	int fd = open(_devPath.c_str(), O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		throw CommunicationException("Could not open serial device " + _devPath + ": " + strerror(errno));
	}

	memset(&tio, 0, sizeof(struct termios));
	tio.c_iflag = IGNCR;
	tio.c_oflag = 0;
	tio.c_cflag = CS8 | CREAD | CLOCAL;
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 1;

	cfsetispeed(&tio, _baud);
	cfsetospeed(&tio, _baud);

	tcsetattr(fd, TCSANOW, &tio);
	_fd = fd;

	sleep(3);
}
void Communication::SendMessage(const std::string & msg)
{
	if (!IsConnected())
	{
		return;
	}
	write(_fd, msg.c_str(), msg.length());
}

bool Communication::GetMessage(std::string & msg)
{
	return false;
}

void Communication::Tokenize(std::vector<std::string>& tokens, const std::string & msg)
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

void Communication::ProcessMessage(const char *str)
{
	if (str[0] == '$') {
		std::string msg(str + 1);
		std::vector<std::string> tokens;
		Tokenize(tokens, msg);
		std::string type = tokens[0];
		if (type == "ENERGY")
		{
		}
		else if (type == "READY")
		{
			_readyReceived++;
		}
		else {
			fprintf(stderr, "Unknown msg received: %s\n", type.c_str());
		}
	}
	else {
		printf("Arduino: %s\n", str);
	}
}

int Communication::GetReadyReceived() const
{
	return _readyReceived;
}

void *Communication::ReadThread(Communication * self)
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