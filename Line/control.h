#pragma once

#include <string>
#include <vector>
#include <stdexcept>

#ifndef _WIN32
#include <pthread.h>
#endif

/*
 * Protocol:
 * 9600 8n1
 * From arduino:
 * $ENERGY,in,out
 * To arduino:
 * $DRIVE,dist  # dist cm [-100, 100]
 * $TURN,angle  # angle deg [-180, 180]
 * $SWEEP,on    # on bool [0,1]
 */


class ControlException : public std::runtime_error
{
public:
	ControlException(const char *msg) : std::runtime_error(msg) {}
	ControlException(const std::string &msg) : std::runtime_error(msg) {}
};

class Control {
public:
	Control(const std::string &dev);
	~Control();

	bool IsConnected();

	void Forward(float distance);
	void Turn(float angle);

	void Sweep(bool on);

	float GetEnergyIn();
	float GetEnergyOut();
private:
	std::string _devPath;
#ifndef _WIN32
	void Connect();
	void SendMessage(const std::string &msg);
	bool GetMessage(std::string &msg);
	void Tokenize(std::vector<std::string> &tokens, const std::string &msg);
	void ProcessMessage(const char *str);
	static void *ReadThread(Control *self);

	int _fd;
	bool _running;
	pthread_t _readingThread;
#endif
};