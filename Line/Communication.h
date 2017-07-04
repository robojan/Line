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


class CommunicationException : public std::runtime_error
{
public:
	CommunicationException(const char *msg) : std::runtime_error(msg) {}
	CommunicationException(const std::string &msg) : std::runtime_error(msg) {}
};

class Communication {
public:
	Communication(const std::string &dev, int baud);
	~Communication();

	bool IsConnected();

	void Forward(int distance, int speed);
	void Turn(int angle, int speed);

	void Sweep(bool on);

	float GetEnergyIn();
	float GetEnergyOut();
	int GetReadyReceived() const;
private:
	std::string _devPath;
	int _baud;
	int _readyReceived;
#ifndef _WIN32
	void Connect();
	void SendMessage(const std::string &msg);
	bool GetMessage(std::string &msg);
	void Tokenize(std::vector<std::string> &tokens, const std::string &msg);
	void ProcessMessage(const char *str);
	static void *ReadThread(Communication *self);

	int _fd;
	bool _running;
	pthread_t _readingThread;
#endif
};