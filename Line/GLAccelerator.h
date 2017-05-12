#pragma once

#include <string>
#include <opencv2/core/mat.hpp>
#include <stdexcept>

#ifndef _WIN32
#include <X11/Xlib.h>
#include <EGL/egl.h>
#endif

class ShaderProgramException : public std::runtime_error
{
public:
	ShaderProgramException(std::string msg) : std::runtime_error(msg) {}
	ShaderProgramException(const char *msg) : std::runtime_error(msg) {}
	~ShaderProgramException() {}
};

class GLAcceleratorException : public std::runtime_error
{
public:
	GLAcceleratorException(std::string msg) : std::runtime_error(msg) {}
	GLAcceleratorException(const char *msg) : std::runtime_error(msg) {}
	~GLAcceleratorException() {}
};

class GLAccelerator
{
public:
	GLAccelerator();
	~GLAccelerator();

	void Update();

	void SetOutputSize(int width, int height);
	void SetThreshold1(cv::Scalar low, cv::Scalar high);
	void SetThreshold2(cv::Scalar low, cv::Scalar high);
	void SetThreshold3(cv::Scalar low, cv::Scalar high);

	void ProcessFrame(const cv::Mat &input, cv::Mat &maskImg);

	void LoadVertexShader(const std::string &path);
	void LoadFragmentShader(const std::string &path);
	void SetVertexShader(const std::string &source);
	void SetFragmentShader(const std::string &source);
	void LinkProgram();

private:
	void InitGL();
	std::string ReadFile(const std::string &path);
	void ReadInputTexture(const cv::Mat &input);
	void WriteOutputImage(cv::Mat &output, int fbo);
#ifndef _WIN32
	void CreateX11Window();

	// X11 variables
	Display *_x_dpy;
	Window _x_win;
	EGLSurface _egl_surf;
	EGLContext _egl_ctx;
	EGLDisplay _egl_dpy;
#endif

	int _window;
	cv::Scalar _threshLow1;
	cv::Scalar _threshLow2;
	cv::Scalar _threshLow3;
	cv::Scalar _threshHigh1;
	cv::Scalar _threshHigh2;
	cv::Scalar _threshHigh3;

	// GL variables
	cv::Size _outputSize;
	cv::Size _inputTextureSize;
	unsigned int _inputTexture;
	unsigned int _vertexShader;
	unsigned int _fragmentShader;
	unsigned int _program;
	unsigned int _vbo;
	unsigned int _uvbo;
	unsigned int _vao;
	unsigned int _outputFBO;
	unsigned int _outputTexture;

	
	// Shader location
	struct
	{
		int a_vertex;
		int a_uv;
		int inputTexture;
		int threshold1_low;
		int threshold2_low;
		int threshold3_low;
		int threshold1_high;
		int threshold2_high;
		int threshold3_high;
	} _programLocs;
};
