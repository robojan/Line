#pragma once

#include <string>
#include <opencv2/core/mat.hpp>
#include <stdexcept>
#include <map>

#ifndef _WIN32

#ifndef _RASPI
#include <X11/Xlib.h>
#endif

#include <EGL/egl.h>
#endif

class GLAcceleratorException : public std::runtime_error
{
public:
	GLAcceleratorException(std::string msg) : std::runtime_error(msg) {}
	GLAcceleratorException(const char *msg) : std::runtime_error(msg) {}
	~GLAcceleratorException() {}
};

class ShaderProgramException : public GLAcceleratorException
{
public:
	ShaderProgramException(std::string msg) : GLAcceleratorException(msg) {}
	ShaderProgramException(const char *msg) : GLAcceleratorException(msg) {}
	~ShaderProgramException() {}
};

class GLProgram;

class GLAccelerator
{
	friend class GLProgram;
public:
	GLAccelerator(const std::string &shaderPath);
	~GLAccelerator();

	void Update();

	void ProcessFrame(const std::string &name, const cv::Mat &input);
	void ProcessFrame(const std::string &name, unsigned int glTexture);
	void GetResult(const std::string &name, cv::Mat &output);
	unsigned int GetResultTexture(const std::string &name);
	void CreateProgram(const std::string &name, cv::Size outputSize);
	void SetProgramUniform(const std::string &program, const std::string &uniform, int x);
	void SetProgramUniform(const std::string &program, const std::string &uniform, float x);
	void SetProgramUniform(const std::string &program, const std::string &uniform, const cv::Vec3f &x);
	void SetProgramUniformColor(const std::string &program, const std::string &uniform, const cv::Scalar &x);

private:
	void InitGL();
	const std::string &GetBaseShaderPath();
	unsigned int GetVertexShader();
	void BindVertexAttributes(int vertexLoc, int uvLoc);
	void LoadVertexShader(const std::string &path);
	void SetVertexShader(const std::string &source);
#ifndef _WIN32
	void CreateWindow();
	void CreateEGLWindow(EGLNativeWindowType win);

	// X11 variables
#ifdef _RASPI
	DISPMANX_ELEMENT_HANDLE_T _dispmanElement;
	DISPMANX_DISPLAY_HANDLE_T _dispmanDisplay;
	DISPMANX_UPDATE_HANDLE_T _dispmanUpdate;
	EGL_DISPMANX_WINDOW_T nativewindow;
#else
	Display *_x_dpy;
	Window _x_win;
#endif
	EGLSurface _egl_surf;
	EGLContext _egl_ctx;
	EGLDisplay _egl_dpy;
#endif

	int _window;
	std::string _basePath;
	std::map<std::string, cv::Ptr<GLProgram>> _programs;

	// GL variables
	unsigned int _vertexShader;
	unsigned int _vbo;
	unsigned int _uvbo;
};

class GLTexture
{
public:
	GLTexture();
	~GLTexture();

	unsigned int GetTexture();
	const cv::Size &GetSize();
	bool IsValid() const;

	void ReadTexture(const cv::Mat &input);
	void Bind();

private:
	unsigned int _texture;
	cv::Size _size;
};

class GLFramebuffer
{
public:
	GLFramebuffer(cv::Size size);
	~GLFramebuffer();

	unsigned int GetFramebuffer();
	unsigned int GetTexture();
	const cv::Size &GetSize();
	bool IsValid() const;
	void Bind();

	void GetImage(cv::Mat &output);

private:
	unsigned int _fbo;
	unsigned int _texture;
	cv::Size _size;
};

class GLProgram
{
public:
	GLProgram(GLAccelerator *accelerator, const std::string &name, cv::Size outputSize);
	~GLProgram();

	void SetUniform(const std::string &name, int i);
	void SetUniform(const std::string &name, float f);
	void SetUniform(const std::string &name, const cv::Vec3f &x);
	void SetUniformColor(const std::string &name, const cv::Scalar &x);

	void Process(const cv::Mat &input);
	void Process(unsigned int glTexture);
	void GetResult(cv::Mat &output);
	unsigned int GetResultTexture();
private:
	int GetUniformLoc(const std::string &name);
	int GetAttributeLoc(const std::string &name);
	void LoadFragmentShader(const std::string &name);
	void SetFragmentShader(const std::string &source);
	void LinkProgram();

	GLAccelerator *_accelerator;
	std::string _name;
	unsigned int _program;
	unsigned int _fragmentShader;
	GLTexture _input;
	GLFramebuffer _output;

	std::map<std::string, int> _uniformLocs;
	std::map<std::string, int> _attributeLocs;

};
