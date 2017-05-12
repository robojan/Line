#include "GLAccelerator.h"
#ifdef _WIN32
#include <gl/glew.h>
#include <GL/freeglut.h>
#else
#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <sys/types.h>
#include <cstdlib>
#endif

#include <iostream>
#include <fstream>
#include <streambuf>
#include <sstream>

#ifdef _WIN32
static void displayCallback()
{
}

#else

void GLAccelerator::CreateX11Window()
{
	_x_dpy = XOpenDisplay(NULL);
	if(!_x_dpy)
	{
		std::stringstream ss;
		ss << "Couldn't open display " << getenv("DISPLAY");
		throw ShaderProgramException(ss.str());
	}

	_egl_dpy = eglGetDisplay(_x_dpy);
	if(!_egl_dpy)
	{
		std::stringstream ss;
		ss << "eglGetDisplay() failed";
		throw ShaderProgramException(ss.str());
	}

	if(!eglInitialize(_egl_dpy, NULL, NULL))
	{
		std::stringstream ss;
		ss << "eglInitialize() failed";
		throw ShaderProgramException(ss.str());
	}

	Window root = DefaultRootWindow(_x_dpy);
	XSetWindowAttributes swa;
	swa.event_mask = ExposureMask;

	_x_win = XCreateWindow(_x_dpy, root, 0, 0, 640, 480, 0,
		CopyFromParent, InputOutput,
		CopyFromParent, CWEventMask,
		&swa);

	XSetWindowAttributes xattr;
	Atom atom;
	int one = 1;

	xattr.override_redirect = False;
	XChangeWindowAttributes(_x_dpy, _x_win, CWOverrideRedirect, &xattr);

	XWMHints hints;
	hints.input = True;
	hints.flags = InputHint;
	XSetWMHints(_x_dpy, _x_win, &hints);

	XMapWindow(_x_dpy, _x_win);
	XStoreName(_x_dpy, _x_win, "EVC GL accelerator");

	EGLint attr[] = {
		EGL_BUFFER_SIZE,16,
		EGL_RENDERABLE_TYPE,
		EGL_OPENGL_ES2_BIT,
		EGL_NONE
	};

	EGLConfig ecfg;
	EGLint num_config;

	if(!eglChooseConfig(_egl_dpy, attr, &ecfg, 1, &num_config))
	{
		std::stringstream ss;
		ss << "Failed to choose config(eglError: " << std::hex << eglGetError() << ")";
		throw ShaderProgramException(ss.str());
	}
	if(num_config != 1)
	{
		std::stringstream ss;
		ss << "Didn't get exactly one config, but " << num_config; 
		throw ShaderProgramException(ss.str());
	}

	_egl_surf = eglCreateWindowSurface(_egl_dpy, ecfg, _x_win, NULL);
	if( _egl_surf == EGL_NO_SURFACE)
	{
		std::stringstream ss;
		ss << "Unable to create EGL surface (eglError: " << std::hex << eglGetError() << ")";
		throw ShaderProgramException(ss.str());
	}

	EGLint ctxattr[] = {
		EGL_CONTEXT_CLIENT_VERSION, 2,
		EGL_NONE
	};
	_egl_ctx = eglCreateContext(_egl_dpy, ecfg, EGL_NO_CONTEXT, ctxattr);
	if(_egl_ctx == EGL_NO_CONTEXT)
	{
		std::stringstream ss;
		ss << "Unable to create EGL context (eglError: " << std::hex << eglGetError() << ")";
		throw ShaderProgramException(ss.str());
	}

	if(!eglMakeCurrent(_egl_dpy, _egl_surf, _egl_surf, _egl_ctx))
	{
		std::stringstream ss;
		ss << "eglMakeCurrent() failed";
		throw ShaderProgramException(ss.str());
	}

}
#endif

GLAccelerator::GLAccelerator() :
	_window(0), _outputSize(640,480)
{
#ifdef _WIN32
	int argc = 1;
	char *argv[1] = { (char*)"GLAccelerator" };
	glutInit(&argc, argv);
	glutInitContextVersion(2, 0);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	_window = glutCreateWindow("ComputeWindow");
	glutDisplayFunc(displayCallback);
	glewExperimental = true;
	if(glewInit() != GLEW_OK)
	{
		fprintf(stderr, "Could not initialize GLEW");
	}
#else
	CreateX11Window();
#endif

	InitGL();
}

GLAccelerator::~GLAccelerator()
{
#ifdef _WIN32
	glutDestroyWindow(_window);
#else

#endif
}

void GLAccelerator::Update()
{
#ifdef _WIN32
	glutMainLoopEvent();
#else

#endif
}

void GLAccelerator::SetOutputSize(int width, int height)
{
	_outputSize.width = width;
	_outputSize.height = height;

	glBindFramebuffer(GL_FRAMEBUFFER, _outputFBO);
	glDeleteTextures(1, &_outputTexture);
	glGenTextures(1, &_outputTexture);
	glBindTexture(GL_TEXTURE_2D, _outputTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _outputSize.width, _outputSize.height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _outputTexture, 0);
	//GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
	//glDrawBuffers(1, drawBuffers);
}

void GLAccelerator::SetThreshold1(cv::Scalar low, cv::Scalar high)
{
	_threshLow1 = low;
	_threshHigh1 = high;
}

void GLAccelerator::SetThreshold2(cv::Scalar low, cv::Scalar high)
{
	_threshLow2 = low;
	_threshHigh2 = high;
}

void GLAccelerator::SetThreshold3(cv::Scalar low, cv::Scalar high)
{
	_threshLow3 = low;
	_threshHigh3 = high;
}

void GLAccelerator::ReadInputTexture(const cv::Mat &input)
{
	int height = input.rows;
	int width = input.cols;

	if(height != _inputTextureSize.height || width != _inputTextureSize.width)
	{
		if(glIsTexture(_inputTexture))
		{
			glDeleteTextures(1, &_inputTexture);
		}
		glGenTextures(1, &_inputTexture);
		glBindTexture(GL_TEXTURE_2D, _inputTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	}
	glBindTexture(GL_TEXTURE_2D, _inputTexture);

	if (input.isContinuous())
	{
		const unsigned char *srcPtr = input.ptr<uchar>(0);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, srcPtr);
	}
	else
	{
		for (int y = 0; y < height; y++)
		{
			const unsigned char *srcPtr = input.ptr<uchar>(y);
			for (int x = 0; x < width; x++)
			{
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, y, width, 1, GL_RGB, GL_UNSIGNED_BYTE, srcPtr);
			}
		}
	}
}

void GLAccelerator::WriteOutputImage(cv::Mat& output, int fbo)
{
	output.create(_outputSize, CV_8UC3);
#ifdef _WIN32
	glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
#else
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
#endif
	glReadPixels(0, 0, _outputSize.width, _outputSize.height, GL_RGB, GL_UNSIGNED_BYTE, output.ptr());
}

void GLAccelerator::ProcessFrame(const cv::Mat &input, cv::Mat &maskImg)
{
	ReadInputTexture(input);

	glBindFramebuffer(GL_FRAMEBUFFER, _outputFBO);
	glViewport(0, 0, _outputSize.width, _outputSize.height);

	glUseProgram(_program);
	glEnableVertexAttribArray(_programLocs.a_vertex);
	glBindBuffer(GL_ARRAY_BUFFER, _vbo);
	glVertexAttribPointer(_programLocs.a_vertex, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
	glEnableVertexAttribArray(_programLocs.a_uv);
	glBindBuffer(GL_ARRAY_BUFFER, _uvbo);
	glVertexAttribPointer(_programLocs.a_uv, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, _inputTexture);
	glUniform1i(_programLocs.inputTexture, 0);
	glUniform3f(_programLocs.threshold1_low, (float)_threshLow1[0], (float)_threshLow1[1], (float)_threshLow1[2]);
	glUniform3f(_programLocs.threshold2_low, (float)_threshLow2[0], (float)_threshLow2[1], (float)_threshLow2[2]);
	glUniform3f(_programLocs.threshold3_low, (float)_threshLow3[0], (float)_threshLow3[1], (float)_threshLow3[2]);
	glUniform3f(_programLocs.threshold1_high, (float)_threshHigh1[0], (float)_threshHigh1[1], (float)_threshHigh1[2]);
	glUniform3f(_programLocs.threshold2_high, (float)_threshHigh2[0], (float)_threshHigh2[1], (float)_threshHigh2[2]);
	glUniform3f(_programLocs.threshold3_high, (float)_threshHigh3[0], (float)_threshHigh3[1], (float)_threshHigh3[2]);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glDisableVertexAttribArray(0);

	WriteOutputImage(maskImg, _outputFBO);

#ifdef _WIN32
	glutSwapBuffers();
#else

#endif
}

void GLAccelerator::LoadVertexShader(const std::string& path)
{
	std::string src = ReadFile(path);
	SetVertexShader(src);
}

void GLAccelerator::LoadFragmentShader(const std::string& path)
{
	std::string src = ReadFile(path);
	SetFragmentShader(src);
}

void GLAccelerator::SetVertexShader(const std::string &source)
{
	_vertexShader = glCreateShader(GL_VERTEX_SHADER);
	if(_vertexShader == 0 )
	{
		std::stringstream ss;
		ss << "Error creating vertex shader: " << (glGetError());
		throw ShaderProgramException(ss.str());
	}
	int length = (int)source.length();
	const char *sourceCstr = source.c_str();
	glShaderSource(_vertexShader, 1, &sourceCstr, &length);
	GLenum error = glGetError();
	if(error != GL_NO_ERROR)
	{
		std::stringstream ss;
		ss << "Error assigning source to vertex shader: " << (error);
		throw ShaderProgramException(ss.str());
	}

	glCompileShader(_vertexShader);
	GLint status;
	glGetShaderiv(_vertexShader, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE)
	{
		std::stringstream ss;
		ss << "Error compiling vertex shader: " << (error);
		GLint logLen;
		glGetShaderiv(_vertexShader, GL_INFO_LOG_LENGTH, &logLen);
		if(logLen > 0)
		{
			char *log = new char[logLen+1];
			int logResLen;
			glGetShaderInfoLog(_vertexShader, logLen + 1, &logResLen, log);
			ss << "\n" << log;
			delete[] log;
		}
		throw ShaderProgramException(ss.str());
	}
}

void GLAccelerator::SetFragmentShader(const std::string &source)
{
	_fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	if (_fragmentShader == 0)
	{
		std::stringstream ss;
		ss << "Error creating fragment shader: " << (glGetError());
		throw ShaderProgramException(ss.str());
	}
	int length = (int)source.length();
	const char *sourceCstr = source.c_str();
	glShaderSource(_fragmentShader, 1, &sourceCstr, &length);
	GLenum error = glGetError();
	if (error != GL_NO_ERROR)
	{
		std::stringstream ss;
		ss << "Error assigning source to fragment shader: " << (error);
		throw ShaderProgramException(ss.str());
	}

	glCompileShader(_fragmentShader);
	GLint status;
	glGetShaderiv(_fragmentShader, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE)
	{
		std::stringstream ss;
		ss << "Error compiling fragment shader: " << (error);
		GLint logLen;
		glGetShaderiv(_fragmentShader, GL_INFO_LOG_LENGTH, &logLen);
		if (logLen > 0)
		{
			char *log = new char[logLen + 1];
			int logResLen;
			glGetShaderInfoLog(_fragmentShader, logLen + 1, &logResLen, log);
			ss << "\n" << log;
			delete[] log;
		}
		throw ShaderProgramException(ss.str());
	}
}

void GLAccelerator::LinkProgram()
{
	_program = glCreateProgram();
	if(_program == 0)
	{
		std::stringstream ss;
		ss << "Error creating program: " << glGetError();
		throw ShaderProgramException(ss.str());
	}
	glAttachShader(_program, _vertexShader);
	GLenum error = glGetError();
	if (error != GL_NO_ERROR)
	{
		std::stringstream ss;
		ss << "Error attaching vertex shader to program: " << error;
		throw ShaderProgramException(ss.str());
	}
	glAttachShader(_program, _fragmentShader);
	error = glGetError();
	if (error != GL_NO_ERROR)
	{
		std::stringstream ss;
		ss << "Error attaching fragment shader to program: " << error;
		throw ShaderProgramException(ss.str());
	}
	glLinkProgram(_program);
	GLint status;
	glGetProgramiv(_program, GL_LINK_STATUS, &status);
	if (status == GL_FALSE)
	{
		std::stringstream ss;
		ss << "Error linking program: " << error;
		GLint logLen;
		glGetProgramiv(_program, GL_INFO_LOG_LENGTH, &logLen);
		if (logLen > 0)
		{
			char *log = new char[logLen + 1];
			int logResLen;
			glGetProgramInfoLog(_program, logLen + 1, &logResLen, log);
			ss << "\n" << log;
			delete[] log;
		}
		throw ShaderProgramException(ss.str());
	}

	_programLocs.inputTexture = glGetUniformLocation(_program, "inputTexture");
	if(_programLocs.inputTexture < 0)
	{
		fprintf(stderr, "Warning: can't find uniform inputTexture in shader program\n");
	}
	_programLocs.threshold1_low = glGetUniformLocation(_program, "threshold1_low");
	if (_programLocs.inputTexture < 0)
	{
		fprintf(stderr, "Warning: can't find uniform threshold1_low in shader program\n");
	}
	_programLocs.threshold2_low = glGetUniformLocation(_program, "threshold2_low");
	if (_programLocs.inputTexture < 0)
	{
		fprintf(stderr, "Warning: can't find uniform threshold2_low in shader program\n");
	}
	_programLocs.threshold3_low = glGetUniformLocation(_program, "threshold3_low");
	if (_programLocs.inputTexture < 0)
	{
		fprintf(stderr, "Warning: can't find uniform threshold3_low in shader program\n");
	}
	_programLocs.threshold1_high = glGetUniformLocation(_program, "threshold1_high");
	if (_programLocs.inputTexture < 0)
	{
		fprintf(stderr, "Warning: can't find uniform threshold2_high in shader program\n");
	}
	_programLocs.threshold2_high = glGetUniformLocation(_program, "threshold2_high");
	if (_programLocs.inputTexture < 0)
	{
		fprintf(stderr, "Warning: can't find uniform threshold2_high in shader program\n");
	}
	_programLocs.threshold3_high = glGetUniformLocation(_program, "threshold3_high");
	if (_programLocs.inputTexture < 0)
	{
		fprintf(stderr, "Warning: can't find uniform threshold3_high in shader program\n");
	}
	_programLocs.a_vertex = glGetAttribLocation(_program, "a_vertex");
	if (_programLocs.a_vertex < 0)
	{
		fprintf(stderr, "Warning: can't find attrib a_vertex in shader program\n");
	}
	_programLocs.a_uv = glGetAttribLocation(_program, "a_uv");
	if (_programLocs.a_uv < 0)
	{
		fprintf(stderr, "Warning: can't find attrib a_uv in shader program\n");
	}

}

void GLAccelerator::InitGL()
{
	const float vertexData[] = {
		-1, 1,
		1,1,
		-1,-1,
		1,-1
	};
	const float uvData[] = {
		0, 1,
		1, 1,
		0, 0,
		1, 0
	};
	_inputTexture = -1;
	_inputTextureSize = cv::Size();
	glGenBuffers(1, &_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, _vbo);
	glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float), vertexData, GL_STATIC_DRAW);
	glGenBuffers(1, &_uvbo);
	glBindBuffer(GL_ARRAY_BUFFER, _uvbo);
	glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float), uvData, GL_STATIC_DRAW);
	glGenFramebuffers(1, &_outputFBO);
	glBindFramebuffer(GL_FRAMEBUFFER, _outputFBO);
	glGenTextures(1, &_outputTexture);
	glBindTexture(GL_TEXTURE_2D, _outputTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _outputSize.width, _outputSize.height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _outputTexture, 0);
	//GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
	//glDrawBuffers(1, drawBuffers);
	//glGenVertexArrays(1, &_vao);
	//glBindVertexArray(_vao);
}

std::string GLAccelerator::ReadFile(const std::string& path)
{
	std::ifstream fs(path.c_str(), std::ios_base::in);
	std::string result;
	fs.seekg(0, std::ios::end);
	result.reserve(fs.tellg());
	fs.seekg(0, std::ios::beg);

	result.assign((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());
	return result;
}
