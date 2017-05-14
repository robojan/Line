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

#define GL_CHECK_ERROR() \
	do { \
		GLenum error = glGetError(); \
		if (error != GL_NO_ERROR) \
		{ \
			std::stringstream ss; \
			ss << "OpenGL error " << __FILE__ << "(" << __LINE__ << "): " << std::hex << (error); \
			throw GLAcceleratorException(ss.str()); \
		} \
	} while (false)

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


std::string ReadFile(const std::string &path)
{
	std::ifstream fs(path.c_str(), std::ios_base::in);
	if(!fs.is_open())
	{
		throw std::runtime_error("Could not open the file: " + path);
	}
	std::string result;
	fs.seekg(0, std::ios::end);
	result.reserve(fs.tellg());
	fs.seekg(0, std::ios::beg);

	result.assign((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());
	return result;
}

GLTexture::GLTexture() : 
	_texture(-1), _size(0,0)
{
}

GLTexture::~GLTexture()
{
	if(IsValid())
	{
		glDeleteTextures(1, &_texture);
	}
}

unsigned int GLTexture::GetTexture()
{
	return _texture;
}

const cv::Size& GLTexture::GetSize()
{
	return _size;
}

bool GLTexture::IsValid() const
{
	return glIsTexture(_texture) == GL_TRUE;
}

void GLTexture::ReadTexture(const cv::Mat &input)
{
	int height = input.rows;
	int width = input.cols;

	if (height != _size.height || width != _size.width)
	{
		if (IsValid())
		{
			glDeleteTextures(1, &_texture);
		}
		GL_CHECK_ERROR();
		glGenTextures(1, &_texture);
		GL_CHECK_ERROR();
		glBindTexture(GL_TEXTURE_2D, _texture);
		GL_CHECK_ERROR();
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
		GL_CHECK_ERROR();
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		GL_CHECK_ERROR();
	}
	glBindTexture(GL_TEXTURE_2D, _texture);
	GL_CHECK_ERROR();

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
	GL_CHECK_ERROR();
}

GLFramebuffer::GLFramebuffer(cv::Size size) :
	_size(size), _fbo(-1), _texture(-1)
{
	GL_CHECK_ERROR();
	glGenFramebuffers(1, &_fbo);
	GL_CHECK_ERROR();
	glGenTextures(1, &_texture);
	GL_CHECK_ERROR();
	glBindTexture(GL_TEXTURE_2D, _texture);
	GL_CHECK_ERROR();
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _size.width, _size.height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	GL_CHECK_ERROR();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	GL_CHECK_ERROR();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	GL_CHECK_ERROR();
	Bind();
	GL_CHECK_ERROR();
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _texture, 0);
	GL_CHECK_ERROR();
}

GLFramebuffer::~GLFramebuffer()
{
	if(glIsTexture(_texture))
	{
		glDeleteTextures(1, &_texture);
	}
	if(glIsFramebuffer(_fbo))
	{
		glDeleteFramebuffers(1, &_fbo);
	}
}

unsigned int GLFramebuffer::GetFramebuffer()
{
	return _fbo;
}

unsigned int GLFramebuffer::GetTexture()
{
	return _texture;
}

const cv::Size& GLFramebuffer::GetSize()
{
	return _size;
}

bool GLFramebuffer::IsValid() const
{
	return (glIsFramebuffer(_fbo) && glIsTexture(_texture));
}

void GLFramebuffer::Bind()
{
	glBindFramebuffer(GL_FRAMEBUFFER, _fbo);
}

void GLFramebuffer::GetImage(cv::Mat& output)
{
	output.create(_size, CV_8UC3);
#ifdef _WIN32
	glBindFramebuffer(GL_READ_FRAMEBUFFER, _fbo);
#else
	glBindFramebuffer(GL_FRAMEBUFFER, _fbo);
#endif
	glReadPixels(0, 0, _size.width, _size.height, GL_RGB, GL_UNSIGNED_BYTE, output.ptr());
}

GLAccelerator::GLAccelerator(const std::string &shaderPath) :
	_window(0), _basePath(shaderPath)
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
	GL_CHECK_ERROR();
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
	glutSwapBuffers();
#else

#endif
#ifdef _WIN32
	glutMainLoopEvent();
#else

#endif
}

void GLAccelerator::ProcessFrame(const std::string &name, const cv::Mat &input)
{
	auto program = _programs.at(name);

	program->Process(input);

}

void GLAccelerator::ProcessFrame(const std::string& name, unsigned glTexture)
{
	auto program = _programs.at(name);

	program->Process(glTexture);
}

void GLAccelerator::CreateProgram(const std::string& name, cv::Size outputSize)
{
	_programs[name] = new GLProgram(this, name, outputSize);
}

void GLAccelerator::LoadVertexShader(const std::string& path)
{
	std::string src = ReadFile(path);
	SetVertexShader(src);
}

GLProgram::GLProgram(GLAccelerator* accelerator, const std::string& name, cv::Size outputSize) :
	_accelerator(accelerator), _name(name), _output(outputSize)
{
	LoadFragmentShader(name);
	LinkProgram();
}

GLProgram::~GLProgram()
{
	if(glIsShader(_fragmentShader))
	{
		glDeleteShader(_fragmentShader);
	}
	if(glIsProgram(_program))
	{
		glDeleteProgram(_program);
	}
}

void GLProgram::SetUniform(const std::string& name, int i)
{
	glUseProgram(_program);
	glUniform1i(GetUniformLoc(name), i);
}

void GLProgram::SetUniform(const std::string& name, float f)
{
	glUseProgram(_program);
	glUniform1f(GetUniformLoc(name), f);
}

void GLProgram::SetUniform(const std::string& name, const cv::Vec3f& x)
{
	glUseProgram(_program);
	glUniform3f(GetUniformLoc(name), x[0], x[1], x[2]);
}

void GLProgram::SetUniformColor(const std::string& name, const cv::Scalar& x)
{
	glUseProgram(_program);
	glUniform3f(GetUniformLoc(name), float(x[0]), float(x[1]), float(x[2]));
}

void GLProgram::Process(const cv::Mat& input)
{
	_input.ReadTexture(input);
	_output.Bind();
	cv::Size outputSize = _output.GetSize();
	glViewport(0, 0, outputSize.width, outputSize.height);

	glUseProgram(_program);
	int vertexLoc = GetAttributeLoc("a_vertex");
	int uvLoc = GetAttributeLoc("a_uv");
	_accelerator->BindVertexAttributes(vertexLoc, uvLoc);
	glActiveTexture(GL_TEXTURE0);
	_input.Bind();
	glUniform1i(GetUniformLoc("inputTexture"), 0);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glDisableVertexAttribArray(0);
}

void GLProgram::Process(unsigned glTexture)
{
	_output.Bind();
	cv::Size outputSize = _output.GetSize();
	glViewport(0, 0, outputSize.width, outputSize.height);

	glUseProgram(_program);
	int vertexLoc = GetAttributeLoc("a_vertex");
	int uvLoc = GetAttributeLoc("a_uv");
	_accelerator->BindVertexAttributes(vertexLoc, uvLoc);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, glTexture);
	glUniform1i(GetUniformLoc("inputTexture"), 0);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glDisableVertexAttribArray(0);
}

int GLProgram::GetUniformLoc(const std::string& name)
{
	auto locIt = _uniformLocs.find(name);
	if (locIt == _uniformLocs.end())
	{
		int loc = glGetUniformLocation(_program, name.c_str());
		_uniformLocs[name] = loc;
		if (_uniformLocs[name] < 0)
		{
			fprintf(stderr, std::string("Warning: can't find uniform " + name + " in shader program\n").c_str());
		}
		return loc;
	}
	return locIt->second;
}

int GLProgram::GetAttributeLoc(const std::string& name)
{
	auto locIt = _attributeLocs.find(name);
	if(locIt == _attributeLocs.end())
	{
		int loc = glGetAttribLocation(_program, name.c_str());
		_attributeLocs[name] = loc;
		if (_attributeLocs[name] < 0)
		{
			fprintf(stderr, std::string("Warning: can't find attrib " + name + " in shader program\n").c_str());
		}
		return loc;
	}
	return locIt->second;
}

void GLProgram::LoadFragmentShader(const std::string& name)
{
	std::string src = ReadFile(_accelerator->GetBaseShaderPath() + name + ".glsl");
	SetFragmentShader(src);
}

void GLAccelerator::SetVertexShader(const std::string &source)
{
	GL_CHECK_ERROR();
	_vertexShader = glCreateShader(GL_VERTEX_SHADER);
	if(!glIsShader(_vertexShader))
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
	GL_CHECK_ERROR();
}

void GLProgram::SetFragmentShader(const std::string &source)
{
	GL_CHECK_ERROR();
	_fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	if (!glIsShader(_fragmentShader))
	{
		std::stringstream ss;
		ss << "Error creating fragment shader: " << std::hex << (glGetError());
		throw ShaderProgramException(ss.str());
	}
	int length = (int)source.length();
	const char *sourceCstr = source.c_str();
	glShaderSource(_fragmentShader, 1, &sourceCstr, &length);
	GLenum error = glGetError();
	if (error != GL_NO_ERROR)
	{
		std::stringstream ss;
		ss << "Error assigning source to fragment shader: " << std::hex << (error);
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
	GL_CHECK_ERROR();
}

void GLProgram::LinkProgram()
{
	GL_CHECK_ERROR();
	_program = glCreateProgram();
	if(!glIsProgram(_program))
	{
		std::stringstream ss;
		ss << "Error creating program: " << std::hex << glGetError();
		throw ShaderProgramException(ss.str());
	}
	glAttachShader(_program, _accelerator->GetVertexShader());
	GLenum error = glGetError();
	if (error != GL_NO_ERROR)
	{
		std::stringstream ss;
		ss << "Error attaching vertex shader to program: " << std::hex << error;
		throw ShaderProgramException(ss.str());
	}
	glAttachShader(_program, _fragmentShader);
	error = glGetError();
	if (error != GL_NO_ERROR)
	{
		std::stringstream ss;
		ss << "Error attaching fragment shader to program: " << std::hex << error;
		throw ShaderProgramException(ss.str());
	}
	glLinkProgram(_program);
	GLint status;
	glGetProgramiv(_program, GL_LINK_STATUS, &status);
	if (status == GL_FALSE)
	{
		std::stringstream ss;
		ss << "Error linking program: " << std::hex << error;
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

	GL_CHECK_ERROR();
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
	GL_CHECK_ERROR();
	glGenBuffers(1, &_vbo);
	GL_CHECK_ERROR();
	glBindBuffer(GL_ARRAY_BUFFER, _vbo);
	GL_CHECK_ERROR();
	glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float), vertexData, GL_STATIC_DRAW);
	GL_CHECK_ERROR();
	glGenBuffers(1, &_uvbo);
	GL_CHECK_ERROR();
	glBindBuffer(GL_ARRAY_BUFFER, _uvbo);
	GL_CHECK_ERROR();
	glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float), uvData, GL_STATIC_DRAW);
	GL_CHECK_ERROR();
	LoadVertexShader(GetBaseShaderPath() + "vert.glsl");
	GL_CHECK_ERROR();
}

const std::string& GLAccelerator::GetBaseShaderPath()
{
	return _basePath;
}

unsigned int GLAccelerator::GetVertexShader()
{
	return _vertexShader;
}

void GLTexture::Bind()
{
	glBindTexture(GL_TEXTURE_2D, _texture);
}

void GLAccelerator::BindVertexAttributes(int vertexLoc, int uvLoc)
{
	glEnableVertexAttribArray(vertexLoc);
	glBindBuffer(GL_ARRAY_BUFFER, _vbo);
	glVertexAttribPointer(vertexLoc, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
	glEnableVertexAttribArray(uvLoc);
	glBindBuffer(GL_ARRAY_BUFFER, _uvbo);
	glVertexAttribPointer(uvLoc, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
}

void GLAccelerator::SetProgramUniformColor(const std::string& program, const std::string& uniform, const cv::Scalar &x)
{
	auto programPtr = _programs.at(program);

	programPtr->SetUniformColor(uniform, x);
}

void GLAccelerator::SetProgramUniform(const std::string& program, const std::string& uniform, const cv::Vec3f& x)
{
	auto programPtr = _programs.at(program);

	programPtr->SetUniform(uniform, x);
}

void GLAccelerator::SetProgramUniform(const std::string& program, const std::string& uniform, int x)
{
	auto programPtr = _programs.at(program);

	programPtr->SetUniform(uniform, x);
}

void GLAccelerator::SetProgramUniform(const std::string& program, const std::string& uniform, float x)
{
	auto programPtr = _programs.at(program);

	programPtr->SetUniform(uniform, x);
}

void GLAccelerator::GetResult(const std::string &name, cv::Mat& output)
{
	auto program = _programs.at(name);

	program->GetResult(output);
}

unsigned GLAccelerator::GetResultTexture(const std::string& name)
{
	auto program = _programs.at(name);

	return program->GetResultTexture();
}

void GLProgram::GetResult(cv::Mat& output)
{
	_output.GetImage(output);
}

unsigned int GLProgram::GetResultTexture()
{
	return _output.GetTexture();
}
