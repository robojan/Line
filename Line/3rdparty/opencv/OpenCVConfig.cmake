# ===================================================================================
#  The OpenCV CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(OpenCV REQUIRED)
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${OpenCV_LIBS})
#
#    Or you can search for specific OpenCV modules:
#
#    FIND_PACKAGE(OpenCV REQUIRED core imgcodecs)
#
#    If the module is found then OPENCV_<MODULE>_FOUND is set to TRUE.
#
#    This file will define the following variables:
#      - OpenCV_LIBS                     : The list of libraries to link against.
#      - OpenCV_INCLUDE_DIRS             : The OpenCV include directories.
#      - OpenCV_COMPUTE_CAPABILITIES     : The version of compute capability
#      - OpenCV_VERSION                  : The version of this OpenCV build: "3.2.0"
#      - OpenCV_VERSION_MAJOR            : Major version part of OpenCV_VERSION: "3"
#      - OpenCV_VERSION_MINOR            : Minor version part of OpenCV_VERSION: "2"
#      - OpenCV_VERSION_PATCH            : Patch version part of OpenCV_VERSION: "0"
#      - OpenCV_VERSION_STATUS           : Development status of this build: "-dev"
#
#    Advanced variables:
#      - OpenCV_SHARED
#
# ===================================================================================
#
#    Windows pack specific options:
#      - OpenCV_STATIC
#      - OpenCV_CUDA

if(CMAKE_VERSION VERSION_GREATER 2.6)
  get_property(OpenCV_LANGUAGES GLOBAL PROPERTY ENABLED_LANGUAGES)
  if(NOT ";${OpenCV_LANGUAGES};" MATCHES ";CXX;")
    enable_language(CXX)
  endif()
endif()

if(NOT DEFINED OpenCV_STATIC)
  # look for global setting
  if(BUILD_SHARED_LIBS)
    set(OpenCV_STATIC OFF)
  else()
    set(OpenCV_STATIC ON)
  endif()
endif()

if(NOT DEFINED OpenCV_CUDA)
  # if user' app uses CUDA, then it probably wants CUDA-enabled OpenCV binaries
  if(CUDA_FOUND)
    set(OpenCV_CUDA ON)
  endif()
endif()

if(MSVC)
  if(CMAKE_CL_64)
    set(OpenCV_ARCH x64)
  elseif((CMAKE_GENERATOR MATCHES "ARM") OR ("${arch_hint}" STREQUAL "ARM") OR (CMAKE_VS_EFFECTIVE_PLATFORMS MATCHES "ARM|arm"))
    # see Modules/CmakeGenericSystem.cmake
    set(OpenCV_ARCH ARM)
  else()
    set(OpenCV_ARCH x86)
  endif()
  if(MSVC_VERSION EQUAL 1400)
    set(OpenCV_RUNTIME vc8)
  elseif(MSVC_VERSION EQUAL 1500)
    set(OpenCV_RUNTIME vc9)
  elseif(MSVC_VERSION EQUAL 1600)
    set(OpenCV_RUNTIME vc10)
  elseif(MSVC_VERSION EQUAL 1700)
    set(OpenCV_RUNTIME vc11)
  elseif(MSVC_VERSION EQUAL 1800)
    set(OpenCV_RUNTIME vc12)
  elseif(MSVC_VERSION EQUAL 1900)
    set(OpenCV_RUNTIME vc14)
  elseif(MSVC_VERSION EQUAL 1910)
    set(OpenCV_RUNTIME vc15)
  endif()
elseif(MINGW)
  set(OpenCV_RUNTIME mingw)

  execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpmachine
                  OUTPUT_VARIABLE OPENCV_GCC_TARGET_MACHINE
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(OPENCV_GCC_TARGET_MACHINE MATCHES "amd64|x86_64|AMD64")
    set(MINGW64 1)
    set(OpenCV_ARCH x64)
  else()
    set(OpenCV_ARCH x86)
  endif()
endif()

if(NOT OpenCV_FIND_QUIETLY)
  message(STATUS "OpenCV ARCH: ${OpenCV_ARCH}")
  message(STATUS "OpenCV RUNTIME: ${OpenCV_RUNTIME}")
  message(STATUS "OpenCV STATIC: ${OpenCV_STATIC}")
endif()

get_filename_component(OpenCV_CONFIG_PATH "${CMAKE_CURRENT_LIST_FILE}" PATH)
if(OpenCV_RUNTIME AND OpenCV_ARCH)
  if(OpenCV_STATIC AND EXISTS "${OpenCV_CONFIG_PATH}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/staticlib/OpenCVConfig.cmake")
    if(OpenCV_CUDA AND EXISTS "${OpenCV_CONFIG_PATH}/gpu/${OpenCV_ARCH}/${OpenCV_RUNTIME}/staticlib/OpenCVConfig.cmake")
      set(OpenCV_LIB_PATH "${OpenCV_CONFIG_PATH}/gpu/${OpenCV_ARCH}/${OpenCV_RUNTIME}/staticlib")
    else()
      set(OpenCV_LIB_PATH "${OpenCV_CONFIG_PATH}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/staticlib")
    endif()
  elseif(EXISTS "${OpenCV_CONFIG_PATH}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/lib/OpenCVConfig.cmake")
    if(OpenCV_CUDA AND EXISTS "${OpenCV_CONFIG_PATH}/gpu/${OpenCV_ARCH}/${OpenCV_RUNTIME}/lib/OpenCVConfig.cmake")
      set(OpenCV_LIB_PATH "${OpenCV_CONFIG_PATH}/gpu/${OpenCV_ARCH}/${OpenCV_RUNTIME}/lib")
    else()
      set(OpenCV_LIB_PATH "${OpenCV_CONFIG_PATH}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/lib")
    endif()
  endif()
endif()

if(OpenCV_LIB_PATH AND EXISTS "${OpenCV_LIB_PATH}/OpenCVConfig.cmake")
  include("${OpenCV_LIB_PATH}/OpenCVConfig.cmake")

  if(NOT OpenCV_FIND_QUIETLY)
    message(STATUS "Found OpenCV ${OpenCV_VERSION} in ${OpenCV_LIB_PATH}")
    if(NOT OpenCV_LIB_PATH MATCHES "/staticlib")
      get_filename_component(_OpenCV_LIB_PATH "${OpenCV_LIB_PATH}/../bin" ABSOLUTE)
      file(TO_NATIVE_PATH "${_OpenCV_LIB_PATH}" _OpenCV_LIB_PATH)
      message(STATUS "You might need to add ${_OpenCV_LIB_PATH} to your PATH to be able to run your applications.")
      if(OpenCV_LIB_PATH MATCHES "/gpu/")
        string(REPLACE "\\gpu" "" _OpenCV_LIB_PATH2 "${_OpenCV_LIB_PATH}")
        message(STATUS "GPU support is enabled so you might also need ${_OpenCV_LIB_PATH2} in your PATH (it must go after the ${_OpenCV_LIB_PATH}).")
      endif()
    endif()
  endif()
else()
  if(NOT OpenCV_FIND_QUIETLY)
    message(WARNING
"Found OpenCV Windows Pack but it has no binaries compatible with your configuration.
You should manually point CMake variable OpenCV_DIR to your build of OpenCV library."
    )
  endif()
  set(OpenCV_FOUND FALSE)
endif()
