#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseWrappers)

set(DEPENDENCIES realsense2)

if (BUILD_PYTHON_BINDINGS OR BUILD_PYTHON_DOCS)
    if (NOT INTERNET_CONNECTION)
        message(WARNING "No internet connection. Cloning Python bindings may fail")
    endif()
    add_subdirectory(python)

endif()


if (BUILD_CV_EXAMPLES)
  add_subdirectory(opencv)
endif()

if (BUILD_DLIB_EXAMPLES)
  add_subdirectory(dlib)
endif()

if (BUILD_OPENVINO_EXAMPLES)
  add_subdirectory(openvino)
endif()

if (BUILD_OPEN3D_EXAMPLES)
  add_subdirectory(open3d)
endif()

if(BUILD_MATLAB_BINDINGS)
    add_subdirectory(matlab)
endif()

if (BUILD_PCL_EXAMPLES)
  add_subdirectory(pcl)
endif()

if(BUILD_UNITY_BINDINGS)
	if(NOT WIN32)
		message( FATAL_ERROR "BUILD_UNITY_BINDINGS is supported only for Windows OS" )
	endif()
	if(NOT BUILD_CSHARP_BINDINGS)
		message( WARNING "Enabling BUILD_CSHARP_BINDINGS and set DOTNET_VERSION_LIBRARY to 3.5 since BUILD_UNITY_BINDINGS is enabled" )
	endif()
	SET(BUILD_CSHARP_BINDINGS ON)
	SET(DOTNET_VERSION_LIBRARY 3.5)
endif()

if(BUILD_CSHARP_BINDINGS)
	if(NOT WIN32)
		message( WARNING "BUILD_CSHARP_BINDINGS is supported only for Windows OS" )
	endif()
	if(NOT BUILD_SHARED_LIBS)
		message( FATAL_ERROR "BUILD_SHARED_LIBS flag must be enabled when generating CSharp bindings" )
	endif()
    add_subdirectory(csharp)
endif()

if(BUILD_OPENNI2_BINDINGS)
	add_subdirectory(openni2)
endif()

if(BUILD_PC_STITCHING)
	add_subdirectory(pointcloud/pointcloud-stitching)
endif()

cmake_minimum_required(VERSION 3.1.0)

if(NOT WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
endif()

project(RealsensePCLExamples)

find_package( PCL REQUIRED )

if (PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  link_directories(${PCL_LIBRARY_DIRS})
endif()

list(APPEND DEPENDENCIES ${PCL_LIBRARIES})

find_package(OpenGL)
if(NOT OPENGL_FOUND)
    message(FATAL_ERROR "\n\n OpenGL package is missing!\n\n")
endif()

list(APPEND DEPENDENCIES ${OPENGL_LIBRARIES})

if(WIN32)
    list(APPEND DEPENDENCIES glfw3)
else()
    find_package(glfw3 REQUIRED)
    list(APPEND DEPENDENCIES glfw)
endif()

# add_subdirectory(pcl)
# add_subdirectory(pcl-color)

#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealSensePCLExample)

add_executable(rs-pcl rs-pcl.cpp)
set_property(TARGET rs-pcl PROPERTY CXX_STANDARD 11)
target_link_libraries(rs-pcl ${DEPENDENCIES})
if(NOT glfw3_DIR)
    target_include_directories(rs-pcl PUBLIC "${CMAKE_SOURCE_DIR}/third-party/glfw/include")
    target_link_directories(rs-pcl PUBLIC "${CMAKE_BINARY_DIR}/third-party/glfw/src/${CMAKE_BUILD_TYPE}")
endif()
set_target_properties (rs-pcl PROPERTIES
    FOLDER "Examples/PCL"
)

foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO
        CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
    if(${flag_var} MATCHES "/MT")
        string(REGEX REPLACE "/MT" "/MD" ${flag_var} "${${flag_var}}")
    endif(${flag_var} MATCHES "/MT")
endforeach(flag_var)

install(
    TARGETS

    rs-pcl

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)
