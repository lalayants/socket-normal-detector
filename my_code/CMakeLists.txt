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
