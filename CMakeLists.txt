cmake_minimum_required(VERSION 2.8.3)
project(mjpeg_server)
include(ExternalProject)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib cv_bridge image_transport)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenSSL)

ExternalProject_Add(cpp-netlib
  PREFIX cpp-netlib
  GIT_REPOSITORY https://github.com/cpp-netlib/cpp-netlib.git
  GIT_TAG 0.11-devel
  BUILD_IN_SOURCE
  BUILD_COMMAND cmake --build .
  INSTALL_COMMAND ""
)
# Extract include directory
set(SOURCE_DIR_OLD ${SOURCE_DIR})
ExternalProject_Get_Property(cpp-netlib SOURCE_DIR)
set(CPPNETLIB_INCLUDE_DIRS ${SOURCE_DIR})
set(SOURCE_DIR ${SOURCE_DIR_OLD})

# Extract binary directory
set(BINARY_DIR_OLD ${BINARY_DIR})
ExternalProject_Get_Property(cpp-netlib BINARY_DIR)
set(CPPNETLIB_BINARY_DIR ${BINARY_DIR})
set(BINARY_DIR ${BINARY_DIR_OLD})

set(CPPNETLIB_LIBRARIES
  ${CPPNETLIB_BINARY_DIR}/libs/network/src/libcppnetlib-uri.a
  ${CPPNETLIB_BINARY_DIR}/libs/network/src/libcppnetlib-server-parsers.a
  ${CPPNETLIB_BINARY_DIR}/libs/network/src/libcppnetlib-client-connections.a)


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########




## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CPPNETLIB_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/mjpeg_server.cpp)
add_dependencies(${PROJECT_NAME} cpp-netlib)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CPPNETLIB_LIBRARIES} ${OPENSSL_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
