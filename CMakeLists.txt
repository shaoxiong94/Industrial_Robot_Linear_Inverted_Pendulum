cmake_minimum_required(VERSION 2.8)
project(Industrial_Robot_Linear_Inverted_Pendulum)

find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)


#########
#########
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

#########
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror


#########
#########
FILE(GLOB_RECURSE readme ./*.md*  ./*.gitignore*)
FILE(GLOB HEAD_FILES "./include/*.h")
FILE(GLOB SRC_FILES "./src/*.cpp" )
add_custom_target(whatever SOURCES ${readme} ${HEAD_FILES} ${SRC_FILES})  ## blog.csdn.net/seamanj/article/details/80410457


#add_executable(${PROJECT_NAME} "src/main.cpp")


add_executable(${PROJECT_NAME} ${HEAD_FILES} ${SRC_FILES})
