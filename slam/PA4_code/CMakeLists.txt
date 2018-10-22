cmake_minimum_required (VERSION 2.8)

# projectname is the same as the main-executable
project(PA4_code)

add_definitions('-g')
add_definitions('-Wall')
#add_definitions('-std=c++11')
#SET(OpenCV_DIR /usr/share/OpenCV)
find_package(OpenCV REQUIRED)
include_directories(
	${OpenCV_INCLUDE_DIRS}
)

add_executable(undistort_image undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBS})
