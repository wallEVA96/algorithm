cmake_minimum_required (VERSION 2.8)

# projectname is the same as the main-executable
project(PA5_code)

add_definitions('-g')
add_definitions('-Wall')
add_definitions('-std=c++11')

find_package(OpenCV REQUIRED)
include_directories(
${OpenCV_INCLUDE_DIRS}
)
add_executable(computeORB computeORB.cpp)
target_link_libraries(computeORB ${OpenCV_LIBS})
