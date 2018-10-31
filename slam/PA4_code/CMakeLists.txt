cmake_minimum_required (VERSION 2.8)

# projectname is the same as the main-executable
project(PA4_code)

add_definitions('-g')
add_definitions('-Wall')
#add_definitions('-std=c++11')
#SET(OpenCV_DIR /usr/share/OpenCV)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
${Sophus_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

add_executable(undistort_image undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBS})
add_executable(distortion_fun distortion_fun.cpp)
add_executable(disparity disparity.cpp)
target_link_libraries(disparity ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${EIGEN3_LIBS})
add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries(gaussnewton ${EIGEN3_LIBS} ${OpenCV_LIBS} ${Sophus_LIBRARIES})
