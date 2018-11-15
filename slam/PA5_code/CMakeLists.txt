cmake_minimum_required (VERSION 2.8)

# projectname is the same as the main-executable
project(PA5_code)

add_definitions('-g')
add_definitions('-Wall')
add_definitions('-std=c++11')

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
${Sophus_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
)

add_executable(computeORB computeORB.cpp)
target_link_libraries(computeORB ${OpenCV_LIBS})
add_executable(E2Rt E2Rt.cpp)
target_link_libraries(E2Rt ${Sophus_LIBRARIES})
