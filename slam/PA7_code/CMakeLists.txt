cmake_minimum_required (VERSION 2.8)

# projectname is the same as the main-executable
project(PA7_code)

add_definitions('-g')
add_definitions('-Wall')
add_definitions('-std=c++11')

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)
find_package(OpenCV 3.4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${Sophus_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
)

add_executable(directBA directBA.cpp)
target_link_libraries(directBA ${OpenCV_LIBS} ${Pangolin_LIBS} ${EIGEN3_LIBS})
