cmake_minimum_required (VERSION 2.8)

# projectname is the same as the main-executable
project(useEigen)

add_definitions('-g')
add_definitions('-Wall')
add_definitions('-std=c++11')

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(G2O REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
	${G2O_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR}
)

message(STATUS ${EIGEN3_INCLUDE_DIR})
message(STATUS ${G2O_INCLUDE_DIR})
message(STATUS ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
add_executable(useG2O useG2O.cpp)
target_link_libraries(useG2O ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
add_executable(geometry geometry.cpp)
add_executable(c++11 c++11.cpp)

