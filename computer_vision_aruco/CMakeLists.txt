cmake_minimum_required(VERSION 3.19)

project(OmniwheelAruco VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(OpenCV_DIR "C:/visp-ws/opencv-4.12.0/build-vc18")
find_package(OpenCV REQUIRED)

set(VISP_DIR "C:/visp-ws/visp-build-vc18")
find_package(VISP REQUIRED)
include(${VISP_USE_FILE})

add_executable(omni_robot main.cpp OneEuroFilter.cpp)

target_include_directories(omni_robot PRIVATE ${VISP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_link_libraries(omni_robot PRIVATE ${VISP_LIBRARIES} ${OpenCV_LIBS})