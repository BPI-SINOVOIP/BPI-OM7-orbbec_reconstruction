cmake_minimum_required(VERSION 3.16)

project(orbbec_reconstruction LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTING "Build tests" ON)

find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED CONFIG)

set(ORBBECSDK_ROOT "" CACHE PATH "OrbbecSDK root directory (contains include/ and lib/)")
if(NOT ORBBECSDK_ROOT AND DEFINED ENV{ORBBECSDK_ROOT})
    set(ORBBECSDK_ROOT "$ENV{ORBBECSDK_ROOT}")
endif()

set(_ORBBEC_HINTS
    ${ORBBECSDK_ROOT}
    /opt/OrbbecSDK
    /opt/OrbbecSDK_v2.7.6
    /usr/local
)

find_path(ORBBECSDK_INCLUDE_DIR
    NAMES libobsensor/ObSensor.hpp
    PATH_SUFFIXES include
    PATHS ${_ORBBEC_HINTS}
)

find_library(ORBBECSDK_LIBRARY
    NAMES OrbbecSDK libOrbbecSDK
    PATH_SUFFIXES lib lib64
    PATHS ${_ORBBEC_HINTS}
)

if(NOT ORBBECSDK_INCLUDE_DIR OR NOT ORBBECSDK_LIBRARY)
    message(FATAL_ERROR "OrbbecSDK not found. Set -DORBBECSDK_ROOT=<path> or export ORBBECSDK_ROOT.")
endif()

message(STATUS "Current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")


add_executable(cloud_point_visualization
    src/cloud_point_visualization.cpp
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.1)
    target_compile_options(cloud_point_visualization PRIVATE -Wno-psabi)
endif()


target_include_directories(cloud_point_visualization 
    PRIVATE
        ${ORBBECSDK_INCLUDE_DIR}               
        ${CMAKE_CURRENT_SOURCE_DIR}/include        
)

target_link_libraries(cloud_point_visualization 
    PRIVATE
        Open3D::Open3D
        ${ORBBECSDK_LIBRARY}
        Eigen3::Eigen
)


add_executable(sample_data
    src/sample_data.cpp
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.1)
    target_compile_options(sample_data PRIVATE -Wno-psabi)
endif()
target_include_directories(sample_data
    PRIVATE
        ${ORBBECSDK_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(sample_data
    PRIVATE
        Open3D::Open3D
        ${ORBBECSDK_LIBRARY}
        Eigen3::Eigen
)


add_executable(filter_isolated_clouds
    src/filter_isolated_clouds.cpp
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.1)
    target_compile_options(filter_isolated_clouds PRIVATE -Wno-psabi)
endif()
target_include_directories(filter_isolated_clouds
    PRIVATE
        ${ORBBECSDK_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(filter_isolated_clouds
    PRIVATE
        Open3D::Open3D
        ${ORBBECSDK_LIBRARY}
        Eigen3::Eigen
)

add_executable(filter_object
    src/filter_object.cpp
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.1)
    target_compile_options(filter_object PRIVATE -Wno-psabi)
endif()
target_include_directories(filter_object
    PRIVATE
        ${ORBBECSDK_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(filter_object
    PRIVATE
        Open3D::Open3D
        ${ORBBECSDK_LIBRARY}
        Eigen3::Eigen
)

add_executable(main
    src/main.cpp
    src/ransac_registration.cpp
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.1)
    target_compile_options(main PRIVATE -Wno-psabi)
endif()
target_include_directories(main
    PRIVATE
        ${ORBBECSDK_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(main
    PRIVATE
        Open3D::Open3D
        ${ORBBECSDK_LIBRARY}
        Eigen3::Eigen
)
