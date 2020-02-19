cmake_minimum_required(VERSION 3.12)
# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set the project name and version
project(StepGenerator VERSION 1.0)

add_executable(test_planner 
    lib/planner/trj_jointss.cpp
    lib/planner/trj_planner.cpp
    lib/planner/trj_segment.cpp
    lib/planner/trj_util.cpp
    test/test_planner/main.cpp
    lib/Unity/src/unity.c
    )

target_include_directories(test_planner PUBLIC 
    lib/planner/
    ${CMAKE_CURRENT_LIST_DIR} 
    lib/Unity/src
    lib/FastCRC/src
    arduino_include )
                           
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
IF(DEFINED CMAKE_BUILD_TYPE AND CMAKE_VERSION VERSION_GREATER "2.8")
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS  ${CMAKE_CONFIGURATION_TYPES})
ENDIF()