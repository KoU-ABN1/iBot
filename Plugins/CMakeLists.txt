cmake_minimum_required(VERSION 3.15)
project(iBot_v1)

include_directories(include)
# include_directories(/home/hao/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04/programming/include)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED)

if(NOT LIBPLUGIN_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
    else()
        set(LIBPLUGIN_DIR /home/hao/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04/programming/libPlugin)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${LIBPLUGIN_DIR}/cmake)
find_package(CoppeliaSim 4.2.0.0 REQUIRED)

coppeliasim_add_plugin(
    iBot_v1
    LEGACY
    SOURCES
    ./src/sim_plugin.cpp
    ./src/sim_main.cpp
    ./src/sim_data.cpp
    ${COPPELIASIM_COMMON_DIR}/scriptFunctionData.cpp
    ${COPPELIASIM_COMMON_DIR}/scriptFunctionDataItem.cpp
)