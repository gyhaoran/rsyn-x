# Rsyn
# CMake Examples: https://github.com/ttroy50/cmake-examples

#########
# Setup #
#########

# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.5)

# Message
message(STATUS "Rsyn")
message(STATUS ${CMAKE_CURRENT_SOURCE_DIR})

# Options
option(ISPD18 "Create an executable for ISPD18 Contest" OFF)

# Find includes in the build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Turn on automatic invocation of the MOC
set(CMAKE_AUTOMOC ON)

# Project's name
project(rsyn)

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(WINDOWS TRUE)
elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(LINUX TRUE)
elseif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(MACOSX TRUE)
endif()

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/x/cmake)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/release")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

set_property(SOURCE file.hh PROPERTY SKIP_AUTOMOC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS False)
message(STATUS "CXX Standard: ${CMAKE_CXX_STANDARD}")

set(PATH_RSYN ${CMAKE_CURRENT_SOURCE_DIR}/rsyn)
set(PATH_X ${CMAKE_CURRENT_SOURCE_DIR}/x)

set(PATH_QT ${CMAKE_CURRENT_SOURCE_DIR}/rsyn/src/rsyn/qt)

if(NOT DEFINED PKGS_PATH)
    set(PKGS_PATH /home/usergrp/data/pkgs)
endif()

message(STATUS "PKGS_PATH: ${PKGS_PATH}")

set(BOOST_PATH ${PKGS_PATH}/boost)
set(QT5_PATH ${PKGS_PATH}/qt/qt5)
set(LIBPNG_PATH ${PKGS_PATH}/libpng)

list(APPEND CMAKE_PREFIX_PATH ${BOOST_PATH})
list(APPEND CMAKE_PREFIX_PATH ${QT5_PATH})
list(APPEND CMAKE_PREFIX_PATH ${LIBPNG_PATH})

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

###################
# Warnings/Errors #
###################

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Werror=return-type -Wno-deprecated-declarations -Wno-register -Wno-unused-result")

######################
# Check Dependencies #
######################
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
#find_package(OpenGL REQUIRED)
find_package(PNG REQUIRED)
find_package(COIN)
find_package(GLPK)

# OpenMP (not working)
#find_package(OpenMP)
#if(OPENMP_FOUND)
#  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()

# OpenMP (work around)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fopenmp")

find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
# find_package(Qt5OpenGL REQUIRED)

#find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

###############
# Source Code #
###############
file(GLOB_RECURSE SRC_FILES_RSYN 
    ${PATH_RSYN}/src/*.cpp 
    ${PATH_RSYN}/src/*.cc 
    ${PATH_RSYN}/src/*.c)

file(GLOB_RECURSE SRC_FILES_X
    ${PATH_X}/src/*.cpp 
    ${PATH_X}/src/*.cc 
    ${PATH_X}/src/*.c)

set(SRC_FILES ${SRC_FILES_RSYN} ${SRC_FILES_X})

if(NOT COIN_FOUND OR NOT GLPK_FOUND)
  list(REMOVE_ITEM SRC_FILES ${PATH_X}/src/x/opto/example/LemonLP.h)
  list(REMOVE_ITEM SRC_FILES ${PATH_X}/src/x/opto/example/LemonLP.cpp)
  list(REMOVE_ITEM SRC_FILES ${PATH_X}/src/x/opto/ufrgs/qpdp/OverlapRemover.h)
  list(REMOVE_ITEM SRC_FILES ${PATH_X}/src/x/opto/ufrgs/qpdp/OverlapRemover.cpp)
endif()

if(COIN_FOUND)
  add_definitions(-DRSYN_COIN_FOUND=1)
endif()

if(GLPK_FOUND)
  add_definitions(-DRSYN_GLPK_FOUND=1)
endif()

#####################
# Qt Pre-Processing #
#####################

file(GLOB_RECURSE UI_FILES
    ${PATH_QT}/ui/*.ui)

file(GLOB_RECURSE QRC_FILES
    ${PATH_QT}/resource/*.qrc)

# Generate code from ui files
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

# Generate rules for building source files from the resources
qt5_add_resources(QRCS ${QRC_FILES})

# Tell CMake to create the executable
list(APPEND SRC_FILES ${UI_HEADERS})
list(APPEND SRC_FILES ${QRCS})
list(APPEND SRC_FILES ${GUI_FILES})

#message(STATUS ${SRC_FILES})

#################
# Library Paths #
#################
# Need to come before target is created.

if(LINUX)
link_directories(${PATH_RSYN}/lib/linux)
link_directories(${PATH_X}/lib/linux)
endif()

if(MACOSX)
link_directories(${PATH_RSYN}/lib/macosx)
link_directories(${PATH_X}/lib/macosx)
endif()

###########
# Targets #
###########

add_executable(rsyn ${SRC_FILES})

#######################
# Include Directories #
#######################

target_include_directories(rsyn PUBLIC "${PATH_RSYN}/src/rsyn/export")

target_include_directories(rsyn PUBLIC "${PATH_RSYN}/src")
target_include_directories(rsyn PUBLIC "${PATH_RSYN}/include")
target_include_directories(rsyn PUBLIC "${PATH_X}/src")
target_include_directories(rsyn PUBLIC "${PATH_X}/include")

##################
# Compiler Flags #
##################

target_compile_options(rsyn PUBLIC "-O3")
target_compile_options(rsyn PUBLIC "-Wno-unused-result -Wno-format -Wno-return-type -Wno-unused-local-typedefs")

if(${ISPD18_BIN})
message(STATUS "ISPD18 binary will be generated")
add_definitions(-DISPD18_BIN=1)
endif()

#################
# Linker Flags  #
#################

target_link_libraries(rsyn lef)
target_link_libraries(rsyn def)
target_link_libraries(rsyn emon) # not a typo

# OpenGL
#include_directories(${OPENGL_INCLUDE_DIR})
#target_link_libraries(rsyn ${OPENGL_gl_LIBRARY})
#target_link_libraries(rsyn ${OPENGL_glu_LIBRARY})

# Boost   
target_include_directories(rsyn PUBLIC ${Boost_INCLUDE_DIR})
target_link_libraries(rsyn ${Boost_LIBRARIES})
    
# PNG
target_include_directories(rsyn PUBLIC ${PNG_INCLUDE_DIR})
target_link_libraries(rsyn ${PNG_LIBRARY})

# COIN
if(COIN_FOUND)
  target_include_directories(rsyn PUBLIC ${COIN_INCLUDE_DIR})
  target_link_libraries(rsyn ${COIN_LIBRARIES})
endif()

# GLPK
if(GLPK_FOUND)
  target_link_libraries(rsyn glpk)
endif()

# Qt5 Widgets for linking
target_link_libraries(rsyn Qt5::Widgets)
target_link_libraries(rsyn Qt5::PrintSupport)
# target_link_libraries(rsyn Qt5::OpenGL)

############
# Options  #
############
if(ISPD18)
  message("ISPD18 mode is on.")
  target_compile_options(rsyn PUBLIC "-DISPD18_BIN")
endif()

# Install rules       
install(TARGETS rsyn
    DESTINATION rsyn/bin
)

install(FILES x/demo/simple/pin-assess.rsyn x/demo/simple/macro-cost.rsyn x/demo/simple/pac.lef x/demo/simple/asap7sc7p5t_28_R_1x_220121a.lef
    DESTINATION ${CMAKE_INSTALL_PREFIX}/rsyn/data
)

set_target_properties(rsyn PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib;$ORIGIN/../lib64"
)