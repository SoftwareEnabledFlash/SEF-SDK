########################################################
#
# Project Options
#

option(sef_lib_coverage "Build code with coverage enabled build type" OFF)

option(sef_sanitizer "Build code with sanitizer enabled build type" OFF)

option(sef_thread_sanitizer "Build code with thread sanitizer enabled and debug build type" OFF)

option(sef_enable_simulator "Build the code against the simulator; will build against Emulator if ON" OFF)

option(sef_lib_unit_test "Build the unit tests" OFF)

option(sef_debug_log "Add Define -> SEFLIB_DEBUG_LOG" OFF)

option(sef_debug_out_file "Add Define -> SEFLIB_DEBUG_OUT_FILE_ENABLE" OFF)

option(sef_extra_warnings "Turn on -Wextra" OFF)

########################################################
#
# Project Settings
#

cmake_minimum_required(VERSION 3.8)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

project (
    sef-lib
    VERSION 1.0
    LANGUAGES CXX C
)

# C version
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_GNU_SOURCE)

# set CMake Path
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# build name extention
set(PROJECT_NAME_EXTENTION "")

# generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

########################################################
#
# Code Prep
#

#set sef_sdk_version
if(PROJECT_IS_TOP_LEVEL)
    include(../cmake/sdkVersion.cmake)
    sdk_version(sef_sdk_version "${CMAKE_SOURCE_DIR}/..")
    add_definitions(-DSEFSDKVersion="${PROJECT_NAME}:${sef_sdk_version}")
endif()

########################################################
#
# Option Based Changes
#

# check for code coverage argument
if(sef_lib_coverage)
    if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
    elseif(${CMAKE_C_COMPILER_ID} STREQUAL "Clang")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-instr-generate -fcoverage-mapping")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-instr-generate -fcoverage-mapping")
    endif(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
    add_definitions(-DENV_COVERAGE)
endif()

# check for sanitizer argument
if (sef_sanitizer)
    set(PROJECT_NAME_EXTENTION "${PROJECT_NAME_EXTENTION}s")

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fsanitize=undefined -fsanitize=leak")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize=undefined -fsanitize=leak")
endif()

# check for thread sanitizer argument
if (sef_thread_sanitizer)
    set(PROJECT_NAME_EXTENTION "${PROJECT_NAME_EXTENTION}m")

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=thread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
endif()

# simulator based or libsef based
if (sef_enable_simulator)
    add_definitions(-DENV_SIMULATOR)
endif()

# itest - tests for emulator or fpga
if (sef_enable_emulator)
    add_definitions(-DENV_EMULATOR)
endif()

# enable debug logs
if(sef_debug_log)
    add_definitions(-DSEFLIB_DEBUG_LOG)
endif()

# enable debug logs to go to file
if(sef_debug_out_file)
    add_definitions(-DSEFLIB_DEBUG_OUT_FILE_ENABLE)
endif()

if (sef_extra_warnings)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra ")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra ")
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-nonnull-compare -Wno-nonnull")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wno-nonnull-compare -Wno-nonnull")
endif()


########################################################
#
# Print Info
#

message("CMAKE_BUILD_TYPE        : " ${CMAKE_BUILD_TYPE})
message("CMAKE_C_COMPILER_ID     : " ${CMAKE_C_COMPILER_ID})
message("CMAKE_CXX_COMPILER_ID   : " ${CMAKE_CXX_COMPILER_ID})
message("CMAKE_C_FLAGS           : " ${CMAKE_C_FLAGS})
message("CMAKE_C_FLAGS_DEBUG     : " ${CMAKE_C_FLAGS_DEBUG})
message("CMAKE_C_FLAGS_RELEASE   : " ${CMAKE_C_FLAGS_RELEASE})
message("CMAKE_CXX_FLAGS         : " ${CMAKE_CXX_FLAGS})
message("CMAKE_CXX_FLAGS_DEBUG   : " ${CMAKE_CXX_FLAGS_DEBUG})
message("CMAKE_CXX_FLAGS_RELEASE : " ${CMAKE_CXX_FLAGS_RELEASE})

########################################################
#
# Run Sub Directories
#

include_directories(${PROJECT_SOURCE_DIR}/../api)
include_directories(${PROJECT_SOURCE_DIR}/src/include)

if (NOT sef_enable_simulator)
    add_subdirectory(src/lib)
endif()
if (sef_lib_unit_test)
    add_subdirectory(test/ut/lib)
    add_subdirectory(test/it)
    if(sef_enable_simulator)
        message("Lib Driver does not support simulator based library")
    endif()
endif()
