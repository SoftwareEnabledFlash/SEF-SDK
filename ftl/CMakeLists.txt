########################################################
#
# Project Options
#

option(sef_ftl_coverage "Build code with coverage enabled build type" OFF)

option(sef_sanitizer "Build code with sanitizer enabled build type" OFF)

option(sef_thread_sanitizer "Build code with thread sanitizer enabled and debug build type" OFF)

option(sef_test_manual_copy "Build tests using manual copy instead of nameless copy" OFF)

option(sef_extra_warnings "Turn on -Wextra" OFF)

########################################################
#
# Project Settings
#

cmake_minimum_required(VERSION 3.8)

project (
    sef-ftl
    VERSION 1.0
    LANGUAGES CXX C
)

cmake_minimum_required(VERSION 3.8)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

# C version
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set CMake Path
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# build name extention
set(PROJECT_NAME_EXTENTION "")
set(DEPENDENCY_NAME_EXTENTION "")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (sef_extra_warnings)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra ")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra ")
endif()

########################################################
#
# Code Prep
#

if(PROJECT_IS_TOP_LEVEL)
    include(../cmake/sdkVersion.cmake)
    sdk_version(sef_sdk_version "${CMAKE_SOURCE_DIR}/..")
    add_definitions(-DSEFSDKVersion="${PROJECT_NAME}:${sef_sdk_version}")
endif()

########################################################
#
# Option Based Changes
#

# check for manual copy test argument
if (sef_test_manual_copy)
    set(PROJECT_NAME_EXTENTION "${PROJECT_NAME_EXTENTION}c")
    add_definitions(-DManualCopyTest)
endif()

# check for code coverage argument
if(sef_ftl_coverage)
    if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
    elseif(${CMAKE_C_COMPILER_ID} STREQUAL "Clang")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-instr-generate -fcoverage-mapping")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-instr-generate -fcoverage-mapping")
    endif(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
endif()

# check for sanitizer argument
if (sef_sanitizer)
    set(PROJECT_NAME_EXTENTION "${PROJECT_NAME_EXTENTION}s")
    set(DEPENDENCY_NAME_EXTENTION "${DEPENDENCY_NAME_EXTENTION}s")

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fsanitize=undefined -fsanitize=leak")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize=undefined -fsanitize=leak")
endif()

# check for thread sanitizer argument
if (sef_thread_sanitizer)
    set(PROJECT_NAME_EXTENTION "${PROJECT_NAME_EXTENTION}m")
    set(DEPENDENCY_NAME_EXTENTION "${DEPENDENCY_NAME_EXTENTION}m")

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=thread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
endif()


# ignore deprecated warnings if debug
# and add rdynamic for error injection
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -rdynamic")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -rdynamic")
endif(CMAKE_BUILD_TYPE MATCHES Debug)

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

add_subdirectory(src)

