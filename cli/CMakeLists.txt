########################################################
#
# Project Options
#

option(sef_sanitizer "Build code with sanitizer enabled build type" OFF)

option(sef_cli_coverage "Build code with coverage enabled build type" OFF)

option(sef_thread_sanitizer "Build code with thread sanitizer enabled build type" OFF)

option(sef_enable_simulator "Build the code against the simulator; will build against FPGA if OFF" OFF)

option(sef_enable_devtool "Add DevTools, used to generate supporting help material, to the build" OFF)

option(sef_cli_unit_test "Build the unit tests" OFF)

option(sef_python_version "Build code with specific python verison" OFF)

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
    sef-cli
    VERSION 2.7
    LANGUAGES CXX C
)

# C version
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# build name extention
set(PROJECT_NAME_EXTENTION "")
set(DEPENDENCY_NAME_EXTENTION "")

if (sef_extra_warnings)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra ")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra ")
endif()


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

if (sef_python_version)
    set(PYTHON_VERSION ${sef_python_version})
    include_directories(/usr/include/${PYTHON_VERSION})
else()
    find_package(Python3 COMPONENTS Development)
    if (Python3_FOUND)
        message("Python_Development_FOUND")
        set(PYTHON_VERSION ${Python3_LIBRARIES})
        include_directories(${Python3_INCLUDE_DIRS})
    else()
        set(PYTHON_VERSION python3.6m)
        include_directories(/usr/include/${PYTHON_VERSION})
    endif()
endif()



########################################################
#
# Option Based Changes (Debug Only)
#

# check for code coverage argument
if(sef_cli_coverage)
    # The coverage dependencies are not needed and perform slower
    #set(DEPENDENCY_NAME_EXTENTION "${DEPENDENCY_NAME_EXTENTION}c")

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
    if (Python_LIBRARIES)
        get_filename_component(PYTHON_LIB_FILENAME "${Python_LIBRARIES}" NAME)
        write_file(../suppress_sanitizer.conf "leak:/${PYTHON_LIB_FILENAME}")
    else()
        write_file(../suppress_sanitizer.conf "leak:/lib${PYTHON_VERSION}.so")
    endif()
endif()

# check for thread sanitizer argument
if (sef_thread_sanitizer)
    set(PROJECT_NAME_EXTENTION "${PROJECT_NAME_EXTENTION}m")
    set(DEPENDENCY_NAME_EXTENTION "${DEPENDENCY_NAME_EXTENTION}m")

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=thread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
endif()

# check for devtools argument
if (sef_enable_devtool)
    add_definitions(-DDEVTOOL)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
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

add_subdirectory(src)

if (sef_cli_unit_test)
    add_subdirectory(test/ut)
endif()

