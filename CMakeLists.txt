########################################################
#
# Project Options
#

option(sef_coverage "Build code with coverage enabled build type" OFF)

option(sef_sanitizer "Build code with sanitizer enabled build type" OFF)

option(sef_thread_sanitizer "Build code with thread sanitizer enabled and debug build type" OFF)

option(sef_extra_warnings "Turn on -Wextra" OFF)

########################################################
#
# Project Settings
#

cmake_minimum_required(VERSION 3.8)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

project(
    sef-sdk
    VERSION 1.0
    LANGUAGES CXX C
)

# C version
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (sef_extra_warnings)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra ")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra ")
endif()


# set CMake Path
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#set sef_sdk_version
include(sdkVersion)
sdk_version(sef_sdk_version "${CMAKE_SOURCE_DIR}")
add_definitions(-DSEFSDKVersion="${sef_sdk_version}")

# Fetch googletest
if (download_googletest)
if (sef_sim_unit_test OR 
	sef_lib_unit_test OR 
	sef_ftl_unit_test OR 
	sef_cli_unit_test)
    include(FetchContent)
    FetchContent_Declare(
       googletest
       # Specify the commit you depend on and update it regularly.
       URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
    )
    FetchContent_MakeAvailable(googletest)
endif()
endif()

if (sef_sim_only)
	add_subdirectory(sim)
elseif(sef_lib_only)
	add_subdirectory(lib)
elseif(sef_driver_only)
	find_program(MAKE_EXECUTABLE NAMES gmake make REQUIRED)
	add_custom_target(driver-clean ALL COMMAND ${MAKE_EXECUTABLE} clean
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/driver)
	include(ExternalProject)
	if (sef_driver_install)
		ExternalProject_Add(driver
	                PREFIX "${CMAKE_SOURCE_DIR}/driver"
			DEPENDS ""
	                SOURCE_DIR "${CMAKE_SOURCE_DIR}/driver"
			BINARY_DIR "${CMAKE_SOURCE_DIR}/driver"
	                CONFIGURE_COMMAND ""
			BUILD_COMMAND ${MAKE_EXECUTABLE}
			INSTALL_COMMAND ${MAKE_EXECUTABLE} modules_install
	        )
	else()
		ExternalProject_Add(driver
	                PREFIX "${CMAKE_SOURCE_DIR}/driver"
			DEPENDS ""
	                SOURCE_DIR "${CMAKE_SOURCE_DIR}/driver"
	                CONFIGURE_COMMAND ""
			BUILD_IN_SOURCE 1
			BUILD_COMMAND ${MAKE_EXECUTABLE}
			INSTALL_COMMAND ""
	        )
	endif()

elseif(sef_ftl_only)
	add_subdirectory(ftl)
elseif(sef_cli_only)
	add_subdirectory(cli)
elseif(DEFINED sef_enable_simulator OR DEFINED sef_enable_emulator)
	add_subdirectory(sim)
	add_subdirectory(lib)
	add_subdirectory(ftl)
	include_directories(${PROJECT_SOURCE_DIR}/ftl/src/)
	include_directories(${PROJECT_SOURCE_DIR}/api)
	add_subdirectory(cli)
else()
	add_subdirectory(lib)
	add_subdirectory(ftl)
	include_directories(${PROJECT_SOURCE_DIR}/ftl/src/)
	include_directories(${PROJECT_SOURCE_DIR}/api)
	add_subdirectory(cli)
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

