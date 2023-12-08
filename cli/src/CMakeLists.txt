########################################################
#
# Build the Project
#

# project name
project (
    sef-cli-src
    VERSION 2.7
    LANGUAGES C
)

# create project name
set(PROJECT_NAME "sef-cli${PROJECT_NAME_EXTENTION}")

# get the source files
if (sef_cli_unit_test)
    include_directories(${PROJECT_SOURCE_DIR}/../../ftl/src/)
    include_directories(${PROJECT_SOURCE_DIR}/../../api)
endif()
file( GLOB_RECURSE sources *.c)

# remove the extra files based on options (devtools)
if (NOT sef_enable_devtool)
    file(GLOB devtools devtool/*.c)
    list(REMOVE_ITEM sources ${devtools})
endif()

# create executable
add_executable(${PROJECT_NAME} ${sources})

########################################################
#
# Link Libraries
#
if (sef_enable_simulator)
    message("sef_enable_simulator is ENABLED!      : " ${sef_enable_simulator})
    include_directories(${PROJECT_SOURCE_DIR}/../../sim/include/sim)
    add_definitions(-DSIMULATOR)
    target_link_libraries(${PROJECT_NAME} sef-sim${DEPENDENCY_NAME_EXTENTION})
    find_package(yaml-cpp REQUIRED)
    target_link_libraries(${PROJECT_NAME} yaml-cpp)
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
        target_link_libraries(${PROJECT_NAME} stdc++)
    endif()
    find_library(EXECINFO_LIB execinfo)
    if(EXECINFO_LIB)
        target_link_libraries(${PROJECT_NAME} execinfo)
    endif()
else()
    target_link_libraries(${PROJECT_NAME} sef${DEPENDENCY_NAME_EXTENTION})
endif()

target_link_libraries(${PROJECT_NAME} sef-ftl${DEPENDENCY_NAME_EXTENTION})
target_link_libraries(${PROJECT_NAME} ${PYTHON_VERSION})
target_link_libraries(${PROJECT_NAME} pthread)
#Needed on some platforms
#target_link_libraries(${PROJECT_NAME} atomic)

########################################################
#
# Project Install
#
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES ${PROJECT_SOURCE_DIR}/../sef-cli.1 DESTINATION ${CMAKE_INSTALL_MANDIR})

install(FILES ${PROJECT_SOURCE_DIR}/../sef-cli_completion.sh RENAME ${PROJECT_NAME} DESTINATION /etc/bash_completion.d)

