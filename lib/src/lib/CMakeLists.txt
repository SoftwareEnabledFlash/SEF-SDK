########################################################
#
# Project Options
#

option(sef_lib_dynamic "Build dynamic library" ON)

########################################################
#
# Build the Project
#

# project info
project (
    sef-src
    LANGUAGES C
)

# create project name
set(PROJECT_NAME "sef${PROJECT_NAME_EXTENTION}")

#include directories
include_directories(${PROJECT_SOURCE_DIR}/../liburing/src/include)

# get the source files
file(GLOB_RECURSE SOURCE_FILES *.c)

# create lib
if (sef_lib_dynamic)
    add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
else()
    add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})
endif()

########################################################
#
# Link Libraries
#

target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/../liburing/src/liburing.a)
target_link_libraries(${PROJECT_NAME} udev)

########################################################
#
# Project Install
#

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

install(FILES ${PROJECT_SOURCE_DIR}/../../../api/SEFAPI.h DESTINATION include)
