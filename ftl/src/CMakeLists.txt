########################################################
#
# SubProject Options
#

option(sef_sdk_dynamic "Build dynamic library" ON)


########################################################
#
# Build the SubProject
#

# SubProject info
project (
    sef-ftl-src
    LANGUAGES C
)


# create SubProject name
set(PROJECT_NAME "sef-ftl${PROJECT_NAME_EXTENTION}")

# get the source files
file(GLOB_RECURSE sources *.c)

# create lib
if (sef_sdk_dynamic)
    add_library(${PROJECT_NAME} SHARED ${sources})
else()
    add_library(${PROJECT_NAME} STATIC ${sources})
endif()


########################################################
#
# Link Libraries
#

target_link_libraries(${PROJECT_NAME} sef${DEPENDENCY_NAME_EXTENTION})

target_link_libraries(${PROJECT_NAME} pthread)
#Needed on certain platforms
#target_link_libraries(${PROJECT_NAME} atomic)


########################################################
#
# Project Install
#

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

install(FILES ${PROJECT_SOURCE_DIR}/sef-utils.h DESTINATION include)

install(FILES ${PROJECT_SOURCE_DIR}/utils/buddy.h DESTINATION include)

install(FILES ${PROJECT_SOURCE_DIR}/sef-block-module.h DESTINATION include)
