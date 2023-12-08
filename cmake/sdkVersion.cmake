function(sdk_version _sef_sdk_version rootDir)
    find_package(Git QUIET)
    if(GIT_FOUND)
        execute_process(
            COMMAND     bash -c "${rootDir}/scripts/sdk_ver.sh"
            OUTPUT_VARIABLE     ver_hash
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
    else()
        set(ver_hash "unavailable")
    endif()
    set(version "${CMAKE_PROJECT_VERSION}.${ver_hash}")
    message("${PROJECT_NAME} version ${version}")
    set(${_sef_sdk_version} ${version} PARENT_SCOPE)
#add_definitions(-DSEFSDKVersion=${SHORT_HASH_TEXT})
endfunction()
