#!/bin/bash -e

CMAKE_ARGS=""
OPT_TEST_FLAG_ON="false"

while getopts "cdelrstwzb:" opt; do
  case ${opt} in
    c ) OPT_FLAG_COVERAGE_ON="true";;
    d ) OPT_FLAG_DEBUGLOG_ON="true";;
    e ) OPT_FLAG_ENVEMU_ON="true";;
    f ) OPT_FLAG_DEBUGOUTFILE_ON="true";;
    r ) OPT_FLAG_RELEASE_BUILD="true";;
    s ) OPT_FLAG_ENVSIM_ON="true";;
    b ) BUILD_ARG="$OPTARG" ;;
    l ) OPT_FLAG_JUST_LIB="true" ;;
    t ) 
        CMAKE_ARGS="${CMAKE_ARGS} -Dsef_lib_unit_test=ON"
        OPT_TEST_FLAG_ON="true" ;;
    * ) echo "Usage: ${THIS_SCRIPT} [-c] [-d] [-e] [-f] [-l] [-r] [-s] [-t] [-b:]"
        echo " -c : Library with code coverage (DEPRECATED)"
        echo " -e : itest for emulator"
	echo " -l : Build just the library and not the driver"
        echo " -r : Release build (DEPRECATED)"
        echo " -s : No Library - use the simulator"
        echo " -d : Library with debug log on"
        echo " -f : Library with debug output file"
        echo " -t : Build unit and integration tests"
        echo " -b : Build type (release, coverage, sanitizer, thread)"
        exit;;
  esac
done

#update submodule
git submodule update --init ../api src/liburing || true

if [[ ${OPT_FLAG_JUST_LIB} != "true" ]]; then
# build driver
if [[ -z "$SEF_KERNEL_SRC" ]]; then
    echo "SEF_KERNEL_SRC not set in environment - not building sef_ko" 1>&2
    OPT_FLAG_JUST_LIB="true"
else
pushd ../driver
SEFSDKVersion=`../scripts/sdk_ver.sh`
make SEFSDKVersion="1.0.$SEFSDKVersion"
popd
fi
fi

# lib uring build
pushd src/liburing
make clean
./configure --cc="gcc -fPIC"
make -j`nproc`
# Projects link directly to build liburing.a so install
# is not normally required.
# make install
popd

# make the output directory
rm -rf bin
mkdir bin
pushd bin

# set the build type
if [[ $BUILD_ARG == "release" ]]; then
    echo "Building release"
    CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Release"
elif [[ $BUILD_ARG == "coverage" ]]; then
    echo "Building coverage"
    CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Debug -Dsef_lib_coverage=ON"
    NAME_EXT="${NAME_EXT}c"
elif [[ $BUILD_ARG == "sanitizer" ]]; then
    echo "Building sanitizer"
    CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Debug -Dsef_sanitizer=ON"
    NAME_EXT="${NAME_EXT}s"
elif [[ $BUILD_ARG == "thread" ]]; then
    echo "Building thread sanitizer"
    CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Debug -Dsef_thread_sanitizer=ON"
    NAME_EXT="${NAME_EXT}m"
else
    echo "Building debug"
    CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Debug"
fi

if [ "${OPT_FLAG_ENVEMU_ON}" = "true" ]; then CMAKE_ARGS="${CMAKE_ARGS} -Dsef_enable_emulator=ON"; fi
if [ "${OPT_FLAG_ENVSIM_ON}" = "true" ]; then CMAKE_ARGS="${CMAKE_ARGS} -Dsef_enable_simulator=ON"; fi
if [ "${OPT_FLAG_DEBUGLOG_ON}" = "true" ]; then CMAKE_ARGS="${CMAKE_ARGS} -Dsef_debug_log=ON"; fi
if [ "${OPT_FLAG_DEBUGOUTFILE_ON}" = "true" ]; then CMAKE_ARGS="${CMAKE_ARGS} -Dsef_debug_out_file=ON"; fi

# DEPRECATED ARGUMENTS
if [ "${OPT_FLAG_COVERAGE_ON}" = "true" ]; then 
    echo "Building coverage"
    CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Debug -Dsef_lib_coverage=ON"
    NAME_EXT="${NAME_EXT}c"
fi
if [ "${OPT_FLAG_RELEASE_BUILD}" = "true" ]; then CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Release"; fi

# prepare and build
echo cmake ${CMAKE_ARGS} ..
cmake ${CMAKE_ARGS} ..
echo cmake done
make clean
make -j`nproc`
if [ "$?" -ne "0" ]; then
     echo build error
     exit $?
fi

# install the library if not test
if [ "${OPT_TEST_FLAG_ON}" = "false" ]; then 
    make install
fi

# copy artifacts to the root
if [ "${OPT_FLAG_ENVSIM_ON}" = "false" ]; then
    cp src/lib/libsef*.so .
fi

if [ "${OPT_TEST_FLAG_ON}" = "true" ]; then 
    cp test/it/sef-it-test* .
    cp test/ut/lib/sef-ut-test* .
fi

popd
