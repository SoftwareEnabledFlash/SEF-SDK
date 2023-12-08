#!/bin/bash -e

CMAKE_ARGS=""
BUILD_SIM=0
IS_TEST="0"
NAME_EXT=""
PYTHON_VERSION=python3.6m

# parse and check cli arguments
while getopts "b:p:n:shrt" opt; do
  case ${opt} in
    b) 
      BUILD_ARG="$OPTARG"
      ;;
    p)
      CMAKE_ARGS="${CMAKE_ARGS} -Dsef_python_version=${OPTARG}"
      PYTHON_VERSION=${OPTARG}
      ;;
    n)
      SERIAL="$OPTARG"
      ;;
    s)
      CMAKE_ARGS="${CMAKE_ARGS} -Dsef_enable_simulator=ON"
      LIB_ARG="simulator"
      ;;
    t)
      CMAKE_ARGS="${CMAKE_ARGS} -Dsef_cli_unit_test=ON"
      IS_TEST="1"
      ;;
    h)
      echo "Usage: cmd [-b for build type] [-l for fpga build] [-p for python version]"
      exit 1
      ;;
  esac
done

# set the build type
if [[ $BUILD_ARG == "release" ]]; then
    echo "Building release"
    CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Release"
elif [[ $BUILD_ARG == "coverage" ]]; then
    echo "Building coverage"
    CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Debug -Dsef_cli_coverage=ON"
elif [[ $BUILD_ARG == "sanitizer" ]]; then
    echo "Building sanitizer"
    CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Debug -Dsef_sanitizer=ON"
    NAME_EXT="${NAME_EXT}s"
    export LSAN_OPTIONS="suppressions=$(pwd)/suppress_sanitizer.conf"
elif [[ $BUILD_ARG == "thread" ]]; then
    echo "Building thread sanitizer"
    CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Debug -Dsef_thread_sanitizer=ON"
    NAME_EXT="${NAME_EXT}m"
    export TSAN_OPTIONS="suppressions=$(pwd)/suppress_tsan.conf"
else
    echo "Building debug"
    CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Debug"
fi

# make the output directory
rm -rf bin
mkdir bin
pushd bin

# prepare and build
cmake ${CMAKE_ARGS} ..
make clean
make -j`nproc`
if [ "$?" -ne "0" ]; then
     echo build error
     exit $?
fi


popd

if [[ $IS_TEST == "0" ]]; then
  pushd bin
  make install
  popd
else
  pushd bin/test/ut
  UNIT_TEST_NAME="./sef-cli-test${NAME_EXT}"
  ${UNIT_TEST_NAME} $SERIAL --gtest_output=xml
  popd

  if [[ $LIB_ARG == "simulator" ]]; then
    pushd test/it
    ./test1.sh
    ./test2.sh
    ./test3.sh
    ./test4.sh
    ./test5.sh
    ./test6.sh
    ./test7.sh
    ./test8.sh
    ./test9.sh
    ./test10.sh
    ./test11.sh
    ./test12.sh
    ./test13.sh
    ./test14.sh
    ./test14.sh -s "$(pwd)/shellTest2.py"
    ./test15.sh
    ./test16.sh
    ./test17.sh

    popd
  fi
fi


