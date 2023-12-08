#!/bin/bash -e

QEMU_ARG="--disable-sef"
NAME_EXT=""

while getopts "seigo:t:c:b:wnp:" opt; do
  case ${opt} in
    s ) ENV_SIM="true"
        QEMU_ARG="--sef-lib-type=simulator"
        ;;
    e ) ENV_EMU="true" 
        QEMU_ARG="--sef-lib-type=emulator"
        ;;
    i ) ENV_INSTALL="true"
        ;;
    o ) ONLY_ARG="$OPTARG"
        ;;
    t ) TEST_ARG="$OPTARG"
        ;;
    c ) COV_ARG="$OPTARG"
        ;;
    b ) BUILD_ARG="$OPTARG"
        ;;
    g ) DOWNLOAD_GOOGLETEST="true"
        ;;
    p ) PYTHON_ARG="$OPTARG"
        ;;
    n ) NO_GIT_SUBMODULES="true"
        ;;
    w ) EXTRA_WARNINGS="true"
        ;;
    * ) echo "Usage: ${THIS_SCRIPT} [-s] [-e] [-i] [-o <type>] [-t <type>] [-c <type>] [-b <type>] [-h]"
        echo " Default: SEF Device (not simulator or emulator)"
        echo " -s : Use the simulator"
        echo " -e : Use the emulator"
        echo " -i : Install the software (requires root privileges)"
        echo " -o : Only Build This (sim|lib|driver|ftl|cli|qemu|fio)"
        echo " -t : Test Type (sim|lib|ftl|cli)"
        echo " -c : Coverage Type (sim|lib|ftl|cli)"
        echo " -b : Build Type (sanitizer|thread)"
        echo " -g : Use preset googletest"
        echo " -n : Don't pull submodules"
        echo " -p : Specify python version"
        echo " -w : Turn on extra warnings"
        echo " -h : Print help and exit"
        exit
        ;;
  esac
done

USER=$([[ "$SUDO_USER" == "" ]] && whoami || echo $SUDO_USER)

if [[ $ONLY_ARG == "fio" ]]; then
	pushd fio
	sudo -u $USER ./configure $QEMU_ARG
	sudo -u $USER make -j`nproc` || exit 1
	if [[ $ENV_INSTALL == "true" ]]; then
	     echo Installing...
	     sudo make install
	fi
	exit $?
fi

if [[ $ONLY_ARG == "qemu" ]]; then
        sudo -u $USER git submodule update --init qemu
	pushd qemu
	sudo -u $USER rm -rf build
	sudo -u $USER mkdir build
	pushd build
	sudo -u $USER ../configure --target-list=x86_64-softmmu --enable-debug $QEMU_ARG
	sudo -u $USER make -j`nproc` || exit 1
	if [[ $ENV_INSTALL == "true" ]]; then
	     echo Installing...
	     sudo make install
	fi
	exit $?
fi

#update submodules, except for QEMU, unless GIT_ARG=no
if [[ $NO_GIT_SUBMODULES != "false" ]]; then
	sudo -u $USER git -c submodule."qemu".update=none submodule update --init --recursive
fi

CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Debug"

if [[ $EXTRA_WARNINGS == "true" ]]; then
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_extra_warnings=ON"
fi


if [[ $ENV_SIM == "true" ]]; then
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_enable_simulator=ON"
elif [[ $ENV_EMU == "true" ]]; then
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_enable_emulator=ON"
fi

if [[ $BUILD_ARG == "sanitizer" ]]; then
    echo "Building sanitizer"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_sanitizer=ON"
    NAME_EXT="s"
elif [[ $BUILD_ARG == "thread" ]]; then
    echo "Building thread sanitizer"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_thread_sanitizer=ON"
    NAME_EXT="m"
fi


if [[ $COV_ARG == "sim" ]]; then
    echo "Building coverage for sim"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_sim_coverage=ON"
elif [[ $COV_ARG == "lib" ]]; then
    echo "Building coverage for lib"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_lib_coverage=ON"
elif [[ $COV_ARG == "ftl" ]]; then
    echo "Building coverage for ftl"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_ftl_coverage=ON"
elif [[ $COV_ARG == "cli" ]]; then
    echo "Building coverage for cli"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_cli_coverage=ON"
fi

if [[ $TEST_ARG == "sim" ]]; then
    echo "Building test for sim"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_sim_unit_test=ON"
elif [[ $TEST_ARG == "lib" ]]; then
    echo "Building test for lib"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_lib_unit_test=ON"
elif [[ $TEST_ARG == "ftl" ]]; then
    echo "Building test for ftl"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_ftl_unit_test=ON"
elif [[ $TEST_ARG == "cli" ]]; then
    echo "Building test for cli"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_cli_unit_test=ON"
fi

if [[ $ONLY_ARG == "sim" ]]; then
    echo "Building sim"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_sim_only=ON"
elif [[ $ONLY_ARG == "lib" ]]; then
    echo "Building lib"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_lib_only=ON"
elif [[ $ONLY_ARG == "ftl" ]]; then
    echo "Building ftl"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_ftl_only=ON"
elif [[ $ONLY_ARG == "cli" ]]; then
    echo "Building cli"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_cli_only=ON"
elif [[ $ONLY_ARG == "driver" ]]; then
    echo "Building driver"
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_driver_only=ON"
    if [[ $ENV_INSTALL == "true" ]]; then
        CMAKE_ARGS="${CMAKE_ARGS} -Dsef_driver_install=ON"
    fi
fi

if [[ $DOWNLOAD_GOOGLETEST == "true" ]]; then
    CMAKE_ARGS="${CMAKE_ARGS} -Ddownload_googletest=ON"
fi

if [[ $PYTHON_ARG != "" ]]; then
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_python_version=$PYTHON_ARG"
fi

if [[ $ENV_SIM != "true" && $ENV_SIM != "true" ]]; then
    if [[ $ONLY_ARG == "" || $ONLY_ARG == "lib" ]]; then
        ## lib uring build
        pushd lib/src/liburing
        sudo -u $USER make clean
        sudo -u $USER ./configure --cc="gcc -fPIC"
        sudo -u $USER make -j`nproc`
        # Projects link directly to build liburing.a so install
        # is not normally required.
        # make install
        popd
    fi
fi

if [[ $ENV_SIM == "" ]] && [[ $TEST_ARG == "sim" || $COV_ARG == "sim" ]]; then
    CMAKE_ARGS="${CMAKE_ARGS} -Dsef_enable_simulator=ON"
fi

# make the output directory
sudo -u $USER rm -rf bin sim/bin lib/bin ftl/bin cli/bin
sudo -u $USER mkdir bin
pushd bin

echo cmake ${CMAKE_ARGS} ..
sudo -u $USER cmake ${CMAKE_ARGS} ..
sudo -u $USER make clean
if [[ $ONLY_ARG == "driver" ]]; then
     if [[ $ENV_INSTALL == "true" ]]; then
          echo Installing...
          sudo SEF_KERNEL_SRC=$SEF_KERNEL_SRC make
          exit
     else
          sudo -u $USER SEF_KERNEL_SRC=$SEF_KERNEL_SRC make
     fi
else
     sudo -u $USER make -j`nproc`
fi
if [ "$?" -ne "0" ]; then
     echo Build failed to complete.
fi

if [[ $ENV_INSTALL == "true" ]]; then
     echo Installing...
     sudo make install
fi
popd # bin

if [[ $TEST_ARG == "sim" ]]; then
  pushd bin/sim/test
  UNIT_TEST_NAME="./sef-sim-test${NAME_EXT}"
  if [[ $BUILD_ARG == "thread" ]]; then
      sudo TSAN_OPTIONS="suppressions=../../../suppress_tsan.conf" $UNIT_TEST_NAME --gtest_output=xml
  else
      sudo $UNIT_TEST_NAME --gtest_output=xml
  fi
  popd
elif [[ $TEST_ARG == "ftl" ]]; then
  pushd bin/ftl/test
  UNIT_TEST_NAME="./sef-ftl-test${NAME_EXT}"
  if [[ $BUILD_ARG == "thread" ]]; then
      sudo TSAN_OPTIONS="suppressions=../../../suppress_tsan.conf" $UNIT_TEST_NAME --gtest_output=xml --gc-test
  else
      sudo $UNIT_TEST_NAME $SERIAL --gtest_output=xml --gc-test
  fi
  if [ "$?" -ne "0" ]; then
      tail mgc.gc.sef/SEFLog.2
      exit 1
  fi
  popd
elif [[ $TEST_ARG == "cli" ]]; then
  pushd bin/cli/test/ut
  UNIT_TEST_NAME="./sef-cli-test${NAME_EXT}"
  sudo ${UNIT_TEST_NAME} $SERIAL --gtest_output=xml
  popd

  if [[ $ENV_SIM == "true" ]]; then
    pushd cli/test/it
    sudo ./test1.sh
    sudo ./test2.sh
    sudo ./test3.sh
    sudo ./test4.sh
    sudo ./test5.sh
    sudo ./test6.sh
    sudo ./test7.sh
    sudo ./test8.sh
    sudo ./test9.sh
    sudo ./test10.sh
    sudo ./test11.sh
    sudo ./test12.sh
    sudo ./test13.sh
    sudo ./test14.sh
    sudo ./test14.sh -s "$(pwd)/shellTest2.py"
    sudo ./test15.sh
    sudo ./test16.sh
    sudo ./test17.sh
    popd
  fi
fi

