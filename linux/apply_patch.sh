#!/bin/bash
fullpath=$(realpath $0)
dirpath=$(dirname $fullpath)
pushd $dirpath
git submodule update --init --recursive
pushd official-linux
patch -N -p1 < ../SEF-LINUX.patch
popd #official-linux
popd #dirpath

