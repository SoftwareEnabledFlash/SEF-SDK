#!/bin/bash
fullpath=$(realpath $0)
dirpath=$(dirname $fullpath)
pushd $dirpath
git submodule update --init --recursive
pushd official-nvme
patch -N -p1 < ../SEF-NVME.patch
popd #official-nvme
popd #dirpath

