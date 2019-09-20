#!/bin/bash
set -e

[ ! -d build ] && mkdir build
cd build

export CC=clang-8
export CXX=clang++-8
#cmake "-DCMAKE_CXX_CLANG_TIDY=/usr/sbin/clang-tidy;-checks=*" ..
cmake ..
make

cd -
