#!/bin/bash
set -e

[ ! -d build ] && mkdir build
cd build

export CC=clang
export CXX=clang++
cmake -DDISABLE_ROS=TRUE ../khmot
make
make test

cd -
