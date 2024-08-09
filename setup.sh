#!/bin/bash
#
# Copyright (c) 2024.  Jan-Hendrik Ewers
# SPDX-License-Identifier: GPL-3.0-only
#

set -eux

SRC_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

INSTALL_LIBJDRONES=""
INSTALL_EIGEN3=""
for i in "$@"; do
  case $i in
    --libjdrones)
      INSTALL_LIBJDRONES=1
      shift
      ;;
    --eigen3)
      INSTALL_EIGEN3=1
      shift
      ;;
    --all)
      INSTALL_EIGEN3=1
      INSTALL_LIBJDRONES=1
      ;;
    -*|--*)
      echo "Unknown option $i"
      exit 1
      ;;
    *)
      ;;
  esac
done

if [ -n "$INSTALL_EIGEN3" ]
then
  # EIGEN3
  cd $SRC_DIR
  curl -OL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  tar -xzvf eigen-3.4.0.tar.gz
  cd eigen-3.4.0
  mkdir -p _build
  cd _build
  cmake ..
  make install
fi

if [ -n "$INSTALL_LIBJDRONES" ]
then
  # LIBJDRONES
  cd $SRC_DIR
  mkdir -p _build
  cd _build
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    ..
  make -j "$(nproc)"
  make install
fi
