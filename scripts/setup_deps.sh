#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
mkdir -p .deps/debs
cd .deps/debs
apt-get download libnlopt-cxx-dev=2.7.1-5build2 libnlopt-dev=2.7.1-5build2 \
  libnlopt0=2.7.1-5build2 libnlopt-cxx0=2.7.1-5build2
for corto_deb in ./*.deb; do dpkg-deb -x "$corto_deb" ../nlopt; done
cd ..
mkdir -p glfw-debs
cd glfw-debs
apt-get download libglfw3=3.3.10-1build1 libglfw3-dev=3.3.10-1build1
for corto_deb in ./*.deb; do dpkg-deb -x "$corto_deb" ../glfw; done
