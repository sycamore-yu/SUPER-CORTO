#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
cd "$SUPER_CORTO_ROOT"
test -f "$MOSEK_ROOT/tools/platform/linux64x86/h/mosek.h"
test -f .deps/nlopt/usr/include/nlopt.hpp
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
colcon build --packages-up-to corto_ros2 perfect_drone_sim --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DNLopt_DIR="$SUPER_CORTO_ROOT/.deps/nlopt/usr/lib/x86_64-linux-gnu/cmake/nlopt"
