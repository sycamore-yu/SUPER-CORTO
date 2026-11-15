#!/usr/bin/env bash
# Source this file from Bash before build, launch or regression commands.
export SUPER_CORTO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
export MOSEK_ROOT="${MOSEK_ROOT:-/home/tong/.local/opt/mosek-11.2.2/mosek/11.2}"
export MOSEKLM_LICENSE_FILE="${MOSEKLM_LICENSE_FILE:-$HOME/mosek/mosek.lic}"
export CMAKE_PREFIX_PATH="$SUPER_CORTO_ROOT/.deps/nlopt/usr:$SUPER_CORTO_ROOT/.deps/glfw/usr:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$MOSEK_ROOT/tools/platform/linux64x86/bin:$SUPER_CORTO_ROOT/.deps/nlopt/usr/lib/x86_64-linux-gnu:$SUPER_CORTO_ROOT/.deps/glfw/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MOSEK_SYS_NUM_CORES=1
if [[ -f "$SUPER_CORTO_ROOT/install/setup.bash" ]]; then
  source "$SUPER_CORTO_ROOT/install/setup.bash"
fi
