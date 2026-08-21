#!/bin/bash
# onnxruntime riscv64 cp312 wheel -- native build inside a riscv64 Ubuntu 24.04
# container under qemu-user. Runs INSIDE the container; build_ort_wheel.sh
# launches it. Same distro as the q-vela guest, so the wheel's glibc,
# libstdc++ and CPython ABI match the guest exactly -- which is the whole
# reason to build rather than borrow: every prebuilt riscv64 Python binding
# that exists targets the wrong Python (Debian/Ubuntu: 3.13) or traps on
# custom silicon instructions QEMU does not implement (spacemit-ort: SIGILL
# at import, measured 2026-08-18).
set -euxo pipefail
export DEBIAN_FRONTEND=noninteractive

ORT_TAG=${ORT_TAG:-v1.23.2}   # the version Debian's riscv64 buildd builds routinely

apt-get update
apt-get install -y --no-install-recommends \
    git ca-certificates build-essential gcc-14 g++-14 cmake ninja-build \
    python3-dev python3-pip python3-numpy python3-setuptools python3-wheel \
    python3-packaging pkg-config

# Debian's riscv64 buildd compiles this tag with GCC 14; follow suit.
export CC=gcc-14 CXX=g++-14

# Idempotent on purpose: a dockerd restart (unattended docker-ce upgrades do
# this) kills the container mid-build, and `docker start` then re-runs this
# script in the same filesystem. Skip the clone, let make resume.
cd /root
if [ ! -d onnxruntime ]; then
  git clone --depth 1 --branch "$ORT_TAG" --recursive \
      https://github.com/microsoft/onnxruntime.git
fi
cd onnxruntime

# No --enable_rvv even though the guest advertises RVV 1.0: TCG emulates
# vector ops instruction-by-instruction and measured ~70x on vectorised float
# kernels (gz-split HANDOFF), so scalar MLAS is the safer default here.
#
# Not $(nproc): the container sees all 192 cores of a shared machine (the
# cgroup pinning applies to shells, not to dockerd), and 192 emulated
# compilers is a hostile neighbour. 48 saturates the emulation overhead
# without owning the box.
./build.sh --config Release --build_wheel --skip_tests --parallel "${PARALLEL:-48}" \
    --allow_running_as_root --compile_no_warning_as_error \
    --cmake_extra_defines onnxruntime_BUILD_UNIT_TESTS=OFF

cp build/Linux/Release/dist/*.whl /out/
echo BUILD_OK
