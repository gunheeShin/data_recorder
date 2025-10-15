#!/usr/bin/env bash

set -euo pipefail

BUILD_DIR="build"

if [[ -d "${BUILD_DIR}" ]]; then
  rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}"

cd build && cmake .. && make

cd ..
