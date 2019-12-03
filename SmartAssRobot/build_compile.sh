#!/usr/bin/sh
DIR="$(dirname $(readlink -f $0))"
cd "${DIR}"
BUILD_DIR="${DIR}/build"
cmake "${DIR}" -B"${BUILD_DIR}" && cmake --build "${BUILD_DIR}"
