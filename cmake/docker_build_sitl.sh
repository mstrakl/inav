#!/bin/bash
rm -r build_SITL
mkdir -p build_SITL

# Use environment variables with defaults
BUILD_TYPE="${CMAKE_BUILD_TYPE:-DEBUG}"
EXTRA_CFLAGS="${SITL_CFLAGS:-}"

# Build cmake command
CMAKE_ARGS="-DSITL=ON -DWARNINGS_AS_ERRORS=OFF -GNinja -B build_SITL .."

if [ "$BUILD_TYPE" = "DEBUG" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DDEBUG=ON"
fi

if [ -n "$BUILD_TYPE" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_BUILD_TYPE=$BUILD_TYPE"
fi

if [ -n "$EXTRA_CFLAGS" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_C_FLAGS='$EXTRA_CFLAGS'"
fi

echo "Running: cmake $CMAKE_ARGS"
cmake $CMAKE_ARGS
cd build_SITL
ninja