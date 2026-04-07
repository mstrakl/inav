#!/bin/bash
set -e

LAST_CMAKE_AT_REV_FILE="docker_cmake.rev"
CURR_REV="$(git rev-parse HEAD)"

initialize_cmake() {
    echo -e "*** CMake was not initialized yet, doing it now.\n"
    # Allow caller to specify build type and extra flags via environment
    BUILD_TYPE="${CMAKE_BUILD_TYPE:-}"
    EXTRA_C_FLAGS="${CMAKE_C_FLAGS:-}"
    EXTRA_CXX_FLAGS="${CMAKE_CXX_FLAGS:-}"

    CMAKE_ARGS="-GNinja .."

    if [ -n "$BUILD_TYPE" ]; then
        CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_BUILD_TYPE=$BUILD_TYPE"
    fi

    if [ -n "$EXTRA_C_FLAGS" ]; then
        CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_C_FLAGS='$EXTRA_C_FLAGS'"
    fi

    if [ -n "$EXTRA_CXX_FLAGS" ]; then
        CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_CXX_FLAGS='$EXTRA_CXX_FLAGS'"
    fi

    echo "Running: cmake $CMAKE_ARGS"
    cmake $CMAKE_ARGS
    echo "$CURR_REV" > "$LAST_CMAKE_AT_REV_FILE"
}

# Check if CMake has never been initialized
if [ ! -f build.ninja ]; then
    initialize_cmake
fi

# Check if CMake was initialized for a different Git revision (new targets may have been added)
if [ -f "$LAST_CMAKE_AT_REV_FILE" ]; then
    LAST_CMAKE_AT_REV="$(cat $LAST_CMAKE_AT_REV_FILE)"
    if [[ "$LAST_CMAKE_AT_REV" != "SKIP" ]] && [[ "$LAST_CMAKE_AT_REV" != "$CURR_REV" ]]; then
        initialize_cmake
    fi
else
    initialize_cmake
fi

# Let Make handle the arguments coming from the build script
ninja "$@"
