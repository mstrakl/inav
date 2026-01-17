# INAV Build Guide

## Prerequisites

- Docker (for containerized builds)
- Git

## Building Firmware

### Standard Hardware Targets

Build any hardware target using:

```bash
./build.sh <TARGET>
```

Examples:
```bash
./build.sh MATEKF405SE
./build.sh AOCODARCH7DUAL
```

### Multiple Targets

```bash
./build.sh MATEKF405SE AOCODARCH7DUAL
```

### SITL (Software In The Loop)

The SITL target runs the firmware as a native host application for testing and simulation.

#### Basic Build

```bash
./build.sh SITL
```

This builds with default settings: `DEBUG` mode with warnings as errors enabled.

#### Build Options

Control the build using environment variables:

**CMAKE_BUILD_TYPE**: Set the build type
- `DEBUG` (default) - Debug symbols, optimizations disabled
- `Release` - Optimized build, no debug symbols
- `RelWithDebInfo` - Optimized with debug symbols
- `MinSizeRel` - Size-optimized build

**SITL_CFLAGS**: Add custom compiler flags

#### Examples

Release build:
```bash
CMAKE_BUILD_TYPE=Release ./build.sh SITL
```

Disable specific warnings:
```bash
SITL_CFLAGS="-Wno-double-promotion -Wno-ignored-qualifiers" ./build.sh SITL
```

Release build with custom optimizations:
```bash
CMAKE_BUILD_TYPE=Release SITL_CFLAGS="-O3 -march=native" ./build.sh SITL
```

Combined options:
```bash
CMAKE_BUILD_TYPE=RelWithDebInfo SITL_CFLAGS="-Wno-error=double-promotion" ./build.sh SITL
```

#### SITL Output

The built executable is located at:
```
build/build_SITL/SITL
```

Run it with:
```bash
./build/build_SITL/SITL
```

## Cleaning

Clean a specific target:
```bash
./build.sh clean_MATEKF405SE
./build.sh clean_SITL
```

Clean all targets:
```bash
./build.sh clean
```

## Listing Targets

Show release targets:
```bash
./build.sh release_targets
```

Show all valid targets:
```bash
./build.sh valid_targets
```

Show all targets with details:
```bash
./build.sh help | less
```

## Build Artifacts

Hardware target builds produce `.hex` files in:
```
build/*.hex
```

SITL builds produce an executable in:
```
build/build_SITL/SITL
```

## Docker Image

The build system automatically builds and caches a Docker image with all required toolchains and dependencies. If you need to rebuild the image:

```bash
docker rmi inav-build
./build.sh <TARGET>
```

## Troubleshooting

### Permission Denied Errors

If you see "permission denied" errors for scripts inside Docker:

```bash
chmod +x cmake/*.sh
```

### CMake Cache Issues

If you get CMake cache errors, clean the build directory:

```bash
rm -rf build/*
./build.sh <TARGET>
```

### SITL Compilation Errors

For warnings treated as errors, disable specific warnings:

```bash
SITL_CFLAGS="-Wno-error=<warning-type>" ./build.sh SITL
```

Common warning types:
- `double-promotion`
- `ignored-qualifiers`
- `maybe-uninitialized`
- `return-local-addr`
