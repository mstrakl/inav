#!/usr/bin/env bash
set -Eeuo pipefail

# Setup cleanup handler
cleanup() {
    #echo ""
    #echo "Shutting down..."
    # Kill all background processes in this script's process group
    kill -- -$$ 2>/dev/null || true
}

trap cleanup INT TERM EXIT

# ---------------------------------------------------------- #
# Start INAV simulator components
# ---------------------------------------------------------- #

ENV_DIR="env"
PYTHON_BIN="$ENV_DIR/bin/python3"

# Start Python simulator
echo "Starting Python simulator..."
$PYTHON_BIN main.py --sim=inav --joy & #> rotsim.log 2>&1 &
PYTHON_PID=$!

sleep 1


# Start INAV SITL simulator
echo "Starting INAV SITL..."
BINARY="../build/build_SITL/inav_9.0.1_SITL"
if [ ! -f "$BINARY" ]; then
    echo "✗ Error: SITL binary not found at $BINARY"
    echo "  Run: bash ../build_sitl_mac.sh (macOS) or bash ../build.sh (Linux)"
    exit 1
fi


OS="$(uname -s)"
if [[ "$OS" == "Linux" ]]; then
    stdbuf -oL -eL $BINARY \
        --path=./eeprom.bin \
        --sim=adum \
        --chanmap=M01-01,M02-02,M03-03,M04-04,S01-05,S02-06,S03-07,S04-08,S05-09,S06-10 > sitl.log 2>&1 &
    INAV_PID=$!
elif [[ "$OS" == "Darwin" ]]; then
    $BINARY \
        --path=./eeprom.bin \
        --sim=adum \
        --chanmap=M01-01,M02-02,M03-03,M04-04,S01-05,S02-06,S03-07,S04-08,S05-09,S06-10 > sitl.log 2>&1 &
    INAV_PID=$!
else
    echo "Unsupported OS: $OS"
    exit 1
fi



#sleep 2
#
#echo "Starting socat PTY bridge..."
#socat -d -d \
#  pty,raw,echo=0,link=/tmp/inav_uart6 \
#  tcp:localhost:5765 &
#SOCAT_PID=$!
#

echo ""
echo "✓ All components started"
echo "  Python simulator:  localhost:2323"
echo "  INAV SITL:         TCP sockets on various ports"
echo "  UART6 PTY:         /tmp/inav_uart6"
echo ""
echo "Press Ctrl+C to stop all components."
echo ""

# Keep script alive and wait for all background processes
wait
