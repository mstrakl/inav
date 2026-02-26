# JSBSim - INAV SITL Integration

This directory contains scripts to interface JSBSim with INAV SITL (Software In The Loop) using the socket protocol expected by `adumsim.c`.

## Files

- **`jsbsim_inav_bridge.py`** - Main bridge server that connects JSBSim to INAV SITL
- **`test_protocol.py`** - Test script to verify protocol without INAV
- **`main.py`** - Standalone JSBSim demo with motor control
- **`extract_state_example.py`** - Example of extracting state data from JSBSim

## Quick Start

### 1. Test JSBSim Standalone

Run the demo to verify JSBSim works:

```bash
python main.py
```

### 2. Test Protocol Without INAV

Terminal 1 - Start the bridge:
```bash
python jsbsim_inav_bridge.py
```

Terminal 2 - Run test client:
```bash
python test_protocol.py
```

### 3. Connect to INAV SITL

Terminal 1 - Start JSBSim bridge:
```bash
cd /home/m.strakl/Mitja/adum/adum26/inav/jsbsim
python jsbsim_inav_bridge.py
```

Terminal 2 - Start INAV SITL:
```bash
cd /home/m.strakl/Mitja/adum/adum26/inav
# Build INAV SITL if needed
make TARGET=SITL

# Run INAV with adumsim
./obj/main/inav_SITL --sim-adum=127.0.0.1:2323
```

## Protocol Details

### INAV → JSBSim (Motor Commands)
INAV sends motor commands as semicolon-separated floats (0-1 range):
```
motor0;motor1;motor2;motor3;\n
```

### JSBSim → INAV (Sensor Data)
Bridge sends semicolon-separated state data:
```
trel;lat*1e7;lon*1e7;alt_msl;track;hdg;posx;posy;agl;gspeed;
roll;pitch;yaw;ax;ay;az;gx;gy;gz;ch1;...;ch16;\n
```

Fields:
- **0:** Time (s)
- **1-2:** Latitude/Longitude (int, degrees * 1e7)
- **3:** Altitude MSL (m)
- **4:** Track angle (deg)
- **5:** Heading (deg)
- **6-7:** Local position X,Y (m) - not currently used
- **8:** Altitude AGL (m)
- **9:** Ground speed (m/s)
- **10-12:** Roll, Pitch, Yaw (deg)
- **13-15:** Accelerations X,Y,Z (g)
- **16-18:** Angular rates P,Q,R (deg/s)
- **19-34:** RC Channels 1-16 (normalized floats)

## Motor Mapping

The bridge maps INAV motor outputs to JSBSim quadcopter motors:

```
INAV Motors (assumed):     JSBSim Quad X:
  motor[0] = Rear-Right      fcs/se_motor (SE)
  motor[1] = Front-Right     fcs/ne_motor (NE)
  motor[2] = Rear-Left       fcs/sw_motor (SW)
  motor[3] = Front-Left      fcs/nw_motor (NW)
```

Verify your actual INAV motor configuration and adjust `apply_motor_commands()` in the bridge if needed.

## Configuration

Edit `jsbsim_inav_bridge.py` to change:

- `LISTEN_IP` / `LISTEN_PORT` - Server address (default: 127.0.0.1:2323)
- `JSBSIM_DT` - Simulation timestep (default: 0.01s = 100Hz)
- Initial conditions in `setup_jsbsim()`
- Motor mapping in `apply_motor_commands()`

## Quadcopter Model

The bridge uses the quadcopter defined in `run/aircraft/quad/quad.xml`:

- **Mass:** 0.84 lbs (0.381 kg)
- **Configuration:** X-frame with 4 motors
- **Max thrust per motor:** 0.84 lbs (3.74 N)
- **Motor naming:** NE (front-right), SE (rear-right), SW (rear-left), NW (front-left)

## Troubleshooting

**Connection refused:**
- Make sure bridge is started before INAV SITL
- Check if port 2323 is available: `netstat -an | grep 2323`

**Quad not responding to INAV commands:**
- Check motor mapping matches your INAV configuration
- Verify motor mixing in INAV CLI: `mixer` and `mmix`
- Enable debug output in bridge to see received motor values

**Simulation too fast/slow:**
- Adjust `JSBSIM_DT` in bridge script
- Check CPU usage - simulation runs in real-time

## Requirements

```bash
pip install numpy jsbsim
```

## References

- JSBSim: https://github.com/JSBSim-Team/jsbsim
- INAV SITL: https://github.com/iNavFlight/inav/blob/master/docs/SITL/SITL.md
