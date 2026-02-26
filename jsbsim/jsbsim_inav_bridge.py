#!/usr/bin/env python3
"""
JSBSim to INAV SITL Bridge

This script creates a socket interface between JSBSim and INAV SITL (adumsim.c).
It receives motor commands from INAV and sends back sensor/state data.

Protocol (from adumsim.c):
- INAV sends: "motor0;motor1;motor2;motor3;\n" (4 floats, 0-1 range)
- INAV expects to receive semicolon-separated values in this order:
  0:trel, 1:lat*1e7, 2:lon*1e7, 3:alt_msl(m), 4:track(deg), 5:hdg(deg),
  6:posx(m), 7:posy(m), 8:agl(m), 9:gspeed(m/s),
  10:roll(deg), 11:pitch(deg), 12:yaw(deg),
  13-15:acc_xyz(g), 16-18:gyro_xyz(deg/s),
  19-34:channels 1-16 (as floats -1 to 1 or 0 to 1)
"""

import socket
import sys
import time
import numpy as np
import jsbsim

# Configuration
LISTEN_IP = "127.0.0.1"
LISTEN_PORT = 2323  # Default ADUM_XP_PORT from adumsim.c
JSBSIM_DT = 0.01  # 100 Hz

# JSBSim setup
jsbsim.FGJSBBase().debug_lvl = 0
PATH_TO_JSBSIM_FILES = "./run"
sim = jsbsim.FGFDMExec(PATH_TO_JSBSIM_FILES)


def setup_jsbsim():
    """Initialize JSBSim with quadcopter model."""
    print("Loading JSBSim quadcopter model...")
    sim.load_model("./quad")
    sim.set_dt(JSBSIM_DT)
    
    # Initial conditions - start hovering in air
    sim["ic/lat-gc-deg"] = 46.16
    sim["ic/long-gc-deg"] = 15.65
    sim["ic/h-agl-ft"] = 10.0  # 10 ft AGL
    sim["ic/vt-kts"] = 0.0
    sim["ic/psi-true-deg"] = 0.0
    sim["ic/theta-deg"] = 0.0
    sim["ic/phi-deg"] = 0.0
    
    # Zero all velocities
    sim["ic/u-fps"] = 0.0
    sim["ic/v-fps"] = 0.0
    sim["ic/w-fps"] = 0.0
    sim["ic/p-rad_sec"] = 0.0
    sim["ic/q-rad_sec"] = 0.0
    sim["ic/r-rad_sec"] = 0.0
    
    sim.run_ic()
    
    # Initialize motors to zero
    sim["fcs/ne_motor"] = 0.0
    sim["fcs/se_motor"] = 0.0
    sim["fcs/sw_motor"] = 0.0
    sim["fcs/nw_motor"] = 0.0
    
    print("JSBSim initialized")


def get_jsbsim_state():
    """Extract state data from JSBSim in the format expected by adumsim.c."""
    FT_TO_M = 0.3048
    G = 9.80665
    
    state = {}
    
    # Time
    state["trel"] = sim.get_sim_time()
    
    # Position
    state["lat"] = sim["position/lat-gc-deg"]
    state["lon"] = sim["position/long-gc-deg"]
    state["alt"] = sim["position/h-sl-ft"] * FT_TO_M  # MSL altitude in meters
    state["agl"] = sim["position/h-agl-ft"] * FT_TO_M  # AGL in meters
    
    # Velocities
    vn = sim["velocities/v-north-fps"] * FT_TO_M
    ve = sim["velocities/v-east-fps"] * FT_TO_M
    state["gvel"] = np.sqrt(vn**2 + ve**2)
    
    # Attitude (degrees)
    state["roll"] = sim["attitude/phi-deg"]
    state["pitch"] = sim["attitude/theta-deg"]
    state["yaw"] = sim["attitude/psi-deg"]
    state["hdg"] = sim["attitude/psi-deg"]
    state["trk"] = sim["attitude/psi-deg"]
    
    # Accelerations (in 'g' units)
    state["ax"] = sim["accelerations/a-pilot-x-ft_sec2"] * FT_TO_M / G
    state["ay"] = sim["accelerations/a-pilot-y-ft_sec2"] * FT_TO_M / G
    state["az"] = sim["accelerations/a-pilot-z-ft_sec2"] * FT_TO_M / G
    
    # Angular rates (deg/s)
    state["p"] = np.degrees(sim["velocities/p-rad_sec"])
    state["q"] = np.degrees(sim["velocities/q-rad_sec"])
    state["r"] = np.degrees(sim["velocities/r-rad_sec"])
    
    # Local positions (meters) - not used by adumsim but required in protocol
    state["posx"] = 0.0
    state["posy"] = 0.0
    state["posz"] = state["agl"]
    
    return state


def apply_motor_commands(motors):
    """
    Apply motor commands to JSBSim.
    
    INAV motor order (from adumsim.c debug): motor[0], motor[1], motor[2], motor[3]
    Need to map to JSBSim quad X configuration.
    
    Typical mapping:
    INAV: motor[0]=RR, motor[1]=FR, motor[2]=RL, motor[3]=FL
    JSBSim: ne(front-right), se(rear-right), sw(rear-left), nw(front-left)
    """
    # Map INAV motors to JSBSim (adjust based on your actual configuration)
    # Assuming: motor[0]=RR, motor[1]=FR, motor[2]=RL, motor[3]=FL
    sim["fcs/ne_motor"] = motors[1]  # Front-Right (motor 1)
    sim["fcs/se_motor"] = motors[0]  # Rear-Right (motor 0)
    sim["fcs/sw_motor"] = motors[2]  # Rear-Left (motor 2)
    sim["fcs/nw_motor"] = motors[3]  # Front-Left (motor 3)


def build_message(state):
    """Build message string in format expected by adumsim.c."""
    # Convert lat/lon to int32 (multiply by 1e7)
    lat_1e7 = int(state["lat"] * 1e7)
    lon_1e7 = int(state["lon"] * 1e7)
    
    # Build message with all required fields
    # Format matches adumsim.c parsing (indices 0-34)
    msg = (
        f"{state['trel']:.6f};"      # 0: trel
        f"{lat_1e7};"                 # 1: lat*1e7
        f"{lon_1e7};"                 # 2: lon*1e7
        f"{state['alt']:.6f};"        # 3: altitude MSL (m)
        f"{state['trk']:.6f};"        # 4: track (deg)
        f"{state['hdg']:.6f};"        # 5: heading (deg)
        f"{state['posx']:.6f};"       # 6: pos x (m)
        f"{state['posy']:.6f};"       # 7: pos y (m)
        f"{state['agl']:.6f};"        # 8: altitude AGL (m)
        f"{state['gvel']:.6f};"       # 9: ground speed (m/s)
        f"{state['roll']:.6f};"       # 10: roll (deg)
        f"{state['pitch']:.6f};"      # 11: pitch (deg)
        f"{state['yaw']:.6f};"        # 12: yaw (deg)
        f"{state['ax']:.6f};"         # 13: accel x (g)
        f"{state['ay']:.6f};"         # 14: accel y (g)
        f"{state['az']:.6f};"         # 15: accel z (g)
        f"{state['p']:.6f};"          # 16: gyro x (deg/s)
        f"{state['q']:.6f};"          # 17: gyro y (deg/s)
        f"{state['r']:.6f};"          # 18: gyro z (deg/s)
    )
    
    # Add 16 RC channels (19-34) - all zeros for now
    for i in range(16):
        msg += f"0.0;"
    
    msg = msg.rstrip(';') + '\n'
    return msg


def run_server():
    """Main server loop - listens for INAV connection and runs simulation."""
    
    # Setup JSBSim
    setup_jsbsim()
    
    # Create TCP server socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((LISTEN_IP, LISTEN_PORT))
    server_socket.listen(1)
    
    print(f"JSBSim-INAV Bridge listening on {LISTEN_IP}:{LISTEN_PORT}")
    print("Waiting for INAV SITL connection...")
    
    conn, addr = server_socket.accept()
    print(f"Connected to INAV SITL from {addr}")
    
    # Disable Nagle's algorithm for low latency
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    
    # Set socket to non-blocking mode
    conn.setblocking(False)
    
    rx_buffer = ""
    motor_commands = [0.0, 0.0, 0.0, 0.0]
    last_print_time = 0.0
    
    try:
        while True:
            # Try to receive motor commands from INAV
            try:
                data = conn.recv(1024)
                if data:
                    rx_buffer += data.decode('utf-8', errors='ignore')
                    
                    # Process complete lines
                    while '\n' in rx_buffer:
                        line, rx_buffer = rx_buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            # Parse motor commands: "m0;m1;m2;m3;"
                            parts = line.split(';')
                            if len(parts) >= 4:
                                try:
                                    for i in range(4):
                                        motor_commands[i] = float(parts[i])
                                except (ValueError, IndexError) as e:
                                    print(f"Error parsing motors: {e}")
                else:
                    # Connection closed
                    print("INAV disconnected")
                    break
                    
            except BlockingIOError:
                # No data available right now, that's OK
                pass
            except Exception as e:
                print(f"Receive error: {e}")
                break
            
            # Apply motor commands to JSBSim
            apply_motor_commands(motor_commands)
            
            # Run one JSBSim simulation step
            sim.run()
            
            # Get current state
            state = get_jsbsim_state()
            
            # Build and send message to INAV
            msg = build_message(state)
            try:
                conn.sendall(msg.encode('utf-8'))
            except Exception as e:
                print(f"Send error: {e}")
                break
            
            # Print status occasionally
            t = state['trel']
            if t - last_print_time >= 1.0:
                print(f"[{t:.1f}s] Alt: {state['agl']:.1f}m, "
                      f"Motors: [{motor_commands[0]:.2f}, {motor_commands[1]:.2f}, "
                      f"{motor_commands[2]:.2f}, {motor_commands[3]:.2f}], "
                      f"Roll: {state['roll']:.1f}°, Pitch: {state['pitch']:.1f}°")
                last_print_time = t
            
            # Sleep to maintain real-time simulation rate
            time.sleep(JSBSIM_DT)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        conn.close()
        server_socket.close()
        print("Server closed")


if __name__ == "__main__":
    print("=" * 60)
    print("JSBSim to INAV SITL Bridge")
    print("=" * 60)
    run_server()
