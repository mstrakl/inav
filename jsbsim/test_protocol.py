#!/usr/bin/env python3
"""
Test script to verify JSBSim-INAV protocol without running full INAV.
Simulates what INAV SITL sends/receives.
"""

import socket
import time
import sys

SERVER_IP = "127.0.0.1"
SERVER_PORT = 2323

def test_connection():
    """Connect to JSBSim bridge and exchange test data."""
    
    print("Connecting to JSBSim bridge...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    try:
        sock.connect((SERVER_IP, SERVER_PORT))
        print(f"Connected to {SERVER_IP}:{SERVER_PORT}")
        
        # Disable Nagle's algorithm
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        
        rx_buffer = ""
        
        # Test motor commands: gradually increase throttle
        for i in range(100):
            # Simulate motor ramp-up
            throttle = 0.0 if i < 20 else min(0.3, (i - 20) * 0.01)
            
            # Send motor commands (4 motors, equal thrust)
            motor_msg = f"{throttle:.6f};{throttle:.6f};{throttle:.6f};{throttle:.6f};\n"
            sock.sendall(motor_msg.encode('utf-8'))
            
            # Receive state data
            data = sock.recv(4096)
            if data:
                rx_buffer += data.decode('utf-8', errors='ignore')
                
                # Process complete lines
                while '\n' in rx_buffer:
                    line, rx_buffer = rx_buffer.split('\n', 1)
                    
                    # Parse received data
                    parts = line.split(';')
                    if len(parts) >= 19:
                        try:
                            trel = float(parts[0])
                            lat = int(parts[1]) / 1e7
                            lon = int(parts[2]) / 1e7
                            alt = float(parts[3])
                            agl = float(parts[8])
                            gvel = float(parts[9])
                            roll = float(parts[10])
                            pitch = float(parts[11])
                            yaw = float(parts[12])
                            ax = float(parts[13])
                            ay = float(parts[14])
                            az = float(parts[15])
                            
                            print(f"[{trel:.2f}s] Throttle: {throttle:.2f}, "
                                  f"Alt: {agl:.1f}m, "
                                  f"Att: R={roll:.1f}° P={pitch:.1f}° Y={yaw:.1f}°, "
                                  f"Acc: [{ax:.2f}, {ay:.2f}, {az:.2f}]g")
                        except (ValueError, IndexError) as e:
                            print(f"Parse error: {e}")
            
            time.sleep(0.05)  # 20 Hz
            
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()
        print("Connection closed")


if __name__ == "__main__":
    print("=" * 60)
    print("JSBSim-INAV Protocol Test")
    print("=" * 60)
    print("Make sure jsbsim_inav_bridge.py is running first!")
    print()
    time.sleep(2)
    test_connection()
