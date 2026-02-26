import os, sys
sys.path.append(os.path.abspath("../"))  # Add parent directory to path for imports
import socket
import numpy as np
import jsbsim
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from src.tools import *
from src.inputs import *


def main():
    

    dt = 0.02
    

    PATH_TO_JSBSIM_FILES = "../run"
    sim = jsbsim.FGFDMExec(PATH_TO_JSBSIM_FILES)
    sim.set_debug_level(1)
    
    # ================================ #
    # Quadcopter Setup
    # ================================ #
    sim.load_model("./evis")          
    sim.set_dt(dt)

    # Set initial conditions - start in the air to avoid ground contact bounce
    sim["ic/lat-gc-deg"] = 46.6312
    sim["ic/long-gc-deg"] = 16.1769
    sim["ic/h-agl-ft"] = 100 
    sim["ic/vt-kts"] = 30.0 
    sim["ic/psi-true-deg"] = 0.0  # Initial heading (North)
    sim["ic/theta-deg"] = 0.0  # Level pitch
    sim["ic/phi-deg"] = 0.0    # Level roll

    # Important: Zero out all velocities to start in stable hover
    #sim["ic/u-fps"] = 0.0
    #sim["ic/v-fps"] = 0.0
    #sim["ic/w-fps"] = 0.0
    #sim["ic/p-rad_sec"] = 0.0
    #sim["ic/q-rad_sec"] = 0.0
    #sim["ic/r-rad_sec"] = 0.0

    # Initialize simulation
    sim.run_ic()
    
    
    
    while sim.get_sim_time() < 10.0:
        
        sim.run()  # Advance simulation by one time step

        if sim.get_sim_time() > 2.0:

            sim["fcs/aileron-cmd-norm"] = 0.0
            sim["fcs/elevator-cmd-norm"] = 0.0
            sim["fcs/rudder-cmd-norm"] = 0.0
        
        # Print every 0.5 seconds
        if sim.get_sim_time() < 0.025 or sim.get_sim_time() % 0.1 < 0.01:
        
            # Add basic state printing for debugging
            lat = sim["position/lat-gc-deg"]
            lon = sim["position/long-gc-deg"]
            alt = sim["position/h-sl-ft"]

            
            # Kinematics
            veln = sim["velocities/v-north-fps"]  # velocity north in feet per second
            vele = sim["velocities/v-east-fps"]  # velocity east in feet per second
            veld = sim["velocities/v-down-fps"]  # velocity down in feet per second
            gvel = np.sqrt(veln**2 + vele**2)
            roll = sim["attitude/phi-deg"]
            pitch = sim["attitude/theta-deg"]
            yaw = sim["attitude/psi-deg"]
            ax = sim["accelerations/a-pilot-x-ft_sec2"]  # acceleration in body x in feet per second squared
            ay = sim["accelerations/a-pilot-y-ft_sec2"]  # acceleration in body y in feet per second squared
            az = sim["accelerations/a-pilot-z-ft_sec2"]  # acceleration in body z in feet per second squared
            p = sim["velocities/p-rad_sec"]  # roll rate in radians per second
            q = sim["velocities/q-rad_sec"]  # pitch rate in radians per second
            r = sim["velocities/r-rad_sec"]  # yaw rate in radians per second   
            
            # Alpha / Beta
            alpha = sim["aero/alpha-deg"]
            beta = sim["aero/beta-deg"]
            
            
                        
            print(f"Time: {sim.get_sim_time():.2f} s")
            print(f"Lat: {lat:.4f} deg, Lon: {lon:.4f} deg, Alt: {alt:.2f} ft")
            print(f"Alpha: {alpha:.2f} deg, Beta: {beta:.2f} deg")
            print(f"Roll: {roll:.2f} deg, Pitch: {pitch:.2f} deg, Yaw: {yaw:.2f} deg")
            print(f"VelN: {veln:.2f} fps, VelE: {vele:.2f} fps, VelD: {veld:.2f} fps, GndSpd: {gvel:.2f} fps")
            print(f"AccX: {ax:.2f} ft/s², AccY: {ay:.2f} ft/s², AccZ: {az:.2f} ft/s²")
            print(f"p: {p:.2f} rad/s, q: {q:.2f} rad/s, r: {r:.2f} rad/s")  
        
            print("-" * 50)
            
if __name__ == "__main__":
    main()