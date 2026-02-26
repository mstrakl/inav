"""
Example script demonstrating JSBSim state data extraction.

This script shows how to extract all the required state variables from JSBSim
and pack them into a dictionary matching the inav_interface.py format.
"""

import numpy as np
import jsbsim

jsbsim.FGJSBBase().debug_lvl = 0
PATH_TO_JSBSIM_FILES = "./run"
sim = jsbsim.FGFDMExec(PATH_TO_JSBSIM_FILES)


def get_jsbsim_state():
    """
    Extract state data from JSBSim and pack into dictionary.
    
    Returns dict with:
    - trel: relative time (seconds)
    - lat, lon: geodetic coordinates (degrees)
    - alt: altitude above sea level (meters)
    - trk, hdg: track and heading angles (degrees)
    - posx, posy, posz: local position (meters)
    - gvel: ground velocity (m/s)
    - roll, pitch, yaw: attitude angles (degrees)
    - ax, ay, az: accelerations in body frame (g units)
    - p, q, r: angular rates in body frame (deg/s)
    """
    # Conversion constants
    FT_TO_M = 0.3048
    G = 9.80665
    
    state = {}
    
    # Time (seconds)
    state["trel"] = sim.get_sim_time()
    
    # Position - Geodetic
    state["lat"] = sim["position/lat-gc-deg"]
    state["lon"] = sim["position/long-gc-deg"]
    state["alt"] = sim["position/h-sl-ft"] * FT_TO_M  # altitude in meters
    
    # Track and Heading (degrees)
    state["trk"] = sim["attitude/psi-deg"]  # track angle
    state["hdg"] = sim["attitude/psi-deg"]  # heading
    
    # Position - Local NED coordinates (meters)
    # Note: For accurate local positions, you'd need to integrate velocities
    # or convert from ECEF coordinates relative to a reference point
    state["posx"] = 0.0  # North position (needs integration or tracking)
    state["posy"] = 0.0  # East position (needs integration or tracking)
    state["posz"] = sim["position/h-agl-ft"] * FT_TO_M  # altitude AGL in meters
    
    # Ground velocity (m/s)
    vn = sim["velocities/v-north-fps"] * FT_TO_M
    ve = sim["velocities/v-east-fps"] * FT_TO_M
    state["gvel"] = np.sqrt(vn**2 + ve**2)
    
    # Attitude angles (degrees)
    state["roll"] = sim["attitude/phi-deg"]
    state["pitch"] = sim["attitude/theta-deg"]
    state["yaw"] = sim["attitude/psi-deg"]
    
    # Accelerations in body frame (in 'g' units)
    # Use pilot accelerations which represent what an IMU would measure (specific force)
    state["ax"] = sim["accelerations/a-pilot-x-ft_sec2"] * FT_TO_M / G
    state["ay"] = sim["accelerations/a-pilot-y-ft_sec2"] * FT_TO_M / G
    state["az"] = sim["accelerations/a-pilot-z-ft_sec2"] * FT_TO_M / G
    
    # Angular rates in body frame (deg/s)
    state["p"] = sim["velocities/p-deg_sec"]  # roll rate
    state["q"] = sim["velocities/q-deg_sec"]  # pitch rate
    state["r"] = sim["velocities/r-deg_sec"]  # yaw rate
    
    return state


def get_motor_info():
    """
    Extract motor information from JSBSim.
    
    Returns dict with motor commands (0-1), thrust (N), and estimated RPM for each motor.
    """
    # Conversion constants
    LBS_TO_N = 4.44822  # pounds-force to Newtons
    MAX_THRUST_LBS = 0.84  # from quad.xml
    MAX_THRUST_N = MAX_THRUST_LBS * LBS_TO_N
    
    # Estimate RPM range (typical small quad)
    MAX_RPM = 12000
    
    motors = {}
    motor_names = ['ne', 'se', 'sw', 'nw']
    
    for name in motor_names:
        cmd = sim[f"fcs/{name}_motor"]
        thrust_N = cmd * MAX_THRUST_N
        rpm = cmd * MAX_RPM
        
        motors[name] = {
            'command': cmd,
            'thrust_N': thrust_N,
            'thrust_lbs': cmd * MAX_THRUST_LBS,
            'rpm_est': rpm
        }
    
    motors['total'] = {
        'thrust_N': sum(m['thrust_N'] for m in motors.values()),
        'thrust_lbs': sum(m['thrust_lbs'] for m in motors.values())
    }
    
    return motors


def main():
    """Simple demonstration of state extraction."""
    
    # Load quadcopter model
    sim.load_model("./quad")
    sim.set_dt(0.01)  # 100 Hz
    
    # Set initial conditions
    sim["ic/lat-gc-deg"] = 46.16
    sim["ic/long-gc-deg"] = 15.65
    sim["ic/h-agl-ft"] = 100.0  # 100 ft AGL
    sim["ic/vt-kts"] = 0.0
    sim["ic/psi-true-deg"] = 45.0  # 45° heading
    sim["ic/theta-deg"] = 0.0
    sim["ic/phi-deg"] = 0.0
    
    # Initialize
    sim.run_ic()
    
    # Set some motor commands to make the quad move
    sim["fcs/ne_motor"] = 0.6
    sim["fcs/se_motor"] = 0.6
    sim["fcs/sw_motor"] = 0.6
    sim["fcs/nw_motor"] = 0.6
    
    print("=" * 70)
    print("JSBSim State Extraction Example")
    print("=" * 70)
    
    # Run simulation for 5 seconds
    while sim.get_sim_time() <= 5.0:
        sim.run()
        
        # Extract state every 1 second
        t = sim.get_sim_time()
        if t > 0 and abs(t - round(t)) < 0.01:
            state = get_jsbsim_state()
            motors = get_motor_info()
            
            print(f"\n--- Time: {state['trel']:.2f}s ---")
            print(f"Position:")
            print(f"  lat:  {state['lat']:12.6f} deg")
            print(f"  lon:  {state['lon']:12.6f} deg")
            print(f"  alt:  {state['alt']:12.3f} m")
            print(f"  posx: {state['posx']:12.3f} m")
            print(f"  posy: {state['posy']:12.3f} m")
            print(f"  posz: {state['posz']:12.3f} m")
            
            print(f"Velocity:")
            print(f"  gvel: {state['gvel']:12.3f} m/s")
            
            print(f"Attitude:")
            print(f"  roll:  {state['roll']:12.3f} deg")
            print(f"  pitch: {state['pitch']:12.3f} deg")
            print(f"  yaw:   {state['yaw']:12.3f} deg")
            print(f"  hdg:   {state['hdg']:12.3f} deg")
            print(f"  trk:   {state['trk']:12.3f} deg")
            
            print(f"Accelerations (g):")
            print(f"  ax: {state['ax']:12.6f}")
            print(f"  ay: {state['ay']:12.6f}")
            print(f"  az: {state['az']:12.6f}")
            
            print(f"Angular Rates (deg/s):")
            print(f"  p: {state['p']:12.3f}")
            print(f"  q: {state['q']:12.3f}")
            print(f"  r: {state['r']:12.3f}")
            
            print(f"Motors:")
            print(f"  NE: cmd={motors['ne']['command']:.3f}, thrust={motors['ne']['thrust_N']:6.2f}N, rpm≈{motors['ne']['rpm_est']:5.0f}")
            print(f"  SE: cmd={motors['se']['command']:.3f}, thrust={motors['se']['thrust_N']:6.2f}N, rpm≈{motors['se']['rpm_est']:5.0f}")
            print(f"  SW: cmd={motors['sw']['command']:.3f}, thrust={motors['sw']['thrust_N']:6.2f}N, rpm≈{motors['sw']['rpm_est']:5.0f}")
            print(f"  NW: cmd={motors['nw']['command']:.3f}, thrust={motors['nw']['thrust_N']:6.2f}N, rpm≈{motors['nw']['rpm_est']:5.0f}")
            print(f"  Total: {motors['total']['thrust_N']:.2f}N ({motors['total']['thrust_lbs']:.2f}lbs)")
    
    
    print("\n" + "=" * 70)
    print("Example complete!")
    print("=" * 70)


if __name__ == "__main__":
    main()
