
import numpy as np

def set_motor_commands(sim, cmd):
    """
    Set motor commands in X configuration.
    
    Motor layout (looking from above):
    (1)NW(CCW)    NE(CW)(3)
             \    /
               \/
               /\
             /    \
    (2)SW(CW)     SE(CCW)(4)
        
    #     Roll   Pitch   Yaw
    1      -1      1      -1   
    2      -1     -1       1
    3       1      1       1
    4       1     -1      -1
    
    """
    
    sim["fcs/ne_motor"] = cmd["s3"]
    sim["fcs/se_motor"] = cmd["s4"]
    sim["fcs/sw_motor"] = cmd["s2"]
    sim["fcs/nw_motor"] = cmd["s1"]
    
#    sim["fcs/ne_motor"] = 0.40 #cmd["s3"]
#    sim["fcs/se_motor"] = 0.40 #cmd["s4"]
#    sim["fcs/sw_motor"] = 0.40 #cmd["s2"]
#    sim["fcs/nw_motor"] = 0.40 #cmd["s1"]
#    
#    if sim.get_sim_time() > 1.8:
#
#        sim["fcs/ne_motor"] = 0.40 * 1.00 #cmd["s3"]
#        sim["fcs/se_motor"] = 0.40 * 1.00 #cmd["s4"]
#        sim["fcs/sw_motor"] = 0.40 * 1.10 #cmd["s2"]
#        sim["fcs/nw_motor"] = 0.40 * 1.10 #cmd["s1"]
#

def get_jsbsim_state(sim, state, init=False):
    """
    Extract state data from JSBSim and pack into dictionary.
    
    Returns dict with:
    - Angles in degrees
    - Accelerations in 'g' (multiples of 9.80665 m/s²)
    - Rates in deg/s
    - Positions in meters
    - Ground velocity in m/s
    - Altitude in meters
    """
    # Convert feet to meters
    FT_TO_M = 0.3048
    # Convert knots to m/s
    KT_TO_MS = 0.514444
    # Gravity constant
    G = 9.80665
    
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
    # JSBSim doesn't directly provide NED, but we can use velocities integrated or ECEF conversion
    # For simplicity, using displacement from initial position
    state["posx"] = 0.0  # Will need to track this externally if needed
    state["posy"] = 0.0  # Will need to track this externally if needed
    state["posz"] = sim["position/h-agl-ft"] * FT_TO_M  # altitude AGL in meters
    
    # Ground velocity (m/s)
    vn = sim["velocities/v-north-fps"] * FT_TO_M
    ve = sim["velocities/v-east-fps"] * FT_TO_M
    state["gvel"] = np.sqrt(vn**2 + ve**2)
    
    # Attitude angles (degrees)
    state["roll"] = sim["attitude/phi-deg"]
    state["pitch"] = -sim["attitude/theta-deg"]
    state["yaw"] = sim["attitude/psi-deg"]
    
    # Accelerations in body frame (in 'g' units)
    # Use pilot accelerations which represent what an IMU would measure (specific force)
    # These are the accelerations felt in the body frame, excluding gravity
    state["ax"] = sim["accelerations/a-pilot-x-ft_sec2"] * FT_TO_M / G
    state["ay"] = - sim["accelerations/a-pilot-y-ft_sec2"] * FT_TO_M / G
    state["az"] = - sim["accelerations/a-pilot-z-ft_sec2"] * FT_TO_M / G
    
    # Angular rates in body frame (deg/s)
    state["p"] = np.degrees(sim["velocities/p-rad_sec"])  # roll rate
    state["q"] = np.degrees(sim["velocities/q-rad_sec"])  # pitch rate
    state["r"] = np.degrees(sim["velocities/r-rad_sec"])  # yaw rate
    
    if init:
        state["ch1"] = 0.0  
        state["ch2"] = 0.0  
        state["ch3"] = 0.0  
        state["ch4"] = 0.0  
        state["ch5"] = 0.0  
        state["ch6"] = 0.0  
        state["ch7"] = 0.0  
        state["ch8"] = 0.0  
        state["ch9"] = 0.0  
        state["ch10"] = 0.0 
        state["ch11"] = 0.0           
        state["ch12"] = 0.0 
        state["ch13"] = 0.0 
        state["ch14"] = 0.0 
        state["ch15"] = 0.0 
        state["ch16"] = 0.0 
    
    return state


def get_motor_info(sim):
    """
    Extract motor information from JSBSim.
    
    Returns dict with motor commands (0-1), thrust (N), and estimated RPM for each motor.
    
    Based on quad.xml: each motor generates thrust = motor_cmd * 0.84 lbs
    """
    # Conversion constants
    LBS_TO_N = 4.44822  # pounds-force to Newtons
    MAX_THRUST_LBS = 0.30  # from quad.xml
    MAX_THRUST_N = MAX_THRUST_LBS * LBS_TO_N
    
    # Estimate RPM range (typical small quad 5000-12000 RPM)
    MIN_RPM = 0
    MAX_RPM = 12000
    
    motors = {}
    
    # Read motor commands (0-1 normalized values)
    motor_names = ['ne', 'se', 'sw', 'nw']
    for name in motor_names:
        cmd = sim[f"fcs/{name}_motor"]
        
        # Calculate thrust in Newtons
        thrust_N = cmd * MAX_THRUST_N
        
        # Estimate RPM (linear approximation, actual relationship is more complex)
        # For more accuracy, use: RPM ≈ k * sqrt(thrust), but linear is simpler
        rpm = cmd * MAX_RPM
        
        motors[name] = {
            'command': cmd,           # 0-1 normalized command
            'thrust_N': thrust_N,     # Thrust in Newtons
            'thrust_lbs': cmd * MAX_THRUST_LBS,  # Thrust in pounds
            'rpm_est': rpm            # Estimated RPM (linear approximation)
        }
    
    # Calculate total thrust
    total_thrust_N = sum(m['thrust_N'] for m in motors.values())
    total_thrust_lbs = sum(m['thrust_lbs'] for m in motors.values())
    
    motors['total'] = {
        'thrust_N': total_thrust_N,
        'thrust_lbs': total_thrust_lbs,
        'avg_command': sum(m['command'] for m in motors.values()) / 4,
        'avg_rpm_est': sum(m['rpm_est'] for m in motors.values()) / 4
    }
    
    return motors


def sim_cmd(t, state):
    
    
    # Acro mode
    state["ch6"] = -0.8
    
    if t > 8.0:
        state["ch5"] = 1.0
    
    
    if t > 8.5:
        state["ch3"] = 0.5
        
    
    if t > 9.0:
        state["ch2"] = 0.05
        
    return state