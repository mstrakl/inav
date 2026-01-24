import os, sys, time
import socket

from rotorpy.environments import Environment
from rotorpy.wind.dryden_winds import DrydenGust
from rotorpy.vehicles.multirotor import Multirotor
from rotorpy.vehicles.hummingbird_params import quad_params
from rotorpy.sensors.imu import Imu

import numpy as np
from scipy.spatial.transform import Rotation as R
import src.sim_utils as sutil


quad_params['mass'] = 0.500      # kg
quad_params['Ixx']  = 3.65e-2    # kg*m^2
quad_params['Iyy']  = 3.68e-2    # kg*m^2
quad_params['Izz']  = 3.00e-2    # kg*m^2


class InavSimulate:
    
    def __init__(self, run_isolated=False):

        self.vehicle=Multirotor(quad_params)
        self.sim_state = self.vehicle.initial_state
        
        self.imu = Imu()
        self._enable_imu_noise = True  # Always add a bit of noise to avoid stale detection
        
        
        dt = 1.0 / 60.0
        avg_wind = np.array([2.0, 1.5, 0.0])  # Mean wind speed (m/s)
        sig_wind = np.array([2.7, 1.0, 0.5])  # Wind turbulence (m/s)
        altitude = 20  # Altitude (m)

        # Normal model
        self.wind = DrydenGust(dt=dt, 
                               avg_wind=avg_wind, 
                               sig_wind=sig_wind, 
                               altitude=altitude)

            
        # Socket receive buffer for handling partial/multiple messages
        self._rx_buffer = ""

        self.Nav = sutil.NavProj(
            46.631361,
            16.177206,
            185.0
        )
        
        
        self.cmd_motor_speeds = [
            0,0,0,0
        ]
        # targets are set from incoming commands; actual speeds follow with lag
        self.cmd_motor_targets = [0, 0, 0, 0]
        # motor lag time constant (seconds) for a first-order low-pass
        # smaller = faster response; realistic motors ~0.05-0.2s
        self.motor_time_constant = 0.10
        
        # Update once
        self.sim_state = self.vehicle.step(
            self.sim_state, {'cmd_motor_speeds': self.cmd_motor_speeds}, 0)
        
        self.state = {
            "trel": 0,
            
            "lat": self.Nav.lat0,
            "lon": self.Nav.lon0,
            "alt": self.Nav.alt0,
            "trk": 0,
            "hdg": 0,
            
            "posx": 0,
            "posy": 0,
            "posz": 0,
            "gvel": 0,
           
            "roll": 0,
            "pitch": 0,
            "yaw": 0,
            
            "ax": 0,
            "ay": 0,
            "az": 0,
            
            "p": 0,
            "q": 0,
            "r": 0,
            
            
            "ch1": 0,
            "ch2": 0,
            "ch3": 0,
            "ch4": 0,
            "ch5": 0,
            "ch6": 0,
            "ch7": 0,
            "ch8": 0,

        }
        
        self.msg = ""

        self.__moveDrone = False
        self.__isolatedRun = False # If no inav is online
        
        if run_isolated:
            self.__moveDrone = True
            self.__isolatedRun = True
                      
            init_vals = [475.0, 475.1, 475.1, 475.0]
            for i in range(4):
                self.cmd_motor_speeds[i] = init_vals[i]
                self.cmd_motor_targets[i] = init_vals[i]
            
        self.__lastTime = None
        
        # Initialize sensor error models (bias, noise, lag)
        self._init_sensor_models()
        
        
        

    def update(self, trel:float):
        
        if self.__lastTime is None:
            self.__lastTime = 0.0
        
        dt  = trel - self.__lastTime
        
        writeOutputState = False
        
        a_ned, omega_ned = np.zeros(3,), np.zeros(3,)
        #self.__moveDrone = True
        if self.__moveDrone or self.__isolatedRun:
            # apply motor lag filter before stepping the vehicle
            if dt > 0:
                self._apply_motor_lag(dt)
                
            self.sim_state["wind"] = self.wind.update(trel, self.sim_state["x"])

            self.sim_state = self.vehicle.step(self.sim_state, {'cmd_motor_speeds': self.cmd_motor_speeds}, dt)
            a_ned, omega_ned = self._imu(self.sim_state, self.vehicle.s_dot)
            
        
        self.sim_state["x"][2] = max(self.sim_state["x"][2], 0.0)  # Don't go below ground
        state = self.sim_state

        lat, lon, alt, trk = self.Nav.to_geodetic(
            state["x"][0], 
            state["x"][1],
            state["x"][2]
        )

        gvel = (state["v"][0]**2 + state["v"][1]**2)**0.5
                
        r = R.from_quat(state["q"])
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        

        # Update state for comms 
        # --------------------------------------- #
        
        self.__updateState("trel",  trel)
        
        T_ARM = 15.0
        
        if not self.__isolatedRun:
            
            self.__updateState("ch3",  0)
            self.__updateState("ch5",  0)
            self.__updateState("ch6", -1)
            
            
            if trel > T_ARM - 1.0:
                self.__moveDrone = True
                
            if trel > T_ARM - 0.8:
                writeOutputState = True
            
            if trel > T_ARM:
                self.__updateState("ch5",  1.0) # Arm
            
            if trel > T_ARM + 1:
                self.__updateState("ch3",  0.85) # Add power

            if trel > T_ARM + 4:
                self.__updateState("ch6",  0.75) # Angle mode + Alt Hold
                #self.__updateState("ch2",  0.5)  # Tilt forward to get some momentum
                
            if trel > T_ARM + 11:
                self.__updateState("ch2",  0.0) 
                
            
            # Switch to WP mode
            #if trel > T_ARM + 12:
            #    self.__updateState("ch6", -0.75)
            #    self.__updateState("ch7",  0.75) # WP Mode
            
            # Switch to POS HOLD 
            if trel > T_ARM + 12:
                self.__updateState("ch8",  0.75)

            if trel > T_ARM + 14:
                self.__updateState("ch2",  -0.25) 

            if trel > T_ARM + 20:
                self.__updateState("ch2",  0.0) 
                
        # Stop drone on landing
        if trel > T_ARM + 5 and state["x"][2] < 0.0:
            self.__moveDrone = False

        # Output States
        
        if writeOutputState:

            # Apply sensor bias/noise/lag to published sensor values
            lat_m, lon_m, alt_m, gvel_m, roll_m, pitch_m, a_ned_m, omega_ned_m = \
                self._apply_sensor_errors(trel, dt, lat, lon, alt, gvel, roll, pitch, a_ned, omega_ned)

            self.__updateState("lat", lat_m)
            self.__updateState("lon", lon_m)
            self.__updateState("alt", alt_m)

            # Don't update trk if too slow
            #
            if gvel_m > 1.0:
                self.__updateState("trk", trk)
                #self.__updateState("hdg", trk)

            # keep internal position (posx/posy/posz) as true simulated state
            self.__updateState("posx", state["x"][0])
            self.__updateState("posy", state["x"][1])
            self.__updateState("posz", state["x"][2])
            self.__updateState("gvel", gvel_m) 

            # Attitude: publish filtered/noisy roll/pitch; yaw/hdg remain true
            self.__updateState("roll", roll_m)
            self.__updateState("pitch", -pitch_m)
            self.__updateState("yaw", -yaw + 90.0)
            self.__updateState("hdg", -yaw + 90.0)

            # Accelerations: convert to g for published fields (apply sign conventions)
            self.__updateState("ax", a_ned_m[0] / 9.80665)
            self.__updateState("ay", -a_ned_m[1] / 9.80665)
            self.__updateState("az", -a_ned_m[2] / 9.80665)

            # Angular rates: publish in degrees/sec
            self.__updateState("p", omega_ned_m[0] * np.rad2deg(1.0))
            self.__updateState("q", omega_ned_m[1] * np.rad2deg(1.0))
            self.__updateState("r", omega_ned_m[2] * np.rad2deg(1.0))

            
        # Debug ------------------------- #

        #print("Time:", trel)
        #for k,v in state.items():
        #    if k != "q" and k != "rotor_speeds":
        #        sutil.print_vec(v, k)
        #
        #sutil.print_vec(np.array([roll, pitch, yaw]), "ypr")
        
        for k, v in self.state.items():
            print(f"{k:>12} = {v:12.6f}")

        for i, spd in enumerate(state["rotor_speeds"]):
            txt = f"motor{i}"
            print(f"{txt:>12} = {spd:12.6f}")
        print("# ----------------------------------- #")
        
        self.__lastTime = trel
        
        return self.state
    
    

    def __updateState(self, k, v, operation=""):
        
        if k not in self.state:
            raise RuntimeError (f"Key {k} not a member of self.state!")
        

        if operation == "":
            self.state[k] = v
        
        elif operation == "+":
            self.state[k] += v
            
        else:
            raise RuntimeError (f"Unknown operation: {operation}")
    
    
    def _imu(self, state, statedot):
        meas = self.imu.measurement(state, statedot, with_noise=self._enable_imu_noise)
        a_flu = meas["accel"]
        omega_flu = meas["gyro"]
        # FLU -> FRD
        a_frd = np.array([a_flu[0], -a_flu[1], -a_flu[2]], dtype=float)
        omega_frd = np.array([omega_flu[0], -omega_flu[1], -omega_flu[2]], dtype=float)
        return a_frd, omega_frd


    def _init_sensor_models(self):
        """Initialize sensor parameters and filter states."""
        # Per-sensor parameters (reasonable defaults, can be tuned)
        # Accelerometers: bias (g), noise_std (g), lag tau (s)
        self.sensor_params = {
            'accel': {
                'bias': np.array([0.01, -0.01, 0.04], dtype=float),
                'noise_std': np.array([0.02, 0.001, 0.005], dtype=float),
                'tau': 0.02
            },
            # Gyros: bias (rad/s), noise_std (rad/s), tau (s)
            'gyro': {
                'bias': np.array([0.002, 0.005, -0.010], dtype=float),
                'noise_std': np.array([0.005, 0.005, 0.005], dtype=float),
                'tau': 0.02
            },
            # Attitude (roll, pitch) in degrees: bias (deg), noise_std (deg), tau (s)
            'att': {
                'bias': np.array([0.0, 0.0], dtype=float),
                'noise_std': np.array([0.2, 0.2], dtype=float),
                'tau': 1.0
            },
            # GPS position noise and bias provided in meters (lat, lon, alt)
            'gps_pos': {
                'bias_m': np.array([0.0, 0.0, 0.0], dtype=float),
                'noise_std_m': np.array([5.0, 5.0, 10.0], dtype=float),
                'tau': 2.0
            },
            # GPS speed (m/s)
            'gps_speed': {
                'bias': 0.0,
                'noise_std': 1.0,
                'tau': 0.5
            }
        }

        # Filtered sensor state (stores previous filtered values)
        self._sensor_state = {}


    def _apply_filter(self, name, target, tau, dt):
        """First-order low-pass filter applied to scalar or vector target.

        Returns the filtered value and updates internal state.
        """
        if isinstance(target, np.ndarray):
            target = target.astype(float)
        else:
            # make scalar into float
            try:
                target = float(target)
            except Exception:
                target = np.array(target, dtype=float)

        prev = self._sensor_state.get(name, None)
        if prev is None:
            # Initialize filter state to the first target value
            self._sensor_state[name] = target
            return target

        if dt <= 0 or tau <= 1e-9:
            self._sensor_state[name] = target
            return target

        alpha = 1.0 - np.exp(-dt / float(tau))

        # support both scalar and numpy arrays
        new = prev + (target - prev) * alpha
        self._sensor_state[name] = new
        return new


    def _apply_sensor_errors(self, trel, dt, lat, lon, alt, gvel, roll, pitch, a_ned, omega_ned):
        """Apply bias, noise and lag to sensors and return measured values.

        Inputs:
          dt - time step (s)
          lat, lon (deg), alt (m)
          gvel (m/s)
          roll, pitch (deg)
          a_ned (3,) m/s^2
          omega_ned (3,) rad/s

        Returns tuple with same order but with applied sensor errors.
        """
        # Accelerometers
        ap = self.sensor_params['accel']
        a_noise = np.random.normal(0.0, ap['noise_std'])
        a_target = np.array(a_ned, dtype=float) + ap['bias'] + a_noise
        a_meas = self._apply_filter('accel', a_target, ap['tau'], dt)

        # Gyros
        gp = self.sensor_params['gyro']
        g_noise = np.random.normal(0.0, gp['noise_std'])
        g_target = np.array(omega_ned, dtype=float) + gp['bias'] + g_noise
        g_meas = self._apply_filter('gyro', g_target, gp['tau'], dt)

        # Attitude (roll, pitch) in degrees
        ap_att = self.sensor_params['att']
        att_vec = np.array([roll, pitch], dtype=float)
        att_noise = np.random.normal(0.0, ap_att['noise_std'])
        att_target = att_vec + ap_att['bias'] + att_noise
        att_meas = self._apply_filter('att', att_target, ap_att['tau'], dt)
        
        att_meas[0] += np.sin(np.deg2rad(trel*0.1)) * 1.0
        att_meas[1] += np.sin(np.deg2rad(trel*0.1)) * 2.0

        # GPS position: convert meter-level noise to lat/lon degrees
        gp_pos = self.sensor_params['gps_pos']
        pos_noise_m = np.random.normal(0.0, gp_pos['noise_std_m'])
        pos_bias_m = gp_pos['bias_m']
        # meters -> degrees approx: lat ~ 111320 m per deg, lon scaled by cos(lat)
        meters_to_deg_lat = 1.0 / 111320.0
        meters_to_deg_lon = 1.0 / (111320.0 * float(np.cos(np.deg2rad(self.Nav.lat0))))

        dlat = (pos_bias_m[0] + pos_noise_m[0]) * meters_to_deg_lat
        dlon = (pos_bias_m[1] + pos_noise_m[1]) * meters_to_deg_lon
        dalt = pos_bias_m[2] + pos_noise_m[2]

        pos_target = np.array([lat + dlat, lon + dlon, alt + dalt], dtype=float)
        pos_meas = self._apply_filter('gps_pos', pos_target, gp_pos['tau'], dt)

        # GPS speed
        sp = self.sensor_params['gps_speed']
        s_noise = np.random.normal(0.0, sp['noise_std'])
        s_target = float(gvel) + sp['bias'] + s_noise
        s_meas = self._apply_filter('gps_speed', s_target, sp['tau'], dt)

        return pos_meas[0], pos_meas[1], pos_meas[2], s_meas, float(att_meas[0]), float(att_meas[1]), a_meas, g_meas
    
    
    
    def rx(self, conn:socket.socket):
        """
        Receive motor commands from INAV via socket.
        Uses a buffer to handle partial messages and ensure we process complete lines.
        """
        # Make socket non-blocking to avoid hanging
        conn.setblocking(False)
        
        try:
            # Try to receive data
            data = conn.recv(1024)
            if data:
                self._rx_buffer += data.decode("utf-8", errors="ignore")
                #print(f"Received data: {data}")
        except BlockingIOError:
            # No data available, that's ok
            pass
        except Exception as e:
            print(f"Error receiving data: {e}")
            return
        
        # Process complete lines from buffer
        while "\n" in self._rx_buffer:
            line, self._rx_buffer = self._rx_buffer.split("\n", 1)
            line = line.strip()

            if not line:
                continue
            
            GAIN = 800.0
            vals = line.split(";")
            
            if len(vals) >= 4:
                try:
                    # set targets; actual speeds will lag via filter
                    self.cmd_motor_targets[0] = GAIN * float(vals[3])   # Motor FL
                    self.cmd_motor_targets[1] = GAIN * float(vals[1])   # Motor FR
                    self.cmd_motor_targets[2] = GAIN * float(vals[0])   # Motor RR
                    self.cmd_motor_targets[3] = GAIN * float(vals[2])   # Motor RL
                    
                    #self.cmd_motor_speeds[0] *= 1.000   
                    #self.cmd_motor_speeds[1] *= 1.000     
                    #self.cmd_motor_speeds[2] *= 1.000    
                    #self.cmd_motor_speeds[3] *= 1.000
                except (ValueError, IndexError) as e:
                    print(f"Error parsing motor speeds: {e}, line: {line}")   

    def _apply_motor_lag(self, dt: float):
        """First-order low-pass on motor speeds towards targets.

        Implements: d/dt x = (target - x) / tau  => discrete: x += (target-x) * (1 - exp(-dt/tau))
        """
        if dt <= 0:
            return

        tau = max(1e-6, float(self.motor_time_constant))
        alpha = 1.0 - np.exp(-dt / tau)
        for i in range(4):
            cur = float(self.cmd_motor_speeds[i])
            tgt = float(self.cmd_motor_targets[i])
            cur += (tgt - cur) * alpha
            self.cmd_motor_speeds[i] = cur
    
    
    def tx(self, conn:socket.socket):
        
        def appendToMsg(key, override=None):
            val = self.state[key]
            
            if override is not None:
                val = float(override)
            self.msg += f"{val:.6f};"
            
            
        self.msg = ""
        
        
        appendToMsg("trel")         #0        
        appendToMsg("lat")          #1
        appendToMsg("lon")          #2
        appendToMsg("alt")          #3
        appendToMsg("trk")          #4
        appendToMsg("hdg")          #5
        appendToMsg("posx")         #6
        appendToMsg("posy")         #7
        appendToMsg("posz")         #8
        appendToMsg("gvel")         #9
        appendToMsg("roll")         #10
        appendToMsg("pitch")        #11
        appendToMsg("yaw")          #12
        
        appendToMsg("ax")           #13
        appendToMsg("ay")           #14
        appendToMsg("az")           #15
        appendToMsg("p")            #16
        appendToMsg("q")            #17
        appendToMsg("r")            #18
        
        
        
        # Channels
        appendToMsg("ch1")          #19
        appendToMsg("ch2")          #20              
        appendToMsg("ch3")          #21
        appendToMsg("ch4")          #22
        appendToMsg("ch5")          #23
        appendToMsg("ch6")          #24
        appendToMsg("ch7")          #25
        appendToMsg("ch8")          #26
            

        # Finish line
        msg = self.msg.rsplit(";", 1)[0] + "\n"
        #print("msg:", msg)

        conn.sendall(msg.encode("utf-8"))