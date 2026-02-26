import os, sys, time
import socket

from rotorpy.wind.dryden_winds import DrydenGust
from rotorpy.vehicles.multirotor import Multirotor
from rotorpy.vehicles.hummingbird_params import quad_params
from rotorpy.sensors.imu import Imu

import numpy as np
from scipy.spatial.transform import Rotation as R
import src.sim_utils as sutil


quad_params['mass'] = 1.500      # kg
quad_params['Ixx']  = 8.65e-2    # kg*m^2
quad_params['Iyy']  = 11.68e-2   # kg*m^2
quad_params['Izz']  = 9.00e-2    # kg*m^2

SIMULATE_SENSOR_NOISE = False
SIMULATE_WIND = False

class InavSimulate:
    
    def __init__(self, run_isolated=False):

        self.vehicle=Multirotor(quad_params)
        self.sim_state = self.vehicle.initial_state
        
        self.imu = Imu()
        self._enable_imu_noise = True  # Always add a bit of noise to avoid stale detection
        
        
        dt = 1.0 / 60.0
        gain = 0.50
        avg_wind = np.array([0.71, 0.71, 0.0]) * gain  # Mean wind speed (m/s)
        sig_wind = np.array([0.7, 0.7, 0.5]) * gain  # Wind turbulence (m/s)
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
        self.motor_time_constant = 0.02
        
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
            "ch9": 0,
            "ch10": 0,
            "ch11": 0,
            "ch12": 0,
            "ch13": 0,
            "ch14": 0,
            "ch15": 0,
            "ch16": 0,
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
        
        self.__TARM = 9999999.0

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
    

    def update(self, trel:float, tx:dict):
        
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

            if SIMULATE_WIND:
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

        if not self.__isolatedRun:
            
            self.__updateState("ch3",  0)
            self.__updateState("ch5",  0)
            self.__updateState("ch6", -1)
            
            if (tx["swD"] == 2 and self.__TARM >= 9999998.0):
                self.__TARM = trel
            elif(tx["enable"] == False and self.__TARM >= 9999998.0):
                if trel > 7.0:
                    self.__TARM = trel + 1.0
            
            print("self.__TARM:", self.__TARM)
            
            if trel > self.__TARM - 1.0:
                self.__moveDrone = True
                
            if trel > self.__TARM - 0.8:
                writeOutputState = True
            
            if trel > self.__TARM + 1:
                self.__updateState("ch5",  1.0) # Arm
            
            if trel > self.__TARM + 2:
                self.__updateState("ch3",  0.99) # Add power

            if trel > self.__TARM + 5:
                self.__updateState("ch6",  0.75) # Angle mode + Alt Hold
                self.__updateState("ch2",  0.50)  # Tilt forward to get some momentum
                
            if trel > self.__TARM + 10:
                self.__updateState("ch2",  0.00)  
                
            # Switch to WP mode, also listen to joystick commands now
            if trel > self.__TARM + 12:
                self.__updateState("ch7",  0.75) # WP Mode
                
                if (tx["enable"]):
                    self.__updateState("ch10",  tx["potS1"]) 
                    self.__updateState("ch11",  tx["potS2"]) 
                    self.__updateState("ch12",  tx["swA"]) 

                else:
                    self.__updateState("ch12",  1.0)

#
#            # Wobble around for testing
#            if trel > self.__TARM + 45:
#                #val = np.sin((trel - (self.__TARM + 30)) * 0.5)
#                val = 1.0
#                self.__updateState("ch1",  val) 
#                
#                #val2 = np.sin((trel - (self.__TARM + 30)) * 0.5)
#                val2 = 0.0
#                self.__updateState("ch2",  val2) 
#            
#            
#            
#            # Wobble around for testing
#            if trel > self.__TARM + 50:
#                #val = np.sin((trel - (self.__TARM + 30)) * 0.5)
#                val = 0.0
#                self.__updateState("ch1",  val) 
#                
#                #val2 = np.sin((trel - (self.__TARM + 30)) * 0.5)
#                val2 = 0.0
#                self.__updateState("ch2",  val2) 
#                
                
                
            # Switch to POS HOLD 
            #if trel > self.__TARM + 12:
            #    self.__updateState("ch8",  0.75)

            #if trel > self.__TARM + 14:
            #    self.__updateState("ch2",  -0.25) 

            #if trel > self.__TARM + 20:
            #    self.__updateState("ch2",  0.0) 
                
        # Stop drone on landing
        if trel > self.__TARM + 5 and state["x"][2] < 0.0:
            self.__moveDrone = False

        # Output States
        
        if writeOutputState:

            self.__updateState("lat", lat)
            self.__updateState("lon", lon)
            self.__updateState("alt", alt)
                
            # Don't update trk if too slow
            #
            if gvel > 1.0:
                self.__updateState("trk", trk)
                #self.__updateState("hdg", trk)
            
            self.__updateState("posx", state["x"][0])
            self.__updateState("posy", state["x"][1])
            self.__updateState("posz", state["x"][2])
            self.__updateState("gvel", gvel) 

            self.__updateState("roll", roll)
            self.__updateState("pitch", -pitch)
            self.__updateState("yaw", -yaw + 90.0)
            self.__updateState("hdg", -yaw + 90.0)
            self.__updateState("ax", a_ned[0] / 9.80665)
            self.__updateState("ay", -a_ned[1] / 9.80665)
            self.__updateState("az", -a_ned[2] / 9.80665)
            self.__updateState("p", omega_ned[0] * np.rad2deg(1.0))
            self.__updateState("q", omega_ned[1] * np.rad2deg(1.0))
            self.__updateState("r", omega_ned[2] * np.rad2deg(1.0))

            
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
            
            GAIN = 1200.0
            vals = line.split(";")
            
            
            
            # Increase gains when close to ground, to simulate landing
            # for inav
            
            if self.state["trel"] > self.__TARM + 12:

                if self.state["posz"] < 5.0:
                    GAIN = np.interp(self.state["posz"], 
                                    [0.0, 5.0], 
                                    [GAIN * 2.0, GAIN])
                
                if GAIN > 1300.0:
                    print("Increasing gains:", GAIN)


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
                
            if key == "lat" or key == "lon":
                val_1e7 = int(val * 1e7)
                self.msg += f"{val_1e7};"
            else:   
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
           
        appendToMsg("ch9")          #27
        appendToMsg("ch10")         #28              
        appendToMsg("ch11")         #29
        appendToMsg("ch12")         #30
        appendToMsg("ch13")         #31
        appendToMsg("ch14")         #32
        appendToMsg("ch15")         #33
        appendToMsg("ch16")         #34

        # Finish line
        msg = self.msg.rsplit(";", 1)[0] + "\n"
        #print("msg:", msg)

        conn.sendall(msg.encode("utf-8"))