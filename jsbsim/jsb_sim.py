import os, sys
import socket
import numpy as np
import jsbsim
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from src.tools import *
from src.inputs import *


class JsbSimulation:
    def __init__(self, dt):
        
        self.dt = dt
        
        jsbsim.FGJSBBase().debug_lvl = 0
        PATH_TO_JSBSIM_FILES = "./run"
        self.sim = jsbsim.FGFDMExec(PATH_TO_JSBSIM_FILES)
        
        # ================================ #
        # Quadcopter Setup
        # ================================ #
        self.sim.load_model("./quad")          
        self.sim.set_dt(self.dt)

        # Set initial conditions - start in the air to avoid ground contact bounce
        self.sim["ic/lat-gc-deg"] = 46.6312
        self.sim["ic/long-gc-deg"] = 16.1769
        self.sim["ic/h-agl-ft"] = 0.25  
        self.sim["ic/vt-kts"] = 0.0    # No initial velocity
        self.sim["ic/psi-true-deg"] = 90.0  # Initial heading (North)
        self.sim["ic/theta-deg"] = 0.0  # Level pitch
        self.sim["ic/phi-deg"] = 0.0    # Level roll

        # Important: Zero out all velocities to start in stable hover
        self.sim["ic/u-fps"] = 0.0
        self.sim["ic/v-fps"] = 0.0
        self.sim["ic/w-fps"] = 0.0
        self.sim["ic/p-rad_sec"] = 0.0
        self.sim["ic/q-rad_sec"] = 0.0
        self.sim["ic/r-rad_sec"] = 0.0


        # Initialize motors to zero BEFORE first run
        self.sim["fcs/ne_motor"] = 0.0
        self.sim["fcs/se_motor"] = 0.0
        self.sim["fcs/sw_motor"] = 0.0
        self.sim["fcs/nw_motor"] = 0.0

        # Initialize simulation
        self.sim.run_ic()
        
        # Debug
        if False:
            for obj in dir(self.sim):
                print(obj)
            
            for pp in self.sim.get_property_catalog():
                if "external" in pp:
                    print(pp, " ", self.sim.get_property_value(pp.split(" ")[0]))
            sys.exit(1)

        # ================================ #
        # State
        # ================================ #
        self.state = get_jsbsim_state(self.sim, {}, init=True)
        
        
        
        # ================================ #
        # Inav command channels
        # ================================ #
        self.cmd = {
            "s1": 0.0,
            "s2": 0.0,
            "s3": 0.0,
            "s4": 0.0,
            "s5": 0.0,
            "s6": 0.0,
            "s7": 0.0,
            "s8": 0.0,
        }
        
        
        # Data logging
        time_log = []
        alt_log = []
        roll_log = []
        pitch_log = []
        yaw_log = []
        vn_log = []
        ve_log = []
        vd_log = []
        motor_ne_log = []
        motor_se_log = []
        motor_sw_log = []
        motor_nw_log = []
        
        print("=" * 60)
        print("FLIGHT DEMONSTRATION")
        print("=" * 60)
        
        self.print_dt = 0.10
        self.throttle = 0.0
        

    def update(self, joystick_input):
        
        t = self.sim.get_sim_time()
        
        # Get current state
        alt_agl =self.sim["position/h-agl-ft"]
        roll =self.sim["attitude/phi-deg"]
        pitch =self.sim["attitude/theta-deg"]
        yaw =self.sim["attitude/psi-deg"]
        
        # Control logic for different phases
        roll_cmd = 0.0
        pitch_cmd = 0.0
        yaw_cmd = 0.0
        
        
#        # Wait on ground with motors off
#        if t < 1.0:
#            self.throttle = 0.0
#        
#        # Phase 0: Takeoff (1-3s) - Spin up motors and lift off
#        elif t < 5.0:
#            self.throttle += 0.01
#            if self.throttle > 0.3:
#                self.throttle = 0.3
#        
#        # End of demo
#        else:
#            throttle = 0.0
#            pass


        
        # Apply motor commands
        # --------------------------------- #
        self.sim, self.state = sim_cmd(t, self.sim, self.state)
        if t > 3.0:
            set_motor_commands(self.sim, self.cmd)

                
        # Run simulation step
        self.sim.run()
        
        # Extract complete state data
        self.state = get_jsbsim_state(self.sim, self.state)
                
        # Get motor information
        motor_info = get_motor_info(self.sim)
        
        # Print state data every 0.5 seconds for demonstration
        if t > 0 and abs(t - round(t / self.print_dt) * self.print_dt) < 0.01:
            print(f"\n--- State at t={t:.2f}s ---")
            print(f"  Position: lat={self.state['lat']:.6f}°, lon={self.state['lon']:.6f}°, alt={self.state['alt']:.2f}m")

            print(f"  Velocity (m/s): vn={self.state['veln']:.3f}, ve={self.state['vele']:.3f}, vd={self.state['veld']:.3f}")
            print(f"  Accel (g): ax={self.state['ax']:.3f}, ay={self.state['ay']:.3f}, az={self.state['az']:.3f}")
            print(f"  Attitude: roll={self.state['roll']:.2f}°, pitch={self.state['pitch']:.2f}°, yaw={self.state['yaw']:.2f}°")
            print(f"  Rates (°/s): p={self.state['p']:.2f}, q={self.state['q']:.2f}, r={self.state['r']:.2f}")
            
            print(f"  Motors:")
            
            print(f"    NE: cmd={motor_info['ne']['command']:.3f}, thrust={motor_info['ne']['thrust_N']:.2f}N, rpm≈{motor_info['ne']['rpm_est']:.0f}")
            #print(f"    NE: vector={self.sim['fcs/ne_motor/direction/x']:.0f}")
            print(f"    SE: cmd={motor_info['se']['command']:.3f}, thrust={motor_info['se']['thrust_N']:.2f}N, rpm≈{motor_info['se']['rpm_est']:.0f}")
            print(f"    SW: cmd={motor_info['sw']['command']:.3f}, thrust={motor_info['sw']['thrust_N']:.2f}N, rpm≈{motor_info['sw']['rpm_est']:.0f}")
            print(f"    NW: cmd={motor_info['nw']['command']:.3f}, thrust={motor_info['nw']['thrust_N']:.2f}N, rpm≈{motor_info['nw']['rpm_est']:.0f}")
            print(f"    Total thrust: {motor_info['total']['thrust_N']:.2f}N ({motor_info['total']['thrust_lbs']:.2f}lbs)")
            
        
#        
#        # Log data every 0.1 seconds
#        if len(time_log) == 0 or t - time_log[-1] >= 0.1:
#            time_log.append(t)
#            alt_log.append(alt_agl)
#            roll_log.append(roll)
#            pitch_log.append(pitch)
#            yaw_log.append(yaw)
#            vn_log.append(sim["velocities/v-north-fps"])
#            ve_log.append(sim["velocities/v-east-fps"])
#            vd_log.append(sim["velocities/v-down-fps"])
#            motor_ne_log.append(sim["fcs/ne_motor"])
#            motor_se_log.append(sim["fcs/se_motor"])
#            motor_sw_log.append(sim["fcs/sw_motor"])
#            motor_nw_log.append(sim["fcs/nw_motor"])
#    
#        print(f"\n[{sim.get_sim_time():.1f}s] Simulation Complete")
#        print("=" * 60)
#        
#        # Plot results
#        plot_results(time_log, alt_log, roll_log, pitch_log, yaw_log,
#                    vn_log, ve_log, vd_log, 
#                    motor_ne_log, motor_se_log, motor_sw_log, motor_nw_log)

    
    
    def getDt(self):
        return self.dt

    
    def rx(self, conn:socket.socket):

        # Make socket non-blocking to avoid hanging
        conn.setblocking(False)
        
        buffer = ""
        
        try:
            # Try to receive data
            data = conn.recv(1024)
            if data:
                buffer += data.decode("utf-8", errors="ignore")
                #print(f"Received data: {data}")
        except BlockingIOError:
            # No data available, that's ok
            pass
        except Exception as e:
            print(f"Error receiving data: {e}")
            return
        
        # Process complete lines from buffer
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()

            if not line:
                continue
            
            vals = line.split(";")
            
            if len(vals) >= 4:
                try:
                    self.cmd["s1"] = float(vals[0])
                    self.cmd["s2"] = float(vals[1])
                    self.cmd["s3"] = float(vals[2])
                    self.cmd["s4"] = float(vals[3])

                    #self.cmd_motor_targets[0] = GAIN * float(vals[3])   # Motor FL
                    #self.cmd_motor_targets[1] = GAIN * float(vals[1])   # Motor FR
                    #self.cmd_motor_targets[2] = GAIN * float(vals[0])   # Motor RR
                    #self.cmd_motor_targets[3] = GAIN * float(vals[2])   # Motor RL

                except (ValueError, IndexError) as e:
                    print(f"Error parsing motor speeds: {e}, line: {line}")   


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
        
        #print("Ch1-5:", self.state["ch1"], self.state["ch2"], self.state["ch3"], self.state["ch4"], self.state["ch5"])

        conn.sendall(msg.encode("utf-8"))

        
        

def plot_results(time, alt, roll, pitch, yaw, vn, ve, vd, 
                 m_ne, m_se, m_sw, m_nw):
    """Plot flight data."""
    fig = plt.figure(figsize=(14, 10))
    gs = GridSpec(3, 2, figure=fig)
    
    # Altitude
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(time, alt, 'b-', linewidth=2)
    ax1.set_ylabel('Altitude AGL (ft)')
    ax1.set_xlabel('Time (s)')
    ax1.set_title('Altitude Profile')
    ax1.grid(True)
    
    # Attitudes
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(time, roll, 'r-', label='Roll', linewidth=2)
    ax2.plot(time, pitch, 'g-', label='Pitch', linewidth=2)
    #ax2.plot(time, yaw, 'b-', label='Yaw', linewidth=2)
    ax2.set_ylabel('Angle (deg)')
    ax2.set_xlabel('Time (s)')
    ax2.set_title('Attitude Angles')
    ax2.legend()
    ax2.grid(True)
    
    # Velocities
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(time, vn, 'r-', label='North', linewidth=2)
    ax3.plot(time, ve, 'g-', label='East', linewidth=2)
    ax3.plot(time, vd, 'b-', label='Down', linewidth=2)
    ax3.set_ylabel('Velocity (fps)')
    ax3.set_xlabel('Time (s)')
    ax3.set_title('Velocity Components')
    ax3.legend()
    ax3.grid(True)
    
    # Motor commands
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(time, m_ne, 'r-', label='NE (CW)', linewidth=2)
    ax4.plot(time, m_se, 'g-', label='SE (CCW)', linewidth=2)
    ax4.plot(time, m_sw, 'b-', label='SW (CW)', linewidth=2)
    ax4.plot(time, m_nw, 'k-', label='NW (CCW)', linewidth=2)
    ax4.set_ylabel('Motor Command (0-1)')
    ax4.set_xlabel('Time (s)')
    ax4.set_title('Motor Commands')
    ax4.legend()
    ax4.grid(True)
    
    # 2D trajectory
    ax5 = fig.add_subplot(gs[2, 0])
    pos_n = np.cumsum(np.array(vn)) * 0.1  # Approximate north position
    pos_e = np.cumsum(np.array(ve)) * 0.1  # Approximate east position
    ax5.plot(pos_e, pos_n, 'b-', linewidth=2)
    ax5.plot(pos_e[0], pos_n[0], 'go', markersize=10, label='Start')
    ax5.plot(pos_e[-1], pos_n[-1], 'ro', markersize=10, label='End')
    ax5.set_ylabel('North Position (ft)')
    ax5.set_xlabel('East Position (ft)')
    ax5.set_title('2D Trajectory')
    ax5.legend()
    ax5.grid(True)
    ax5.axis('equal')
    
    # 3D trajectory
    ax6 = fig.add_subplot(gs[2, 1], projection='3d')
    ax6.plot(pos_e, pos_n, alt, 'b-', linewidth=2)
    ax6.plot([pos_e[0]], [pos_n[0]], [alt[0]], 'go', markersize=10, label='Start')
    ax6.plot([pos_e[-1]], [pos_n[-1]], [alt[-1]], 'ro', markersize=10, label='End')
    ax6.set_xlabel('East (ft)')
    ax6.set_ylabel('North (ft)')
    ax6.set_zlabel('Altitude (ft)')
    ax6.set_title('3D Trajectory')
    ax6.legend()
    
    plt.tight_layout()
    plt.savefig('quadcopter_flight.png', dpi=150)
    print("\nPlot saved as 'quadcopter_flight.png'")
    plt.show()


#if __name__ == "__main__":
#    main()