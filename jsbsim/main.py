import os, sys
import numpy as np
import jsbsim
from src.simtools import trim, debug_out
from run.src.data import Data as _Data

jsbsim.FGJSBBase().debug_lvl = 0
PATH_TO_JSBSIM_FILES = "./run"
sim = jsbsim.FGFDMExec(PATH_TO_JSBSIM_FILES)  


Data = _Data()


def main():  

    # ================================ #
    # Mini Fox glider
    # ================================ #
    sim.load_model("./fox")          
    sim.set_dt(0.02)
    # ================================ #

    # Set initial conditions for level flight
    # LJMS, sredina modelarske piste
    sim["ic/lat-gc-deg"] = 46.16
    sim["ic/long-gc-deg"] = 15.65
    sim["ic/h-sl-ft"] = 200.0 # 200 ft agl
    sim["ic/vt-kts"] =  23        # Initial speed in knots
    sim["ic/psi-true-rad"] = np.deg2rad(200.0)       # Initial hdg
    
    # Trim the aircraft for steady flight at this initial condition
    sim["fcs/aileron-cmd-norm"] = 0.0    # Neutral aileron
    sim["fcs/elevator-cmd-norm"] = 0.0   # Neutral elevator
    sim["fcs/rudder-cmd-norm"] = 0.0     # Neutral rudder
    
    sim.run_ic()  # Run initialize
    
    trim(sim, debug=False)

    #sim["atmosphere/turb-type"] = 4
    #sim["atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps"] = 10
    #sim["atmosphere/turbulence/milspec/severity"] = 1.0
    
    i=0
    while sim.get_sim_time() <= 10.0:

        sim.run()  # Advance the simulation by one time step
        
        #if i == 10:
        #for p in sim.get_property_catalog():
        #    if p.startswith("aero") or p.startswith("moment") \
        #        or p.startswith("force") or p.startswith("ic"):
        #        try:
        #            xv = sim[p.split(" ")[0].strip()]
        #            print(f"{p:>30} := {xv:>16.4f}")
        #        except:
        #            print(f"{p} := no val")
        #print("#-------------------------------------------#")
        #sys.exit(1)

        Data.Out.read(sim)
        
        trel = sim.get_sim_time(),

        sim["fcs/aileron-cmd-norm"] = 0.0
        sim["fcs/elevator-cmd-norm"] = 0.0
        sim["fcs/speedbrake-cmd-norm"] = 0.0
        
        
        
        if (i+1) % 50 == 0:
            debug_out(sim, i)



        # End
        i += 1


    
if __name__ == "__main__":
    main()