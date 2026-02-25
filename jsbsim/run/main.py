import os, sys
import numpy as np
import matplotlib.pyplot as plt
import jsbsim

from src.simplot import SimPlot
from src.data import Data as _Data
from src.autoflight.stab import FlCtrl as _FlCtrl
from src.simtools import *

jsbsim.FGJSBBase().debug_lvl = 0


Data = _Data()
FlCtrl = _FlCtrl()

PATH_TO_JSBSIM_FILES = "./"
sim = jsbsim.FGFDMExec(PATH_TO_JSBSIM_FILES)  


from src.autoflight.nav import *
# LJMS modelarska
# Thr (sredina)
# End (juzni konec)
p_thr = NavPnt(46.631344, 16.177207, 0)
p_end = NavPnt(46.631039, 16.177051, 0)

sys.exit(1)

dist, hdg = Navigation.calcDistBearing(p_thr, p_end)

geomPnts = [] 
geomPnts.append(GeomPnt(  50.0,  -25.0,   0.0))
geomPnts.append(GeomPnt(   0.0,  -50.0,   0.0))
geomPnts.append(GeomPnt( -80.0,  -50.0,   0.0))
geomPnts.append(GeomPnt(-100.0,  -25.0,   0.0))
geomPnts.append(GeomPnt( -80.0,    0.0,   0.0))
geomPnts.append(GeomPnt(   0.0,    0.0,   0.0))


InitPos = NavPnt(
    p_thr.lat,
    p_thr.lon,
    300.0 * 0.3048,
)

Dp = DeadStickPilot(RunwayFrame(p_thr, hdg), InitPos, geomPnts, 10.0, debug=True)

def main():  

    # ================================ #
    # Mini Fox glider
    # ================================ #
    sim.load_model("./fox")      
    sim.set_dt(0.02)
    # ================================ #

    # Set initial conditions for level flight
    # LJMS, sredina modelarske piste
    sim["ic/lat-gc-deg"] = InitPos.lat
    sim["ic/long-gc-deg"] = InitPos.lon
    sim["ic/h-sl-ft"] = InitPos.agl / 0.3048
    sim["ic/vt-kts"] =  23        # Initial speed in knots
    sim["ic/psi-true-rad"] = np.deg2rad(200)       # Initial hdg
    
    # Trim the aircraft for steady flight at this initial condition
    sim["fcs/aileron-cmd-norm"] = 0.0    # Neutral aileron
    sim["fcs/elevator-cmd-norm"] = 0.0   # Neutral elevator
    sim["fcs/rudder-cmd-norm"] = 0.0     # Neutral rudder
    
    sim.run_ic()  # Run initialize
    #sim.do_trim(1)
    
    trim(sim, debug=False)
    #sys.exit(1)

    #sim["atmosphere/turb-type"] = 4
    #sim["atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps"] = 10
    #sim["atmosphere/turbulence/milspec/severity"] = 1.0
    
    track =  []

    i=0
    i_pntold = 1
    while sim.get_sim_time() < 300.0:
        
        sim.run()  # Advance the simulation by one time step
        
        #if i == 10:
        #for p in sim.get_property_catalog():
        #    if p.startswith("aero") or p.startswith("moment") or p.startswith("force"):
        #        try:
        #            xv = sim[p.split(" ")[0].strip()]
        #            print(f"{p:>30} := {xv:>16.4f}")
        #        except:
        #            print(f"{p} := no val")
        #print("#-------------------------------------------#")
        #sys.exit(1)

        Data.Out.read(sim)
        
        # Loop autoflight
        
        pos = NavPnt(
            Data.Out.lat(),
            Data.Out.lon(),
            Data.Out.agl()
        )

        Dp.update(pos, Data)
        
        brg = Dp.getNavRelativeBearing()
        
        pos = Dp.getCurrentGeomPos()
        track.append([pos.xe, pos.xn])

        FlCtrl.update(Data, brg)
        
        sim["fcs/aileron-cmd-norm"] = Data.In.ail()
        sim["fcs/elevator-cmd-norm"] = Data.In.ele()

        debug_out(sim, i)
  
        if Data.Out.agl() < 1.0:
            break
        
        #if Dp.getCurrentPointIndex() == 4:
        #    break

        #if ( np.round(sim.get_sim_time(),2) % 1.0 < 0.01 ):
        #    print("waiting...")
        #    input()

        #if Dp.getCurrentPointIndex() != i_pntold:
        #    i_pntold = Dp.getCurrentPointIndex()
        #    input()
        #    break
            
        i += 1

    track = np.array(track)
    
    plt.plot(track[:,0], track[:,1])
    
    for gp in geomPnts:
        plt.plot(gp.xe, gp.xn, "o")
    
    
    plt.show()

    Plot = SimPlot(Data._convertToCommon())

    Plot.add(0, "roll")
    Plot.add(0, "in_trgt_roll")
    Plot.add(0, "in_ail", y2=True)

    Plot.add(1, "pitch")
    Plot.add(1, "in_ele", y2=True)
    
    Plot.add(2, "gspd")
    Plot.add(3, "hdg")

    Plot.add(4, "agl")
    Plot.add(4, "glid", y2=True)
    
    Plot.add(5, "alpha")

    Plot.plot()  
    



    
    
    
if __name__ == "__main__":
    main()