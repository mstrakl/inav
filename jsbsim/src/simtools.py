import os, sys, time
import jsbsim
import numpy as np

from src.utils import GeneralPID, RateLimiter

def trim(sim, debug=False):
    
    trimPid = GeneralPID(0.0, 0.02, 0.05, 0.0, 0.0, dtermCutFreq=0.0)

    virtualTime = 0.0
    
    # Save original file descriptors
    old_stdout = os.dup(1)
    old_stderr = os.dup(2)
    
    # Redirect stdout and stderr to devnull
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 1)
    os.dup2(devnull, 2)
    
    try:
        for i in range(10001):
                
            sim.run_ic()  # Run initialize

            if debug:
                trim_out(sim, i)  
            
            sim["fcs/elevator-cmd-norm"] = -(1.0) * trimPid(
                0.0, sim["accelerations/qdot-rad_sec2"], virtualTime)


            virtualTime += 0.02
    finally:
        # Restore original file descriptors
        os.dup2(old_stdout, 1)
        os.dup2(old_stderr, 2)
        os.close(devnull)
        os.close(old_stdout)
        os.close(old_stderr)



def trim_out(sim, t_step):
    
    if (t_step) % 10 == 0:
    
        print("Trim step:", t_step)
        
        print("  V[km/h]:", round(sim["velocities/vtrue-kts"]*1.852,4))
        print("qbar[psf]:", round(sim["aero/qbar-psf"],4))
        print("    Alpha:", round(sim["aero/alpha-deg"],4))
        print("    C.Ail:", round(sim["fcs/left-aileron-pos-deg"],4))
        print("    C.Ele:", round(sim["fcs/elevator-pos-deg"],4))

        print("     pdot:", round(sim["velocities/vtrue-kts"]*1.852, 4))
        print("     qdot:", round(sim["accelerations/qdot-rad_sec2"], 4))
        print("     rdot:", round(sim["accelerations/rdot-rad_sec2"], 4))
        print("     udot:", round(sim["accelerations/udot-ft_sec2"], 4))
        print("     vdot:", round(sim["accelerations/vdot-ft_sec2"], 4))
        print("     wdot:", round(sim["accelerations/wdot-ft_sec2"], 4))
    
        #print("#-----------------------------------#")
        print()
        
        #time.sleep(0.2)


    
def debug_out(sim, t_step):
    
    if (t_step+1) % 50 == 0:
        
        print("Time         :", round(sim.get_sim_time(), 4))
        print("          Lat:", round(sim["position/lat-gc-deg"], 7))
        print("         Long:", round(sim["position/long-gc-deg"], 7))
        
        print("      V[km/h]:", round(sim["velocities/vtrue-kts"]*1.852,4))
        print("   Pitch[deg]:", round(sim["attitude/theta-deg"],4))
        print("    Roll[deg]:", round(sim["attitude/phi-deg"],4))
        print("        Alpha:", round(sim["aero/alpha-deg"],4))
        
        print("        C.Ail:", round(sim["fcs/left-aileron-pos-deg"],4))
        print("        C.Ele:", round(sim["fcs/elevator-pos-deg"],4))

        #print("#-----------------------------------#")
        print()
    