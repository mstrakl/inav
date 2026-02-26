
import os, sys
import numpy as np



def sim_cmd(t, sim, state):
    
    
    # Acro / Angle mode
    state["ch6"] = 1.0
    
    if t > 7.5:
        state["ch5"] = 1.0
        
        
    if t > 8.0:
        state["ch3"] = 0.75       
    

    if t > 10.0:
        sim["propulsion/tilt-angle-rad"] = np.deg2rad(10.0)
        
#    # WP ON
#    if t > 10.0:
#        state["ch7"] = 1.0

   
#    
#    if t > 9:
#        state["ch2"] = 0.5
#
#
        
        
    return sim, state