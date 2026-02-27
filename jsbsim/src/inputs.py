
import os, sys
import numpy as np



def sim_cmd(t, sim, state, cmd):
    
    # Init
    state["ch5"] = -1.0
    state["ch6"] = -1.0
    state["ch7"] = -1.0
    state["ch8"] =  1.0

    # Acro / Angle mode
    state["ch6"] = 1.0

    # Arm
    if t > 6:
        state["ch5"] = 1.0 

    if t > 7:
        state["ch3"] = 0.75  

    # Transition
    if t > 10:
        state["ch8"] = 0.0

    # Fixed wing
    if t > 15:
        state["ch5"] = -1.0 
        state["ch8"] = -1.0


#    # Transition
#    if t > 10:
#        state["ch8"] = 0.0

#    # Fixed wing
#    if t > 15:
#        state["ch8"] = -1.0

#
#    state["ch7"] = -1.0
#
#    # Acro / Angle mode
#    state["ch6"] = 1.0
#    
#    if t > 8:
#        state["ch5"] = 1.0
#        
#        
#    if t > 9.0:
#        state["ch3"] = 0.75   
#
#    #WP ON
#    if t > 12.0:
#        state["ch7"] = 1.0
      
    return sim, state, cmd