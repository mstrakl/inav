
import os, sys
import numpy as np


def sim_cmd1(t, sim, state, cmd):

    # Init
    state["ch5"] = -1.0
    state["ch6"] = -1.0
    state["ch7"] = -1.0

    # Start as fixed wing in angle mode
    state["ch8"] = -1.0
    state["ch6"] = 1.0

    # Takeoff
    if t > 6:
        state["ch5"] = 1.0 

    if t > 7:
        state["ch3"] = 0.05

    if t > 10:
        state["ch8"] = 0.0

    if t > 20:
        state["ch8"] = 1.0

    return sim, state, cmd

def sim_cmd(t, sim, state, cmd):
    
    # Init
    state["ch5"] = -1.0
    state["ch6"] = -1.0
    state["ch7"] = -1.0

    # Start as fixed wing in angle mode
    state["ch8"] = -1.0
    state["ch6"] = 1.0

    # Takeoff
    if t > 6:
        state["ch5"] = 1.0 

    if t > 7:
        state["ch3"] = 0.75
        state["ch2"] = -0.5
    
    if t > 12:
        state["ch7"] = 1.0

    if t > 40:
        state["ch8"] = 1.0


    return sim, state, cmd



def sim_cmd2(t, sim, state, cmd):
    
    # Init
    state["ch5"] = -1.0
    state["ch6"] = -1.0
    state["ch7"] = -1.0

    # VTOL
    state["ch8"] = 1.0

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