
import os, sys



def sim_cmd(t, state):
    
    
    # Acro / Angle mode
    state["ch6"] = 1.0
    
    if t > 7.5:
        state["ch5"] = 1.0
        
        
    if t > 8.0:
        state["ch3"] = 0.75       
    

    # WP ON
    if t > 10.0:
        state["ch7"] = 1.0

   
#    
#    if t > 9:
#        state["ch2"] = 0.5
#
#
        
        
    return state