
import numpy as np

class DataVector:
    
    def __init__(self, val=0.0):
        self.val = np.array([val])

    def append(self, x):
        self.val = np.append(self.val, x)
        
    def __call__(self, i=-1):
        if i > len(self.val)-1:
            raise RuntimeError(f"Index out of range, max is {len(self.val)-1}")
        return self.val[i]
    
    def all(self):
        return self.val
   

class DataIn:
    
    def __init__(self):
        
        self.rpNorth = DataVector()
        self.rpEast = DataVector()
        self.wpNorth = DataVector()
        self.wpEast = DataVector()
        self.wpDistTo = DataVector()
        self.wpHdgTo = DataVector()
        
        self.rollTarget = DataVector()
        self.pitchTarget = DataVector()
        self.modHdgErr = DataVector()
        self.rnedHdg = DataVector()
        self.rnedHdgTarget = DataVector()
        
        self.vs = DataVector()
        self.glid = DataVector()
        self.flex = DataVector()
        
        self.ail = DataVector()
        self.ele = DataVector()
        self.thr = DataVector()
        self.rud = DataVector()
        self.flp = DataVector()
        self.sbrk = DataVector()


class DataOut:
    
    def __init__(self):
        
        self.trel = DataVector()
        
        self.lat = DataVector()
        self.lon = DataVector()
        self.hdg = DataVector()
        self.trk = DataVector()
        self.msl = DataVector()
        self.agl = DataVector()
        self.vs = DataVector()
        self.gspd = DataVector()
        self.roll = DataVector()
        self.pitch = DataVector()
        self.p = DataVector()
        self.q = DataVector()
        self.r = DataVector()
        self.alpha = DataVector()
        
        #self.glid = DataVector()

    def read(self, prop):
        
        self.trel.append(prop["simulation/sim-time-sec"])
        
        self.lat.append(prop["position/lat-gc-deg"])
        self.lon.append(prop["position/long-gc-deg"])
        self.hdg.append(prop["attitude/heading-true-rad"] * np.rad2deg(1.0))
        self.trk.append(prop["flight-path/psi-gt-rad"] * np.rad2deg(1.0))
        self.msl.append(prop["position/h-sl-meters"])
        self.agl.append(prop["position/h-agl-km"] * 1000.0) # m
        self.vs.append(prop["velocities/h-dot-fps"] * 0.3048) # m/s
        self.gspd.append(prop["velocities/vg-fps"] * 0.3048) # m/s
        self.roll.append(prop["attitude/phi-deg"])
        self.pitch.append(prop["attitude/theta-deg"])
        self.p.append(prop["velocities/p-rad_sec"] * np.rad2deg(1.0)) # deg/s 
        self.q.append(prop["velocities/q-rad_sec"] * np.rad2deg(1.0)) # deg/s
        self.r.append(prop["velocities/r-rad_sec"] * np.rad2deg(1.0)) # deg/s   
        
        self.alpha.append(prop["aero/alpha-deg"])
        
        #self.glid.append(
        #    self.gspd() / self.vs()
        #)
        
        
        
class _CommonData:
    def __init__(self):
        pass
    
class Data:
    def __init__(self):
        self.In = DataIn()
        self.Out = DataOut()
        
    def _convertToCommon(self):
        
        cd = _CommonData()
        
        for obj in dir(self.In):
            if not obj.startswith("_"):
                setattr(cd, f"in_{obj}", getattr(self.In, obj))        
        
        for obj in dir(self.Out):
            if not obj.startswith("_"):
                setattr(cd, obj, getattr(self.Out, obj))     
        
        return cd