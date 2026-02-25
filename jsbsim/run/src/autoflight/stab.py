import os, sys
import numpy as np

from src.data import Data as _Data
from src.utils import GeneralPID, RateLimiter

class FlCtrl:
    
    def __init__(self):
        self.pidRoll =  GeneralPID(0.0, 0.25, 0.10, 0.03, 0.25, dtermCutFreq=10.0)
        self.pidPitch = GeneralPID(0.0, 0.25, 0.10, 0.03, 0.25, dtermCutFreq=10.0)
        
        self.rlRollTarget = RateLimiter(10.0, 0.02)
        
    def update_outer(self, targetRelativeBearing):

        roll = np.interp(
            targetRelativeBearing,
            np.array([-60.0, 0.0, 60.0 ]),
            np.array([-30.0, 0.0, 30.0 ]),
        )

        return self.rlRollTarget(roll)

    def update(self, d: _Data, targetRelativeBearing=0):

        
        d.In.trgt_pitch.append(0)

        d.In.trgt_relbrg.append(targetRelativeBearing)
        
        d.In.trgt_roll.append( 
            self.update_outer(d.In.trgt_relbrg()) )

        d.In.ail.append(
            (1.0) * self.pidRoll(d.In.trgt_roll(),
                                 d.Out.roll(),
                                 d.Out.trel())
        )
        
        d.In.ele.append(
            (-1.0)  *self.pidPitch(d.In.trgt_pitch(),
                                   d.Out.pitch(),
                                   d.Out.trel())
        )
                        
        return d