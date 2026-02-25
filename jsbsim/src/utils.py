

import numpy as np
import copy 

class SecondOrderLowPassFilter:
    def __init__(self, cutoff_frequency, quality_factor=0.707):
        self.cutoff_frequency = cutoff_frequency
        self.quality_factor = quality_factor

        # Initialize filter coefficients
        self.a0 = 1.0
        self.a1 = 0.0
        self.a2 = 0.0
        self.b0 = 0.0
        self.b1 = 0.0
        self.b2 = 0.0

        # Initialize the filter state
        self.previous_inputs = [0.0, 0.0]
        self.previous_outputs = [0.0, 0.0]
        self.last_time = None

    def update_coefficients(self, sampling_rate):
        omega = 2 * np.pi * self.cutoff_frequency / sampling_rate
        alpha = np.sin(omega) / (2 * self.quality_factor)

        self.a0 = 1 + alpha
        self.a1 = -2 * np.cos(omega)
        self.a2 = 1 - alpha
        self.b0 = (1 - np.cos(omega)) / 2
        self.b1 = 1 - np.cos(omega)
        self.b2 = (1 - np.cos(omega)) / 2

        # Normalize coefficients
        self.b0 /= self.a0
        self.b1 /= self.a0
        self.b2 /= self.a0
        self.a1 /= self.a0
        self.a2 /= self.a0

    def __call__(self, input_value, input_time):
        
        if self.cutoff_frequency <= 0.0:
            return input_value
        
        
        # Calculate the sampling rate based on input time
        if self.last_time is None:
            
            self.previous_inputs[1] = input_value
            self.previous_inputs[0] = input_value
            self.previous_outputs[1] = input_value
            self.previous_outputs[0] = input_value
            
            self.last_time = input_time
        
            return input_value
        
        delta_time = input_time - self.last_time
        sampling_rate = 1 / delta_time if delta_time > 0 else 1.0
        self.update_coefficients(sampling_rate)
        self.last_time = input_time

        # Apply the filter
        output_value = (self.b0 * input_value +
                        self.b1 * self.previous_inputs[0] +
                        self.b2 * self.previous_inputs[1] -
                        self.a1 * self.previous_outputs[0] -
                        self.a2 * self.previous_outputs[1])

        # Update state
        self.previous_inputs[1] = self.previous_inputs[0]
        self.previous_inputs[0] = input_value
        self.previous_outputs[1] = self.previous_outputs[0]
        self.previous_outputs[0] = output_value

        return output_value

    def reset(self):
        """Reset the filter state."""
        self.previous_inputs = [0.0, 0.0]
        self.previous_outputs = [0.0, 0.0]
        self.last_time = None
        
        
        
class RateLimiter:
    
    def __init__(self, max_rate_of_change: float, dt: float):
        self.max_rate = max_rate_of_change
        self.value = 0.0
        self.dt = dt
        
    def setMaxRate(self, rate):
        self.max_rate = rate

    def __call__(self, target_value: float) -> float:

        # Calculate allowed maximum change in value for this time step
        max_change = self.max_rate * self.dt

        # Calculate the desired change toward the target
        delta = target_value - self.value
        limited_delta = max(-max_change, min(delta, max_change))

        # Update the value with the limited delta
        self.value += limited_delta

        return self.value
        

class GeneralPID:
    
    def __init__( self, Kff, Kp, Ki, Kd, Kaw,
                  beta=1.0, gamma=1.0,
                  min=-1.0, max=1.0,
                  deadband=0.0,
                  dtermCutFreq=0.0 ):
        
        
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kff = Kff
        self.Kaw = Ki * Kaw
        
        self.beta = beta
        self.gamma = gamma

        self.eD_prev = 0.0
        self.t_curr = 0.0
        self.t_prev = -1.0
        
        self.termFF = 0.0
        self.termP = 0.0
        self.termI = 0.0
        self.termD = 0.0
        self.termAW = 0.0
        
        self.PV = 0.0
        self.SP = 0.0
        self.MV = 0.0
        
        self.umin = min
        self.umax = max       
        self.deadband = deadband
        
        self.dtermCutFreq = dtermCutFreq
        self.lpf = SecondOrderLowPassFilter(dtermCutFreq, 0.25)


    def __call__( self, _sp, PV, t ):
        
        # Initialize on first
        if self.t_prev == -1.0:
            self.t_prev = t
            return 0.0
        
        # Check positive time diff:
        if (t - self.t_prev) < 1.0e-9:
            return 0.0

        self.SP = _sp
        self.PV = PV
        self.t_curr = t

        if abs(_sp) < self.deadband:
            SP = 0.0
        else:
            SP = _sp
            
        
        self.calcFF(SP)
        
        self.calcP(SP, PV)
        
        self.calcI(SP, PV, t)
        
        self.calcD(SP, PV, t)
        
        # Requested manipulated variable
        req_MV = self.termFF + self.termP + self.termI + self.termD
        
        # Actual MV (within bounds)
        self.MV = min( self.umax, max( self.umin, req_MV ) )
        
        # Calculate anti windup term
        if self.Kaw != 0.0 and self.Ki != 0.0:
            self.calcAW( self.MV - req_MV )
        else:
            self.termAW = 0.0
        
        self.t_prev = t

        return self.MV
        
        
    def calcFF(self, SP):
        
        self.termFF = self.Kff * SP
        
    
    def calcP(self, SP, PV):
        
        self.termP = self.Kp*(self.beta*SP - PV)
        
    
    def calcI(self, SP, PV, t):
                
        #_I = self.Ki*(SP - PV) + self.termAW
        
        if self.termAW == 0.0:
            _I = self.Ki*(SP - PV)
        else:
            _I = 0
        
        self.termI += _I*(t - self.t_prev)

        
    def calcD(self, SP, PV, t):
        
        eD = self.gamma*SP - PV
        
        self.termD = self.Kd * (eD - self.eD_prev)/(t - self.t_prev)
        
        self.termD = self.lpf( self.termD, t  )

        self.eD_prev = eD
        
    
    def calcAW( self, u_err ):

        self.termAW = u_err * self.Kaw

    
    def resetIntegrator(self):
        
        self.termI = 0.0
        
    
    def debugPrint(self):
        
        print(f"   PID.tcurr={self.t_curr:.3f}")
        print(f"      PID.SP={self.SP:.3f}")
        print(f"      PID.PV={self.PV:.3f}")
        print(f"      PID.MV={self.MV:.3f}")
        
        print(f"  PID.termFF={self.termFF:.3f}")
        print(f"   PID.termP={self.termP:.3f}")
        print(f"   PID.termI={self.termI:.3f}")
        print(f"   PID.termD={self.termD:.3f}")
        print(f"  PID.termAW={self.termAW:.3f}")
        
        print("--------------------------------------")
        
    def reset(self):
        
        self.eD_prev = 0.0
        self.t_prev = -1.0
        
        self.termFF = 0.0
        self.termP = 0.0
        self.termI = 0.0
        self.termD = 0.0
        self.termAW = 0.0
        
        self.MV = 0.0
        
        self.lpf.reset()