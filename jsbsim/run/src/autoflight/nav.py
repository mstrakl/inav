import os, sys, copy
import numpy as np
from src.data import Data as _Data
from src.utils import SecondOrderLowPassFilter

class NavPnt:
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.agl = alt

class GeomPnt:
    def __init__(self, xn, xe, alt):
        self.xn = xn
        self.xe = xe
        self.agl = alt

class RunwayFrame:
    
    def __init__(self, p_thr:NavPnt, r_hdg):
        self.rthr = p_thr
        self.rhdg = r_hdg
        
        # Rotation matrix
        self.Rmat = np.array([
            [np.cos(np.deg2rad(-self.rhdg)), -np.sin(np.deg2rad(-self.rhdg))],
            [np.sin(np.deg2rad(-self.rhdg)),  np.cos(np.deg2rad(-self.rhdg))],
        ])
    
    
    def convertVelocityHdg(self, umag, hdg):
        
        u_vec = np.zeros(2)
        u_vec[0] = umag * np.cos(np.deg2rad(hdg))
        u_vec[1] = umag * np.sin(np.deg2rad(hdg))
        #print(f"uv={u_vec[0]:.2f}, {u_vec[1]:.2f}")
        
        u_conv = np.matmul(self.Rmat, u_vec) 
        #print(f"uc={u_conv[0]:.2f}, {u_conv[1]:.2f}")
        
        un = u_conv[0]
        ue = u_conv[1]
        
        u_hdg = np.rad2deg(np.atan2(u_conv[1],u_conv[0]))

        return un, ue, u_hdg
    
    
    def convertTo(self, pnt:NavPnt):

        dist, hdg = Navigation.calcDistBearing(self.rthr, pnt)
        
        x_vec = np.zeros(2)
        x_vec[0] = dist * np.cos(np.deg2rad(hdg))
        x_vec[1] = dist * np.sin(np.deg2rad(hdg))

        
        x_conv = np.matmul(self.Rmat, x_vec) 
        return GeomPnt(x_conv[0], x_conv[1], pnt.agl)


    def convertFrom(self, pnt:GeomPnt):
        
        # Convert to Navigational Frame
        x_vec = np.array([pnt.xn, pnt.xe])
        
        hdg = np.arctan2(pnt.xe, pnt.xn)*np.rad2deg(1.0)
        dist = np.sqrt(pnt.xe**2+pnt.xn**2)

        x_conv = np.matmul(self.Rmat.transpose(), x_vec) 
        
        nfpnt = GeomPnt(x_conv[0], x_conv[1], pnt.agl)
                
        hdg = np.arctan2(nfpnt.xe, nfpnt.xn)*np.rad2deg(1.0)
        dist = np.sqrt(nfpnt.xe**2+nfpnt.xn**2)

        new_lat, new_lon = Navigation.calcCoordinates(self.rthr, dist, hdg)
        return NavPnt(new_lat, new_lon, pnt.agl)
        
    
class _Navigation:
    
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(_Navigation, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):
        self.__R = 6371.0 * 1000  # Radius of the Earth in meters

    def calcDistBearing(self, pnt1:NavPnt, pnt2:NavPnt):
        
        lat1, lon1 = np.radians(pnt1.lat), np.radians(pnt1.lon)
        lat2, lon2 = np.radians(pnt2.lat), np.radians(pnt2.lon)

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # Distance: Haversine formula
        # ----------------------------------------------- #
        a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

        distance = self.__R * c  # in meters

        # Bearing
        # ----------------------------------------------- #
        x = np.sin(dlon) * np.cos(lat2)
        y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)

        # Calculate the initial bearing
        initial_bearing = np.arctan2(x, y)

        # Convert bearing from radians to degrees
        initial_bearing = np.degrees(initial_bearing)

        # Normalize the bearing to be between 0° and 360°
        bearing = (initial_bearing + 360) % 360
        
        return distance, bearing
    
    def calcCoordinates(self, p_start: NavPnt, dist, hdg):
        
        hdg_rad = np.deg2rad(hdg)
        
        lat1 = np.deg2rad(p_start.lat)
        lon1 = np.deg2rad(p_start.lon)
        
        # Calculate the new coordinates
        lat2 = np.arcsin(np.sin(lat1) * np.cos(dist / self.__R) +
                         np.cos(lat1) * np.sin(dist / self.__R) * np.cos(hdg_rad))

        # Calculate the new longitude
        lon2 = lon1 + np.arctan2(np.sin(hdg_rad) * np.sin(dist / self.__R) * np.cos(lat1),
                                 np.cos(dist / self.__R) - np.sin(lat1) * np.sin(lat2))

        # Convert back to degrees
        lat2 = np.rad2deg(lat2)
        lon2 = np.rad2deg(lon2)

        return lat2, lon2


Navigation = _Navigation()


class DeadStickPilot:
    
    def __init__(self, Rwfr: RunwayFrame, InitPos: NavPnt, pnts: list, gr: float, debug=False):
        
        self.__debug = debug
        
        self.RwFr = Rwfr
        self.__unscaled_pnts = copy.deepcopy(pnts)
        self.__pnts = copy.deepcopy(pnts)
        self.__gr = gr
        self.__ipnt = 0
        self.__ipntlast = -1
        
        # Current Position in rwy frame
        self.rf_pos = GeomPnt(0,0,0)
        self.rf_un = 0.0
        self.rf_ue = 0.0
        self.rf_uhdg = 0.0
        
        # State variables
        self.fdist = 1.15
        self.__targetGeomBrg = 0.0 

        # Initialize traffic pattern scale 
        self.rf_pos = self.RwFr.convertTo(InitPos)
        
        # Insert as first point to pnt list
        self.__pnts.insert(0,self.rf_pos)
        self.__unscaled_pnts.insert(0,self.rf_pos)
        self.__ipnt = 1 # Navigate to pnt 1 first
        self.__pntsDone = False

        self.__totaldist = self.calcRemainingDistance()
        self.__range = self.calcDistRange()
        self.__tpscale = self.__range / self.__totaldist
        self.__grfilt = SecondOrderLowPassFilter(0.1)
        
        self.__grarr = []

        print(f"Initialized with: ")
        print(f"    -dist: {self.__totaldist:.2f} ")
        print(f"   -range: {self.__range:.2f} ")
        print(f" -tpscale: {self.__tpscale:.2f} ")
        print("#----------------------------------------#")
        #sys.exit(1)

    def update(self, _pos: NavPnt, data: _Data):
        
        gspd = data.Out.gspd()
        ghdg = data.Out.hdg()
        
        if data.Out.trel() > 5.0:
            self.__gr = self.__grfilt( np.maximum(0, (-1.0)*data.Out.glid()), data.Out.trel() )

        self.updateScaledPnts(data.Out.roll(), data.Out.trel())

        self.rf_pos = self.RwFr.convertTo(_pos)
        self.rf_un, self.rf_ue, self.rf_uhdg = self.RwFr.convertVelocityHdg(gspd, ghdg)
  
        self.__totaldist = self.calcRemainingDistance()
        self.__range = self.calcDistRange()
        
        # Vector operations of radial to point and current velocity vector
        # - fdot will be positive as long as we are tracking to point
        # - fdot goes negative when we track away from point
        # - fphi is relative bearing, negative left, positive right
        #
        fdot, self.__targetGeomBrg = self.calcVectorizeLeg()
        
        legFrac = self.calcProjectedPathFraction()
        
        # Distance and bearings to next point
        distToPnt = self._calcDist2Pnts(self.rf_pos, self.__pnts[self.__ipnt])
        brgToPnt = self._calcBearing2Pnts(self.rf_pos, self.__pnts[self.__ipnt])
        
        
        if self.__debug:
            
            print(f"                 Sim.Time: {data.Out.trel():.2f} s")
        
            print(f"                GndPos.Xn: {self.rf_pos.xn:.2f} m")
            print(f"                GndPos.Xe: {self.rf_pos.xe:.2f} m")
            print(f"               GndVel.Mag: {gspd:.2f} m/s")
            print(f"                GndVel.Un: {self.rf_un:.2f} m/s")
            print(f"                GndVel.Ue: {self.rf_ue:.2f} m/s")
            print(f"              GndVel.Uhdg: {self.rf_uhdg:.2f} deg")
            print(f"              GndVel.Glid: {self.__gr:.2f}")
            
            print(f"              NavPnt.Ipnt: {self.__ipnt}")
            print(f"                NavPnt.Xn: {self.__pnts[self.__ipnt].xn:.2f} m")
            print(f"                NavPnt.Xe: {self.__pnts[self.__ipnt].xe:.2f} m")
            print(f"              NavPnt.Dist: {distToPnt:.2f} m")
            print(f"           NavPnt.LegFrac: {legFrac:.4f}")
            print(f"               NavPnt.Brg: {brgToPnt:.2f} deg")
            print(f"            NavPnt.RelBrg: {self.__targetGeomBrg:.2f} deg")
            print(f"           NavPnt.TpScale: {self.__tpscale:.2f}")
            
            print(f"                     Fcos: {fdot:.2f}")
            #print(f"                     Fphi: {self.__targetGeomBrg:.2f} deg")

            print(f"Gained distance remaining: {self.__totaldist:.0f} m")
            print(f"          Estimated range: {self.__range:.0f} m")

            print("#----------------------------------------#")
        
        
        #if distToPnt < 50:
        #    if (distToPnt > self.__distPntOld) and self.__ipnt < (len(self.__pnts)-1):
        #        self.__ipnt += 1
        #        self.__distPntOld = 1e+9
        #    self.__distPntOld = distToPnt
        
        #if distToPnt < 50:
        #    if fdot < 0 and self.__ipnt < (len(self.__pnts)-1):
        #        self.__ipnt += 1
        
        if legFrac > 1.0:
            if self.__ipnt < (len(self.__pnts)-1):
                self.__ipnt += 1
            else:
                self.__pntsDone = True

            
    def updateScaledPnts(self, roll, runtime):
        
        if (np.round(runtime,2) % 1.0 == 0):
            gravg = sum(self.__grarr) / len(self.__grarr)
            self.__grarr = []
            
            rng = self.calcDistRange(gr=gravg)
            fsc = rng / self.__totaldist
            
            self.__tpscale *= fsc
                        
        else:
            self.__grarr.append(self.__gr)

        #if self.__ipnt != self.__ipntlast:
        #    print("Factoring distance:")
        #    print("Remaining:", self.__totaldist )
        #    print("Range:", self.__range )
        #    print("Factor scale:", self.__range/self.__totaldist )
        #    print("Track scale:", self.__tpscale )
        #    #input()
        #                        
        #    self.__ipntlast = self.__ipnt
                
        #if self.__ipnt < (len(self.__pnts)-1):
        #    self.__tpscale = self.__tpfilt( self.__range / self.__unscaleddist, runtime )
        
        for i in range(len(self.__pnts)):
            
            self.__pnts[i].xn = self.__tpscale * self.__unscaled_pnts[i].xn
            self.__pnts[i].xe = self.__tpscale * self.__unscaled_pnts[i].xe


        
    def calcRemainingDistance(self):
        
        pnts = []
        pnts.append(self.rf_pos)
                
        for i in range(self.__ipnt,len(self.__pnts)):
            pnts.append(self.__pnts[i])
        
        # Total distance remaining
        dist = 0.0
        for i, p in enumerate(pnts):
            if i > 0:
                dist += np.sqrt(
                    (p.xn-pnts[i-1].xn)**2 
                    + (p.xe-pnts[i-1].xe)**2 )
            
        #print(f"Total dist as calc: {dist:.1f} m")
        
        return dist * self.fdist
       
       
    def calcDistRange(self, gr=None):
        
        if gr is not None:
            return gr * self.rf_pos.agl
        
        return self.__gr * self.rf_pos.agl
    
    
    def calcVectorizeLeg(self):
        
        # Vector from current position to next pnt
        r = np.array([ self.__pnts[self.__ipnt].xn - self.rf_pos.xn, 
                       self.__pnts[self.__ipnt].xe - self.rf_pos.xe ])

        # Velocity vector
        u = np.array([self.rf_un, self.rf_ue])
                        
        fdot = np.dot(r, u)

        #print(f"pos={self.__pnts[self.__ipnt].xn:.2f}, {self.__pnts[self.__ipnt].xe:.2f}")
        #print(f"pos={self.rf_pos.xn:.2f}, {self.rf_pos.xe:.2f}")
        #print(f"r={r[0]:.2f}, {r[1]:.2f}")
        #print(f"u={u[0]:.2f}, {u[1]:.2f}")
        #print(f"fdot={fdot:.2f}")
        
        fphi = fdot / (np.linalg.norm(r) * np.linalg.norm(u))
        fphi = np.arccos(fphi) * np.rad2deg(1.0)
        
        # Detect direction from cross product
        fphi *= np.sign(np.cross(u,r))
        
        return fdot, fphi
    
    
    def calcProjectedPathFraction(self):
        
        leg = np.array([
            self.__pnts[self.__ipnt].xn - self.__pnts[self.__ipnt-1].xn,
            self.__pnts[self.__ipnt].xe - self.__pnts[self.__ipnt-1].xe,
        ])
        
        pos = np.array([
            self.rf_pos.xn - self.__pnts[self.__ipnt-1].xn,
            self.rf_pos.xe - self.__pnts[self.__ipnt-1].xe,
        ])
        
        #print("leg=", leg)
        #print("pos=", pos)
        #print("proj=", np.dot(pos,leg)/np.dot(leg,leg))
        
        res = np.dot(pos,leg)/np.dot(leg,leg)

        return np.maximum(0.01, res)
        
        
    
    # Getters
    
    def getCurrentPointIndex(self):
        return self.__ipnt
    
    def getCurrentGeomPos(self) -> GeomPnt:
        return self.rf_pos
    
    def getNavRelativeBearing(self):
        
        if not self.__pntsDone:
            return self.__targetGeomBrg 
        else:
            return 0.0
    
    def getTpScale(self):
        return self.__tpscale
    
    # Helpers
    
    
    
    def _calcDist2Pnts(self, p1, p2):
        
        return np.sqrt((p2.xn-p1.xn)**2 + (p2.xe-p1.xe)**2)
        
        
    def _calcBearing2Pnts(self, p1, p2):
        
        return np.rad2deg(np.atan2(p2.xe-p1.xe,p2.xn-p1.xn))

        
        

    
