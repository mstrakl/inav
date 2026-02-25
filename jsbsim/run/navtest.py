
import numpy as np
from src.autoflight.nav import *
from src.navplot import *

# LJMS modelarska
# Thr (sredina)
# End (juzni konec)
p_thr = NavPnt(46.631344, 16.177207, 70)
p_end = NavPnt(46.631039, 16.177051, 70)

dist, hdg = Navigation.calcDistBearing(p_thr, p_end)

print(f"Dist: {dist:.2f}, Hdg: {hdg:.2f}")

Rwy = RunwayFrame(p_thr, hdg)

geomPnts = [] 
geomPnts.append(GeomPnt(  50.0,  -50.0,   0.0))
geomPnts.append(GeomPnt( -80.0,  -50.0,   0.0))
geomPnts.append(GeomPnt(-100.0,  -25.0,   0.0))
geomPnts.append(GeomPnt( -80.0,    0.0,   0.0))
geomPnts.append(GeomPnt( -10.0,    0.0,   0.0))

navPnts = []
tdist = 0.0
for i, gp in enumerate(geomPnts):
    
    navPnts.append(Rwy.convertFrom(gp))
    if i > 0:
        dist, dum= Navigation.calcDistBearing(navPnts[i-1], navPnts[i])
        print(f"dist {i} = {dist:.1f}")
        tdist += dist

print("Total dist:", tdist)

#plot_nav_data(navPnts)
print()
print("Pilot starts::")

Dp = DeadStickPilot(Rwy, geomPnts, 10.0)

Dp.update(p_thr, 12.4, 200.0, 10.0)


#Dp.update(p_thr, 12.4, 190.0, 10.0)

#print(dist, hdg)
#
#print("Initial:")
#print(f" -lat={p_end.lat:.7f}")
#print(f" -lon={p_end.lon:.7f}")
#
#p_conv = Rwy.convertTo(p_end)
#
#print("Converted:")
#print(f" -xn={p_conv.xn:.2f}")
#print(f" -xe={p_conv.xe:.2f}")
#
#p_conv = Rwy.convertFrom(p_conv)
#print("Converted back:")
#print(f" -lat={p_conv.lat:.7f}")
#print(f" -lon={p_conv.lon:.7f}")

