# -*- coding: utf-8 -*-
"""
Created on Fri May 13 12:58:33 2016

@author: paftdl
"""

import pyproj
#coordinates1
lat1 = 75#53.32055
lon1 = 33.5#-1.72972
#coordinates2
lat2 =  76#53.31861
lon2 =  32.5#-1.69972

g = pyproj.Geod(ellps='WGS84')
(az12, az21, dist) = g.inv(lon2, lat2, lon1, lat1)

print 'original distance:', dist/1000
print 'original angle:',az12
print az21




from math import *
#coordinates1
#lat1 = 73#53.32055
#lon1 = 33.5#-1.72972
##coordinates2
#lat2 =  73#53.31861
#lon2 =  32.5#-1.69972

Aaltitude = 2000
Oppsite  = 20000

#Haversine Formuala to find vertical angle and distance
lon1, lat1, lon2, lat2 = map(radians, [lon2, lat2, lon1, lat1])

dlon = lon2 - lon1
dlat = lat2 - lat1
a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
c = 2 * atan2(sqrt(a), sqrt(1-a))
Base = 6371 * c

#Horisontal Bearing
def calcBearing(lat1, lon1, lat2, lon2):
    dLon = lon2 - lon1
    y = sin(dLon) * cos(lat2)
    x = cos(lat1) * sin(lat2) \
        - sin(lat1) * cos(lat2) * cos(dLon)
    return atan2(y, x)

Bearing = calcBearing(lat1, lon1, lat2, lon2)
Bearing = degrees(Bearing)

Base2 = Base * 1000
distance = Base * 2 + Oppsite * 2 / 2
Caltitude = Oppsite - Aaltitude


#Convertion from radians to decimals
a = Oppsite/Base
b = atan(a)
c = degrees(b)


#Convert meters into Kilometers
distance = distance / 1000


#Output the data
print("---------------------------------------")
print(":::::Auto Aim Directional Anntenna:::::")
print("---------------------------------------")
print("Horizontial Distance:", Base,"km")
print("   Vertical Distance:", distance,"km")
print("    Vertical Bearing:",c)
print(" Horizontial Bearing:",Bearing)
print("---------------------------------------")
input("Press <enter> to Exit")