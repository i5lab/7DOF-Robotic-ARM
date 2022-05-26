import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
pi = np.pi

from roboticstoolbox.backends.swift import Swift
env = Swift()
env.launch()

ur5 = rtb.models.URDF.UR5()
ur5.q = ur5.qr = np.array([pi/2,-pi/2,pi/4,pi/3,0,0])


env.add(ur5)


arrived = False
i = 1
v = 1

while not arrived:
    i += v

    #ur5.q = ur5.qr = np.array([pi/4,-pi/4,i*pi/4/10000,i*pi/3 / 10000,i*pi/3 / 10000,0])
    ur5.q = ur5.qr = np.array([i / 30000,0, -i/30000,0,0,0])
    
    if i == 60000 or i == 0:
    	v *= -1
    
    env.step(0.1)
