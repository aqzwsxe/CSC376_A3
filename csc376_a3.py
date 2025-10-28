import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from math import pi
from swift import Swift

# Launch simulator swift
env = Swift()
env.launch(realtime = True)

# Q3 1.
panda1 = rtb.models.Panda()
panda2 = rtb.models.Panda()
panda1.q = panda1.qr
panda2.q = panda2.qr

panda1.base = sm.SE3(0.5,0,0) * sm.SE3.Rz(np.pi)
panda2.base = sm.SE3(-0.5,-0.1,0)
#T = panda1.fkine(panda1.q) * sm.SE3.Tx(10) * sm.SE3.Ty(0.2) * sm.SE3.Tz(0.45)

env.add(panda1)
env.add(panda2)
env.hold()



