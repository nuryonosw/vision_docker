# -*- coding: utf-8 -*-
"""
Created on Thu Mar 24 22:15:30 2022

@author: USER
"""
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from spatialmath.base import sym
from roboticstoolbox import ETS as E
l1 = 0.036
l2 = 0.0405
l3 = 0.124
l4 = 0.064
l5 = 0.0405
l6 = 0.028


e = E.tz(l1) * E.rz() * E.tz(l2) * E.ry() * E.tx(l3) *E.tz(-0.024)*E.ry()*E.tx(l4)*E.rx()*E.tx(l5)*E.ry()*E.tx(l6)*E.rx()


q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
robot = rtb.ERobot(e)
print(robot)
E.plot(e,q)
print(e)
