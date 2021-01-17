#!/usr/bin/env python3

import math

class DeltaRoboticArm():
    def __init__(self):
        self.upper_base = 170 # lenght upper base f
        self.low_base = 56 # lenght low base e
        self.upper_shoulder = 104 # lenght upper shoulder rf
        self.low_shoulder = 270 # lenght low shoulder re
        self.sqrt3 = math.sqrt(3)
        self.pi = math.pi
        self.sin120 = self.sqrt3 / 2
        self.cos120 = -0.5
        self.tan60 = self.sqrt3
        self.sin30 = 0.5
        self.tan30 = 1 / self.sqrt3

    def delta_forward(self,theta1,theta2,theta3):
        t = (self.upper_base - self.low_base) * self.tan30 / 2
        y1 = -1 * (t +self.upper_shoulder * math.cos(theta1))
        z1 = -1 * self.upper_shoulder * math.sin(theta1)
        
        y2 = (t + self.upper_shoulder * math.cos(theta2)) * self.sin30
        x2 = y2 * self.tan60
        z2 = -1 * self.upper_shoulder * math.sin(theta2)

        y3 = (t + self.upper_shoulder * math.cos(theta3)) * self.sin30
        x3 = (-1 * y3 * self.tan60)
        z3 = -1 * self.upper_shoulder * math.sin(theta3)

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = y1 * y1 + z1 * z1
        w2 = x2 * x2 + y2 * y2 + z2 * z2
        w3 = x3 * x3 + y3 * y3 + z3 * z3

        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -1 * ((w2 - w1) * (y3 - y1) - (w3 - w1) * ( y2 - y1)) / 2.0
        a2 = -1 *( z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        a = a1*a1 + a2*a2 + dnm*dnm
        b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm)
        c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - self.low_shoulder * self.low_shoulder)
        d = b*b - 4.0*a*c
        if (d < 0):
            #print('broken operation')
            return -1
        z0 = -0.5*(b+math.sqrt(d))/a
        x0 = (a1*z0 + b1)/dnm
        y0 = (a2*z0 + b2)/dnm
        return(x0,y0,z0)

    def delta_ahgleYZ(self, x0,y0,z0):
        y1 = -0.5 * self.tan30 * self.upper_base
        y0 -= 0.5 * self.tan30 * self.low_base
        a = (x0 * x0 + y0 * y0 + z0 * z0 + self.upper_shoulder * self.upper_shoulder - self.low_shoulder * self.low_shoulder - y1 *y1) / (2 * z0)
        b = ( y1 - y0) / z0
        d = -1 * (a + b * y1) * (a + b * y1) + self.upper_shoulder * (b * b * self.upper_shoulder + self.upper_shoulder)
        if d < 0:
            #print('broken operation')
            return -1
            
        yj =(y1 - a * b - math.sqrt(d)) / (b * b + 1)
        zj = a + b * yj
        if yj > y1:
            theta = 180 * math.atan( (-1 * zj) / (y1 - yj)) / self.pi + 180
        else:
            theta = 180 * math.atan( (-1 * zj) / (y1 - yj)) / self.pi
        return(theta) 

    def InversProblem(self,x0,y0,z0):
        theta1 = theta2 = theta3 = 0
        status = self.delta_ahgleYZ(x0,y0,z0)
        theta1 = status
        theta2 = self.delta_ahgleYZ(x0 * self.cos120 + y0 * self.sin120,y0 * self.cos120 - x0 * self.sin120,z0)
        theta3 = self.delta_ahgleYZ(x0 * self.cos120 - y0 * self.sin120,y0 * self.cos120 + x0 * self.sin120,z0)
        if theta1 == -1 or theta2 == -1 or theta3 == -1:
            return False,list((0,0,0))
        else:
            return True,list((theta1/57.3,theta2/57.3,theta3/57.3))



