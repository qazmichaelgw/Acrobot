from math import cos, sin, pi, copysign
from collections import namedtuple
from random import random

AcrobotState = namedtuple('AcrobotState', 'theta1 theta2 theta1Dot theta2Dot')

def sqr(x): return x*x

class AcrobotParam:
    maxTheta1 = pi
    maxTheta2 = pi
    maxTheta1Dot = 4 * pi
    maxTheta2Dot = 9 * pi
    m1 = 1.0
    m2 = 1.0
    l1 = 1.0
    l2 = 1.0
    lc1 = 0.5
    lc2 = 0.5
    I1 = 1.0
    I2 = 1.0
    g = 9.8
    dt = 0.05
    goalPosition = 1.0

    transitionNoise = 0.0

class Acrobot(object):
    def __init__(self, param = AcrobotParam):
        self.param = param

    def initState(self):
        return AcrobotState(0.0, 0.0, 0.0, 0.0)

    def next(self, s, torque):
        #assert -1 <= torque <= 1
        p = self.param
        #noise = p.transitionNoise * 2.0 * (random() - 0.5)
        #torque += noise

        d1 = p.m1 * sqr(p.lc1) + p.m2 * (sqr(p.l1) + sqr(p.lc2) + 2 * p.l1 + p.lc2 * cos(s.theta2)) + p.I1 + p.I2
        d2 = p.m2 * (sqr(p.lc2) + p.l1 * p.lc2 * cos(s.theta2)) + p.I2

        phi_2 = p.m2 * p.lc2 * p.g * cos(s.theta1 + s.theta2 - pi / 2.0)
        phi_1 = -(p.m2 * p.l1 * p.lc2 * sqr(s.theta2Dot) * sin(s.theta2) - 2 * p.m2 * p.l1 * p.lc2 * s.theta1Dot * s.theta2Dot * sin(s.theta2)) + (p.m1 * p.lc1 + p.m2 * p.l1) * p.g * cos(s.theta1 - pi / 2.0) + phi_2

        theta2_ddot = (torque + (d2 / d1) * phi_1 - p.m2 * p.l1 * p.lc2 * sqr(s.theta1Dot) * sin(s.theta2) - phi_2) / (p.m2 * sqr(p.lc2) + p.I2 - sqr(d2) / d1)
        theta1_ddot = -(d2 * theta2_ddot + phi_1) / d1

        ntheta1Dot = s.theta1Dot + theta1_ddot * p.dt
        ntheta2Dot = s.theta2Dot + theta2_ddot * p.dt
        ntheta1 = s.theta1 + ntheta1Dot * p.dt
        ntheta2 = s.theta2 + ntheta2Dot * p.dt

        if abs(ntheta1Dot) > p.maxTheta1Dot:
            ntheta1Dot = copysign(p.maxTheta1Dot, ntheta1Dot)
        if abs(ntheta2Dot) > p.maxTheta2Dot:
            ntheta2Dot = copysign(p.maxTheta2Dot, ntheta2Dot)

        # XXX ICRA dynamic has this (for ntheta2 only)
        #if abs(ntheta2) > pi:
        #    ntheta2 = copysign(pi, ntheta2)
        #    ntheta2Dot = 0.0

        #if abs(ntheta1) > pi:
            #ntheta1 = copysign(pi, ntheta1)
            #ntheta1Dot = 0.0

        def normpi(t):
            while t > pi:
                t -= pi * 2
            while t < -pi:
                t += pi * 2
            return t
        ntheta1 = normpi(ntheta1)
        ntheta2 = normpi(ntheta2)

        ns = AcrobotState(ntheta1, ntheta2, ntheta1Dot, ntheta2Dot)
        if abs(ns.theta1) > 10 or abs(ns.theta2) > 10:
            print p.m2, ns
        return ns

    def height(self, s):
        p = self.param
        firstJointEndHeight = p.l1 * cos(s.theta1)
        secondJointEndHeight = p.l2 * sin(pi/2 - s.theta1 - s.theta2)
        return firstJointEndHeight + secondJointEndHeight

def test():
    p = AcrobotParam()
    #p.m2 = 1.1
    a = Acrobot(p)
    s = a.initState()
    for i in range(1000):
        #t = i%2*2-1
        #t = copysign(1,s.theta1*s.theta1Dot)
        t = 1
        s = a.next(s, t)
        print s
        print a.height(s)
        if (a.height(s) < -p.goalPosition):
            print "above the goal"
            break


if __name__=='__main__':
    test()
