#/usr/bin/env python

## @file
#
#
#

## @author Brendan Andrade

import numpy as np
import math as m

##
#
#
class BelugaWaypointController(object):
    
    ##
    #
    #
    def __init__(self, k1, k2, kz, beta, lambda_, max_velocity, stop_at_wp = False):
        
        self.k1 = float(k1)
        self.k2 = float(k2)
        self.kz = float(kz)
        self.beta = float(beta)
        self.lamb = float(lambda_)
        self.v_max = float(max_velocity) # m/s

        self.stop_at_wp = stop_at_wp

    ##
    #
    #
    def control_law(self, x, waypoint):
        
        # Perform control law for planar motion from A Smooth Control 
        # Law for Graceful Motion of Differential Wheeled Mobile Robots 
        # by Park and Kuipers
        r = m.sqrt((x[0] - waypoint[0])**2 + (x[1] - waypoint[1])**2) # Planar distance to goal
        alpha = m.atan2(waypoint[1] - x[1], waypoint[0] - x[0]) # Angle from x-axis of vector from x to waypoint
        psi = waypoint[3] - alpha # Error from goal (waypoint) heading to alpha
        delta = m.atan2(x[5], x[6]) - alpha # Error from current heading to alpha

        psi = self.wrap_to_pi(psi)
        delta = self.wrap_to_pi(delta)
        
        print psi
        print delta

        kappa = -1.0/r * (self.k2*(delta-m.atan(-self.k1*psi)) + (1.0 + self.k1/(1.0+(self.k1*psi)**2)) * m.sin(delta)) # Control law for curvature from path
        
        v = self.v_max/(1.0 + self.beta*abs(kappa)**self.lamb) # Control law for velocity
        omega = kappa*v # Omega = curvative*velocity
        
        print "v: " + str(v)
        print "kappa: " + str(kappa)
        print "omega: " + str(omega)

        # Proportional control for z position
        vz = -self.kz*(x[2] - waypoint[2])
        
        return np.array([v, omega, vz])

    def wrap_to_pi(self, angle):
        while angle <= -m.pi or angle > m.pi:
            angle += -m.copysign(2.0*m.pi, angle) # if angle < 0 add 2pi, if angle > 0 substract 2pi
        return angle
