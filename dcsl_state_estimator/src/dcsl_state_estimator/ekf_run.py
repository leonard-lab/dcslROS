#!/usr/bin/env python


from ekf_API import ekf
import numpy as np
import math as m

def main():
    init_t = 0
    init_x = np.zeros(7)
    init_P = np.ones(7)*0.1
    e = ekf(init_t, init_x, init_P, f, h, None, None, None, None, None, None)
    e.update_u(0,np.array([0.05, .2, 0.]))
    e.update_u(1,np.array([0.05, -.2, 0.]))
    x_new = e.propagate_state(2)
    print x_new[-1]

def f(x, u, t):
    x_dot = np.zeros(7)
    x_dot[0] = u[0]*m.cos(x[5])
    x_dot[1] = u[0]*m.sin(x[5])
    x_dot[2] = u[2]
    x_dot[3] = 0
    x_dot[4] = 0
    x_dot[5] = u[1]
    x_dot[6] = 0
    return x_dot

def h(x, t):
    y = np.zeros(4)
    y[0] = x[0]
    y[1] = x[1]
    y[3] = x[5]

if __name__ == "__main__":
    main()
