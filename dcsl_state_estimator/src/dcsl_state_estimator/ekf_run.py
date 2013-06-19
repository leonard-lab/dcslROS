#!/usr/bin/env python


from ekf_API import ekf
import numpy as np
import math as m
from random import gauss

def main():
    init_t = 0.
    init_x = np.zeros(7)
    init_P = np.ones((7,7))*0.1
    noise = np.array([0.01, 0.01, 0., 0., 0., 0.01, 0.]) 
    R = np.identity(4)
    R[0][0] = noise[0]
    R[1][1] = noise[1]
    R[2][2] = noise[2]
    R[3][3] = noise[5]
    Q = np.identity(7) * 0.1
    e = ekf(init_t, init_x, init_P, f, h, F, G, H, L, Q, R)
    t_list = np.linspace(init_t, 10, 101)
    dt = t_list[1] - t_list[0]
    x_hist = []
    x_hat_hist = []
    y_hist = []
    x = init_x
    u = np.array([0.05, .2, 0.])
    e.update_u(t_list[0], u)
    t_list = np.delete(t_list, 0, 0)
    for t in t_list:
        x, y = direct_prop(x, u, dt, noise)
        x_hat = e.estimate(t, y)
        e.update_u(t, u)
        x_hist.append(x)
        y_hist.append(y)
        x_hat_hist.append(x_hat)
    
    print "Final states: x_hat[0]: " + str(x_hat_hist[-1][0]) + " x[0]: " + str(x_hist[-1][0]) + " y[0] " + str(y_hist[-1][0])

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
    y[2] = x[2]
    y[3] = x[5]
    return y

def F(x, u, t):
    F = np.zeros((7, 7))
    F[0][5] = u[0]*-1*m.sin(x[5])
    F[1][5] = u[0]*m.cos(x[5])
    return F

def G(x, u, t):
    G = np.zeros((7,3))
    G[0][0] = m.cos(x[5])
    G[1][0] = m.sin(x[5])
    G[5][1] = 1
    return G

def H(x, t):
    H = np.zeros((4,7))
    H[0][0] = 1.
    H[1][1] = 1.
    H[2][2] = 1.
    H[3][5] = 1.
    return H

def L(x, u, t):
    L = np.identity(7)
    return L

def direct_prop(x, u, dt, noise):
    x_out = np.zeros(7)
    if abs(u[1]) < 0.001:
        x_out[5] = x[5]
        x_out[0] = u[0]*dt*m.cos(x[5]) + x[0]
        x_out[1] = u[0]*dt*m.sin(x[5]) + x[1]
    else:
        x_out[5] = x[5] + u[1]*dt
        r = u[0]/u[1]
        x_out[0] = x[0] + r*(m.sin(x_out[5]) - m.sin(x[5]))
        x_out[1] = x[1] + r*(m.cos(x[5]) - m.cos(x_out[5]))
    x_out[3] = u[0]
    x_out[6] = u[1]
    x_out[4] = u[2]
    x_out[3] = x[3] + u[2]*dt
    y_out = np.zeros(4)
    
    y_out[0] = x_out[0] + gauss(0, noise[0])
    y_out[1] = x_out[1] + gauss(0, noise[1])
    y_out[2] = x_out[2] + gauss(0, noise[2])
    y_out[3] = x_out[5] + gauss(0, noise[3])
    return x_out, y_out

if __name__ == "__main__":
    main()
