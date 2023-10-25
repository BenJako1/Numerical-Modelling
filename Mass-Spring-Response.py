#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 22 16:35:36 2023

@author: ben
"""

import numpy as np
import matplotlib.pyplot as plt

#------------------------------------------------------------------------------
# System paramters

m = 1 # Mass in kg
c = 0.1 # Damping constant in Nm/s
k = 1 # Spring constant N/m

x_0 = 1 # Initial diplacement
v_0 = 0 # Initial velocity

# Forcing parameters
F_0 = 1
Omega = 1
#------------------------------------------------------------------------------
# Simulation parameters

r_max = 2
N = 200

t_0 = 0
t_end = 30
dt = 0.01

#------------------------------------------------------------------------------
# Function definitions

# Forcing function
def F(t):
    return (F_0 * np.cos(Omega * t))

# Runge-Kutta function
def Runge_Kutta_4(x, v, dt, t):
    k1_x = v
    k1_v = -(c * v + k * x - F(t)) / m
    
    k2_x = v + 0.5 * dt * k1_v
    k2_v = -(c * (v + 0.5 * dt * k1_v) + k * (x + 0.5 * dt * k1_x) -
             F(t + 0.5 * dt)) / m
    
    k3_x = v + 0.5 * dt * k2_v
    k3_v = -(c * (v + 0.5 * dt * k2_v) + k * (x + 0.5 * dt * k2_x) -
             F(t + 0.5 * dt)) / m
    
    k4_x = v + dt * k3_v
    k4_v = -(c * (v + dt * k3_v) + k * (x + dt * k3_x) -
             F(t + dt)) / m
    
    x_new = x + (dt / 6) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x)
    v_new = v + (dt / 6) * (k1_v + 2 * k2_v + 2 * k3_v + k4_v)
    
    return x_new, v_new
    
def simulate(x_0, v_0, t_0, t_end, dt):
    t = np.arange(t_0, t_end, dt)
    x = x_0
    v = v_0

    xList = [x_0]
    vList = [v_0]

    for n in t[1:]:
        x, v = Runge_Kutta_4(x, v, dt, n)
        xList.append(x)
        vList.append(v)
    
    return t, xList, vList
#------------------------------------------------------------------------------
# Initialising variables and running loop

w_0 = np.sqrt(k / m)
T = 2 * np.pi / w_0

responseList = []

rList = np.linspace(0, r_max, N)

for Omega in (rList * w_0):
    _, x, _ = simulate(x_0, v_0, t_0, t_end, dt)
    X = max(x)
    responseList.append(X / x_0)

#------------------------------------------------------------------------------
# Display   
plt.plot(rList, responseList)
plt.grid()
plt.show()
'''
plt.plot(t, x, 'b-')
plt.plot(t, F(t), 'r-')
plt.title("Mass-Spring Oscillator")
plt.xlabel("time (s)")
plt.ylabel("displacement (m)")
plt.grid()
plt.show()
'''