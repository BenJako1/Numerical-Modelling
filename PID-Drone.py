#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 19:07:49 2023

@author: ben
"""

# Importing libraries
from numpy import random
import matplotlib.pyplot as plt

#------------------------------------------------------------------------------
# System specifications

mass = 1 # Mass in kg
gravity = 9.81 # Gravitational acceleration in m/s^2

# Simulation parameters
t_0 = 0
t_end = 10
dt = 0.1
height_0 = 0
velocity_0 = 0
disturbanceMagnitude = 5
damping = True
desiredHeight = 5

# Control parameters
kp = 7
ki = 3.6
kd = 1
dampingConst = 4

#------------------------------------------------------------------------------
# Variable initialisation

t = t_0
height = height_0
velocity = velocity_0

timeList = [t]
heightList = [height]
velocityList = [velocity]

iForce = 0
prev_error = 0

#------------------------------------------------------------------------------
# PID function definition

def PID(previousError, setPoint, processVariable):
    global iForce # Declare iForce as global
    
    error = setPoint - processVariable
    if damping:
        pForce = error * kp - velocity * dampingConst
    else:
        pForce = error * kp
    iForce += error * dt * ki
    dForce = (error - previousError) / dt * kd
    controlVariable = pForce + iForce + dForce
    return controlVariable

#------------------------------------------------------------------------------
# Running Loop

while t < t_end:
    # PID controller
    force = PID(prev_error, desiredHeight, height)
    force += (random.rand() - 0.5) * disturbanceMagnitude
    
    # State updates
    acceleration = force / mass - gravity
    velocity += acceleration * dt
    height += velocity * dt
    
    # Increment time
    t += dt
    
    # Store error buffer
    prev_error = desiredHeight - height
    
    # Append variables to lists for plotting
    timeList.append(t)
    heightList.append(height)
    velocityList.append(velocity)

#------------------------------------------------------------------------------
# Plotting results

plt.plot(timeList, heightList, 'b-')
plt.plot(timeList, [desiredHeight] * len(timeList), 'k-')
plt.title("Altitude of a Drone Over Time with PID Thrust Control")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.grid()
plt.show()