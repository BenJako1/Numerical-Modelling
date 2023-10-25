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
disturbanceMagnitude = 0
desiredHeight = 10

# Control parameters
kp = 8
ki = 5
kd = 4
dampingConst = 6

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

setPoint = desiredHeight
setPoint_buffer = [height_0]
setPointList = [desiredHeight]

#------------------------------------------------------------------------------
# PID function definition

def PID(previousError, setPoint, processVariable):
    global iForce # Declare iForce as global
    
    error = setPoint - processVariable
    pForce = error * kp - velocity * dampingConst
    iForce += error * dt * ki
    dForce = (error - previousError) / dt * kd
    controlVariable = pForce + iForce + dForce
    return controlVariable

#------------------------------------------------------------------------------
# Running Loop

while t < t_end:
    # PID controller
    force = PID(prev_error, setPoint, height)
    force += (random.rand() - 0.5) * disturbanceMagnitude
    
    # State updates
    acceleration = force / mass - gravity
    velocity += acceleration * dt
    height += velocity * dt
    
    # Increment time
    t += dt
    
    # Store error buffer
    prev_error = setPoint - height
    
    # Append variables to lists for plotting
    timeList.append(t)
    heightList.append(height)
    velocityList.append(velocity)
    setPointList.append(setPoint)

#------------------------------------------------------------------------------
# Plotting results

plt.plot(timeList, heightList, 'b-')
plt.plot(timeList, [desiredHeight] * len(timeList), 'k-')
plt.title("Altitude of a Drone Over Time with PID Thrust Control")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.grid()
plt.show()