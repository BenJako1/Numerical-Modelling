#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 19 10:56:11 2023

@author: ben
"""
# Library imports
import numpy as np
from numpy import random
import matplotlib.pyplot as plt

#------------------------------------------------------------------------------
# System specifications

# System parameters
length = 1 # Wire length in m
diameter  = 0.5 # Wire diameter in mm
emissivity = 0.7 # Emissivity constant
SB = 0.00000005670374419 # Stefan-Boltzmann constant
ambientTemp = 300 # Ambient temperature in Kelvin
H = 30 # Convective heat transfer coefficient
c = 480 # Specific heat capacity in J/kgK
density = 8400 # Density in kg/m^3

# Simulation parameters
t_0 = 0 # Beginning of simulation run time
t_end = 60 # End of simulation run time
dt = 0.1 # Simulation time step
temp_0 = ambientTemp
disturbanceMagnitude = 5
desiredTemp = 400 # Desired temperature in Kelvin

# Control parameters
kp = 2 # Proportional gain
ki = 0.1 # Integral gain

#------------------------------------------------------------------------------
# Parameter calculations and variable initialisation

surface = 2 * np.pi * (diameter / 2000) * length # Surface area in m^2
mass = np.pi * (diameter / 2000)**2 * length * density # mass in kg

t = t_0
temp = temp_0
power_in = 0
iPower = 0

timeList = [t]
tempList = [temp]
powerList = [0]

#------------------------------------------------------------------------------
# Running Loop

while t < t_end:
    # Power out through calculations
    radiation = emissivity * surface * SB * (temp**4 - ambientTemp**4)
    convection = H * surface * (temp - ambientTemp)
    power_out = radiation + convection
    power_out += (random.rand() - 0.5) * disturbanceMagnitude
    
    # PI Controller
    error = desiredTemp - temp
    pPower = error * kp
    iPower += error * dt * ki
    pPower = max(0, pPower)
    iPower = max(0, iPower)
    
    # Control variable and temp calculation
    power_in = pPower + iPower
    power = power_in - power_out
    temp += power * dt / (mass * c)
    
    # Increment time
    t += dt
    
    # Append variables to lists for plotting
    timeList.append(t)
    tempList.append(temp)

#------------------------------------------------------------------------------
# Plotting results

plt.plot(timeList, tempList, 'b-')
plt.plot(timeList, [desiredTemp] * len(timeList), 'k-')
plt.title("Temperature Over Time of a Wire with PI Power Control")
plt.xlabel("Time (s)")
plt.ylabel("Temperature (K)")
plt.grid()
plt.show()