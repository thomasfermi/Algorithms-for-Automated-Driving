import matplotlib.pyplot as plt
from pyclothoids import Clothoid
import numpy as np

def normalize_angle(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))   

def get_random_racetrack():
    steps = 5
    res = 100
    length = 200
    radius = 5
    theta_jump = np.pi/10
    xc, yc, thetac = 0,0,0 
    rx,ry = np.zeros(steps*res),np.zeros(steps*res)
    for i in range(steps):
        c,s = np.cos(thetac), np.sin(thetac)
        l = np.random.uniform(0.8*length, 1.2*length)
        l = length
        x = xc + l*c + np.random.uniform(-radius,radius)
        y = yc + l*s + np.random.uniform(-radius,radius)
        theta = thetac+np.random.uniform(-theta_jump, theta_jump)
        clothoid = Clothoid.G1Hermite(xc,yc,thetac, x,y, theta)
        rx[i*res:(i+1)*res], ry[i*res:(i+1)*res]= clothoid.SampleXY(res)
        xc,yc,thetac = x,y,theta
    return rx,ry