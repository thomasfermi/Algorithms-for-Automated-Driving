import numpy as np
#from scipy.integrate import odeint

def normalize_angle(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))    

class Vehicle:
    def __init__(self, x=0, y=0, theta=0, v=0, wheel_base=2):
        self.x = x
        self.y = y
        self.v = v
        self.theta = normalize_angle(theta)
        self.wheel_base = wheel_base

    def update(self, dt, delta, a):
        a = max(0.0,a)
        self.x += dt*self.v * np.cos(self.theta)
        self.y += dt*self.v * np.sin(self.theta)
        theta_dot = self.v*np.tan(delta)/self.wheel_base
        self.theta = normalize_angle(self.theta + theta_dot*dt)
        aeff = a - self.v**2 * 0.001
        self.v += dt*aeff

    def print(self):
        print("vehicle.x")
        print(self.x)
        print("vehicle.y")
        print(self.y)
        print("vehicle.theta")
        print(self.theta)


if __name__=="__main__":
    vehicle = Vehicle(0,0,0,10)
    vehicle.update(0.1, 0.2, 1)
    vehicle.print()