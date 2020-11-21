import numpy as np
import matplotlib.pyplot as plt
#from track_generator import get_random_racetrack
from clothoid_generator import get_random_racetrack
import copy

class Track:
    #def __init__(self, rad = 0.2, edgy = 0.01, scale=1000, seed=42):
    #    x,y = get_random_racetrack(rad, edgy, scale, seed)
    #    self.waypoints = np.stack((x,y)).T

    def __init__(self, seed=42):
        np.random.seed(seed)
        x,y = get_random_racetrack()
        self.waypoints = np.stack((x,y)).T
        track_length = np.linalg.norm(np.diff(self.waypoints,axis=0),axis=1).sum()
        obj_dist = 50
        num_obj = int(track_length/obj_dist)*2
        index_diff = int(len(self.waypoints)*obj_dist/track_length)
        self.obj = np.zeros((num_obj,2))
        j = 0
        while j+1<num_obj:
            index = int(j/2*index_diff)
            if index >= len(self.waypoints)-1:
                index = -2
            p0 = self.waypoints[index]
            p1 = self.waypoints[index+1]
            dx = p1[0]-p0[0]
            dy = p1[1]-p0[1]
            theta = np.arctan2(dy,dx)
            self.obj[j]   = p0 + np.array([-np.sin(theta), np.cos(theta)]) * 3
            self.obj[j+1] = p0 - np.array([-np.sin(theta), np.cos(theta)]) * 3
            j+=2
            


    def transform(self,x,y,theta, data):
        xc = data[:,0] - x
        yc = data[:,1] - y
        c,s = np.cos(theta), np.sin(theta)
        xcr = xc * c + yc*s
        ycr = -xc*s + yc*c
        close = xcr**2 +ycr**2 < 600**2
        in_front = xcr > -5
        ret_x, ret_y = xcr[close & in_front], ycr[close & in_front]   
        path =np.stack((ret_x, ret_y)).T
        return path

    def get_vehicle_path(self, x,y,theta):
        return self.transform(x,y,theta,self.waypoints)

    def get_obj(self,x,y,theta):
        return self.transform(x,y,theta,self.obj)

    def get_start_pose(self):
        px, py = self.waypoints[:,0], self.waypoints[:,1]
        theta = np.arctan2(py[1]-py[0], px[1]-px[0])
        return px[0], py[0], theta

    def plot(self, color="red"):
        plt.plot(self.waypoints[:,0], self.waypoints[:,1], color=color)   
        plt.scatter(self.obj[:,0], self.obj[:,1], color=color)   

if __name__ == "__main__":
    track=Track()
    track.plot()
    plt.axis("equal")
    plt.show()