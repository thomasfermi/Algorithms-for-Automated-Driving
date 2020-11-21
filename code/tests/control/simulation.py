import numpy as np
import copy
import cv2
import sys
from track import Track
from vehicle import Vehicle

sys.path.append("../../util")
from geometry_util import dist_point_linestring
import matplotlib.pyplot as plt

import PIL.Image
from io import BytesIO
import IPython.display
import time

# helper functions
def resize(img, scale_percent):
    scale_percent = 60  # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    return resized


def show_img(a, fmt="png"):
    a = np.uint8(a)
    f = BytesIO()
    PIL.Image.fromarray(a).save(f, fmt)
    IPython.display.display(IPython.display.Image(data=f.getvalue()))


def lane_from_centerline(x, y, width):
    dx = np.gradient(x)
    dy = np.gradient(y)
    theta = np.arctan2(dy, dx)
    c, s = np.cos(theta), np.sin(theta)

    xl = x - s * 0.5 * width
    yl = y + c * 0.5 * width

    xr = x + s * 0.5 * width
    yr = y - c * 0.5 * width

    return xl, yl, xr, yr


def uv_fix(u, v):
    mask = (u > -20) & (u < 1100) & (v > -20) & (v < 600)
    return u[mask], v[mask]


def xy_to_XYZ(x, y):
    return np.stack((y, np.zeros_like(x) + 3, x + 15))


def xy_to_uv(x, y, K):
    X = xy_to_XYZ(x, y)
    uv1 = (K @ X).T
    u, v = uv1[:, 0] / uv1[:, 2], uv1[:, 1] / uv1[:, 2]
    # return u,v
    return uv_fix(u, v)


def xy_to_shape(x, y):
    theta = np.linspace(0, 2 * np.pi, 8)
    c, s = np.cos(theta), np.sin(theta)
    r = 0.3
    X = x + r * c
    Y = y + r * s
    return np.stack((X, Y)).T


def render_shape_xy(image, x, y, K):
    shape = xy_to_shape(x, y)
    u, v = xy_to_uv(shape[:, 0], shape[:, 1], K)
    pl = np.stack((u, v)).T
    cv2.polylines(
        image, np.int32([pl]), isClosed=True, color=[255, 0, 0], thickness=2
    )


class Simulation:
    def __init__(self, vehicle, track, controller, desired_velocity=25):
        self.controller = controller
        self.track = track
        self.vehicle = vehicle
        self.desired_velocity = desired_velocity
        vehicle.x, vehicle.y, vehicle.theta = track.get_start_pose()
        self.dt = 0.05
        self.traj = []
        self.cross_track_errors = []
        self.velocities = []
        self.waypoints = self.track.get_vehicle_path(
            self.vehicle.x, self.vehicle.y, self.vehicle.theta
        )
        self.obj = self.track.get_obj(
            self.vehicle.x, self.vehicle.y, self.vehicle.theta
        )
        self.K = np.array(
            [
                [1.23607734e03, 0.00000000e00, 5.12000000e02],
                [0.00000000e00, 1.23607734e03, 2.56000000e02],
                [0.00000000e00, 0.00000000e00, 1.00000000e00],
            ]
        )
        self.a, self.delta = 0, 0
        image_fn = "../../../data/carla_vehicle_bg_2.png"
        image_vehicle = cv2.imread(image_fn)
        self.image_vehicle = cv2.cvtColor(image_vehicle, cv2.COLOR_BGR2RGB)

    def step(self):
        self.waypoints = self.track.get_vehicle_path(
            self.vehicle.x, self.vehicle.y, self.vehicle.theta
        )
        self.obj = self.track.get_obj(
            self.vehicle.x, self.vehicle.y, self.vehicle.theta
        )
        self.a, self.delta = self.controller.get_control(
            self.waypoints, self.vehicle.v, self.desired_velocity, self.dt
        )
        self.a = np.clip(self.a, 0, 3)
        self.vehicle.update(self.dt, self.delta, self.a)
        self.traj.append([self.vehicle.x, self.vehicle.y])
        self.cross_track_errors.append(
            dist_point_linestring(np.array([0, 0]), self.waypoints)
        )
        self.velocities.append(self.vehicle.v)

    def plot_error(self):
        plt.plot(self.cross_track_errors)
        plt.title("Cross Track Error")
        plt.xlabel("Simulation step")
        plt.ylabel("error in meters")

    def plot_velocity(self):
        plt.plot(self.velocities)
        plt.plot([self.desired_velocity] * len(self.velocities), ls="--")
        plt.title("Velocity")
        plt.xlabel("Simulation step")
        plt.ylabel("v (m/s)")

    def cv_plot(self):
        wp = self.waypoints
        x, y = wp[:, 0], wp[:, 1]
        u, v = xy_to_uv(x, y, self.K)
        xl, yl, xr, yr = lane_from_centerline(x, y, width=3)
        ul, vl = xy_to_uv(xl, yl, self.K)
        ur, vr = xy_to_uv(xr, yr, self.K)

        # render lane
        arr = copy.deepcopy(self.image_vehicle)
        for lb in [np.stack((ul, vl)).T, np.stack((ur, vr)).T]:
            cv2.polylines(
                arr,
                np.int32([lb]),
                isClosed=False,
                color=[255, 255, 255],
                thickness=3,
            )

        # render objects beside lane
        x, y = self.obj[:, 0], self.obj[:, 1]
        point_array = np.stack((x, y)).T
        for point in point_array:
            x, y = point
            render_shape_xy(arr, x, y, self.K)

        # render steering wheel
        center = np.array([920, 80])
        radius = 40
        theta = np.linspace(0, 2 * np.pi, 20)
        u = center[0] + radius * np.cos(theta)
        v = center[1] + radius * np.sin(theta)
        pl = np.stack((u, v)).T
        cv2.polylines(
            arr, np.int32([pl]), isClosed=True, color=[0, 0, 255], thickness=3
        )
        u0 = center[0] + radius * np.cos(self.delta * 50)
        v0 = center[1] + radius * np.sin(self.delta * 50)
        u1 = center[0] - radius * np.cos(self.delta * 50)
        v1 = center[1] - radius * np.sin(self.delta * 50)
        pl = np.array([[u0, v0], [u1, v1]])
        cv2.polylines(
            arr, np.int32([pl]), isClosed=True, color=[0, 0, 255], thickness=3
        )

        # render text
        cte = self.cross_track_errors[-1]
        mystring = "cross track error = {:.2f}m, velocity={:.2f}m/s".format(
            cte, self.vehicle.v
        )

        font = cv2.FONT_HERSHEY_SIMPLEX
        org = (50, 50)
        fontScale = 1
        color = (255, 0, 0)
        thickness = 2
        arr = cv2.putText(
            arr, mystring, org, font, fontScale, color, thickness, cv2.LINE_AA
        )

        arr = resize(arr[0:512, 0:1024, :], 50)
        return arr

