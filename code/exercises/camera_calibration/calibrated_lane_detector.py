import numpy as np
from ..lane_detection.lane_detector import LaneDetector
from ..lane_detection.camera_geometry import CameraGeometry
from collections import deque

class CalibratedLaneDetector(LaneDetector):
    def __init__(self, calib_cut_v=200, cam_geom=CameraGeometry(), model_path='./lane_segmentation_model.pth', 
        encoder='efficientnet-b0', encoder_weights='imagenet', history_len=50):
        # call parent class constructor
        super().__init__(cam_geom, model_path, encoder, encoder_weights)

        self.calib_cut_v = calib_cut_v
        self.uv_grid = self.init_uv_grid() # build u,v grid
        self.pitch_yaw_history = deque([], maxlen=history_len) # maintains queue of latest n predictions.
        self.mean_residuals_thresh = 2.5 #TODO: adjust this thresh hold to avoid calibration process at curves.
        self.mean_pitch, self.mean_yaw = None, None
        
    def init_uv_grid(self):
        uv = []
        for v in range(self.calib_cut_v, self.cg.image_height):
            for u in range(self.cg.image_width):
                uv.append(np.array([u,v]))
        return np.array(uv)
    
    def _fit_line_v_of_u(self, probs):
        probs_flat = np.ravel(probs[self.calib_cut_v:, :])
        mask = probs_flat > 0.3
        coeffs, residuals, _, _, _ = np.polyfit(
            self.uv_grid[:,0][mask], self.uv_grid[:,1][mask], deg=1, w=probs_flat[mask], full=True)
        mean_residuals = residuals/len(self.uv_grid[:,0][mask])
        #print(mean_residuals)
        if mean_residuals < self.mean_residuals_thresh:
            return np.poly1d(coeffs)
        else:
            return None
            
    @staticmethod    
    def get_intersection(line1, line2):
        m1, c1 = line1
        m2, c2 = line2
        #TODO: find intersection of the line.
        raise NotImplementedError
        
    @staticmethod
    def get_py_from_vp(u_i, v_i, K):
        #TODO compute pitch and yaw given the camera intrinsic matrix and vanishing point.
        raise NotImplementedError
        return pitch, yaw

    def calibrate(self, image, mpl_axis=None):
        _, left_probs, right_probs = self.detect(image)
        line_left  = self._fit_line_v_of_u(left_probs)
        line_right = self._fit_line_v_of_u(right_probs)
        if line_left is None or line_right is None:
            return False
            
        vanishing_point = CalibratedLaneDetector.get_intersection(line_left, line_right)
        if vanishing_point is None:
            return False      
        
        u_i, v_i = vanishing_point
        pitch, yaw = CalibratedLaneDetector.get_py_from_vp(u_i, v_i, self.cg.intrinsic_matrix)
        self.add_to_pitch_yaw_history(pitch, yaw)
        
        if mpl_axis is not None:
            self.visualize_vanishing_point(image, line_left, line_right, vanishing_point, mpl_axis)
        
        return True
        
    def get_mean_py(self):
        py = np.array(self.pitch_yaw_history)
        mean_pitch = np.mean(py[:,0])
        mean_yaw = np.mean(py[:,1])
        return mean_pitch, mean_yaw

    def add_to_pitch_yaw_history(self, pitch, yaw):
        self.pitch_yaw_history.append([pitch, yaw])
        mean_pitch, mean_yaw = self.get_mean_py() 
        self.update_cam_geometry(mean_pitch, mean_yaw)
        self.mean_pitch, self.mean_yaw = mean_pitch, mean_yaw
        print("yaw, pitch = ", np.rad2deg(mean_yaw), np.rad2deg(mean_pitch))

    def update_cam_geometry(self, pitch, yaw):
        self.cg = CameraGeometry(pitch_deg = np.rad2deg(pitch), yaw_deg=np.rad2deg(yaw))
        self.cut_v, self.grid = self.cg.precompute_grid()
        
    def visualize_vanishing_point(self, image, line_left, line_right, vanishing_point, mpl_axis):
        u = np.arange(0, self.cg.image_width, 1)
        v_left = line_left(u)
        v_right = line_right(u)

        mpl_axis.imshow(image)
        # plot detected lane lines
        mpl_axis.plot(u, v_left, '-c')
        mpl_axis.plot(u, v_right, '-c')
        mpl_axis.set_xlim(0, self.cg.image_width)
        mpl_axis.set_ylim(self.cg.image_height, 0)

        # plot intersection of lane lines
        u_i, v_i = vanishing_point
        mpl_axis.scatter([u_i], [v_i], marker="o", s=100, color="y", zorder=10)
