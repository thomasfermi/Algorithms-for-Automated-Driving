import numpy as np
from ..lane_detection.lane_detector import LaneDetector
from ..lane_detection.camera_geometry import CameraGeometry


def get_intersection(line1, line2):
    m1, c1 = line1
    m2, c2 = line2
    #TODO: find intersection of the line.
    raise NotImplementedError

def get_py_from_vp(u_i, v_i, K):
    #TODO compute pitch and yaw given the camera intrinsic matrix and vanishing point.
    raise NotImplementedError
    return pitch, yaw

class CalibratedLaneDetector(LaneDetector):
    def __init__(self, calib_cut_v = 200, cam_geom=CameraGeometry(), model_path='./fastai_model.pth'):
        # call parent class constructor
        super().__init__(cam_geom, model_path)

        self.calib_cut_v = calib_cut_v
        self.estimated_pitch_deg = 0
        self.estimated_yaw_deg = 0
        self.update_cam_geometry()
        self.mean_residuals_thresh = 1e6 #TODO: adjust this thresh hold to avoid calibration process at curves.
        self.pitch_yaw_history = []
        self.calibration_success = False

    def get_fit_and_probs(self, image):
        _, left_probs, right_probs = self.detect(image)
        line_left  = self._fit_line_v_of_u(left_probs)
        line_right = self._fit_line_v_of_u(right_probs)
        if (line_left is not None) and (line_right is not None):
            # TODO: If both `line_left` and `line_right` are not None, 
            # try to compute the vanishing point using your `get_intersection` function. 
            # Then compute pitch and yaw from the vanishing point
            # Finally store the pitch and yaw values in `self.pitch_yaw_history`.
            # This `get_fit_and_probs` function will be called again and again over time.
            # Once enough data is gathered in `self.pitch_yaw_history`, 
            # compute mean values for pitch and yaw and store them in ` self.estimated_pitch_deg`and ` self.estimated_yaw_deg` 
            # Finally call `update_cam_geometry()` so that the new estimated values are being used.
            raise NotImplementedError

        left_poly = self.fit_poly(left_probs)
        right_poly = self.fit_poly(right_probs)
        return left_poly, right_poly, left_probs, right_probs
    
    def _fit_line_v_of_u(self, probs):
        v_list, u_list = np.nonzero(probs > 0.3)
        if v_list.size == 0:
            return None
        coeffs, residuals, _, _, _ = np.polyfit(
            u_list, v_list, deg=1, full=True)
            
        mean_residuals = residuals/len(u_list)
        #print(mean_residuals)
        if mean_residuals > self.mean_residuals_thresh:
            return None
        else:
            return np.poly1d(coeffs)

    def update_cam_geometry(self):
        self.cg = CameraGeometry(
            height = self.cg.height, 
            roll_deg = self.cg.roll_deg,
            image_width = self.cg.image_width,
            image_height = self.cg.image_height, 
            field_of_view_deg = self.cg.field_of_view_deg,
            pitch_deg = self.estimated_pitch_deg, 
            yaw_deg = self.estimated_yaw_deg )
        self.cut_v, self.grid = self.cg.precompute_grid()

