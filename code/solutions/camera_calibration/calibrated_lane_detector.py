import numpy as np
from ..lane_detection.lane_detector import LaneDetector
from ..lane_detection.camera_geometry import CameraGeometry


def get_intersection(line1, line2):
    m1, c1 = line1
    m2, c2 = line2
    if m1 == m2:
        return None
    u_i = (c2 - c1) / (m1 - m2)
    v_i = m1*u_i + c1
    return u_i, v_i


def get_py_from_vp(u_i, v_i, K):
    p_infinity = np.array([u_i, v_i, 1])
    K_inv = np.linalg.inv(K)
    K_inv_p_infinity = K_inv @ p_infinity
    K_inv_p_infinity
    
    r3 = K_inv_p_infinity / np.linalg.norm(K_inv_p_infinity)
    yaw = np.arctan2(r3[0], r3[2])
    pitch = -np.arcsin(r3[1])
    
    return pitch, yaw

class CalibratedLaneDetector(LaneDetector):
    def __init__(self, calib_cut_v = 200, cam_geom=CameraGeometry(), model_path='./lane_segmentation_model.pth', 
        encoder = 'efficientnet-b0', encoder_weights = 'imagenet'):
        # call parent class constructor
        super().__init__(cam_geom, model_path, encoder, encoder_weights)

        self.calib_cut_v = calib_cut_v

        # build u,v grid
        uv = []
        for v in range(calib_cut_v, self.cg.image_height):
            for u in range(self.cg.image_width):
                uv.append(np.array([u,v]))
        self.uv_grid = np.array(uv)

        self.pitch_yaw_history = []
        self.calibration_success = False
    
    def _fit_line_v_of_u(self, probs):
        probs_flat = np.ravel(probs[self.calib_cut_v:, :])
        mask = probs_flat > 0.3
        coeffs, residuals, _, _, _ = np.polyfit(
            self.uv_grid[:,0][mask], self.uv_grid[:,1][mask], deg=1, w=probs_flat[mask], full=True)
        mean_residuals = residuals/len(self.uv_grid[:,0][mask])
        #print(mean_residuals)
        if mean_residuals < 15:
            return np.poly1d(coeffs)
        else:
            return None

    def calibrate(self, image, mpl_axis=None):
        _, left_probs, right_probs = self.detect(image)
        line_left  = self._fit_line_v_of_u(left_probs)
        line_right = self._fit_line_v_of_u(right_probs)
        if line_left is None or line_right is None:
            return False
        vanishing_point = get_intersection(line_left, line_right)
        if vanishing_point is None:
            return False      
          
        if mpl_axis is not None:
            self.visualize_vanishing_point(
                image, line_left, line_right, vanishing_point, mpl_axis)
        u_i, v_i = vanishing_point
        pitch, yaw = get_py_from_vp(u_i, v_i, self.cg.intrinsic_matrix)
        self.add_to_pitch_yaw_history(pitch, yaw)
        return True

    def add_to_pitch_yaw_history(self, pitch, yaw):
        self.pitch_yaw_history.append([pitch, yaw])
        if len(self.pitch_yaw_history) > 50:
            py = np.array(self.pitch_yaw_history)
            mean_pitch = np.mean(py[:,0])
            mean_yaw = np.mean(py[:,1])
            self.update_cam_geometry(mean_pitch, mean_yaw)
            self.calibration_success = True
            self.pitch_yaw_history = []
            print("yaw, pitch = ", mean_yaw*180/np.pi, mean_pitch*180/np.pi)

    def update_cam_geometry(self, pitch, yaw):
        # TODO: CameraGeometry currently does not allow to set different yaw value :(
        self.cg = CameraGeometry(pitch_deg = np.rad2deg(pitch))
        self.cut_v, self.grid = self.cg.precompute_grid()
    
    def visualize_vanishing_point(self, image, line_left, line_right, vanishing_point, mpl_axis):
        u = np.arange(0,self.cg.image_width, 1)
        v_left = line_left(u)
        v_right = line_right(u)

        mpl_axis.imshow(image)
        # plot detected lane lines
        mpl_axis.plot(u,v_left)
        mpl_axis.plot(u,v_right)
        mpl_axis.set_xlim(0,self.cg.image_width)
        mpl_axis.set_ylim(self.cg.image_height,0)

        # plot intersection of lane lines
        u_i, v_i = vanishing_point
        mpl_axis.scatter([u_i],[v_i], marker="o", s=100, color="r", zorder=10)

    def __call__(self, img):
        self.calibrate(img)
        # only return lane lines once calibrated
        if self.calibration_success:
            return super().__call__(img)
        else:
            return None
