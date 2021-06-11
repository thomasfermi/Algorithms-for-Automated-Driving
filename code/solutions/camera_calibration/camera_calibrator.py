import numpy as np


class CameraCalibrator():
    def __init__(self, lane_detector, cut_v):
        self.ld = lane_detector
        self.cut_v = cut_v
        # build u,v grid
        uv = []
        for v in range(cut_v, self.ld.cg.image_height):
            for u in range(self.ld.cg.image_width):
                uv.append(np.array([u,v]))
        self.uv = np.array(uv)

    def _fit_line_v_of_u(self, probs):
        probs_flat = np.ravel(probs[self.cut_v:, :])
        mask = probs_flat > 0.3
        coeffs = np.polyfit(self.uv[:,0][mask], self.uv[:,1][mask], deg=1, w=probs_flat[mask])
        return np.poly1d(coeffs)
    
    def get_lane_lines_from_file(self, image_fn):
        image = self.ld.read_imagefile_to_array(image_fn)
        return self.get_lane_lines(image)

    def get_lane_lines(self, image):
        _, left, right = self.ld.detect(image)
        return self._fit_line_v_of_u(left), self._fit_line_v_of_u(right)

    def get_vanishing_point_from_file(self, image_fn):
        image = self.ld.read_imagefile_to_array(image_fn)
        return self.get_vanishing_point(image)

    def get_vanishing_point(self, poly_left, poly_right):
        m1, c1 = poly_left
        m2, c2 = poly_right

        u_i = (c2 - c1) / (m1 - m2)
        v_i = m1*u_i + c1
        return u_i, v_i

    def get_py_from_vp(self, u_i, v_i, K):
        p_infinity = np.array([u_i, v_i, 1])
        K_inv = np.linalg.inv(K)
        K_inv_p_infinity = K_inv @ p_infinity
        K_inv_p_infinity
        
        r3 = K_inv_p_infinity / np.linalg.norm(K_inv_p_infinity)
        yaw = np.arctan2(r3[0], r3[2])
        pitch = -np.arcsin(r3[1])
        
        return (pitch, yaw)
        
