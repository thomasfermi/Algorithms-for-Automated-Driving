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
        raise NotImplementedError

    def get_py_from_vp(u_i, v_i, K):
        raise NotImplementedError
        
