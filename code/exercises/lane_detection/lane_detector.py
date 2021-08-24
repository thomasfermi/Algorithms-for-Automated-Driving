from .camera_geometry import CameraGeometry
import numpy as np
import cv2


class LaneDetector():
    # TODO: You will probably want to add your own arguments to the constructor
    def __init__(self, model_path, cam_geom=CameraGeometry()):
        self.cg = cam_geom
        self.cut_v, self.grid = self.cg.precompute_grid()
        # TODO: Add variables for your lane segmentation deep learning model
        # ...
        self.model = None # change this line

    def read_imagefile_to_array(self, filename):
        image = cv2.imread(filename)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image
    
    def detect_from_file(self, filename):
        img_array = self.read_imagefile_to_array(filename)
        return self.detect(img_array)

    def detect(self, img_array):
        """
        Detects which pixels are part of a lane boundary.

        Parameters
        ----------
        img_array : array_like, shape (image_height,image_width)
            Image as a numpy array

        Returns:
        --------
        background, prob_left, prob_right: Each one array_like, shape (image_height,image_width)
            Probability maps. For example
            prob_left[v,u] = probability(pixel (u,v) is part of left lane boundary)
        """
        # TODO: Use your lane segmentation deep learning model to implement this function
        raise NotImplementedError

    def fit_poly(self, probs):
        """ 
        Fits a polynomial of order 3 to the lane boundary.
        
        Parameters
        ----------
        probs : array_like, shape (image_height,image_width)
            Probability for each pixel that it shows a lane boundary

        Returns:
        --------
        poly: numpy.poly1d
            numpy poly1d object representing the lane boundary as a polynomial.
            The polynomial is y(x)=c0+c1*x+c2*x**2+c3*x**3, where x is the forward
            direction along the road in meters and y is the sideways direction.
            Hint:
            You will want to use self.grid here. 
            self.grid[:,0] contains all x values
            self.grid[:,1] contains all y values
            np.ravel(probs[self.cut_v:, :]) contains all probability values.
        """
        # TODO: Implement this function. You will need self.cut_v, and self.grid 
        raise NotImplementedError

    def __call__(self, image):
        if isinstance(image, str):
            image = self.read_imagefile_to_array(image)
        left_poly, right_poly, _, _ = self.get_fit_and_probs(image)
        return left_poly, right_poly

    def get_fit_and_probs(self, img):
        _, left, right = self.detect(img)
        left_poly = self.fit_poly(left)
        right_poly = self.fit_poly(right)
        return left_poly, right_poly, left, right

    
