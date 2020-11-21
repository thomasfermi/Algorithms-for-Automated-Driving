import numpy as np

def get_intrinsic_matrix(field_of_view_deg, image_width, image_height):
    """
    Returns intrinsic matrix K.
    """
    # For our Carla camera alpha_u = alpha_v = alpha
    # alpha can be computed given the cameras field of view via
    field_of_view_rad = field_of_view_deg * np.pi/180
    alpha = (image_width / 2.0) / np.tan(field_of_view_rad / 2.)
    # TODO step 1: Complete this function
    raise NotImplementedError

def project_polyline(polyline_world, trafo_world_to_cam, K):
    """
    Returns array uv which contains the pixel coordinates of the polyline.

    Parameters
    ----------
    polyline_world : array_like, shape (M,3)
        Each row of this array is a vertex (x,y,z) of the polyline.
    trafo_world_to_cam : array_like, shape (4,4)
        Transformation matrix, that maps vectors (x_world, y_world, z_world, 1) 
        to vectors (x_cam, y_cam, z_cam, 1).
    K: array_like, shape (3,3)
        Intrinsic matrix of  the camera.   
    
    Returns:
    --------
    uv : ndarray, shape (M,2)
        Pixel coordinates of the projected polyline
        First column is u, second column is v
    """
    # TODO step 1: Write this function
    raise NotImplementedError


class CameraGeometry(object):
    def __init__(self, height=1.3, pitch_deg=5, image_width=1024, image_height=512, field_of_view_deg=45):
        # scalar constants
        self.height = height
        self.pitch_deg = pitch_deg
        self.image_width = image_width
        self.image_height = image_height
        self.field_of_view_deg = field_of_view_deg
        # camera intriniscs and extrinsics
        self.intrinsic_matrix = get_intrinsic_matrix(field_of_view_deg, image_width, image_height)
        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)
        ## Note that "rotation_cam_to_road" has the math symbol R_{rc} in the book
        pitch = pitch_deg * np.pi/180
        cpitch, spitch = np.cos(pitch), np.sin(pitch)
        # TODO step 2: replace the four 'None' values in the following code with correct expressions
        self.rotation_cam_to_road = None
        self.translation_cam_to_road = None
        self.trafo_cam_to_road = None
        # compute vector nc. Note that R_{rc}^T = R_{cr}
        self.road_normal_camframe = None


    def camframe_to_roadframe(self,vec_in_cam_frame):
        return self.rotation_cam_to_road @ vec_in_cam_frame + self.translation_cam_to_road

    def uv_to_roadXYZ_camframe(self,u,v):
        """
        Inverse perspective mapping from pixel coordinates to 3d coordinates.
        
        Parameters
        ----------
        u,v: Both float
            Pixel coordinates of some part of the road.
        
        Returns:
        --------
        XYZ: array_like, shape(3,)
            Three dimensional point in the camera reference frame that lies on the road
            and was mapped by the camera to pixel coordinates u,v
        """
        # TODO step 2: Write this function
        raise NotImplementedError
    
    def uv_to_roadXYZ_roadframe(self,u,v):
        r_camframe = self.uv_to_roadXYZ_camframe(u,v)
        return self.camframe_to_roadframe(r_camframe)

    def uv_to_roadXYZ_roadframe_iso8855(self,u,v):
        X,Y,Z = self.uv_to_roadXYZ_roadframe(u,v)
        return np.array([Z,-X,-Y]) # read book section on coordinate systems to understand this

    def precompute_grid(self,dist=60):
        """
        Precomputes a grid that will be used for polynomial fitting at a later stage.

        Parameters
        ----------
        dist : float
            Distance thereshold in meters. For the grid, only pixel coordinates [u,v]
            are considered that depict parts of the road plane that are no more than
            a distance `dist` away along the road.
        
        Returns:
        --------
        cut_v: float
            Threshold for the pixel coordinate v, that corresponds to the `dist`input.

        grid: array_like, shape (M,2)
            A list of x,y coordinates. Each element corresponds to the x-y coordinates
            of one pixel [u,v] (v>cut_v).
        """
        cut_v = int(self.compute_minimum_v(dist=dist)+1)
        # TODO step 3: compute `grid`
        grid = None
        return cut_v, grid

    def compute_minimum_v(self, dist):
        """
        Find cut_v such that pixels with v<cut_v are irrelevant for polynomial fitting.
        Everything that is further than `dist` along the road is considered irrelevant.
        """        
        trafo_road_to_cam = np.linalg.inv(self.trafo_cam_to_road)
        point_far_away_on_road = trafo_road_to_cam @ np.array([0,0,dist,1])
        uv_vec = self.intrinsic_matrix @ point_far_away_on_road[:3]
        uv_vec /= uv_vec[2]
        cut_v = uv_vec[1]
        return cut_v