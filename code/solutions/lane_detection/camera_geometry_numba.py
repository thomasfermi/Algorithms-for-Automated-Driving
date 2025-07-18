import numpy as np
from numba import int32, float64    # import the types
from numba.experimental import jitclass
from numba.core.errors import NumbaPerformanceWarning
import warnings

warnings.simplefilter('ignore', category=NumbaPerformanceWarning)

spec = [
    ('height', float64),
    ('pitch_deg', float64),     
    ('roll_deg', float64), 
    ('yaw_deg', float64), 
    ('image_width', int32),
    ('image_height', int32),
    ('field_of_view_deg', float64), 
    ('intrinsic_matrix', float64[:,:]), 
    ('inverse_intrinsic_matrix', float64[:,:]), 
    ('rotation_cam_to_road', float64[:,:]), 
    ('translation_cam_to_road', float64[:]), 
    ('trafo_cam_to_road', float64[:,:]),
    ('road_normal_camframe', float64[:])
]


@jitclass(spec)
class CameraGeometry(object):
    def __init__(self, height=1.3, yaw_deg=0, pitch_deg=-5, roll_deg=0, image_width=1024, image_height=512, field_of_view_deg=45):
        # scalar constants
        self.height = height
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.yaw_deg = yaw_deg
        self.image_width = image_width
        self.image_height = image_height
        self.field_of_view_deg = field_of_view_deg
        # camera intriniscs and extrinsics
        self.intrinsic_matrix = self.get_intrinsic_matrix(field_of_view_deg, image_width, image_height)
        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)
        ## Note that "rotation_cam_to_road" has the math symbol R_{rc} in the book
        yaw = np.deg2rad(yaw_deg)
        pitch = np.deg2rad(pitch_deg)
        roll = np.deg2rad(roll_deg)
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        rotation_road_to_cam = np.array([[cr*cy+sp*sr*sy, cr*sp*sy-cy*sr, -cp*sy],
                                            [cp*sr, cp*cr, sp],
                                            [cr*sy-cy*sp*sr, -cr*cy*sp -sr*sy, cp*cy]])
        self.rotation_cam_to_road = rotation_road_to_cam.T # for rotation matrices, taking the transpose is the same as inversion
        self.translation_cam_to_road = np.array([0.0,-self.height,0.0])
        self.trafo_cam_to_road = np.eye(4)
        self.trafo_cam_to_road[0:3,0:3] = self.rotation_cam_to_road
        self.trafo_cam_to_road[0:3,3] = self.translation_cam_to_road
        # compute vector nc. Note that R_{rc}^T = R_{cr}
        self.road_normal_camframe = self.rotation_cam_to_road.T @ np.array([0.0,1.0,0.0], dtype=np.float64)

    @staticmethod
    def get_intrinsic_matrix(field_of_view_deg : np.float64, image_width : np.float64, image_height : np.float64):
        # For our Carla camera alpha_u = alpha_v = alpha
        # alpha can be computed given the cameras field of view via
        field_of_view_rad = field_of_view_deg * np.pi/180
        alpha = (image_width / 2.0) / np.tan(field_of_view_rad / 2.)
        Cu = image_width / 2.0
        Cv = image_height / 2.0
        return np.array([[alpha, 0, Cu],
                        [0, alpha, Cv],
                        [0, 0, 1.0]])

    def camframe_to_roadframe(self,vec_in_cam_frame):
        return self.rotation_cam_to_road @ vec_in_cam_frame + self.translation_cam_to_road

    def uv_to_roadXYZ_camframe(self,u,v):
        # NOTE: The results depend very much on the pitch angle (0.5 degree error yields bad result)
        # Here is a paper on vehicle pitch estimation:
        # https://refubium.fu-berlin.de/handle/fub188/26792
        uv_hom = np.array([u,v,1.0])
        Kinv_uv_hom = self.inverse_intrinsic_matrix @ uv_hom
        denominator = self.road_normal_camframe.dot(Kinv_uv_hom)
        return self.height*Kinv_uv_hom/denominator
    
    def uv_to_roadXYZ_roadframe(self,u,v):
        r_camframe = self.uv_to_roadXYZ_camframe(u,v)
        return self.camframe_to_roadframe(r_camframe)

    def uv_to_roadXYZ_roadframe_iso8855(self,u,v):
        X,Y,Z = self.uv_to_roadXYZ_roadframe(u,v)
        return np.array([Z,-X,-Y]) # read book section on coordinate systems to understand this

    def precompute_grid(self,dist=60):
        cut_v = np.int32(self.compute_minimum_v(dist=dist)+1)
        xy = np.zeros((self.image_width*(self.image_height - cut_v),2 ))
        count = 0
        for v in range(cut_v, self.image_height):
            for u in range(self.image_width):
                X,Y,Z= self.uv_to_roadXYZ_roadframe_iso8855(u,v)
                xy[count][0] = X
                xy[count][1] = Y
                count += 1
        return cut_v, xy

    def compute_minimum_v(self, dist):
        """
        Find cut_v such that pixels with v<cut_v are irrelevant for polynomial fitting.
        Everything that is further than `dist` along the road is considered irrelevant.
        """        
        trafo_road_to_cam = np.linalg.inv(self.trafo_cam_to_road)
        point_far_away_on_road = trafo_road_to_cam @ np.array([0.0,0.0,dist,1.0])
        uv_vec = self.intrinsic_matrix @ point_far_away_on_road[:3]
        uv_vec /= uv_vec[2]
        cut_v = uv_vec[1]
        return cut_v
        