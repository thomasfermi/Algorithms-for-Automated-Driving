# To run this, cd into the parent directory of the code folder an then run
# python -m code.tests.lane_detection.camera_geometry_unit_test 1
import numpy as np
from pathlib import Path
import argparse
import logging
from ...solutions.lane_detection.camera_geometry import CameraGeometry as sln_CameraGeometry

def compare_arrays(sln, ex, failure_string, success_string):
    if ex is None:
        print("You returned None instead of a proper numpy array!")
    if type(sln) != type(ex):
        print(failure_string, "You did not return a numpy array!")
        return False
    if sln.shape != ex.shape:
        print(failure_string)
        print("The numpy array you have returned should have shape {} but its shape is {}!".format(sln.shape, ex.shape))
        return False
    if np.isclose(sln, ex).all():
        print(success_string)
        return True
    else:
        print(failure_string, "You returned:\n {}\n but the solution is:\n {}\n.".format(ex, sln))
        return False

def test_project_polyline(boundary, trafo_world_to_cam, K):
    from ...exercises.lane_detection.camera_geometry import project_polyline as ex_project_polyline
    from ...solutions.lane_detection.camera_geometry import project_polyline as sln_project_polyline

    res_sln = sln_project_polyline(boundary[:,0:3], trafo_world_to_cam, K)
    
    try:
        res_ex = ex_project_polyline(boundary[:,0:3], trafo_world_to_cam, K) 
        result = compare_arrays(res_sln, res_ex, "Test for project_polyline failed.",
            "Your function project_polyline seems to be correct!")
    except NotImplementedError:
        print("Test for project_polyline failed. You did not implement the function!")
        return False
    except BaseException:
        logging.exception("Test for project_polyline failed. Your code raised an exception! I will show you the traceback:")
        return False
    return result
    

def test_get_intrinsic_matrix():
    from ...exercises.lane_detection.camera_geometry import get_intrinsic_matrix as ex_get_intrinsic_matrix
    from ...solutions.lane_detection.camera_geometry import get_intrinsic_matrix as sln_get_intrinsic_matrix
    
    res_sln = sln_get_intrinsic_matrix(45, 1024, 512)

    try:
        res_ex = ex_get_intrinsic_matrix(45, 1024, 512)
        result = compare_arrays(res_sln, res_ex, "Test for get_intrinsic_matrix failed.",
            "Your function get_intrinsic_matrix seems to be correct!")
    except NotImplementedError:
        print("Test for get_intrinsic_matrix failed. You did not implement the function!")
        return False
    except BaseException:
        logging.exception("Test for get_intrinsic_matrix failed. Your code raised an exception! I will show you the traceback:")
        return False
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Perform unit tests for the camera_geometry module.')
    parser.add_argument('step', type=int, help='Can be either 1,2, or 3. You should first pass all tests of step 1, then step 2, and finally step 3')
    args = parser.parse_args()
    step = args.step
    if step in [1,2,3]:
        print("-------------------------")
        print("Running tests for step ", step)
        print("-------------------------")
    else:
        print("Error! Step argument needs to be 1, 2, or 3. For example you can run\npython -m code.tests.lane_detection.camera_geometry_unit_test 1")


    sln_cg = sln_CameraGeometry()

    # Load some data
    data_path = Path('data/')
    boundary_fn = data_path / "Town04_Clear_Noon_09_09_2020_14_57_22_frame_625_validation_set_boundary.txt"
    boundary = np.loadtxt(boundary_fn)

    trafo_fn = data_path / "Town04_Clear_Noon_09_09_2020_14_57_22_frame_625_validation_set_trafo.txt"
    trafo_world_to_cam = np.loadtxt(trafo_fn)

    # Run tests
    
    # Test exercise 1
    if step == 1:
        test_project_polyline(boundary, trafo_world_to_cam, sln_cg.intrinsic_matrix)
        test_get_intrinsic_matrix()

    # Test exercise 2
    if step == 2:
        from ...exercises.lane_detection.camera_geometry import CameraGeometry as ex_CameraGeometry

        try:
            ex_cg = ex_CameraGeometry()
            compare_arrays(sln_cg.rotation_cam_to_road, ex_cg.rotation_cam_to_road, 
                "You did not calculate rotation_cam_to_road correctly in your CameraGeometry class.",
                "It seems that you computed rotation_cam_to_road correctly!")

            compare_arrays(sln_cg.translation_cam_to_road, ex_cg.translation_cam_to_road, 
                "You did not calculate translation_cam_to_road correctly in your CameraGeometry class.",
                "It seems that you computed translation_cam_to_road correctly!")

            compare_arrays(sln_cg.trafo_cam_to_road, ex_cg.trafo_cam_to_road, 
                "You did not calculate trafo_cam_to_road correctly in your CameraGeometry class.",
                "It seems that you computed trafo_cam_to_road correctly!")

            compare_arrays(sln_cg.road_normal_camframe, ex_cg.road_normal_camframe, 
                "You did not calculate road_normal_camframe correctly in your CameraGeometry class.",
                "It seems that you computed road_normal_camframe correctly!")

            for u,v in [(76,982), (444, 711), (2,1022)]:
                compare_arrays(sln_cg.uv_to_roadXYZ_camframe(u,v), ex_cg.uv_to_roadXYZ_camframe(u,v), 
                    "Your function uv_to_roadXYZ_camframe() did not compute the correct result for u,v = {},{}".format(u,v),
                    "Your function uv_to_roadXYZ_camframe() worked correctkly for u,v={},{}!".format(u,v))
        except BaseException:
            logging.exception("An exception was thrown in your CameraGeometry class! I will show you the traceback:")

    # Test exercise 3
    if step == 3:
        from ...exercises.lane_detection.camera_geometry import CameraGeometry as ex_CameraGeometry
        try:
            ex_cg = ex_CameraGeometry()            
            compare_arrays(sln_cg.precompute_grid()[1], ex_cg.precompute_grid()[1], 
                "Your function precompute_grid() did not compute the correct grid.",
                "Your function precompute_grid() seems to be correct!")       
        except BaseException:
            logging.exception("An exception was thrown in your CameraGeometry class! I will show you the traceback:")