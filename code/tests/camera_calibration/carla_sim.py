# Code based on Carla examples, which are authored by 
# Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).

# How to run: 
# cd into the parent directory of the 'code' directory and run
# python -m code.tests.control.carla_sim


import carla
import random
from pathlib import Path
import numpy as np
import pygame
from ...util.carla_util import carla_vec_to_np_array, carla_img_to_array, CarlaSyncMode, find_weather_presets, draw_image_np, should_quit
from ...util.geometry_util import dist_point_linestring
import argparse
import cv2
import copy



main_image_shape = (800, 600)
model_filename = "fastai_model.pth"


def get_trajectory_from_lane_detector(ld, image):
    # get lane boundaries using the lane detector
    img = carla_img_to_array(image)
    poly_left, poly_right, left_mask, right_mask = ld.get_fit_and_probs(img)
    # trajectory to follow is the mean of left and right lane boundary
    # note that we multiply with -0.5 instead of 0.5 in the formula for y below
    # according to our lane detector x is forward and y is left, but
    # according to Carla x is forward and y is right.
    x = np.arange(-2,60,1.0)
    y = -0.5*(poly_left(x)+poly_right(x))
    # x,y is now in coordinates centered at camera, but camera is 0.5 in front of vehicle center
    # hence correct x coordinates
    x += 0.5
    traj = np.stack((x,y)).T
    return traj, ld_detection_overlay(img, left_mask, right_mask)

def ld_detection_overlay(image, left_mask, right_mask):
    res = copy.copy(image)
    res[left_mask > 0.5, :] = [0,0,255]
    res[right_mask > 0.5, :] = [255,0,0]
    return res


def get_trajectory_from_map(m, vehicle):
    # get 80 waypoints each 1m apart. If multiple successors choose the one with lower waypoint.id
    wp = m.get_waypoint(vehicle.get_transform().location)
    wps = [wp]
    for _ in range(20):
        next_wps = wp.next(1.0)
        if len(next_wps) > 0:
            wp = sorted(next_wps, key=lambda x: x.id)[0]
        wps.append(wp)

    # transform waypoints to vehicle ref frame
    traj = np.array(
        [np.array([*carla_vec_to_np_array(x.transform.location), 1.]) for x in wps]
    ).T
    trafo_matrix_world_to_vehicle = np.array(vehicle.get_transform().get_inverse_matrix())

    traj = trafo_matrix_world_to_vehicle @ traj
    traj = traj.T
    traj = traj[:,:2]
    return traj

def send_control(vehicle, throttle, steer, brake,
                 hand_brake=False, reverse=False):
    throttle = np.clip(throttle, 0.0, 1.0)
    steer = np.clip(steer, -1.0, 1.0)
    brake = np.clip(brake, 0.0, 1.0)
    control = carla.VehicleControl(throttle, steer, brake, hand_brake, reverse)
    vehicle.apply_control(control)



def main(yaw_deg=0, pitch_deg = 0, ex=False, save_video=False, half_image=False):
    # Imports
    if ex:
        #from ...exercises.camera_calibration.calibrated_lane_detector import CalibratedLaneDetector
        from ...exercises.lane_detection.camera_geometry import CameraGeometry
        from ...exercises.control.pure_pursuit import PurePursuitPlusPID
    else:
        from ...solutions.camera_calibration.calibrated_lane_detector import CalibratedLaneDetector
        from ...solutions.lane_detection.camera_geometry_numba import CameraGeometry
        from ...solutions.control.pure_pursuit import PurePursuitPlusPID        

    if save_video:
        import atexit
        import imageio
        #import time
        images = []
        from tqdm import tqdm
        video_writer = imageio.get_writer('my_video.mp4', format='FFMPEG', mode='I', fps=30)
        
        def write_images_to_video(images, video_writer):
            print("Writing images to video file...")
            for img in tqdm(images): 
                video_writer.append_data(img)
            video_writer.close()
        atexit.register(lambda: write_images_to_video(images, video_writer))

    actor_list = []
    pygame.init()

    display = pygame.display.set_mode(
        main_image_shape,
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = pygame.font.SysFont("monospace", 15)
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(80.0)

    #client.load_world('Town06')
    client.load_world('Town04')
    world = client.get_world()

    weather_preset, _ = find_weather_presets()[0]
    world.set_weather(weather_preset)

    controller = PurePursuitPlusPID()

    try:
        m = world.get_map()

        blueprint_library = world.get_blueprint_library()

        veh_bp = random.choice(blueprint_library.filter('vehicle.audi.tt'))
        veh_bp.set_attribute('color','64,81,181')
        vehicle = world.spawn_actor(
            veh_bp,
            m.get_spawn_points()[90])
        actor_list.append(vehicle)


        # visualization cam (no functionality)
        camera_rgb = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-10)),
            attach_to=vehicle)
        actor_list.append(camera_rgb)
        sensors = [camera_rgb]

        if half_image:
            cam_geom = CameraGeometry(image_width=512, image_height=256)
        else:
            cam_geom = CameraGeometry()

        if not ex:
            ld = CalibratedLaneDetector(model_path=Path("code/solutions/lane_detection/"+ model_filename).absolute(), cam_geom=cam_geom, calib_cut_v = 200)            
        else:
            # TODO: Change this so that it works with your lane detector implementation
            # pass cam_geom to make sure that this works with both half_image==True and ==False
            ld = CalibratedLaneDetector(cam_geom=cam_geom)
        #windshield cam
        cg = cam_geom
        cam_windshield_transform = carla.Transform(carla.Location(x=0.5, z=cg.height), carla.Rotation(pitch=pitch_deg, yaw=yaw_deg))
        bp = blueprint_library.find('sensor.camera.rgb')
        fov = cg.field_of_view_deg
        bp.set_attribute('image_size_x', str(cg.image_width))
        bp.set_attribute('image_size_y', str(cg.image_height))
        bp.set_attribute('fov', str(fov))
        camera_windshield = world.spawn_actor(
            bp,
            cam_windshield_transform,
            attach_to=vehicle)
        actor_list.append(camera_windshield)
        sensors.append(camera_windshield)


        frame = 0
        max_error = 0
        FPS = 30
        # Create a synchronous mode context.
        with CarlaSyncMode(world, *sensors, fps=FPS) as sync_mode:
            while True:
                if should_quit():
                    return
                clock.tick()          
                
                # Advance the simulation and wait for the data. 
                tick_response = sync_mode.tick(timeout=2.0)

                snapshot, image_rgb, image_windshield = tick_response
                if frame % 2 == 0: 
                    traj, viz = get_trajectory_from_lane_detector(ld, image_windshield)                 
                    if not ld.calibration_success:
                        print("ld still calibrating")

                # get velocity and angular velocity
                vel = carla_vec_to_np_array(vehicle.get_velocity())
                forward = carla_vec_to_np_array(vehicle.get_transform().get_forward_vector())
                right = carla_vec_to_np_array(vehicle.get_transform().get_right_vector())
                up = carla_vec_to_np_array(vehicle.get_transform().get_up_vector())
                vx = vel.dot(forward)
                vy = vel.dot(right)
                vz = vel.dot(up)
                ang_vel = carla_vec_to_np_array(vehicle.get_angular_velocity())
                w = ang_vel.dot(up)
                print("vx vy vz w {:.2f} {:.2f} {:.2f} {:.5f}".format(vx,vy,vz,w))

                speed = np.linalg.norm( carla_vec_to_np_array(vehicle.get_velocity()))
                throttle, steer = controller.get_control(traj, speed, desired_speed=25, dt=1./FPS)
                send_control(vehicle, throttle, steer, 0)

                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                dist = dist_point_linestring(np.array([0,0]), traj)

                cross_track_error = int(dist*100)
                max_error = max(max_error, cross_track_error)

                # Draw the display.
                image_rgb = copy.copy(carla_img_to_array(image_rgb))
                # draw lane detection viz
                viz = cv2.resize(viz, (400,200), interpolation = cv2.INTER_AREA)
                image_rgb[0:viz.shape[0], 0:viz.shape[1],:] = viz
                # white background for text
                image_rgb[10:220,-280:-10, : ] = [255,255,255]
                
                draw_image_np(display, image_rgb)

                # draw txt
                dy = 20
                texts = ["FPS (real):             {}".format(int(clock.get_fps())),
                         "FPS (simulated):        {}".format(fps),
                         "speed (m/s):            {:.2f}".format(speed),
                         "lateral error (cm):     {}".format(cross_track_error),
                         "max lat. error (cm):    {}".format(max_error),
                         "true yaw (deg):         {:.1f}".format(yaw_deg),
                         "calib yaw (deg):        {:.1f}".format(ld.estimated_yaw_deg),
                         "true pitch (deg):       {:.1f}".format(pitch_deg),
                         "calib pitch (deg):      {:.1f}".format(ld.estimated_pitch_deg)
                        ]
                
                for it,t in enumerate(texts):
                    display.blit(
                        font.render(t, True, (0,0,0)), (image_rgb.shape[1]-270, 20+dy*it))

                pygame.display.flip()

                frame += 1
                if save_video and frame > 0:
                    print("frame=",frame)
                    imgdata = pygame.surfarray.array3d(pygame.display.get_surface())
                    imgdata = imgdata.swapaxes(0,1)
                    images.append(imgdata)
                

    finally:

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        pygame.quit()
        print('done.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs Carla simulation with your control algorithm and the calibrated lane detector.',
        epilog="Example usage:\n\n   python -m code.tests.camera_calibration.carla_sim 3 -4 --vid\n \n",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("yaw_deg", type=float, help="camera mounting yaw angle in degrees")
    parser.add_argument("pitch_deg", type=float, help="camera mounting pitch angle in degrees")
    parser.add_argument("--ex", action="store_true", help="Run student code")
    parser.add_argument("--vid", action="store_true", help="Save video after simulation")
    parser.add_argument("--half_image", action="store_true", help="Pass images with (width, height) = (512,256) to lane detector instead of the default (1024,512). This will speed up the simulation, but might hurt accuracy.")
    args = parser.parse_args()

    try:
        main(args.yaw_deg, args.pitch_deg, ex = args.ex, save_video=args.vid, half_image=args.half_image)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
