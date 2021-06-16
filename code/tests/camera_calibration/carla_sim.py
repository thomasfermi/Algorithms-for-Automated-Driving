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
from ...util.carla_util import carla_vec_to_np_array, carla_img_to_array, CarlaSyncMode, find_weather_presets, draw_image, get_font, should_quit
from ...util.geometry_util import dist_point_linestring
import argparse




save_gif = True

if save_gif:
    import atexit
    import imageio
    #import time
    images = []
    atexit.register(lambda: imageio.mimsave('control.gif',
                                            images, fps=30))


def get_trajectory_from_lane_detector(ld, image):
    # get lane boundaries using the lane detector
    image_arr = carla_img_to_array(image)
    poly_left, poly_right = ld(image_arr)
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
    return traj

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



def main(yaw_err_deg=0, pitch_err_deg = 0, use_calibrated_lane_detector=False, ex=False):
    # Imports
    if not use_calibrated_lane_detector:
        if ex:
            from ...exercises.lane_detection.lane_detector import LaneDetector
            from ...exercises.lane_detection.camera_geometry import CameraGeometry    
        else:
            from ...solutions.lane_detection.lane_detector import LaneDetector
            from ...solutions.lane_detection.camera_geometry import CameraGeometry
    else:
        if ex:
            print("#todo")
            #from ...exercises.camera_calibration.calibrated_lane_detector import CalibratedLaneDetector
            #from ...exercises.camera_calibration.calibrated_lane_detector import CameraGeometry    
        else:
            from ...solutions.camera_calibration.calibrated_lane_detector import CalibratedLaneDetector
            from ...solutions.camera_calibration.calibrated_lane_detector import CameraGeometry

    if ex:
        from ...exercises.control.pure_pursuit import PurePursuitPlusPID
    else:
        from ...solutions.control.pure_pursuit import PurePursuitPlusPID

    actor_list = []
    pygame.init()

    display = pygame.display.set_mode(
        (800, 600),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(80.0)

    #client.load_world('Town06')
    client.load_world('Town04')
    world = client.get_world()
    weather_presets = find_weather_presets()
    world.set_weather(weather_presets[3][0])

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
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            attach_to=vehicle)
        actor_list.append(camera_rgb)
        sensors = [camera_rgb]

        

           
        if not ex:
            if use_calibrated_lane_detector:
                ld = CalibratedLaneDetector(model_path=Path("code/solutions/lane_detection/best_model_multi_dice_loss.pth").absolute())
            else:
                ld = LaneDetector(model_path=Path("code/solutions/lane_detection/best_model_multi_dice_loss.pth").absolute())
        else:
            # TODO: Change this so that it works with your lane detector implementation
            if use_calibrated_lane_detector:
                ld = CalibratedLaneDetector()
            else:
                ld = LaneDetector()
        #windshield cam
        cg = CameraGeometry()
        cam_windshield_transform = carla.Transform(carla.Location(x=0.5, z=cg.height), carla.Rotation(pitch=cg.pitch_deg+pitch_err_deg, yaw=cg.yaw_deg + yaw_err_deg))
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
                if use_calibrated_lane_detector:
                    if ld.calibration_success:
                        traj = get_trajectory_from_lane_detector(ld, image_windshield)
                    else:
                        ld.calibrate(carla_img_to_array(image_windshield))
                        traj = get_trajectory_from_map(m, vehicle)
                        print("ld still calibrating")
                else: #standard lane detector
                    traj = get_trajectory_from_lane_detector(ld, image_windshield)

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
                draw_image(display, image_rgb)
                display.blit(
                    font.render('     FPS (real) % 5d ' % clock.get_fps(), True, (255, 255, 255)),
                    (8, 10))
                display.blit(
                    font.render('     FPS (simulated): % 5d ' % fps, True, (255, 255, 255)),
                    (8, 28))
                display.blit(
                    font.render('     speed: {:.2f} m/s'.format(speed), True, (255, 255, 255)),
                    (8, 46))
                display.blit(
                    font.render('     cross track error: {:03d} cm'.format(cross_track_error), True, (255, 255, 255)),
                    (8, 64))
                display.blit(
                    font.render('     max cross track error: {:03d} cm'.format(max_error), True, (255, 255, 255)),
                    (8, 82))

                pygame.display.flip()

                frame += 1
                if save_gif and frame > 1000:
                    print("frame=",frame)
                    imgdata = pygame.surfarray.array3d(pygame.display.get_surface())
                    imgdata = imgdata.swapaxes(0,1)
                    if frame < 1200:
                        images.append(imgdata)
                

    finally:

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        pygame.quit()
        print('done.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs Carla simulation with your control algorithm.')
    parser.add_argument("yaw_error_deg", type=float, help="mounting error in yaw in degrees")
    parser.add_argument("pitch_error_deg", type=float, help="mounting error in pitch in degrees")
    parser.add_argument("--c", action="store_true", help="Use calibrated lane detector")
    parser.add_argument("--ex", action="store_true", help="Run student code")
    args = parser.parse_args()

    try:
        main(args.yaw_error_deg, args.pitch_error_deg, use_calibrated_lane_detector = args.c, ex = args.ex)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
