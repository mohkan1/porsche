import glob
import os
import sys
import time
import math
import weakref

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import random

def main():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)

    enable_rgb_camera = 0
    enable_logarithmic_depth = 0
    enable_depth_camera = 0
    enable_semantic_segmentation = 0
    enable_radar = 0
    enable_lidar = 0
    enable_imu = 0
    enable_gnss = 0
    enable_collision = 0
    enable_lane_invasion = 0
    enable_obstacle_sensor = 0

    follow_car = 1

    try:

        world = client.get_world() 
        ego_vehicle = None
        ego_cam = None
        depth_cam = None
        depth_cam02 = None
        sem_cam = None
        rad_ego = None
        lidar_sen = None
        ego_col = None
        ego_lane = None
        ego_obs = None
        ego_gnss = None
        ego_imu = None

        # --------------
        # Query the recording
        # --------------
        
        # Show the most important events in the recording.  
        print(client.show_recorder_file_info("data/recorder/recording05.log",False))
        # Show actors not moving 1 meter in 10 seconds.  
        #print(client.show_recorder_actors_blocked("data/recorder/recording04.log",10,1))
        # Show collisions between any type of actor.  
        #print(client.show_recorder_collisions("data/recorder/recording04.log",'v','a'))
        

        # --------------
        # Reenact a fragment of the recording
        # --------------
        
        client.replay_file("data/recorder/recording03.log",0,30,0)
        

        # --------------
        # Set playback simulation conditions
        # --------------
        
        # ego_vehicle = world.get_actor(322) #Store the ID from the simulation or query the recording to find out
        

        # --------------
        # Spawn ego vehicle
        # --------------
        
        
        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name','ego')
        print('\nEgo role_name is set')
        ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        ego_bp.set_attribute('color',ego_color)
        print('\nEgo color is set')

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            ego_transform = spawn_points[0]
            ego_vehicle = world.spawn_actor(ego_bp,ego_transform)
            print('\nEgo is spawned')
        else: 
            logging.warning('Could not found any spawn points')

        # --------------
        # Place spectator on ego spawning
        # --------------
        
        spectator = world.get_spectator()
        world_snapshot = world.wait_for_tick() 
        spectator.set_transform(ego_vehicle.get_transform())
        
        ego_vehicle.set_autopilot(True)
        
        # --------------
        # Change weather conditions
        # --------------
        
        # weather = world.get_weather()
        # weather.sun_altitude_angle = -30
        # weather.fog_density = 65
        # weather.fog_distance = 10
        # world.set_weather(weather)

        # --------------
        # Add a RGB camera to ego vehicle.
        # --------------
        if enable_rgb_camera:
            cam_bp = None
            cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
            cam_location = carla.Location(2,0,1)
            cam_rotation = carla.Rotation(0,0,0)
            cam_transform = carla.Transform(cam_location,cam_rotation)
            cam_bp.set_attribute("image_size_x",str(1920))
            cam_bp.set_attribute("image_size_y",str(1080))
            cam_bp.set_attribute("fov",str(105))
            ego_cam = world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            ego_cam.listen(lambda image: image.save_to_disk('data/new_rgb_output/%.6d.jpg' % image.frame))
        

        # --------------
        # Add a Logarithmic Depth camera to ego vehicle. 
        # --------------
        if enable_logarithmic_depth:
            depth_cam = None
            depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
            depth_bp.set_attribute("image_size_x",str(1920))
            depth_bp.set_attribute("image_size_y",str(1080))
            depth_bp.set_attribute("fov",str(105))
            depth_location = carla.Location(2,0,1)
            depth_rotation = carla.Rotation(0,0,0)
            depth_transform = carla.Transform(depth_location,depth_rotation)
            depth_cam = world.spawn_actor(depth_bp,depth_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            # This time, a color converter is applied to the image, to get the semantic segmentation view
            depth_cam.listen(lambda image: image.save_to_disk('data/de_log/%.6d.jpg' % image.frame,carla.ColorConverter.LogarithmicDepth))
        
        # --------------
        # Add a Depth camera to ego vehicle. 
        # --------------
        if enable_depth_camera:
            depth_cam02 = None
            depth_bp02 = world.get_blueprint_library().find('sensor.camera.depth')
            depth_bp02.set_attribute("image_size_x",str(1920))
            depth_bp02.set_attribute("image_size_y",str(1080))
            depth_bp02.set_attribute("fov",str(105))
            depth_location02 = carla.Location(2,0,1)
            depth_rotation02 = carla.Rotation(0,0,0)
            depth_transform02 = carla.Transform(depth_location02,depth_rotation02)
            depth_cam02 = world.spawn_actor(depth_bp02,depth_transform02,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            # This time, a color converter is applied to the image, to get the semantic segmentation view
            depth_cam02.listen(lambda image: image.save_to_disk('data/de/%.6d.jpg' % image.frame,carla.ColorConverter.Depth))
        

        # --------------
        # Add a new semantic segmentation camera to ego vehicle
        # --------------
        if enable_semantic_segmentation:
            sem_cam = None
            sem_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            sem_bp.set_attribute("image_size_x",str(1920))
            sem_bp.set_attribute("image_size_y",str(1080))
            sem_bp.set_attribute("fov",str(105))
            sem_location = carla.Location(2,0,1)
            sem_rotation = carla.Rotation(0,0,0)
            sem_transform = carla.Transform(sem_location,sem_rotation)
            sem_cam = world.spawn_actor(sem_bp,sem_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            # This time, a color converter is applied to the image, to get the semantic segmentation view
            sem_cam.listen(lambda image: image.save_to_disk('data/new_sem_output/%.6d.jpg' % image.frame,carla.ColorConverter.CityScapesPalette))
        

        # --------------
        # Add a new radar sensor to ego vehicle
        # --------------
        if enable_radar:
            rad_cam = None
            rad_bp = world.get_blueprint_library().find('sensor.other.radar')
            rad_bp.set_attribute('horizontal_fov', str(35))
            rad_bp.set_attribute('vertical_fov', str(20))
            rad_bp.set_attribute('range', str(20))
            rad_location = carla.Location(x=2.8, z=1.0)
            rad_rotation = carla.Rotation(pitch=5)
            rad_transform = carla.Transform(rad_location,rad_rotation)
            rad_ego = world.spawn_actor(rad_bp,rad_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            def rad_callback(radar_data):
                velocity_range = 7.5 # m/s
                current_rot = radar_data.transform.rotation
                for detect in radar_data:
                    azi = math.degrees(detect.azimuth)
                    alt = math.degrees(detect.altitude)
                    # The 0.25 adjusts a bit the distance so the dots can
                    # be properly seen
                    fw_vec = carla.Vector3D(x=detect.depth - 0.25)
                    carla.Transform(
                        carla.Location(),
                        carla.Rotation(
                            pitch=current_rot.pitch + alt,
                            yaw=current_rot.yaw + azi,
                            roll=current_rot.roll)).transform(fw_vec)

                    def clamp(min_v, max_v, value):
                        return max(min_v, min(value, max_v))

                    norm_velocity = detect.velocity / velocity_range # range [-1, 1]
                    r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
                    g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
                    b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
                    world.debug.draw_point(
                        radar_data.transform.location + fw_vec,
                        size=0.075,
                        life_time=0.06,
                        persistent_lines=False,
                        color=carla.Color(r, g, b))
            rad_ego.listen(lambda radar_data: rad_callback(radar_data))
        
        # --------------
        # Add a new LIDAR sensor to ego vehicle
        # --------------
        if enable_lidar:
            lidar_cam = None
            lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels',str(32))
            lidar_bp.set_attribute('points_per_second',str(500000))
            lidar_bp.set_attribute('rotation_frequency',str(40))
            lidar_bp.set_attribute('range',str(100))
            lidar_location = carla.Location(0,0,2)
            lidar_rotation = carla.Rotation(0,0,0)
            lidar_transform = carla.Transform(lidar_location,lidar_rotation)
            lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle,attachment_type=carla.AttachmentType.Rigid)
            lidar_sen.listen(lambda point_cloud: point_cloud.save_to_disk('data/new_lidar_output/%.6d.ply' % point_cloud.frame))
        

        # --------------
        # Add IMU sensor to ego vehicle. 
        # --------------
        
        if enable_imu:
            imu_bp = world.get_blueprint_library().find('sensor.other.imu')
            imu_location = carla.Location(0,0,0)
            imu_rotation = carla.Rotation(0,0,0)
            imu_transform = carla.Transform(imu_location,imu_rotation)
            imu_bp.set_attribute("sensor_tick",str(0.1))
            ego_imu = world.spawn_actor(imu_bp,imu_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            def imu_callback(imu):
                print("IMU measure:\n"+str(imu)+'\n')
            ego_imu.listen(lambda imu: imu_callback(imu))

        # --------------
        # Add GNSS sensor to ego vehicle. 
        # --------------
        
        if enable_gnss:
            gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')
            gnss_location = carla.Location(0,0,0)
            gnss_rotation = carla.Rotation(0,0,0)
            gnss_transform = carla.Transform(gnss_location,gnss_rotation)
            gnss_bp.set_attribute("sensor_tick",str(0.1))
            ego_gnss = world.spawn_actor(gnss_bp,gnss_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            def gnss_callback(gnss):
                print("GNSS measure:\n"+str(gnss)+'\n')
            ego_gnss.listen(lambda gnss: gnss_callback(gnss))


        # --------------
        # Add collision sensor to ego vehicle. 
        # --------------
        
        if enable_collision:
            col_bp = world.get_blueprint_library().find('sensor.other.collision')
            col_location = carla.Location(0,0,0)
            col_rotation = carla.Rotation(0,0,0)
            col_transform = carla.Transform(col_location,col_rotation)
            ego_col = world.spawn_actor(col_bp,col_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            def col_callback(colli):
                print("Collision detected:\n"+str(colli)+'\n')
            ego_col.listen(lambda colli: col_callback(colli))
        

        # --------------
        # Add Lane invasion sensor to ego vehicle. 
        # --------------
        
        if enable_lane_invasion:
            lane_bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            lane_location = carla.Location(0,0,0)
            lane_rotation = carla.Rotation(0,0,0)
            lane_transform = carla.Transform(lane_location,lane_rotation)
            ego_lane = world.spawn_actor(lane_bp,lane_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            def lane_callback(lane):
                print("Lane invasion detected:\n"+str(lane)+'\n')
            ego_lane.listen(lambda lane: lane_callback(lane))
        

        # --------------
        # Add Obstacle sensor to ego vehicle. 
        # --------------
        
        if enable_obstacle_sensor:
            obs_bp = world.get_blueprint_library().find('sensor.other.obstacle')
            obs_bp.set_attribute("only_dynamics",str(True))
            obs_location = carla.Location(0,0,0)
            obs_rotation = carla.Rotation(0,0,0)
            obs_transform = carla.Transform(obs_location,obs_rotation)
            ego_obs = world.spawn_actor(obs_bp,obs_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            def obs_callback(obs):
                print("Obstacle detected:\n"+str(obs)+'\n')
            ego_obs.listen(lambda obs: obs_callback(obs))

        # --------------
        # Game loop. Prevents the script from finishing.
        # --------------
        # while True:
        #     world_snapshot = world.wait_for_tick()

        while True:
            # Wait for the world tick
            world_snapshot = world.wait_for_tick()
            
            if follow_car and ego_vehicle is not None:
                # Get the ego vehicle's current transform
                ego_transform = ego_vehicle.get_transform()
                
                # Offset the spectator to be behind the car
                forward_vector = ego_transform.get_forward_vector()
                spectator_location = ego_transform.location - forward_vector * 6 + carla.Location(z=2)  # Offset 6 meters back and 2 meters up
                spectator_rotation = carla.Rotation(pitch=-15, yaw=ego_transform.rotation.yaw)  # Adjust the pitch and yaw
                
                # Set the spectator's transform
                spectator.set_transform(carla.Transform(spectator_location, spectator_rotation))



    finally:
        # --------------
        # Destroy actors
        # --------------
        if ego_vehicle is not None:
            if ego_cam is not None:
                ego_cam.stop()
                ego_cam.destroy()
            if depth_cam is not None:
                depth_cam.stop()
                depth_cam.destroy()
            if sem_cam is not None:
                sem_cam.stop()
                sem_cam.destroy()
            if rad_ego is not None:
                rad_ego.stop()
                rad_ego.destroy()
            if lidar_sen is not None:
                lidar_sen.stop()
                lidar_sen.destroy()
            if ego_col is not None:
                ego_col.stop()
                ego_col.destroy()
            if ego_lane is not None:
                ego_lane.stop()
                ego_lane.destroy()
            if ego_obs is not None:
                ego_obs.stop()
                ego_obs.destroy()
            if ego_gnss is not None:
                ego_gnss.stop()
                ego_gnss.destroy()
            if ego_imu is not None:
                ego_imu.stop()
                ego_imu.destroy()
            ego_vehicle.destroy()
        print('\nNothing to be done.')


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with tutorial_replay.')
