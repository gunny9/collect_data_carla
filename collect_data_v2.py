#!/usr/bin/env python
from __future__ import print_function
import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

###################################### 적절한 파일 path 넣어주기 ######################################
sys.path.append('/home/gun/Desktop/Carla/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg')  
###################################################################################################

import carla
from carla import ColorConverter as cc
import random
import time
import cv2

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')
try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


##############################################################################################세팅 관련
def make_folder_tree(output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    camera_dir_m = os.path.join(output_dir, 'camera_m')
    #imu_dir_m = os.path.join(output_dir, 'imu_m')
    #lidar_dir_m = os.path.join(output_dir, 'lidar_m')
    gnss_dir_m = os.path.join(output_dir, 'gnss_m')

    os.makedirs(camera_dir_m, exist_ok=True)
    #os.makedirs(imu_dir_m, exist_ok=True)
    #os.makedirs(lidar_dir_m, exist_ok=True)
    os.makedirs(gnss_dir_m, exist_ok=True)

    camera_dir_s = os.path.join(output_dir, 'camera_s')
    #imu_dir_s = os.path.join(output_dir, 'imu_s')
    #lidar_dir_s = os.path.join(output_dir, 'lidar_s')
    gnss_dir_s = os.path.join(output_dir, 'gnss_s')

    os.makedirs(camera_dir_s, exist_ok=True)
    #os.makedirs(imu_dir_s, exist_ok=True)
    #os.makedirs(lidar_dir_s, exist_ok=True)
    os.makedirs(gnss_dir_s, exist_ok=True)

    camera_dir_tmp1 = os.path.join(output_dir, 'camera_tmp1')
    #imu_dir_tmp1 = os.path.join(output_dir, 'imu_tmp1')
    #lidar_dir_tmp1 = os.path.join(output_dir, 'lidar_tmp1')
    gnss_dir_tmp1 = os.path.join(output_dir, 'gnss_tmp1')

    os.makedirs(camera_dir_tmp1, exist_ok=True)
    #os.makedirs(imu_dir_tmp1, exist_ok=True)
    #os.makedirs(lidar_dir_tmp1, exist_ok=True)
    os.makedirs(gnss_dir_tmp1, exist_ok=True)

    camera_dir_tmp2 = os.path.join(output_dir, 'camera_tmp2')
    #imu_dir_tmp2 = os.path.join(output_dir, 'imu_tmp2')
    #lidar_dir_tmp2 = os.path.join(output_dir, 'lidar_tmp2')
    gnss_dir_tmp2 = os.path.join(output_dir, 'gnss_tmp2')

    os.makedirs(camera_dir_tmp2, exist_ok=True)
    #os.makedirs(imu_dir_tmp2, exist_ok=True)
    #os.makedirs(lidar_dir_tmp2, exist_ok=True)
    os.makedirs(gnss_dir_tmp2, exist_ok=True)



    return camera_dir_s, gnss_dir_s, camera_dir_m, gnss_dir_m, camera_dir_tmp1, gnss_dir_tmp1, camera_dir_tmp2, gnss_dir_tmp2
    #return camera_dir_s, imu_dir_s, lidar_dir_s, gnss_dir_s, camera_dir_m, imu_dir_m, lidar_dir_m, gnss_dir_m

def set_weather(client, weather): # 날씨 설정
    world = client.get_world()
    weather_presets = {
        'clear': carla.WeatherParameters.ClearNoon,
        'cloudy': carla.WeatherParameters.CloudyNoon,
        'wet': carla.WeatherParameters.WetNoon,
        'wet_cloudy': carla.WeatherParameters.WetCloudyNoon,
        'mid_rain': carla.WeatherParameters.MidRainyNoon,
        'hard_rain': carla.WeatherParameters.HardRainNoon,
        'soft_rain': carla.WeatherParameters.SoftRainNoon,
        'clear_sunset': carla.WeatherParameters.ClearSunset,
        'cloudy_sunset': carla.WeatherParameters.CloudySunset,
        'wet_sunset': carla.WeatherParameters.WetSunset,
        'wet_cloudy_sunset': carla.WeatherParameters.WetCloudySunset,
        'mid_rain_sunset': carla.WeatherParameters.MidRainSunset,
        'hard_rain_sunset': carla.WeatherParameters.HardRainSunset,
        'soft_rain_sunset': carla.WeatherParameters.SoftRainSunset
    }
    if weather in weather_presets:
        world.set_weather(weather_presets[weather])
        print(f"Weather set to {weather}")
    else:
        print("Weather preset not found. Available presets:")
        for preset in weather_presets.keys():
            print(preset)

def list_maps(client): # 가능한 맵 목록 확인
    available_maps = client.get_available_maps()
    print("Available maps:")
    for map_name in available_maps:
        print(map_name)

def change_map(client, map_name): # 맵 바꾸기
    available_maps = client.get_available_maps()
    if map_name in available_maps:
        client.load_world(map_name)
        print(f"Map changed to {map_name}")
    else:
        print(f"Map '{map_name}' not found. Available maps are:")
        for available_map in available_maps:
            print(available_map)

##############################################################################################차량 생성 및 센서 관련 
def spawn_vehicle_randomly(world, blueprint_library, transform): #랜덤으로 생성
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
    vehicle = world.spawn_actor(vehicle_bp, transform)
    return vehicle

def spawn_vehicle(world, blueprint_library, transform, vehicle_type='vehicle.tesla.model3'):
    vehicle_bp = blueprint_library.find(vehicle_type)
    vehicle = world.spawn_actor(vehicle_bp, transform)
    return vehicle

def setup_camera(vehicle, blueprint_library, world, fps):
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_bp.set_attribute('sensor_tick', str(1/fps)) 
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))  # Adjust the position as needed
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    return camera

def setup_imu(vehicle, blueprint_library, world, fps):
    imu_bp = blueprint_library.find('sensor.other.imu')
    imu_bp.set_attribute('sensor_tick', str(1/fps))  
    imu_transform = carla.Transform(carla.Location(x=0, y=0, z=0))  # Adjust the position as needed
    imu = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
    return imu

def setup_lidar(vehicle, blueprint_library, world, fps):
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('sensor_tick', str(1/fps))  
    lidar_bp.set_attribute('range', '50')
    lidar_bp.set_attribute('rotation_frequency', '10')
    lidar_bp.set_attribute('channels', '32')
    lidar_bp.set_attribute('points_per_second', '100000')
    lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=2.4))  # Adjust the position as needed
    lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
    return lidar

def setup_gnss(vehicle, blueprint_library, world, fps):
    gnss_bp = blueprint_library.find('sensor.other.gnss')
    gnss_bp.set_attribute('sensor_tick', str(1/fps))  
    gnss_transform = carla.Transform(carla.Location(x=0, y=0, z=2.4))  # Adjust the position as needed
    gnss = world.spawn_actor(gnss_bp, gnss_transform, attach_to=vehicle)
    return gnss

##############################################################################################데이터 취득 관련
def save_image(image, filename):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]  # Remove alpha channel
    filename = os.path.join(filename, f'{image.frame}.png')
    cv2.imwrite(filename, array)

def save_imu_data(imu_data, output_dir):
    filename = os.path.join(output_dir, f'imu_data_{imu_data.frame}.txt')
    with open(filename, 'w') as f:
        f.write(f"Accelerometer: {imu_data.accelerometer}\n")
        f.write(f"Gyroscope: {imu_data.gyroscope}\n")
        f.write(f"Compass: {imu_data.compass}\n")

def save_lidar_data(lidar_data, output_dir):
    filename = os.path.join(output_dir, f'lidar_data_{lidar_data.frame}.ply')
    points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape(-1, 4)
    with open(filename, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for point in points:
            f.write(f"{point[0]} {point[1]} {point[2]}\n")

def save_gnss_data(gnss_data, output_dir):
    filename = os.path.join(output_dir, f'gnss_data_{gnss_data.frame}.txt')
    with open(filename, 'w') as f:
        f.write(f"Latitude: {gnss_data.latitude}\n")
        f.write(f"Longitude: {gnss_data.longitude}\n")
        f.write(f"Altitude: {gnss_data.altitude}\n")

if __name__ == '__main__':

    ############################################## 환경 설정 ##############################################
    # 시나리오 선택
    '''
    시나리오 1: 차량에 다른 차량이 접근해 오는 시나리오
    시나리오 2: 차량에서 다른 차량이 멀어지는 시나리오 
    '''
    scenario=2 #1 or 2
    seed= "1"
    output_dir = "/home/gun/Desktop/collect_data_carla"  # collect_data_carla clone 한 dir -> data는 clone한 폴더 안에 생성됨
    fps=10 # 센서 fps
    ############################################## 환경 설정 ##############################################

    # Create output directory if it doesn't exist
    output_dir = output_dir +'/data/scenario' + str(scenario) + "_" + seed 
    #camera_dir_s, imu_dir_s, lidar_dir_s, gnss_dir_s, camera_dir_m, imu_dir_m, lidar_dir_m, gnss_dir_m = make_folder_tree(output_dir)
    camera_dir_s, gnss_dir_s, camera_dir_m, gnss_dir_m, camera_dir_tmp1, gnss_dir_tmp1, camera_dir_tmp2, gnss_dir_tmp2 = make_folder_tree(output_dir)

    # Example usage:
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Set the weather to 'clear' (필요하면 아랫줄 주석 풀어서 사용)
    # set_weather(client, 'clear') 

    # Change to a specific map, e.g., 'Town05'
    '''
    가능한 map 목록
    /Game/Carla/Maps/Town04_Opt
    /Game/Carla/Maps/Town05
    /Game/Carla/Maps/Town10HD
    /Game/Carla/Maps/Town02
    /Game/Carla/Maps/Town02_Opt
    /Game/Carla/Maps/Town03_Opt
    /Game/Carla/Maps/Town04
    /Game/Carla/Maps/Town01_Opt
    /Game/Carla/Maps/Town03
    /Game/Carla/Maps/Town01
    /Game/Carla/Maps/Town10HD_Opt
    /Game/Carla/Maps/Town05_Opt
    '''
    change_map(client, '/Game/Carla/Maps/Town05')

    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    if scenario == 1:
        stationary_x = 13
        stationary_y = 2.5
        stationary_z = 0.1
        stationary_yaw = 0
        moving_x = 44
        moving_y = -1.5
        moving_z = 0.1
        moving_yaw = 180

        tmp1_x = 13
        tmp1_y = 5.5
        tmp1_z = 0.1
        tmp1_yaw = 0

        tmp2_x = 5
        tmp2_y = 2.5
        tmp2_z = 0.1
        tmp2_yaw = 0

        
    else:
        stationary_x = 13
        stationary_y = 2.5
        stationary_z = 0.1
        stationary_yaw = 0
        moving_x = 13
        moving_y = 5.5
        moving_z = 0.1
        moving_yaw = 0

        tmp1_x = 44
        tmp1_y = -1.5
        tmp1_z = 0.1
        tmp1_yaw = 180

        tmp2_x = 5
        tmp2_y = 2.5
        tmp2_z = 0.1
        tmp2_yaw = 0

    # Define transform for the stationary vehicle
    stationary_vehicle_transform = carla.Transform(carla.Location(x=stationary_x, y=stationary_y, z=stationary_z), carla.Rotation(yaw=stationary_yaw))   
    stationary_vehicle = spawn_vehicle(world, blueprint_library, stationary_vehicle_transform)
    print(f"Spawned stationary vehicle at {stationary_vehicle_transform.location}")

    # Apply brake to the stationary vehicle
    control = carla.VehicleControl(hand_brake=True)
    stationary_vehicle.apply_control(control)

    # Setup camera, IMU, and LiDAR on stationary vehicle
    camera_s = setup_camera(stationary_vehicle, blueprint_library, world, fps)
    #imu_s = setup_imu(stationary_vehicle, blueprint_library, world, fps)
    #lidar_s = setup_lidar(stationary_vehicle, blueprint_library, world, fps)
    gnss_s = setup_gnss(stationary_vehicle, blueprint_library, world, fps)


    # Define transform for the tmp1 vehicle
    tmp1_transform = carla.Transform(carla.Location(x=tmp1_x, y=tmp1_y, z=tmp1_z), carla.Rotation(yaw=tmp1_yaw))   
    tmp1_vehicle = spawn_vehicle(world, blueprint_library, tmp1_transform, vehicle_type='vehicle.audi.a2')
    print(f"Spawned stationary vehicle at {tmp1_transform.location}")

    # Apply brake to the stationary vehicle
    control = carla.VehicleControl(hand_brake=True)
    tmp1_vehicle.apply_control(control)

    # Setup camera, IMU, and LiDAR on stationary vehicle
    camera_tmp1 = setup_camera(tmp1_vehicle, blueprint_library, world, fps)
    #imu_tmp1 = setup_imu(tmp1_vehicle, blueprint_library, world, fps)
    #lidar_tmp1 = setup_lidar(tmp1_vehicle, blueprint_library, world, fps)
    gnss_tmp1 = setup_gnss(tmp1_vehicle, blueprint_library, world, fps)


    # Define transform for the tmp2 vehicle
    tmp2_transform = carla.Transform(carla.Location(x=tmp2_x, y=tmp2_y, z=tmp2_z), carla.Rotation(yaw=tmp2_yaw))   
    tmp2_vehicle = spawn_vehicle(world, blueprint_library, tmp2_transform, vehicle_type='vehicle.ford.crown')
    print(f"Spawned stationary vehicle at {tmp2_transform.location}")

    # Apply brake to the stationary vehicle
    control = carla.VehicleControl(hand_brake=True)
    tmp2_vehicle.apply_control(control)

    # Setup camera, IMU, and LiDAR on stationary vehicle
    camera_tmp2 = setup_camera(tmp2_vehicle, blueprint_library, world, fps)
    #imu_tmp2 = setup_imu(tmp2_vehicle, blueprint_library, world, fps)
    #lidar_tmp2 = setup_lidar(tmp2_vehicle, blueprint_library, world, fps)
    gnss_tmp2 = setup_gnss(tmp2_vehicle, blueprint_library, world, fps)


    # Define transform for the moving vehicle
    moving_vehicle_transform = carla.Transform(carla.Location(x=moving_x, y=moving_y, z=moving_z), carla.Rotation(yaw=moving_yaw))  
    moving_vehicle = spawn_vehicle(world, blueprint_library, moving_vehicle_transform, vehicle_type='vehicle.dodge.charger_police')
    print(f"Spawned moving vehicle at {moving_vehicle_transform.location}")

    # Setup camera, IMU, and LiDAR on stationary vehicle
    camera_m = setup_camera(moving_vehicle, blueprint_library, world, fps)
    #imu_m = setup_imu(moving_vehicle, blueprint_library, world, fps)
    #lidar_m = setup_lidar(moving_vehicle, blueprint_library, world, fps)
    gnss_m = setup_gnss(moving_vehicle, blueprint_library, world, fps)

    # Apply acceleration to the moving vehicle
    moving_vehicle_control = carla.VehicleControl(throttle=0.5)  # 0 ~ 1.0
    moving_vehicle.apply_control(moving_vehicle_control)




    # Setup listeners to save sensor data             
    camera_s.listen(lambda image: save_image(image, camera_dir_s))
    #imu_s.listen(lambda imu_data: save_imu_data(imu_data, imu_dir_s))
    #lidar_s.listen(lambda lidar_data: save_lidar_data(lidar_data, lidar_dir_s))
    gnss_s.listen(lambda gnss_data: save_gnss_data(gnss_data, gnss_dir_s))


    # Setup listeners to save sensor data             
    camera_tmp1.listen(lambda image: save_image(image, camera_dir_tmp1))
    #imu_tmp1.listen(lambda imu_data: save_imu_data(imu_data, imu_dir_tmp1))
    #lidar_tmp1.listen(lambda lidar_data: save_lidar_data(lidar_data, lidar_dir_tmp1))
    gnss_tmp1.listen(lambda gnss_data: save_gnss_data(gnss_data, gnss_dir_tmp1))

    # Setup listeners to save sensor data             
    camera_tmp2.listen(lambda image: save_image(image, camera_dir_tmp2))
    #imu_tmp2.listen(lambda imu_data: save_imu_data(imu_data, imu_dir_tmp2))
    #lidar_tmp2.listen(lambda lidar_data: save_lidar_data(lidar_data, lidar_dir_tmp2))
    gnss_tmp2.listen(lambda gnss_data: save_gnss_data(gnss_data, gnss_dir_tmp2))





     # Setup listeners to save sensor data              
    camera_m.listen(lambda image: save_image(image, camera_dir_m))
    #imu_m.listen(lambda imu_data: save_imu_data(imu_data, imu_dir_m))
    #lidar_m.listen(lambda lidar_data: save_lidar_data(lidar_data, lidar_dir_m))
    gnss_m.listen(lambda gnss_data: save_gnss_data(gnss_data, gnss_dir_m))

    # Let simulation run for a while to see the vehicle moving
    time.sleep(10)

    # Stop the simulation and destroy vehicles
    stationary_vehicle.destroy()
    tmp1_vehicle.destroy()
    tmp2_vehicle.destroy()
    moving_vehicle.destroy()
    camera_s.destroy()
    #imu_s.destroy()
    #lidar_s.destroy()
    gnss_s.destroy()

    camera_tmp1.destroy()
    #imu_tmp1.destroy()
    #lidar_tmp1.destroy()
    gnss_tmp1.destroy()

    camera_tmp2.destroy()
    #imu_tmp2.destroy()
    #lidar_tmp2.destroy()
    gnss_tmp2.destroy()


    camera_m.destroy()
    #imu_m.destroy()
    #lidar_m.destroy()
    gnss_m.destroy()