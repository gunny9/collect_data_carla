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

sys.path.append('/home/gun/Desktop/Carla/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg')  #적절한 파일 path 넣어주기


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

def list_maps(client): 
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
'''
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

##############################################################################################차량 생성 및 센서 관련 
def spawn_vehicle_randomly(world, blueprint_library, transform): #랜덤으로 생성
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
    vehicle = world.spawn_actor(vehicle_bp, transform)
    return vehicle

def spawn_vehicle(world, blueprint_library, transform, vehicle_type='vehicle.tesla.model3'): #지정 생성
    vehicle_bp = blueprint_library.find(vehicle_type)
    vehicle = world.spawn_actor(vehicle_bp, transform)
    return vehicle

def setup_camera(vehicle, blueprint_library, world):
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_bp.set_attribute('sensor_tick', '1')  # 곱해서 1이 되는 게 해당 FPS / 10 FPS = 0.1

    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))  # Adjust the position as needed
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    
    return camera

def setup_imu(vehicle, blueprint_library, world):
    imu_bp = blueprint_library.find('sensor.other.imu')
    imu_transform = carla.Transform(carla.Location(x=0, y=0, z=0))  # Adjust the position as needed
    imu = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
    
    return imu

def setup_lidar(vehicle, blueprint_library, world):
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range', '50')
    lidar_bp.set_attribute('rotation_frequency', '10')
    lidar_bp.set_attribute('channels', '32')
    lidar_bp.set_attribute('points_per_second', '100000')

    lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=2.4))  # Adjust the position as needed
    lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
    
    return lidar

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


if __name__ == '__main__':


    # Create output directory if it doesn't exist
    output_dir = '/home/gun/Desktop/carla_output/test'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    camera_dir = os.path.join(output_dir, 'camera')
    imu_dir = os.path.join(output_dir, 'imu')
    lidar_dir = os.path.join(output_dir, 'lidar')
    os.makedirs(camera_dir, exist_ok=True)
    os.makedirs(imu_dir, exist_ok=True)
    os.makedirs(lidar_dir, exist_ok=True)

    # Example usage:
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Set the weather to 'clear'
    #set_weather(client, 'clear')
    # List available maps
    #list_maps(client)

    # Change to a specific map, e.g., 'Town03'
    change_map(client, '/Game/Carla/Maps/Town03')

    world = client.get_world()
    blueprint_library = world.get_blueprint_library()


    # Define transform for the stationary vehicle
    stationary_vehicle_transform = carla.Transform(carla.Location(x=89, y=197, z=2), carla.Rotation(yaw=180))
    stationary_vehicle = spawn_vehicle(world, blueprint_library, stationary_vehicle_transform)
    print(f"Spawned stationary vehicle at {stationary_vehicle_transform.location}")

    # Apply brake to the stationary vehicle
    control = carla.VehicleControl(hand_brake=True)
    stationary_vehicle.apply_control(control)

    # Setup camera, IMU, and LiDAR on stationary vehicle
    camera = setup_camera(stationary_vehicle, blueprint_library, world)
    imu = setup_imu(stationary_vehicle, blueprint_library, world)
    lidar = setup_lidar(stationary_vehicle, blueprint_library, world)


    # Define transform for the approaching vehicle
    approaching_vehicle_transform = carla.Transform(carla.Location(x=50, y=197, z=2), carla.Rotation(yaw=0))
    approaching_vehicle = spawn_vehicle(world, blueprint_library, approaching_vehicle_transform)
    print(f"Spawned approaching vehicle at {approaching_vehicle_transform.location}")

    # Move the approaching vehicle straight towards the stationary vehicle
    approaching_vehicle_control = carla.VehicleControl(throttle=1)  #0~1.0
    approaching_vehicle.apply_control(approaching_vehicle_control)


    # Setup listeners to save sensor data
    camera.listen(lambda image: save_image(image, camera_dir))
    imu.listen(lambda imu_data: save_imu_data(imu_data, imu_dir))
    lidar.listen(lambda lidar_data: save_lidar_data(lidar_data, lidar_dir))


    # Let simulation run for a while to see the vehicle moving
    time.sleep(10)

    # Stop the simulation and destroy vehicles
    stationary_vehicle.destroy()
    approaching_vehicle.destroy()
    camera.destroy()
    imu.destroy()
    lidar.destroy()