import numpy as np
import sys
import glob
import os
import random
import pygame
import gc

sys.path.insert(1,'/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla') 
sys.path.append(glob.glob('/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla/dist/carla-0.9.6-py3.5-linux-x86_64.egg')[0])

'''sys.path.insert(1,'/media/hamamgpu/Drive31/Modular-approach-based-self-driving-car/Carla/PythonAPI/carla') 

sys.path.append(glob.glob('/media/hamamgpu/Drive31/Modular-approach-based-self-driving-car/Carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
'''
import carla

from ExodianAgent import Exodian
from synchronous_mode import CarlaSyncMode

actor_list = []

def spawn_player(world):
	blueprint_library = world.get_blueprint_library()
	bp = blueprint_library.filter("model3")[0] #to select a vhiecle
	spawn_point = random.choice(world.get_map().get_spawn_points())
	vehicle = world.spawn_actor(bp, spawn_point) # create some vehile of attributes of (bp) and spawn it at the spawn point you've choosen beore
	return vehicle,spawn_point

def spawn_recording_camera(vehicle):
	world = vehicle.get_world()
	blueprint_library = world.get_blueprint_library()
	camera_bp = blueprint_library.find('sensor.camera.rgb')
	camera_bp.set_attribute("fov" , "110") # set FOV to 110
	camera_bp.set_attribute('image_size_x', '608')
	camera_bp.set_attribute('image_size_y', '608')
	camera_spawn_point = carla.Transform(carla.Location(x=-5.5,z=2.8) , carla.Rotation(pitch=-15))
	camera = world.spawn_actor(camera_bp , camera_spawn_point , attach_to = vehicle)			
	return camera

def make_image_rgb(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return array

def game_loop():
	pygame.init()
	display = pygame.display.set_mode((800, 800),pygame.HWSURFACE | pygame.DOUBLEBUF)
	clock = pygame.time.Clock()

	client = carla.Client('localhost',2000)
	client.set_timeout(10.0)
	world = client.get_world()

	print('connection has been established successfully')
	print('Starting the Synchronous mode')
	settings = world.get_settings()
	settings.synchronous_mode = True
	settings.fixed_delta_seconds = 0.05
	world.apply_settings(settings)

	player,start_point = spawn_player(world)
	actor_list.append(player)

	exodian = Exodian(vehicle=player)
	camera , lidar   = exodian.spawn_sensors()
	recording_camera = spawn_recording_camera(player)  
	actor_list.append(camera)
	actor_list.append(lidar)

	exodian.initialize_the_model()

	start_location = start_point.location
	end_waypoint   = random.choice(world.get_map().get_spawn_points())
	end_location   = end_waypoint.location
	while start_location.distance(end_location) <= 10:
		end_waypoint   = random.choice(world.get_map().get_spawn_points())
		end_location   = end_waypoint.location

	exodian.set_destination(start_location , end_location)

	try:
		counter = 1
		with CarlaSyncMode(world, camera,lidar,recording_camera,fps=20) as sync_mode:
			while True:
				if player.get_location() is end_location:
					return

				sync_mode.tick_no_data()
				clock.tick()
				_, rgb_data, lidar_data,recording_data = sync_mode.tick(timeout=2.0)
				print("Tick {}".format(counter))

				recording_image = make_image_rgb(recording_data)
				exodian.take_data(rgb_data,lidar_data)

				control = exodian.run_step()
				player.apply_control(control)

				counter += 1
				image_surface = pygame.surfarray.make_surface(recording_image.swapaxes(0, 1))
				display.blit(image_surface, (0, 0))
				pygame.display.flip()

	finally:
		print("distroying actors...")
		for actor in actor_list:
			actor.destroy()


		print('Closing Carla Safely')
		settings = world.get_settings()
		settings.synchronous_mode = False 
		world.apply_settings(settings)		



if __name__ == '__main__':
    try:
        game_loop()

    finally:
    	print("haha")


