import numpy as np
import sys
import glob
import os
import random
import pygame
import gc

sys.path.insert(1,'/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla') 
sys.path.append(glob.glob('/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla/dist/carla-0.9.6-py3.5-linux-x86_64.egg')[0])

import carla
from ExodianAgent import Exodian
from synchronous_mode import CarlaSyncMode


def spawn_player(world):
	blueprint_library = world.get_blueprint_library()
	bp = blueprint_library.filter("model3")[0] #to select a vhiecle
	spawn_point = random.choice(world.get_map().get_spawn_points())
	vehicle = world.spawn_actor(bp, spawn_point) # create some vehile of attributes of (bp) and spawn it at the spawn point you've choosen beore
	spawn_point_transform = spawn_point
	return vehicle,spawn_point_transform

def show_rgb_image(display,image):
	out = np.frombuffer(image.raw_data , dtype=np.dtype('uint8'))
	out = out.reshape((800,800,4))
	out = out[:,:,:3]	
	surface = pygame.surfarray.make_surface(out.swapaxes(0,1))
	display.blit(surface,(0,0))
	pygame.display.flip()

def game_loop():
	window_width = 800
	window_hight = 800

	pygame.init()
	display = pygame.display.set_mode( (window_width, window_hight), pygame.HWSURFACE | pygame.DOUBLEBUF)
	clock = pygame.time.Clock()

	print('connecting to server ...')
	client = carla.Client("localhost", 2000)
	client.set_timeout(10.0)
	world = client.get_world()

	print('connection has been established successfully')
	print('Starting the Synchronous mode')
	settings = world.get_settings()
	settings.synchronous_mode = True
	settings.fixed_delta_seconds = 0.05	
	world.apply_settings(settings)

	print('Choosing Hero')
	player,start_point_transform = spawn_player(world)	
	start_location = start_point_transform.location

	print('Starting the Exodian')
	exodian = Exodian(player , adjust_speed=False)
	camera,lidar = exodian.spawn_sensors()	
	exodian.initialize_the_model()
	
	print('Choosing start point and destination point ...')
	dest_transform = random.choice(world.get_map().get_spawn_points())
	dest_location  = dest_transform.location
	while start_location.distance(dest_location) <= 10: #meters
		dest_transform = random.choice(world.get_map().get_spawn_points())
		dest_location  = dest_transform.location
	dest_location  = dest_transform.location	

	exodian.set_destination(start_location=start_location , end_location=dest_location)
	recording_camera = exodian.recording_camera_attachment()


	try:

		with CarlaSyncMode(world , camera , lidar , recording_camera ,fps = 20) as sync:

			counter = 1
			print('STARTING GAME LOOP')
			while True:

				if player.get_location() is dest_location:
					print('Reached Destination Successfully !!')
					break

				the_wait_signal = 1
				while the_wait_signal < 5:
					sync.tick_no_data
					clock.tick()
					the_wait_signal += 1

				print('***** Ticking '+str(counter))
				_,rgb_data,lidar_data,recording_data = sync.tick(timeout = 2.0)
				
				exodian.take_data(rgb_data , lidar_data)
				control = exodian.run_step(debug=True)
				print('destination waypoint: ',dest_location)
				print('distance to the destination point: ',player.get_location().distance(dest_location))

				show_rgb_image(display,recording_data)

				player.apply_control(control)

				counter += 1
				print(' ')


	finally:
		print('Terminating the Synchronous mode safely')
		settings = world.get_settings()
		settings.synchronous_mode = False	
		world.apply_settings(settings)

		print('Removing Sensors')
		camera.destroy()
		lidar.destroy()

		print('Killing Hero')
		player.destroy()		







if __name__ == '__main__':

    try:

        game_loop()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

