import glob
import os
import sys
import random
import time

try:
    sys.path.append(glob.glob('/media/hamamgpu/Drive31/Modular-approach-based-self-driving-car/Carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


actor_list = []
try:
	client = carla.Client("localhost",2000)
	client.set_timeout(5.0) #in seconds
	world = client.get_world()	
	blurprint_library = world.get_blueprint_library()


	bp = blurprint_library.filter("model3")[0] #to select a vhiecle
	spawn_point = random.choice(world.get_map().get_spawn_points()) # nazel el vehicle somewhere randomly
	vehicle = world.spawn_actor(bp, spawn_point) # create some vehile of attributes of (bp) and spawn it at the spawn point you've choosen beore
	vehicle.set_autopilot(True) # sha3'al el auto pilot for this vehicle
	actor_list.append(vehicle)  # 7ot el vehicle fe el list of actors beta3tak
	'''
	to be able to control the vehicle use the following command which you will find it in details in the documentation 
	vehicle.apply_control(carla.VhiecleControl(throttle=1.0, steer=0.0)) # only forward
	'''
	

	camera_bp = blurprint_library.find('sensor.camera.rgb')
	camera_bp.set_attribute("fov" , "110") # set FOV to 110
	camera_spawn_point = carla.Transform(carla.Location(x=2.5,z=0.7)) # Da el location beta3 el camera 3ala el vehicle nafsaha 	
	camera = world.spawn_actor(camera_bp , camera_spawn_point , attach_to = vehicle)
	actor_list.append(camera)  # 7ot el camera fe el list of actors beta3tak
	'''
	To specify the hight and width of the taken image by this camera you may use these commands:
	IM_WIDTH = 480
	IM_WIDTH = 640
	camera_bp.set_attribute("image_size_x",f"{IM_WIDTH}")
	camera_bp.set_attribute("image_size_y",f"{IM_HIGHT}")
	'''
	
	# Now let's start listening to this camera
	camera.listen(lambda image: image.save_to_disk('%06d.png' % image.frame))

	time.sleep(20)


finally:
	for actor in actor_list:
		actor.destroy()
	print("3'yartaha aho")




















