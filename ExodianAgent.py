import numpy as np

from agent import Agent
from Planner import LocalPlanner
from Mapping import Mapper
from Topologiest import topologiest
from localization import localizer
from perception import perception, LaneFollow, LaneChange 
from synchronous_mode import CarlaSyncMode



import sys
import glob
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
'''
sys.path.insert(1,'/media/hamamgpu/Drive31/Modular-approach-based-self-driving-car/Carla/PythonAPI/carla') 

sys.path.append(glob.glob('/media/hamamgpu/Drive31/Modular-approach-based-self-driving-car/Carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
'''
#sys.path.insert(1,'/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla') 

#sys.path.append(glob.glob('/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
#        sys.version_info.major,
#        sys.version_info.minor,
#        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
#sys.path.append(glob.glob('/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla/dist/carla-0.9.6-py3.5-linux-x86_64.egg')[0])

sys.path.insert(1,'/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla') 
sys.path.append(glob.glob('/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla/dist/carla-0.9.6-py3.5-linux-x86_64.egg')[0])
import carla



class Exodian(Agent):

	def __init__(self, vehicle, adjust_speed = True):
		
		super(Exodian, self).__init__(vehicle)
		self.vehicle = vehicle
		if not adjust_speed:
			self.target_speed = 20 #Km/H
		else:
			self.target_speed = None


		args_lateral_dict = {
			'K_P': 1,
			'K_D': 0.02,
			'K_I': 0,
			'dt': 1.0/20.0}
		args_longitudinal_dict = {
			'K_P': 1.0,
			'K_D': 0,
			'K_I': 1,
			'dt': 1.0/20.0}

		self.local_planner = LocalPlanner(self.vehicle ,opt_dict={'longitudinal_control_dict':args_longitudinal_dict,'lateral_control_dict':args_lateral_dict})
		self._hop_resolution = 2.0
		self._path_seperation_hop = 2
		self._path_seperation_threshold = 0.5
		self.global_route_plan = None
		self._grp = None
		self.destination = None

		# define modules
		self.localizer = localizer(self.vehicle,self.vehicle.get_world())
		self.perception = perception(self.vehicle)

		self.recording_camera = None


	def set_destination(self , start_location , end_location):

		start_waypoint = self.vehicle.get_world().get_map().get_waypoint(start_location)
		end_waypoint   = self.vehicle.get_world().get_map().get_waypoint(end_location)

		route_trace = self._trace_route(start_waypoint, end_waypoint)
		assert route_trace

		with open('testing_file.txt','w+') as file:
			file.write('start location: ' + str(start_waypoint.transform.location.x) + str(' , ')+ str(start_waypoint.transform.location.y) + str(' , ')+ str(start_waypoint.transform.location.z) + '\n')
			for element in route_trace:
				file.write(str(element[0].transform.location.x) + str(' , ') + str(element[0].transform.location.y) + str(' , ') + str(element[0].transform.location.z) + '\n' )

		self.local_planner.set_global_plan(route_trace)
	

	def initialize_the_model(self):
		self.perception.initialize_model()

	def spawn_sensors(self):
		camera , lidar = self.perception.spawn_sensors()
		return camera,lidar	

	def take_data(self , rgb_data , lidar_data):
		self.perception.save_rgb(rgb_data)
		self.perception.filter_lidar_data(lidar_data)


	def _trace_route(self, start_waypoint, end_waypoint): 
		"""
		This method sets up a global router and returns the optimal route
		from start_waypoint to end_waypoint by using the mapper and the topologiest
		"""

		# Setting up global router
		if self._grp is None:
			topology = topologiest(self._vehicle.get_world().get_map(), self._hop_resolution)
			mapperr = Mapper(topology)
			mapperr.setup()
			self._grp = mapperr

		# Obtain route plan
		route = self._grp.trace_route(
			start_waypoint.transform.location,
			end_waypoint.transform.location)

		return route  
	
	def adjust_speed(self):
		# Under Construction
		return 20
	
	def recording_camera_attachment(self):
		world = self.vehicle.get_world()
		blurprint_library = world.get_blueprint_library()
		camera_bp = blurprint_library.find('sensor.camera.rgb')
		IM_WIDTH = 800
		IM_HIGHT = 800
		camera_bp.set_attribute("image_size_x",str(IM_WIDTH))
		camera_bp.set_attribute("image_size_y",str(IM_HIGHT))
		camera_bp.set_attribute("fov" , "110")
		camera_spawn_point = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
		camera = world.spawn_actor(camera_bp , camera_spawn_point , attach_to = self.vehicle)
		self.recording_camera = camera
		return camera
		
	def record(self):
		self.recording_camera.listen(lambda image: image.save_to_disk('demo/%06d.png' % image.frame))

	def reroute(self,begin):
		set_destination(location = self.destination , start_location = begin)

	def emergency_stop(self):
		control = carla.VehicleControl()
		control.steer = 0.0
		control.throttle = 0.0
		control.brake = 1.0
		return control
		
	def run_step(self,debug=False):
		target_speed = self.adjust_speed()
		control = self.local_planner.run_step(target_speed = self.target_speed,debug=debug)
		'''
		print('**** running perception')
		self.perception.run_step() # This ill update the two variables coming from the perception
		print('**** running scenarios')
		# Scenario 1: Open way 
		if self.perception.LaneFollow is LaneFollow.open_way:
			target_speed = self.adjust_speed()
			control = self.local_planner.run_step(target_speed = self.target_speed)


		# Scenario 2: Conjusted way 
		elif self.perception.LaneFollow is LaneFollow.conjusted_way:
			target_speed = self.adjust_speed()
			control = self.local_planner.run_step(target_speed = self.target_speed)


		# Scenario 3: Blocked way with available Lane Change
		elif (self.perception.LaneFollow is LaneFollow.blocking_way) and (self.perception.LaneChange is not LaneChange.block):
			if self.perception.LaneChange is LaneChange.left:
				current_waypoint = self.localizer.waypoint()
				left_waypoint = current_waypoint.get_left_lane()
				if left_waypoint is None:
					control = self.emergency_stop()
				else:
					control = self.local_planner.perform_lane_change(left_waypoint,target_speed = 10)
					self.reroute(begin=left_waypoint)
			elif self.perception.LaneChange is LaneChange.right:
				current_waypoint = self.localizer.waypoint()
				right_waypoint = current_waypoint.get_right_lane()
				if right_waypoint is None:
					control = self.emergency_stop()
				else:
					control = self.local_planner.perform_lane_change(right_waypoint,target_speed = 10)
					self.reroute(begin=right_waypoint)
                      

		else:
			control = self.emergency_stop()
		'''
		return control



