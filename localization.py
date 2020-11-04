import glob
import os
import sys

# Next commented section only will be needed if you put your main here
'''
try:
    sys.path.append(glob.glob('/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError: 'Ezbot el EGG file'

sys.path.insert(1,'/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla')
sys.path.insert(1,'/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla/agents/navigation')   
from basic_agent import BasicAgent

import carla
'''

class localizer():
	def __init__(self , actor , world): # actors must be in the form of carla.Actor refer to documantation 
		self.actor = actor
		self.local_position  = actor.get_transform().location # Exact position 
		self.local_rotation  = actor.get_transform().rotation  # refer to this image https://d26ilriwvtzlb.cloudfront.net/8/83/BRMC_9.jpg 
		self.local_transform = actor.get_transform() 
		self.velocity = actor.get_velocity()
		self.acceleration = actor.get_acceleration()
		self.angular_velocity = actor.get_angular_velocity()
		
	
	# refer to this paper 'http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.4H.pdf#page=20'
	def laneId(self):
		return self.waypoint.lane_id # integer
	def rodaId(self):
		return self.waypoint.road_id # integer
	
	# refer to this section 'https://carla.readthedocs.io/en/latest/python_api/#carlalanemarkingtype'
	def right_lane_type(self):
		return self.waypoint.right_lane_marking.type
	def left_lane_type(self):
		return self.waypoint.left_lane_marking.type
	
	# refer to this section 'https://carla.readthedocs.io/en/latest/python_api/#carlalanechange'
	def lane_change(self):
		return self.waypoint.lane_change # it can be (NONE / Both / Left / Right)

	def location(self):
		return self.actor.get_location()

	def transform(self):
		return self.actor.get_transform()

	def waypoint(self):
		location = self.location()
		world = self.actor.get_world()
		m = world.get_map()
		wp = m.get_waypoint(location)
		return wp


def dispLocation(location , label = 'haha'):
	if label is not 'haha':
		print('-------' +label+'-------')
	print('x = ',location.x)
	print('y = ',location.y)
	print('z = ',location.z)
	print('-------------------')

def dispWaypoint(waypoint , label = 'haha'):
	if label is not 'haha':
		print('-------' +label+'-------')
	print('waypoint x = ', waypoint.transform.location.x)
	print('waypoint y = ', waypoint.transform.location.y)
	print('waypoint z = ', waypoint.transform.location.z)
	print('-------------------')
