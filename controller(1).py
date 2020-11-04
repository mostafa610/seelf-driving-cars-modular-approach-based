import numpy as np
import carla
from collections import deque
from localization import location
import math

'''
Input of this Module:
* This Module needs 



'''



class controller():
	def __init__(vehicle , args_lateral , args_long):
		self.args_long = args_long
		self.args_lateral = args_lateral
		self.vehicle = vehicle
		self.long_control = longController(args_long)
		self.lat_control  = latController(args_lateral)

	def run(self , target_speed , target_waypoint , emergency_stop = False):

		loc = location(self.vehicle)
		_3DVelocity = loc.velocity
		current_speed = 3.6 * math.sqrt( (_3DVelocity.x**2) + (_3DVelocity.y**2) + (_3DVelocity.z**2)) # in Km/H		
		current_location = loc.local_transform	

		throttle = self.long_control.run(target_speed , current_speed)
		steering = self.lat_control.run(target_waypoint , current_location)

		# refer to this link 'https://carla.readthedocs.io/en/latest/python_api/#carlavehiclecontrol'
		vehicle_control = carla.VehicleControl()
		vehicle_control.throttle = throttle
		vehicle_control.steering = steering
		if emergency_stop:
			vehicle_control.brake = 1.0 
		vehicle_control.brake = 0.0
		vehicle_control.hand_brake = False
		vehicle_control.reverse = False
		vehicle_control.manual_gear_shift = False

		return vehicle_control
 
class longController():
	def __init__(args):
		self.kp = args['kp']
		self.kd = args['kd']
		self.ki = args['ki']
		self.dt = args['dt']
		self.error = deque(maxlen=30)

	def run(self, target_speed , current_speed):
		e = (target_speed - current_speed)
		self.error.append(e)

		if len(self.error) >= 2: # habda2 a7seb ba3d 2 readings
			de = (self.error[-1] - self.error[-2]) / self.dt # computing differential term over the error 
			ie = sum(self.error) * self.dt 					 # computing integral term over the error
		else:
			de = 0.0
			ie = 0.0

		throttle = (self.kp * e) + (self.kd * de / self.dt) + (self.ki * ie * self.dt)
		throttle = np.clip(throttle , 0.0 , 1.0) # if <0 make it = 0 and if >1 make it = 1
		return throttle

class latController():
	def __init__(args):
		self.kp = args['kp']
		self.kd = args['kd']
		self.ki = args['ki']
		self.dt = args['dt']
		self.error = deque(maxlen=10)

	def run(self, target_waypoint, current_location):
		begin_location = current_location.location
		end_location   = begin_location + carla.Location(x= math.cos(math.radians(current_location.rotation.yaw)),
														 y= math.sin(math.radians(current_location.rotation.yaw)))
		v_vec = np.array([begin_location.x - end_location.x , begin_location.y - end_location.y , 0.0])
		w_vec = np.array([target_waypoint.transform.location.x - begin_location.x,
						  target_waypoint.transform.location.y - begin_location.y,
						  0.0])
		
		norm = np.linalg.norm(w_vec) * np.linalg.norm(v_vec)
		dot = np.dot(w_vec , v_vec) 
		dot = dot / norm
		dot = np.clip(dot , -1.0 , 1.0)

		cross = np.cross(v_vec , w_vec)
		if cross[2] < 0:
			dot *= -1.0

		self.error.append(dot)
		if len(self.error) >= 2: # habda2 a7seb ba3d 2 readings
			de = (self.error[-1] - self.error[-2]) / self.dt # computing differential term over the error 
			ie = sum(self.error) * self.dt 					 # computing integral term over the error
		else:
			de = 0.0
			ie = 0.0

		steering = (self.kp * dot) + (self.kd * de / self.dt) + (self.ki * ie * self.dt)
		steering = np.clip(steering , -1.0 , 1.0)

		return steering



							




