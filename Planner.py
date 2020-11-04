from enum import Enum
from collections import deque
import random
import carla

from misc import distance_vehicle, draw_waypoints 
from localization import localizer
from Controller import controller


'''
    Things to Do:
        1. Ezbot el Target Speed
        2. File el misc.py shofo
        3. FPS
'''
FPS = 20

class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations when moving from a segment of lane to other.
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6
    

class LocalPlanner(object):


    def __init__(self, vehicle, opt_dict=None):

        self._vehicle = vehicle
        self._map = self._vehicle.get_world().get_map()
        self._world = self._vehicle.get_world()
        self._dt = None
        self._target_speed = None
        self._min_distance = 3 # by trial and error
        self._current_waypoint = None
        self._target_road_option = None
        self.target_waypoint = None
        self._global_plan = None
        # queue with tuples of (waypoint, RoadOption)
        self._waypoints_queue = deque(maxlen=20000)
        self._buffer_size = 5
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        self._sampling_radius=None

        # define the localizer
        self.localizer = localizer(actor= self._vehicle , world=self._world)

        # define the controller
        self.controller = None
        self._init_controller(opt_dict)

    def __del__(self):
        if self._vehicle:
            self._vehicle.destroy()
        print("Destroying Exodian")

    def reset_vehicle(self):
        self._vehicle = None
        print("Resetting Exodian")

    def _init_controller(self, opt_dict):
        """
        Controller initialization.

        :param opt_dict: dictionary of arguments.
        :return:
        """
        # default params
        self._dt = 1.0 / FPS
        self._target_speed = self._vehicle.get_speed_limit() # Km/h <----------- Needs modification
        args_lateral_dict = {
            'K_P': 1.95,
            'K_D': 0.01,
            'K_I': 1.4,
            'dt': self._dt}
        args_longitudinal_dict = {
            'K_P': 1.0,
            'K_D': 0,
            'K_I': 1,
            'dt': self._dt}

        # parameters overload
        ''' 
        if opt_dict:
            if 'dt' in opt_dict:
                self._dt = opt_dict['dt']
            if 'target_speed' in opt_dict:
                self._target_speed = opt_dict['target_speed']
            if 'sampling_radius' in opt_dict:
                self._sampling_radius = self._target_speed * \
                    opt_dict['sampling_radius'] / 3.6
            if 'lateral_control_dict' in opt_dict:
                args_lateral_dict = opt_dict['lateral_control_dict']
            if 'longitudinal_control_dict' in opt_dict:
                args_longitudinal_dict = opt_dict['longitudinal_control_dict']'''

        self._current_waypoint = self.localizer.waypoint()
        self.controller = controller(self._vehicle,args_lateral_dict,args_longitudinal_dict)


    def set_speed(self, speed):
        self._target_speed = speed

    def choose_next_waypoints(self, step=3):
        if len(self._waypoints_queue) > steps:
            return self._waypoints_queue[steps]
        else:
            try:
                waypoint , MyRoadOption = self._waypoints_queue[-1]
                return waypoint , MyRoadOption
            except IndexError as i:
                print(i)
                return None, RoadOption.VOID
        return None, RoadOption.VOID 


    def set_global_plan(self, current_plan):
        #self._waypoints_queue.clear()
        self._waypoints_queue = None
        self._waypoints_queue = deque(maxlen=20000)
        for elem in current_plan:
            self._waypoints_queue.append(elem)
        self._target_road_option = RoadOption.LANEFOLLOW
        self._global_plan = True

    def run_step(self, debug=True ,target_speed = None ):
        """
        Execute one step of local planning which involves running the longitudinal and lateral PID controllers to
        follow the waypoints trajectory.

        :param debug: boolean flag to activate waypoints debugging
        :return:
        """

        if target_speed is not None:
            self._target_speed = target_speed
        else:
            self._target_speed = self._vehicle.get_speed_limit()


        if len(self._waypoints_queue) == 0: # wasalt 5alas
            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
            control.manual_gear_shift = False

            return control

        #   Buffering the waypoints
        if not self._waypoint_buffer: # lw el buffer fady!! 
            for i in range(self._buffer_size): 
                if self._waypoints_queue:
                    self._waypoint_buffer.append(
                        self._waypoints_queue.popleft())
                else:
                    break

        # current vehicle waypoint
        self._current_waypoint = self.localizer.waypoint()
        # target waypoint
        self.target_waypoint, self._target_road_option = self._waypoint_buffer[0]
        # move using PID controllers
        control = self.controller.run_step(self._target_speed, self.target_waypoint)

        # purge the queue of obsolete waypoints
        vehicle_transform = self._vehicle.get_transform()
        max_index = -1

        for i, (waypoint, _) in enumerate(self._waypoint_buffer):
            if distance_vehicle(
                    waypoint, vehicle_transform) < self._min_distance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                self._waypoint_buffer.popleft()

        if debug:
            print('current waypoint is at: ',self._vehicle.get_location())
            print('Next waypoint is at: ',self.target_waypoint.transform.location)
            print('distance to the next wp: ', self._vehicle.get_location().distance(self.target_waypoint.transform.location))
            print('number of waypoints in the queue:', len(self._waypoints_queue))

        return control
