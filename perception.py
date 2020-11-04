import numpy as np
from enum import Enum
from ai import cs
import math
import sys
import glob
import os
import tensorflow.compat.v1.keras.backend as k
from synchronous_mode import CarlaSyncMode
from localization import localizer


'''sys.path.append('/media/hamamgpu/Drive31/Modular-approach-based-self-driving-car/Modules/YOLOv2-keras-master_2')
import YOLO_v2,utils,config

sys.path.insert(1,'/media/hamamgpu/Drive31/Modular-approach-based-self-driving-car/Carla/PythonAPI/carla') 

sys.path.append(glob.glob('/media/hamamgpu/Drive31/Modular-approach-based-self-driving-car/Carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
'''
sys.path.insert(1,'/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla') 
sys.path.append(glob.glob('/home/deio/Desktop/CARLA_0.9.6/PythonAPI/carla/dist/carla-0.9.6-py3.5-linux-x86_64.egg')[0])
sys.path.append('/home/deio/Desktop/Final Modules/YOLOv2-keras-master_2')
import carla

from YOLO_v2 import *
from utils import *
from config import anchors,class_names

class LaneFollow(Enum):
	open_way = 1
	conjusted_way = 2
	blocking_way = 3

class LaneChange(Enum):
	left = 1
	right = 2
	block = 3
	both = 4


class perception():
	def __init__(self , vehicle):
		self.vehicle = vehicle
		self.camera = None
		self.lidar  = None

		self.current_rgb = None 
		self.current_pc  = None

		self.LaneFollow = None
		self.LaneChange = None

		self.current_scores=None
		self.current_boxes=None
		self.current_classes=None
		self.actor_list = []

		self.image_width=1920
		self.image_height=640
		self.current_distances=None
		self.angles = None

		self.currentDistance=0
		self.prevDistance=0
		self.saftyFactor = 5
		self.speed = 0		
		self.unit_time = 1

		self.model = None
		self.anchors = None
		self.class_names = None
		self.synch = None
		self.run_first_time = True

		self.localizer = localizer(vehicle,vehicle.get_world())

		self.scores = None
		self.boxes = None
		self.classes = None		


	def destroy_actors(self):
		for actor in self.actor_list:
        	        actor.destroy()
	
	def spawn_sensors(self):
		world = self.vehicle.get_world()
		blueprint_library = world.get_blueprint_library()

		camera_bp = blueprint_library.find('sensor.camera.rgb')
		camera_bp.set_attribute("fov" , "110") # set FOV to 110
		camera_bp.set_attribute('image_size_x', '1920')
		camera_bp.set_attribute('image_size_y', '640')
		camera_spawn_point = carla.Transform(carla.Location(x=2.5,z=0.7)) # Da el location beta3 el camera 3ala el vehicle nafsaha 	
		self.camera = world.spawn_actor(camera_bp , camera_spawn_point , attach_to = self.vehicle)
		self.actor_list.append(self.camera)  # 7ot el camera fe el list of actors beta3tak
		
		lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
		lidar_bp.set_attribute('range', '1000')
		lidar_bp.set_attribute('channels', '64')
		lidar_bp.set_attribute('lower_fov', '-50')
		lidar_bp.set_attribute('points_per_second', '100000')
		lidar_bp.set_attribute('rotation_frequency', '100')
		lidar_spawn_point = carla.Transform(carla.Location(x=2.5,z=0.7))
		self.lidar = world.spawn_actor(lidar_bp , lidar_spawn_point , attach_to = self.vehicle)
		self.actor_list.append(self.lidar)
		return self.camera , self.lidar

		
	def filter_lidar_data(self , LidarMeasurement):
		xyz_points = np.frombuffer(LidarMeasurement.raw_data,dtype=np.dtype('f4'))
		xyz_points = np.array(xyz_points)
		xyz_points = xyz_points.reshape((int(len(xyz_points)/3.0) , 3))

		l = len(xyz_points)
		sph_points = np.empty((l,3), dtype='float64')


        #----------------------- Go To Sph ----------------------
		for point in range(l):
        	        x = xyz_points[point][0]
        	        y = xyz_points[point][1]
        	        z = xyz_points[point][2]
        	        
        	        r, phai, theta = cs.cart2sp(x, y , z)
        	        phai = math.degrees(phai)
        	        theta = math.degrees(theta)	
        	        sph_points[point][0] = r
        	        sph_points[point][1] = theta
        	        sph_points[point][2] = phai
      	  
        #------------------ Filtering Theta----------------------
		valid_points = []
		for i in range(l):
        	        if (sph_points[i][1] > -135) and (sph_points[i][1] < -45) : 
        		        valid_points.append(sph_points[i])
		valid_points = np.array(valid_points)		
        #--------------- BACK TO CART ----------------------
		l1 = len(valid_points)
		filtered_points = np.empty ((l1,3),dtype='float64')
		for point in range (l1):
			r = valid_points[point][0]
			theta = valid_points[point][1]
			theta= math.radians(theta)
			phai= valid_points[point][2]
			phai=math.radians(phai)
			x,y,z=cs.sp2cart(r,phai,theta)
			filtered_points[point][0]=x
			filtered_points[point][1]=y
			filtered_points[point][2]=z	

		self.current_pc = filtered_points
	

	def save_rgb (self,image):
		#image.save_to_disk('haha.png')
		out = np.frombuffer(image.raw_data , dtype=np.dtype('uint8'))
		out = out.reshape((640,1920,4))
		out = out[:,:,:3]
		self.current_rgb = out

	def initialize_model (self):
		anchors = [[ 0.57273 ,  0.677385],[ 1.87446 ,  2.06253 ],[ 3.33843 ,  5.47434 ], [ 7.88282 ,  3.52778 ],[ 9.77052 ,  9.16828 ]]
		class_names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
		print('** Loading Model')
		self.model = load_model(input_shape = (608,608,3))
		print('** Loading weights')
		self.model.load_weights('weights/weights.h5')
		self.anchors = np.array(anchors)
		self.class_names = class_names 

		yolo_outputs = yolo_head(self.model.output, anchors, len(class_names))
		image_shape = (608.0 , 608.0)
		scores, boxes, classes = yolo_eval(yolo_outputs, image_shape)
		self.scores = scores
		self.boxes  = boxes
		self.classes= classes  		

	def run_net (self, img_path):

		model = self.model
		scores = self.scores
		boxes  = self.boxes
		classes= self.classes 

		assert model!=None		
		image_test = Im.array_to_img(img_path) 
		image_data = Im.img_to_array(image_test) 
		image_data /= 255.0 
		image_data = np.expand_dims(image_data, axis = 0)

		start = time.time()    
		sess = tf.compat.v1.keras.backend.get_session()
		out_scores, out_boxes, out_classes = sess.run([scores, boxes, classes], feed_dict={model.input: image_data})
		end = time.time()
		print('Session: ', end - start)

		self.current_scores=out_scores
		self.current_boxes=out_boxes
		self.current_classes=out_classes 


	def pred_dis (self):
		boundary_boxes = self.current_boxes 
		point_cloud_3D = self.current_pc

		image_RGB = self.current_rgb
		FOV = 90

		fx=(0.5* self.image_width) / (math.tan((FOV*math.pi)/360)) 
		fy=(0.5* self.image_height) / (math.tan((FOV*math.pi)/360)) 

		M_calb= np.array  ([ [fx , 0 , (0.5* self.image_width) ] , 
		                     [0 , fy , (0.5* self.image_height)] , 
							 [0 , 0, 1]                              ])
		
		l=len(point_cloud_3D ) 
		LiDAR_3D_id=np.zeros((l,4) , dtype='float64') 
		for counter in range (l):
			LiDAR_3D_id[counter][0]=point_cloud_3D[counter][0]
			LiDAR_3D_id[counter][1]=point_cloud_3D[counter][1]
			LiDAR_3D_id[counter][2]=point_cloud_3D[counter][2]
			LiDAR_3D_id[counter][3]=counter

		LiDAR_2D= np.empty ((l,3)  , dtype='float64')
		x = np.ones((1,))
		for counter in range (l):
			#a = np.asarray(point_cloud_3D[counter])
			x=point_cloud_3D[counter][0]
			y=point_cloud_3D[counter][1]
			z=point_cloud_3D[counter][2]	
			a=np.array([[x],[y],[z]])
			#x = [1]
			#b = np.array(x)
			#c = np.concatenate((a,x),axis=0)
			haha = M_calb.dot(a)
			hehe = haha.transpose()
			#LiDAR_2D[counter] = np.concatenate((hehe , np.array([counter])),axis=0)
			#print(hehe)
			LiDAR_2D[counter] = hehe

		#continue as boundary_boxes are in the form of corners 
		#with open('haha.txt','w+') as new_file:
		#	for i in range(len(LiDAR_2D)):
		#		new_file.write(str(LiDAR_2D[i][0])+ ',' + str(LiDAR_2D[i][1]) +','+ str(LiDAR_2D[i][2])+'\n')

		Distances=[]
		Thetas = []
		valid_xyz=[]
		valid_thetas=[]
		
		for box in boundary_boxes:
			 
			valid_IDs=[] 
			valid_r_points=[]
			X_min= box[0][0]
			X_max= box[1][0]
			y_min= box[0][1]
			y_max= box[2][1]

			for j in range (len(LiDAR_2D)) : 
				x=LiDAR_2D[j][0]
				y=LiDAR_2D[j][1]
				if ((x>=X_min) and (x<=X_max)) and ((y>=y_min) and  (y<=y_max)):
					print('------------- found a point ---------------')
					valid_IDs.append(LiDAR_3D_id[j][3])
					#valid_Z_points.append(LiDAR_3D_id[j][2])
					valid_xyz.append([x,y,LiDAR_3D_id[j][2]])
					r, phai, theta = cs.cart2sp(x, y,LiDAR_3D_id[j][2])
					phai = math.degrees(phai)
					theta = math.degrees(theta)
					valid_thetas.append(theta)
					valid_r_points.append(r)
			if len(valid_r_points) == 0:
				continue 
			Distances.append(min(valid_r_points)/100)
			Thetas.append(valid_thetas[valid_r_points.index(min(valid_r_points))])
				
		return Distances , Thetas


	def check_if_clear_ahead(self):
		"""
		this function return one if the road is open
		"""
		bounding_boxes = self.current_boxes
		distances = self.current_distances
		clear=1
		for i in range(len(bounding_boxes)):
			bbox = bounding_boxes[i]
			points = [(int(bbox[j,0]) , int(bbox[j,1])) for j in range(4)]
			points=np.asarray(points)
			xmin , ymin=np.min(points, axis=0)
			xmax , ymax=np.max(points, axis=0)
			if xmin < (1920/2) and xmax > (1920/2)-20:
				return 0 ,distances[i], bbox

		return clear, -1, -1

	def adjustSpeed(self):
		_,self.currentDistance,_=self.check_if_clear_ahead()
		differance_between_distances=self.currentDistance-self.prevDistance
		if differance_between_distances==0 and self.currentDistance >=self.saftyFactor:
			self.prevDistance = self.currentDistance
			self.speed+=5
		elif differance_between_distances==0 and self.currentDistance <self.saftyFactor:
			self.prevDistance=self.currentDistance
			self.speed-=5
		elif differance_between_distances!=0:
			self.prevDistance=self.currentDistance
			self.speed+=(differance_between_distances/self.unit_time)


	def temproral_function_for_testing(self):
		self.LaneFollow = LaneFollow.open_way
		self.LaneChange = LaneChange.both


	def check_data(self):
		print('RGB size is ',self.current_rgb.shape)
		print('RGB size is ',self.current_pc.shape)

	def run_step(self):

		if self.current_rgb is None:
			print('Missed Image')
		else:
			print('*** Running Net')
			self.run_net(self.current_rgb) 
			print('*** Predicting distances')
			self.current_distances ,  self.angles = self.pred_dis()
			print('*** checking ahead')
			clear , distance_ahead , bbox_ahead = self.check_if_clear_ahead()
			print('*** Updating Lane Follow')
		

		self.temproral_function_for_testing()		

		'''	
		# Updating Lane Follow
		if clear == 1: # There's no ahead objects
			self.LaneFollow = LaneFollow.open_way
			# Speed in this case 

		if clear == 0: # Ther's an object a head
			adjustSpeed() # Update (speed & prevDistance & currentDistance)
			if distance_ahead > 5:
				self.LaneFollow = LaneFollow.conjusted_way
			elif distance_ahead <= 5:
				self.LaneFollow = LaneFollow.blocking_way
		print('***** Updating Lane Change ******')
		# Updating Lane Change
		left_available  = True
		right_available = True

		for i in range(len(self.current_distances)):
			if self.current_distances[i] < 10:
				if self.angles[i] > 45 and self.angles[i] < 85:    # car is on the right
					right_available = False 
				elif self.angles[i] > 95 and self.angles[i] < 135: # car is on the left
					left_available = False

		if left_available==True and right_available==False:
			wp = localizer.waypoint()
			if wp.left_lane_marking.type is carla.LaneMarkingType.Broken
				self.LaneChange = LaneChange.left
			else:
				self.LaneChange = LaneChange.block
		
		elif left_available==False and right_available==True:
			wp = localizer.waypoint()
			if wp.right_lane_marking.type is carla.LaneMarkingType.Broken:
				self.LaneChange = LaneChange.right
			else:
				self.LaneChange = LaneChange.block
		
		elif left_available==False and right_available==False:
			self.LaneChange = LaneChange.block
		
		else:
			wp = localizer.waypoint()
			if wp.left_lane_marking.type is carla.LaneMarkingType.Broken and wp.right_lane_marking.type is carla.LaneMarkingType.Broken:
				self.LaneChange = LaneChange.both
			elif wp.left_lane_marking.type is carla.LaneMarkingType.Broken:
				self.LaneChange = LaneChange.left
			elif wp.right_lane_marking.type is carla.LaneMarkingType.Broken:
				self.LaneChange = LaneChange.right
			else:
				self.LaneChange = LaneChange.block
		'''




