import cv2
import numpy as np
import glob
from os import listdir

img_array = []
path = '/home/deio/Desktop/Final Modules/demo'
names = listdir(path)
photos_names = []

for name in names:
	if name.find('.png'):
		photos_names.append(name)

photos_names.sort()
for name in photos_names:
	img = cv2.imread(path+name)
	img_array.append(img)


hight = 600
width = 800
size = (width,hight)

 
out = cv2.VideoWriter('Demo1.mp4',cv2.VideoWriter_fourcc('m','p','4','v'), 10, size)
 
for i in range(len(img_array)):
    out.write(img_array[i])
out.release()
