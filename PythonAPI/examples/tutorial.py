#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
from __future__ import print_function
import glob
import os
import sys
import cv2

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
os.environ['HDF5_DISABLE_VERSION_CHECK']='2'
import random
import time


import tensorflow as tf

from tensorflow.python.keras.models import load_model



graph = tf.get_default_graph()
import argparse
import keras
from keras.models import load_model
import random
from keras.preprocessing import image
from keras.preprocessing.image import img_to_array
import time

import numpy as np




import math


from PIL import Image
from keras.preprocessing import image


# Camera resolution settings

width = 320

height = 160

model = load_model('data/drive-model.h5')
OUTPUT_NORMALIZATION = 6600
previous_deviation = 0

 

# definition
def get_angle(predict):
    angle = predict[0][0]
    angle *= OUTPUT_NORMALIZATION
    angle /= 100
    return int(angle)




def lane_detection(image):

    """

    Detect Lines on the road. Return (None,None) if no valuable detection

 

    :param img: path to the image

    :return: (deviation, img_computed)

    """

 

    global height

    global width

    global previous_deviation
    global graph
    global sess

 

    # Image conversion from raw bytes data, to RGBA reshaped to RGB
    i = np.array(image.raw_data)

    i2 = i.reshape((height, width, 4))

    img = cv2.cvtColor(i2, cv2.COLOR_RGBA2RGB)
    img = Image.fromarray(img, 'RGB')
    
    left = 0
    top = height / 6
    right = 320
    bottom = height
    img = img.crop((left, top, right, bottom))
    img = img.resize((200,66))
    arr = img_to_array(img)
    i2=arr/ 255.0
    arr = np.reshape(arr, (1,) + arr.shape)
    
   
    with graph.as_default():
        
        angle = get_angle(model.predict(arr, batch_size = 1))
    vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=angle))


  

actor_list = []

 

try:

############### Don't forget to use the 4th map with the Clear weather preset using the folowing command in the PythonAPI/util directory : ###############

############### "python config.py -m Town04 --weather ClearNoon" ################

 

    # Connection to the carla server

    client = carla.Client("localhost", 2000)

    client.set_timeout(2.0)
    

 

    # Retrieving the map

    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

   

    bp = blueprint_library.filter("grandtourer")[0]

    spawn_point = random.choice(world.get_map().get_spawn_points())

    # Spawning the vehicle at a random map point, and applying a throttle input to move it

    vehicle = world.spawn_actor(bp, spawn_point)

    vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))

    actor_list.append(vehicle)

 

    # Spawning the camera and attaching it to the spawned vehicle

    camera_bp = blueprint_library.find('sensor.camera.rgb')

    # Setup og the camera

    camera_bp.set_attribute('image_size_x', str(width))

    camera_bp.set_attribute('image_size_y', str(height))



    camera_bp.set_attribute('sensor_tick', '0.08')

    # Attaching the camera to the front of the vehicle

    spawn_point = carla.Transform(carla.Location(x=1.5, z=1.7))

    camera = world.spawn_actor(camera_bp, spawn_point, attach_to=vehicle)

    actor_list.append(camera)

 

    # Listening to the camera data and process of the image and correction of the trajectory

    camera.listen(lambda data: lane_detection(data))

   

    # Timeout before closing the connection and destroying the spwaned vehicle and camera

    time.sleep(200)

 

finally:

    for actor in actor_list:

        actor.destroy()

    print("Cleaned Up")