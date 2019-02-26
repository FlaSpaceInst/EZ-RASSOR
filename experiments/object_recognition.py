#!/usr/bin/env python

import rospy
import cv2
import roslib
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
from keras.preprocessing import image
from keras.applications.resnet50 import ResNet50, preprocess_input, decode_predictions

# import model and  implement fix found here.
# https://github.com/fchollet/keras/issues/2397
model = ResNet50(weights='imagenet')
model._make_predict_function()
graph = tf.get_default_graph()
target_size = (224, 224)

rospy.init_node('classify', anonymous=True)

detectedObjectPub = rospy.Publisher('/ez_rassor/object_detected', String, queue_size = 1)
probabilityPub = rospy.Publisher('/ez_rassor/object_detected_probability', Float32, queue_size = 1)
bridge = CvBridge()

msg_string = String()
msg_float = Float32()

def callback(image_msg):
    #First convert the image to OpenCV image 
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    cv_image = cv2.resize(cv_image, target_size)  # resize image
    np_image = np.asarray(cv_image)               # read as np array
    np_image = np.expand_dims(np_image, axis=0)   # Add another dimension for tensorflow
    np_image = np_image.astype(float)             # preprocess needs float64 and img is uint8
    np_image = preprocess_input(np_image)         # Regularize the data
    
    global graph                                  # This is a workaround for asynchronous execution
    with graph.as_default():
       prediction = model.predict(np_image)            # Classify the image
       prediction_string = decode_predictions(prediction, top=1)[0]   # Decode top 1 predictions
       msg_string.data = prediction_string[0][1]
       msg_float.data = float(prediction_string[0][2])
       detectedObjectPub.publish(msg_string)
       probabilityPub.publish(msg_float)      

rospy.Subscriber("/ez_rassor/front_camera/left/image_raw", Image, callback, queue_size = 1, buff_size = 16777216)

while not rospy.is_shutdown():
  rospy.spin()