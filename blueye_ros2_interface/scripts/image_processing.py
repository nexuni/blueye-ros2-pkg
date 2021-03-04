#!/usr/bin/env python3
#import tensorflow as tf
#from tensorflow import keras
import rospy
import cv2
import numpy as np
import os
import math
import time
from cv_bridge import CvBridge

from std_msgs.msg import Bool
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point

###
#   Blurs and resizes the original picture to lose detail
#   of the cage net on purpose.
#
#   args: original image of a cage
#   returns: blurred and not blurred image of all prominent edges found in the original image,
#           not blurred but initially more blurred image for bottom net detection
###
def preprocess_image_for_lines(input_img):
    copied_img = cv2.blur(input_img,(3,3))
    CLAHE_img = get_clahe_image(copied_img)
    resized = resize_img(20, CLAHE_img)
    gray = cv2.cvtColor(resized, cv2.COLOR_RGB2GRAY)
    gray_bottom = cv2.blur(gray, (5, 5))
    gray = cv2.blur(gray,(3,3))
    canny = cv2.Canny(gray, 10, 30, 3)
    canny_bottom = cv2.Canny(gray_bottom, 10, 30, 3)
    resized_large = resize_img(500, canny)
    resized_large_bottom = resize_img(500, canny_bottom)
    blurred_canny = cv2.blur(resized_large,(3,3))
    return blurred_canny, resized_large, resized_large_bottom

###
#   args: an image
#   returns: image whose histogram is equalized using clAHE algorithm
#   MORE INFO:  https://www.geeksforgeeks.org/clahe-histogram-eqalization-opencv/ [accessed: 11th Januray 2021]
###
def get_clahe_image(img):
    lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l,a,b = cv2.split(lab_img)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(16,16))
    clahe_img = clahe.apply(l)
    combo = cv2.merge((clahe_img, a, b))
    CLAHE_img = cv2.cvtColor(combo, cv2.COLOR_LAB2BGR)
    return CLAHE_img

###
#   args:   percentage used for scaling the picture, picture to resize
#   returns: resized image
###
def resize_img(scale_percent, input_img):
    width = int(input_img.shape[1] * scale_percent / 100)
    height = int(input_img.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized = cv2.resize(input_img, dim, interpolation = cv2.INTER_AREA)
    return resized

###
#   args:   lines whose shape is (x1,y1,x2,y2)
#   returns:lines of same shape which are not horizontal
###
def get_vertical_lines(all_lines):
    return_array = []
    if all_lines is not None:
        for line in all_lines:
            x1, y1, x2, y2 = line.reshape(4)
            len_of_line = math.sqrt((x1-x2)**2 + (y1-y2)**2)
            if(abs(y1-y2) > (7/8 * len_of_line)):
                return_array.append([x1, y1, x2, y2])
    return return_array

###
#   args:   array of lines whose shape is (x1, y1, x2, y2)
#   returns:    array of lines that are either long enough or
#               have a number of other lines with similar enough X values
###
def get_useable_lines(lines_array):
    return_array = []
    if lines_array is not None:
        for line in lines_array:
            x1 = line[0]
            y1 = line[1]
            x2 = line[2]
            y2 = line[3]

            if(abs(y1-y2) > 250):
                return_array.append(line)
                print("Found a long line, it must be a rope!")
                continue

            count = -1
            for other in lines_array:
                other_x1 = other[0]
                other_y1 = other[1]
                other_x2 = other[2]
                other_y2 = other[3]
                if(abs(x1 - other_x1) < 3):
                    count = count + 1
            if(count > 1):
                return_array.append(line)

    return return_array

###
#   args:   array of lines, image used for dimension
#   returns:    all x1 values of lines on the LEFT hand side of the image,
#               all x1 values of lines on the RIGHT hand side of the image
###
def separate_lines(lines_array, input_img):
    avg_l = []
    avg_r = []
    if lines_array is not None:
        for point in lines_array:
            if(point[0] <= input_img.shape[1]/2):
                avg_l.append(point[0])
            else:
                avg_r.append(point[0])
    return avg_l, avg_r

###
#   args:   integer, integer, image that will be used for dimensions of return image
#   returns: image of 2 lines drawn based on the two integer values
##
def get_two_lines(value1, value2, input_img2):
    return_image = np.zeros_like(input_img2)
    cv2.line(return_image, (value1, 200), (value1, 800), (255,0,0), 10)
    cv2.line(return_image, (value2, 200), (value2, 800), (255,0,0), 10)
    return return_image


###
#   args:   image that should be same dimensions as original image, all detected straight lines on the original image
#   returns:    all non horizontal lines,
#               average of all lines grouped by on which side of the picture they are,
#               image of same dimensions as original, with 2 lines drawn on it
#                   based on the result from the average of all lines
###
def display_lines(input_img, input_lines):
    potential_points = get_vertical_lines(input_lines)
    actual_points = get_useable_lines(potential_points)

    avg_l, avg_r = separate_lines(actual_points, input_img)

    avg_of_lines = [-1, -1]
    if(avg_l):
        avg_of_lines[0] = int(np.median(avg_l))
    if(avg_r):
        avg_of_lines[1] = int(np.median(avg_r))

    line_image_avg = get_two_lines(avg_of_lines[0], avg_of_lines[1], input_img)
    return actual_points, avg_of_lines, line_image_avg


###
#   args:   RGB image, Grayscale image
#   returns: grayscale image overlayed over the RGB image
###
def overlay_color_gray_images(color, gray):
    r1 = cv2.resize(color, (1280, 720))
    r = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    r2 = cv2.resize(r, (1280, 720))
    r3 = cv2.addWeighted(r1, 0.5, r2, 0.5, 0)
    return r3

###
#   args:   full canny image
#   returns:    True: if it is a bottom of a net or no net at all
#               False: if the net continues
#   current threshold = 0.9
###
def bottom_net(image):
    image = image_crop(image)
    black_percent = getBlackProportion(image, 255)
    #print(black_percent)

    if black_percent < 0.9:
        return False
    else:
        return True

### 
#   args:   full canny image 
#   returns:    cropped canny image (currently from 2/3 to 3/3 height) 
### 
def image_crop(image): 
    maxHeight, maxWidth = image.shape 
    minHeight =	 int(maxHeight*2/3) 
    return image[minHeight:maxHeight, 0:maxWidth] 
 
### 
#   args:   cropped canny image, threshold for color detection (black) 
#   returns:    percentage of clear space 
### 
def getBlackProportion(img, threshold): 
	maxHeight, maxWidth = img.shape 
	imgSize = maxWidth * maxHeight 
	cv2.threshold(img, 127, threshold, -1, cv2.THRESH_BINARY) 
	nonzero = cv2.countNonZero(img) 
	return (imgSize - nonzero) / (imgSize) 

###
#
#   args: original image of a cage(expected dimension 1080p)
#   returns: image of 2 lines of where the 2 ropes are, overlayed over the original image and the positions of the detected lines
###
def get_final_image(original_image):
	blurred_canny_image, large_image, bottom_net_image = preprocess_image_for_lines(original_image)
	is_bottom_net = bottom_net(bottom_net_image)
	lines = cv2.HoughLinesP(blurred_canny_image, 2, np.pi/180, 100, np.array([]), minLineLength=50, maxLineGap=5)
	points, averaged_lines, line_image_avg = display_lines(large_image, lines)
	combined_image = overlay_color_gray_images(original_image, line_image_avg)

	if averaged_lines[0] != -1:
		averaged_lines[0] /= original_image.shape[1]
	if averaged_lines[1] != -1:
		averaged_lines[1] /= original_image.shape[1]
	return combined_image, averaged_lines, is_bottom_net

class image_processing():
				
	def rawimage_callback(self, data):
		start = time.time()
		image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
		combined_image, averaged_lines, is_bottom_net = get_final_image(image)
		end = time.time()
		print("Processing time: ", end - start)
		vertical_lines_message = Float32MultiArray()
		vertical_lines_message.data = averaged_lines
		bottom_detected = Bool(is_bottom_net)
		#net_dirtiness_matrix = self.clean_net_CNN(image)
		
		self.v_lines_det.publish(vertical_lines_message)
		self.bottom_reached.publish(bottom_detected)
			
	def camerainfo_callback(self, data):
		pass

	def __init__(self):
		
		rospy.Subscriber("blueye_camera/image_raw", Image, self.rawimage_callback)
		rospy.Subscriber("blueye_camera/camera_info", CameraInfo, self.camerainfo_callback)

		self.v_lines_det = rospy.Publisher("detected_vertical_lines", Float32MultiArray , queue_size = 1)
		self.dirty_net = rospy.Publisher("detected_net_dirtiness",Int8MultiArray, queue_size = 1)	
		self.bottom_reached = rospy.Publisher("detected_bottom_of_net", Bool, queue_size = 1)
		#self.clean_net_CNN = 
		#self.clean_net_CNN.load_weights('clean_net_CNN')
		self.bridge = CvBridge()

	def run(self):
		while not rospy.is_shutdown():
			rospy.spin()

if __name__ == "__main__":
	rospy.init_node('image_processing')
	try:
		image_processing = image_processing()
		image_processing.run()
	except rospy.ROSInterruptException:
		pass
