#!/usr/bin/env python
import rospy
import numpy as np
from reto_final.msg import set_point

counter = 0
actual_time = 0

def triangle():
    global actual_value, direction, step, max_value, min_value
    actual_value += (direction*(-step)) or (step)
    if (actual_value > max_value):
        direction = True
    elif (actual_value < min_value):
        direction = False
    return actual_value

def cuadrada():
 	global counter, actual_value, direction, step, lon_top, lon_bottom
 	if(counter <= 0):
 		counter = (direction*(lon_bottom)) or (lon_top)
 		direction = not direction
 	counter -= step
 	actual_value = (direction*(max_value)) or (min_value)
 	return actual_value

def constant():
	global lon_top, actual_value
	actual_value = lon_top 
	return actual_value

def sinusoidal():
	global actual_value, step, counter

	return actual_value

def get_params():
	global step, max_value, min_value, lon_top, lon_bottom, f_s, actual_time
	step = rospy.get_param("step", 0.01)
	lon_top = rospy.get_param("lon_top", 50)
	lon_bottom = rospy.get_param("lon_bottom", 25)
	max_value = rospy.get_param("max_value", 20)
	min_value = rospy.get_param("min_value", -20)
	f_s = rospy.get_param("function", "crd")

if __name__ == "__main__":
	global actual_value, direction, step, max_value, min_value, f_s
	get_params()
	actual_value = rospy.get_param("initial_pos", 0)
	direction = True

	pub_1 = rospy.Publisher("set_point", set_point, queue_size=10)
	rospy.init_node("Input")
	rate = rospy.Rate(200)

	fcn = {"trg": triangle, "crd": cuadrada, "cons": constant, "sin": sinusoidal}

	msg_toSend = set_point()
	init_time = rospy.get_time()

	while not rospy.is_shutdown():
		get_params()
		actual_time = rospy.get_time() - init_time
		msg_toSend.tm = actual_time
		msg_toSend.input = (fcn[f_s])()
		pub_1.publish(msg_toSend)
		rate.sleep()