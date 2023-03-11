#!/usr/bin/env python
import rospy
import numpy as np
from reto_final.msg import set_point

def triangle():
    global actual_value, direction, step, max_value, min_value
    actual_value += (direction*(-step)) or (step)
    if (actual_value > max_value):
        direction = True
    elif (actual_value < min_value):
        direction = False
    return actual_value

def cuadrada():
 	global actual_value, direction, step, lon_1, lon_2
 	if(actual_value <= 0):
 		actual_value = (direction*(lon_1)) or (lon_2)
 		direction = not direction
 		return 0.0
 	actual_value -= 1
 	return (direction*(1)) or (-1)


if __name__ == "__main__":
	global actual_value, direction, step, max_value, min_value
	actual_value = rospy.get_param("initial_pos", 0)
	direction = rospy.get_param("boolean", True)
	step = rospy.get_param("step", 0.01)
	lon_1 = rospy.get_param("lon_1", 5)
	lon_2 = rospy.get_param("lon_2", 15)
	max_value = rospy.get_param("max_value", 1)
	min_value = rospy.get_param("min_value", -1)
	f_s = rospy.get_param("function", "crd")

	pub_1 = rospy.Publisher("set_point", set_point, queue_size=10)
	rospy.init_node("Input")
	rate = rospy.Rate(10)

	fcn = {"trg": triangle, "crd": cuadrada}

	msg_toSend = set_point()
	init_time = rospy.get_time()

	while not rospy.is_shutdown():
		msg_toSend.tm = rospy.get_time() - init_time
		msg_toSend.input = (fcn[f_s])()
		pub_1.publish(msg_toSend)
		rate.sleep()