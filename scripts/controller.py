#!/usr/bin/env python
import rospy
import numpy as np
from reto_final.msg import set_point
from reto_final.msg import motor_input
from reto_final.msg import motor_output

#Setup parameters, vriables and callback functions here (if required)
integral = 0
last_error = 0

def pid_control(target_value, current_value):
    global integral, last_error, kp, ki, kd, dt

    error = target_value - current_value
    
    proportional = kp * error

    integral += error * dt
	#integral = np.clip(integral, -1, 1) # limite la acumulacion
    integral_term = ki * integral

    derivative = error - last_error
    derivative_term = kd * derivative

    output = proportional + integral_term + derivative_term

    last_error = error

    return output

def setpoint_subscriber(msg):
    global setpoint_msg
    setpoint_msg.input = msg.input
    setpoint_msg.tm = msg.tm

def motor_output_subscriber(msg):
    global motor_output_msg
    motor_output_msg.output = msg.output
    motor_output_msg.tm = msg.tm
    motor_output_msg.st = msg.st


if __name__=='__main__':
    global motor_input_msg, motor_output_msg, setpoint_msg, kp, ki, kd, dt
    
    kp = rospy.get_param("controller_kp", 0.03)
    kd = rospy.get_param("controller_kd", 0.02)
    ki = rospy.get_param("controller_ki", 0.01)

    rt = rospy.get_param("sampling", 100)
    dt = 1/rt

    motor_input_msg = motor_input()
    motor_output_msg = motor_output()
    setpoint_msg = set_point()

    #Initialise and Setup node
    rospy.init_node("Controller")
    rate = rospy.Rate(rt)

    #Setup Publishers and subscribers here
    rospy.Subscriber("set_point", set_point, setpoint_subscriber)
    rospy.Subscriber("motor_output", motor_output, motor_output_subscriber)
    pub_2 = rospy.Publisher("motor_input", motor_input, queue_size = 10)

    while not rospy.is_shutdown():

        #Write your code here
        motor_input_msg.input = pid_control(setpoint_msg.input, motor_output_msg.output)
        motor_input_msg.tm = setpoint_msg.tm
        pub_2.publish(motor_input_msg)

        rate.sleep()

