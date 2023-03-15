#!/usr/bin/env python
import rospy
import numpy as np
from reto_final.msg import set_point
from reto_final.msg import motor_input
from reto_final.msg import motor_output

integral = 0
last_error = 0

def pid_control(target_value, current_value):
    global integral, last_error, kp, ki, kd, dt

    error = target_value - current_value

    integral += error
    integral = np.clip(integral, 0, 35) # limite la acumulacion
     
    derivative = error - last_error

    proportional = error*(not (target_value > 0 and error < 0) or (target_value < 0 and error > 0))

    output = (kp * proportional) + (ki * integral) + (kd * derivative)
    output = output*(not (target_value > 0 and error < 0) or (target_value < 0 and error > 0))

    last_error = error
    return output

def caract_motor(x):
    v1 = 0.000007199
    v2 = 0.0006643
    v3 = 0.02327
    v4 = 0.3739
    v5 = 2.622
    v6 = 3.486
    v7 = 15.52

    pwm = v1*pow(x, 6) - v2*pow(x, 5) + v3*pow(x, 4) - v4*pow(x, 3) + v5*pow(x, 2) - v6*x + v7
    return pwm

def setpoint_subscriber(msg):
    global setpoint_msg
    setpoint_msg.input = msg.input
    setpoint_msg.tm = msg.tm


def motor_output_subscriber(msg):
    global motor_output_msg
    motor_output_msg.output = msg.output
    motor_output_msg.tm = msg.tm
    motor_output_msg.st = msg.st

def get_params():
    global kp, ki, kd, is_pid
    kp = rospy.get_param("controller_kp", 1)
    kd = rospy.get_param("controller_kd", 1)
    ki = rospy.get_param("controller_ki", 1)
    is_pid = rospy.get_param("pid", True)

if __name__=='__main__':
    global motor_input_msg, motor_output_msg, setpoint_msg, dt, is_pid

    rt = 16
    dt = 1/rt

    motor_input_msg = motor_input()
    motor_output_msg = motor_output()
    setpoint_msg = set_point()
    flag_tm = True

    rospy.init_node("Controller")
    rate = rospy.Rate(rt)

    rospy.Subscriber("set_point", set_point, setpoint_subscriber)
    rospy.Subscriber("motor_output", motor_output, motor_output_subscriber)
    pub_2 = rospy.Publisher("motor_input", motor_input, queue_size = 10)

    while not rospy.is_shutdown():
        get_params()
        #motor_input_msg.input = max(min((caract_motor(pid_control(setpoint_msg.input, motor_output_msg.output))/255)*is_pid or caract_motor(setpoint_msg.input)/255, 1), -1)
        if(setpoint_msg.input < 0):
            temp = -1
        else: 
            temp = 1
        #motor_input_msg.input = temp*pid_control(abs(setpoint_msg.input), abs(motor_output_msg.output))/35
        if (is_pid):
            motor_input_msg.input = temp*pid_control(abs(setpoint_msg.input), abs(motor_output_msg.output))/35
        else:
            motor_input_msg.input = setpoint_msg.input/35
        motor_input_msg.tm = flag_tm*35 or -35
        flag_tm = not flag_tm
        pub_2.publish(motor_input_msg)

        rate.sleep()