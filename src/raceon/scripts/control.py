#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Pose
from raceon.msg import AckermannDrive


SERVO_MIN = -900
SERVO_MIDDLE = 0
SERVO_MAX = 900

class Controller():
    
    def __init__(self):
        self.topic_name_pos_err = rospy.get_param("topic_name_position_error", "position/error")
        self.topic_name_control = rospy.get_param("topic_name_control", "control")
        
        # Parameters for control
        self.motor_speed = rospy.get_param("~motor_speed", 200)
        self.target = rospy.get_param("~target", 0)
        self.kp = rospy.get_param("~kp", 1)

        # Joystick
        self.topic_name_joystick_steer = rospy.get_param("topic_name_joystick_steer", "joystick/steer")
        self.topic_name_joystick_gas = rospy.get_param("topic_name_joystick_gas", "joystick/gas")
        self.topic_name_joystick_on = rospy.get_param("topic_name_joystick_on", "joystick/on")
    
    def start(self):
        self.sub_pos_err = rospy.Subscriber(self.topic_name_pos_err, Pose, self.pos_err_callback)
        self.pub_control = rospy.Publisher(self.topic_name_control, AckermannDrive, queue_size=10)

        # joystick
        self.sub_joystick_steer = rospy.Subscriber(self.topic_name_joystick_steer, Float32, self.joystick_steer_callback)
        self.sub_joystick_gas = rospy.Subscriber(self.topic_name_joystick_gas, Float32, self.joystick_gas_callback)
        self.sub_joystick_on = rospy.Subscriber(self.topic_name_joystick_on, Bool, self.joystick_on_callback)

        rospy.spin()

    def pos_err_callback(self, pos_err_msg):
        pos_err = self.target - pos_err_msg.position.x
        
        rospy.loginfo("Current error: pos_err = " + str(pos_err))
        
        servo_pos = self.control_servo(pos_err)
        motor_speed = self.motor_speed
        
        rospy.loginfo("Control command: servo_pos = " + str(servo_pos) + ", motor_speed = " + str(motor_speed))
        
        control_msg = AckermannDrive()
        control_msg.speed = motor_speed
        control_msg.steering_angle = servo_pos
        self.pub_control.publish(control_msg)
        
    # TODO: Implement PID
    def pid(self, error):
        return error * self.kp

    def control_servo(self, error):
        correction = self.pid(error)
        servo_pos = SERVO_MIDDLE + correction

        if servo_pos > SERVO_MAX:
            servo_pos = SERVO_MAX
        if servo_pos < SERVO_MIN:
            servo_pos = SERVO_MIN

        return servo_pos

if __name__ == "__main__":
    rospy.init_node("control")
    controller = Controller()
    try:
        controller.start()
    except rospy.ROSInterruptException:
        pass