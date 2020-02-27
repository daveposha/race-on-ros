#!/usr/bin/env python

import pygame
import rospy
from std_msgs.msg import Float32, Bool



class JoystickNode:

    def __init__(self):
        self.topic_name_joystick_steer = rospy.get_param("topic_name_joystick_steer", "joystick/steer")
        self.topic_name_joystick_gas = rospy.get_param("topic_name_joystick_gas", "joystick/gas")
        self.topic_name_joystick_on = rospy.get_param("topic_name_joystick_on", "joystick/on")

        self.pub_joystick_steer = rospy.Publisher(self.topic_name_joystick_steer, Float32, queue_size=1)
        self.pub_joystick_gas = rospy.Publisher(self.topic_name_joystick_gas, Float32, queue_size=1)
        self.pub_joystick_on = rospy.Publisher(self.topic_name_joystick_on, Bool, queue_size=1)

        self.RATE = 60  # Publishing rate of new data per second
        self.rospy_rate = rospy.Rate(self.RATE)

        # Setup joystick
        self.joystickOn = False
        pygame.init()
        self.clock = pygame.time.Clock()
        pygame.joystick.init()
        self.joystick_count = pygame.joystick.get_count()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.axes = self.joystick.get_numaxes


# Execute this when run as a script
if __name__ == '__main__':

    rospy.init_node('joystick')
    joystick_node = JoystickNode()

    while not rospy.is_shutdown():

        for event in pygame.event.get():  # User did something.
            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            elif event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")

        steering = joystick_node.joystick.get_axis(0)  # left stick x axis
        # print("steering: " + str(steering))
        joystick_node.pub_joystick_steer.publish(steering)
        # DUTY_CYCLE = int(SERVO_MIDDLE + 500000 * steering)

        gas = (joystick_node.joystick.get_axis(5) + 1) / 2  # right trigger, normalize from (-1, 1) to (0, 1)
        # print("gas: " + str(gas))
        joystick_node.pub_joystick_gas.publish(gas)

        # motor.duty_cycle = int(MOTOR_BRAKE + (180000 - MOTOR_BRAKE) * gas)

        if joystick_node.joystick.get_button(2):  # "X" button
            joystick_node.joystickOn = False
        # TODO: add button
        elif joystick_node.joystick.get_button(...):  # "A" button
            joystick_node.joystickOn = True
        joystick_node.pub_joystick_on.publish(joystick_node.joystickOn)

        # Wait to match the rate
        joystick_node.rospy_rate.sleep()

    # joystick
    pygame.quit()