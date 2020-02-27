#!/usr/bin/env python

import rospy
import pygame

class JoystickControl():

    def __init__(self):
        # Setup joystick
        pygame.init()
        self.clock = pygame.time.Clock()
        pygame.joystick.init()
        self.joystick_count = pygame.joystick.get_count()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        self.axes = joystick.get_numaxes()

# joystick
for event in pygame.event.get():  # User did something.
    if event.type == pygame.JOYBUTTONDOWN:
        print("Joystick button pressed.")
    elif event.type == pygame.JOYBUTTONUP:
        print("Joystick button released.")

steering = joystick.get_axis(0)  # left stick x axis
print("steering: " + str(steering))
DUTY_CYCLE = int(SERVO_MIDDLE + 500000 * steering)

gas = (joystick.get_axis(5) + 1) / 2  # right trigger, normalize from (-1, 1) to (0, 1)
print("gas: " + str(gas))
motor.duty_cycle = int(MOTOR_BRAKE + (180000 - MOTOR_BRAKE) * gas)

if joystick.get_button(2):  # "x" button
    RUN_TIMER = 0  # stop the loop

# Execute this when run as a script
if __name__ == '__main__':

    rospy.init_node('publisher')

    pub  = rospy.Publisher('data', Float32, queue_size=1)
    rate = rospy.Rate(RATE)
    step = 0

    while not rospy.is_shutdown():

        # Generate new data point
        value = math.sin(2*math.pi*step/RATE)

        # Log and publish data
        rospy.loginfo("Publishing {:.3f}".format(value))
        pub.publish(value)

        # Advance the sequence
        step = step + 1

        # Wait to match the rate
        rate.sleep()

    # joystick
    pygame.quit()