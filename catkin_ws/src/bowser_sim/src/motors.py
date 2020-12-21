#!/usr/bin/env python

import rospy
from pynput import keyboard
from geometry_msgs.msg import Twist

motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)  # creating publisher for topic /cmd_vel (that the robot subscribes to)

# Twist message to be published, modified by on_press and on_release
twist = Twist()
twist.linear.y = 0
twist.linear.z = 0
twist.angular.x = 0
twist.angular.y = 0

# tells us when the Twist message needs to be published again
change_flag = False

def on_press(key):
    ''' Every time a key is pressed, create and publish a twist message (if key pressed was an arrow key) '''

    global change_flag

    try:
        k = key.char
    except AttributeError:
        k = key.name

    # modify twist message depending on which arrow key pressed    
    if( k == 'left' ):
        twist.angular.z = -1
        change_flag = True
    if( k == 'right' ):
        twist.angular.z = 1
        change_flag = True
    if( k == 'up' ):
        twist.linear.x = -1
        change_flag = True
    if( k == 'down' ):
        twist.linear.x = 1
        change_flag = True      

    # update published twist message
    update()    

def on_release(key):
    ''' When a key is released, zero that key out in the Twist message '''    

    global change_flag

    try:
        k = key.char
    except AttributeError:
        k = key.name

    # zero out the field of whatever key was released    
    if( k == 'left' ):
        twist.angular.z = 0
        change_flag = True
    if( k == 'right' ):
        twist.angular.z = 0
        change_flag = True
    if( k == 'up' ):
        twist.linear.x = 0
        change_flag = True
    if( k == 'down' ):
        twist.linear.x = 0 
        change_flag = True 

    # if Escape is pressed, exit keyboard listener
    if key == keyboard.Key.esc:
        # Stop listener
        return False    

    # update published twiest message
    update()


def update():

    global change_flag

    # if a new key was pressed or released, update the twist message
    if (change_flag):
        motor_pub.publish(twist)
        change_flag = False
        

def main():

    rospy.init_node('motor_controller', anonymous=True) # node that will handle sending commands
    listener = keyboard.Listener()  # pynput keyboard listener for catching arrow key input

    # when key is pressed, run on_press(), when a key is released, run on_release()
    with keyboard.Listener(
        on_press=on_press,  
        on_release=on_release) as listener: 
        listener.join()

    rospy.spin()    # keeps this ros node from closing

if __name__=='__main__':
    main()