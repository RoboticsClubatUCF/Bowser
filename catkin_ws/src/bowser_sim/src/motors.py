#!/usr/bin/env python
import rospy

from pynput import keyboard

from geometry_msgs.msg import Twist

motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)  # creating publisher for topic /cmd_vel (that the robot subscribes to)

def on_press(key):
    ''' Every time a key is pressed, create and publish a twist message (if key pressed was an arrow key) '''

    # create Twist message, set unused fields to 0
    twist = Twist()
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0

    try:
        k = key.char
    except AttributeError:
        k = key.name

    # modify twist message depending on which arrow key pressed    
    if( k == 'left' ):
        twist.angular.z = -1
    if( k == 'right' ):
        twist.angular.z = 1
    if( k == 'up' ):
        twist.linear.x = -1
    if( k == 'down' ):
        twist.linear.x = 1      

    # publish the message on /cmd_vel for the robot to pick up    
    motor_pub.publish(twist)

def on_release(key):
    ''' When a key is released, send a zeroed out Twist message to stop the robot '''    

    # create Twist message
    twist = Twist()
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0

    try:
        k = key.char
    except AttributeError:
        k = key.name

    # zero out the field of whatever key was released    
    if( k == 'left' ):
        twist.angular.z = 0
    if( k == 'right' ):
        twist.angular.z = 0
    if( k == 'up' ):
        twist.linear.x = 0
    if( k == 'down' ):
        twist.linear.x = 0  

    # publish the zeroed Twist message    
    motor_pub.publish(twist)

    # if Escape is pressed, exit keyboard listener
    if key == keyboard.Key.esc:
        # Stop listener
        return False    

def main():

    rospy.init_node('motor_controller', anonymous=True) # node that will handle sending commands
    listener = keyboard.Listener()  # pynput keyboard listener for catching arrow key input

    # when key is pressed, run on_press(), when a key is released, run on_release()
    with keyboard.Listener(
        on_press=on_press,  
        on_release=on_release) as listener: 
        listener.join()

    rospy.spin()    # basically just blocks out main so the node doesn't close


if __name__=='__main__':
    main()