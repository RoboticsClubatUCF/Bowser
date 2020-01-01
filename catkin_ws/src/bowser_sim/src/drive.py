from pynput import keyboard
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('key_ex', anonymous=True)
twist_pub = rospy.Publisher('bowser/diff_drive', Twist, queue_size=10)

def on_press(key):

	twist = Twist()
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0

	try:
		k = key.char
	except AttributeError:
		k = key.name

	if( k == 'left' ):
		twist.angular.z = -1
	if( k == 'right' ):
		twist.angular.z = 1
	if( k == 'up' ):
		twist.linear.x = -1
	if( k == 'down' ):
		twist.linear.x = 1		

	twist_pub.publish(twist)

def on_release(key):

	twist = Twist()
	twist.linear.x = 0
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0

	twist_pub.publish(twist)

	if key == keyboard.Key.esc:
		# Stop listener
		return False    

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()