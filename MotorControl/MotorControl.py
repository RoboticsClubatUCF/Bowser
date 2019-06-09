#!/usr/bin/env python
import serial
import time

class motocommand:
	throttleVar = 0
	steeringVar = 1
	ser = serial.Serial(
		port='/dev/cu.usbmodemMDC2XXX1',
		baudrate = 115200,
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		bytesize=serial.EIGHTBITS,
		timeout=1
		)

	def __init__(self):
		self.velosub = rospy.Subscriber("commandVelocity", Float32MultiArray, callback)
		return

	def callback(self, data):
		ser.write('!VAR %d %d\r'%(throttleVar, data[0]))
		ser.write('!VAR %d %d\r'%(steeringVar, data[1]))
		return


def main(args):
  rospy.init_node('motorcommanding', anonymous=False)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
