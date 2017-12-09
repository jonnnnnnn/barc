#!/usr/bin/env python
import rospy
from barc.msg import ECU
# import geometry_msgs
from geometry_msgs.msg import Twist

msgout = ECU()

class RateRelay(object):

	def __init__(self, dt=0.05):
		self.sub = rospy.Subscriber('turtle1/cmd_vel', Twist, self._publish_ecu)
		self.pub = rospy.Publisher('ecu', ECU, queue_size=10)
		self.rate = rospy.Rate(1.0/dt)

	def _publish_ecu(self,data):
		#add code to assemble the message to publish
		msgout.motor = -data.linear.x
		msgout.servo = -data.angular.z
		self.pub.publish(msgout)

	def _keypress(data):
		pass
		#add code to interpret different keypresses

	def run(self):
		while not rospy.is_shutdown():
			self._publish_ecu()
			self.rate.sleep()

def main():
	relay = RateRelay()
	relay.run
	rospy.spin()

if __name__=='__main__':
	rospy.init_node('process_key')
	main()