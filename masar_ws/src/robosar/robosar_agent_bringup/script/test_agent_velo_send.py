#!/usr/bin/env python3
# license removed for brevity

import rospy
import math
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
# Import message file
#from khepera_communicator.msg import K4_controls, SensorReadings

start = time.time()

def talker():
	# Set up publisher
	pub = rospy.Publisher('/robosar_agent_bringup_node/agent1/control', Twist, queue_size=10)
	rospy.init_node('Central_Algo', anonymous=True)
	
	# Set publish rate
	rate = rospy.Rate(10) # 10hz
	
	# Message type
	msg = Twist()

	while not rospy.is_shutdown():
		
		end = time.time()
		t = end - start
		
		msg.angular.z = 0.0
		msg.linear.x = 200 * math.sin(3.14159 * t) 
		msg.linear.x = 0

		#rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass