#!/usr/bin/python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from px4_mavros import Px4Controller
from gazebo_msgs.msg import ModelStates
from math import sin,cos,sqrt,atan2,acos,pi

P0, P1, P2, P3, P3_v = None, None, None, None, None
cmd_vel = Twist()

def odom(msg):

	global P0, P1, P2, P3, P3_v
	
	UAV0_index = msg.name.index('iris_0')
	UAV1_index = msg.name.index('iris_1')
	UAV2_index = msg.name.index('iris_2')
	UAV3_index = msg.name.index('solo_3')

	P0 = np.array([msg.pose[UAV0_index].position.x, msg.pose[UAV0_index].position.y, msg.pose[UAV0_index].position.z])
	P1 = np.array([msg.pose[UAV1_index].position.x, msg.pose[UAV1_index].position.y, msg.pose[UAV1_index].position.z])
	P2 = np.array([msg.pose[UAV2_index].position.x, msg.pose[UAV2_index].position.y, msg.pose[UAV2_index].position.z])
	P3 = np.array([msg.pose[UAV3_index].position.x, msg.pose[UAV3_index].position.y, msg.pose[UAV3_index].position.z])
	P3_v = np.array([msg.twist[UAV3_index].linear.x, msg.twist[UAV3_index].linear.y, msg.twist[UAV3_index].linear.z])
	
def	controller():

	global cmd_vel

	tra = [3*cos(t*pi), 3*sin(t*pi)]
	cmd_vel.linear.x = 0.7*(tra[0] - P3[0])
	cmd_vel.linear.y = 0.7*(tra[1] - P3[1])
	cmd_vel.linear.z = 0.5 - P3[2]

	px4_3.vel_control(cmd_vel)

if __name__ == '__main__':

	try:
		rospy.init_node('controller_3')
		px4_3 = Px4Controller("solo_3")
		rospy.Subscriber('/gazebo/model_states', ModelStates, odom, queue_size=10)
		rate = rospy.Rate(100)

		while P3 is None:

			rate.sleep()

		t = 0

		while not rospy.is_shutdown():

			controller()

			t += 0.0005
			rate.sleep()

	except rospy.ROSInterruptException:

		pass