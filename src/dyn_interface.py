#!/usr/bin/env python
import rospy
import tf
from auv_msgs.msg import NavSts
from uw_vs.msg import PilotRequest
from geometry_msgs.msg import Pose, TwistStamped
import numpy as np

global TOPIC_NAV # get the vehicle simulated pose
global TOPIC_POSE # publishes simulated pose on UWSim
global TOPIC_CMD # get the controller command
global TOPIC_PILOT # publishes the command to the pilot
global sub_nav # subscribes to simulator to get navigation data
global pub_pose # publishes the pose on UWSim (on simulation)
global sub_cmd # subscribes to controller to get the command
global pub_pilot # publishes the command to the vehicle pilot (on simulation)
global pose # vehicle pose [x,y,z,r,p,q]

def callbackCmd(data):	
	# get command and publishes it to simulator
	pr = PilotRequest()
	pr.header.stamp = rospy.Time.now()
	pr.velocity = np.array([data.twist.linear.x,
							data.twist.linear.y,
							data.twist.linear.z,
							data.twist.angular.x,
							data.twist.angular.y,
							data.twist.angular.z])
	pub_pilot.publish(pr)

def callbackNav(data):
	# get simulated pose and publishes it into UWSim
	pose.position.x = data.position.north
	pose.position.y = data.position.east
	pose.position.z = data.position.depth
	quaternion = tf.transformations.quaternion_from_euler(data.orientation.roll, data.orientation.pitch, data.orientation.yaw)
	pose.orientation.x = quaternion[0]
	pose.orientation.y = quaternion[1]
	pose.orientation.z = quaternion[2]
	pose.orientation.w = quaternion[3]
	pub_pose.publish(pose)

def repeater():
	rospy.init_node('dyn_interface', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	rospy.spin()

if __name__ == '__main__':
	rospy.loginfo("main")
	TOPIC_POSE = '/nessie/pose'
	TOPIC_NAV = '/nav/nav_sts'
	TOPIC_CMD = '/cmd/twist'
	TOPIC_PILOT = '/pilot/velocity_req'
	pose = Pose()
	sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, callbackNav)
	pub_pose = rospy.Publisher(TOPIC_POSE, Pose, queue_size=10)
	sub_cmd = rospy.Subscriber(TOPIC_CMD, TwistStamped, callbackCmd)
	pub_pilot = rospy.Publisher(TOPIC_PILOT, PilotRequest, queue_size=10)
	repeater()

