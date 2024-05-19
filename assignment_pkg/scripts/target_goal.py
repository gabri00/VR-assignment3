#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int64
from assignment_pkg.msg import DistData
import airsim
import time

from Logger import Logger

class PlanningGoal:
	def __init__(self, node_name):
		self._node_name = node_name
		rospy.init_node(self._node_name)

		# Init logger
		self.__logger = Logger(self._node_name)
		self.__logger.loginfo("Node started.")

		# Load parameters
		self.__load_params()
		
		# Initialize AirSim client
		self.__init_client()
		
		time.sleep(2)

		rospy.Timer(rospy.Duration(2), self.target_goal)


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def __init_client(self):
		self.client = airsim.MultirotorClient(ip=self.host, port=self.port)
		self.client.confirmConnection()
		self.client.enableApiControl(True)
		self.client.armDisarm(True)
		self.client.takeoffAsync().join()


	def target_goal(self, event):
		# Create the action client
		self.client_action = actionlib.SimpleActionClient('target_point', assignment_pkg.msg.PlanningAction)
    
		# Wait for the server to be started
		self.client_action.wait_for_server()

		while not rospy.is_shutdown():
	  
		        self.target_long = rospy.get_param('~target_long')
			self.target_lat = rospy.get_param('~target_lat')
   
			# Create the goal position that the drone has to reach
	,               self.goal = assignment_pkg.msg.PlanningGoal()
			self.goal.latitude = self.target_lat
			self.goal.longitude = self.target_long
			

			# Send the goal to the server
			self.client_action.send_goal(goal)

			# The user has 10s in order to cancel the goal by typing 'c'
			self.logger.loginfo("Enter 'c' to cancel the goal:")
			self.val = select.select([sys.stdin], [], [], 10)[0]
      
			if self.val:
				self.value = sys.stdin.readline().rstrip()
				   
				if (self.value == "c"):
					# Goal cancelled
					self.logger.loginfo("Goal has been cancelled!")
					self.action_client.cancel_goal()


def main():
	PlanningGoal('target_goal_node')
	rospy.spin()


if __name__ == '__main__':
	main()
