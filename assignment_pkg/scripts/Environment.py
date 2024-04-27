import airsim
import numpy as np

class DroneEnvironment():
    
    def __init__(self,):

        # Initialize AirSim client
	host = rospy.get_param('~host')
	self.client = airsim.MultirotorClient(ip=host, port=41451)
	self.client.confirmConnection()

	# Parameters
        self.scaling_factor = 3.8 # action scaling factor
        self.duration = 0.1 # seconds (Desired time to send this command for drivetrain)
        self.max_altitude = -1.15

        # Goal setup
        object_id = "Wp1"
        pose = self.client.simGetObjectPose(object_id).position # the object ID from unreal_engine World Outlier panel
        self.goal = np.array((pose.x_val, pose.y_val))

        # Quad initials
        position = self.client.getMultirotorState().kinematics_estimated.position
        self.initial_position = np.array([position.x_val, position.y_val])
        self.initial_distance = self.calculate_distance(self.initial_position, self.goal)



    
    def step(self, action, is_rate=True, angle_value=0):            
        # Interpret action
        self.interpret_action(action) # saves in self.quad_offset
        
        # Apply action
        if is_rate:
            self.client.moveByVelocityZBodyFrameAsync(
                vx = self.quad_offset[0], 
                vy = self.quad_offset[1], 
                z  = self.max_altitude, 
                duration = self.duration, 
                yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=self.quad_offset[2])
                ).join()
        else:
            self.client.moveByVelocityZBodyFrameAsync(
                vx = self.quad_offset[0], 
                vy = self.quad_offset[1], 
                z  = self.max_altitude, 
                duration = self.duration + 0.5, 
                yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=angle_value)
                ).join()
        
        # New Quad Kinematics
        KinematicsState = self.client.getMultirotorState().kinematics_estimated
        
        # Calculate reward
        position = KinematicsState.position
        position_array = np.array([position.x_val, position.y_val])
        new_quaternion = KinematicsState.orientation
        
        solved = self.compute_reward(position_array, new_quaternion)
                            
        return solved

    def calculate_distance(self, position_array, target_point):
        """Get distance between current state and goal state"""
        distance = np.linalg.norm(position_array - target_point, ord=2) # norm 2
        return distance

    # retrieve the estimated position and orientation of the multirotor.
    def current_position(self):
        position = self.client.getMultirotorState().kinematics_estimated.position
        orientation = self.client.getMultirotorState().kinematics_estimated.orientation
        position = np.array([position.x_val, position.y_val])
        return position, orientation

    # Get angle between quad position to goal position.
    def calculate_angle(self, position_array, quaternion, for_Mline=False):         
        euler = airsim.to_eularian_angles(quaternion) # Convert quaternion to Euler angles   
        yaw_angle = np.degrees(euler[2]) # Extract yaw angle from Euler angles
        
        # Calculate direction vector from current position to goal position
        direction = self.goal - position_array
        # Calculate angle between direction vector and yaw angle
        heading_angle = np.degrees(np.arctan2(direction[1], direction[0])) - yaw_angle
        
        # Wrap the angle within 0 to 360 degrees
        theta = heading_angle % 360
        if for_Mline:
            return theta
        else:
            if theta > 180:
                theta = 360 - theta
            return theta

    def compute_reward(self, position_array, quaternion):
        solved = False
        self.client.simGetCollisionInfo().has_collided
        # according to distance
        current_distance = self.calculate_distance(position_array, self.goal)
        self.previous_distance = current_distance
        
        # if goal is close
        if current_distance < 3:
            solved = True
            print("======== + Solved + ========")

        return solved


    def interpret_action(self, action):
        """Interprete action"""
        if action == 0:
            self.quad_offset = (self.scaling_factor, 0, 0)   # Move Forward (increase X)
        elif action == 1:
            self.quad_offset = (-self.scaling_factor, 0, 0)  # Move Backward (decrease X)
        elif action == 2:
            self.quad_offset = (0, self.scaling_factor, 0)   # Move Right (increase Y)
        elif action == 3:
            self.quad_offset = (0, -self.scaling_factor, 0)  # Move Left (decrease Y)
        elif action == 4:
            self.quad_offset = (0, 0, self.scaling_factor*15)   # Rotate to Right (+)
        elif action == 5:
            self.quad_offset = (0, 0, -self.scaling_factor*15)  # Rotate to left (-)
          

    def dist_sensors(self):
        distance_sensorData = {
            '0'     : self.client.getDistanceSensorData('Distance_0').distance, 
            '-30'   : self.client.getDistanceSensorData('Distance_M30').distance,
            '30'    : self.client.getDistanceSensorData('Distance_30').distance,
            }
        return np.array(list(distance_sensorData.values()))
    
    
    def all_distance_sensors(self):
        distance_dict = {
            '0'     : self.client.getDistanceSensorData('Distance_0').distance, 
            '-30'   : self.client.getDistanceSensorData('Distance_M30').distance,
            '-90'   : self.client.getDistanceSensorData('Distance_M90').distance,
            '30'    : self.client.getDistanceSensorData('Distance_30').distance,
            '90'   : self.client.getDistanceSensorData('Distance_90').distance,
            }
        return distance_dict
