#!/usr/bin/python
# -*- coding: utf-8 -*-



from math import pow, atan2, sqrt, cos, sin
import rospy
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from actionlib_msgs.msg import GoalStatusArray
import tf
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerRequest, SetBool, SetBoolRequest
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2

class drone_path_execution:
    def __init__(self):
        self.SetCmdTrajSub = rospy.Subscriber('m100/command/trajectory', MultiDOFJointTrajectory, self.update_trajectory, queue_size=100)
        #self.CmdVelSub = rospy.Subscriber('cmd_vel', Twist, self.update_lee_controller, queue_size=100)
        self.LocalOdomSub = rospy.Subscriber('m100/ground_truth/odometry_throttled', Odometry, self.update_local_pos, queue_size=100)  
        self.SubsampleSub = rospy.Subscriber('gbplanner_node/occupied_nodes', MarkerArray, self.subsample_map, queue_size=1)
        self.MapPub = rospy.Publisher('output_map', MarkerArray, queue_size=1)
        self.PointCloudSub = rospy.Subscriber('gbplanner_node/surface_pointcloud', PointCloud2, self.subsample_pointcloud, queue_size=1)
        self.PointCloudPub = rospy.Publisher('output_pointcloud', PointCloud2, queue_size=1)
        self.MoveBaseStatusSub = rospy.Subscriber('move_base/status', GoalStatusArray, self.goal_status_update, queue_size=100) 
        self.SetWaypointSub = rospy.Subscriber('waypoint', PointStamped, self.set_waypoint, queue_size=100)
        self.GoalPub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=100)
        self.PathCompleteService = rospy.ServiceProxy('drone_path_complete/std_srvs', Trigger)
        self.current_trajectory = MultiDOFJointTrajectory()
        self.current_trajectory_index = 0
        self.current_setpoint = PoseStamped()
        self.odom_pos = PoseStamped()
        self.r = rospy.Rate(30)
        self.goal_failure = False
        self.waypoint_mode = False
        self.previous_map = MarkerArray()
        self.previous_map.markers.append(Marker())
        self.previous_pointcloud = PointCloud2()
        # Parameters
        self.send_only_last_pose = rospy.get_param("~send_only_last_pose", True)
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.2)
        self.global_frame_id = rospy.get_param("~global_frame_id", "world")
        self.new_area_min = rospy.get_param("~min_area_update", 1000)
        print("Min area update : ", str(self.new_area_min))

    #def update_lee_controller(self,twist):
        #converting twist to multidoftrajectory to publish to lee position controller
    def subsample_pointcloud(self, data):
        
        if len(data.fields) > len(self.previous_pointcloud.fields) + self.new_area_min:
            self.PointCloudPub.publish(data)  
            self.previous_pointcloud = data

    def subsample_map(self, data):
        
        if len(data.markers[0].points) > len(self.previous_map.markers[0].points) + self.new_area_min:
            self.MapPub.publish(data)
            self.previous_map = data

    def update_local_pos(self, data):
        # Convert the odom_to pose stamped
        self.odom_pos=PoseStamped()
        self.odom_pos.header.stamp = data.header.stamp
        self.odom_pos.pose.position = data.pose.pose.position
        self.odom_pos.pose.orientation = data.pose.pose.orientation     
    
    def goal_status_update(self, status):
        #if status.status_list[0].status >= 4 and not self.goal_failure :
        #    self.goal_failure = True
        #    print("Impossible Goal, trigger high level replanning")
        #    PathComplete = TriggerRequest()
        #    self.PathCompleteService(PathComplete)
        #elif status.status_list[0].status < 4 and self.goal_failure :
        self.goal_failure = False

    def next_setpoint(self):
        # Pass to next point in the trajectory + Tranform to Pose converstion
        transform = self.current_trajectory.points[self.current_trajectory_index].transforms[0]
        self.current_setpoint = PoseStamped()
        self.current_setpoint.header.frame_id = self.global_frame_id
        self.current_setpoint.pose.position.x = transform.translation.x
        self.current_setpoint.pose.position.y = transform.translation.y
        self.current_setpoint.pose.position.z = self.odom_pos.pose.position.z
        self.current_setpoint.pose.orientation = transform.rotation
        self.current_trajectory_index = self.current_trajectory_index + 1
        self.GoalPub.publish(self.current_setpoint)
        print("Goto next point in trajectory: x=",self.current_setpoint.pose.position.x,", y=",self.current_setpoint.pose.position.y,", z=",self.current_setpoint.pose.position.z)
    
    def update_trajectory(self, trajectory):
        # Stores new trajectory and reinitialize the path execution
        self.current_trajectory = trajectory
        print("New trajectory: length=",len(self.current_trajectory.points))
        if self.send_only_last_pose:
            self.current_trajectory_index = len(self.current_trajectory.points)-1
        else:
            self.current_trajectory_index = 0 
        if (not self.waypoint_mode):
            self.next_setpoint()  

    def set_waypoint(self, waypoint):
        self.waypoint_mode = True
        self.current_setpoint = PoseStamped()
        self.current_setpoint.header.frame_id = self.global_frame_id
        self.current_setpoint.pose.position.x = waypoint.point.x
        self.current_setpoint.pose.position.y = waypoint.point.y
        self.current_setpoint.pose.position.z = self.odom_pos.pose.position.z
        self.current_setpoint.pose.orientation = self.odom_pos.pose.orientation
        self.GoalPub.publish(self.current_setpoint)
        print("Goto waypoint: x=",self.current_setpoint.pose.position.x,", y=",self.current_setpoint.pose.position.y,", z=",self.current_setpoint.pose.position.z)
    
    def distance_from_setpoint(self):
        # Euclidean distance between current pose and the goal.
        distance = sqrt(pow((self.current_setpoint.pose.position.x - self.odom_pos.pose.position.x), 2) + pow((self.current_setpoint.pose.position.y - self.odom_pos.pose.position.y), 2))
        return distance

    def runner(self): 
        while (not rospy.is_shutdown()) and (KeyboardInterrupt):
            
            if ((len(self.current_trajectory.points) != 0 or self.waypoint_mode) and self.distance_from_setpoint() <= self.distance_threshold):
                if self.waypoint_mode:
                    self.waypoint_mode = False
                    print("Waypoint reached!")
                    PathComplete = TriggerRequest()
                    self.PathCompleteService(PathComplete)
                    print("Compute new path")
                    self.current_trajectory = MultiDOFJointTrajectory()
                    self.current_trajectory_index = 0
                else:
                    if(self.current_trajectory_index >= len(self.current_trajectory.points)):    
                        PathComplete = TriggerRequest()
                        self.PathCompleteService(PathComplete)
                        print("Path complete service call")
                        self.current_trajectory = MultiDOFJointTrajectory()
                        self.current_trajectory_index = 0
                    else:
                        self.next_setpoint()                    

            self.r.sleep()


if __name__ == '__main__':
    try:
        # ROS Init
        rospy.init_node('drone_path_execution', anonymous=True, disable_signals=True)
        drone = drone_path_execution()
        drone.runner()
    except KeyboardInterrupt:
        print('drone_path_execution Shutting Down ...')


