import sys
import re
import math
import rospy
import cv2
from copy import deepcopy
import actionlib
from rosoclingo.msg import ROSoClingoOut, ROSoClingoIn, ROSoClingoAction, ROSoClingoGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.srv import *

import std_srvs.srv
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import numpy as np
from sensor_msgs.msg import Image,JointState
from cv_bridge import CvBridge, CvBridgeError

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from geometry_msgs.msg import Twist
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_interface_mapping.srv import *
from rosplan_interface_movebase.srv import *

class SenseBaseInterface(object):
    re_sense = re.compile(r'^\s*sense\(\s*([^\s]+)\s*\)')
    def __init__(self,roc_in_tp_name,roc_out_tp_name,timeout=60):
        self.pose_ = None
        self.out_message_ = None
        self.goal_name_ = None
        self.timeout_ = timeout
        self.target_name = None
        self.detect_result = False
        self.red_min = np.array([0, 60, 60])
        self.red_max = np.array([6, 255, 255])
        self.yaw = 0
        self.sub_odom_ = rospy.Subscriber("/odom", Odometry, self.get_rotation_cb_)
        self.angular_cmd_pub_ = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.sub_pose_ = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.robot_pose_cb_)
        rospy.loginfo("waiting for ROSoClingo sense_base dependencies")
        while not self.pose_:
            rospy.sleep(1.0)
        rospy.wait_for_service("/move_base/clear_costmaps")
        self.mb_cc_srv_ = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty)

        self.mb_ac_ = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        self.mb_ac_.wait_for_server()

        self.pub_roc_in_ = rospy.Publisher(roc_in_tp_name, ROSoClingoIn)
        self.sub_roc_out_ = rospy.Subscriber(roc_out_tp_name, ROSoClingoOut, self.rosoclingo_out_cb_)

        self.bridge = CvBridge()
        self.subArucoDetection = rospy.Subscriber("/aruco_single/pose", PoseStamped, self.aruco_pose_callback)
        self.mostPossibleAngle = 90
        self.mostPossibleViewpoint = None

        self.knowledge_base_srv_ = rospy.ServiceProxy("rosplan_knowledge_base/update",KnowledgeUpdateService)
        self.add_waypoint_srv_ = rospy.ServiceProxy("rosplan_roadmap_server/add_waypoint",AddWaypoint)
        self.knowledge_item = KnowledgeItem()

        self.get_container_state_srv = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        self.container_gazebo_model = GetModelStateRequest()
        self.container_gazebo_model.model_name = "aruco_cube_582"
        self.actionlib_client = actionlib.SimpleActionClient('/rosoclingo', ROSoClingoAction)
        self.actionlib_client.wait_for_server()
        rospy.wait_for_service("rosplan_knowledge_base/update")
        rospy.loginfo("ROSoClingo sense_base configured")

    def aruco_pose_callback(self, message):
        self.detect_result = True
        self.mostPossibleAngle = 90
        self.mostPossibleViewpoint = message

    def robot_pose_cb_(self,message):
        self.pose_frame_id_ = message.header.frame_id
        self.pose_ = message.pose.pose

    def get_rotation_cb_(self,message):
        roll = pitch = yaw = 0
        orientation_q = message.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll,pitch,self.yaw) = euler_from_quaternion(orientation_list)

    def move_base_rotate(self, target_angle):
        # euler_angles = (math.pi/2, math.pi, 3*math.pi, 0)
        print("target_angle")
        print(target_angle)
        euler_angle = target_angle * math.pi / 180
        print("euler_angle")
        print(euler_angle)
        q_angle = quaternion_from_euler(0, 0, euler_angle, axes = 'sxyz')
        print("q_angle")
        print(q_angle)
        q = Quaternion(*q_angle)
        print("q")
        print(q)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = self.pose_.position
        goal.target_pose.pose.orientation = q
        self.mb_ac_.send_goal(goal)
        finished_within_time = self.mb_ac_.wait_for_result(rospy.Duration(60))
        if not finished_within_time:
            self.mb_ac_.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.mb_ac_.get_state()
            rospy.loginfo("print goal state")
            rospy.loginfo(state)
            if state == 3:
                rospy.loginfo("successfully reached the goal orientation")
                self.out_message_.value = "success"
            else:
                rospy.loginfo("failed to reach the goal orientation")
                self.out_message_.value = "failure"
            self.pub_roc_in_.publish(self.out_message_)
            self.out_message_ = None
        if self.detect_result:
            rospy.loginfo("however, found the target")
            self.out_message_.value = "success"
        else:
            rospy.loginfo("and not found the target")

    # observe(angle1): if false: then observe(angle2); if true: then transition(v1)
    def rotate_action(self,target_angle):
        target_rad = target_angle * math.pi / 180
        print("target_rad")
        print(target_rad)
        delta_rad = target_rad - self.yaw
        print("self.yaw")
        print(self.yaw)
        print("delta_rad")
        print(delta_rad)
        command = Twist()
        print("self.result")
        print(self.detect_result)
        self.detect_result = False
        while abs(delta_rad) > 0.1 and self.detect_result == False:
            command.angular.z = 0.5 * delta_rad
            self.angular_cmd_pub_.publish(command)
            delta_rad = target_rad - self.yaw
        command.angular.z = 0.0
        self.angular_cmd_pub_.publish(command)
        if self.detect_result == True:
            self.out_message_.value = "success"
            self.object_state = self.get_container_state_srv(self.container_gazebo_model)
            self.object_position = (self.object_state.pose.position.x, self.object_state.pose.position.y + 0.5, self.object_state.pose.position.z)
            self.object_orientation = (self.object_state.pose.orientation.x, self.object_state.pose.orientation.y,
                                       self.object_state.pose.orientation.z, self.object_state.pose.orientation.w)
            print(self.object_position)
            print(self.object_orientation)
        elif self.detect_result == False:
            rospy.loginfo("sense action failed")
            self.out_message_.value = "failure"
        self.pub_roc_in_.publish(self.out_message_)
        self.out_message_ = None

    def rotate_tiago_head_action(self, rotate_angle):
        self.detect_result = False
        try:
            rospy.wait_for_service("rotate_head_to_a1")
            self.rotate_tiago_head_client = rospy.ServiceProxy("rotate_head_to_a1", Empty)
            if self.detect_result == True:
                self.out_message_.value = "success"
                print(self.object_position)
                print(self.object_orientation)
            elif self.detect_result == False:
                rospy.loginfo("sense action failed")
                self.out_message_.value = "failure"
            self.pub_roc_in_.publish(self.out_message_)
            self.out_message_ = None
        except:
            rospy.logwarn("call rotate_head_to_a1 service failed")

    def transition_tiago_action(self, position_msg):
        self.detect_result = False
        mbg = MoveBaseGoal()
        mbg.target_pose.header.frame_id = 'map'
        mbg.target_pose.header.stamp = rospy.Time.now()
        mbg.target_pose.pose = self.pose_
        mbg.target_pose.pose.position.x = position_msg[0]
        mbg.target_pose.pose.position.y = position_msg[1]
        print(mbg)
        self.mb_ac_.send_goal(mbg)
        finished_within_time = self.mb_ac_.wait_for_result(rospy.Duration(60))
        if not finished_within_time:
            self.mb_ac_.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.mb_ac_.get_state()
            rospy.loginfo("print goal state")
            rospy.loginfo(state)
            if state == 3:
                rospy.loginfo("successfully reached the goal position")
            else:
                rospy.loginfo("failed to reach the goal position")
        if self.detect_result == True:
            rospy.loginfo("found the target")
            self.out_message_.value = "success"
        else:
            rospy.loginfo("not found the target")
            self.out_message_.value = "failure"
        self.pub_roc_in_.publish(self.out_message_)
        self.out_message_ = None
        self.goal_name_ = None

    def rosoclingo_out_cb_(self,message):
        self.out_message_ = ROSoClingoIn()
        self.out_message_.id = message.id
        rospy.loginfo("sense_base_interface receives: ")
        rospy.loginfo(message.action)
        str_arr = message.action.split(",") #senseTransition/Rotation(ID,L,V,W)
        self.target_name = str_arr[1]
        angle_or_vp = str_arr[2]
        print("angle_or_wp:")
        print(angle_or_vp)
        if "senseRotation" in message.action:
            rospy.loginfo("rotate start")
            if angle_or_vp == "a1":
                rospy.loginfo("rotate the head to a1")
                self.rotate_tiago_head_action(angle_or_vp) # move head to right
        elif "senseTransition" in message.action:
            rospy.loginfo("transition start")
            if angle_or_vp == "v0":
                rospy.loginfo("transition to v0")
                v1_position = [1.4153478771,-0.794749303837,-0.00159685552558]
                self.transition_tiago_action(v1_position) # move to the middle of the table
            elif angle_or_vp == "v1":
                rospy.loginfo("transition to v1")
                v2_position = [2.61212333491,-0.0022436156019,-0.00157654947018]
                self.transition_tiago_action(v2_position) # move to the right end of the table
                self.out_message_.value = "success"
                self.pub_roc_in_.publish(self.out_message_)
                self.out_message_ = None
                self.goal_name_ = None
        else:
            rospy.loginfo("ignoring the invalid sensing action")
        if self.out_message_:
            self.mb_ac_.cancel_goal()

    def send_to_pose_(self, position_msg, orientation_msg):
        mbg = MoveBaseGoal()
        mbg.target_pose.header.frame_id = 'map'
        mbg.target_pose.header.stamp = rospy.Time.now()
        mbg.target_pose.pose = self.pose_
        mbg.target_pose.pose.position.x = position_msg[0]
        mbg.target_pose.pose.position.y = position_msg[1]
        print(mbg)
        self.mb_ac_.send_goal(mbg)
        finished_within_time = self.mb_ac_.wait_for_result(rospy.Duration(60))
        if not finished_within_time:
            self.mb_ac_.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.mb_ac_.get_state()
            rospy.loginfo("print goal state")
            rospy.loginfo(state)
            if state == 3:
                rospy.loginfo("successfully reached the goal position")
                self.out_message_.value = "success"
            else:
                rospy.loginfo("failed to reach the goal position")
                self.out_message_.value = "failure"
                if self.detect_result:
                    rospy.loginfo("however, found the target")
                    self.out_message_.value = "success"
                else:
                    rospy.loginfo("and not found the target")
        self.pub_roc_in_.publish(self.out_message_)
        self.out_message_ = None
        self.goal_name_ = None







