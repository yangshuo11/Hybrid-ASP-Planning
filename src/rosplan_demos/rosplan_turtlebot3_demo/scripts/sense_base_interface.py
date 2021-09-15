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
    def __init__(self,roc_in_tp_name,roc_out_tp_name,wp_map,angle_map,vps_map,timeout=60):
        self.wp_map_ = wp_map
        self.angle_map_ = angle_map
        self.vps_map_ = vps_map
        self.pose_ = None
        self.goal_position_ = None
        self.out_message_ = None
        self.goal_name_ = None
        self.timeout_ = timeout
        # self.mbtimer_ = None
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
        # self.subImage = rospy.Subscriber("/camera/rgb/image_raw", Image, self.img_process_callback)
        self.subArucoDetection = rospy.Subscriber("/aruco_single/pose", PoseStamped, self.aruco_pose_callback)
        self.mostPossibleAngle = 90
        self.mostPossibleViewpoint = None

        # self.aruco_pose_client = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

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

    # def img_process_callback(self, ros_image_message):
    #     try:
    #         frame = self.bridge.imgmsg_to_cv2(ros_image_message,"bgr8")
    #         self.real_time_image = np.array(frame,dtype=np.uint8)
    #         self.detect_result = self.image_process(self.real_time_image,self.target_name)
    #         # if self.detect_result: # if detect the target,stop the movement process
    #             # self.mb_ac_.cancel_goal()
    #     except CvBridgeError:
    #         rospy.loginfo("CvBridgeError")

    # def image_process(self, image, target_name):
    #     detect_result = False
    #     if image is not None:
    #         image = cv2.resize(image, (640,360), interpolation = cv2.INTER_CUBIC)
    #         img_h, img_w = image.shape[:2]
    #         img_center_x = img_w / 2
    #         img_center_y = img_h / 2
    #         gs_frame = cv2.GaussianBlur(image, (5,5), 0)
    #         hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
    #         mask = cv2.inRange(hsv, self.red_min, self.red_max)
    #         mask = cv2.erode(mask, None, iterations = 2)
    #         kernel = np.ones((5,5), np.uint8)
    #         mask = cv2.dilate(mask, kernel, iterations = 2)
    #         cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    #         if len(cnts) > 0:
    #             c = max(cnts,key=cv2.contourArea)
    #             rect = cv2.minAreaRect(c)
    #             box = cv2.boxPoints(rect)
    #             # cv2.drawContours(image, [np.int0(box)], -1, (0,255,255), 2)
    #             c_x, c_y = rect[0]
    #             h, w = rect[1]
    #             c_angle = rect[2]
    #             if h * w >= 2000:
    #                 print("detect the target")
    #                 detect_result = True
    #             # cv2.circle(image, (int(c_x), int(c_y)), 3, (216, 0, 255), -1)
    #             # cv2.putText(image, "target: " + str(h * w),
    #             #                 (int(c_x), int(c_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)#(0, 0, 255)BGR
    #             cv2.imshow("Image", image)
    #             cv2.waitKey(1)
    #     return detect_result

    def aruco_pose_callback(self, message):
        # rospy.loginfo("aruco_pose_callback: detected the aruco")
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
            # local_out_message_ = ROSoClingoIn()
            # local_out_message_.value = "failure"
            self.out_message_.value = "failure"
            # goal = ROSoClingoGoal()
            #goal.request = "failed(red,a1)"
            #self.actionlib_client.send_goal(goal)
        self.pub_roc_in_.publish(self.out_message_)
        self.out_message_ = None

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
            # self.rotate_action(self.mostPossibleAngle)
            # self.move_base_rotate(self.mostPossibleAngle)
            if angle_or_vp in self.angle_map_:
                angle_value = self.angle_map_[angle_or_vp]
                rospy.loginfo("rotate start")
                self.rotate_action(angle_value)
        elif "senseTransition" in message.action:
            self.send_to_pose_(self.object_position, self.object_orientation)
            # self.send_to_pose_(self.mostPossibleViewpoint)
            # self.send_to_pose_(angle_or_vp)
        else:
            rospy.loginfo("ignoring the invalid sensing action")
        if self.out_message_:
            self.mb_ac_.cancel_goal()


    def get_best_orientation_(self,waypoints):
        if not waypoints:
            raise ValueError("waypoints must not be empty")
        last = waypoints[-1]
        for curr in reversed(waypoints):
            if get_distance_(curr.pose, last.pose) > 0.5:
                diff_x = last.pose.position.x - curr.pose.position.x
                diff_y = last.pose.position.y - curr.pose.position.y
                angle = math.atan2(diff_y, diff_x)
                orient = Quaternion()
                orient.x = 0
                orient.y = 0
                orient.z = 1
                orient.w = math.cos(angle)
                return orient
        return last.pose.orientation

    def get_goal_pose_(self,goal_name):
        sp = PoseStamped()
        sp.header.frame_id = self.pose_frame_id_
        sp.header.stamp = rospy.Time.now()
        sp.pose = self.pose_

        (vps_x,vps_y) = self.vps_map_[goal_name]
        gp = PoseStamped()
        gp.header.frame_id = self.pose_frame_id_
        gp.header.stamp = rospy.Time.now()
        gp.pose = deepcopy(self.pose_)
        gp.pose.position.x = vps_x
        gp.pose.position.y = vps_y
        return gp

    def add_real_time_waypoint(self,waypoint):
        try:
            print(waypoint)
            self.add_waypoint_srv_("wp10",waypoint,8.0)
            print("adding real-time waypoint result:")
        except rospy.ServiceException as e:
            print("service call failed")

    def send_to_pose_(self, position_msg, orientation_msg):
        # rospy.loginfo("goal pose: {0}".format(pose_msg))
        # self.goal_pose = pose_msg
        # rospy.loginfo("goal location: ")
        # rospy.loginfo(self.goal_pose)
        mbg = MoveBaseGoal()
        # mbg.target_pose.header.frame_id = self.goal_pose.header.frame_id
        # mbg.target_pose.header.frame_id = 'map'
        mbg.target_pose.header.frame_id = 'map'
        mbg.target_pose.header.stamp = rospy.Time.now()
        mbg.target_pose.pose = self.pose_
        # mbg.target_pose.pose = self.object_state.pose
        # mbg.target_pose.pose.position = self.goal_pose.pose.position
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

    # def send_to_pose_(self,goal_name):
    #     self.goal_name_ = goal_name
    #     gp = self.get_goal_pose_(self.goal_name_)
    #
    #     rospy.loginfo("goal pose: {0}".format(gp))
    #     self.goal_pose_ = gp.pose
    #
    #     rospy.loginfo("sending the robot to: {0}".format(self.goal_name_))
    #     rospy.loginfo("goal location: ")
    #     rospy.loginfo(self.goal_pose_)
    #
    #     mbg = MoveBaseGoal()
    #     mbg.target_pose.header.frame_id = gp.header.frame_id
    #     mbg.target_pose.header.stamp = rospy.Time.now()
    #     mbg.target_pose.pose = gp.pose
    #
    #     self.mb_ac_.send_goal(mbg)
    #     finished_within_time = self.mb_ac_.wait_for_result(rospy.Duration(60))
    #     if not finished_within_time:
    #         self.mb_ac_.cancel_goal()
    #         rospy.loginfo("Timed out achieving goal")
    #     else:
    #         state = self.mb_ac_.get_state()
    #         rospy.loginfo("print goal state")
    #         rospy.loginfo(state)
    #         if state == 3:
    #             rospy.loginfo("successfully reached the goal position")
    #             self.out_message_.value = "success"
    #             # if self.detect_result:
    #             #     rospy.loginfo("successfully found the target")
    #             #     self.out_message_.value = "success"
    #             # else:
    #             #     self.out_message_.value = "failure"
    #             #     self.add_real_time_waypoint(gp)
    #
    #                 # self.knowledge_item.knowledge_type = KnowledgeItem.FACT
    #                 # self.attribute_name = "target_at"
    #                 # self.value.append(diagnostic_msgs.msg.KeyValue("o","red"))
    #                 # self.value.append(diagnostic_msgs.msg.KeyValue("wp","o0"))
    #                 # try:
    #                 #     print("updating knowlegde base")
    #                 #     res = knowledge_base_srv_(1,self.knowledge_item)
    #                 #     print("response is: ")
    #                 #     print(res)
    #                 # except:
    #                 #     print("service call failed")
    #             # self.mbtimer_.shutdown_()
    #         else:
    #             rospy.loginfo("failed to reach the goal position")
    #             self.out_message_.value = "failure"
    #             if self.detect_result:
    #                 rospy.loginfo("however, found the target")
    #                 self.out_message_.value = "success"
    #             else:
    #                 rospy.loginfo("and not found the target")
    #     self.pub_roc_in_.publish(self.out_message_)
    #     self.out_message_ = None
    #     self.goal_name_ = None

    # def mbtimer_shutdown_(self):
    #     if self.mbtimer_:
    #         self.mbtimer_.shutdown()
    #         self.mbtimer_ = None

def get_distance_(pose1,pose2):
    posn1 = pose1.position
    posn2 = pose2.position
    diff_x = abs(posn1.x - posn2.x)
    diff_y = abs(posn1.y - posn2.y)
    diff = diff_x * diff_x + diff_y * diff_y
    return math.sqrt(diff)






