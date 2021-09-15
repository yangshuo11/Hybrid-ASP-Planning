#------------------------------------------------------------------------
# A class to handle ROSoClingo's interface to move_base
# Inputs:
# - rosoclingo_in topic_name
# - rosoclingo_out topic name
# - a map of waypoint names to (x,y) coordinate pairs
#------------------------------------------------------------------------
import re
import math
import rospy
import tf
from copy import deepcopy
import actionlib
from rosoclingo.msg import ROSoClingoOut, ROSoClingoIn
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
import std_srvs.srv 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MoveBaseInterface(object):
    re_GO = re.compile(r'^\s*go\(\s*([^\s]+)\s*\)')

    def __init__(self, roc_in_tp_name, roc_out_tp_name, location_map, timeout=60):
        self.location_map_ = location_map
        self.pose_ = None
        self.out_message_ = None  # The rosoclingo_out message
        self.goal_name_ = None
        self.timeout_ = timeout
        self.mbtimer_ = None
        self.sub_pose_ = rospy.Subscriber('/amcl_pose', #/amcl_pose
                                          PoseWithCovarianceStamped, #PoseWithCovarianceStamped
                                          self.pose_cb_)

        # Wait for the various dependencies to be setup
        rospy.loginfo("Waiting for ROSoClingo move_base dependencies")
        while not self.pose_:
            rospy.sleep(1.0)
#        rospy.wait_for_service('/move_base/make_plan')
#        self.mb_mp_srv_ = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

        rospy.wait_for_service('/move_base/clear_costmaps')
        self.mb_cc_srv_ = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty)
        
        self.mb_ac_ = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.mb_ac_.wait_for_server()

#        rospy.loginfo("Robot currently at {0}".format(self.pose_))

        # Now setup the link to rosoclingo
        self.pub_roc_in_ = rospy.Publisher(roc_in_tp_name, ROSoClingoIn)
        self.sub_roc_out_ = rospy.Subscriber(roc_out_tp_name,
                                             ROSoClingoOut,
                                             self.rosoclingo_out_cb_)
        rospy.loginfo("ROSoClingo move_base configured")

    #------------------------------------------------------------------
    # Callback to track the current pose
    #------------------------------------------------------------------
    def pose_cb_(self, message):
        self.pose_frame_id_ = message.header.frame_id
        self.pose_ = message.pose.pose
        # rospy.loginfo("current pose: {0}".format(self.pose_))

    #------------------------------------------------------------------
    # Callback function for messages coming from rosoclingo 
    # (rosoclingo_out) to tell the robot to move to a given location.
    #
    # For the move_base controller we only want to respond to "go(X)" 
    # messages. Other messages can be safely ignored.
    #------------------------------------------------------------------
    def rosoclingo_out_cb_(self, message):
        matched = MoveBaseInterface.re_GO.match(message.action)
        if not matched:
           rospy.logdebug("Ignoring: {0}".format(message.action))
           return
        goto = matched.group(1)
        if goto not in self.location_map_:
            failed = ROSoClingoIn()
            failed.id = message.id
            failed.value = "failure"
            rospy.logerr("Unknown location: {0}".format(goto))
            self.pub_roc_in_.publish(failed)
            return

        # If there is a previous goal then cancel it
        if self.out_message_:
            self.mb_ac_.cancel_goal()
        self.mbtimer_shutdown_() # Shutdown any timers

        # Construct the return message (but don't send it yet)
        self.out_message_ = ROSoClingoIn()
        self.out_message_.id = message.id
        self.out_message_.value = "success"

        # Send the robot to the location        
        self.send_to_pose_(goto)

        # Now setup the failure timeout 
        self.mbtimer_ = rospy.Timer(rospy.Duration(self.timeout_), 
                                    self.mbtimer_cb_, True)

    #------------------------------------------------------------------
    # ROS timer callback which indicates that the robot has failed to
    # arrive at its location within a timeout period.
    #------------------------------------------------------------------
    def mbtimer_cb_(self, event):
        self.mbtimer_ = None
        if not self.out_message_:
            return   # Must have almost timed out

        rospy.loginfo(("Robot has failed to arrive at {0} within: "
                       "{1} seconds").format(self.goal_name_, self.timeout_))

        # Robot has failed so need to report this to rosoclingo.
        # Remove self.out_message to avoid problems when
        # cancelling the action.
        self.out_message_.value = "failure"
        tmp = self.out_message_
        self.out_message_ = None
        self.goal_name_ =  None

        # Need to clear the current action but also want to clear the
        # costmap so it doesn't get corrupted by the last failure. 
        # The blockage will still be saved at the higher level.
        self.mb_ac_.cancel_goal()
        self.mb_cc_srv_()

        rospy.loginfo("Sending navigation failure message")
        self.pub_roc_in_.publish(tmp)


        

    #------------------------------------------------------------------
    # move_base actionlib callbacks when a goal is sent
    #------------------------------------------------------------------
    # def mb_done_cb_(self, terminal_state, result):
    #     # If we've already responded then do nothing
    #     rospy.loginfo("enter in mb_done_cb")
    #     rospy.loginfo("terminal_state = {0}".format(terminal_state))
    #     if not self.out_message_:
    #         return
    #     self.mbtimer_shutdown_() # kill the timer
    #     # Success
    #     if terminal_state == 4:
    #         rospy.loginfo("Robot arrived at: {0}".format(self.goal_name_))
    #         self.out_message_.value = "success"
    #     else:
    #         rospy.loginfo(("Robot failed to arrive at: {0}. Error "
    #                        "state: {1}").format(self.goal_name_, terminal_state))
    #         self.out_message_.value = "failure"
    #
    #     # Publish and clean up
    #     self.pub_roc_in_.publish(self.out_message_)
    #     self.out_message_ = None
    #     self.goal_name_ =  None
    #
    # def mb_active_cb_(self):
    #     pass
    #
    # def mb_feedback_cb_(self, feedback):
    #     # If we've already responded then do nothing
    #     if not self.out_message_: return
    #
    #     curr = feedback.base_position.pose
    #     rospy.loginfo("robot curr at: {0}".format(curr))
    #     rospy.loginfo("goal_pose at: {0}".format(self.goal_pose_))
    #     if get_distance_(curr, self.goal_pose_) < 0.3:
    #
    #         # Stop the action and kill the timer
    #         self.mb_ac_.cancel_goal()
    #         self.mbtimer_shutdown_()
    #
    #         rospy.loginfo("Robot arriving at: {0}".format(self.goal_name_))
    #
    #         # Send a success message back to ROSoClingo
    #         self.pub_roc_in_.publish(self.out_message_)
    #         self.out_message_ = None
    #         self.goal_name_ = None



    #------------------------------------------------------------------
    # Internal helper functions
    #------------------------------------------------------------------


    #------------------------------------------------------------------
    # From a list of way points search backwards to find the 
    # orientation to use for a goal position. Doing this to try because
    # we do not specify an orientation and want to use an orientation
    # that is in the direction being travelled by the robot.
    #------------------------------------------------------------------
    def get_best_orientation_(self, waypoints):
        if not waypoints: 
            raise ValueError("Waypoints must not be empty")
        last = waypoints[-1]
        for curr in reversed(waypoints):
            if get_distance_(curr.pose, last.pose) > 0.5:
                diff_x = last.pose.position.x - curr.pose.position.x
                diff_y = last.pose.position.y - curr.pose.position.y
                angle = math.atan2(diff_y, diff_x)
#                print "ANGLE: {0}\n".format(3.14/180*angle)
                orient = Quaternion()
                orient.x = 0
                orient.y = 0
                orient.z = 1
                orient.w = math.cos(angle)
#                print "CURR: {0}\n, LAST: {1}\n".format(curr,last)
#                print "Using orientation:\n{0}\n".format(orient)
                return orient
#                print "Using pose:\n{0}\n instead of\n{1}\n".format(
#                curr.pose.orientation,
#                last.pose.orientation)
#                return last.pose.orientation
        return last.pose.orientation
            

    #------------------------------------------------------------------
    # Internal function to compute a goal pose when given the goal 
    # location name. NOTE: goal location must be valid
    #------------------------------------------------------------------
    def get_goal_pose_(self, goal_name):
        # Because we are not given an orientation we want to look at the
        # poses of the waypoints leading up to the goal pose to work out
        # an orientation that should make sense.

        sp = PoseStamped()
        sp.header.frame_id = self.pose_frame_id_
        sp.header.stamp = rospy.Time.now()
        sp.pose = self.pose_

        (goto_x, goto_y) = self.location_map_[goal_name]
        gp = PoseStamped()
        gp.header.frame_id = self.pose_frame_id_
        gp.header.stamp = rospy.Time.now()
        gp.pose = deepcopy(self.pose_)
        gp.pose.position.x = goto_x
        gp.pose.position.y = goto_y
        return gp

        ###NOTE: IGNORING ALL THE STUFF ABOUT TRYING TO BE SMART
        ###      ABOUT COMPUTING A BETTER GOAL ORIENTATION
        try:
            result = self.mb_mp_srv_(sp, gp, 0.4)
#            print "RESULT: {0}".format(result)
            plan = result.plan
            num = len(plan.poses) 
            if not plan.poses:
                rospy.logwarn("Plan service returned empty plan")
            else:
                gp.pose.orientation = self.get_best_orientation_(plan.poses)
        except rospy.ServiceException, e:
          rospy.logerr("Service call failed: {0}".format(e))

        return gp

    #------------------------------------------------------------------
    # Internal function to send the robot to some named location
    # NOTE: goal location must be valid
    #------------------------------------------------------------------
    def send_to_pose_(self, goal_name):
        # Work out the goal pose
        self.goal_name_ = goal_name
        gp = self.get_goal_pose_(self.goal_name_)
        rospy.loginfo("goal pose: {0}".format(gp))
        self.goal_pose_ = gp.pose

        # Now send the robot to the location using the move_base actionlib
        rospy.loginfo("Sending the robot to: {0}".format(self.goal_name_))

        mbg = MoveBaseGoal()
        mbg.target_pose.header.frame_id = gp.header.frame_id
        mbg.target_pose.header.stamp = rospy.Time.now()
        mbg.target_pose.pose = gp.pose
        
        self.mb_ac_.send_goal(mbg)
        finished_within_time = self.mb_ac_.wait_for_result(rospy.Duration(10))
        if not finished_within_time:
            self.mb_ac_.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.mb_ac_.get_state()
            if state == 3: # 0: PENDING; 1: ACTIVE; 2: PREEMPTED; 3: SUCCEEDED; 4: ABORTED; 5: REJCTED; 6: PREEMPTING
                rospy.loginfo("Goal succeeded!")
                self.out_message_.value = "success"
                self.mbtimer_shutdown_() # kill the timer
            else:
                rospy.loginfo("Goal unsucceeded!")
        # Publish and clean up
        self.pub_roc_in_.publish(self.out_message_)
        self.out_message_ = None
        self.goal_name_ =  None


    # self.mb_ac_.send_goal(mbg, self.mb_done_cb_,
        #                       self.mb_active_cb_, self.mb_feedback_cb_)

        # self.mb_ac_.wait_for_result()
#        print "Sending the robot to:\n{0}\n\n{1}\n\n".format(goal_name, gp)

    #------------------------------------------------------------------
    # Shutdown the timer if it is running
    #------------------------------------------------------------------
    def mbtimer_shutdown_(self):
        if self.mbtimer_: 
#            rospy.logdebug("Shutting down timer")
            self.mbtimer_.shutdown()
            self.mbtimer_ = None
                                        

#------------------------------------------------------------------
# Helper functions
#------------------------------------------------------------------

#------------------------------------------------------------------
# Get the distance between two Pose object (looking only at the
# difference of positions)
#------------------------------------------------------------------
def get_distance_(pose1, pose2):
    posn1 = pose1.position
    posn2 = pose2.position
    diff_x = abs(posn1.x - posn2.x)
    diff_y = abs(posn1.y - posn2.y)
#        diff_z = abs(posn1.z - posn2.z)
    diff = diff_x*diff_x + diff_y*diff_y
    return math.sqrt(diff)

