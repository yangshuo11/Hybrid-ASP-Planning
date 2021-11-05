#!/usr/bin/env python
#----------------------------
# simulated sensing behavior to sense the dynamic target
#----------------------------

import rospy
from rosoclingo.msg import ROSoClingoOut, ROSoClingoIn, ROSoClingoAction, ROSoClingoGoal
from rosoclingo.srv import *
from tiago_sense_base_interface import SenseBaseInterface
import xml.etree.ElementTree as ET

topic = rospy.Publisher("/rosoclingo/in",ROSoClingoIn)
get_information = rospy.ServiceProxy(rospy.resolve_name("rosoclingo") + "/get_goal_information", GetGoalInformation)

# def generatename2waypoints(file):
#     output = {}
#     tree = ET.parse(file)
#     root = tree.getroot()
#     for child in root:
#         output[child.attrib["name"]] = (float(child.attrib["xcoord"]),float(child.attrib["ycoord"]))
#     return output
#
# def generatename2angles(file):
#     angle_output = {}
#     tree = ET.parse(file)
#     root = tree.getroot()
#     for child in root:
#         angle_output[child.attrib["name"]] = float(child.attrib["xcoord"])
#     return angle_output
#
# def generatename2vps(file):
#     vps_output = {}
#     tree = ET.parse(file)
#     root = tree.getroot()
#     for child in root:
#         vps_output[child.attrib["name"]] = (float(child.attrib["xcoord"]),float(child.attrib["ycoord"]))
#     return vps_output

# def rosoclingoout_callback(message):
    # print("message received: ")
    # print(message.action)
    # if "senseRotation" in message.action: #senseRotation(1,l0,a1,o0)
    #     str = message.action.split("(")[1] #l0,o0)
    #     object = str.split(",")[0] #l0
    #     print(object)
    #     location = str.split(",")[1]
    #     location = location.split(")")[0] #o0
    #     print(location)
    #     out_message = ROSoClingoIn()
    #     out_message.id = message.id
    #     out_message.value = "failure"
    #     rospy.loginfo("Robot failed to find the target: " + object)
    #     topic.publish(out_message)
    #
    # elif "senseTransition" in message.action:  #senseTransition(1,l0,v2,o0)
    #     out_message = ROSoClingoIn()
    #     out_message.id = message.id
    #     out_message.value = "success"
    #     rospy.loginfo("Robot successfully find the target")
    #     topic.publish(out_message)
    # else:
    #     rospy.loginfo("not matched sense behavior")

if __name__ == '__main__':
    try:
        rospy.init_node("tiago_sense_action_interface")
        # name2waypoint = generatename2waypoints(sys.argv[1])
        # name2angles = generatename2angles(sys.argv[2])
        # name2vps = generatename2vps(sys.argv[3])
        # sb = SenseBaseInterface("/rosoclingo/in","/rosoclingo/out",name2waypoint,name2angles,name2vps)
        sb = SenseBaseInterface("/rosoclingo/in","/rosoclingo/out")
        # rospy.Subscriber("/rosoclingo/out",ROSoClingoOut,rosoclingoout_callback)
        rospy.spin()
    except SystemExit:
        print "system exception"
    except rospy.ROSInterruptException:
        print "ROSInterrupt exception"
    except:
        print "unexpected exception"
        raise