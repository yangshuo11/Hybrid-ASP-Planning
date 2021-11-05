#! /usr/bin/env python
#---------------------------------------------------------------------
# ROSoClingo interface to control ROS move_base configured for the
# Willowgarage map. 
#---------------------------------------------------------------------

import sys
import roslib; roslib.load_manifest('rosoclingo_interfaces')
import rospy
from rosoclingo.msg import ROSoClingoOut, ROSoClingoIn
from move_base_interface import MoveBaseInterface
import xml.etree.ElementTree as ET

def generatename2waypoints(file):
    output = {}
    tree = ET.parse(file)
    root = tree.getroot()
    for child in root:
        output[child.attrib["name"]] = (float(child.attrib["xcoord"]),float(child.attrib["ycoord"]))
    return output

#---------------------------------------------------------------------
# Main
#---------------------------------------------------------------------
if __name__ == '__main__':
    try:
        rospy.init_node('goto_action_interface')
        name2waypoint = generatename2waypoints(sys.argv[1])
        print name2waypoint
        mb = MoveBaseInterface('/rosoclingo/in', "/rosoclingo/out", name2waypoint)
        rospy.spin()
    except SystemExit:
        print "System exception"
    except rospy.ROSInterruptException:
        print "ROSInterrupt Exception"
    except:
        print "Unexpected exception: ", sys.exc_info()[0]
        raise
