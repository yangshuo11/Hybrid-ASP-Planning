#!/usr/bin/env python
import roslib; roslib.load_manifest('rosoclingo')
import rospy
import sys
from rosoClingo import ROSoClingo

if __name__ == '__main__':
    rospy.init_node('rosoclingo',argv=sys.argv)
    rosoclingo = ROSoClingo(rospy.myargv(),[])
    rosoclingo.run()
