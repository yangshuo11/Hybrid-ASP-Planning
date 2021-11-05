#!/usr/bin/env python

import sys
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *

query = []

def call_service():
    print "Waiting for service"
    rospy.wait_for_service('/rosplan_knowledge_base/query_state')
    try:
        print "Calling Service"
        query_proxy = rospy.ServiceProxy('rosplan_knowledge_base/query_state', KnowledgeQueryService)
        resp1 = query_proxy(query)
        print "Response is:", resp1.results
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    # QUERY 1 (robot_at kenny wp0)
    query1 = KnowledgeItem()
    query1.knowledge_type = KnowledgeItem.FACT
    query1.attribute_name = "visited"
    # query1.values.append(diagnostic_msgs.msg.KeyValue("v", "kenny"))
    query1.values.append(diagnostic_msgs.msg.KeyValue("wp", "wp1"))
    query.append(query1)

    call_service()
    sys.exit(1)
