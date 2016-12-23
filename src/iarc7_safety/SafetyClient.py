#!/usr/bin/env python

###########################################################################
#
# SafetyClient
#
# Class implements an easy way to use the node_monitor to notify of safety events
#
###########################################################################

import rospy
from bondpy import bondpy
from std_msgs.msg import String

class SafetyClient:
    def __init__(self, bondId):
        self.bond = bondpy.Bond("bond_topic", bondId)
        self.safetyActive = False
        self.fatalActive = False
        self.bondId = bondId

    def form_bond(self):
        self.bond.start()
        if self.bond.wait_until_formed(rospy.Duration(5.0)) == False:
            return False

        rospy.Subscriber("safety", String, self.process_safety_message)

        return True

    def process_safety_message(self, message):
        
        if(message.data == self.bondId):
            self.safetyActive = True

        if(message.data == "FATAL"):
            self.fatalActive = True
            self.safetyActive = True

    def is_safety_active(self):
        return self.safetyActive

    def is_fatal_active(self):
        return self.fatalActive
