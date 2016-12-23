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
        self.bond = bondpy.Bond(bondId)
        self.safetyActive = False
        self.fatalActive = False
        self.bondId = bondId

    def form_bond(self):
        bond.start()
        if(bond.wait_until_formed(rospy.Duration(5.0))):
            return False

        rospy.Subscriber("safety", String, processSafetyMessage)

        return True

    def process_safety_message(self, message):
        
        if(message.data == bondId):
            safetyActive = True

        if(message.data == "FATAL"):
            fatalActive = True
            safetyActive = True

    def is_safety_active(self):
        return self.safetyActive

    def is_fatal_active(self):
        return self.fatalActive
