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
        self.bond = bondpy.Bond("bond_topic", bondId, self.on_broken, self.on_formed)
        self.formed = False
        self.broken = False
        self.safetyActive = False
        self.fatalActive = False
        self.bondId = bondId

        self.bond.set_heartbeat_period(0.2)
        self.bond.set_heartbeat_timeout(0.5)

        rospy.Subscriber("safety", String, self.process_safety_message)

    def form_bond(self):
        self.bond.start()
        
        return self.wait_until_safe()

    def wait_until_safe(self):
        while not rospy.is_shutdown():
            if self.broken:
                return False

            if self.formed:
                return True
            rospy.sleep(0.1)

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

    def on_broken(self):
        self.broken = True
        self.formed = False

        self.safetyActive = True
        self.fatalActive = True

    def on_formed(self):
        self.broken = False
        self.formed = True
