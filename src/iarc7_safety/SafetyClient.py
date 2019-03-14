#!/usr/bin/env python

###########################################################################
#
# SafetyClient
#
# Class implements an easy way to use the node_monitor to notify of safety events
#
###########################################################################

import threading
import rospy
from bondpy import bondpy
from std_msgs.msg import String

class SafetyClient:
    def __init__(self, bondId):
        self._bond = bondpy.Bond("bond_topic", bondId, self._on_broken, self._on_formed)
        self._formed = False
        self._broken = False
        self._safetyActive = False
        self._fatalActive = False
        self._safetyResponseActive = False
        self._bondId = bondId

        self._bond.set_heartbeat_period(0.2)
        self._bond.set_heartbeat_timeout(1.5)
        self._bond.set_connect_timeout(60.0)
        
        self._lock = threading.RLock()

        rospy.Subscriber("safety", String, self._process_safety_message)

    def form_bond(self):
        self._bond.start()
        
        return self.wait_until_safe()

    def wait_until_safe(self):
        while not rospy.is_shutdown():
            if self._broken:
                return False

            if self._formed:
                return True
            rospy.sleep(0.1)
        return False

    # Private method
    def _process_safety_message(self, message):
        with self._lock:
            if(message.data == self._bondId):
                self._safetyActive = True

            if(message.data == "FATAL"):
                self._fatalActive = True
                self._safetyActive = True

    def is_safety_active(self):
        return self._safetyActive

    def is_fatal_active(self):
        return self._fatalActive

    def set_safety_response_active(self):
        self._safetyResponseActive = True

    def is_safety_response_active(self):
        return self._safetyResponseActive

    # Private method
    def _on_broken(self):
        with self._lock:
            self._broken = True
            self._formed = False

            self._safetyActive = True
            self._fatalActive = True

    # Private method
    def _on_formed(self):
        with self._lock:
            self._broken = False
            self._formed = True

    def get_id(self):
        return self._bondId
