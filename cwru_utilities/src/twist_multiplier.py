#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Edward Venator, Case Western Reserve University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Case Western Reserve University nor the names 
#    of its contributors may be used to endorse or promote products 
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('cwru_utilities')
import rospy
from geometry_msgs.msg import Twist

class TwistMultiplier:
    
    def twistCallback(self, twist_in):
        """Latches the input message for republishing"""
        self._msg = twist_in
        self._last_msg = rospy.Time.now()
        self._pub.publish(self._msg)
    
    def run(self):
        """Runs the publisher at the rate specified. If the time is the last message 
        longer than 1/inRate, will not publish"""
        rateTimer = rospy.Rate(self._outRate)
        while not rospy.is_shutdown():
            period = (rospy.Time.now() - self._last_msg).to_sec()
            if period > self._inPeriod:
                rospy.logdebug("Twist multipler input slower than minimum. Messages coming at %f Hz", period)
            elif self._msg != None:
                self._pub.publish(self._msg)
            rateTimer.sleep()
    
    def __init__(self, inRate, outRate):
        """Sets up a twist repeater that outputs at a frequency higher than the input
        inRate is the minimum input frequency
        outRate is the minimum output frequency"""
        if inRate <= 0.0:
            rospy.signal_shutdown("Input rate must be positive")
        if outRate <= 0.0:
            rospy.signal_shutdown("Output rate must be positive")
        
        self._msg = None
        self._inPeriod = 1.0/inRate
        self._outRate = outRate
        self._last_msg = rospy.Time.now()
        
        self._pub = rospy.Publisher("/twist_out", Twist)
        self._sub = rospy.Subscriber("/twist_in", Twist, self.twistCallback)
        rospy.loginfo("Twist multiplier started with input timeout %f secs and output rate %f Hz",self._inPeriod, self._outRate)

if __name__ == "__main__":
    rospy.init_node('twist_frequency_multiplier')
    inRate = rospy.get_param("~in_rate", 1.0)
    outRate = rospy.get_param("~out_rate", 1.0)
    multiplierNode = TwistMultiplier(inRate, outRate)
    multiplierNode.run()
