#! /usr/bin/env python

import roslib; roslib.load_manifest('abby_arm_actions')
import rospy

import actionlib

from abby_arm_actions.msg import *

class AbbyArmAction(object):
  # create messages that are used to publish feedback/result
  _feedback = ArmActionFeedback()
  _result   = ArmActionResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, ArmActionAction, execute_cb=self.execute_cb)
    self.stowArm = StowArm()
    self.storeObject = StoreObject()
    self._as.start()
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = True
    
    #This (and the underlying classes) should be rewritten to give feedback during the action
    if goal.command == goal.STOW_ARM:
        self.stowArm.runUntilSuccess()
    elif goal.command == goal.STORE_OBJECT:
        self.storeObject.storeObject()
    elif goal.command == goal.GRAB_OBJECT:
        self.goPosition(goal.position, goal.grasp_approach)
    else:
        rospy.logerror("Undefined behavior requested.")
    self._as.set_succeeded()
      
if __name__ == '__main__':
  rospy.init_node('abby_arm_action')
  AbbyArmAction(rospy.get_name())
  rospy.spin()
