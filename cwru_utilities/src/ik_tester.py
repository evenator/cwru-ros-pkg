#! /usr/bin/python

import roslib; roslib.load_manifest('cwru_utilities')
import rospy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from arm_navigation_msgs.srv import *
from kinematics_msgs.msg import *
from kinematics_msgs.srv import *
from std_srvs.srv import Empty
import tf
import random
#TODO: move these to params
joint_names = [
  'joint1',
  'joint2',
  'joint3',
  'joint4',
  'joint5',
  'joint6'
]
joint_limits = [
  [-2.88, 2.88],
  [-1.92, 1.92],
  [-1.575, 1.23],
  [-2.80, 2.80],
  [-2.10, 2.10],
  [-3.142, 3.142],
]
increment = .2
tipLinkName = 'gripper_body'
baseFrame = 'irb_120_base_link'

failCount = 0
successCount = 0

def getIKFromJoints(jointAngles):
  #publish joint states
  msg = JointState()
  msg.header.stamp = rospy.Time.now()
  msg.name = joint_names
  msg.position = jointAngles
  jointPub.publish(msg)
  #wait for robot state to update TF (robot state runs at 50 hz)
  rospy.sleep(0.025)
  (trans, rot)=tfListener.lookupTransform(tipLinkName, baseFrame, rospy.Time(0))
  p = Point(trans[0], trans[1], trans[2])
  q = Quaternion(rot[0], rot[1], rot[2], rot[3])
  pose = Pose(p, q)
  getIKFromPose(pose)
  
def getIKFromPose(pose):
  ik_request = PositionIKRequest()
  ik_request.ik_link_name = tipLinkName
  for name in joint_names:
    ik_request.ik_seed_state.joint_state.name.append(name)
    ik_request.ik_seed_state.joint_state.position.append(0.0)
  ik_request.pose_stamped = PoseStamped()
  ik_request.pose_stamped.header.stamp = rospy.Time.now()
  ik_request.pose_stamped.header.frame_id = baseFrame
  ik_request.pose_stamped.pose = pose
  try:
    ikClient(ik_request, rospy.Duration(5.0))
  except rospy.service.ServiceException:
    print "ik request failed"
    global failCount
    failCount += 1
  else:
    global successCount
    successCount += 1

def iterateOverRange(jointLimits, index, jointAngles):
  jointRange = jointLimits[index]
  angle = jointRange[0]
  while angle < jointRange[1]:
    jointAngles[index] = angle
    if index == 0:
      getIKFromJoints(jointAngles)
    else:
      iterateOverRange(joint_limits, index-1, jointAngles)
    angle += increment
    if index == 5:
      print(jointAngles)

def testIK(jointLimits, numSamples):
  for i in range(numSamples):
    jointAngles = []
    for j in range(len(jointLimits)):
      jointAngles.append(random.uniform(jointLimits[j][0],jointLimits[j][1]))
    print jointAngles
    getIKFromJoints(jointAngles)

if __name__ == "__main__":
  rospy.init_node('ik_tester')
  jointPub = rospy.Publisher('/infrequent_joint_states',JointState)
  ikClient = rospy.ServiceProxy('/abby_irb_120_kinematics/get_ik', GetPositionIK)
  rospy.wait_for_service('/environment_server/set_planning_scene_diff')
  setPlanning = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)
  rospy.wait_for_service('/abby_irb_120_kinematics/get_ik')
  rospy.wait_for_service('/abby_irb_120_kinematics/get_ik_solver_info')
  setPlanning(SetPlanningSceneDiffRequest())
  tfListener = tf.TransformListener()
  while not rospy.is_shutdown():
    try:
      tfListener.lookupTransform(tipLinkName, baseFrame, rospy.Time(0))
    except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    else:
      break
  #iterateOverRange(joint_limits, 5, [0,0,0,0,0,0])
  testIK(joint_limits, 5000)
  print "Success: {}; Failure: {}; Percentage: {}".format(successCount, failCount, 100.0*float(successCount)/float(successCount + failCount))
