#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
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
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import string
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import math
## END_SUB_TUTORIAL

from std_msgs.msg import String

def toQuaternion(pitch, roll, yaw):
  pose_target = geometry_msgs.msg.Pose()
  t0 = math.cos(yaw * 0.5)
  t1 = math.sin(yaw * 0.5)
  t2 = math.cos(roll * 0.5)
  t3 = math.sin(roll * 0.5)
  t4 = math.cos(pitch * 0.5)
  t5 = math.sin(pitch * 0.5)

  pose_target.orientation.w = t0 * t2 * t4 + t1 * t3 * t5
  pose_target.orientation.x = t0 * t3 * t4 - t1 * t2 * t5
  pose_target.orientation.y = t0 * t2 * t5 + t1 * t3 * t4
  pose_target.orientation.z = t1 * t2 * t4 - t0 * t3 * t5
  return pose_target

def calc_ik(x,y,z):
  #lengths of links
  l_1 = 206.0
  l_2 = 190.0
  l_3 = 220.0
  l_4 = 45.0

  joint_vals = [0.0] * 4

  if (x == 0.0) and (y == 0.0) and (z == 0.0):
    return joint_vals

  # position of the "wrist"
  z_prime = z+45.0

  # distance from the top of the end of the "waist" joint
  dd = math.sqrt(math.pow(x,2) + math.pow(y,2) + math.pow((z_prime-l_1),2))

  # phi is the angle from normal to the position of the wrist
  phi = math.asin((z_prime-l_1)/dd)

  #angle about joint 2 to the line from the top of the "waist" to the "wrist"
  C_1 = math.acos((math.pow(l_3,2)-math.pow(dd,2)-math.pow(l_2,2))/(-2.0*dd*l_2))

  #angle about joint 3 between links 2 and 3
  C_2 = math.acos((math.pow(dd,2)-math.pow(l_3,2)-math.pow(l_2,2))/(-2.0*l_3*l_2))

  joint_vals[0] = math.atan2(y,x)
  joint_vals[1] = math.radians(90.0)-phi-C_1
  joint_vals[2] = math.radians(180.0)-C_2
  joint_vals[3] = math.radians(180.0)-(math.radians(180.0)-C_1-C_2)-(math.radians(180.0)-math.radians(90.0)-phi)
  if joint_vals[1] > 85:
    raise UserWarning('Joints may be out of bounds')

  return joint_vals

def cpr_move_group_python_interface():
  ## First initialize moveit_commander and rospy.
  rospy.sleep(1)
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('cpr_move_group_python_interface',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("robot")
  # group.set_planner_id("RRTkConfigDefault")
  # group.set_planner_id("PRMstarkConfigDefault")
  # group.set_planner_id("RRTstarkConfigDefault")
  group.set_planner_id("ESTkConfigDefault")

  #################################Initialize connection to robot arm
  commands_publisher_ = rospy.Publisher("CPRMoverCommands", String, queue_size = 1);

  try:
    while True:
      # input_string = raw_input('Please enter the desired joint angles (in degrees) to move to deliminated by spaces-->')
      # angles = string.split(input_string,' ')
      # if len(angles) < 4:
      #   print 'INVALID NUMBER OF ANGLES, WE HAVE 4 JOINTS!'
      #   continue
      input_string = raw_input('Please enter the desired end effector position (in cm) to move to deliminated by spaces (type 0 0 0 to return to home) -->')
      xyz_vals = string.split(input_string,' ')
      if len(xyz_vals) < 3:
        print 'TOO FEW NUMBERS FOR COORDINATES IN 3D!!'
        continue
      try:
        joint_vals = calc_ik(float(xyz_vals[0])*10.0, float(xyz_vals[1])*10.0, float(xyz_vals[2])*10.0)
      except:
        print 'INVERSE KINEMATICS FAILED!!!'
        continue
      print "The following are the calculated values for the joint values"
      joint_vals_degrees = [0.0]*4
      joint_vals_degrees[0] = math.degrees(joint_vals[0])
      joint_vals_degrees[1] = math.degrees(joint_vals[1])
      joint_vals_degrees[2] = math.degrees(joint_vals[2])
      joint_vals_degrees[3] = math.degrees(joint_vals[3])
      # print joint_vals_degrees
      print repr(joint_vals_degrees[0]) + ' ' + repr(joint_vals_degrees[1]) + ' ' + repr(joint_vals_degrees[2]) + ' ' + repr(joint_vals_degrees[3])
      input_string = raw_input('hit ENTER to continue')
      rospy.sleep(1)
      msgCommands = String();
      msgCommands.data = "Connect";
      commands_publisher_.publish(msgCommands);
      rospy.sleep(1)
      msgCommands.data = "Reset";
      commands_publisher_.publish(msgCommands);
      rospy.sleep(1)
      msgCommands.data = "Enable";
      commands_publisher_.publish(msgCommands);
      rospy.sleep(1)

      ## Getting Basic Information
      ## ^^^^^^^^^^^^^^^^^^^^^^^^^
      ##
      ## We can get the name of the reference frame for this robot
      print "============ Reference frame: %s" % group.get_planning_frame()

      ## We can also print the name of the end-effector link for this group
      print "============ Reference frame: %s" % group.get_end_effector_link()

      ## We can get a list of all the groups in the robot
      print "============ Robot Groups:"
      print robot.get_group_names()

      ## Sometimes for debugging it is useful to print the entire state of the
      ## robot.
      print "============ Printing robot state"
      print robot.get_current_state()
      print "============"

      # group.clear_pose_targets()

      ###############NOTE!!!: EST WITH Pose at w=1.0, x=0.2, y=0.0, and z=0.3 works!

      ## Planning to a Pose goal
      ## ^^^^^^^^^^^^^^^^^^^^^^^
      ## We can plan a motion for this group to a desired pose for the 
      ## end-effector
      # print "============ Generating plan 1"
      # pose_target = geometry_msgs.msg.Pose()
      # pose_target.orientation.w = 1.0
      # pose_target.position.x = 0.7
      # pose_target.position.y = -0.05
      # pose_target.position.z = 1.1
      # pose_target.orientation.w = 1.0
      # pose_target.position.x = 0.30
      # pose_target.position.y = -0.00
      # pose_target.position.z = 0.30
      # pose_target.orientation.w = 1.0
      # pose_target.orientation.x = 0.0
      # pose_target.orientation.y = 0.0
      # pose_target.orientation.z = 0.0
      # pose_target.position.x = float(angles[0])
      # pose_target.position.y = float(angles[1])
      # pose_target.position.z = float(angles[2])
      # group.set_goal_orientation_tolerance(1.5)
      # print group.get_goal_orientation_tolerance()
      # print group.get_goal_position_tolerance()
      # print group.get_goal_tolerance()
      # print group.get_goal_joint_tolerance()
      # print pose_target
      # group.set_pose_target(pose_target)

      ## Here is just a position planning
      # print "============ Generating plan 1.5"
      # position_target = geometry_msgs.msg.Point()
      # position_target.orientation.w = 1.0
      # position_target.position.x = 0.7
      # position_target.position.y = -0.05
      # position_target.position.z = 1.1
      # position_target.orientation.w = 1.0
      # position_target.position.x = 0.30
      # position_target.position.y = -0.00
      # position_target.position.z = 0.30
      # position_target = [0.2, 0, 0.2]
      # position_target = [float(angles[0]), float(angles[1]), float(angles[2])]
      # group.set_goal_orientation_tolerance(1.5)
      # print group.get_goal_orientation_tolerance()
      # print group.get_goal_position_tolerance()
      # print group.get_goal_tolerance()
      # print group.get_goal_joint_tolerance()
      # print position_target
      # group.set_position_target(position_target)


      ## Now, we call the planner to compute the plan
      ## and visualize it if successful
      ## Note that we are just planning, not asking move_group 
      ## to actually move the robot
      # plan1 = group.plan()

      # print plan1

      ## Moving to a pose goal
      ## ^^^^^^^^^^^^^^^^^^^^^
      ##
      ## Moving to a pose goal is similar to the step above
      ## except we now use the go() function. Note that
      ## the pose goal we had set earlier is still active 
      ## and so the robot will try to move to that goal. We will
      ## not use that function in this tutorial since it is 
      ## a blocking function and requires a controller to be active
      ## and report success on execution of a trajectory.

      # Uncomment below line when working with a real robot
      # group.go(wait=True)

      ## Planning to a joint-space goal 
      ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      ##
      ## Let's set a joint space goal and move towards it. 
      ## First, we will clear the pose target we had just set.

      group.clear_pose_targets()

      ## Then, we will get the current set of joint values for the group
      group_variable_values = group.get_current_joint_values()
      print "============ Joint values: ", group_variable_values

      ## Now, let's modify one of the joints, plan to the new joint
      ## space goal and visualize the plan
      # group_variable_values[0] = float(angles[0])*(math.pi/180)
      # group_variable_values[1] = float(angles[1])*(math.pi/180)
      # group_variable_values[2] = float(angles[2])*(math.pi/180)
      # group_variable_values[3] = float(angles[3])*(math.pi/180)
      group_variable_values[0] = float(joint_vals[0])
      group_variable_values[1] = float(joint_vals[1])
      group_variable_values[2] = float(joint_vals[2])
      group_variable_values[3] = float(joint_vals[3])
      group.set_joint_value_target(group_variable_values)

      plan2 = group.plan()

      print plan2

      group.go(wait=True)

      # print "============ Waiting while RVIZ displays plan2..."
      rospy.sleep(5)


  except KeyboardInterrupt:
    print "Exiting through keyboard"
    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    print "============ STOPPING"



if __name__=='__main__':
  try:
    cpr_move_group_python_interface()
  except rospy.ROSInterruptException:
    pass

