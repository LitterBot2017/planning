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
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import math
## END_SUB_TUTORIAL

from std_msgs.msg import String

def cpr_move_group_python_interface():
  ## First initialize moveit_commander and rospy.
  rospy.sleep(5)
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
  group.set_planner_id("RRTkConfigDefault")

  #################################Initialize connection to robot arm
  commands_publisher_ = rospy.Publisher("CPRMoverCommands", String, queue_size = 1);
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

  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  # pose_target.orientation.w = 1.0
  # pose_target.position.x = 0.7
  # pose_target.position.y = -0.05
  # pose_target.position.z = 1.1
  # pose_target.orientation.w = 1.0
  # pose_target.position.x = 0.30
  # pose_target.position.y = -0.00
  # pose_target.position.z = 0.30
  # group.set_goal_orientation_tolerance(1.5)
  # print group.get_goal_orientation_tolerance()
  # print group.get_goal_position_tolerance()
  # print group.get_goal_tolerance()
  # print group.get_goal_joint_tolerance()
  print pose_target
  group.set_pose_target(pose_target)

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
  # group_variable_values[0] = math.pi/4
  # group_variable_values[1] = math.pi/4
  # group_variable_values[2] = math.pi/4
  # group_variable_values[3] = math.pi/4
  group_variable_values[0] = 0
  group_variable_values[1] = 0
  group_variable_values[2] = 0
  group_variable_values[3] = 0
  group.set_joint_value_target(group_variable_values)

  plan2 = group.plan()

  print plan2


  group.go(wait=True)

  print "============ Waiting while RVIZ displays plan2..."
  # rospy.sleep(5)

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  print "============ STOPPING"


if __name__=='__main__':
  try:
    cpr_move_group_python_interface()
  except rospy.ROSInterruptException:
    pass

