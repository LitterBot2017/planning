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
# import sensor_msgs.msg
from sensor_msgs.msg import JointState
#camera
# from downview_cam.msg import po 

# camera/pressure info
# float32 x
# float32 y
# string is_centered
# string pickup_state
from navigation.msg import Arm

import math
## END_SUB_TUTORIAL

from std_msgs.msg import String

import numpy as np


class Melle_Arm(object):
    """docstring for Melle_Arm"""
    def __init__(self):
        ## First initialize moveit_commander and rospy.
        print "=============Starting Mell-e Arm Node============"
        moveit_commander.roscpp_initialize(sys.argv) # I actually have no idea what this line does
        rospy.init_node('melle_arm_interface_node',
                        anonymous=True)

        ## Instantiate a RobotCommander object.  This object is an interface to
        ## the robot as a whole.
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a PlanningSceneInterface object.  This object is an interface
        ## to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a MoveGroupCommander object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the left
        ## arm.  This interface can be used to plan and execute motions on the left
        ## arm.
        self.group = moveit_commander.MoveGroupCommander("robot")

        ## Set planner type here, some may potentially generate better plans than others, have not done extensive tests
        # group.set_planner_id("RRTkConfigDefault")
        # group.set_planner_id("PRMstarkConfigDefault")
        # group.set_planner_id("RRTstarkConfigDefault")
        self.group.set_planner_id("ESTkConfigDefault")

        #################################Initialize connection to robot arm
        self.commands_publisher_ = rospy.Publisher("CPRMoverCommands", String, queue_size = 1)

        #################################Initialize publisher for signaling in the middle of a grasp
        self.robot_state_publisher = rospy.Publisher("arm_state", String, queue_size = 1)

        #################################
        # TODO: get the litter stated, and also position of the piece of litter from the camera
        # self.camera_subscriber = rospy.Subscriber("down_cam_msg", po, self.down_cam_cb)
        # self.po = po()

        # self.pressure_subscriber = rospy.Subscriber("Seal_msg", String, self.seal_cb)
        # self.sealed = String()
        # self.sealed.data = "off"
        self.cam_and_pressure_subscriber = rospy.Subscriber("arm", Arm, self.cam_and_pressure_cb)
        self.cam_and_pressure_data = Arm()
        self.cam_and_pressure_data.x = 0.0
        self.cam_and_pressure_data.y = 0.0
        self.cam_and_pressure_data.is_centered = 'not_centered'
        self.cam_and_pressure_data.pickup_state = 'off'

        #to check joint_states to make sure the arm has stopped executing
        self.joint_subscriber = rospy.Subscriber("joint_states", JointState, self.joint_states_cb)
        # self.joint_states = [[0]*4]*5
        self.joint_states = [JointState()]*5
        # import IPython
        # IPython.embed()


        ################################ Set arm state to be "done"
        feedback = String()
        feedback.data = 'pickup_done'
        self.robot_state_publisher.publish(feedback)


    # x y z are all in CM's
    def calc_ik(self,x,y,z):
        #lengths of links
        l_1 = 206.0
        l_2 = 190.0
        l_3 = 220.0
        l_4 = 45.0

        joint_vals = [0.0] * 4
        joint_vals[3] = math.radians(-90.0)

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

        # if this joint goes more than this I think it breaks and I need to recalibrate the arm
        if joint_vals[1] > math.radians(85):
            raise UserWarning('Joints may be out of bounds')
            joint_vals = [0.0] * 4

        return joint_vals

    def cam_and_pressure_cb(self,data):
        self.cam_and_pressure_data = data

    # def down_cam_cb(self,data):
    #     self.po = data

    # def seal_cb(self,data):
    #     self.sealed = data

    def joint_states_cb(self,data):
        for idx in range(0,4):
            self.joint_states[idx] = self.joint_states[idx+1]
        self.joint_states[4] = data

    def joints_stable(self):
        stable = True
        for idx in range(4):
            test_stable = self.joint_states[idx].position == self.joint_states[idx+1].position
            stable = stable & test_stable
            rospy.sleep(0.1)
        print "joints stable?:" + repr(stable)
        return stable

    def pick_up_signal(self):
        if self.cam_and_pressure_data.is_centered == 'centered':
            return True
        else:
            return False

    def homography(self,x,y):
        # h_mat = np.array([[2.20112274361046e-05, -0.00104907565122139, 0.716837052934637], 
        #         [-0.00106749217814114, -2.52155668261608e-05, 0.697069869544084], 
        #         [-1.14045687505024e-07, 3.65086924495886e-07, 0.0153621383290117]])

        # h_mat = np.array([[5.46746420977118e-06, 0.00186858537929480, -0.770939373319039],
        #                   [0.00187846177904567, -6.78829638396309e-06, -0.636755466418008], 
        #                   [-3.69398387440614e-07, 2.44271999503352e-07, -0.0137090607660856]])

        h_mat = np.array([[-0.000109081062020685, -0.00179116587204159, 0.864555067791766], 
                            [-0.00175609183828732, 0.000101439555606069, 0.502352545883134], 
                            [-1.76373377005859e-08, -1.42370718069173e-06, 0.0134216271701275]])

        # h_mat = np.array([[0.000192002622251096, 0.00308145301657239, -0.114684138090243], 
        #                     [0.00302882040810846, -0.000169892835332421, -0.993148051435149], 
        #                     [2.40497923689312e-07, 1.88279634116435e-06, 0.0220399991633980]])        

        point = np.array([[x],[y],[1]])

        q = np.dot(h_mat,point)
        real_x = q[0]/q[2]
        real_y = q[1]/q[2]

        return float(real_x), float(real_y)

    def go_to_coordinate(self, x, y, z, far):
            # first calculate inverse kinmatics here
            try:
                joint_vals = self.calc_ik(float(x)*10.0, float(y)*10.0, float(z)*10.0)
            except:
                print 'INVERSE KINEMATICS FAILED!!!'
                ### This sould never happen, arm system should completely abort if this were to happen
                exit()
                return None

            # print joint vals here to make sure they are valid
            print "The following are the calculated values for the joint values"
            joint_vals_degrees = [0.0]*4
            joint_vals_degrees[0] = math.degrees(joint_vals[0])
            joint_vals_degrees[1] = math.degrees(joint_vals[1])
            joint_vals_degrees[2] = math.degrees(joint_vals[2])
            joint_vals_degrees[3] = math.degrees(joint_vals[3])
            print repr(joint_vals_degrees[0]) + ' ' + repr(joint_vals_degrees[1]) + ' ' + repr(joint_vals_degrees[2]) + ' ' + repr(joint_vals_degrees[3])
            # input_string = raw_input('hit ENTER to continue')


            # initialize actual commands to the cpr_mover arm to ready it for new commands
            rospy.sleep(1)
            msgCommands = String();
            msgCommands.data = "Connect";
            self.commands_publisher_.publish(msgCommands);
            rospy.sleep(1)
            msgCommands.data = "Reset";
            self.commands_publisher_.publish(msgCommands);
            rospy.sleep(1)
            msgCommands.data = "Enable";False
            self.commands_publisher_.publish(msgCommands);
            rospy.sleep(1)

            # This is from the tutorial code for generating the plans and executing them
            ## Getting Basic Information
            ## ^^^^^^^^^^^^^^^^^^^^^^^^^
            ##
            ## We can get the name of the reference frame for this robot
            print "============ Reference frame: %s" % self.group.get_planning_frame()

            ## We can also print the name of the end-effector link for this group
            print "============ Reference frame: %s" % self.group.get_end_effector_link()

            ## We can get a list of all the groups in the robot
            print "============ Robot Groups:"
            print self.robot.get_group_names()

            ## Sometimes for debugging it is useful to print the entire state of the
            ## robot.
            print "============ Printing robot state"
            print self.robot.get_current_state()
            print "============"

            ## Planning to a joint-space goal 
            ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            ##
            ## Let's set a joint space goal and move towards it. 
            ## First, we will clear the pose target we had just set. <---- I'm still doing this just to be safe

            self.group.clear_pose_targets()

            ## Then, we will get the current set of joint values for the group
            group_variable_values = self.group.get_current_joint_values()
            print "============ Joint values: ", group_variable_values

            ## Now, let's modify one of the joints, plan to the new joint
            ## space goal and visualize the plan
            group_variable_values[0] = float(joint_vals[0])
            group_variable_values[1] = float(joint_vals[1])
            group_variable_values[2] = float(joint_vals[2])
            group_variable_values[3] = float(joint_vals[3])
            self.group.set_joint_value_target(group_variable_values)

            plan2 = self.group.plan()

            # print plan2

            # Actually Execute the command on the arm
            self.group.go(wait=True)

            # print "============ Waiting while RVIZ displays plan2..." <- this is the old thing
            # not this sleep is simply to wait for the arm to execute the command before handing back to the CV system
            if far == True:
                rospy.sleep(2.)
                while self.joints_stable() == False:
                    rospy.sleep(0.1)
                rospy.sleep(1.0)
            else:
                rospy.sleep(1.)
                while self.joints_stable() == False:
                    rospy.sleep(0.1)
                rospy.sleep(1.0)

    def return_to_home(self):
        joint_vals = self.calc_ik(0.0, 0.0, 0.0) #calc_ik will go to dump position with these values

        # print joint vals here to make sure they are valid
        print "RETURNING TO HOME!!!: The following are the calculated values for the joint values"
        joint_vals_degrees = [0.0]*4
        joint_vals_degrees[0] = math.degrees(joint_vals[0])
        joint_vals_degrees[1] = math.degrees(joint_vals[1])
        joint_vals_degrees[2] = math.degrees(joint_vals[2])
        joint_vals_degrees[3] = math.degrees(joint_vals[3])
        print repr(joint_vals_degrees[0]) + ' ' + repr(joint_vals_degrees[1]) + ' ' + repr(joint_vals_degrees[2]) + ' ' + repr(joint_vals_degrees[3])
        # input_string = raw_input('hit ENTER to continue')


        # initialize actual commands to the cpr_mover arm to ready it for new commands
        rospy.sleep(1)
        msgCommands = String();
        msgCommands.data = "Connect";
        self.commands_publisher_.publish(msgCommands);
        rospy.sleep(1)
        msgCommands.data = "Reset";
        self.commands_publisher_.publish(msgCommands);
        rospy.sleep(1)
        msgCommands.data = "Enable";
        self.commands_publisher_.publish(msgCommands);
        rospy.sleep(1)

        # This is from the tutorial code for generating the plans and executing them
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        ##
        ## We can get the name of the reference frame for this robot
        print "============ Reference frame: %s" % self.group.get_planning_frame()

        ## We can also print the name of the end-effector link for this group
        print "============ Reference frame: %s" % self.group.get_end_effector_link()

        ## We can get a list of all the groups in the robot
        print "============ Robot Groups:"
        print self.robot.get_group_names()

        ## Sometimes for debugging it is useful to print the entire state of the
        ## robot.
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print "============"

        ## Planning to a joint-space goal 
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ##
        ## Let's set a joint space goal and move towards it. 
        ## First, we will clear the pose target we had just set. <---- I'm still doing this just to be safe

        self.group.clear_pose_targets()

        ## Then, we will get the current set of joint values for the group
        group_variable_values = self.group.get_current_joint_values()
        print "============ Joint values: ", group_variable_values

        ## Now, let's modify one of the joints, plan to the new joint
        ## space goal and visualize the plan
        group_variable_values[0] = float(joint_vals[0])
        group_variable_values[1] = float(joint_vals[1])
        group_variable_values[2] = float(joint_vals[2])
        group_variable_values[3] = float(joint_vals[3])
        self.group.set_joint_value_target(group_variable_values)

        plan2 = self.group.plan()

        # print plan2

        # Actually Execute the command on the arm
        self.group.go(wait=True)

        # print "============ Waiting while RVIZ displays plan2..." <- this is the old thing
        # not this sleep is simply to wait for the arm to execute the command before handing back to the CV system
        rospy.sleep(2.)
        while self.joints_stable() == False:
            rospy.sleep(0.1)
            print self.joint_states
        rospy.sleep(1.0)


    # x y z are all in CM's
    def pick_up_litter(self):
        feedback = String()
        feedback.data = 'in_progress'
        self.robot_state_publisher.publish(feedback)
        #wait for settling
        rospy.sleep(5.0)
        x_fudge_factor = 0.0#4.0

        local_cam_x = self.cam_and_pressure_data.x
        local_cam_y = self.cam_and_pressure_data.y
        # end effector is ~3inches + base2ground is 4.65in
        # soda can is about 2.13 inches across        
        # note closest x is about 24 cm to be safe
        # z_thres = (-4.65+3+0.5)*2.54+(1.5*2.54)
        # z = (3.0)*2.54-2.54*1.5
        pass_count = 0
        #run through picking up routine, and then return home
        for idx in range(5):
            print idx
            z = 9 #starting height
            z_thres = 5#lowest height

            x,y = self.homography(local_cam_x,local_cam_y)
            print x
            print y
            target_x = x*0.875
            target_y = y*0.875

            sweep_factor = 5
            if pass_count == 0:
                x,y = self.correct_bounds(target_x,target_y)
            if pass_count == 1:
                x,y = self.correct_bounds(target_x+sweep_factor,target_y+sweep_factor)
            if pass_count == 2:
                x,y = self.correct_bounds(target_x+sweep_factor,target_y-sweep_factor)
            if pass_count == 3:
                x,y = self.correct_bounds(target_x-sweep_factor,target_y+sweep_factor)
            if pass_count == 4:
                x,y = self.correct_bounds(target_x-sweep_factor,target_y-sweep_factor)

            while (z > z_thres) and ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                # raw_input('hit enter to continue')
                if pass_count == 0:
                    # x,y = self.correct_bounds(target_x,target_y)
                    self.go_to_coordinate(x,y,z,True)
                    if not ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                        break
                    # x,y = self.correct_bounds(target_x+sweep_factor,target_y+sweep_factor)
                    # self.go_to_coordinate(x,y,z,False)
                    # if not ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                    #     break
                    # x,y = self.correct_bounds(target_x+sweep_factor,target_y-sweep_factor)
                    # self.go_to_coordinate(x,y,z,False)
                    # if not ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                    #     break
                    # x,y = self.correct_bounds(target_x-sweep_factor,target_y+sweep_factor)
                    # self.go_to_coordinate(x,y,z,False)
                    # if not ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                    #     break
                    # x,y = self.correct_bounds(target_x-sweep_factor,target_y-sweep_factor)
                    # self.go_to_coordinate(x,y,z,False)
                    # if not ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                    #     break
                else:
                    # x,y = self.correct_bounds(target_x,target_y)
                    self.go_to_coordinate(x,y,z,False)
                    if not ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                        break
                    # x,y = self.correct_bounds(target_x+sweep_factor,target_y+sweep_factor)
                    # self.go_to_coordinate(x,y,z,False)
                    # if not ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                    #     break
                    # x,y = self.correct_bounds(target_x+sweep_factor,target_y-sweep_factor)
                    # self.go_to_coordinate(x,y,z,False)
                    # if not ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                    #     break
                    # x,y = self.correct_bounds(target_x-sweep_factor,target_y+sweep_factor)
                    # self.go_to_coordinate(x,y,z,False)
                    # if not ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                    #     break
                    # x,y = self.correct_bounds(target_x-sweep_factor,target_y-sweep_factor)
                    # self.go_to_coordinate(x,y,z,False)
                    # if not ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                    #     break
                z -= 1.0
            if not ((self.cam_and_pressure_data.pickup_state == 'off') or (self.cam_and_pressure_data.pickup_state == '')):
                break
            pass_count += 1


        self.return_to_home()

        feedback.data = 'pickup_done'
        for x in xrange(1,10):
            self.robot_state_publisher.publish(feedback)
            rospy.sleep(0.1)

    def correct_bounds(self, x, y):
        sign_y = 0
        if y >= 0:
            sign_y = 1
        else:
            sign_y = -1

        if x < 24:
            x = 24
        if x > 35:
            x = 35

        mag = math.sqrt(math.pow(x,2)+math.pow(y,2))
        if mag > 35:
            y = math.sqrt(pow(35,2)-pow(x,2))
            y = y*sign_y

        return x,y

if __name__=='__main__':
    rospy.sleep(1)
  
    Arm = Melle_Arm()
  
    rate = rospy.Rate(120) #hz
    while not rospy.is_shutdown():
        if Arm.pick_up_signal() is True:
            Arm.pick_up_litter()
        else:
            # print 'no cans'
            rate.sleep()