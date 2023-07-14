#!/usr/bin/env python3

from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg 
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import cv2
import numpy as np
from pdf2image import convert_from_path
import matplotlib.pyplot as plt
from PIL import Image
from pypdf import PdfReader
import time

COMPENSATION = 2 #Compensacao para o movimento indesejado em Z
SCALE = 0.01
pdf_path = 'path01.pdf'

# Leitura dos pontos gerados pelo script planejador
dots_3d = np.loadtxt(pdf_path)
x_coords, y_coords, z_coords = zip(*[(dot[0], dot[1], dot[2]) for dot in dots_3d])



#Configuracao inicial
node_name = "ur5_moveit_config"

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node(node_name, anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

move_group.set_pose_reference_frame('base_link')

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)


group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())



#Goto home
print("============ Indo a posição home")
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -1.5447
joint_goal[2] = 1.5794
joint_goal[3] = -1.5794
joint_goal[4] = -1.5794
joint_goal[5] = 0
#Alterando o valor de cada junta

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)
#Calling ``stop()`` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets().
move_group.clear_pose_targets()


#Ciclo iterativo pelo arquivo TXT
go = True
i = 0
while go == True:
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    scale = SCALE
    #scale = 1e-6

    x_axis = float(x_coords[i])
    y_axis = float(y_coords[i])
    z_axis = float(z_coords[i])
    
    
    move_group.stop()
    move_group.clear_pose_targets()
    waypoints = []
    wpose = move_group.get_current_pose().pose
    pose_target = geometry_msgs.msg.PoseStamped()

    pose_target.header.frame_id = 'world'
    pose_target.pose.position.x = x_axis * scale
    pose_target.pose.position.y = y_axis * scale
    pose_target.pose.position.z = z_axis * scale
    pose_target.pose.orientation.x = -1.5787790218870077
    pose_target.pose.orientation.y = -1.5794293157488664
    pose_target.pose.orientation.z = -1.5794293157488664
    pose_target.pose.orientation.w = 1.0

    try:
        wpose.position.x -= scale * x_axis  # Move forward/backwards in (x)
        wpose.position.y += scale * y_axis # Move sideways (y)
        wpose.position.z -= scale * (z_axis+COMPENSATION) # Move up/down (z)
        waypoints.append(copy.deepcopy(wpose)) # Send the values of the axis

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        #return plan, fraction

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        move_group.execute(plan, wait=True)
        
        
    except rospy.ROSInterruptException:
        pass
    except:
        print("Erro ao realizar movimento")
        go = False
    else:
        if i<len(dots_3d):
            print("Movimento " + str(i))
            print("Movimento executado com sucesso. Aguardando 2s")
            time.sleep(2)
            i = i+1
        else:
            print("Movimento executado com sucesso. Finalizando.")
            go = False    


    