#!/usr/bin/env python3

from __future__ import print_function
#from six.moves import input

import sys
import copy
import rospy
import moveit_commander
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


dots_3d = np.loadtxt('pathDots.txt')
x_coords, y_coords, z_coords = zip(*[(dot[0], dot[1], dot[2]) for dot in dots_3d])

node_name = "teste_moveit"

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node(node_name, anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "ur5_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)


# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

go = True
i = 0
while go == True:
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 0.4
    # pose_goal.position.y = 0.1
    # pose_goal.position.z = 0.4

    # move_group.set_pose_target(pose_goal)

    # `go()` returns a boolean indicating whether the planning and execution was successful.
    # success = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets()

    waypoints = []

    scale = 1.0

    wpose = move_group.get_current_pose().pose

    x_axis = float(x_coords[i])
    y_axis = float(y_coords[i])
    z_axis = float(z_coords[i])

    try:
        wpose.position.x -= scale * x_axis  # Move forward/backwards in (x)
        wpose.position.y += scale * y_axis # Move sideways (y)
        wpose.position.z -= scale * z_axis # Move up/down (z)
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
    except:
        print("Erro ao realizar movimento")
        go = False
    else:
        if i<len(dots_3d):
            i = i+1
            print("Movimento " + str(i))
            print("Movimento executado com sucesso. Aguardando 5s")
            time.sleep(5)
        else:
            print("Movimento executado com sucesso. Finalizando.")
            go = False    


    