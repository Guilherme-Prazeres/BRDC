#!/usr/bin/env python3

from __future__ import print_function
#from six.moves import input

import sys
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion
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

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)


# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())


print("============ Indo a posição home")
# We get the joint values from the group and change some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -1.5447
joint_goal[2] = 1.5794
joint_goal[3] = -1.5794
joint_goal[4] = -1.5794
joint_goal[5] = 0


go = True
i = 0
while go == True:
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    scale = 0.0001

    x_axis = float(x_coords[i])
    y_axis = float(y_coords[i])
    z_axis = float(z_coords[i])

    pose_target = Pose()
    pose_target.position.x = x_axis * scale
    pose_target.position.y = y_axis * scale
    pose_target.position.z = z_axis * scale
    pose_target.orientation.x = 0.0
    pose_target.orientation.y = 0.0
    pose_target.orientation.z = 0.0
    pose_target.orientation.w = 1.0

    try:
        # Set the target pose for the arm
        move_group.set_pose_target(pose_target)

        # Plan the trajectory
        plan = move_group.go(wait=True)

        # Execute the planned trajectory
        move_group.execute(plan, wait=True)

        # Shutdown the moveit_commander module
        moveit_commander.roscpp_shutdown()
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


    