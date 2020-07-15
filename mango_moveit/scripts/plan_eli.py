#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('plan_1',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm_1")
group_2 = moveit_commander.MoveGroupCommander("arm_2")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

joint_goal = group.get_current_joint_values()
joint_goal_2 = group_2.get_current_joint_values()
# arm_1_home
joint_goal[0] = -3.14
joint_goal[1] = -1.2
joint_goal[2] = 0
joint_goal[3] = 0.6
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)

#arm2_home
joint_goal_2[0] = -1.57
joint_goal_2[1] = -0.4486
joint_goal_2[2] = -0.276
joint_goal_2[3] = -3.14
group_2.go(joint_goal_2, wait=True)

#arm_1_move1
joint_goal[0] = -1.57
joint_goal[1] = 0
joint_goal[2] = -0.4141
joint_goal[3] = -0.1724
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)
#arm_1_pick
joint_goal[0] = -1.57
joint_goal[1] = 0.5
joint_goal[2] = -0.6556
joint_goal[3] = -0.5521
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)

#arm2_pick
joint_goal_2[0] = -1.57
joint_goal_2[1] = -2.8
joint_goal_2[2] = 0.2415
joint_goal_2[3] = 0
group_2.go(joint_goal_2, wait=True)

#arm_1_move2
joint_goal[0] = 0
joint_goal[1] = 0.1035
joint_goal[2] = -0.5866
joint_goal[3] = -0.069
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)

#arm2_move1
joint_goal_2[0] = 0
joint_goal_2[1] = -2.5
joint_goal_2[2] = 0.5175
joint_goal_2[3] = -0.276
group_2.go(joint_goal_2, wait=True)

#arm_1_weight
joint_goal[0] = 0
joint_goal[1] = 0.3451
joint_goal[2] = -0.5866
joint_goal[3] = -0.3105
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)
#arm_1_move2
joint_goal[0] = 0
joint_goal[1] = 0.1035
joint_goal[2] = -0.5866
joint_goal[3] = -0.069
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)
#arm_1_weight
joint_goal[0] = 0
joint_goal[1] = 0.3451
joint_goal[2] = -0.5866
joint_goal[3] = -0.3105
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)

#arm_1_eliminate
joint_goal[0] = -3.14
joint_goal[1] = 0
joint_goal[2] = -0.6
joint_goal[3] = -0.1724
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)
#arm_1_move1
joint_goal[0] = -1.57
joint_goal[1] = 0
joint_goal[2] = -0.4141
joint_goal[3] = -0.1724
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)
#arm_1_pick
joint_goal[0] = -1.57
joint_goal[1] = 0.5
joint_goal[2] = -0.6556
joint_goal[3] = -0.5521
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)
#arm_1_move2
joint_goal[0] = 0
joint_goal[1] = 0.1035
joint_goal[2] = -0.5866
joint_goal[3] = -0.069
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)
#arm_1_weight
joint_goal[0] = 0
joint_goal[1] = 0.3451
joint_goal[2] = -0.5866
joint_goal[3] = -0.3105
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)
#arm_1_move2
joint_goal[0] = 0
joint_goal[1] = 0.1035
joint_goal[2] = -0.5866
joint_goal[3] = -0.069
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)
#arm_1_weight
joint_goal[0] = 0
joint_goal[1] = 0.3451
joint_goal[2] = -0.5866
joint_goal[3] = -0.3105
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)
#arm_1_move3
joint_goal[0] = 1.572
joint_goal[1] = 0
joint_goal[2] = -0.9710
joint_goal[3] = 0.4
joint_goal[4] = 3.14
group.go(joint_goal, wait=True)

#arm_1_put
joint_goal[0] = 1.572
joint_goal[1] = -0.138
joint_goal[2] = -0.3105
joint_goal[3] = -0.207
joint_goal[4] = 3.14
group.go(joint_goal, wait=True)

#arm2_move2
joint_goal_2[0] = 1.57
joint_goal_2[1] = -2
joint_goal_2[2] = 0
joint_goal_2[3] = -0.276
group_2.go(joint_goal_2, wait=True)
#arm2_putA
joint_goal_2[0] = 1.2
joint_goal_2[1] = -2.5879
joint_goal_2[2] = 1.0697
joint_goal_2[3] = -2.2429
group_2.go(joint_goal_2, wait=True)

# arm_1_home
joint_goal[0] = -3.14
joint_goal[1] = -1.2
joint_goal[2] = 0
joint_goal[3] = 0.6
joint_goal[4] = 1.57
group.go(joint_goal, wait=True)

#arm2_home
joint_goal_2[0] = -1.57
joint_goal_2[1] = -0.4486
joint_goal_2[2] = -0.276
joint_goal_2[3] = -3.14
group_2.go(joint_goal_2, wait=True)
#group.stop()
#rospy.sleep(5)

moveit_commander.roscpp_shutdown()
