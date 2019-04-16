controller_manager_ns: controller_manager
controller_list:
  - name: left_arm_controller
    action_ns: follow_joint_trajectory
    default: True
    type: FollowJointTrajectory
    joints:
      - arm_left_1_joint
      - arm_left_2_joint
      - arm_left_3_joint
      - arm_left_4_joint
      - arm_left_5_joint
      - arm_left_6_joint
      - arm_left_7_joint
  - name: left_arm_torso_controller
    action_ns: follow_joint_trajectory
    default: True
    type: FollowJointTrajectory
    joints:
      - torso_lift_joint
      - arm_left_1_joint
      - arm_left_2_joint
      - arm_left_3_joint
      - arm_left_4_joint
      - arm_left_5_joint
      - arm_left_6_joint
      - arm_left_7_joint
  - name: right_arm_controller
    action_ns: follow_joint_trajectory
    default: True
    type: FollowJointTrajectory
    joints:
      - arm_right_1_joint
      - arm_right_2_joint
      - arm_right_3_joint
      - arm_right_4_joint
      - arm_right_5_joint
      - arm_right_6_joint
      - arm_right_7_joint
  - name: right_arm_torso_controller
    action_ns: follow_joint_trajectory
    default: True
    type: FollowJointTrajectory
    joints:
      - torso_lift_joint
      - arm_right_1_joint
      - arm_right_2_joint
      - arm_right_3_joint
      - arm_right_4_joint
      - arm_right_5_joint
      - arm_right_6_joint
      - arm_right_7_joint
  - name: torso_controller
    action_ns: follow_joint_trajectory
    default: True
    type: FollowJointTrajectory
    joints:
      [torso_lift_joint]
  - name: both_arms_torso_controller
    action_ns: follow_joint_trajectory
    default: True
    type: FollowJointTrajectory
    joints:
      - torso_lift_joint
      - arm_left_1_joint
      - arm_left_2_joint
      - arm_left_3_joint
      - arm_left_4_joint
      - arm_left_5_joint
      - arm_left_6_joint
      - arm_left_7_joint
      - arm_right_1_joint
      - arm_right_2_joint
      - arm_right_3_joint
      - arm_right_4_joint
      - arm_right_5_joint
      - arm_right_6_joint
      - arm_right_7_joint
@[if left_end_effector == "pal-gripper"]@
  - name: left_gripper_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_left_left_finger_joint
      - gripper_left_right_finger_joint
@[end if]@
@[if left_end_effector == "schunk-wsg"]@
  - name: left_gripper_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_left_finger_joint
@[end if]@
@[if left_end_effector == "pal-hey5"]@
  - name: hand_left_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - hand_left_index_joint
      - hand_left_thumb_joint
      - hand_left_mrl_joint
@[end if]@
@[if right_end_effector == "pal-gripper"]@
  - name: right_gripper_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_right_left_finger_joint
      - gripper_right_right_finger_joint
@[end if]@
@[if right_end_effector == "schunk-wsg"]@
  - name: right_gripper_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_right_finger_joint
@[end if]@
@[if right_end_effector == "pal-hey5"]@
  - name: hand_right_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - hand_right_index_joint
      - hand_right_thumb_joint
      - hand_right_mrl_joint
@[end if]@