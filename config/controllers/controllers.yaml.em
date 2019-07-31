controller_manager_ns: controller_manager
controller_list:
  - name: arm_left_controller
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
  - name: arm_right_controller
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
  - name: torso_controller
    action_ns: follow_joint_trajectory
    default: True
    type: FollowJointTrajectory
    joints:
      [torso_lift_joint]
@[if end_effector_left == "pal-gripper"]@
  - name: gripper_left_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_left_left_finger_joint
      - gripper_left_right_finger_joint
@[end if]@
@[if end_effector_left == "schunk-wsg"]@
  - name: gripper_left_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_left_finger_joint
@[end if]@
@[if end_effector_left == "pal-hey5"]@
  - name: hand_left_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - hand_left_index_joint
      - hand_left_thumb_joint
      - hand_left_mrl_joint
@[end if]@
@[if end_effector_right == "pal-gripper"]@
  - name: gripper_right_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_right_left_finger_joint
      - gripper_right_right_finger_joint
@[end if]@
@[if end_effector_right == "schunk-wsg"]@
  - name: gripper_right_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - gripper_right_finger_joint
@[end if]@
@[if end_effector_right == "pal-hey5"]@
  - name: hand_right_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - hand_right_index_joint
      - hand_right_thumb_joint
      - hand_right_mrl_joint
@[end if]@
