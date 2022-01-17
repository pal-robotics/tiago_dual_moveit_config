<?xml version="1.0" ?>
<!--This does not replace URDF, and is not an extension of URDF.
    This is a format for representing semantic information about the robot structure.
    A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined
-->
<robot name="tiago_dual">
    <!--GROUPS: Representation of a set of joints and links. This can be useful for specifying DOF to plan for, defining arms, end effectors, etc-->
    <!--LINKS: When a link is specified, the parent joint of that link (if it exists) is automatically included-->
    <!--JOINTS: When a joint is specified, the child link of that joint (which will always exist) is automatically included-->
    <!--CHAINS: When a chain is specified, all the links along the chain (including endpoints) are included in the group. Additionally, all the joints that are parents to included links are also included. This means that joints along the chain and the parent joint of the base link are included in the group-->
    <!--SUBGROUPS: Groups can also be formed by referencing to already defined group names-->
@[for side in ['left', 'right']]@
@[if repr("has_arm_" + side)]@
    <group name="arm_@(side)">
        <joint name="arm_@(side)_1_joint" />
        <joint name="arm_@(side)_2_joint" />
        <joint name="arm_@(side)_3_joint" />
        <joint name="arm_@(side)_4_joint" />
        <joint name="arm_@(side)_5_joint" />
        <joint name="arm_@(side)_6_joint" />
        <joint name="arm_@(side)_7_joint" />
        <joint name="arm_@(side)_tool_joint" />
    </group>
    <group name="arm_@(side)_torso">
        <group name="arm_@(side)" />
        <group name="torso" />
    </group>
	@[end if]@
@[end for]@
@[if has_arm_left and has_arm_right]@
    <group name="both_arms_torso">
        <group name="torso" />
        <group name="arm_right" />
        <group name="arm_left" />
    </group>
@[end if]@
    <group name="torso">
        <joint name="torso_lift_joint" />
    </group>

@[for side in ['left', 'right']]@
@{
if side == "left":
    end_effector = end_effector_left
if side == "right":
    end_effector = end_effector_right
}@
@[if end_effector == 'pal-gripper']@
    <group name="gripper_@(side)">
        <link name="gripper_@(side)_left_finger_link" />
        <link name="gripper_@(side)_right_finger_link" />
        <link name="gripper_@(side)_link" />
        <joint name="gripper_@(side)_left_finger_joint" />
        <joint name="gripper_@(side)_right_finger_joint" />
    </group>
    <!--END EFFECTOR: Purpose: Represent information about an end effector.-->
    <end_effector name="gripper_@(side)" parent_link="arm_@(side)_tool_link" group="gripper_@(side)" parent_group="arm_@(side)_torso" />
@[end if]@
@[if end_effector == 'schunk-wsg']@
    <group name="gripper_@(side)">
        <link name="gripper_@(side)_left_finger_link" />
        <link name="gripper_@(side)_right_finger_link" />
        <link name="gripper_@(side)_link" />
        <joint name="gripper_@(side)_finger_joint" />
    </group>
    <!--END EFFECTOR: Purpose: Represent information about an end effector.-->
    <end_effector name="gripper_@(side)" parent_link="arm_@(side)_tool_link" group="gripper_@(side)" parent_group="arm_@(side)_torso" />
@[end if]@
@[if end_effector in ["robotiq-2f-85", "robotiq-2f-140"]]@
    <group name="gripper_@(side)">
        <link name="gripper_@(side)_coupler_link" />
        <link name="gripper_@(side)_base_link" />
        <link name="gripper_@(side)_left_outer_knuckle" />
        <link name="gripper_@(side)_left_outer_finger" />
        <link name="gripper_@(side)_left_inner_finger" />
        <link name="gripper_@(side)_left_inner_finger_pad" />
        <link name="gripper_@(side)_left_inner_knuckle" />
        <link name="gripper_@(side)_right_inner_knuckle" />
        <link name="gripper_@(side)_right_outer_knuckle" />
        <link name="gripper_@(side)_right_outer_finger" />
        <link name="gripper_@(side)_right_inner_finger" />
        <link name="gripper_@(side)_right_inner_finger_pad" />
        <joint name="gripper_@(side)_finger_joint" />
    </group>
    <!--END EFFECTOR: Purpose: Represent information about an end effector.-->
    <end_effector name="gripper_@(side)" parent_link="arm_@(side)_tool_link" group="gripper_@(side)" parent_group="arm_@(side)_torso" />
    <!--PASSIVE JOINT: Purpose: this element is used to mark joints that are not actuated-->
    <passive_joint name="gripper_@(side)_left_inner_finger_joint" />
    <passive_joint name="gripper_@(side)_left_inner_knuckle_joint" />
    <passive_joint name="gripper_@(side)_right_inner_knuckle_joint" />
    <passive_joint name="gripper_@(side)_right_outer_knuckle_joint" />
    <passive_joint name="gripper_@(side)_right_inner_finger_joint" />
@[end if]@

@[if end_effector == 'pal-hey5']@

    <group name="hand_@(side)">
        <joint name="hand_@(side)_index_joint" />
        <joint name="hand_@(side)_thumb_joint" />
        <joint name="hand_@(side)_mrl_joint" />
        <link name="hand_@(side)_index_link" />
        <link name="hand_@(side)_thumb_link" />
        <link name="hand_@(side)_mrl_link" />
        <link name="hand_@(side)_palm_link" />
    </group>
    <!--END EFFECTOR: Purpose: Represent information about an end effector.-->
    <end_effector name="hand_@(side)" parent_link="arm_@(side)_tool_link" group="hand_@(side)" />
    <!--PASSIVE JOINT: Purpose: this element is used to mark joints that are not actuated-->
    <passive_joint name="hand_@(side)_grasping_fixed_joint" />
    <passive_joint name="hand_@(side)_index_abd_joint" />
    <passive_joint name="hand_@(side)_index_flex_1_joint" />
    <passive_joint name="hand_@(side)_index_flex_2_joint" />
    <passive_joint name="hand_@(side)_index_flex_3_joint" />
    <passive_joint name="hand_@(side)_index_virtual_1_joint" />
    <passive_joint name="hand_@(side)_index_virtual_2_joint" />
    <passive_joint name="hand_@(side)_index_virtual_3_joint" />
    <passive_joint name="hand_@(side)_little_abd_joint" />
    <passive_joint name="hand_@(side)_little_flex_1_joint" />
    <passive_joint name="hand_@(side)_little_flex_2_joint" />
    <passive_joint name="hand_@(side)_little_flex_3_joint" />
    <passive_joint name="hand_@(side)_little_virtual_1_joint" />
    <passive_joint name="hand_@(side)_little_virtual_2_joint" />
    <passive_joint name="hand_@(side)_little_virtual_3_joint" />
    <passive_joint name="hand_@(side)_middle_abd_joint" />
    <passive_joint name="hand_@(side)_middle_flex_1_joint" />
    <passive_joint name="hand_@(side)_middle_flex_2_joint" />
    <passive_joint name="hand_@(side)_middle_flex_3_joint" />
    <passive_joint name="hand_@(side)_middle_virtual_1_joint" />
    <passive_joint name="hand_@(side)_middle_virtual_2_joint" />
    <passive_joint name="hand_@(side)_middle_virtual_3_joint" />
    <passive_joint name="hand_@(side)_palm_joint" />
    <passive_joint name="hand_@(side)_ring_abd_joint" />
    <passive_joint name="hand_@(side)_ring_flex_1_joint" />
    <passive_joint name="hand_@(side)_ring_flex_2_joint" />
    <passive_joint name="hand_@(side)_ring_flex_3_joint" />
    <passive_joint name="hand_@(side)_ring_virtual_1_joint" />
    <passive_joint name="hand_@(side)_ring_virtual_2_joint" />
    <passive_joint name="hand_@(side)_ring_virtual_3_joint" />
    <passive_joint name="hand_@(side)_thumb_abd_joint" />
    <passive_joint name="hand_@(side)_thumb_flex_1_joint" />
    <passive_joint name="hand_@(side)_thumb_flex_2_joint" />
    <passive_joint name="hand_@(side)_thumb_virtual_1_joint" />
    <passive_joint name="hand_@(side)_thumb_virtual_2_joint" />
@[end if]@
@[end for]@



    <!--DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially come into collision with any other link in the robot. This tag disables collision checking between a specified pair of links. -->
    <disable_collisions link1="base_antenna_left_link" link2="base_antenna_right_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="base_cover_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="base_laser_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="base_link" reason="Adjacent" />
    <disable_collisions link1="base_antenna_left_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="base_cover_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="base_laser_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="base_link" reason="Adjacent" />
    <disable_collisions link1="base_antenna_right_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="base_laser_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="base_link" reason="Adjacent" />
    <disable_collisions link1="base_cover_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="base_link" reason="Adjacent" />
    <disable_collisions link1="base_laser_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="base_link" link2="caster_back_left_1_link" reason="Adjacent" />
    <disable_collisions link1="base_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="base_link" link2="caster_back_right_1_link" reason="Adjacent" />
    <disable_collisions link1="base_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="base_link" link2="caster_front_left_1_link" reason="Adjacent" />
    <disable_collisions link1="base_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="base_link" link2="caster_front_right_1_link" reason="Adjacent" />
    <disable_collisions link1="base_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="base_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="base_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="base_link" link2="torso_fixed_column_link" reason="Adjacent" />
    <disable_collisions link1="base_link" link2="torso_fixed_link" reason="Adjacent" />
    <disable_collisions link1="base_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="base_link" link2="wheel_left_link" reason="Adjacent" />
    <disable_collisions link1="base_link" link2="wheel_right_link" reason="Adjacent" />
    <disable_collisions link1="base_link" link2="base_sonar_01_link" reason="Never"/>
    <disable_collisions link1="base_link" link2="base_sonar_02_link" reason="Never"/>
    <disable_collisions link1="base_link" link2="base_sonar_03_link" reason="Never"/>
    <disable_collisions link1="caster_back_left_1_link" link2="caster_back_left_2_link" reason="Adjacent" />
    <disable_collisions link1="caster_back_left_1_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="caster_back_right_2_link" reason="Adjacent" />
    <disable_collisions link1="caster_back_right_1_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="caster_front_left_2_link" reason="Adjacent" />
    <disable_collisions link1="caster_front_left_1_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="caster_front_right_2_link" reason="Adjacent" />
    <disable_collisions link1="caster_front_right_1_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="head_1_link" link2="head_2_link" reason="Adjacent" />
    <disable_collisions link1="head_1_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="head_1_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="head_1_link" link2="torso_lift_link" reason="Adjacent" />
    <disable_collisions link1="head_1_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="head_1_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="head_2_link" link2="torso_fixed_column_link" reason="Never" />
    <disable_collisions link1="head_2_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="head_2_link" link2="torso_lift_link" reason="Never" />
    <disable_collisions link1="head_2_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="head_2_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="torso_fixed_column_link" link2="torso_fixed_link" reason="Default" />
    <disable_collisions link1="torso_fixed_column_link" link2="torso_lift_link" reason="Default" />
    <disable_collisions link1="torso_fixed_column_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="torso_fixed_column_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="torso_fixed_link" link2="torso_lift_link" reason="Adjacent" />
    <disable_collisions link1="torso_fixed_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="torso_fixed_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="torso_lift_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="torso_lift_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="wheel_left_link" link2="wheel_right_link" reason="Never" />

    <!-- Next disables generated with: https://gist.github.com/awesomebytes/18fe75b808c4c644bd3d -->
    <!-- Disabled because they are adjacent -->
    <!-- Tree looks like:
    base_footprint (NO COLLISION)
      base_link
      base_laser_link (NO COLLISION)
      base_antenna_left_link
      base_antenna_right_link
      base_imu_link (NO COLLISION)
      wheel_right_link
      wheel_left_link
        caster_front_right_1_link (NO COLLISION)
        caster_front_right_2_link
        caster_front_left_1_link (NO COLLISION)
        caster_front_left_2_link
        caster_back_right_1_link (NO COLLISION)
        caster_back_right_2_link
        caster_back_left_1_link (NO COLLISION)
        caster_back_left_2_link
        torso_fixed_link
          torso_lift_link
            head_1_link
              head_2_link
                xtion_link (NO COLLISION)
                xtion_optical_frame (NO COLLISION)
                  xtion_depth_frame (NO COLLISION)
                  xtion_depth_optical_frame (NO COLLISION)
                  xtion_rgb_frame (NO COLLISION)
                  xtion_rgb_optical_frame (NO COLLISION)
            arm_1_link
              arm_2_link
                arm_3_link
                  arm_4_link
                    arm_5_link
                      arm_6_link
                        arm_7_link
                          arm_tool_link




      torso_fixed_column_link
    base_cover_link (NO COLLISION)

     -->
    <disable_collisions link1="base_footprint" link2="base_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="base_laser_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="base_antenna_left_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="base_antenna_right_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="base_imu_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="wheel_right_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="wheel_left_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="caster_front_right_1_link" reason="Adjacent"/>
    <disable_collisions link1="caster_front_right_1_link" link2="caster_front_right_2_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="caster_front_left_1_link" reason="Adjacent"/>
    <disable_collisions link1="caster_front_left_1_link" link2="caster_front_left_2_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="caster_back_right_1_link" reason="Adjacent"/>
    <disable_collisions link1="caster_back_right_1_link" link2="caster_back_right_2_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="caster_back_left_1_link" reason="Adjacent"/>
    <disable_collisions link1="caster_back_left_1_link" link2="caster_back_left_2_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="torso_fixed_link" reason="Adjacent"/>
    <disable_collisions link1="torso_fixed_link" link2="torso_lift_link" reason="Adjacent"/>
    <disable_collisions link1="torso_lift_link" link2="head_1_link" reason="Adjacent"/>
    <disable_collisions link1="head_1_link" link2="head_2_link" reason="Adjacent"/>
    <disable_collisions link1="head_2_link" link2="xtion_link" reason="Adjacent"/>
    <disable_collisions link1="xtion_link" link2="xtion_optical_frame" reason="Adjacent"/>
    <disable_collisions link1="xtion_link" link2="xtion_depth_frame" reason="Adjacent"/>
    <disable_collisions link1="xtion_depth_frame" link2="xtion_depth_optical_frame" reason="Adjacent"/>
    <disable_collisions link1="xtion_link" link2="xtion_rgb_frame" reason="Adjacent"/>
    <disable_collisions link1="xtion_rgb_frame" link2="xtion_rgb_optical_frame" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="torso_fixed_column_link" reason="Adjacent"/>
    <disable_collisions link1="base_footprint" link2="base_cover_link" reason="Adjacent"/>

    <!-- Disabled because they don't have collision mesh so they can't collide between themselves-->
    <disable_collisions link1="base_laser_link" link2="base_footprint" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="base_cover_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="base_imu_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="xtion_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="xtion_optical_frame" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="xtion_depth_frame" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="xtion_depth_optical_frame" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="base_footprint" link2="base_cover_link" reason="Never" />
    <disable_collisions link1="base_footprint" link2="base_imu_link" reason="Never" />
    <disable_collisions link1="base_footprint" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="base_footprint" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="base_footprint" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="base_footprint" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="base_footprint" link2="xtion_link" reason="Never" />
    <disable_collisions link1="base_footprint" link2="xtion_optical_frame" reason="Never" />
    <disable_collisions link1="base_footprint" link2="xtion_depth_frame" reason="Never" />
    <disable_collisions link1="base_footprint" link2="xtion_depth_optical_frame" reason="Never" />
    <disable_collisions link1="base_footprint" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="base_footprint" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="base_imu_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="xtion_link" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="xtion_optical_frame" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="xtion_depth_frame" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="xtion_depth_optical_frame" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="xtion_link" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="xtion_optical_frame" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="xtion_depth_frame" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="xtion_depth_optical_frame" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="xtion_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="xtion_optical_frame" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="xtion_depth_frame" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="xtion_depth_optical_frame" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="xtion_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="xtion_optical_frame" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="xtion_depth_frame" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="xtion_depth_optical_frame" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="xtion_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="xtion_optical_frame" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="xtion_depth_frame" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="xtion_depth_optical_frame" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="xtion_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="xtion_optical_frame" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="xtion_depth_frame" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="xtion_depth_optical_frame" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="xtion_link" link2="xtion_optical_frame" reason="Never" />
    <disable_collisions link1="xtion_link" link2="xtion_depth_frame" reason="Never" />
    <disable_collisions link1="xtion_link" link2="xtion_depth_optical_frame" reason="Never" />
    <disable_collisions link1="xtion_link" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="xtion_link" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="xtion_optical_frame" link2="xtion_depth_frame" reason="Never" />
    <disable_collisions link1="xtion_optical_frame" link2="xtion_depth_optical_frame" reason="Never" />
    <disable_collisions link1="xtion_optical_frame" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="xtion_optical_frame" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="xtion_depth_frame" link2="xtion_depth_optical_frame" reason="Never" />
    <disable_collisions link1="xtion_depth_frame" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="xtion_depth_frame" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="xtion_depth_optical_frame" link2="xtion_rgb_frame" reason="Never" />
    <disable_collisions link1="xtion_depth_optical_frame" link2="xtion_rgb_optical_frame" reason="Never" />
    <disable_collisions link1="xtion_rgb_frame" link2="xtion_rgb_optical_frame" reason="Never" />

    <!-- Disables because the second links doesn't have collision mesh -->
    <disable_collisions link1="base_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="base_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="base_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="base_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="base_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="base_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="base_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="base_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="base_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="base_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="base_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="base_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="base_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="base_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>

@[for side in ['left', 'right']]@
@{
if side == "left":
    end_effector = end_effector_left
if side == "right":
    end_effector = end_effector_right
}@
    @[if repr("has_arm_" + side)]@
    <disable_collisions link1="arm_@(side)_1_link" link2="arm_@(side)_2_link" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_1_link" link2="arm_@(side)_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="arm_@(side)_4_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="arm_@(side)_5_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="arm_@(side)_6_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="arm_@(side)_7_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="arm_@(side)_tool_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="base_antenna_left_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="base_antenna_right_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="base_cover_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="base_laser_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="base_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="torso_fixed_column_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_1_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="torso_lift_link" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_1_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="arm_@(side)_3_link" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_2_link" link2="arm_@(side)_4_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="arm_@(side)_5_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="arm_@(side)_6_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="arm_@(side)_7_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="arm_@(side)_tool_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="base_antenna_left_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="base_antenna_right_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="base_cover_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="base_laser_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="base_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="head_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="torso_fixed_column_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_2_link" link2="torso_fixed_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="arm_@(side)_4_link" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_3_link" link2="arm_@(side)_5_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="arm_@(side)_6_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="arm_@(side)_7_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="arm_@(side)_tool_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="base_antenna_left_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="base_antenna_right_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="base_cover_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="base_laser_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="base_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="head_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="arm_@(side)_5_link" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_4_link" link2="arm_@(side)_6_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="arm_@(side)_7_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="arm_@(side)_tool_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="base_antenna_left_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="base_antenna_right_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="base_laser_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_front_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_front_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_front_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_front_right_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="wheel_left_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="arm_@(side)_6_link" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_5_link" link2="arm_@(side)_7_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_5_link" link2="arm_@(side)_tool_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="arm_@(side)_7_link" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_6_link" link2="arm_@(side)_tool_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="arm_@(side)_tool_link" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_7_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="base_antenna_left_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="caster_back_left_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="caster_back_left_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="caster_back_right_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="caster_back_right_2_link" reason="Never" />
    <disable_collisions link1="torso_lift_link" link2="arm_@(side)_1_link" reason="Adjacent"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="arm_@(side)_2_link" reason="Adjacent"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="arm_@(side)_3_link" reason="Adjacent"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="arm_@(side)_4_link" reason="Adjacent"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="arm_@(side)_5_link" reason="Adjacent"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="arm_@(side)_6_link" reason="Adjacent"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="arm_@(side)_7_link" reason="Adjacent"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="arm_@(side)_tool_link" reason="Adjacent"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="xtion_rgb_optical_frame" reason="Never"/>
@[end if]@
@[end for]@
@[if has_arm_left and has_arm_right]@
    <disable_collisions link1="arm_left_1_link" link2="arm_right_1_link" reason="Never" />
    <disable_collisions link1="arm_left_1_link" link2="arm_right_2_link" reason="Never" />
    <disable_collisions link1="arm_left_1_link" link2="arm_right_3_link" reason="Never" />
    <disable_collisions link1="arm_left_1_link" link2="arm_right_4_link" reason="Never" />
    <disable_collisions link1="arm_left_1_link" link2="arm_right_5_link" reason="Never" />
    <disable_collisions link1="arm_left_1_link" link2="arm_right_6_link" reason="Never" />
    <disable_collisions link1="arm_left_1_link" link2="arm_right_tool_link" reason="Never" />
    <disable_collisions link1="arm_left_2_link" link2="arm_right_1_link" reason="Never" />
    <disable_collisions link1="arm_left_2_link" link2="arm_right_2_link" reason="Never" />
    <disable_collisions link1="arm_left_2_link" link2="arm_right_3_link" reason="Never" />
    <disable_collisions link1="arm_left_2_link" link2="arm_right_4_link" reason="Never" />
    <disable_collisions link1="arm_left_2_link" link2="arm_right_5_link" reason="Never" />
    <disable_collisions link1="arm_left_2_link" link2="arm_right_6_link" reason="Never" />
    <disable_collisions link1="arm_left_2_link" link2="arm_right_tool_link" reason="Never" />
    <disable_collisions link1="arm_left_3_link" link2="arm_right_1_link" reason="Never" />
    <disable_collisions link1="arm_left_3_link" link2="arm_right_2_link" reason="Never" />
    <disable_collisions link1="arm_left_3_link" link2="arm_right_3_link" reason="Never" />
    <disable_collisions link1="arm_left_3_link" link2="arm_right_4_link" reason="Never" />
    <disable_collisions link1="arm_left_3_link" link2="arm_right_5_link" reason="Never" />
    <disable_collisions link1="arm_left_3_link" link2="arm_right_6_link" reason="Never" />
    <disable_collisions link1="arm_left_3_link" link2="arm_right_tool_link" reason="Never" />
    <disable_collisions link1="arm_left_4_link" link2="arm_right_1_link" reason="Never" />
    <disable_collisions link1="arm_left_4_link" link2="arm_right_2_link" reason="Never" />
    <disable_collisions link1="arm_left_4_link" link2="arm_right_3_link" reason="Never" />
    <disable_collisions link1="arm_left_4_link" link2="arm_right_4_link" reason="Never" />
    <disable_collisions link1="arm_left_4_link" link2="arm_right_5_link" reason="Never" />
    <disable_collisions link1="arm_left_4_link" link2="arm_right_6_link" reason="Never" />
    <disable_collisions link1="arm_left_4_link" link2="arm_right_tool_link" reason="Never" />
    <disable_collisions link1="arm_left_5_link" link2="arm_right_1_link" reason="Never" />
    <disable_collisions link1="arm_left_5_link" link2="arm_right_2_link" reason="Never" />
    <disable_collisions link1="arm_left_5_link" link2="arm_right_3_link" reason="Never" />
    <disable_collisions link1="arm_left_5_link" link2="arm_right_4_link" reason="Never" />
    <disable_collisions link1="arm_left_5_link" link2="arm_right_5_link" reason="Never" />
    <disable_collisions link1="arm_left_5_link" link2="arm_right_6_link" reason="Never" />
    <disable_collisions link1="arm_left_5_link" link2="arm_right_tool_link" reason="Never" />
    <disable_collisions link1="arm_left_6_link" link2="arm_right_1_link" reason="Never" />
    <disable_collisions link1="arm_left_6_link" link2="arm_right_2_link" reason="Never" />
    <disable_collisions link1="arm_left_6_link" link2="arm_right_3_link" reason="Never" />
    <disable_collisions link1="arm_left_6_link" link2="arm_right_4_link" reason="Never" />
    <disable_collisions link1="arm_left_6_link" link2="arm_right_5_link" reason="Never" />
    <disable_collisions link1="arm_left_6_link" link2="arm_right_6_link" reason="Never" />
    <disable_collisions link1="arm_left_6_link" link2="arm_right_tool_link" reason="Never" />
    <disable_collisions link1="arm_left_tool_link" link2="arm_right_1_link" reason="Never" />
    <disable_collisions link1="arm_left_tool_link" link2="arm_right_2_link" reason="Never" />
    <disable_collisions link1="arm_left_tool_link" link2="arm_right_3_link" reason="Never" />
    <disable_collisions link1="arm_left_tool_link" link2="arm_right_4_link" reason="Never" />
    <disable_collisions link1="arm_left_tool_link" link2="arm_right_5_link" reason="Never" />
    <disable_collisions link1="arm_left_tool_link" link2="arm_right_6_link" reason="Never" />
    <disable_collisions link1="arm_left_tool_link" link2="arm_right_tool_link" reason="Never" />
@[end if]@

@[for side in ['left', 'right']]@
@{
if side == "left":
    end_effector = end_effector_left
if side == "right":
    end_effector = end_effector_right
}@
@[if end_effector in ['pal-gripper', 'schunk-wsg']]@
    <disable_collisions link1="caster_back_left_1_link" link2="gripper_@(side)_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="gripper_@(side)_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="gripper_@(side)_link" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="gripper_@(side)_right_finger_link" reason="Default" />
    <disable_collisions link1="gripper_@(side)_link" link2="gripper_@(side)_right_finger_link" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_link" link2="gripper_@(side)_right_finger_link" reason="Adjacent"/>
    <disable_collisions link1="gripper_@(side)_link" link2="gripper_@(side)_left_finger_link" reason="Adjacent"/>
    <disable_collisions link1="gripper_@(side)_link" link2="gripper_@(side)_grasping_frame" reason="Adjacent"/>
    <disable_collisions link1="head_1_link" link2="gripper_@(side)_left_finger_link" reason="Never" />
    <disable_collisions link1="head_1_link" link2="gripper_@(side)_right_finger_link" reason="Never" />
    <disable_collisions link1="head_2_link" link2="gripper_@(side)_left_finger_link" reason="Never" />
    <disable_collisions link1="head_2_link" link2="gripper_@(side)_right_finger_link" reason="Never" />
    <disable_collisions link1="base_laser_link" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="base_footprint" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_link" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_optical_frame" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_depth_frame" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_depth_optical_frame" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_rgb_frame" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_rgb_optical_frame" link2="gripper_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="base_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_right_finger_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="gripper_@(side)_left_finger_link" link2="xtion_rgb_optical_frame" reason="Never"/>

    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_left_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_link" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_right_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_link" reason="Adjacent"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_left_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_right_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_left_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_right_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_left_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_right_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_left_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_right_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_left_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_right_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="gripper_@(side)_left_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="gripper_@(side)_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_7_link" link2="gripper_@(side)_right_finger_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="gripper_@(side)_grasping_frame" reason="Never"/>
  @[end if]@
@[end for]@

@[for side in ['left', 'right']]@
@{
if side == "left":
    end_effector = end_effector_left
    ft_sensor = ft_sensor_left
if side == "right":
    end_effector = end_effector_right
    ft_sensor = ft_sensor_right
}@
@[if end_effector in ["robotiq-2f-85", "robotiq-2f-140"]]@
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_base_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_base_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_base_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_base_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_base_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_base_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_coupler_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_base_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_coupler_link" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="base_antenna_right_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_base_link" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="base_sonar_01_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="base_sonar_02_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_base_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_base_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="gripper_@(side)_base_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="gripper_@(side)_coupler_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_@(side)_coupler_link" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_@(side)_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_@(side)_left_inner_knuckle" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_@(side)_left_outer_knuckle" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_@(side)_right_inner_knuckle" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_@(side)_right_outer_knuckle" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="gripper_right_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="gripper_@(side)_left_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="gripper_@(side)_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="wheel_right_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_@(side)_left_inner_finger_pad" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_@(side)_left_inner_knuckle" reason="Default" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_@(side)_left_outer_finger" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_right_coupler_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_right_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_right_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="gripper_right_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="gripper_@(side)_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="gripper_right_coupler_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="gripper_right_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_@(side)_left_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_@(side)_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_right_left_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_right_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_right_left_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_right_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_right_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="gripper_right_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_@(side)_left_outer_knuckle" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_right_base_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_right_coupler_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_right_left_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_right_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_right_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_right_right_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_right_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_right_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_right_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="gripper_right_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_@(side)_right_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_@(side)_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_right_base_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_right_left_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_right_left_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_right_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_right_left_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_right_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_right_right_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_right_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_right_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_right_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="gripper_right_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger" link2="gripper_@(side)_right_inner_finger_pad" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger" link2="gripper_@(side)_right_inner_knuckle" reason="Default" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger" link2="gripper_@(side)_right_outer_finger" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger" link2="gripper_right_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger" link2="gripper_right_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger_pad" link2="gripper_@(side)_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger_pad" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger_pad" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger_pad" link2="gripper_right_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger_pad" link2="gripper_right_right_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger_pad" link2="gripper_right_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger_pad" link2="gripper_right_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_knuckle" link2="gripper_@(side)_right_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_knuckle" link2="gripper_@(side)_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_knuckle" link2="gripper_right_left_inner_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_knuckle" link2="gripper_right_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_knuckle" link2="gripper_right_right_inner_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_knuckle" link2="gripper_right_right_inner_finger_pad" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_outer_finger" link2="gripper_@(side)_right_outer_knuckle" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_right_outer_finger" link2="gripper_right_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_outer_knuckle" link2="gripper_right_left_outer_finger" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_outer_knuckle" link2="gripper_right_left_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_outer_knuckle" link2="gripper_right_right_outer_knuckle" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_outer_knuckle" link2="wheel_right_link" reason="Never" />
@[end if]@

@[if end_effector == "pal-hey5"]@
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_index_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_index_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_index_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_index_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_index_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_index_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_index_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_mrl_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_palm_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_thumb_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_index_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_index_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_mrl_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_palm_link" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_safety_box" reason="Adjacent" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_thumb_link" reason="Default" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_palm_link" reason="Adjacent"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_safety_box" reason="Adjacent"/>
    <disable_collisions link1="arm_@(side)_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_3_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_4_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_5_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_6_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_7_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="arm_@(side)_tool_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="base_antenna_left_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_index_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_index_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="caster_front_left_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="caster_front_right_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_virtual_1_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_palm_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_virtual_1_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_virtual_2_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_index_virtual_2_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_index_virtual_3_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_index_virtual_3_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_mrl_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_palm_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_palm_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_virtual_1_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_palm_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_little_virtual_1_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_little_virtual_2_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_little_virtual_2_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_little_virtual_3_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_palm_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_virtual_1_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_palm_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_middle_virtual_1_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_middle_virtual_2_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_middle_virtual_3_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_palm_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_palm_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_ring_abd_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_ring_virtual_1_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_thumb_abd_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_thumb_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_virtual_1_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_ring_virtual_3_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_safety_box" reason="Default" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_safety_box" link2="hand_@(side)_thumb_abd_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_safety_box" link2="hand_@(side)_thumb_flex_1_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_safety_box" link2="hand_@(side)_thumb_flex_2_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_safety_box" link2="hand_@(side)_thumb_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_safety_box" link2="hand_@(side)_thumb_virtual_1_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_safety_box" link2="hand_@(side)_thumb_virtual_2_link" reason="Default" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Adjacent" />
    <disable_collisions link1="hand_@(side)_thumb_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_thumb_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_index_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_mrl_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_thumb_abd_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_index_abd_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_virtual_1_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_index_flex_1_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_virtual_2_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_index_flex_2_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_index_virtual_3_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_index_flex_3_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_middle_abd_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_virtual_1_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_middle_flex_1_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_middle_flex_2_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_middle_flex_3_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_ring_abd_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_virtual_1_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_ring_flex_3_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_little_abd_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_virtual_1_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_little_flex_1_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_little_virtual_2_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_little_flex_2_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_little_flex_3_link" reason="Adjacent"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_grasping_frame" reason="Adjacent"/>
    <disable_collisions link1="base_laser_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="base_footprint" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="base_cover_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="base_imu_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="caster_front_right_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="caster_front_left_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="caster_back_right_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="caster_back_left_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_optical_frame" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_depth_frame" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_depth_optical_frame" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_rgb_frame" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="xtion_rgb_optical_frame" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="base_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_left_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="base_antenna_right_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="wheel_right_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="wheel_left_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="caster_front_right_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="caster_front_left_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="caster_back_right_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="caster_back_left_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="torso_fixed_column_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="torso_lift_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="head_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="head_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_palm_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="base_laser_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="base_footprint" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="base_cover_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="base_imu_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="caster_front_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="caster_front_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="caster_back_right_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="caster_back_left_1_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="xtion_link" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="xtion_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="xtion_depth_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="xtion_depth_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="xtion_rgb_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="xtion_rgb_optical_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_safety_box" link2="hand_@(side)_grasping_frame" reason="Never"/>
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_abd_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_1_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_flex_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_1_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_index_virtual_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_abd_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_1_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_flex_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_1_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_little_virtual_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_abd_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_1_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_flex_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_1_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_middle_virtual_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_mrl_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_abd_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_1_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_flex_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_1_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_ring_virtual_3_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_abd_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_1_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_flex_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_virtual_1_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="hand_@(side)_grasping_frame" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_thumb_virtual_2_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_grasping_frame" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="hand_@(side)_grasping_frame" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="hand_@(side)_safety_box" link2="hand_@(side)_palm_link" reason="Never" />
@[end if]@


<!-- FT link disable collisions -->
@[if ft_sensor == "schunk-ft"]@
    <disable_collisions link1="wrist_@(side)_ft_link" link2="arm_@(side)_5_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="arm_@(side)_6_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="arm_@(side)_7_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="arm_@(side)_tool_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="arm_@(side)_5_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="arm_@(side)_6_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="arm_@(side)_7_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="arm_@(side)_tool_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="wrist_@(side)_ft_tool_link" reason="Adjacent" />

@[if end_effector in ['pal-gripper', 'schunk-wsg']]@
    <disable_collisions link1="wrist_@(side)_ft_link" link2="gripper_@(side)_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="gripper_@(side)_right_finger_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="gripper_@(side)_left_finger_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="gripper_@(side)_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="gripper_@(side)_right_finger_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="gripper_@(side)_left_finger_link" reason="Never" />
@[end if]@
@[if end_effector in ["robotiq-2f-85", "robotiq-2f-140"]]@
    <disable_collisions link1="gripper_@(side)_base_link" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_base_link" link2="wrist_@(side)_ft_tool_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_coupler_link" link2="wrist_@(side)_ft_tool_link" reason="Adjacent" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger" link2="wrist_@(side)_ft_tool_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_finger_pad" link2="wrist_@(side)_ft_tool_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_inner_knuckle" link2="wrist_@(side)_ft_tool_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_finger" link2="wrist_@(side)_ft_tool_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_left_outer_knuckle" link2="wrist_@(side)_ft_tool_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger" link2="wrist_@(side)_ft_tool_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger_pad" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_finger_pad" link2="wrist_@(side)_ft_tool_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_knuckle" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_inner_knuckle" link2="wrist_@(side)_ft_tool_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_outer_finger" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_outer_finger" link2="wrist_@(side)_ft_tool_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_outer_knuckle" link2="wrist_@(side)_ft_link" reason="Never" />
    <disable_collisions link1="gripper_@(side)_right_outer_knuckle" link2="wrist_@(side)_ft_tool_link" reason="Never" />
@[end if]@
@[if end_effector == "pal-hey5"]@
    <!-- Disable collisions with FT sensor -->
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_safety_box" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_palm_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_palm_link" reason="Adjacent" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_index_abd_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_tool_link" link2="hand_@(side)_safety_box" reason="Never" />

    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_index_abd_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_index_flex_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_index_flex_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_index_flex_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_index_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_index_virtual_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_index_virtual_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_index_virtual_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_little_abd_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_little_flex_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_little_flex_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_little_flex_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_little_virtual_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_little_virtual_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_little_virtual_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_middle_abd_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_middle_flex_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_middle_flex_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_middle_flex_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_middle_virtual_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_middle_virtual_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_middle_virtual_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_mrl_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_ring_abd_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_ring_flex_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_ring_flex_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_ring_flex_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_ring_virtual_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_ring_virtual_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_ring_virtual_3_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_thumb_abd_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_thumb_flex_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_thumb_flex_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_thumb_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_thumb_virtual_1_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_thumb_virtual_2_link" reason="Never" />
    <disable_collisions link1="wrist_@(side)_ft_link" link2="hand_@(side)_safety_box" reason="Never" />
@[end if]@
@[end if]@
@[end for]@


</robot>
