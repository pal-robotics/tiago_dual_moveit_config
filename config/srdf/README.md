These are generated manually, on demand because there are over 40 combinations.

To generate one, execute moveit assistant:
`roslaunch moveit_setup_assistant setup_assistant.launch`

Select edit existing package and select this package.
On xacro arguments use the variation of the following params that you want to use:
`--inorder end_effector_left:=pal-gripper end_effector_right:=pal-hey5 ft_sensor_left:=schunk-ft ft_sensor_right:=schunk-ft laser_model:=sick-571 camera_model:=orbbec-astra`

Click on **Load Files**
Go to the Self Collision section, select the max amount of Highest Sampling Density, and Generate Collision Matrix.

Export only the modified srdf and copy it with the appropiate name in this directory.
