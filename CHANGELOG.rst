^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_dual_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.4 (2022-03-18)
------------------
* Merge branch 'mlu/feature/omni-base-srdf' into 'erbium-devel'
  Add generated SRDFs for omni_base and epick
  See merge request robots/tiago_dual_moveit_config!19
* Add vacuum joint to end effector group of Robotiq EPick
* Add config files for the controller of the epick
* Add SRDFs for robotiq-epick
* Add generated SRDFs for omni_base
* Patch launch files to offer all configuration options
* Merge branch 'mlu/fix/python3-repr' into 'erbium-devel'
  Use repr() instead of backticks
  See merge request robots/tiago_dual_moveit_config!16
* Use repr() instead of backticks
  backticks are deprecated in python2.7 and got removed in python3
* Contributors: Mathias Lüdtke, davidfernandez, saikishor, thomaspeyrucain

0.4.3 (2021-12-02)
------------------
* Merge branch 'add_base_type' into 'erbium-devel'
  Add base_type argument to the moveit launch files
  Closes #1
  See merge request robots/tiago_dual_moveit_config!15
* Add base_type argument to the moveit launch files
* Contributors: Sai Kishor Kothakota, victor

0.4.2 (2021-05-21)
------------------
* Merge branch 'remove_grasping_frame' into 'erbium-devel'
  chore: removed the non-existing grasping_frame
  See merge request robots/tiago_dual_moveit_config!14
* chore: removed the non-existing grasping_frame
* Add note about no_safety_eps
* Contributors: Victor Lopez, saikishor, yueerro

0.4.1 (2021-05-18)
------------------
* Fixes to demo.launch
* Contributors: Victor Lopez

0.4.0 (2021-05-06)
------------------
* Merge branch 'robotiq_gripper' into 'erbium-devel'
  Robotiq gripper
  See merge request robots/tiago_dual_moveit_config!13
* generated the controllers config files for new combinations of the robotiq gripper 85 and 140
* added the combinations of the robotiq gripper SRDF
* Contributors: Sai Kishor Kothakota, saikishor

0.3.17 (2021-04-13)
-------------------
* Merge branch 'custom-end-effector' into 'erbium-devel'
  Custom end effector
  See merge request robots/tiago_dual_moveit_config!11
* fix: controller not working if only one ee is custom
* fix: look for proper combiation of ee
* feat: enable custom end effector
* Contributors: daniellopez, jordanpalacios

0.3.16 (2020-12-14)
-------------------
* Merge branch 'fix-gripper-parent' into 'erbium-devel'
  Fix gripper parent group
  See merge request robots/tiago_dual_moveit_config!10
* Fix gripper parent group
* Add Readme
* Contributors: Victor Lopez, davidfernandez, victor

0.3.15 (2020-10-01)
-------------------
* Merge branch 'hey5_marker' into 'erbium-devel'
  Hey5 marker
  See merge request robots/tiago_dual_moveit_config!9
* Autogenerate srdf files
* Add links for hey5 group
* Contributors: Adria Roig, victor

0.3.14 (2020-05-08)
-------------------
* Regenerate srdf after wrist changes
* Contributors: Victor Lopez

0.3.13 (2020-04-21)
-------------------
* Merge branch 'templatize-srdf' into 'erbium-devel'
  Templatize SRDF generation
  See merge request robots/tiago_dual_moveit_config!7
* Merge branch 'more-templatize-srdf' into 'templatize-srdf'
  Reuse generator code for both sides
  See merge request robots/tiago_dual_moveit_config!8
* Reuse generator code for both sides
* Templatize SRDF generation
* Contributors: Victor Lopez, davidfernandez, victor

0.3.12 (2020-04-16)
-------------------
* Update srdf file
* Added another srdf file
* Contributors: Victor Lopez

0.3.11 (2020-04-08)
-------------------
* Merge branch 'add-arm-sides' into 'erbium-devel'
  Add arm sides
  See merge request robots/tiago_dual_moveit_config!6
* Add head controller
* Add no arm versions
* Contributors: Victor Lopez, victor

0.3.10 (2020-03-23)
-------------------
* Add more srdfs
* Contributors: Victor Lopez

0.3.9 (2020-03-23)
------------------
* Add new srdfs
* Contributors: Victor Lopez

0.3.8 (2020-01-17)
------------------
* Merge branch 'moveit_fix' into 'erbium-devel'
  fix moveit cartesian goals issue on robot
  See merge request robots/tiago_dual_moveit_config!5
* fix moveit cartesian goals issue on robot
* Contributors: Sai Kishor Kothakota

0.3.7 (2019-08-07)
------------------
* Merge branch 'fix_moveit_camera' into 'erbium-devel'
  Fixed the parameters for the moveit camera use for the octomap
  See merge request robots/tiago_dual_moveit_config!4
* Fixed the parameters for the moveit camera use for the octomap
* Contributors: Victor Lopez, alessandrodifava

0.3.6 (2019-07-31)
------------------
* Merge branch 'fix-gripper-controller-name' into 'erbium-devel'
  Fix controller name
  See merge request robots/tiago_dual_moveit_config!3
* Fix controller name
* Contributors: Victor Lopez

0.3.5 (2019-07-08)
------------------
* Add srdf for gripper + gripper FT
* Merge branch 'fix_ikinematic_solver_config' into 'erbium-devel'
  Add KDL config for arms groups
  See merge request robots/tiago_dual_moveit_config!1
* Add KDL config for arms groups
* Contributors: Luca Marchionni, Victor Lopez

0.3.4 (2019-04-16)
------------------
* Fix typos
* Remove unused controllers
* f
* Add contorllers.yaml.em
* Contributors: Victor Lopez

0.3.3 (2019-04-15)
------------------
* Add gripper/gripper srdf
* Regenerate for more end effector combinations
* Regenerate for hey5
* Contributors: Victor Lopez

0.3.2 (2019-03-26)
------------------
* Remove description dependency, it's not needed
* Contributors: Victor Lopez

0.3.1 (2019-03-26)
------------------
* Work in progress
* Initial commit
* Contributors: Victor Lopez
