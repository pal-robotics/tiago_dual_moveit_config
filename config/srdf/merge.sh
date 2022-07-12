#!/bin/bash
set -e

this_folder=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
tiago_moveit_srdf="$(rospack find tiago_moveit_config)/config/srdf"
source "$tiago_moveit_srdf/generate_srdf.sh" "$this_folder/tiago_dual.srdf.xacro"

ref=${1:-HEAD}

for f in "$tiago_moveit_srdf"/end_effectors/*.srdf.xacro; do
    end_effector=$(basename "$f" .srdf.xacro)
    end_effector_name=$(grep -Po '<xacro:property name="end_effector_name" value="\K[^"]+' "$f" || echo "gripper")
    for tiago in tiago_dual tiago_dual_omni; do
        add_diff_matrix_to_xacro_from_ref "$ref" "${tiago}_${end_effector}_no-arm-right.srdf" "${end_effector_name}_left" "disable_collisions/left_$end_effector.srdf.xacro"
        add_diff_matrix_to_xacro_from_ref "$ref" "${tiago}_no-arm-left_${end_effector}.srdf" "${end_effector_name}_right" "disable_collisions/right_$end_effector.srdf.xacro"
    done
done

for f in "$srdf_folder"/disable_collisions/*.srdf.xacro; do
    p=${f#"$srdf_folder"/}
    add_diff_matrix_to_xacro_from_ref "$ref" "$p" "" "$p"
done
