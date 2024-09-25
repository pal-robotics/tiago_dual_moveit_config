#!/bin/bash
set -e
set -o pipefail
pal_moveit_config_generator=$(ros2 pkg prefix pal_moveit_config_generator)
tiago_dual_moveit_srdf="$(ros2 pkg prefix tiago_dual_moveit_config)/share/tiago_dual_moveit_config/config/srdf"
source "$pal_moveit_config_generator/share/pal_moveit_config_generator/srdf_utils.sh" "$(dirname "${BASH_SOURCE[0]}")/../tiago_dual.srdf.xacro"

end_effectors=()
for end_effector_file in "$tiago_dual_moveit_srdf"/end_effectors/*.srdf.xacro; do
     end_effectors+=($(basename "$end_effector_file" .srdf.xacro))
done
ft_sensors=(no-ft-sensor schunk-ft)

# crawl all end effectors and generate the corresponding subtree SRDF
for end_effector in "${end_effectors[@]}"; do
    if [ "$end_effector" = "no-end-effector" ]; then
        end_effector_value="no-ee"
    else
        end_effector_value="$end_effector"
    fi


    for ft_sensor in "${ft_sensors[@]}"; do
        args=("ft_sensor_left:=$ft_sensor" "ft_sensor_right:=$ft_sensor" end_effector_left:="$end_effector" end_effector_right:="$end_effector")
        for side in left right; do
            if [ "$ft_sensor" != "no-ft-sensor" ]; then
                generate_disable_collisions_subtree "arm_${side}_tool_link" "${side}_${end_effector_value}_${ft_sensor}" "${side}_${end_effector_value}" "${args[@]}"
            else
                generate_disable_collisions_subtree "arm_${side}_tool_link" "${side}_${end_effector_value}"  "" "${args[@]}"
            fi
        done
    done
done

function get_name() {
    local end_effector=$1; shift
    local ft_sensor=$1; shift
            if [ "$end_effector" = "no-end-effector" ]; then
                end_effector="no-ee"
            fi

            name=
            if [ "$ft_sensor" != "no-ft-sensor" ]; then
                echo "${end_effector}_$ft_sensor"
            else
                echo "${end_effector}"
            fi
}

for base_type in pmb2 omni_base ; do
    # Generate base disable collision pairs
    prefix="${robot}"
    if [ "$base_type" = "omni_base" ]; then
        prefix="${robot}_omni"
    fi
    args=(base_type:="$base_type" ft_sensor_left:="no-ft-sensor" ft_sensor_right:="no-ft-sensor" end_effector_left:="no-end-effector" end_effector_right:="no-end-effector")
    generate_disable_collisions "${prefix}_no-arm-left_no-arm-right" "" "${args[@]}" arm_type_left:="no-arm" arm_type_right:="no-arm" # base & torso only
    generate_disable_collisions "${prefix}_no-arm-left" "${prefix}_no-arm-left_no-arm-right" "${args[@]}" arm_type_left:="no-arm" # base & torso & right arm
    generate_disable_collisions "${prefix}_no-arm-right" "${prefix}_no-arm-left_no-arm-right" "${args[@]}" arm_type_right:="no-arm" # base & torso & left arm
    generate_disable_collisions "${prefix}_no-ee_no-ee" "${prefix}_no-arm-left:${prefix}_no-arm-right" "${args[@]}" # base & torso & arms

    # Generate disable collision for single arm configurations
    for end_effector in "${end_effectors[@]}"; do
        for ft_sensor in "${ft_sensors[@]}"; do
            name="$(get_name "$end_effector" "$ft_sensor")"
            generate_srdf "${prefix}_${name}_no-arm-right" \
                          "${prefix}_no-arm-right:left_${name}" \
                          base_type:="$base_type" \
                          arm_type_right:="no-arm" \
                          ft_sensor_left:="$ft_sensor"\
                          ft_sensor_right:="no-ft-sensor" \
                          end_effector_left:="$end_effector" \
                          end_effector_right:="no-end-effector"

            generate_srdf "${prefix}_no-arm-left_${name}" \
                          "${prefix}_no-arm-left:right_${name}" \
                          base_type:="$base_type" \
                          arm_type_left:="no-arm" \
                          ft_sensor_left:="no-ft-sensor" \
                          ft_sensor_right:="$ft_sensor" \
                          end_effector_left:="no-end-effector" \
                          end_effector_right:="$end_effector"
        done
    done

    # Generate disable collision for dual arm configurations
    for end_effector_left in "${end_effectors[@]}"; do
        for ft_sensor_left in "${ft_sensors[@]}"; do
            for end_effector_right in "${end_effectors[@]}"; do
                for ft_sensor_right in "${ft_sensors[@]}"; do
                    left_name="$(get_name "$end_effector_left" "$ft_sensor_left")"
                    right_name="$(get_name "$end_effector_right" "$ft_sensor_right")"
                    generate_srdf "${prefix}_${left_name}_${right_name}" \
                                  "${prefix}_no-ee_no-ee:${prefix}_${left_name}_no-arm-right:${prefix}_no-arm-left_${right_name}" \
                                  base_type:="$base_type" \
                                  ft_sensor_left:="$ft_sensor_left" \
                                  ft_sensor_right:="$ft_sensor_right" \
                                  end_effector_left:="$end_effector_left" \
                                  end_effector_right:="$end_effector_right"
                done
            done
        done
    done
done
