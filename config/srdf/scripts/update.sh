#!/bin/bash
set -e
pal_moveit_config_generator="$(rospack find pal_moveit_config_generator)"
tiago_moveit_srdf="$(rospack find tiago_moveit_config)/config/srdf"
source "$pal_moveit_config_generator/srdf_utils.sh" "$(dirname "${BASH_SOURCE[0]}")/../tiago_dual.srdf.xacro"

end_effectors=()
for end_effector_file in "$tiago_moveit_srdf"/end_effectors/*.srdf.xacro; do
     end_effectors+=($(basename "$end_effector_file" .srdf.xacro))
done
ft_sensors=(false schunk-ft)

# crawl all end effectors and generate the corresponding subtree SRDF
for end_effector in "${end_effectors[@]}"; do
    for ft_sensor in "${ft_sensors[@]}"; do
        args=("ft_sensor_left:=$ft_sensor" "ft_sensor_right:=$ft_sensor" end_effector_left:="$end_effector" end_effector_right:="$end_effector")
        for side in left right; do
            if [ "$ft_sensor" != false ]; then
                generate_disable_collisions_subtree "arm_${side}_tool_link" "${side}_${end_effector}_${ft_sensor}" "${side}_${end_effector}" "${args[@]}"
            else
                generate_disable_collisions_subtree "arm_${side}_tool_link" "${side}_${end_effector}"  "" "${args[@]}"
            fi
        done
    done
done

function get_name() {
    local end_effector=$1; shift
    local ft_sensor=$1; shift
            name=
            if [ "$ft_sensor" != false ]; then
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
    args=(base_type:="$base_type" ft_sensor_left:=false ft_sensor_right:=false end_effector_left:=false end_effector_right:=false)
    generate_disable_collisions "${prefix}_no-arm" "" "${args[@]}" arm_left:=false arm_right:=false # base & torso only
    generate_disable_collisions "${prefix}_no-arm-left" "${prefix}_no-arm" "${args[@]}" arm_left:=false arm_right:=true # base & torso & right arm
    generate_disable_collisions "${prefix}_no-arm-right" "${prefix}_no-arm" "${args[@]}" arm_left:=true arm_right:=false # base & torso & left arm
    generate_disable_collisions "${prefix}_no-ee_no-ee" "${prefix}_no-arm-left:${prefix}_no-arm-right" "${args[@]}" arm_left:=true arm_right:=true # base & torso & arms

    # Generate disable collision for single arm configurations
    for end_effector in "${end_effectors[@]}"; do
        for ft_sensor in "${ft_sensors[@]}"; do
            name="$(get_name "$end_effector" "$ft_sensor")"
            generate_srdf "${prefix}_${name}_no-arm-right" \
                          "${prefix}_no-arm-right:left_${name}" \
                          base_type:="$base_type" \
                          arm_left:=true \
                          arm_right:=false \
                          ft_sensor_left:="$ft_sensor"\
                          ft_sensor_right:=false \
                          end_effector_left:="$end_effector" \
                          end_effector_right:=false

            generate_srdf "${prefix}_no-arm-left_${name}" \
                          "${prefix}_no-arm-left:right_${name}" \
                          base_type:="$base_type" \
                          arm_left:=false \
                          arm_right:=true \
                          ft_sensor_left:=false \
                          ft_sensor_right:="$ft_sensor" \
                          end_effector_left:=false \
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
                                  arm_left:=true \
                                  arm_right:=true \
                                  ft_sensor_left:="$ft_sensor_left" \
                                  ft_sensor_right:="$ft_sensor_right" \
                                  end_effector_left:="$end_effector_left" \
                                  end_effector_right:="$end_effector_right"
                done
            done
        done
    done
done
