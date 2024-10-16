joy_teleop:
  ros__parameters:
    move:
      type: topic
      interface_type: geometry_msgs/msg/Twist
      topic_name: cmd_vel
      deadman_buttons: [5]
      axis_mappings:
        linear-x:
          axis: 1
          scale: 1.0
          offset: 0.0
        angular-z:
          axis: 2
          scale: 1.0
          offset: 0.0

    joy_priority:
      type: action
      interface_type: twist_mux_msgs/action/JoyPriority
      action_name: joy_priority_action
      buttons: [9]

    joy_turbo_decrease:
      type: action
      interface_type: twist_mux_msgs/action/JoyTurbo
      action_name: joy_turbo_decrease
      buttons: [8, 1]

    joy_turbo_increase:
      type: action
      interface_type: twist_mux_msgs/action/JoyTurbo
      action_name: joy_turbo_increase
      buttons: [8, 3]

    joy_turbo_angular_decrease:
      type: action
      interface_type: twist_mux_msgs/action/JoyTurbo
      action_name: joy_turbo_angular_decrease
      buttons: [8, 2]

    joy_turbo_angular_increase:
      type: action
      interface_type: twist_mux_msgs/action/JoyTurbo
      action_name: joy_turbo_angular_increase
      buttons: [8, 0]

    joy_turbo_reset:
      type: action
      interface_type: twist_mux_msgs/action/JoyTurbo
      action_name: joy_turbo_reset
      buttons: [10, 11]

    torso_up:
      type: action
      interface_type: teleop_tools_msgs/action/Increment
      action_name: /torso_controller/increment
      action_goal:
        increment_by: [0.05]
      buttons: [4] # right pad, top button

    torso_down:
      type: action
      interface_type: teleop_tools_msgs/action/Increment
      action_name: /torso_controller/increment
      action_goal:
        increment_by: [-0.05]
      buttons: [6] # right pad, bottom button

    head_down:
      type: action
      interface_type: teleop_tools_msgs/action/Increment
      action_name: /head_controller/increment
      action_goal:
        increment_by: [-0.1, 0.0]
      buttons: [2] # right pad, bottom button

    head_up:
      type: action
      interface_type: teleop_tools_msgs/action/Increment
      action_name: /head_controller/increment
      action_goal:
        increment_by: [0.1, 0.0]
      buttons: [0] # right pad, top button

    head_left:
      type: action
      interface_type: teleop_tools_msgs/action/Increment
      action_name: /head_controller/increment
      action_goal:
        increment_by: [0.0, 0.1]
      buttons: [3] # right pad, left button

    head_right:
      type: action
      interface_type: teleop_tools_msgs/action/Increment
      action_name: /head_controller/increment
      action_goal:
        increment_by: [0.0, -0.1]
      buttons: [1] # right pad, right button

@[if end_effector == "pal-hey5"]@
    close_hand:
      type: action
      interface_type: play_motion_msgs/action/PlayMotion
      action_name: /play_motion
      action_goal:
        motion_name: 'close_hand'
        skip_planning: True
      buttons: [7]

    open_hand:
      type: action
      interface_type: play_motion_msgs/action/PlayMotion
      action_name: /play_motion
      action_goal:
        motion_name: 'open_hand'
        skip_planning: True
      buttons: [5]
@[end if]@

@[if end_effector == "pal-gripper"]@
    close_gripper:
      type: action
      interface_type: teleop_tools_msgs/action/Increment
      action_name: /gripper_controller/increment
      action_goal:
        increment_by: [-0.01, -0.01]
      buttons: [7] # R2

    open_gripper:
      type: action
      interface_type: teleop_tools_msgs/action/Increment
      action_name: /gripper_controller/increment
      action_goal:
        increment_by: [0.01, 0.01]
      buttons: [5] # R1
@[end if]@
