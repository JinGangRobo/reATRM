#/bin/zsh

source install/setup.sh

ros2 topic pub -r 10 /decision_num rm_decision_interfaces/msg/DecisionNum "{
    decision_num: 1,
}" &

ros2 topic pub -r 10 /robot_status rm_decision_interfaces/msg/RobotStatus "{
    robot_id: 7,
    current_hp: 600,
    shooter_heat: 0,
    team_color: 0,
    is_attacked: 0,
}" &

ros2 topic pub -r 1 /game_status rm_decision_interfaces/msg/GameStatus "{
    game_progress: 4, # 4:比赛开始
    stage_remain_time: 220,
}" &

ros2 topic pub -r 3 /robot_hp rm_decision_interfaces/msg/AllRobotHP "{
    red_1_robot_hp: 100,
    red_2_robot_hp: 100,
    red_3_robot_hp: 100,
    red_4_robot_hp: 100,
    red_5_robot_hp: 200,
    red_7_robot_hp: 200,
    red_outpost_hp: 20,
    red_base_hp: 1000,
    blue_1_robot_hp: 100,
    blue_2_robot_hp: 100,
    blue_3_robot_hp: 100,
    blue_4_robot_hp: 200,
    blue_5_robot_hp: 200,
    blue_7_robot_hp: 200,
    blue_outpost_hp: 1000,
    blue_base_hp: 1000
}" &

ros2 topic pub -r 5 /detector/armors auto_aim_interfaces/msg/Armors "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'my_frame'
  },

  # armors: [ # 取消注释，代表识别到敌人
  #   {
  #     number: '1',
  #     type: '0',
  #     distance_to_image_center: 1.0,
  #     pose: {
  #       position: {x: 0.0, y: 0.0, z: 0.0},
  #       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  #     }
  #   }
  # ]
}" &

wait
