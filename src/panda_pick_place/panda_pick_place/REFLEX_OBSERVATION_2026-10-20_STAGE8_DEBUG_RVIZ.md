# Panda 제어·TF + pick and place + Panda 디버깅 RViz 5분 관찰

- 시간대: Asia/Seoul
- 상태: 5분 로그 확인 구간 완료; 동작은 reflex로 중단되어 5분 연속 동작을 완료하지 못함
- 구성: Panda 제어·TF + pick and place + Panda 디버깅 RViz (카메라·좌표 정렬·사람 인식·nvblox 제외)
- RViz 설정(사용자 제공): ~/panda_real_ws/src/my_people_nvblox_bringup/config/visualization/panda_sphere_debug.rviz
- RViz OpenGL 초기화 확인: 2026-10-20T17:05:15.124+09:00
- 구성 근거: 사용자 제공 실행 명령 및 로그. 이전 카메라·인식 구성에는 종료 기록이 있고 이번 구성에 신규 인식 시작 로그는 확인되지 않았음. 프로세스 전수 검사는 수행하지 않음.
- 첫 home 동작 요청: 2026-10-20T17:05:13.802+09:00
- 관찰 기준 시작: 2026-10-20T17:05:16.000+09:00
- 관찰 예정 종료: 2026-10-20T17:10:16.000+09:00
- 마지막 확인: 2026-10-20T17:10:16.007+09:00
- 관찰 구간 경과: 300초
- 결과: reflex 기록 발견
- 최대 Cycle 시작 번호: 6 (완료 횟수 또는 실제 물체 이송 성공 횟수를 의미하지 않음)
- reflex 관련 로그 줄 수: 96 (반복 경고 포함; 개별 발생 횟수가 아님)
- 기타 오류/종료 이벤트 줄 수: 15
- 방법: 약 5초 간격으로 로그만 읽음. 관찰 도구는 로봇 명령이나 복구를 실행하지 않음.
- pick and place 자체는 매 이동 전 오류 복구 서비스를 호출하는 구현이므로, 동작이 이어져도 reflex 기록을 별도로 확인함.
- 이전 단독 동작 실험과 별도 실행이며, 이번 Panda launch는 17:05:03에 시작함.
- 한계: 로그 기반 결과이며 실제 관절 움직임·물체 이송 성공·제어 통신 성공률을 별도로 측정한 결과는 아님.

## 마지막 동작 로그

```text
[ERROR] [1792483646.321313392] [pick_place_node]: Trajectory to pick_above failed
```

## 이벤트

- 최초 reflex 경고: 2026-10-20T17:07:21.412+09:00
- 관찰 시작 이후: 125.413초

```text
[WARN] [1792483641.412781422] [FrankaHardwareInterface]: Reflex error not recovered - arm won't move. Call ~/error_recovery.
```

- Panda 사용자 종료: 2026-10-20T17:08:57.253+09:00 (이후 시간은 연속 동작 시간에 포함하지 않음)

- 2026-10-20T17:07:26.295+09:00: `[ERROR] [1792483646.295192089] [tolerances]: State tolerances failed for joint 0:`
- 2026-10-20T17:07:26.295+09:00: `[ERROR] [1792483646.295313783] [tolerances]: Position Error: 0.633240, Position Tolerance: 0.050000`
- 2026-10-20T17:07:26.321+09:00: `[ERROR] [1792483646.321313392] [pick_place_node]: Trajectory to pick_above failed`
- 2026-10-20T17:07:26.295+09:00: `[WARN] [1792483646.295356407] [panda_arm_controller]: Aborted due to goal_time_tolerance exceeding by 2.000388 seconds`
- 2026-10-20T17:08:55.757+09:00: `[INFO] [1792483735.757411498] [rclcpp]: signal_handler(SIGINT/SIGTERM)`
- 2026-10-20T17:08:57.253+09:00: `[INFO] [1792483737.253238741] [rclcpp]: signal_handler(SIGINT/SIGTERM)`
- 2026-10-20T17:08:57.253+09:00: `[INFO] [1792483737.253461041] [controller_manager]: Shutting down all controllers in the controller manager.`
- 2026-10-20T17:08:57.253+09:00: `[INFO] [1792483737.253595630] [controller_manager]: Shutting down controller 'franka_robot_state_broadcaster'`
- 2026-10-20T17:08:57.253+09:00: `[INFO] [1792483737.253699533] [controller_manager]: Shutting down controller 'panda_arm_controller'`
- 2026-10-20T17:08:57.253+09:00: `[INFO] [1792483737.253771595] [controller_manager]: Shutting down controller 'joint_state_broadcaster'`
- 2026-10-20T17:08:57.255+09:00: `[INFO] [1792483737.255601356] [controller_manager]: Shutting down the controller manager.`
- 2026-10-20T17:08:57.253+09:00: `1792483737.2536917 [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)`
- 2026-10-20T17:08:57.253+09:00: `1792483737.2626145 [franka_gripper_node-7] [INFO] [1792483737.253240469] [rclcpp]: signal_handler(SIGINT/SIGTERM)`
- 2026-10-20T17:08:57.417+09:00: `1792483737.4175816 [INFO] [launch]: process[franka_control2_node-3] was required: shutting down launched system`
- 2026-10-20T17:08:57.253+09:00: `[INFO] [1792483737.253240469] [rclcpp]: signal_handler(SIGINT/SIGTERM)`

## 증거

- `/home/esjin/.ros/log/franka_control2_node_229719_1792483504083.log`
- `/home/esjin/.ros/log/python3_229900_1792483510467.log`
- `/home/esjin/.ros/log/2026-10-20-17-05-03-351440-jetson-229708/launch.log`
- `/home/esjin/.ros/log/franka_gripper_node_229727_1792483504070.log`
- `/home/esjin/.ros/log/rviz2_229938_1792483514679.log`

- 전체 이벤트/상태: `reflex_observation_2026-10-20_stage8_debug_rviz.jsonl`
- 원본 사본: `panda_stage8_debug_rviz_control_2026-10-20.log`
- 원본 사본: `panda_stage8_debug_rviz_pick_place_2026-10-20.log`
- 원본 사본: `panda_stage8_debug_rviz_launch_2026-10-20.log`
- 원본 사본: `panda_stage8_debug_rviz_gripper_2026-10-20.log`
- 원본 사본: `panda_stage8_debug_rviz_rviz_2026-10-20.log`

## 최종 요약

- 최초 reflex 경고: 17:07:21.412 KST.
- 세 구성 동시 관찰 시작(17:05:16)부터 약 2분 5초, 첫 home 동작(17:05:13.802)부터 약 2분 8초 후 최초 경고 확인.
- 발생 단계: Cycle 6의 시작에서 pick_above로 이동하던 중.
- 17:07:26.321에 Trajectory to pick_above failed 기록.
- RViz에는 17:08:55 종료 신호, Panda launch에는 17:08:57 사용자 Ctrl-C가 기록됨. 종료 이후는 연속 동작 시간으로 보지 않음.
- 이번 저장 로그에는 최초 abort의 오류 세부 코드와 control_command_success_rate가 없어 특정하지 않음.
- 반복되는 복구 미완료 경고는 개별 reflex 발생 횟수를 의미하지 않음.
