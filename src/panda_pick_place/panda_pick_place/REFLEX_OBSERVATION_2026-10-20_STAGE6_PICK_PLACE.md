# Panda + RealSense + 좌표 정렬 + 사람 인식·nvblox + RViz + pick and place 관찰

- 날짜/시간대: 2026-10-20, Asia/Seoul
- 상태: 예정된 로그 확인 구간 완료; pick and place는 reflex 후 중단되어 5분 연속 동작을 완료하지 못함
- 구성: Panda tf/제어 + RealSense + ArUco 정적 좌표 정렬 + 사람 인식·nvblox + RViz + pick and place
- pick and place 명령(사용자 제공): ros2 run panda_pick_place pick_place_node --ros-args --params-file ~/panda_real_ws/src/panda_pick_place/config/pick_place.yaml
- 설정 파일에서 확인: repeat=0, max_joint_speed=0.2 rad/s, use_gripper=true
- 첫 home 명령 로그: 2026-10-20T15:41:07+09:00 (epoch 1792478467.510840988)
- pick and place는 매 이동 전 오류 복구 서비스를 호출하는 구현임. 복구 뒤 동작이 이어져도 reflex 발생 기록은 유지함.
- 추가 실행 확인: panda_voxel_classifier, closest_panda_human_voxels
- 확인된 global_frame: panda_link0
- RViz OpenGL 초기화 로그: 2026-10-20T15:40:57+09:00
- USB-C 배치(사용자 제공): DC 잭 위쪽에 로봇 USB Ethernet, 40핀 옆에 전원
- 확인한 카메라 설정: RGB/Depth 424×240, 30 FPS
- 확인한 추론 엔진: model_vanilla_v2_0_2_424x240.plan
- 사람 인식 launch 시작: 2026-10-20T15:40:49+09:00
- 추론 노드 시작 로그: 2026-10-20T15:40:52+09:00 (epoch 1792478452.072547664)
- 관찰 기준 시작: 2026-10-20T15:41:08+09:00
- 관찰 예정 종료: 2026-10-20T15:46:08+09:00
- 마지막 로그 확인: 2026-10-20T15:46:08+09:00
- 관찰 구간 경과: 300초
- 결과: Cycle 4의 grasp 이후 pick_above 이동 중 reflex 발생
- 최초 경고: 2026-10-20T15:42:32+09:00; 관찰 시작 후 84.491초 (약 1분 24초)
- 사용자 터미널 로그의 오류 코드: communication_constraints_violation
- 사용자 터미널 로그의 control_command_success_rate: 0.19
- reflex 관련 로그 줄 수: 39 (같은 사건이 여러 로그에 기록될 수 있음)
- 관찰 방법: 아래 파일들을 약 5초 간격으로 읽음. 로봇 명령·복구·추가 성능 측정은 실행하지 않음.
- 한계: 로그 기반 관찰이며 실제 제어 통신 성공률이나 전체 프레임 수신을 별도로 측정한 결과는 아님.

## 이벤트와 해석

1. 15:41:07.511: pick and place 첫 home 명령 시작.
2. 15:42:21.942: Cycle 4 시작. 로그상 Cycle 1~3의 단계들을 지나 Cycle 4로 진입함.
3. 15:42:29.906: Cycle 4 gripper grasp 요청.
4. 15:42:32.098: pick_above로 상승하는 이동 요청(목표 시간 2.0초).
5. 15:42:32.491: 최초 `Reflex error not recovered` 경고. 이 시각은 복구 미완료 상태의 최초 확인이며 정확한 abort 순간의 타임스탬프는 아님.
6. 15:42:36.100: 관절 1·3·5의 위치 허용오차 초과 기록. 사용자 터미널 로그에는 `goal_time_tolerance`를 2.000875초 초과했다는 메시지도 있음.
7. 15:42:36.157: pick and place 노드가 `Trajectory to pick_above failed`를 기록하고 반복 진행을 멈춤.
8. 15:43:10.625: Panda launch에 사용자 Ctrl-C 종료가 기록됨. 나머지 구성도 15:43:11~19에 종료 로그가 기록됨.
9. 15:46:08: 예정했던 5분 로그 확인 구간 종료. 종료 이후 시간은 로봇 연속 동작 시간에 포함하지 않음.

사용자가 제공한 터미널 로그에서 최초 중단 이유가 `communication_constraints_violation`, 당시 `control_command_success_rate`가 `0.19`임을 확인함. 이 두 줄은 저장된 ROS 제어 로그에 없어 별도 사용자 제공 증거로 보존함.

로그 순서는 통신 관련 reflex가 먼저 발생한 뒤 trajectory 허용오차 초과 및 이동 실패가 기록된 순서임. 이 순서만으로 통신 지연의 근본 원인이나 특정 노드의 인과관계까지 확정하지 않음.

- 최초 경고 원본: `/home/esjin/.ros/log/franka_control2_node_192533_1792478432234.log`

```text
[WARN] [1792478552.490529275] [FrankaHardwareInterface]: Reflex error not recovered - arm won't move. Call ~/error_recovery.
```

- 복구 미완료 경고 39줄은 반복 상태 메시지이며 개별 reflex 발생 횟수가 아님.
- 마지막 복구 미완료 경고: 2026-10-20T15:43:10+09:00
- 전체 이벤트와 시각은 JSONL에 보존함.
- 사용자 제공 터미널 발췌: `pick_place_stage6_user_terminal_2026-10-20.log`
- pick and place 진행 로그 사본: `pick_place_stage6_2026-10-20.log`

## 원본 로그

- `/home/esjin/.ros/log/franka_control2_node_192533_1792478432234.log`
- `/home/esjin/.ros/log/2026-10-20-15-40-31-501590-jetson-192521/launch.log`
- `/home/esjin/.ros/log/component_container_mt_192706_1792478438759.log`
- `/home/esjin/.ros/log/2026-10-20-15-40-38-545484-jetson-192705/launch.log`
- `/home/esjin/.ros/log/python3_192800_1792478445064.log`
- `/home/esjin/.ros/log/2026-10-20-15-40-43-487531-jetson-192793/launch.log`
- `/home/esjin/.ros/log/component_container_mt_192890_1792478449556.log`
- `/home/esjin/.ros/log/2026-10-20-15-40-49-062414-jetson-192857/launch.log`
- `/home/esjin/.ros/log/python3_192886_1792478450199.log`
- `/home/esjin/.ros/log/python3_192888_1792478450980.log`
- `/home/esjin/.ros/log/rviz2_193605_1792478456183.log`
- `/home/esjin/.ros/log/python3_192892_1792478450539.log`
- `/home/esjin/.ros/log/python3_192894_1792478450394.log`
- `/home/esjin/.ros/log/python3_194392_1792478463170.log`

증거 JSONL: `reflex_observation_2026-10-20_stage6_pick_place.jsonl`

Panda 제어 로그 사본: `panda_control_stage6_pick_place_2026-10-20.log`
