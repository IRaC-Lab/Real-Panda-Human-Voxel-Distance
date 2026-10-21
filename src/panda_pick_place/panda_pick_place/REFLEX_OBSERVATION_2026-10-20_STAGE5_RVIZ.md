# Panda + RealSense + 좌표 정렬 + 사람 인식·nvblox + RViz 관찰

- 날짜/시간대: 2026-10-20, Asia/Seoul
- 상태: 관찰 완료
- 구성: Panda tf/제어 + RealSense + ArUco 정적 좌표 정렬 + 사람 인식·nvblox + RViz
- 추가 실행 확인: panda_voxel_classifier, closest_panda_human_voxels
- 확인된 global_frame: panda_link0
- RViz OpenGL 초기화 로그: 2026-10-20T15:32:35+09:00
- USB-C 배치(사용자 제공): DC 잭 위쪽에 로봇 USB Ethernet, 40핀 옆에 전원
- 확인한 카메라 설정: RGB/Depth 424×240, 30 FPS
- 확인한 추론 엔진: model_vanilla_v2_0_2_424x240.plan
- 사람 인식 launch 시작: 2026-10-20T15:32:28+09:00
- 추론 노드 시작 로그: 2026-10-20T15:32:31+09:00 (epoch 1792477951.508822923)
- 관찰 기준 시작: 2026-10-20T15:32:36+09:00
- 관찰 예정 종료: 2026-10-20T15:37:36+09:00
- 마지막 로그 확인: 2026-10-20T15:37:36+09:00
- 관찰 구간 경과: 300초
- 결과: 관찰 중 reflex 발생, 최초 경고 2026-10-20T15:36:11+09:00
- RViz 포함 관찰 시작부터 최초 경고까지: 215.565초 (약 3분 36초)
- 사람 인식 launch 시작부터 최초 경고까지: 223.115초 (약 3분 43초)
- reflex 관련 로그 줄 수: 80 (같은 사건이 여러 로그에 기록될 수 있음)
- 관찰 방법: 아래 파일들을 약 5초 간격으로 읽음. 로봇 명령·복구·추가 성능 측정은 실행하지 않음.
- 한계: 로그 기반 관찰이며 실제 제어 통신 성공률이나 전체 프레임 수신을 별도로 측정한 결과는 아님.

## 이벤트

- 최초 경고: 2026-10-20T15:36:11+09:00
- 최초 경고는 reflex가 복구되지 않은 상태를 확인한 시각이며, 실제 abort 순간과는 구분함.
- 원본: `/home/esjin/.ros/log/franka_control2_node_141688_1792477936472.log`

```text
[WARN] [1792478171.564899758] [FrankaHardwareInterface]: Reflex error not recovered - arm won't move. Call ~/error_recovery.
```

- 마지막 관찰 경고: 2026-10-20T15:37:30+09:00
- 같은 복구 미완료 경고가 총 80줄 기록됨. 개별 reflex 발생 횟수가 아님.
- ERROR/FATAL 또는 종료 이벤트 줄 수: 0
- 오류 세부 코드와 control_command_success_rate는 이번에 확인한 저장 로그에 없어 특정하지 않음.
- 이 구성은 직전 실험에 비해 RViz뿐 아니라 panda_voxel_classifier와 closest_panda_human_voxels도 추가됨. 이번 관찰만으로 RViz가 원인이라고 판단할 수 없음.
- 전체 이벤트와 각 타임스탬프는 JSONL에 보존함.

## 원본 로그

- `/home/esjin/.ros/log/franka_control2_node_141688_1792477936472.log`
- `/home/esjin/.ros/log/2026-10-20-15-32-15-728235-jetson-141677/launch.log`
- `/home/esjin/.ros/log/component_container_mt_141854_1792477939128.log`
- `/home/esjin/.ros/log/2026-10-20-15-32-18-921361-jetson-141853/launch.log`
- `/home/esjin/.ros/log/python3_141962_1792477945080.log`
- `/home/esjin/.ros/log/2026-10-20-15-32-23-487905-jetson-141961/launch.log`
- `/home/esjin/.ros/log/component_container_mt_142049_1792477948951.log`
- `/home/esjin/.ros/log/2026-10-20-15-32-28-449480-jetson-142016/launch.log`
- `/home/esjin/.ros/log/python3_142045_1792477949542.log`
- `/home/esjin/.ros/log/python3_142047_1792477950442.log`
- `/home/esjin/.ros/log/rviz2_142451_1792477953499.log`
- `/home/esjin/.ros/log/python3_142051_1792477949851.log`
- `/home/esjin/.ros/log/python3_142053_1792477949871.log`

증거 JSONL: `reflex_observation_2026-10-20_stage5_rviz.jsonl`

Panda 제어 로그 사본: `panda_control_stage5_rviz_2026-10-20.log`
