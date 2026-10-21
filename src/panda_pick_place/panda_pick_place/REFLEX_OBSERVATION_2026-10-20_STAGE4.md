# Panda + RealSense + 좌표 정렬 + 사람 인식·nvblox 관찰

- 날짜/시간대: 2026-10-20, Asia/Seoul
- 상태: 최초 5분 관찰 완료; 이후 연장 실행 중 reflex 재발 확인
- 구성: Panda tf/제어 + RealSense + ArUco 정적 좌표 정렬 + 사람 인식·nvblox
- USB-C 배치(사용자 제공): DC 잭 위쪽에 로봇 USB Ethernet, 40핀 옆에 전원
- 확인한 카메라 설정: RGB/Depth 424×240, 30 FPS
- 확인한 추론 엔진: model_vanilla_v2_0_2_424x240.plan
- 사람 인식 launch 시작: 2026-10-20T15:13:11+09:00
- 추론 노드 시작 로그: 2026-10-20T15:13:16+09:00 (epoch 1792476796.384654278)
- 관찰 기준 시작: 2026-10-20T15:13:17+09:00
- 관찰 예정 종료: 2026-10-20T15:18:17+09:00
- 마지막 로그 확인: 2026-10-20T15:18:17+09:00
- 관찰 구간 경과: 300초
- 최초 5분 결과: 관찰 로그에서 reflex 기록 없음
- 최초 5분 reflex 관련 로그 줄 수: 0 (같은 사건이 여러 로그에 기록될 수 있음)
- 관찰 방법: 아래 파일들을 약 5초 간격으로 읽음. 로봇 명령·복구·추가 성능 측정은 실행하지 않음.
- 한계: 로그 기반 관찰이며 실제 제어 통신 성공률이나 전체 프레임 수신을 별도로 측정한 결과는 아님.

## 이벤트

현재까지 reflex, ERROR/FATAL 또는 종료 이벤트가 기록되지 않음.

## 원본 로그

- `/home/esjin/.ros/log/franka_control2_node_29737_1792476780227.log`
- `/home/esjin/.ros/log/2026-10-20-15-12-59-498651-jetson-29726/launch.log`
- `/home/esjin/.ros/log/component_container_mt_29906_1792476782633.log`
- `/home/esjin/.ros/log/2026-10-20-15-13-02-421137-jetson-29902/launch.log`
- `/home/esjin/.ros/log/python3_29987_1792476787226.log`
- `/home/esjin/.ros/log/2026-10-20-15-13-05-692379-jetson-29986/launch.log`
- `/home/esjin/.ros/log/component_container_mt_30080_1792476791855.log`
- `/home/esjin/.ros/log/2026-10-20-15-13-11-363002-jetson-30047/launch.log`
- `/home/esjin/.ros/log/python3_30076_1792476792383.log`
- `/home/esjin/.ros/log/python3_30078_1792476793219.log`

증거 JSONL: `reflex_observation_2026-10-20_stage4.jsonl`

Panda 제어 로그 사본: `panda_control_stage4_2026-10-20.log`

## 추가 확인 — 15:28:08 KST

- 동일한 10개 원본 로그를 다시 읽어 누적 기록을 확인함. 최초 5분 관찰 이후에는 이번 시점의 소급 확인임.
- 추론 시작(15:13:16) 후 약 14분 53초 경과.
- reflex, communication_constraints_violation, ERROR/FATAL, 노드 종료 기록 없음.
- nvBlox 통계 로그가 15:28:08까지 계속 갱신됨(확인 시 파일 크기 1,143,032바이트).

## 최종 추가 확인 — reflex 재발

- 최초 타임스탬프 경고: 2026-10-20T15:28:28.057+09:00
- 경고 원문: `[WARN] [1792477708.057872043] [FrankaHardwareInterface]: Reflex error not recovered - arm won't move. Call ~/error_recovery.`
- 이 시각은 최초 복구 미완료 경고 시각이며, 실제 명령 중단 순간은 저장된 ROS 로그에서 정확히 특정되지 않음.
- 추론 시작(15:13:16.385)부터 최초 경고까지: 911.673초, 약 15분 12초.
- 사람 인식 launch 시작부터 최초 경고까지: 916.695초, 약 15분 17초.
- 사용자 제공 화면에서 확인한 중단 오류: `communication_constraints_violation`.
- 사용자 제공 화면에서 확인한 `control_command_success_rate`: `0.29`. 중단 메시지와 이 수치는 저장된 제어 로그에는 없으므로 화면 증거로 구분함.
- 직전 소급 확인(15:28:08) 후 약 20.1초 뒤 최초 reflex 경고가 기록됨.
- Panda launch 로그에는 15:28:51 사용자 Ctrl-C 및 정상 종료가 기록됨. reflex 이후 종료이며, 최초 오류와 구분함.
- 결론: USB-C 포트 교환 후에도 전체 구성에서 통신 관련 reflex가 재발함. 이번 단일 시행의 발생 시간 차이로 포트 교환에 의한 개선을 확정할 수 없음.
- 발생 후 제어 로그 사본: `panda_control_stage4_2026-10-20_after_reflex.log`
- 발생 후 launch 로그 사본: `panda_launch_stage4_2026-10-20_after_reflex.log`
