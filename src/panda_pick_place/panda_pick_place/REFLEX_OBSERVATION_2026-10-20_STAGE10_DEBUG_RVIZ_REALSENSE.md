# Panda 제어·TF + pick and place + Panda 디버깅 RViz + RealSense 5분 관찰

- 시간대: Asia/Seoul
- 상태: 관찰 완료
- 구성: Panda 제어·TF + pick and place + Panda 디버깅 RViz + RealSense (좌표 정렬·사람 인식·nvblox 제외)
- RealSense 설정: RGB/Depth 424×240, 30 FPS
- RealSense 시작 확인: 2026-10-20T17:21:10.697+09:00
- RViz 설정(사용자 제공): ~/panda_real_ws/src/my_people_nvblox_bringup/config/visualization/panda_sphere_debug.rviz
- RViz OpenGL 초기화 확인: 2026-10-20T17:21:24.649+09:00
- 구성 근거: 사용자 제공 실행 명령 및 로그. 이전 카메라·인식 구성에는 종료 기록이 있고 이번 구성에 신규 인식 시작 로그는 확인되지 않았음. 프로세스 전수 검사는 수행하지 않음.
- 첫 home 동작 요청: 2026-10-20T17:21:22.991+09:00
- 관찰 기준 시작: 2026-10-20T17:21:25.000+09:00
- 관찰 예정 종료: 2026-10-20T17:26:25.000+09:00
- 마지막 확인: 2026-10-20T17:26:25.011+09:00
- 관찰 구간 경과: 300초
- 결과: reflex 기록 없음
- 최대 Cycle 시작 번호: 13 (완료 횟수 또는 실제 물체 이송 성공 횟수를 의미하지 않음)
- reflex 관련 로그 줄 수: 0 (반복 경고 포함; 개별 발생 횟수가 아님)
- 기타 오류/종료 이벤트 줄 수: 0
- 방법: 약 5초 간격으로 로그만 읽음. 관찰 도구는 로봇 명령이나 복구를 실행하지 않음.
- pick and place 자체는 매 이동 전 오류 복구 서비스를 호출하는 구현이므로, 동작이 이어져도 reflex 기록을 별도로 확인함.
- 이전 단독 동작 실험과 별도 실행이며, 이번 Panda launch는 17:21:03에 시작함. 이전 디버깅 RViz 실험과 별도 실행임.
- 한계: 로그 기반 결과이며 실제 관절 움직임·물체 이송 성공·제어 통신 성공률을 별도로 측정한 결과는 아님.

## 마지막 동작 로그

```text
[INFO] [1792484780.446709040] [pick_place_node]: -> pick_above (5.9 s)
```

## 이벤트

현재까지 reflex·오류·종료 이벤트 없음.

## 증거

- `/home/esjin/.ros/log/franka_control2_node_234467_1792484464355.log`
- `/home/esjin/.ros/log/python3_234747_1792484478422.log`
- `/home/esjin/.ros/log/2026-10-20-17-21-03-614598-jetson-234456/launch.log`
- `/home/esjin/.ros/log/franka_gripper_node_234475_1792484464339.log`
- `/home/esjin/.ros/log/rviz2_234789_1792484484191.log`
- `/home/esjin/.ros/log/component_container_mt_234645_1792484469486.log`
- `/home/esjin/.ros/log/2026-10-20-17-21-09-265122-jetson-234640/launch.log`

- 전체 이벤트/상태: `reflex_observation_2026-10-20_stage10_debug_rviz_realsense.jsonl`
- 원본 사본: `panda_stage10_debug_rviz_realsense_control_2026-10-20.log`
- 원본 사본: `panda_stage10_debug_rviz_realsense_pick_place_2026-10-20.log`
- 원본 사본: `panda_stage10_debug_rviz_realsense_launch_2026-10-20.log`
- 원본 사본: `panda_stage10_debug_rviz_realsense_gripper_2026-10-20.log`
- 원본 사본: `panda_stage10_debug_rviz_realsense_rviz_2026-10-20.log`
- 원본 사본: `panda_stage10_debug_rviz_realsense_camera_2026-10-20.log`
- 원본 사본: `panda_stage10_debug_rviz_realsense_camera_launch_2026-10-20.log`

## 최종 요약

- Panda 제어·TF + pick and place + Panda 디버깅 RViz + RealSense 구성의 17:21:25~17:26:25 5분 관찰 완료.
- 로그상 reflex, trajectory 실패, 그리퍼·카메라 오류, 종료 이벤트 없음.
- 관찰 종료 시 Cycle 13 진입 확인. Cycle 13 완료 또는 실제 물체 이송 성공 횟수를 의미하지 않음.
- RealSense는 RGB/Depth 424×240, 30 FPS로 시작했으나 실제 수신 FPS를 별도로 측정하지는 않음.
- 이 결과는 해당 5분 구간에 한정되며 장시간 안정성이나 특정 장치의 원인 여부를 확정하지 않음.
