# 전체 시스템 + pick and place 10분 관찰

- 시간대: Asia/Seoul
- 상태: 관찰 완료
- 구성: Panda 제어·TF + RealSense + ArUco 정적 좌표 정렬 + 사람 인식·nvblox + RViz + pick and place
- 추가 실행 확인: panda_voxel_classifier, closest_panda_human_voxels
- 카메라 시작 설정: RGB/Depth 424×240, 30 FPS
- nvBlox global_frame: panda_link0
- 좌표 정렬 보정 완료 및 정적 TF 발행: 17:30:11
- 사람 인식 추론 노드 시작: 17:30:20.350
- RViz OpenGL 초기화: 17:30:23.138
- 구성 근거: 각 구성의 현재 실행 로그. 실제 화면 표시 및 카메라 수신 FPS는 별도로 측정하지 않음.
- 첫 home 동작 요청: 2026-10-20T17:30:31.658+09:00
- 관찰 기준 시작: 2026-10-20T17:30:32.000+09:00
- 관찰 예정 종료: 2026-10-20T17:40:32.000+09:00
- 마지막 확인: 2026-10-20T17:40:32.197+09:00
- 관찰 구간 경과: 600초
- 결과: reflex 경고 후 복구된 구간 4개 확인; 이후 동작 진행
- 최대 Cycle 시작 번호: 25 (완료 횟수 또는 실제 물체 이송 성공 횟수를 의미하지 않음)
- reflex 관련 로그 줄 수: 7 (반복 경고 포함; 개별 발생 횟수가 아님)
- 기타 오류/종료 이벤트 줄 수: 0
- 방법: 약 5초 간격으로 로그만 읽음. 관찰 도구는 로봇 명령이나 복구를 실행하지 않음.
- pick and place 자체는 매 이동 전 오류 복구 서비스를 호출하는 구현이므로, 동작이 이어져도 reflex 기록을 별도로 확인함.
- 이전 실험과 별도 실행이며 이번 Panda launch는 17:30:00에 시작함.
- 한계: 로그 기반 결과이며 실제 관절 움직임·물체 이송 성공·제어 통신 성공률을 별도로 측정한 결과는 아님.

## 마지막 동작 로그

```text
[INFO] [1792485623.842531831] [pick_place_node]: -> pick_above (5.9 s)
```

## 이벤트

- 최초 reflex 경고: 2026-10-20T17:35:24.496+09:00
- 관찰 시작 이후: 292.496초

```text
[WARN] [1792485324.496049569] [FrankaHardwareInterface]: Reflex error not recovered - arm won't move. Call ~/error_recovery.
```


## 증거

- `/home/esjin/.ros/log/franka_control2_node_236949_1792485000938.log`
- `/home/esjin/.ros/log/python3_238408_1792485027320.log`
- `/home/esjin/.ros/log/2026-10-20-17-30-00-208792-jetson-236938/launch.log`
- `/home/esjin/.ros/log/franka_gripper_node_236957_1792485000926.log`
- `/home/esjin/.ros/log/rviz2_237768_1792485022291.log`
- `/home/esjin/.ros/log/component_container_mt_237124_1792485004553.log`
- `/home/esjin/.ros/log/2026-10-20-17-30-04-329544-jetson-237120/launch.log`
- `/home/esjin/.ros/log/python3_237210_1792485009892.log`
- `/home/esjin/.ros/log/2026-10-20-17-30-08-270920-jetson-237205/launch.log`
- `/home/esjin/.ros/log/component_container_mt_237308_1792485017715.log`
- `/home/esjin/.ros/log/2026-10-20-17-30-17-198504-jetson-237275/launch.log`
- `/home/esjin/.ros/log/python3_237310_1792485018645.log`
- `/home/esjin/.ros/log/python3_237312_1792485018646.log`
- `/home/esjin/.ros/log/python3_237304_1792485018300.log`
- `/home/esjin/.ros/log/python3_237306_1792485019054.log`

- 전체 이벤트/상태: `reflex_observation_2026-10-20_stage11_whole_system.jsonl`
- 원본 사본: `panda_stage11_whole_system_control_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_pick_place_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_launch_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_gripper_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_rviz_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_camera_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_camera_launch_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_alignment_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_alignment_launch_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_segmentation_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_segmentation_launch_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_voxel_classifier_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_closest_voxels_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_segmentation_input_2026-10-20.log`
- 원본 사본: `panda_stage11_whole_system_native_mask_2026-10-20.log`

## 최종 요약 및 복구 구간

- 전체 시스템 17:30:32~17:40:32의 10분 로그 관찰 완료.
- 최초 경고: 17:35:24.496, 관찰 시작 후 292.496초(약 4분 52초), 첫 home 동작 후 약 4분 53초.
- 최초 구간: Cycle 12의 gripper open 요청 이후, 다음 place_above 요청 직전.
- 최초 복구: 17:35:25.468. 이후 제어 루프 재시작 및 Cycle 13 이상 진행이 기록됨.
- 총 4개의 경고→복구 구간과 경고 7줄이 기록됨. 반복 경고 줄 수와 구간 수를 구분함.
- 관찰 종료 시 Cycle 25 진입 확인. 무오류 10분 실행이나 실제 물체 이송 성공을 의미하지 않음.
- pick and place가 매 이동 전 호출하는 오류 복구 서비스에 의해 제어가 이어진 것으로 코드와 로그에서 확인함. 관찰 도구가 복구를 실행한 것은 아님.
- 저장된 이번 로그에는 최초 abort의 오류 세부 코드와 control_command_success_rate가 없어 특정하지 않음.

| 최초 경고(KST) | 관찰 시작 후(초) | Cycle | 경고 줄 수 | 복구(KST) |
| --- | ---: | ---: | ---: | --- |
| 2026-10-20T17:35:24.496+09:00 | 292.496 | 12 | 1 | 2026-10-20T17:35:25.468+09:00 |
| 2026-10-20T17:37:26.308+09:00 | 414.308 | 17 | 3 | 2026-10-20T17:37:28.877+09:00 |
| 2026-10-20T17:37:39.583+09:00 | 427.584 | 18 | 2 | 2026-10-20T17:37:41.000+09:00 |
| 2026-10-20T17:37:42.897+09:00 | 430.897 | 18 | 1 | 2026-10-20T17:37:43.061+09:00 |

각 구간 직전 pick and place 로그(해당 단계의 실행 중인지 이후 대기 중인지는 별도 구분 필요):

- 2026-10-20T17:35:24.496+09:00: `[INFO] [1792485322.896220327] [pick_place_node]: gripper: open`
- 2026-10-20T17:37:26.308+09:00: `[INFO] [1792485444.449070863] [pick_place_node]: -> place (2.0 s)`
- 2026-10-20T17:37:39.583+09:00: `[INFO] [1792485458.912603597] [pick_place_node]: gripper: grasp`
- 2026-10-20T17:37:42.897+09:00: `[INFO] [1792485461.004035630] [pick_place_node]: -> pick_above (2.0 s)`
