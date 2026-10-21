# Panda 제어·TF + pick and place + Panda 디버깅 RViz 재시험 5분 관찰

- 시간대: Asia/Seoul
- 상태: 관찰 완료
- 구성: Panda 제어·TF + pick and place + Panda 디버깅 RViz (카메라·좌표 정렬·사람 인식·nvblox 제외)
- RViz 설정(사용자 제공): ~/panda_real_ws/src/my_people_nvblox_bringup/config/visualization/panda_sphere_debug.rviz
- RViz OpenGL 초기화 확인: 2026-10-20T17:09:58.847+09:00
- 구성 근거: 사용자 제공 실행 명령 및 로그. 이전 카메라·인식 구성에는 종료 기록이 있고 이번 구성에 신규 인식 시작 로그는 확인되지 않았음. 프로세스 전수 검사는 수행하지 않음.
- 첫 home 동작 요청: 2026-10-20T17:09:56.582+09:00
- 관찰 기준 시작: 2026-10-20T17:09:59.000+09:00
- 관찰 예정 종료: 2026-10-20T17:14:59.000+09:00
- 마지막 확인: 2026-10-20T17:14:59.012+09:00
- 관찰 구간 경과: 300초
- 결과: reflex 기록 없음
- 최대 Cycle 시작 번호: 13 (완료 횟수 또는 실제 물체 이송 성공 횟수를 의미하지 않음)
- reflex 관련 로그 줄 수: 0 (반복 경고 포함; 개별 발생 횟수가 아님)
- 기타 오류/종료 이벤트 줄 수: 0
- 방법: 약 5초 간격으로 로그만 읽음. 관찰 도구는 로봇 명령이나 복구를 실행하지 않음.
- pick and place 자체는 매 이동 전 오류 복구 서비스를 호출하는 구현이므로, 동작이 이어져도 reflex 기록을 별도로 확인함.
- 이전 단독 동작 실험과 별도 실행이며, 이번 Panda launch는 17:09:46에 시작함. 첫 디버깅 RViz 실험과 별도 실행임.
- 한계: 로그 기반 결과이며 실제 관절 움직임·물체 이송 성공·제어 통신 성공률을 별도로 측정한 결과는 아님.

## 마지막 동작 로그

```text
[INFO] [1792484091.201594376] [pick_place_node]: -> pick_above (5.9 s)
```

## 이벤트

현재까지 reflex·오류·종료 이벤트 없음.

## 증거

- `/home/esjin/.ros/log/franka_control2_node_231505_1792483786937.log`
- `/home/esjin/.ros/log/python3_231676_1792483790765.log`
- `/home/esjin/.ros/log/2026-10-20-17-09-46-188945-jetson-231494/launch.log`
- `/home/esjin/.ros/log/franka_gripper_node_231513_1792483786923.log`
- `/home/esjin/.ros/log/rviz2_231745_1792483798197.log`

- 전체 이벤트/상태: `reflex_observation_2026-10-20_stage9_debug_rviz_retest.jsonl`
- 원본 사본: `panda_stage9_debug_rviz_retest_control_2026-10-20.log`
- 원본 사본: `panda_stage9_debug_rviz_retest_pick_place_2026-10-20.log`
- 원본 사본: `panda_stage9_debug_rviz_retest_launch_2026-10-20.log`
- 원본 사본: `panda_stage9_debug_rviz_retest_gripper_2026-10-20.log`
- 원본 사본: `panda_stage9_debug_rviz_retest_rviz_2026-10-20.log`

## 최종 비교

- 동일 구성 재시험: 17:09:59~17:14:59, 5분간 reflex·동작 오류·종료 이벤트 기록 없음.
- 종료 시 Cycle 13 진입 확인. Cycle 13 완료나 실제 물체 이송 성공 횟수를 의미하지 않음.
- 첫 시행은 RViz 포함 관찰 시작 후 약 2분 5초, Cycle 6에서 reflex 경고가 기록됐음.
- 동일 구성의 이번 시행에서는 5분간 재현되지 않았으므로, RViz 실행이 매번 즉시 reflex를 일으킨다고 단정할 수 없음. 관찰 시간도 장시간 안정성 확인에는 제한적임.
