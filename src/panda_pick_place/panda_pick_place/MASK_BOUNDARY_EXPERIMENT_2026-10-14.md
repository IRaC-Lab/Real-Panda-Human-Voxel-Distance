# 424×240 마스크 후처리 경계 개선

2026-10-14, D435i, Jetson AGX Orin, ROS 기본 도메인 0.
각 설정을 준비 구간 후 20초 동안 관측했다.

## 변경

`native_mask_node`는 원래 마스크 해상도에서 4방향 연결 요소를 필터링한다.
절반으로 샘플링했다 복원하는 과정을 생략해 얇게 붙은 전경과 좁은 틈을 덜 잃는다.
모델이 이미 놓친 손가락을 복원하는 기능은 아니다.

`mask_component_subsampling:=1`이 기본값이며 `:=2`로 이전 후처리를 비교할 수 있다.
424×240 최소 연결 요소 면적은 390픽셀이다. 기존 모델 가중치·엔진은 유지한다.

## 측정

| 토픽 | 이전 후처리 FPS | 원본 해상도 후처리 FPS |
|---|---:|---:|
| RGB | 29.95 | 30.00 |
| Depth | 29.90 | 29.90 |
| resized | 29.90 | 29.55 |
| BGR | 29.85 | 29.90 |
| people_mask | 30.00 | 29.85 |
| colored_segmentation_mask | 29.50 | 29.25 |
| color_overlay | 29.85 | 29.90 |
| depth_overlay | 30.00 | 29.95 |

Best Effort 수신 FPS이며 관측 누락과 변환 비용을 포함한다. 순차 측정으로 장면·부하가
완전히 동일하지는 않다. 새 후처리의 컬러·Depth overlay header 지연 중앙값은
128.8·124.1 ms였다. 이는 해당 연산만의 시간이 아닌 전체 앞선 작업과 드라이버 시간을
포함한다. 8개 helper 검증과 ROS 패키지 빌드가 통과했다.

## 실행

카메라 터미널:

```bash
source_real_ws
unset ROS_DOMAIN_ID
ros2 launch nvblox_examples_bringup realsense.launch.py \
  run_standalone:=True color_profile:=424x240x30 depth_profile:=424x240x30
```

인지 터미널:

```bash
source_real_ws
unset ROS_DOMAIN_ID
ros2 launch panda_real_bringup panda_realsense_people.launch.py \
  run_realsense:=False run_alignment:=False run_rviz:=False run_panda:=False \
  segmentation_resolution:=424x240 mask_component_subsampling:=1
```

원시 결과: [mask_boundary_measurements_2026-10-14.json](mask_boundary_measurements_2026-10-14.json).
