# 30 FPS 사람 인식·nvblox 최적화 결과

이 문서는 기존 960×544 모델의 최적화 기록이다. 이후 424×240 입출력 엔진을 추가해
최종 overlay까지 약 30 FPS를 측정했다. 현재 실험 설정과 실행 방법은
[NATIVE_SEGMENTATION_424x240.md](NATIVE_SEGMENTATION_424x240.md)에 있다.

2026-10-14, Asia/Seoul. D435i → Jetson AGX Orin, 50W 모드, GPU 최대 816 MHz.
RGB·Depth는 각각 424×240 @ 30 FPS. 기존 TensorRT 엔진과 960×544 모델 입력을 유지했다.
각 측정은 준비 구간 후 20초 동안 동시에 구독했다. 도메인 83에서 외부 DDS 참가자를 분리했다.
원본 도메인 0에서는 기존 구성에서도 직렬화 경고가 발생했고, 분리한 측정에서는 사라졌다.
전역 환경변수나 사용자의 셸 설정을 변경하지 않았다.

## 중간 토픽을 포함한 비교

| 토픽 | 기존 수신 FPS | 개선 수신 FPS |
|---|---:|---:|
| `color/image_raw` | 29.85 | 29.95 |
| `depth/image_rect_raw` | 30.00 | 30.00 |
| `segmentation/image_resized` | 19.70 | 10.65 |
| `segmentation/image_bgr` | 22.55 | 10.30 |
| `segmentation/people_mask` | 12.50 | 12.60 |
| `unet/colored_segmentation_mask` | 10.35 | 10.90 |
| `dynamic_color_frame_overlay` | 9.80 | 13.55 |
| `dynamic_depth_frame_overlay` | 12.65 | 13.65 |

리사이즈/BGR 수신 빈도가 줄어든 것은 새 입력 노드가 처리 가능한 최신 프레임만
선택하기 때문이다. 원본 RGB와 Depth는 계속 약 30 FPS다. 개별 수신 값은 Best Effort
관측값이며, 서로 다른 토픽의 구독 누락 때문에 단계 순서대로 감소하지 않을 수 있다.
중간 토픽을 관측하는 것 자체도 GPU→CPU 변환/직렬화 작업을 추가한다.

## 요청한 5개 토픽만 구독한 비교

| 토픽 | 기존 수신 FPS | 개선 수신 FPS |
|---|---:|---:|
| `color/image_raw` | 29.95 | 29.90 |
| `depth/image_rect_raw` | 30.00 | 29.95 |
| `unet/colored_segmentation_mask` | 11.75 | 9.90 |
| `dynamic_color_frame_overlay` | 11.55 | 13.90 |
| `dynamic_depth_frame_overlay` | 13.35 | 14.05 |

측정은 순차적으로 실행했으므로 장면·부하 변동의 영향을 받는다. 이 값은 출력/수신 빈도이며
특정 연산 한 번의 실행시간이 아니다. TensorRT 엔진 단독 GPU 실행시간은 평균 약 56.5 ms였다.

## 타임스탬프 기준 지연

중간 토픽을 함께 구독한 조건에서, 수신 시각과 원본 `header.stamp` 차이의 중앙값:

| 토픽 | 기존 ms | 개선 ms |
|---|---:|---:|
| `people_mask` | 837.7 | 255.3 |
| `dynamic_color_frame_overlay` | 929.0 | 343.8 |
| `dynamic_depth_frame_overlay` | 891.0 | 338.9 |

이는 카메라 드라이버/시계 변환과 앞선 모든 처리의 시간을 포함한다. 물리적인 촬영 시각을
별도 장비로 검증한 지연값이나 그 단계 하나의 계산시간은 아니다. JSON에는 p95와 표본 수도 있다.

## 적용한 변경

- `segmentation_input_node`: 동일 타임스탬프/프레임의 RGB와 CameraInfo를 짝지어 신뢰성 QoS로
  전달한다. 모델 결과가 오기 전 최대 3장만 유지하고, 슬롯이 생기면 최신 원본을 선택한다.
  과거 프레임이 TensorRT 큐에 쌓이지 않도록 한다. 피드백을 놓치면 1초 후 슬롯을 회복한다.
- `native_mask_node`: 기존 nvblox의 2배 샘플링/4방향 연결 요소 제거를 모델 해상도에서 한 번
  수행한다. 기존 2000픽셀 기준을 유지한다. 그 뒤 면적 기반 축소에서 사람이 조금이라도
  덮은 원본 카메라 픽셀을 사람으로 유지한다. 경계가 카메라 픽셀 단위로 약간 넓어질 수 있다.
- nvblox RGB와 마스크를 원본 카메라 해상도에서 처리하고, 같은 촬영의 원본 CameraInfo를
  사용한다. nvblox의 중복 연결 요소 처리를 비활성화한다. 컬러 오버레이도 424×240이 된다.
- 기존 엔진, BGR/정규화, voxel_size, Depth 해상도 및 로봇 동작 코드는 이 최적화에서 바꾸지 않았다.

## 정확도 및 검증 범위

프레임/보정 정보 일치, 최대 미완료 프레임 수, 잘못된 피드백 무시, 기존 크기 제거 기준,
좁은 전경의 면적 축소 보존, 지원하지 않는 확대 거부에 대한 6개 검증이 통과했다.
두 변경 패키지 빌드와 설치된 실제 런치의 카메라 전용 실행도 검증했다.
실제 Panda의 움직임·충돌 회피 성능은 이번 시험 대상이 아니다.

같은 사람 포함 RGB 8장으로 새 엔진 후보를 비교했다. 직접 FP16은 화면 전체를 사람으로
분류했고, 제공된 보정 파일의 INT8은 일부 프레임에서 사람 영역이 줄어 적용하지 않았다.
BF16 후보는 기존 마스크와 거의 같았으나 실제 연산이 FP32로 남아 성능 이점이 없었다.
저정밀도 엔진을 적용하지 않았으므로 실제 모델의 프레임별 분류는 기존 엔진을 그대로 사용한다.
이 비교는 기존 모델과의 일치도 검사이며 정답 라벨 기반 정확도 평가가 아니다.

## 실행

카메라, 사람 인식, 뷰어 각각의 터미널에서 같은 ROS 도메인을 사용한다.
아래는 깨끗한 카메라 전용 측정에 사용한 도메인 83이다.

터미널 1:

```bash
source_real_ws
export ROS_DOMAIN_ID=83
ros2 launch nvblox_examples_bringup realsense.launch.py \
  run_standalone:=True color_profile:=424x240x30 depth_profile:=424x240x30
```

터미널 2:

```bash
source_real_ws
export ROS_DOMAIN_ID=83
ros2 launch panda_real_bringup panda_realsense_people.launch.py \
  run_realsense:=False run_alignment:=False run_rviz:=False run_panda:=False
```

터미널 3:

```bash
source_real_ws
export ROS_DOMAIN_ID=83
ros2 run rqt_image_view rqt_image_view /nvblox_node/dynamic_color_frame_overlay &
ros2 run rqt_image_view rqt_image_view /nvblox_node/dynamic_depth_frame_overlay
```

모든 로봇 ROS 노드도 같은 도메인을 써야 한다. 원래 동작 비교는 터미널 2에
`optimize_perception:=False`를 추가한다. 저지연 실험은 `max_segmentation_in_flight:=2`로 가능하다.
네이티브 매핑은 모델 마스크보다 큰 카메라 해상도(예: 1280×720)를 지원하지 않는다.
그 경우 `optimize_perception:=False`를 사용한다. 848×480은 크기 조건에 맞지만 이번 속도 시험은
424×240에서만 수행했다.

측정 도구:

```bash
source_real_ws
export ROS_DOMAIN_ID=83
ros2 run panda_pick_place perception_profile --duration 20 --output /tmp/perception_rates.json
```

원시 측정 값은 [perception_measurements_2026-10-14.json](perception_measurements_2026-10-14.json)에 있다.

## 추가 확인: resize에 30 FPS를 공급하면 어떻게 되는가

같은 424×240 @ 30 FPS 카메라, 도메인 83에서 각 20초 동안 관측했다.
resize만 실행하면 원본 RGB는 30.00 FPS, 960×544 resize 출력 수신은 28.70 FPS였다.
전체 파이프라인에서 입력 제한만 해제하고 네이티브 마스크 최적화를 유지한 결과:

| 토픽 | 입력 제한 해제 시 수신 FPS | header 지연 중앙값 ms |
|---|---:|---:|
| `color/image_raw` | 30.00 | 33.4 |
| `depth/image_rect_raw` | 29.95 | 8.7 |
| `segmentation/image_resized` | 18.75 | 129.9 |
| `segmentation/image_bgr` | 20.50 | 162.4 |
| `segmentation/people_mask` | 11.75 | 851.7 |
| `unet/colored_segmentation_mask` | 9.45 | 901.9 |
| `dynamic_color_frame_overlay` | 12.80 | 944.9 |
| `dynamic_depth_frame_overlay` | 12.75 | 927.2 |

入力 제한을 풀어도 최종 출력은 30 FPS에 도달하지 않았고 지연이 늘었다.
resize 자체는 단독으로 거의 30 FPS를 따라갔지만, 전체 구성에는 공유 GPU 부하,
추론 처리 용량, 큐 대기 및 관측용 변환 비용이 함께 작용한다. 이 측정으로 각 원인의
비중을 분리할 수는 없다. 기존 엔진의 단독 연산 약 56.5 ms는 30 FPS의 프레임 간격
33.3 ms보다 길다. 최종 30 FPS에는 추론과 나머지 처리의 실제 작업량을 줄여야 한다.

비교 실행용으로 런치에 `limit_segmentation_input` 옵션을 추가했다. 기본값은 `True`다.
다음 명령은 입력 제한만 해제하고 네이티브 매핑 최적화를 유지한다:

```bash
source_real_ws
export ROS_DOMAIN_ID=83
ros2 launch panda_real_bringup panda_realsense_people.launch.py \
  run_realsense:=False run_alignment:=False run_rviz:=False run_panda:=False \
  limit_segmentation_input:=False
```

카메라 터미널도 같은 도메인을 사용해야 한다. 테스트 노드는 측정 후 종료했다.
추가 원시 값은 [resize_input_experiment_2026-10-14.json](resize_input_experiment_2026-10-14.json)에 있다.
