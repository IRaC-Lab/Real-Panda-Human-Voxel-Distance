# 424×240 모델 입출력 실험

후속 경계 개선에서는 `mask_component_subsampling:=1`을 기본값으로 추가했다.
이 문서의 최초 측정은 이전 2배 샘플링 후처리 기준이다.
`mask_component_subsampling:=2`로 이전 방식을 비교할 수 있다.
후속 결과는 [MASK_BOUNDARY_EXPERIMENT_2026-10-14.md](MASK_BOUNDARY_EXPERIMENT_2026-10-14.md)에 있다.

2026-10-14. RealSense D435i RGB/Depth 424×240 @ 30 FPS, Jetson AGX Orin,
ROS_DOMAIN_ID=83, 기존 전처리(BGR, [-1, 1]) 및 모델 가중치 유지.
각 ROS 측정은 준비 구간 후 20초 동안 동시에 구독한 수신 FPS다.

## 엔진 변경

원본 ONNX는 batch만 동적이고 공간 입력은 960×544로 고정돼 있었다.
TensorRT의 shape 옵션만 424×240로 지정하면 고정 입력 불일치 오류가 발생한다.
네트워크 입력을 직접 변경해도 U-Net의 다섯 차례 축소/복원 과정에서 concat 크기가
맞지 않아 실행할 수 없었다.

별도 TensorRT 네트워크에 다음 처리를 추가했다:

```text
입력 1×3×240×424
 → 오른쪽 24, 아래쪽 16픽셀을 0으로 패딩
 → 기존 모델 가중치, 연산 크기 448×256
 → 오른쪽/아래 패딩 영역 제거
 → argmax_1 출력 1×240×424×1
```

패딩은 정규화 이후 적용하므로 값 0은 중간 밝기에 해당한다.
원본 카메라 영상은 확대하지 않는다. 실제 연산 픽셀 수는 기존의 약 22%다.
FP16/INT8 옵션은 추가하지 않았다. TensorRT 기본 정밀도 정책은 유지했다.
새 엔진은 `models/peoplesemsegnet/vanilla/1/model_vanilla_v2_0_2_424x240.plan`에
별도로 저장했고 기존 엔진은 유지했다. 엔진은 GPU/TensorRT 환경에 따라 재생성이 필요하다.
재생성 도구는 [scripts/build_native_segmentation_engine.py](scripts/build_native_segmentation_engine.py)다.

런치의 `segmentation_resolution:=424x240`은 전처리·TensorRT 엔진·디코더 크기를 함께
선택한다. 같은 촬영의 CameraInfo와 원본 좌표를 유지한다. 기존 최소 연결 요소 면적
2000픽셀은 해상도 비율에 따라 390픽셀로 조정했다. 기존 2배 샘플링 필터의 반올림도
유지하므로 작은 영역 경계는 기존 모델과 정확히 같지는 않다.

## 중간 토픽을 함께 관측한 결과

| 토픽 | 기존 960×544 프로필 FPS | 424×240 프로필 FPS | 새 프로필 header 지연 중앙값 ms |
|---|---:|---:|---:|
| `color/image_raw` | 29.95 | 29.95 | 33.7 |
| `depth/image_rect_raw` | 30.00 | 29.90 | 8.9 |
| `segmentation/image_resized` | 10.65 | 29.85 | 38.7 |
| `segmentation/image_bgr` | 10.30 | 29.90 | 43.4 |
| `segmentation/people_mask` | 12.60 | 29.80 | 69.2 |
| `unet/colored_segmentation_mask` | 10.90 | 29.65 | 73.3 |
| `dynamic_color_frame_overlay` | 13.55 | 29.95 | 111.2 |
| `dynamic_depth_frame_overlay` | 13.65 | 29.80 | 108.8 |

기존 열은 앞서 측정한 최적화된 960×544 프로필이다. 동일 모델의 기존 엔진 단독 GPU
시간은 약 56.5 ms, 새 엔진은 이번 측정 평균 14.3 ms였다. 새 단독 실행시간은 변동성이
있었으며 다른 ROS 작업과 함께 실행한 시간이 아니다.

## 요청한 5개 토픽만 관측한 결과

| 토픽 | 새 프로필 FPS | header 지연 중앙값 ms |
|---|---:|---:|
| `color/image_raw` | 29.95 | 33.6 |
| `depth/image_rect_raw` | 30.00 | 9.1 |
| `unet/colored_segmentation_mask` | 29.95 | 62.9 |
| `dynamic_color_frame_overlay` | 30.00 | 98.1 |
| `dynamic_depth_frame_overlay` | 30.00 | 95.3 |

Best Effort 수신 값이며 구독 누락과 관측용 변환 부하를 포함한다. header 지연은 카메라
시계/드라이버와 앞선 작업을 포함하므로 개별 연산 시간이나 외부 장비로 검증한 촬영 지연은
아니다. 측정은 순차적으로 진행했고 장면은 고정하지 않았다.

## 마스크 비교 범위

같은 실제 RGB 8장을 기존·새 엔진에 넣었다. 기존 마스크를 면적 기반으로 424×240에
투영한 결과와 비교했을 때 사람 영역 IoU는 약 22.7–63.8%, 기존 전경의 포함 비율은
약 95.2–98.2%였다. 새 마스크의 사람 영역이 더 넓었다. 확인한 첫 프레임에서는 기존
마스크가 머리 위주였고 새 마스크는 몸통도 포함했다.

기존 결과는 정답 라벨이 아니므로 이 차이를 정확도 개선이나 저하로 확정할 수 없다.
이번 검사는 소수의 같은 장면 샘플 비교이며 실제 로봇 움직임과 충돌 회피 검증은 수행하지
않았다. 분류 결과가 달라졌고 더 큰 카메라 입력도 기존에 사용하므로 기본 해상도는
960×544로 유지했다. 새 프로필은 아래 옵션으로 명시적으로 선택한다.
임시 카메라 이미지와 비교용 마스크 파일은 측정 후 삭제했다.

## 실행

일반 실행은 각 터미널에서 기본 도메인 0을 사용한다. 아래 명령은 카메라 전용 구성이다.
위 성능 실험은 도메인 83에서 측정했으며, 이후 기본 도메인에서도 약 30 FPS를 확인했다.
기존 카메라가 실행 중이면 먼저 그 터미널에서 Ctrl+C로 종료한다. 카메라 런치는 동일
D435i의 중복 실행을 USB 장치에 접근하기 전에 차단한다.

터미널 1:

```bash
source_real_ws
unset ROS_DOMAIN_ID
ros2 launch nvblox_examples_bringup realsense.launch.py \
  run_standalone:=True color_profile:=424x240x30 depth_profile:=424x240x30
```

터미널 2:

```bash
source_real_ws
unset ROS_DOMAIN_ID
ros2 launch panda_real_bringup panda_realsense_people.launch.py \
  run_realsense:=False run_alignment:=False run_rviz:=False run_panda:=False \
  segmentation_resolution:=424x240
```

터미널 3:

```bash
source_real_ws
unset ROS_DOMAIN_ID
ros2 run panda_pick_place perception_profile --duration 20 --output /tmp/perception424.json
```

960×544로 되돌릴 때는 `segmentation_resolution:=960x544`를 사용한다.
네이티브 매핑에서는 카메라 크기가 선택한 모델 입출력 크기 이하여야 한다.
카메라가 848×480이면 424×240 프로필에 직접 연결하지 않는다.
입력 제한 노드는 유지했지만 새 엔진이 원본 속도를 따라가므로 프레임 전달도 약 30 FPS다.
`image_resized` 토픽 이름은 유지되며 이 프로필의 실제 메시지 크기는 424×240이다.

7개 helper 검증, 런치 패키지 빌드, 설치된 런치의 실제 카메라/인지 측정을 완료했다.
테스트 노드는 종료했다. 원시 결과는
[native_segmentation_measurements_2026-10-14.json](native_segmentation_measurements_2026-10-14.json)에 있다.
