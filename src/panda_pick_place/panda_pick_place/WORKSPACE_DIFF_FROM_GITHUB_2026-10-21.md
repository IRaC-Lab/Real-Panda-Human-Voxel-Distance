# GitHub 원본과 현재 panda_real_ws의 차이

확인일: 2026-10-21, Asia/Seoul.

대상: `/home/esjin/panda_real_ws`와
[IRaC-Lab/Real-Panda-Human-Voxel-Distance](https://github.com/IRaC-Lab/Real-Panda-Human-Voxel-Distance).

이 문서는 현재 소스·설정·로컬 엔진·기존 실험 기록을 비교한 스냅샷이다.
Jetson 전용 시스템 설정은 제외하고, x86 PC로 옮길 때도 필요한 변경을 기록한다.
이번 확인에서 로봇·카메라·추론을 실행하지 않았으며, 과거 실험 결과와 현재 파일 상태를 구분한다.

## GitHub 반영용 추가 정리

아래 비교는 이번 업데이트를 묶기 전의 작업 트리 스냅샷이다.
GitHub 반영을 준비하면서 당시 별도 전달이 필요하다고 적었던 의존 저장소 변경을
`patches/nvblox_runtime_sync_timestamps.patch`와
`patches/franka_runtime_recovery.patch`로 추가했다.
현재 README의 설치 절차는 이 패치들을 적용한다. 따라서 아래의
"패치에 없음", "미커밋" 등의 표현은 비교 당시 상태를 뜻한다.
설치 오류의 공통 수정은 README와 [설치 메모](../../../docs/INSTALLATION_NOTES.md)에,
추가한 내용과 검증 한계는 [CHANGELOG](../../../CHANGELOG.md)에 정리했다.
원시 터미널 로그와 ping 기록은 로컬 자료로 보존하고 게시 파일에서 제외했다.

## 1. 비교 기준과 범위

- 원격 `HEAD` 및 `refs/heads/main`: `65960499ecd0c93b588e51c55e729e350a75afae`.
- 로컬 `HEAD` 및 `origin/main`: 같은 커밋.
- 커밋: [6596049 — Update README.md](https://github.com/IRaC-Lab/Real-Panda-Human-Voxel-Distance/commit/65960499ecd0c93b588e51c55e729e350a75afae), 2026-10-02.
- 원격 값은 `git ls-remote`로 직접 확인했다. 캐시된 `origin/main`만으로 최신 상태를 추정하지 않았다.
- 이 문서 작성 전, 주 저장소에 수정된 추적 파일 **11개**, 미추적 파일 **73개**가 있었다.
- `src/isaac_ros_nvblox/`, `src/franka_arm_ros2/`, 생성된 `*.plan`은 주 저장소의 `.gitignore` 대상이다. 주 저장소의 `git diff`만으로는 전체 차이를 볼 수 없다.
- `build/`, `install/`, `log/`, Python 캐시 등의 빌드 산출물은 변경 목록에서 제외한다.
- 설치 당시 겪은 오류 기록은 별도 문서인
  [공개 설치 메모](../../../docs/INSTALLATION_NOTES.md)에 있다.
  그 기록의 과거 상태 설명보다 아래의 실제 파일 비교 결과를 우선한다.

## 2. 기억한 모델 크기 변경: 960×544 외에 424×240 프로필 추가

**모델 가중치는 그대로이고, 입력·출력 및 내부 연산 해상도를 줄인 별도 TensorRT 엔진을 추가했다.**
기존 엔진을 대체하지 않았으며, 런치의 기본 해상도도 여전히 `960x544`다.

| 구분 | GitHub 원본 구성 | 현재 로컬 구성 |
|---|---|---|
| 모델 입력 해상도 | 기존 960×544 구성 | 960×544 유지 + 424×240 선택지 추가 |
| 해상도 선택 옵션 | `segmentation_resolution` 없음 | `segmentation_resolution:=960x544` 또는 `:=424x240` |
| 기본 해상도 | 기존 960×544 구성 | `960x544` |
| 기존 엔진 | `.plan`은 Git에 미포함, README에서 생성 | `model_vanilla_v2_0_2.plan` 유지 |
| 추가 엔진 | 생성 스크립트·선택 기능 없음 | `model_vanilla_v2_0_2_424x240.plan` 추가 |
| 모델 가중치 파일 | 원본 ONNX | 원본과 SHA-256 동일 |
| 작은 엔진 내부 처리 | 없음 | 424×240 입력 → 448×256 패딩 연산 → 424×240 출력으로 crop |
| 연결 요소 면적 기준 | 기본 2000픽셀 | 960×544에서 2000, 424×240에서 390픽셀 |

새 엔진의 입출력은 다음과 같다. 아래 shape는 너비×높이 표기와 달리 텐서 차원 순서다.

```text
입력: 1×3×240×424
  → 오른쪽 24픽셀 / 아래쪽 16픽셀을 0으로 패딩
  → 기존 U-Net 가중치로 448×256에서 연산
  → 패딩 영역 제거
출력: argmax_1, 1×240×424×1
```

- 기존 모델의 공간 입력은 고정돼 있어 `trtexec` shape 옵션만 변경해서는 만들 수 없었다는 실험 기록이 있다.
- 새 스크립트는 TensorRT 네트워크에 padding/crop을 추가한다. 기존 ONNX 파일을 덮어쓰거나 모델을 재학습하지 않는다.
- 실제 내부 연산 면적은 기존의 약 **22%**다: `(448×256)/(960×544) ≈ 0.220`.
- 새 생성 스크립트는 FP16/INT8 플래그를 추가하지 않고 TensorRT 기본 정밀도 정책을 사용한다.
- 파일 용량을 크게 줄인 변경은 아니다. 기존 엔진은 **125,391,284 bytes**, 새 엔진은 **125,095,884 bytes**로 둘 다 약 120 MiB다.
- ONNX의 원격 LFS 포인터와 로컬 파일에서 확인한 SHA-256:
  `317f22883dcde8dd6bf2e41d6d182b08fbcb77720d9ca95cee793f461efa920b`.
- 2026-10-20 전체 시스템 로그에도 `_424x240.plan`을 읽은 기록이 있다.
  이는 최근 실행에서 새 엔진을 사용했다는 근거이며, 지금 실행 중인 프로세스를 확인한 것은 아니다.

관련 파일:

- [scripts/build_native_segmentation_engine.py](scripts/build_native_segmentation_engine.py)
- `src/panda_real_bringup/launch/panda_realsense_people.launch.py`
- `models/peoplesemsegnet/vanilla/1/model_vanilla_v2_0_2.plan`
- `models/peoplesemsegnet/vanilla/1/model_vanilla_v2_0_2_424x240.plan`
- [NATIVE_SEGMENTATION_424x240.md](NATIVE_SEGMENTATION_424x240.md)
- `panda_stage11_whole_system_segmentation_2026-10-20.log`의 `Read tensor shape information from TRT Model Engine` 줄.

카메라 전용으로 작은 프로필을 고를 때의 기존 실행 예시는 다음과 같다.
카메라 런치와 인지 런치는 같은 ROS 도메인을 사용하며, 기존 카메라와 중복 실행하지 않는다.

```bash
# 카메라 터미널
source_real_ws
unset ROS_DOMAIN_ID
ros2 launch nvblox_examples_bringup realsense.launch.py \
  run_standalone:=True color_profile:=424x240x30 depth_profile:=424x240x30

# 별도의 인지 터미널
source_real_ws
unset ROS_DOMAIN_ID
ros2 launch panda_real_bringup panda_realsense_people.launch.py \
  run_realsense:=False run_alignment:=False run_rviz:=False run_panda:=False \
  segmentation_resolution:=424x240
```

모델 크기를 바꾸면 마스크 예측도 달라진다. 기존 기록의 같은 RGB 8장 비교에서는
IoU가 약 22.7–63.8%였고 작은 프로필이 더 넓은 사람 영역을 내는 경향이 있었다.
기존 마스크가 정답 라벨은 아니므로 정확도 개선·저하로 단정할 수 없다.
또한 네이티브 매핑에서는 카메라 해상도가 모델 마스크 해상도 이하여야 하므로,
848×480 카메라를 424×240 프로필에 직접 연결하는 구성은 지원하지 않는다.

## 3. 인지 파이프라인의 추가 변경

| 항목 | GitHub 원본 대비 현재 변경 | 근거 파일 |
|---|---|---|
| 입력 프레임 제한 | 결과가 나오기 전 최대 3개 프레임만 진행시키고, 슬롯이 생기면 최신 RGB를 선택한다. RGB/CameraInfo의 timestamp·frame_id를 맞춘다. | `segmentation_input_node.py` |
| 입력 제한 복구 | 결과 피드백이 1초 동안 없으면 슬롯을 회복한다. 전달·결과 피드백에는 reliable QoS를 사용한다. | `segmentation_input_node.py` |
| 카메라 크기 마스크 | 모델 마스크를 카메라 해상도로 축소하고 같은 촬영의 CameraInfo와 짝짓는다. 축소 영역에 전경이 조금이라도 있으면 전경으로 유지한다. | `native_mask_node.py` |
| 경계 후처리 | 4방향 연결 요소 필터를 기본적으로 모델 원래 해상도에서 적용한다. `mask_component_subsampling:=1`이 기본이며 `:=2`로 이전 subsampling 방식을 비교한다. | `native_mask_node.py`, 메인 런치 |
| 중복 필터 제거 | 최적화 모드에서는 `multi_mapper.remove_small_connected_components=False`로 nvblox의 중복 필터를 끈다. | 메인 런치 |
| RGB 매핑 | nvblox가 확대된 모델 RGB 대신 원본 카메라 RGB/CameraInfo와 `people_mask_native`를 사용한다. 따라서 overlay 크기도 원본 카메라 크기가 된다. | 메인 런치 |
| 전처리 옵션 | resize 출력에 `encoding_desired=rgb8`을 명시하고 `input_qos`를 전달할 수 있게 했다. 기존 BGR 변환과 `[-1,1]` 정규화는 유지한다. | `patches/vanilla_segmentation_preprocessing.patch` |
| 측정 도구 | 토픽별 수신 FPS, source header age의 median/p95 등을 기록하는 실행 파일을 추가했다. | `perception_profile.py` |

새 런치 옵션의 기본값:

```text
optimize_perception=True
max_segmentation_in_flight=3
limit_segmentation_input=True
mask_component_subsampling=1
segmentation_resolution=960x544
vanilla_engine_file_path=''  # 비우면 선택 해상도에 맞춰 엔진 경로 계산
```

`optimize_perception:=False segmentation_resolution:=960x544`로 입력 제한·네이티브
마스크 매핑 최적화를 끌 수 있다. 다만 깊이 입력 우회, C++ 동기화·timestamp 수정,
기본 YAML 변경까지 되돌리는 옵션은 아니므로 GitHub 원본 전체와 같아지지는 않는다.

이름만 보고 신규 변경으로 오인하지 않을 부분:
**BGR 변환과 `[-1,1]` 정규화 자체는 GitHub의 기존 패치에 이미 포함**돼 있다.
현재 새로 늘어난 부분은 resize encoding/QoS 지정과 새 노드·프로필 연결이다.

## 4. RealSense 깊이 입력과 중복 실행 방지

- nvblox 깊이 입력을 `/camera0/realsense_splitter_node/output/depth`에서
  **`/camera0/camera/depth/image_rect_raw`로 변경**했다.
- 메인 런치와 외부 의존 저장소의 `get_realsense_remappings()` 양쪽에 반영했다.
- `frame_emitter_mode` 메타데이터가 없는 카메라에서도 이 경로로 nvblox에 깊이를 공급한다.
- splitter는 기본적으로 모든 카메라에서 비활성화했다. 현재 구성은 cuVSLAM을 사용하지 않으며,
  추후 splitter의 IR 출력을 사용할 때는 이 설정을 재검토해야 한다.
- 카메라 런치에 USB serial 기반 `flock` 잠금과 기존 `/dev/video*` 사용 프로세스 확인을 추가했다.
  다른 ROS 도메인에서도 같은 물리 카메라의 중복 실행을 막기 위한 로컬 장치 점유 검사다.
- 변경은 `patches/realsense_camera_topic_namespace.patch`에도 저장돼 있다.

참고로 기존 카메라의 IMU 비활성화, emitter flashing 비활성화, frame/topic 이름 조정은
GitHub 패치에 이미 있던 부분이다. 전부 이번 로컬 신규 변경으로 세지 않는다.

## 5. nvblox 설정 및 패치에 빠진 C++ 수정

주 저장소의 `src/my_people_nvblox_bringup/config/nvblox/nvblox_base.yaml` 변경:

| 설정 | GitHub | 로컬 |
|---|---:|---:|
| `maximum_input_queue_length` | 8 | 3 |
| `static_mapper.projective_integrator_max_integration_distance_m` | 5.0 m | 3.0 m |
| `print_timings_to_console` | 해당 줄 없음 | `true` |
| `print_rates_to_console` | 해당 줄 없음 | `true` |
| `print_statistics_on_console_period_ms` | 해당 줄 없음 | `5000` |

다음 수정은 **외부 의존 저장소의 작업 파일에만 있고, 현재 주 저장소의 어느 패치에도 없다.**
따라서 주 저장소 변경분만 옮기면 누락된다.

| 파일 (`src/isaac_ros_nvblox/` 아래) | 원본 → 로컬 |
|---|---|
| `nvblox_ros/include/nvblox_ros/nvblox_node.hpp` | color+mask synchronizer를 ExactTime에서 ApproximateTime 타입으로 변경; overlay timestamp를 전달하는 함수 인자 추가 |
| `nvblox_ros/src/lib/nvblox_node.cpp` | color+mask ApproximateTime 생성; `kQueueSize` 10 → 60; color/depth overlay에 source `header.stamp` 기록 |

`kQueueSize=60`은 message_filters 동기화 큐이고, YAML의
`maximum_input_queue_length=3`은 nvblox 처리 대기 큐다. 서로 다른 설정이다.

의존 저장소 기준은 `isaac_ros_nvblox.repos`에 고정된
`7908a183acf84f4f1ab3fda7b6d6caf3eefc1f78`이고 실제 HEAD도 같다.
내부 `nvblox_core` HEAD도 원래 gitlink인 `669ff1071529ed3c7e811ab584572753eb82b71b`와 같다.

원본 소스에 패치를 적용한 임시 사본과 실제 파일을 비교한 결과:

- 현재 로컬 RealSense·전처리 패치는 해당 런치 변경을 모두 재현한다.
- `nvblox_skip_empty_deletion.patch`는 GitHub 원본에서 바뀌지 않았으며,
  `layer_publishing.cpp`의 수정은 이 기존 패치로 완전히 재현된다. 신규 변경으로 세지 않는다.
- `realsense_splitter/COLCON_IGNORE` 삭제는 원본 README에 이미 있는 설치 절차다.
- 외부 소스의 RViz topic 변경과 `vslam.launch.py` 변경도 GitHub의 기존 패치로 재현된다.
- LFS 필터를 끄면 rosbag `.db3`와 `nvblox_core`의 GIF가 변경처럼 보이지만,
  두 실제 파일의 SHA-256이 원본 LFS 포인터와 일치함을 확인했다. 소스 변경에서 제외한다.

## 6. 로봇 제어·오류 복구 변경

주 저장소:

- `pick_place_node.py`에 `/error_recovery_service_server/error_recovery` 서비스 클라이언트와
  `clear_robot_error()`를 추가했다. 준비 완료 시와 **각 `move_to()` 전에** 호출한다.
- `success=False, error='No errors'`는 오류가 없는 정상 응답으로 처리한다.
- `controllers_real.yaml`과 `controllers_fake.yaml`의 `constraints.goal: 0.05`를
  `constraints.panda_joint1.goal`부터 `panda_joint7.goal`까지의 개별 설정으로 고쳤다.

주 저장소에서 제외된 `src/franka_arm_ros2/`의 추가 수정:

| 파일 | 로컬 변경 |
|---|---|
| `franka_hardware/src/franka_hardware_interface.cpp` | 미복구 오류로 write가 생략될 때 1초 간격 제한 경고를 출력 |
| `franka_hardware/src/franka_error_recovery_service_server.cpp` | 복구할 오류가 없다는 로그를 INFO → DEBUG로 변경 |
| `franka_robot_state_broadcaster/include/franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp` | 마지막 발행 시각과 첫 update 상태를 저장하는 멤버 추가 |
| `franka_robot_state_broadcaster/src/franka_robot_state_broadcaster.cpp` | 설정한 `frequency`를 이용해 발행 주기를 제한하고 activate 시 초기화 |
| `README.md` | broadcaster 문제와 로컬 수정 설명 추가 |

실제 HEAD는 `franka_arm_ros2.repos`에 고정된
`6867bde68970104118d04ae853ddded15bf857fb`와 같다.
위 5개 파일은 **현재 작업 트리의 미커밋 변경**이며, 주 저장소 패치에는 들어 있지 않다.

오류 복구 호출을 추가한 것이 reflex 원인 자체를 제거했다는 의미는 아니다.
[2026-10-20 전체 시스템 관찰](REFLEX_OBSERVATION_2026-10-20_STAGE11_WHOLE_SYSTEM.md)에서는
10분 중 경고→복구 구간 4개가 기록됐다. 실제 물체 이송 성공이나 무오류 운전 검증으로 해석하지 않는다.

## 7. 주 저장소에서 바뀐 추적 파일 전체 목록

위 내용을 GitHub에 반영할 때 검토할 11개 파일이다.

| 경로 | 변경 요약 |
|---|---|
| `README.md` | 기본 생성 예시의 `--fp16` 제거, 별도 424×240 엔진 생성·인지 최적화·중복 카메라 방지·측정 문서 설명 추가; 카메라 전용 예시를 424×240×30으로 변경 |
| `patches/realsense_camera_topic_namespace.patch` | 깊이 splitter 우회, splitter 비활성화, 카메라 중복 점유 차단 추가 |
| `patches/vanilla_segmentation_preprocessing.patch` | resize RGB encoding 및 입력 QoS 옵션 추가 |
| `src/my_people_nvblox_bringup/config/nvblox/nvblox_base.yaml` | 처리 큐 3, 통합 거리 3 m, 주기적 timing/rate 출력 |
| `src/panda_pick_place/config/controllers_fake.yaml` | 관절별 goal tolerance |
| `src/panda_pick_place/config/controllers_real.yaml` | 관절별 goal tolerance |
| `src/panda_pick_place/package.xml` | `python3-numpy`, `python3-opencv` 실행 의존성 추가 |
| `src/panda_pick_place/panda_pick_place/pick_place_node.py` | 오류 복구 서비스 호출 추가 |
| `src/panda_pick_place/setup.py` | `native_mask_node`, `segmentation_input_node`, `perception_profile` 실행 파일 등록 |
| `src/panda_real_bringup/launch/panda_realsense_people.launch.py` | 새 해상도·최적화 옵션, 엔진 자동 선택, 새 노드 및 topic 연결 |
| `src/panda_real_bringup/package.xml` | `panda_pick_place` 실행 의존성 추가 |

주의: README의 로봇 포함 예시는 여전히 848×480×30이며, 카메라 전용 예시만 424×240×30으로 바뀌었다.
`segmentation_resolution` 기본값이 424×240으로 바뀐 것은 아니다.

## 8. GitHub에 없는 추가 파일과 로컬 엔진

새 코드 5개는 아직 미추적 상태다. `setup.py`만 옮기면 새 실행 파일의 구현이 빠진다.

- [native_mask_node.py](native_mask_node.py)
- [segmentation_input_node.py](segmentation_input_node.py)
- [perception_profile.py](perception_profile.py)
- [scripts/build_native_segmentation_engine.py](scripts/build_native_segmentation_engine.py)
- [tests/test_perception_helpers.py](tests/test_perception_helpers.py): 현재 helper 검증 8개가 작성돼 있다. 이번 문서 작업에서 다시 실행하지 않았다.

작성 전 미추적 자료 73개 구성:

| 종류 | 수 | 용도 |
|---|---:|---|
| Python | 5 | 위 새 코드·도구·검증 |
| Markdown | 11 | 인지 최적화·모델/경계 실험 3개, reflex 관찰 단계 4~11의 기록 8개 |
| JSON | 4 | 인지 속도·입력 resize·작은 모델·마스크 경계 측정 |
| JSONL | 8 | reflex 관찰 단계별 시계열 |
| LOG | 43 | 실행 로그 사본 |
| TXT | 2 | ping 측정 기록 |

이 문서 자체는 위 작성 전 집계에 포함하지 않는다. 로그·측정 자료는 동작 코드와 구분해 보존한다.
생성된 두 `.plan` 파일은 무시되는 로컬 산출물이라 위 미추적 집계에도 포함되지 않는다.
x86 PC에서는 그 PC의 GPU/TensorRT 환경에 맞춰 엔진을 재생성해야 한다.

실험 결과를 확인할 문서:

- [PERCEPTION_PERFORMANCE.md](PERCEPTION_PERFORMANCE.md): 기존 960×544 엔진을 유지한 입력·전송·매핑 최적화.
- [NATIVE_SEGMENTATION_424x240.md](NATIVE_SEGMENTATION_424x240.md): 작은 해상도 엔진, padding/crop, 과거 약 30 FPS 측정, 마스크 비교 한계.
- [MASK_BOUNDARY_EXPERIMENT_2026-10-14.md](MASK_BOUNDARY_EXPERIMENT_2026-10-14.md): 원래 해상도 연결 요소 후처리 및 과거 약 30 FPS 측정.
- [REFLEX_OBSERVATION_2026-10-20_STAGE11_WHOLE_SYSTEM.md](REFLEX_OBSERVATION_2026-10-20_STAGE11_WHOLE_SYSTEM.md): 최근 전체 시스템 로그 관찰.

위 FPS·지연값은 해당 실험 환경의 수신 측정값이며, x86 성능이나 현재 실시간 처리율을 보장하지 않는다.

## 9. 다른 PC에서 재현할 때 빠뜨리기 쉬운 내용

1. 주 저장소의 수정 파일 11개 외에 **새 Python 파일 5개**도 필요하다.
2. 해상도에 맞는 엔진을 만들고 선택한다. 작은 엔진을 원래 이름으로 덮어쓰지 않는다.
3. 로컬 수정 패치를 적용해도 **nvblox의 C++ 동기화·timestamp 변경 2개 파일은 별도**로 필요하다.
4. `franka_arm_ros2` 변경 5개도 주 저장소에서 자동으로 전달되지 않는다.
5. 기존 BGR/정규화·빈 삭제 메시지 방지 패치는 GitHub에 이미 있으므로 중복 변경으로 처리하지 않는다.
6. Jetson 전용 커널·전원·NIC·IRQ 설정은 이 목록에 포함하지 않는다.

확인에 사용한 비교 명령은 다음과 같다. 아래 명령은 실행 예시이며, 이 문서만으로 변경분을 자동 적용하지 않는다.

```bash
git ls-remote https://github.com/IRaC-Lab/Real-Panda-Human-Voxel-Distance.git HEAD refs/heads/main
git -C /home/esjin/panda_real_ws diff 65960499ecd0c93b588e51c55e729e350a75afae --stat
git -C /home/esjin/panda_real_ws ls-files --others --exclude-standard
git -C /home/esjin/panda_real_ws/src/isaac_ros_nvblox diff -- nvblox_ros/include nvblox_ros/src
git -C /home/esjin/panda_real_ws/src/franka_arm_ros2 diff --stat
```

## 10. 누락 여부 재점검 (2026-10-21)

후속 확인에서는 `git status` 결과 외에 각 저장소 HEAD의 추적 파일 내용과 실제 파일을
직접 비교했다. 일반 파일은 Git blob 해시, LFS 파일은 포인터의 SHA-256으로 비교했고,
삭제·실행 권한·서브모듈 커밋도 확인했다. 따라서 LFS 다운로드 완료 자체를 변경으로 세지 않는다.

| 확인 범위 | HEAD 추적 항목 수 | 원시 차이 | 해석 |
|---|---:|---:|---|
| 주 저장소 `panda_real_ws` | 88 | 11 | 7절의 목록과 일치 |
| 외부 `isaac_ros_nvblox` | 203 | 9 | 기존 패치·설치 절차로 생기는 차이도 포함. 로컬 수정 패치에 반영된 런치 변경 외에 별도로 옮길 C++ 파일은 2개 |
| 내부 `nvblox_core` | 549 | 0 | 소스·LFS 내용 및 기준 커밋 일치 |
| 외부 `franka_arm_ros2` | 152 | 5 | 6절의 목록과 일치. 주 저장소 패치에 미포함 |
| 워크스페이스 밖 `/home/esjin/libfranka` | 127 | 0 | README가 지정한 0.9.2이며 로컬 추적 소스 변경 없음 |
| 위 libfranka의 `common` | 10 | 0 | 기준 커밋·추적 파일 내용 일치 |

항목 수에는 서브모듈 gitlink도 포함된다. 원시 차이 수는 해당 저장소 HEAD와 비교한
변경·삭제 파일 수이며, GitHub의 설치 패치까지 적용한 결과와의 신규 차이 수는 아니다.
외부 의존 저장소의 미추적 파일과 Git에서 무시하는 파일도 확인했으며, 추가로 기록할
미등록 소스는 발견하지 못했다. 주 저장소의 추가 코드 5개는 이미 8절에 기록돼 있다.

설치 산출물도 제한적으로 확인했다.

- 설치된 메인 런치, RealSense 런치, `controllers_real.yaml`, `nvblox_base.yaml`은 현재 소스 파일로 연결된 심볼릭 링크다.
- Python 패키지는 `egg-link`로 build 경로에 연결되는 editable 설치다. `site-packages`에 `.py` 사본이 없다는 사실만으로 설치 누락이라고 판단하지 않는다.
- 확인한 nvblox·Franka C++ 객체 파일의 수정 시각은 해당 소스보다 뒤였다.
  시각만으로 실제 실행 바이너리의 내용·빌드 옵션까지 일치한다고 보장하지는 않는다.
- 커널·드라이버·apt 패키지·systemd·셸 환경 같은 시스템 차이와 현재 프로세스의 로딩 상태는 이 소스 비교의 범위 밖이다.

이번 재점검에서 위 목록에 추가할 소스 차이는 발견되지 않았다.
모델 해상도 외에도 입력 제한, 마스크 처리, 깊이 경로, 동기화·timestamp,
로봇 오류 복구·허용오차·발행 주기 변경이 있다는 점을 함께 봐야 한다.

## 11. 변경의 이점·절충 및 남은 검증

평가일: 2026-10-21. 아래 평가는 확인한 코드와 기존 실험 기록에 근거하며,
이번 문서 작성 중 새로 성능·정확도·로봇 동작을 시험한 결과는 아니다.

전반적으로 기능 오류 수정과 처리 속도·지연 개선을 목표로 한 변경이다.
그러나 모든 변경이 정확도·효율·안정성을 동시에 높였다고 평가하지 않는다.

깊이 입력이 끊기던 경로 수정, 관절별 목표 허용오차 설정, source timestamp 보존,
카메라 중복 실행 차단은 각각 확인된 문제에 대응한다. broadcaster 발행 주기 처리와
미복구 오류 경고도 코드에서 확인한 문제를 다룬다. 이 개별 수정의 타당성과 전체
시스템의 무오류·안전 운전 검증은 구분한다.

| 변경 | 이점 | 절충 또는 아직 확인할 점 |
|---|---|---|
| 424×240 모델 프로필 추가 | 내부 연산 면적 감소, 과거 실험에서 최종 overlay까지 약 30 FPS 관측 | 기존 모델과 예측이 달라진다. 정답 라벨 기반 정확도 우위 및 실제 로봇 환경에서의 적용 적합성은 미확정 |
| 최신 프레임 선택·미완료 입력 수 제한 | 오래된 입력이 추론 대기열에 쌓이는 것을 줄임 | 모든 카메라 프레임을 처리하는 방식이 아니며 일부 프레임은 선택되지 않음 |
| nvblox 처리 큐 8→3 | backlog와 지연 감소를 목표로 함 | 부하가 높으면 일부 프레임을 더 일찍 버림. 최적값은 대상 PC에서 확인 필요 |
| 통합 거리 5→3 m | 먼 배경의 매핑 연산 감소 | 지도에 반영하는 깊이 범위도 줄어든다. 필요한 관측 범위인지 확인 필요 |
| color+mask ExactTime→ApproximateTime | 정확히 일치하는 timestamp가 없어 매칭이 막히던 문제 완화 | 허용되는 시간 차이를 가진 프레임이 매칭될 수 있으므로 동적 장면에서 정합 확인 필요 |
| 동기화 큐 10→60 | 마스크가 늦게 도착해 대응 영상이 먼저 빠지는 현상 완화 | 대기 지연이 늘 수 있다. 처리 큐 3과는 다른 큐이며, 해당 값이 모든 환경의 최적값은 아님 |
| 원래 마스크 해상도에서 연결 요소 필터 | 얇게 붙은 전경과 좁은 틈을 덜 잃도록 처리 | 기존 subsampling 방식과 결과·처리 비용이 다르다. 모델이 이미 놓친 부위를 복원하는 기능은 아님 |
| FP16 대신 검증한 기존 엔진 사용 | 기존 실험의 화면 전체 전경 오분류 회피 | 낮은 정밀도의 속도 이점을 포기할 수 있다. x86의 GPU/TensorRT 조합에서도 출력 검증 필요 |
| 매 이동 전 자동 오류 복구 | latched error 이후 제어가 계속 무시되는 상황에서 복구를 시도 | reflex 발생 원인 자체를 제거하지 않는다. 동작 재개를 무오류 운전으로 해석하지 않음 |
| timing/rate 디버그 출력 활성화 | 지연·처리율 원인 관찰에 도움 | 측정용 설정이 남아 있는 상태이며 로그와 계측 부하가 있다. 상시 운용 시 필요성 검토 |

최근 전체 시스템 10분 관찰에는 경고→복구 구간 4개가 기록돼 있다(6절 및 연결된
관찰 문서 참조). 따라서 현재 상태는 **기능 오류를 수정하고 처리 속도·지연을 개선했으며,
모델 정확도와 전체 시스템 안정성은 추가 검증이 필요한 상태**로 기록한다.
과거 수신 FPS 측정만으로 x86 성능, 사람 인식 정확도 또는 충돌 회피 성능을 보장하지 않는다.
