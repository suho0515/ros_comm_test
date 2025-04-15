# ros_comm_test

이 저장소는 다양한 ROS 2 환경 간의 통신 테스트를 제공합니다.

## 프로젝트 구조

- `humble_jazzy_cyclonedds/`: CycloneDDS를 사용한 ROS Humble-Jazzy 통신 테스트
- `humble_jazzy_fastdds/`: FastRTPS(FastDDS)를 사용한 ROS Humble-Jazzy 통신 테스트
- `humble_jazzy_cyclonedds_custom_msgs/`: CycloneDDS를 사용한 ROS Humble-Jazzy 커스텀 메시지 통신 테스트

## DDS 구현체 테스트 결과

다양한 DDS(Data Distribution Service) 구현체를 테스트하여 ROS Humble과 ROS Jazzy 간의 통신 호환성을 확인했습니다.

### CycloneDDS와 FastRTPS 비교

| DDS 구현체 | 통신 성공 여부 | 비고 |
|------------|----------------|------|
| CycloneDDS | ✅ 성공 | 서로 다른 ROS 2 배포판 간에 안정적인 통신 제공 |
| FastRTPS/FastDDS | ❌ 실패 | 다른 ROS 2 배포판 간 메시지 교환 실패 |

### 테스트 환경
- ROS Humble (발행자)
- ROS Jazzy (구독자)
- 동일한 테스트 메시지 정의 및 노드 코드 사용
- 동일한 테스트 환경 (Docker 컨테이너, 네트워크 설정 등)

### 커스텀 메시지 테스트

서로 다른 ROS 2 배포판 간에 커스텀 메시지를 통한 통신도 테스트하였습니다:

| 테스트 환경 | 메시지 타입 | 통신 성공 여부 |
|------------|------------|----------------|
| CycloneDDS | 커스텀 메시지 (`test_msgs/TestMessage`) | ✅ 성공 |

커스텀 메시지 테스트에 사용된 메시지 정의:
```
# test_msgs/msg/TestMessage.msg
int32 id
string message
float64 data
builtin_interfaces/Time stamp
```

### 결론

ROS 2의 서로 다른 배포판 간 통신을 위해서는 **CycloneDDS**를 사용하는 것이 권장됩니다. CycloneDDS는 다음과 같은 이점을 제공합니다:

1. 다른 ROS 2 배포판 간 더 나은 호환성
2. 타입 해시 불일치 문제에 대한 더 우수한 처리
3. 간단한 설정 (추가 XML 설정 파일 없이도 작동)
4. 커스텀 메시지 타입의 안정적인 직렬화/역직렬화 지원

FastRTPS(FastDDS)는 기본 DDS 구현체이지만, 다른 ROS 2 배포판 간 통신에는 더 엄격한 타입 검사와 호환성 문제로 인해 제약이 있을 수 있습니다.

## 자세한 정보

각 테스트 환경의 README 파일을 참조하세요:
- [CycloneDDS 테스트](./humble_jazzy_cyclonedds/README.md)
- [FastRTPS 테스트](./humble_jazzy_fastdds/README.md)
- [커스텀 메시지 테스트 (CycloneDDS)](./humble_jazzy_cyclonedds_custom_msgs/README.md)