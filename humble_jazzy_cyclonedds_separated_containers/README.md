# ROS Humble과 Jazzy 통신 테스트 (분리된 컨테이너)

이 프로젝트는 ROS Humble과 ROS Jazzy 간의 통신이 완전히 별도의 컨테이너에서도 작동하는지 테스트하기 위한 환경을 제공합니다.

## 분리된 컨테이너 테스트 목적

이 테스트는 서로 다른 Docker Compose 파일로 실행된 컨테이너 간에도 ROS 2 통신이 제대로 작동하는지 확인하기 위해 설계되었습니다. 특히 아래 사항을 검증합니다:

1. 동일한 호스트 네트워크를 사용하는 별도 컨테이너 간 통신 가능 여부
2. 서로 다른 배포판 간 DDS 검색(discovery) 기능 정상 작동 여부
3. 실행 시간이 다른 노드 간 동적 검색 가능 여부

## 테스트 구성

- `humble-compose.yml`: ROS Humble 노드만 실행하는 Docker Compose 파일
- `jazzy-compose.yml`: ROS Jazzy 노드만 실행하는 Docker Compose 파일

## 구조

- `src/test_msgs`: 테스트에 사용되는 메시지 정의
- `src/test_nodes`: 테스트 노드 (C++ 및 Python)
- `Dockerfile.humble`: ROS Humble 환경을 위한 Dockerfile
- `Dockerfile.jazzy`: ROS Jazzy 환경을 위한 Dockerfile
- `ros_entrypoint.sh`: ROS 환경 설정을 위한 엔트리포인트 스크립트

## 사용 방법

### 이미지 빌드

1. 두 Docker 이미지 모두 빌드:
```bash
cd humble_jazzy_cyclonedds_separated_containers
docker-compose -f humble-compose.yml build
docker-compose -f jazzy-compose.yml build
```

### 별도 컨테이너 실행

1. 첫 번째 터미널에서 Humble 컨테이너 실행:
```bash
docker-compose -f humble-compose.yml up
```

2. 두 번째 터미널에서 Jazzy 컨테이너 실행:
```bash
docker-compose -f jazzy-compose.yml up
```

## 테스트 내용

이 테스트는 ROS Humble에서 발행된 메시지가 별도로 실행된 ROS Jazzy 환경에서 정상적으로 수신되는지 확인합니다.
테스트 메시지에는 다음 필드가 포함됩니다:
- `id`: 메시지 ID (정수)
- `message`: 텍스트 메시지 (문자열)
- `data`: 숫자 데이터 (부동소수점)
- `stamp`: 타임스탬프 (시간)

## 주요 고려사항

1. **네트워크 설정**: 두 컨테이너 모두 `network_mode: "host"`를 사용하여 호스트의 네트워크를 공유합니다.
2. **ROS_DOMAIN_ID**: 두 컨테이너가 동일한 `ROS_DOMAIN_ID`(42)를 사용하여 서로 통신할 수 있습니다.
3. **DDS 구현체**: 두 컨테이너 모두 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`를 사용하여 CycloneDDS를 통해 통신합니다.

## 문제 해결

호스트 네트워크 모드를 사용하므로 로컬 네트워크 설정에 따라 통신이 영향을 받을 수 있습니다.
문제가 발생할 경우 다음을 확인하세요:
- 방화벽 설정
- ROS_DOMAIN_ID 설정
- Docker 네트워크 설정 

### 시간 소스 불일치 문제 해결

ROS2의 다른 배포판 간 통신 시 (예: Humble과 Jazzy) 다음과 같은 런타임 오류가 발생할 수 있습니다:
```
terminate called after throwing an instance of 'std::runtime_error'
what(): can't subtract times with different time sources [1 != 2]
```

이 오류는 서로 다른 시간 소스를 가진 타임스탬프를 비교하거나 연산하려고 할 때 발생합니다.

#### 해결 방법
메시지의 타임스탬프를 사용할 때는 명시적으로 시간 소스를 지정해야 합니다:

```cpp
// 올바른 방법
auto now = this->now();  // RCL_ROS_TIME 사용
auto msg_time = rclcpp::Time(msg->stamp.sec, msg->stamp.nanosec, RCL_ROS_TIME);  // 명시적으로 RCL_ROS_TIME 지정
auto latency = now - msg_time;  // 정상 작동
```

이는 특히 다음과 같은 상황에서 발생할 수 있습니다:
- 서로 다른 ROS2 배포판 간 통신 (예: Humble과 Jazzy)
- message_filters 또는 tf와 같은 라이브러리 사용
- 메시지의 타임스탬프를 이용한 시간 계산

#### CycloneDDS의 장점
CycloneDDS를 RMW 구현체로 사용하면 다른 배포판 간 호환성이 향상될 수 있습니다. 다음과 같이 설정하세요:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

이 프로젝트는 docker-compose.yml에 이미 이 설정을 포함하고 있습니다. 