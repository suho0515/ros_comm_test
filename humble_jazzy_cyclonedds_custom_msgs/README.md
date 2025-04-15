# ROS Humble과 Jazzy 통신 테스트 (CycloneDDS + 커스텀 메시지)

이 프로젝트는 ROS Humble과 ROS Jazzy 간의 커스텀 메시지(`test_msgs`) 통신 호환성을 CycloneDDS를 사용하여 테스트하기 위한 환경을 제공합니다.

## 커스텀 메시지 테스트 목적

이 테스트는 서로 다른 ROS 2 배포판 간에 커스텀 메시지 타입이 올바르게 전송되고 해석되는지 확인하기 위해 설계되었습니다. 특히 아래 사항을 검증합니다:

1. 서로 다른 ROS 2 배포판 간 커스텀 메시지 정의 호환성
2. 메시지 직렬화/역직렬화 과정의 올바른 처리
3. 복합 데이터 타입(문자열, 숫자, 타임스탬프 등)의 올바른 전송

## CycloneDDS 테스트 결과

**결과: ✅ 성공**

서로 다른 ROS 2 배포판(Humble과 Jazzy) 간 CycloneDDS를 사용한 커스텀 메시지 통신 테스트는 성공했습니다.
Publisher(Humble)에서 발행한 메시지가 Subscriber(Jazzy)에서 정상적으로 수신되었습니다.

### 장점:
1. CycloneDDS는 서로 다른 ROS 2 배포판 간에 뛰어난 호환성을 제공합니다.
2. 타입 해시 불일치 문제에 대해 더 관대하게 처리합니다.
3. 별도의 XML 설정 파일 없이도 잘 작동합니다.

### 권장 사항:
서로 다른 ROS 2 배포판 간 통신이 필요한 경우 CycloneDDS를 사용하는 것을 적극 권장합니다.
자세한 비교 결과는 [메인 README](../README.md)를 참조하세요.

## 구조

- `src/test_msgs`: 테스트에 사용되는 메시지 정의
- `src/test_nodes`: 테스트 노드 (C++ 및 Python)
- `Dockerfile.humble`: ROS Humble 환경을 위한 Dockerfile
- `Dockerfile.jazzy`: ROS Jazzy 환경을 위한 Dockerfile
- `ros_entrypoint.sh`: ROS 환경 설정을 위한 엔트리포인트 스크립트

## 사용 방법

### 이미지 빌드 및 실행

1. 이미지 빌드 및 실행:
```bash
cd humble_jazzy_cyclonedds_custom_msgs
docker-compose build
docker-compose up
```

이 명령은 다음을 수행합니다:
- Dockerfile을 사용하여 ROS Humble 및 Jazzy 환경의 이미지를 빌드합니다.
- Humble 환경에서 Publisher 노드를 실행합니다.
- Jazzy 환경에서 Subscriber 노드를 실행합니다.

### Python 노드로 테스트

1. 빌드 및 실행:
```bash
cd humble_jazzy_cyclonedds_custom_msgs
docker-compose --profile python_test up
```

### 개별적으로 실행하기

Humble Publisher만 실행:
```bash
docker-compose up ros_humble_custom
```

Jazzy Subscriber만 실행:
```bash
docker-compose up ros_jazzy_custom
```

## 테스트 내용

이 테스트는 ROS Humble에서 발행된 커스텀 메시지가 ROS Jazzy 환경에서 정상적으로 수신되는지 확인합니다.
테스트 메시지에는 다음 필드가 포함됩니다:
- `id`: 메시지 ID (정수)
- `message`: 텍스트 메시지 (문자열)
- `data`: 숫자 데이터 (부동소수점)
- `stamp`: 타임스탬프 (시간)

Subscriber 노드는 메시지 수신 시 지연 시간을 계산하여 표시합니다.

## 커스텀 메시지 정의

테스트에 사용된 커스텀 메시지 정의는 다음과 같습니다:

```
# test_msgs/msg/TestMessage.msg
int32 id
string message
float64 data
builtin_interfaces/Time stamp
```

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

#### 원인
- ROS2에서는 RCL_SYSTEM_TIME(1)과 RCL_ROS_TIME(2)의 두 가지 주요 시간 소스를 사용합니다.
- 기본 생성자 `rclcpp::Time(msg->stamp.sec, msg->stamp.nanosec)`는 기본적으로 RCL_SYSTEM_TIME을 사용합니다.
- `node->now()`는 RCL_ROS_TIME을 사용합니다.
- 서로 다른 시간 소스를 가진 시간끼리는 연산(예: 뺄셈)이 불가능합니다.

#### 해결 방법
메시지의 타임스탬프를 사용할 때는 명시적으로 시간 소스를 지정해야 합니다:

```cpp
// 잘못된 방법 (오류 발생)
auto now = this->now();  // RCL_ROS_TIME 사용
auto msg_time = rclcpp::Time(msg->stamp.sec, msg->stamp.nanosec);  // 기본값 RCL_SYSTEM_TIME 사용
auto latency = now - msg_time;  // 오류: 서로 다른 시간 소스

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