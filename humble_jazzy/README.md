# ROS Humble과 Jazzy 통신 테스트

이 프로젝트는 ROS Humble과 ROS Jazzy 간의 통신 호환성을 테스트하기 위한 환경을 제공합니다.

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
cd humble_jazzy
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
cd humble_jazzy
docker-compose --profile python_test up
```

### 개별적으로 실행하기

Humble Publisher만 실행:
```bash
docker-compose up ros_humble
```

Jazzy Subscriber만 실행:
```bash
docker-compose up ros_jazzy
```

## 테스트 내용

이 테스트는 ROS Humble에서 발행된 메시지가 ROS Jazzy 환경에서 정상적으로 수신되는지 확인합니다.
테스트 메시지에는 다음 필드가 포함됩니다:
- `id`: 메시지 ID (정수)
- `message`: 텍스트 메시지 (문자열)
- `data`: 숫자 데이터 (부동소수점)
- `stamp`: 타임스탬프 (시간)

Subscriber 노드는 메시지 수신 시 지연 시간을 계산하여 표시합니다.

## Dockerfile 접근 방식의 장점

1. 미리 빌드된 이미지를 사용하므로 실행 시간이 단축됩니다.
2. 빌드 환경과 실행 환경이 분리되어 있습니다.
3. 빌드된 이미지를 재사용할 수 있습니다.
4. 코드 변경이 없다면 빌드 단계를 건너뛸 수 있습니다.

## 문제 해결

호스트 네트워크 모드를 사용하므로 로컬 네트워크 설정에 따라 통신이 영향을 받을 수 있습니다.
문제가 발생할 경우 다음을 확인하세요:
- 방화벽 설정
- ROS_DOMAIN_ID 설정
- Docker 네트워크 설정 