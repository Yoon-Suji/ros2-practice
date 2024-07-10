# ROS 2
## ROS란?
### ROS란?

- **ROS is an open-source, meta-operating system for your robot.**
    - **메타 운영체제 (meta-OS)**: 독립된 OS X, OS위에 설치되어 OS에서 제공하는 것들을 사용 (파일 시스템, process 관리 시스템 등) + 로봇 응용 프로그램에 필요한 필수 기능을 라이브러리 형태로 제공
    - 미들웨어, 소프트웨어 프레임워크
    - 사용자들이 개발한 패키지를 유통하는 생태계를 갖고 있음
    - 기존의 전통적인 OS를 이용하면서, 로봇 App 개발에 필수인 로봇과 센서를 HW 추상화 개념으로 제어하고, 사용자의 로봇 App 개발을 위한 지원 시스템이자 도구

### ROS의 목적

- "로보틱스 SW 개발을 전 세계레벨에서 공동 작업이 가능하도록 환경을 구축하는 것"
- 연구 및 개발에서의 코드 재사용

### ROS의 구성

- **400여 개의 ROS 2 공통 패키지 (Common Packages)로 구성** - Simulation, Robotics Application Framework, Software Development Tool 등..

## ROS 2의 특징
### Platforms

- **Linux, window, macOS** 모두 지원

### Real-time

- 실시간성 지원 - 선별된 HW, Real time supportive OS, RTPS 프로토콜 등을 전제조건으로 할 때

### Security

- 기존: TCPROS는 노드를 관리하는 ROS master의 IP, PORT만 노출되면 모든 시스템 다운가능
- TCP기반 통신 -> **DDS(Data Distribution System)** 도입

### Communication

- **RTPS**를 지원하는 통신 미들웨어 DDS 사용
    - 노드 간 자동 감지 기능 - ROS Master 없이도 통신 가능

### Middleware interface

- **RMW**

### Node manager (Discovery)

- ROS Master 사라짐 -> Dynamic discovery 이용해 DDS 미들웨어를 통해 노드를 직접 검색하여 연결

### Languages

- Python, C++

### Build system

- ament

## DDS
### DDS (Data Distribution System)
- DDS 사양을 만족하는 미들웨어 API, 데이터 통신을 위한 미들웨어
- OS와 App 사이에 있는 소프트웨어 계층
- **UDP**: UDP 멀티캐스트 통신이기 때문에 별도의 설정을 하지 않으면 동일 네트워크 상의 모든 노드가 연결됨 -> ROS_DOMAIN_ID 환경변수를 설정해서 도메인 변경 가능
- Data Centric
- **Dynamic Discovery (동적 검색)** - 어떤 토픽이 지정 도메인 영역에 있으며 어떤 노드가 이를 발신하고 수신하는 지 알 수 있음. -> 노드를 DDS의 Participant로 취급

### QoS of DDS
#### Reliability

- 신뢰성 옵션
- **BEST_EFFORT**: 네트워크 상태에 따라 유실 발생 가능 - publisher가 BEST_EFFORT면 subscriber도 동일
- **RELIABLE**: 데이터 유실 발생 시 재전송을 통해 수신 보장

#### History

- 데이터를 몇 개나 보관할 지 결정
- **KEEP_LAST**: 정해진 메시지 큐 크기만큼의 데이터 보관 (depth = 메시지 큐의 크기)
- **KEEP_ALL**: 모든 데이터 보관 (메시지 큐 크기는 DDS 벤더마다 다름)

#### Durability

- subscriber 생성 전의 데이터를 사용할 것인지
- **TRANSIENT_LOCAL**: 생성 전 데이터도 보관 (publisher만 적용가능)
- **VOLATILE**: 생성 전 데이터 무효

#### Deadline

- 정해진 주기 안에 데이터가 발신/수신 되지 않을 경우 EventCallback 실행
- **deadline_duration**

#### Lifespan

- 정해진 주기 안에 수신되는 데이터만 유효 else 삭제
- **lifespan_duration**

#### Liveliness

- 정해진 주기 안에서 노드 혹은 토픽의 생사 확인
- liveliness: **AUTOMATIC**, **MANUAL_BY_NODE**, **MANUAL_BY_TOPIC**
- lease_duration

#### rmw_qos_profile
* 많이사용하는 QoS 설정을 세트로 표현한 것
- 사용자가 직접 커스텀해서 만들수도 있음

## 패키지와 노드

- **Node**: **최소 단위의 실행 가능한 프로세스**, ROS에서는 보통 노드 단위로 작업
- **Package**: 하나 이상의 노드 또는 노드 실행을 위한 정보를 묶어 놓은 것
- **Metapackage**: 패키지의 묶음

```bash
ros2 pkg list # 설치된 패키지 및 본인이 작성한 패키지 목록
ros2 executables <패키지명> # 패키지에 포함되어 있는 노드 목록
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

## 노드와 메시지 통신

- **노드 간에 입출력 데이터**를 주고받음 -> **Message**
  - Integer, Floating point, Boolean, String 과 같은 데이터 형태, 데이터 구조 또는 배열로도 사용 가능
- 메시지를 주고 받는 방법에 따라 **Topic**, **Service**, **Action**, **Parameter**

### 토픽 Topic

- **비동기성**, **연속성**
- **비동기식 단방향 메시지 송수신 방식 : Publisher - Subscriber**

```bash
ros2 topic info /turtle1/cmd_vel # 토픽 정보
ros2 topic echo <토픽이름> # 토픽 메시지 내용 실시간 표시
ros2 topic pub <topic_name> <msg_type> "<args>" --once
```

#### rosbag

- 퍼블리시되는 토픽을 파일형태로 저장하고 필요할때 저장된 토픽을 불러와 동일한 주기로 재생할 수 있는 기능

```
ros2 bag record <topic_name1> <topic_name2..>
ros2 bag info <rosbag_name>
ros2 bag play <rosbag_name>
```

### 서비스 Service

- **동기식 양방향 메시지 송수신 방식**
- **Service Client - Service Server** = N : 1 / **Service Request - Service Response**
- srv 인터페이스

### 액션 Action

- **비동기식/동기식 양방향 메시지 송수신 방식**
- Goal을 지정하는 **Action Client** - Feedback, Result를 전송하는 **Action Server**
- **토픽과 서비스의 혼합**
- Client = Service Client 3 + Topic Subscriber 2
- action 인터페이스

### 파라미터

- **노드 내/외부에서 노드 내 매개변수를 Set/Get 할 수 있음**
- 모든 노드가 자기만의 Parameter server/ client 가짐 -> 각 노드의 매개변수를 글로벌 매개변수처럼 사용 가능

## 인터페이스

- 노드 간에 데이터를 주고받기 위해 토픽, 서비스, 액션을 이용하는데 이때 사용되는 **데이터 형태(type)를 Interface**라고 함
- **토픽 == msg / 서비스 = srv / 액션 = action**
- 단순 자료형을 기본으로 함;

### 토픽: msg

```
float64 x
float64 y
```

### 서비스: srv

```
float32 x # Request
float32 y
string name
---
string name # Response
```

### 액션: action

```
float32 theta # 목표
---
float32 delta # 결과
---
float32 remaining # 피드백
```

## ROS 2의 표준 단위
### 표준 단위
SI derived unit
| 물리량            | 단위 (SI unit)   | 물리량                | 단위량 (SI unit) |
|-------------------|------------------|-----------------------|------------------|
| length (길이)     | meter (m)        | angle (평면각)        | radian (rad)     |
| mass (질량)       | kilogram (kg)    | frequency (주파수)    | hertz (Hz)       |
| time (시간)       | second (s)       | force (힘)            | newton (N)       |
| current (전류)    | ampere (A)       | power (일률)          | watt (W)         |
| temperature (온도)| celsius (°C)     | voltage (전압)        | volt (V)         |
| magnetism (자기장)| tesla            |                       |                  |

### 좌표 표현
* x forward, y left, z up
* 정회전 방향 == 반시계 방향
<img width="501" alt="images_leesy970906_post_e6042f9c-7c3b-4878-a288-8dca47ab6a33_Screen Shot 2021-04-08 at 4 42 02 PM" src="https://github.com/Yoon-Suji/ros2-practice/assets/70956926/9724bc95-0fe5-4add-8b74-1e7fd853aa25">

### 시계와 시간

- 로봇의 다양한 센서 간 시간 동기화 중요 -> **퍼블리시되는 토픽에 해당 토픽이 퍼블리시되는 시간 포함: std_msgs/msg/header**
- ROS 2에서 사용하는 기본 시계는 **System Clock = UTC**로 표시됨
- ROS Time: **use_sim_time := True** -> /clock 토픽을 subscribe할 때까지 시간을 0으로 초기화
- [Time API 예제](https://github.com/robotpilot/ros2-seminar-examples)

## ROS 2 프로그래밍
### Setup Script
* 새로운 패키지를 빌드하면 설정 스크립트를 실행해야 한다
- **local_setup.bash**: 위치한 경로의 모든 패키지에 대한 환경 설정
- **setup.bash**: 현재 작업 공간이 빌드될 때 환경에 필요한 다른 모든 작업 공간에 대한 local_setup.bash 스크립트 포함. underlay 개발환경의 설정 정보까지 포함
- **overlay**: 개발자가 사용하는 워크스페이스
- **underlay**: 소스코드를 내려받아 빌드해서 설치할때 생기는 워크스페이스
- underlay 개발환경의 setup.bash를 우선 소싱한 후 overlay 환경의 local_setup.bash를 소싱하는 게 정석
  ```bash
  source /opt/ros/foxy/setup.bash
  source ~/robot_ws/install/local_setup.bash
  ```

### [ROS 공식 예제](https://github.com/ros2/examples)
