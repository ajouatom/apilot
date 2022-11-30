APILOT
------
* 읽어보기

  * 설정값을 함부로 건들면 사고의 위험이 있으며, 사고/고장 발생시 본인의 책임입니다.
  * Panda코드를 수정하였으므로, comma connect에 연결되면 BAN당함(연결되어도 업로드안됨)!
  * 배선개조: SCC모듈(레이더)의 CCAN연결을 절단 -> 판다의 CAN2에 연결
  * NDA지원 (Neokii)
  * 신호정지모드
    * 신호정지감속정지상태에서 GAS페달: 신호무시
    * 브레이크홀드기능: 정지상태에서 Brake페달 1초이상: 신호무시하고 정지유지 
  * 테스트된차량: SCC모듈(레이더) 배선개조 CAN BUS2로 연결된차량
    * Hyundai SANTAFE HYBRID Hybrid 2022 (배선개조 안된것도 가능,단,AEB off됨)
    * Kia STINGER
    * Hyundai GENESIS (White PANDA를 이용하여 MDPS개조된차량)
    * KONA EV
  * 인게이지
    * 롱컨모드
      * 배선개조된경우: 크루즈셋버튼, (+)(-) 버튼
      * 순정배선(SCC모듈기능정지,AEB OFF): (+)(-) 버튼
    * SCC모드
      * 크루즈셋버튼: 차량의 크루즈셋 조건에 따름.
  * 크루즈 차량간격: 속도에 따른 자동차간거리
    * 1단계: 연비절감모드,
    * 2단계: 연비절감모드: GAS페달에서 떼면 관성주행, 단,전방레이더 또는 신호 감지시 크루즈ON
    * 3단계: 일반주행,
    * 4단계: 고속모드

* 설정: 토글
  * Experimental openpilot Longitudinal Control(롱컨트롤)
    * 순정 SCC의 기능을 사용하지 않고 오픈파일럿의 크루즈제어 기능 선택
    * 배선 개조시 선택함.
    * 배선 비개조시, Enable RadarTracks(ON)한경우: 테스트차량만 가능
      * 지원: SANTAFE HEV, NEXO
      * 미지원: GENESIS(DH), KONA
  * SCC Module connected BUS2
    * 배선을 개조하여 판다의 BUS2에 연결한 경우 ON
  * EnableRadarTracks
    * SCC모듈에서 제공하는 레이더정보를 무시하고, SCC모듈내의 레이더 정보를 이용함.

* 설정: 기타
  * 차량선택: 현대/기아 차량만 선택가능
  * 업데이트: 현재선택되어 있는 브랜치에 대해 새로운 업데이트가 있으면 적용 후 Rebooting
    * (주의) 반드시 시동을 끄고 할것!
  * NDA지원
    * Neokii의 NDA지원 단말기 설치시 자동으로 연결: 같은 네트워크에 있어야함.
    * 과속카메라 속도제한은 무조건 작동함.
  * NDA:속도제한
    * 0: 도로 제한 속도를 반영안함.
    * 1: 도로 제한 속도를 넘지 않도록 제한.
    * 2: 도로 제한 속도를 반영함.
  * 크루즈소리 
    * 0: 크루즈 작동소리가 안남
    * 1: 일부소리가 남.
    * 2: 모든 크루즈 작동소리가 안남.
  * CustomMapBox입력
    * http://IP주소:8082 접속하여 mapboxtoken을 입력하면 자동으로 켜짐.
    * OFF하면 입력된 Token값을 제거함.
  * Show Debug UI
    * 화면에 디버그 정보를 표시함.

* 설정: 크루즈
  * 가속시 크루즈속도를 맞춤
    * 가속페달을 밟아 설정된 속도보다 높아지면 자동으로 설정속도를 올려줌
  * 자동속도증가모드(100%)
    * 선행차량의 속도가 빨라지면, RoadSpeedLimit * 비율의 속도까지 자동으로 설정속도를 올려줌.
    * 0: 사용안함
  * 선행차속도에 크루즈속도맞추기(+40km/h)
    * 선행차를 만나면 선행차의 속도에 설정속도로 미리 낮추어줌.
  * 운전모드 초기값(3)
    * 1: 연비모드 : 가속을 제한하여 연비위주의 운전모드
    * 2: 관성모드 : 가속페달에서 발을 떼면, 크루즈가 자동으로 꺼짐. 단, 전방에 차량또는 신호를 만나면 자동으로 켜짐.
    * 3: 일반주행모드 : 일반적인 주행모드
    * 4: 고속모드 : 신호정지 기능을 사용안함.
    * 화면의 거리조정(GAP) 표시를 누르면 바뀜.
  * 크루즈버튼작동모드
    * 0: 일반속도제어
      * 1씩증감, 길게누르면 10씩증감
      * (-)버튼: 현재속도로 설정
    * 1: 사용자속도제어1
      * (+)버튼: 도로속도제한까지 한번에 올라감, 이후 +10km/h씩올라감. 
      * (-)버튼: 현재속도로 설정
    * 2: 사용자속도제어2
      * 사용자속도제어1의 기능외에
      * (-)버튼: 또 누르면, 관성제어모드(크루즈꺼짐)
  * 엑셀크루즈ON: 60%이상 엑셀을 밟으면 크루즈가 켜짐.
  * 엑셀크루즈ON: 속도(30): 지정된 속도가 올라가면 크루즈가 켜짐
  * 엑셀크루즈ON: 속도설정방법
    * 0: 현재속도로 세팅
    * 1: 기존 속도로 세팅
    * 2: 선행차 있을때만 기존속도로 세팅
  * 브레이크해제 크루즈ON  사용: 브레이크를 떼면 크루즈를 켜는 기능을 사용
  * 브레이크해제 크루즈ON:주행중,선행차(20): 선행차거리: 브레이크를 떼고 선행차가 일정거리 이상이면 크루즈 ON
  * 브레이크해제 크루즈ON:정지상태, 선행차: 정지상태에서 선행차가 10M이내에 있으면 크루즈 ON
  * 브레이크해제 크루즈ON:주행중, 속도: 0:사용안함, 브레이크를 떼고 일정속도이상이면 크루즈를 현재속도로 ON
  * 브레이크해제 크루즈ON:주행중,신호: 60km/h이하에서 브레이크를 떼고 신호가 감지되면 현재속도로 크루즈 ON

* 설정: 튜닝
  * StopDistance(600cm): 선행차에 대한 정지위치를 조정
  * 신호정지 위치 조정(200cm): 신호등에 대한 정지위치 조정
  * 신호정지 모델속도(OFF): 신호등에 대한 정지시 모델이 제공하는 속도에 따름
    * 사용안함.
  * 롱컨: JERK값(8): 값을 크게하면 가속/감속에 대한 차량의 반응이 강해지나, 브레이크작동이 잘 안되면 값을 조금씩 올림.
  * 롱컨: FF게인(106%): 속도플래너의 가속도에 대한 피드포워드, 이값은 조정하시 말것!
  * 롱컨: P게인(100): 차량 속도제어에 대한 P게인, 50~100정도로 조정.
  * X_EGO_COST(6): 높으면 신호정지위치에 정밀하게 서려고함.
  * J_EGO_COST(5): 고정
  * A_CHANGE_COST(150): 끼워들기 차량에 대해 반응이 너무 강하면 올릴것. (50~200)
  * DANGER_ZONE_COST(100): 고정
  * 차량간격유지 동적제어(OFF): 전방차량이 멀어지는 것에 대한 반응을 빠르게 COST를 동적으로 적용함.
  * 차량간격 동적제어:상대속도(110%): 선행차와의 상대속도에 따라 차량간격을 동적으로 조정함.
  * 차량간격 동적제어:감속(110%): 차량의 감속도에 따라 차량간격을 동적으로 조정함. 급감속을 하면 더 멀리.
  * 차량간격 비율(100%)
    * 위험: 추돌의 위험이 있음.
    * 속도에 따라 차량간격을 자동조정하고 있으나, 좀더 가깝게 하고 싶을때
  * 신호감지 감속율(80%): 신호등감지시 감속도를 조절하여 서서히 속도를 줄이고 싶을때
  * 모델의 자동속도조절의 적용속도(0): 사용안함.
  * 초기가속제한속도(3km/h): 설정속도까 연비운전모드의 가속률을 사용함.
  * 모델혼잡시 조향가속비율적용(ON): 일반적으로 도로상황이 안좋을때, 연비운전모드의 가속률을 사용함.
  * 가속도 제어(100): 가속도를 좀더 올리고 싶을때 사용.
  * 모델커브속도조절(ON): 곡선도로를 만나면 자동으로 속도를 줄여줌.

  

![](https://i.imgur.com/b0ZyIx5.jpg)

Table of Contents
=======================

* [What is openpilot?](#what-is-openpilot)
* [Running in a car](#running-on-a-dedicated-device-in-a-car)
* [Running on PC](#running-on-pc)
* [Community and Contributing](#community-and-contributing)
* [User Data and comma Account](#user-data-and-comma-account)
* [Safety and Testing](#safety-and-testing)
* [Directory Structure](#directory-structure)
* [Licensing](#licensing)

---

What is openpilot?
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, openpilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW), and Lane Departure Warning (LDW) for a growing variety of [supported car makes, models, and model years](docs/CARS.md). In addition, while openpilot is engaged, a camera-based Driver Monitoring (DM) feature alerts distracted and asleep drivers. See more about [the vehicle integration](docs/INTEGRATION.md) and [limitations](docs/LIMITATIONS.md).

<table>
  <tr>
    <td><a href="https://youtu.be/NmBfgOanCyk" title="Video By Greer Viau"><img src="https://i.imgur.com/1w8c6d2.jpg"></a></td>
    <td><a href="https://youtu.be/VHKyqZ7t8Gw" title="Video By Logan LeGrand"><img src="https://i.imgur.com/LnBucik.jpg"></a></td>
    <td><a href="https://youtu.be/VxiR4iyBruo" title="Video By Charlie Kim"><img src="https://i.imgur.com/4Qoy48c.jpg"></a></td>
    <td><a href="https://youtu.be/-IkImTe1NYE" title="Video By Aragon"><img src="https://i.imgur.com/04VNzPf.jpg"></a></td>
  </tr>
  <tr>
    <td><a href="https://youtu.be/iIUICQkdwFQ" title="Video By Logan LeGrand"><img src="https://i.imgur.com/b1LHQTy.jpg"></a></td>
    <td><a href="https://youtu.be/XOsa0FsVIsg" title="Video By PinoyDrives"><img src="https://i.imgur.com/6FG0Bd8.jpg"></a></td>
    <td><a href="https://youtu.be/bCwcJ98R_Xw" title="Video By JS"><img src="https://i.imgur.com/zO18CbW.jpg"></a></td>
    <td><a href="https://youtu.be/BQ0tF3MTyyc" title="Video By Tsai-Fi"><img src="https://i.imgur.com/eZzelq3.jpg"></a></td>
  </tr>
</table>


Running on a dedicated device in a car
------

To use openpilot in a car, you need four things
* A supported device to run this software: a [comma three](https://comma.ai/shop/products/three).
* This software. The setup procedure of the comma three allows the user to enter a URL for custom software.
The URL, openpilot.comma.ai will install the release version of openpilot. To install openpilot master, you can use installer.comma.ai/commaai/master, and replacing commaai with another GitHub username can install a fork.
* One of [the 150+ supported cars](docs/CARS.md). We support Honda, Toyota, Hyundai, Nissan, Kia, Chrysler, Lexus, Acura, Audi, VW, and more. If your car is not supported but has adaptive cruise control and lane-keeping assist, it's likely able to run openpilot.
* A [car harness](https://comma.ai/shop/products/car-harness) to connect to your car.

We have detailed instructions for [how to mount the device in a car](https://comma.ai/setup).

Running on PC
------

All of openpilot's services can run as normal on a PC, even without special hardware or a car. To develop or experiment with openpilot you can run openpilot on recorded or simulated data.

With openpilot's tools, you can plot logs, replay drives, and watch the full-res camera streams. See [the tools README](tools/README.md) for more information.

You can also run openpilot in simulation [with the CARLA simulator](tools/sim/README.md). This allows openpilot to drive around a virtual car on your Ubuntu machine. The whole setup should only take a few minutes but does require a decent GPU.

A PC running openpilot can also control your vehicle if it is connected to a [webcam](https://github.com/commaai/openpilot/tree/master/tools/webcam), a [black panda](https://comma.ai/shop/products/panda), and a [harness](https://comma.ai/shop/products/car-harness).

Community and Contributing
------

openpilot is developed by [comma](https://comma.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/commaai/openpilot). Bug fixes and new car ports are encouraged. Check out [the contributing docs](docs/CONTRIBUTING.md).

Documentation related to openpilot development can be found on [docs.comma.ai](https://docs.comma.ai). Information about running openpilot (e.g. FAQ, fingerprinting, troubleshooting, custom forks, community hardware) should go on the [wiki](https://github.com/commaai/openpilot/wiki).

You can add support for your car by following guides we have written for [Brand](https://blog.comma.ai/how-to-write-a-car-port-for-openpilot/) and [Model](https://blog.comma.ai/openpilot-port-guide-for-toyota-models/) ports. Generally, a car with adaptive cruise control and lane keep assist is a good candidate. [Join our Discord](https://discord.comma.ai) to discuss car ports: most car makes have a dedicated channel.

Want to get paid to work on openpilot? [comma is hiring](https://comma.ai/jobs/).

And [follow us on Twitter](https://twitter.com/comma_ai).

User Data and comma Account
------

By default, openpilot uploads the driving data to our servers. You can also access your data through [comma connect](https://connect.comma.ai/). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software: the user is free to disable data collection if they wish to do so.

openpilot logs the road-facing cameras, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera is only logged if you explicitly opt-in in settings. The microphone is not recorded.

By using openpilot, you agree to [our Privacy Policy](https://comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

Safety and Testing
----

* openpilot observes ISO26262 guidelines, see [SAFETY.md](docs/SAFETY.md) for more details.
* openpilot has software-in-the-loop [tests](.github/workflows/selfdrive_tests.yaml) that run on every commit.
* The code enforcing the safety model lives in panda and is written in C, see [code rigor](https://github.com/commaai/panda#code-rigor) for more details.
* panda has software-in-the-loop [safety tests](https://github.com/commaai/panda/tree/master/tests/safety).
* Internally, we have a hardware-in-the-loop Jenkins test suite that builds and unit tests the various processes.
* panda has additional hardware-in-the-loop [tests](https://github.com/commaai/panda/blob/master/Jenkinsfile).
* We run the latest openpilot in a testing closet containing 10 comma devices continuously replaying routes.

Directory Structure
------
    .
    ├── cereal              # The messaging spec and libs used for all logs
    ├── common              # Library like functionality we've developed here
    ├── docs                # Documentation
    ├── opendbc             # Files showing how to interpret data from cars
    ├── panda               # Code used to communicate on CAN
    ├── third_party         # External libraries
    ├── pyextra             # Extra python packages
    └── system              # Generic services
        ├── camerad         # Driver to capture images from the camera sensors
        ├── clocksd         # Broadcasts current time
        ├── hardware        # Hardware abstraction classes
        ├── logcatd         # systemd journal as a service
        └── proclogd        # Logs information from /proc
    └── selfdrive           # Code needed to drive the car
        ├── assets          # Fonts, images, and sounds for UI
        ├── athena          # Allows communication with the app
        ├── boardd          # Daemon to talk to the board
        ├── car             # Car specific code to read states and control actuators
        ├── controls        # Planning and controls
        ├── debug           # Tools to help you debug and do car ports
        ├── locationd       # Precise localization and vehicle parameter estimation
        ├── loggerd         # Logger and uploader of car data
        ├── manager         # Deamon that starts/stops all other daemons as needed
        ├── modeld          # Driving and monitoring model runners
        ├── monitoring      # Daemon to determine driver attention
        ├── navd            # Turn-by-turn navigation
        ├── sensord         # IMU interface code
        ├── test            # Unit tests, system tests, and a car simulator
        └── ui              # The UI

Licensing
------

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>

[![openpilot tests](https://github.com/commaai/openpilot/workflows/openpilot%20tests/badge.svg?event=push)](https://github.com/commaai/openpilot/actions)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/alerts/)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:python)
[![Language grade: C/C++](https://img.shields.io/lgtm/grade/cpp/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:cpp)
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)
