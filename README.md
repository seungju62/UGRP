# UGRP
Undergraduate Group Research Program

## Diagram

## 1. control
  ### 1.1 CAN (모터를 구동하기 위한 아두이노-모터 사이의 CAN 통신 설정)
   - motor_id.h : 모터의 아이디 설정을 위한 코드이지만, 현재 사용하는 중인 모터의 경우 setting을 위한 프로그램이 따로 있기 때문에 사용하지 않음. (Link 추가 : https://youtu.be/2jzKCeKifuY)
   - motor_set.h : 글쎄 무엇을 설정하는 거였을까?
   - one_way.ino : 아두이노에서 모터로 명령어 보내기 (CAN shield의 조이스틱을 이용하여 모터 돌리기)
   - two_way.ino : 아두이노에서 모터로 명령어 보내고, 모터에서 값을 받아서 아두이노에서 출력
     
  ### 1.2 uno2matlab
  - 01 - sending data : 매트랩에서 아두이노로 숫자 데이터 보내기 (아두이노에서 serial port를 열 수 없기 때문에 매트랩에서 값을 다시 받아 출력)
  - 02 - sending separately : motor1, motor2에 각각 다른 값을 보내기 위한 코드 (매트랩에서 하나의 데이터로 모아서 보낸 후, 아두이노에서 각각 나누어 저장)

## 2. dynamics
  - SLIP_sim(1).m : SLIP model 뛰는 모습 그려줌
  - SLIP_sim(2).m : SLIP model에 따른 two-link 다리 움직임 그려줌 (각도 출력) -> 각도 제대로 출력되는 코드 맞음?(joint2 = theta2-theta1...)
  - SLIP_taumax.m : 스프링 상수 이용해서 필요한 tau 구함

## 3. on Motor
  - library : control/CAN/two_way.ino 파일을 수정하여  move(motor_id, pos, vel, Kp, Kd, Cur)만 입력하면 동작하도록 라이브러리로 만듦
  - 01 - sending num : [1,2,3,4] 이런 간단한 숫자 보내면서 모터 잘 돌아가는지 확인
  - 02 - sending theta1 : SLIP_sim 파일에서 필요한 각도 받아와 motor 하나에다가 보내기
  - 03 - sending theta1,2 : uno2matlab/02 - sending separately 이용해서 theta1, theta2 각각의 모터에게 전송 (다리 길이 0.2, 0.2m로 수정, 최대한 굽혔다가 펼 수 있도록 K 조정)
  - 04 - sending jump : 점프하는 동작에 대한 움직임만 명령을 줌 / 점프하는 시간.. 동안에 대해서는 아래 Sensor/TouchSensor 로 잇기
                      : 그러면... 아직 센서 붙여보기 전에는 -> 값 주고 / delay를 주고? (matlab에서 pause로 조절)
  - 05 - OnlyArduino : 매트랩 사용하지 않고 아두이노로만 모터 구동하기 (/jump_motion.ino : 0927 리니어 가이드에 연결하고 테스트할 때 사용)
    
## 4. test Motor
  - 01 - sending num : 그냥 모터 켜지는지 보기 위해 아두이노만을 가지고 모터 돌리기 (position / velocity 입력)
  - 02 - sending num(1) : 값 하나 보냄 (library 사용 X 그냥 다 적혀있음)
  - 03 - sending num(2) : 값 두 개 보냄 (library 사용 X 그냥 다 적혀있음)

## 5. Sensor
  ### TouchSensor
  - 01 - touch_sensor : 접촉감지센서(RA12P)에서 값 받기 (https://www.devicemart.co.kr/goods/view?no=1327467)
  - 02 - touch_sensor(1) : 1) 바닥에 접촉 시(A0) 점프를 위한 명령어 주기
                             2) over spread(A1) / over bend(A2) 시 모터 stop or off
  ### GyroSensor

## 6. TEST (The Newest)
  - 01 - one_leg.ino : 약간 수정을 했기 때문에 첨부. 가장 최신 파일
  - 02 - four_legs.ino : 다리 4개를 모두 동시에 구동하기 위한 코드 작성. **아직 테스트 해보지 못함.**
  - JumpingModel.m : 로봇의 다리 길이 수정 (thigh 0.2m, calf 0.25m)
    
## arduino_r4 (사용 X)
  - arduino_r4_can.ino : can 통신 등 시도..
  - motor_r4.ino : arduino uno r4 wifi -> CAN 통신 시도 중 ... 해 오기!!! (https://docs.arduino.cc/tutorials/uno-r4-wifi/can)
