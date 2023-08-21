# UGRP
Undergraduate Group Research Program

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
  - 03 - sending theta1,2 : uno2matlab/02 - sending separately 이용해서 theta1, theta2 각각의 모터에게 전송
    (값은 옳게 출력되는 것 같은데, 모터가 값대로 움직이지 않음. 갑자기 급발진함 / theta1은 각도 범위가 너무 작은 것 같기도 함.) 

## arduino_r4 (사용 X)
  - arduino_r4_can.ino : can 통신 등 시도..
  - motor_r4.ino : arduino uno r4 wifi -> CAN 통신 시도 중 ... 해 오기!!! (https://docs.arduino.cc/tutorials/uno-r4-wifi/can)
