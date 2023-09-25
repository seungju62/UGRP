#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include "ht8108_v2.h"

// --------- motor angle ---------
float joint1_float;
float joint2_float;
String joint_txt = "0.0000,0.0341,0.0664,0.0973,0.1270,0.1556,0.1832,0.2101,0.2361,0.2615,0.2863,0.3105,0.3343,0.3576,0.3804,0.4029,0.4250,0.4467,0.4682,0.4894,0.5103,0.5309,0.5514,0.5716,0.5916,0.6114,0.6311,0.6506,0.6699,0.6887,0.6895,0.6904,0.6913,0.6921,0.6930,0.6938,0.6947,0.6956,0.6964,0.6973,0.6981,0.6990,0.6998,0.7007,0.7016,0.7024,0.7033,0.7041,0.7050,0.7058,0.7067,0.7075,0.7084,0.7093,0.7101,0.7110,0.7118,0.7127,0.7135,0.7134,0.6949,0.6763,0.6576,0.6387,0.6196,0.6004,0.5811,0.5615,0.5418,0.5218,0.5017,0.4812,0.4606,0.4396,0.4184,0.3968,0.3749,0.3526,0.3299,0.3068,0.2831,0.259,0.2343,0.2089,0.1828,0.1559,0.1282,0.0994,0.0695,0.0380,0.0021";

// --------- motor id ---------
int id1 = 6;
int id2 = 5;

// --------- sensor ---------
int in0 = A0;
int in1 = A1;
int in2 = A2;

void setup() {

  Serial.begin(115200);                           // 시리얼 통신 설정 (보드레이트 9600)
  // onMotor(id1);
  // onMotor(id2);
  // delay(1000);


  delay(1000);
}

void loop(){

  int sensor0 = analogRead(in0);     // 센서값을 아나로그로 읽어 value 변수에 저장
  int sensor1 = analogRead(in1);
  int sensor2 = analogRead(in2);

  if (sensor1 != 0 || sensor2 != 0)
  {
    offMotor(id1);
    offMotor(id2);
  }

  while (sensor0 != 0)
  {
    Serial.println(sensor0);
    for (int i=0 ; i<92 ; i++)
      {
        String theta = joint_txt.substring(i*7, i*7+6);
        joint1_float = -theta.toFloat();
        joint2_float = theta.toFloat()*2;
        
        runMotor(id1, joint1_float, 0, 1, 0.2, 0);
        runMotor(id2, joint2_float, 0, 1, 0.2, 0);
        
        delay(100);
      }
      
      zero(id1);
      zero(id2);
  }
  

  delay(1000);                                         // 0.01초의 딜레이

}