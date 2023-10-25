// Instruction for 4-legged jumping robot
// Date : 2023. 10. 25

//   |----------|
// 12|    앞    |65
//   |          |
//   |왼      오|
//   |          |
// 34|    뒤    |87
//   |----------|

// direction of the motor
// cw (+) ccw (-)

// offset
// 2,4 (-) 6,8 (+)

// jumping
// inner(골반) : 2,4 (+) 6,8 (-)
// outer(무릎) : 1,3 (-) 5,7 (+)

#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include "ht8108_v2.h"

String joint_txt1 = "0.9081,0.9079,0.8659,0.8265,0.7893,0.7539,0.7201,0.6876,0.6563,0.6259,0.5964,0.5675,0.5393,0.5115,0.4843,0.4573,0.4307,0.4043,0.3781,0.3521,0.3261,0.3002,0.2742,0.2482,0.2222,0.1960,0.1696,0.1430,0.1161,0.0888,0.0612,0.0330,0.0018,0.0000";
String joint_txt2 = "1.2071,1.2069,1.1716,1.1364,1.1013,1.0663,1.0312,0.9960,0.9608,0.9255,0.8900,0.8543,0.8185,0.7824,0.7460,0.7094,0.6724,0.6351,0.5974,0.5593,0.5208,0.4818,0.4423,0.4023,0.3616,0.3203,0.2783,0.2356,0.1920,0.1475,0.1020,0.0551,0.0030,0.0000";

int id1 = 1; int id2 = 2;
int id3 = 3; int id4 = 4;
int id5 = 5; int id6 = 6;
int id7 = 7; int id8 = 8;

int offset = 0.25;

float joint1_float;
float joint2_float;

void setup() {

  Serial.begin(115200);

  if(Canbus.init(CANSPEED_1000))
    Serial.println("CAN Init ok");
  else
    Serial.println("Can't init CAN");
  delay(1000);

//  Serial.println('-------offMotor-------');
  offMotor(id1); offMotor(id2);
  offMotor(id3); offMotor(id4);
  offMotor(id5); offMotor(id6);
  offMotor(id7); offMotor(id8);
  delay(1000);

  Serial.println('-------onMotor-------');
  onMotor(id1); onMotor(id2);
  onMotor(id3); onMotor(id4);
  onMotor(id5); onMotor(id6);
  onMotor(id7); onMotor(id8);
  delay(2000);

  Serial.println('-------Zero-------');
  zero(id1); zero(id2);
  zero(id3); zero(id4);
  zero(id5); zero(id6);
  zero(id7); zero(id8);
  delay(2000);

  Serial.println('------offset------');
  for (int o=0 ; o<5 ; o++)
  {
  runMotor(id2, -offset/5*o, 0, 40, 2, 0); runMotor(id4, -offset/5*o, 0, 40, 2, 0);
  runMotor(id6, offset/5*o, 0, 40, 2, 0); runMotor(id8, offset/5*o, 0, 40, 2, 0);
  runMotor(id1, 0, 0, 40, 2, 0); runMotor(id3, 0, 0, 40, 2, 0);
  runMotor(id5, 0, 0, 40, 2, 0); runMotor(id7, 0, 0, 40, 2, 0);
  }
  delay(1000);
}

void loop() {
  Serial.println('----------LET'S JUMP !!-------------');
  int t = 5;      // time stamp
  int scale = 1;  // multiple the scale
                  // 7 : number of strings

  for (int step=0 ; step<3 ; step++)
  {
    for (int i=0 ; i<92/2/t ; i++)
    {
      String theta1 = joint_txt1.substring(i*t*7, i*t*7+6);
      String theta2 = joint_txt2.substring(i*t*7, i*t*7+6);

      joint1 = theta1.toFloat()*scale +offset;
      joint2 = theta2.toFloat()*scale;
      
      runMotor(id1, -joint1, 0, 40, 2, 0);  runMotor(id3, -joint1, 0, 40, 2, 0);
      runMotor(id5, joint1, 0, 40, 2, 0);   runMotor(id7, joint1, 0, 40, 2, 0);
      runMotor(id2, joint2, 0, 40, 2, 0);   runMotor(id4, joint2, 0, 40, 2, 0);
      runMotor(id6, -joint2, 0, 40, 2, 0);  runMotor(id8, -joint2, 0, 40, 2, 0);      
      delay(50);
    }
  delay(2000);
  
  Serial.println('-------offMotor-------');
  offMotor(id1); offMotor(id2);
  offMotor(id3); offMotor(id4);
  offMotor(id5); offMotor(id6);
  offMotor(id7); offMotor(id8);
}
