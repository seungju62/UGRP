#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include "ht8108_v2.h"

float joint1_float;
float joint2_float;

// 올라갈 때 motor2 id1 골반쪽 (+)
// 올라갈 때 motor1 id2 무릎쪽 (-)
// 처음에 offset (-)

int id1 = 2;
int id2 = 1;
int offset = -0.25;

String joint_txt1 = "0.9081,0.9079,0.8659,0.8265,0.7893,0.7539,0.7201,0.6876,0.6563,0.6259,0.5964,0.5675,0.5393,0.5115,0.4843,0.4573,0.4307,0.4043,0.3781,0.3521,0.3261,0.3002,0.2742,0.2482,0.2222,0.1960,0.1696,0.1430,0.1161,0.0888,0.0612,0.0330,0.0018,0.0000";
String joint_txt2 = "1.2071,1.2069,1.1716,1.1364,1.1013,1.0663,1.0312,0.9960,0.9608,0.9255,0.8900,0.8543,0.8185,0.7824,0.7460,0.7094,0.6724,0.6351,0.5974,0.5593,0.5208,0.4818,0.4423,0.4023,0.3616,0.3203,0.2783,0.2356,0.1920,0.1475,0.1020,0.0551,0.0030,0.0000";

void setup() {

  Serial.begin(115200);

  if(Canbus.init(CANSPEED_1000))  //CAN Speed 1000kbps
    Serial.println("CAN Init ok");
  else
    Serial.println("Can't init CAN");
  delay(1000);

  offMotor(id1);
  offMotor(id2);
  delay(1000);

  onMotor(id1);
  onMotor(id2);
  delay(1000);

  zero(id1);
  zero(id2);
  delay(1000);

  runMotor(id1, offset, 0, 40, 2, 0);
  runMotor(id2, 0, 0, 40, 2, 0);
  delay(1000);
}

void loop() {

for (int step=0 ; step<3 ; step++)
{
  Serial.println('-----------------------------------');
  for (int i=0 ; i<92/3/2 ; i++)
  {
    Serial.println(i);
    String theta1 = joint_txt1.substring(i*3*7, i*3*7+6);
    String theta2 = joint_txt2.substring(i*3*7, i*3*7+6);

    // joint1_float = -theta.toFloat();
    joint1_float = -theta1.toFloat()*1.3 +offset;
    joint2_float = theta2.toFloat()*2*1.3;
    
    // 왼쪽다리 (-)
    // 오른쪽다리 (+)
    runMotor(id1, -joint1_float, 0, 40, 2, 0);
    runMotor(id2, -joint2_float, 0, 40, 2, 0);
    
    delay(100);
  }

  delay(2000);
  // for (int j=0 ; j<20 ; j++)
  // {
  //   runMotor(id1, -joint1_float, 0, 40, 2, 0);
  //   runMotor(id2, -joint2_float, 0, 40, 2, 0);
  //   delay(100);
  // }
}
  offMotor(id1);
  offMotor(id2);
}
