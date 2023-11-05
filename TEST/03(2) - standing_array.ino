#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include "ht8108_v2.h"

float joint1_float;
float joint2_float;

int id1 = 1;
int id2 = 2;
int id3 = 4;
int id4 = 3;

int id5 = 5;
int id6 = 6;
int id7 = 7;
int id8 = 8;

float offset = 0.35;

float joint_txt1[59] = {0.0000,0.0494,0.0974,0.1443,0.1902,0.2351,0.2791,0.3223,0.3647,0.4065,0.4476,0.4882,0.5282,0.5678,0.6069,0.6455,0.6838,0.7218,0.7594,0.7967,0.8337,0.8706,0.9072,0.9436,0.9799,1.0161,1.0522,1.0883,1.1244,1.1598,1.1614,1.1630,1.1646,1.1663,1.1679,1.1695,1.1712,1.1728,1.1744,1.1761,1.1777,1.1793,1.1809,1.1826,1.1842,1.1858,1.1875,1.1891,1.1907,1.1924,1.1940,1.1956,1.1973,1.1989,1.2005,1.2022,1.2038,1.2055,1.2071};
float joint_txt2[59] = {0.0000,0.0295,0.0584,0.0869,0.1149,0.1426,0.1700,0.1972,0.2241,0.2510,0.2777,0.3044,0.3311,0.3578,0.3847,0.4117,0.4389,0.4664,0.4942,0.5225,0.5512,0.5805,0.6106,0.6414,0.6732,0.7060,0.7402,0.7760,0.8136,0.8524,0.8542,0.8561,0.8579,0.8598,0.8616,0.8635,0.8654,0.8672,0.8691,0.8710,0.8729,0.8748,0.8767,0.8786,0.8806,0.8825,0.8844,0.8864,0.8883,0.8903,0.8922,0.8942,0.8961,0.8981,0.9001,0.9021,0.9041,0.9061,0.9081};

int kp = 80;
int kd = 2;

void setup() {

  Serial.begin(115200);

  if(Canbus.init(CANSPEED_1000))  //CAN Speed 1000kbps
    Serial.println("CAN Init ok");
  else
    Serial.println("Can't init CAN");
  delay(1000);

  // -------------------
  offMotor(id1);
  offMotor(id2);
  offMotor(id3);
  offMotor(id4);

  offMotor(id5);
  offMotor(id6);
  offMotor(id7);
  offMotor(id8);
  delay(1000);

  // -------------------
  onMotor(id1);
  onMotor(id2);
  onMotor(id3);
  onMotor(id4);

  onMotor(id5);
  onMotor(id6);
  onMotor(id7);
  onMotor(id8);
  delay(3000);

  zero(id1);
  zero(id2);
  zero(id3);
  zero(id4);
  zero(id5);
  zero(id6);
  zero(id7);
  zero(id8);
  delay(3000);

  runMotor(id1, 0, 0, kp, kd, 0);
  runMotor(id2, -offset, 0, kp, kd, 0);
  runMotor(id3, 0, 0, kp, kd, 0);
  runMotor(id4, -offset, 0, kp, kd, 0);
  runMotor(id5, 0, 0, kp, kd, 0);
  runMotor(id6, offset, 0, kp, kd, 0);
  runMotor(id7, 0, 0, kp, kd, 0);
  runMotor(id8, offset, 0, kp, kd, 0);
  delay(5000);
}

void loop() {

for (int step=0 ; step<2 ; step++)
{
  float scale = 1.2;
  int ts = 1;
  for (int i=0 ; i<sizeof(joint_txt1)/4-1 ; i++)
  {
    joint1_float = joint_txt1[i]*scale;
    joint2_float = -joint_txt2[i]*scale+offset;
    
    Serial.println(joint2_float);

    runMotor(id1, -joint1_float, 0, kp, kd, 0);
    runMotor(id2, -joint2_float, 0, kp, kd, 0);
    runMotor(id3, -joint1_float, 0, kp, kd, 0);
    runMotor(id4, -joint2_float, 0, kp, kd, 0);

    runMotor(id5, joint1_float, 0, kp, kd, 0);
    runMotor(id6, joint2_float, 0, kp, kd, 0);
    runMotor(id7, joint1_float, 0, kp, kd, 0);
    runMotor(id8, joint2_float, 0, kp, kd, 0);
  }
  
  for (int i=sizeof(joint_txt1)/4-1 ; i>0 ; i--)
  {
    Serial.println(i);
    joint1_float = joint_txt1[i]*scale;
    joint2_float = -joint_txt2[i]*scale+offset;

    Serial.println(joint2_float);

    runMotor(id1, -joint1_float, 0, kp, kd, 0);
    runMotor(id2, -joint2_float, 0, kp, kd, 0);
    runMotor(id3, -joint1_float, 0, kp, kd, 0);
    runMotor(id4, -joint2_float, 0, kp, kd, 0);

    runMotor(id5, joint1_float, 0, kp, kd, 0);
    runMotor(id6, joint2_float, 0, kp, kd, 0);
    runMotor(id7, joint1_float, 0, kp, kd, 0);
    runMotor(id8, joint2_float, 0, kp, kd, 0);
  }
  
  delay(2000);
}

  offMotor(id1);
  offMotor(id2);
  offMotor(id3);
  offMotor(id4);

  offMotor(id5);
  offMotor(id6);
  offMotor(id7);
  offMotor(id8);
}
