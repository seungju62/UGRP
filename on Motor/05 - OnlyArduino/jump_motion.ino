#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include "ht8108_v2.h"

float joint1_float;
float joint2_float;

int id1 = 6;
int id2 = 5;

String joint_txt = "0.7134,0.6949,0.6763,0.6576,0.6387,0.6196,0.6004,0.5811,0.5615,0.5418,0.5218,0.5017,0.4812,0.4606,0.4396,0.4184,0.3968,0.3749,0.3526,0.3299,0.3068,0.2831,0.259,0.2343,0.2089,0.1828,0.1559,0.1282,0.0994,0.0695,0.0380,0.0021,0.7135";

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
}

void loop() {

  // runMotor(id1, -0.6, 0, 5, 2, 0);
  // runMotor(id2, 1.2, 0, 5, 2, 0);
  // delay(50);

  for (int i=0 ; i<92/5/2 ; i++)
  {
    Serial.println(i);
    String theta = joint_txt.substring(i*5*7, i*5*7+6);
    joint1_float = -theta.toFloat()*1.3;
    joint2_float = theta.toFloat()*1.3*2;
    
    runMotor(id1, joint1_float, 0, 10, 2, 0);
    runMotor(id2, joint2_float, 0, 10, 2, 0);
    
    delay(100);
  }
  delay(3000);
}
