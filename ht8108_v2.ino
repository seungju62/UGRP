#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include "ht8108_v2.h"




void setup() {
 
 //Initialize serial terminal connection
  Serial.begin(115200);

  delay(3000);

  if(Canbus.init(CANSPEED_1000))  //CAN Speed 1000kbps
    Serial.println("CAN Init ok");
  else
    Serial.println("Can't init CAN");
  delay(1000);
}

void loop() {
  
  onMotor(1);
  delay(1000);
  zero(1);
  delay(1000);
  for (int i=1;i<6;i++){
    runMotor(1, 2*i, 0, 10, 0.2, 0);
    delay(1000);
    runMotor(1, 0, 0, 10, 0.2, 0);
    delay(1000);
    Serial.println(' ');
  }
  delay(1000);  
  offMotor(1);  
  delay(1000);
  while(1);
}
