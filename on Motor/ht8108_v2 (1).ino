#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include "ht8108_v2.h"

#define UP     A1
#define DOWN   A3
#define CLICK  A4

char incomingByte;
char my_Array[20];
int idx = 0;
float data_float;

void setup() {
  pinMode(UP,INPUT);
  pinMode(DOWN,INPUT);
  pinMode(CLICK,INPUT);
  
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(CLICK, HIGH);
  
  Serial.begin(115200);

  if(Canbus.init(CANSPEED_1000))  //CAN Speed 1000kbps
    Serial.println("CAN Init ok");
  else
    Serial.println("Can't init CAN");
  
  delay(500);
  zero(1);
  delay(500);
}

void loop() {
  //------------ setting(on/off) -------------
  if (digitalRead(UP) == 0) {
      Serial.println("Enable motor");
      onMotor(1);
      while(digitalRead(UP) == 0);
  }
  
   if (digitalRead(DOWN) == 0) {
      Serial.println("Disable motor");
      offMotor(1);
      while(digitalRead(DOWN) == 0);
   }
  
//   if (digitalRead(CLICK) == 0) {
//      Serial.println("zero");
//      offMotor(1);
//      zero(1);
//      while(digitalRead(CLICK) == 0);
//   }

  //------------ data read -------------
  if (Serial.available() > 0) 
    {
      incomingByte = Serial.read();
      if (incomingByte != '\n') {
        my_Array[idx] = incomingByte;
        idx++;
      } else { 
        my_Array[idx] = '\0';
        data_float = atof(my_Array);
        Serial.println(data_float);
        idx = 0;
        runMotor(1, data_float, 0, 5, 0.2, 0);
      }
    }
}
