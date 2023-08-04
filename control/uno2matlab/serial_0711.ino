#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include "ht8108_v2.h"

char incomingByte;
char my_Array[20];
char joint1[20];
char joint2[20];

int idx = 0;
float joint1_float;
float joint2_float;

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
  // onMotor(2);
  // delay(1000);
  // zero(2);
  while (Serial.available() > 0) 
    {
      incomingByte = Serial.read();
      if (incomingByte != '\n') 
      {
        my_Array[idx] = incomingByte;
        idx++;
      } else 
      { 
        my_Array[idx] = '\0';
        idx = 0;
        String my_String = String(my_Array);
        String joint1 = my_String.substring(0,6);
        // Serial.println(joint1);
        String joint2 = my_String.substring(6,12);
        // Serial.println(joint2);

        joint1_float = joint1.toFloat(); 
        joint2_float = joint2.toFloat();

        // joint1_float = atof(joint1); 
        // joint2_float = atof(joint2);

        Serial.println(joint1_float);
        Serial.println(joint2_float);
        
        // delay(1000);
        // runMotor(2, data_float, 0, 1, 0.2, 0);
        // delay(1000);  
        // offMotor(2);  
        // delay(1000);
      }
    }
}
