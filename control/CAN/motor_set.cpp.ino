#include "Arduino.h"
#include "motor_set"
#include "motor_id" // have to complete to make

motor_set::motor_set(int num)
{
  _num = num;
}

void motor_set::On()
{
  if (Serial.write(_num) == 8){
    Serial.println("Enable motor");
    message1.data[0] = 0xFF;
    message1.data[1] = 0xFF;
    message1.data[2] = 0xFF;
    message1.data[3] = 0xFF; //formatted in HEX
    message1.data[4] = 0xFF;
    message1.data[5] = 0xFF;
    message1.data[6] = 0xFF;
    message1.data[7] = 0xFC;

  else if (Serial.write(_num) == 2){
    Serial.println("Disable motor");
    message1.data[0] = 0xFF;
    message1.data[1] = 0xFF;
    message1.data[2] = 0xFF;
    message1.data[3] = 0xFF; //formatted in HEX
    message1.data[4] = 0xFF;
    message1.data[5] = 0xFF;
    message1.data[6] = 0xFF;
    message1.data[7] = 0xFD;

  else if (Serial.write(_num) == 5){
    Serial.println("zero");
    message1.data[0] = 0xFF;
    message1.data[1] = 0xFF;
    message1.data[2] = 0xFF;
    message1.data[3] = 0xFF; //formatted in HEX
    message1.data[4] = 0xFF;
    message1.data[5] = 0xFF;
    message1.data[6] = 0xFF;
    message1.data[7] = 0xFE;
  }
