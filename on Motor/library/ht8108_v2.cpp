#include "ht8108_v2.h"
#include "Arduino.h"

tCAN message;
tCAN rxmsg;

unsigned int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12)
  {
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  }
  if (bits == 16)
  {
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
}


float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12){
    pgg = ((float)x_int)*span/4095.0 + offset;
  }
  if (bits == 16){
    pgg = ((float)x_int)*span/65535.0 + offset;
  }
  return pgg;
}

 
void onMotor(int motorID){
  byte buf[8];
  //Scan analog pins. If pin reads low, print the corresponding joystick movement.

  Serial.println("Enable motor");

  message.id = motorID; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC

  message.data[0] = 0xFF;
  message.data[1] = 0xFF;
  message.data[2] = 0xFF;
  message.data[3] = 0xFF; //formatted in HEX
  message.data[4] = 0xFF;
  message.data[5] = 0xFF;
  message.data[6] = 0xFF;
  message.data[7] = 0xFC;

  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  mcp2515_send_message(&message);
  }

void offMotor(int motorID){
  byte buf[8];
  //Scan analog pins. If pin reads low, print the corresponding joystick movement.

  Serial.println("disable motor");

  message.id = motorID; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC

  message.data[0] = 0xFF;
  message.data[1] = 0xFF;
  message.data[2] = 0xFF;
  message.data[3] = 0xFF; //formatted in HEX
  message.data[4] = 0xFF;
  message.data[5] = 0xFF;
  message.data[6] = 0xFF;
  message.data[7] = 0xFD;

  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  mcp2515_send_message(&message);
  }

void zero(int motorID){
  byte buf[8];
  //Scan analog pins. If pin reads low, print the corresponding joystick movement.

  Serial.println("disable motor");

  message.id = motorID; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC

  message.data[0] = 0xFF;
  message.data[1] = 0xFF;
  message.data[2] = 0xFF;
  message.data[3] = 0xFF; //formatted in HEX
  message.data[4] = 0xFF;
  message.data[5] = 0xFF;
  message.data[6] = 0xFF;
  message.data[7] = 0xFE;

  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  mcp2515_send_message(&message);
  }

void runMotor(int motorID, float x_des, float v_des, float kp, float kd, float t_ff){
  message.id = motorID; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC

  float P_des = constrain(x_des, P_MIN, P_MAX);
  float V_des = constrain(v_des, V_MIN, V_MAX);
  float Kp = constrain(kp, KP_MIN, KP_MAX);
  float Kd = constrain(kd, KD_MIN, KD_MAX);
  float T_ff = constrain(t_ff, T_MIN, T_MAX);

  // convert floats to unsigned ints
  unsigned int p_int = float_to_uint(P_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(V_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(Kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(Kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(T_ff, T_MIN, T_MAX, 12);
  // pack ints into the can buffer

  message.data[0] = p_int >> 8;
  message.data[1] = p_int & 0xFF;
  message.data[2] = v_int >> 4;
  message.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  message.data[4] = kp_int & 0xFF;
  message.data[5] = kd_int >> 4;
  message.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  message.data[7] = t_int & 0xFF;

  mcp2515_send_message(&message);

  if (mcp2515_check_message()) // ready to read
    {
      if (mcp2515_get_message(&rxmsg)) // save received data to the rxmsg
      {
        
        //unsigned char rxId = message.data[0];
        unsigned int actualPositionRaw = (rxmsg.data[1]<<8) | (rxmsg.data[2]);
        unsigned int actualVelocityRaw = (rxmsg.data[3]<<4)|(rxmsg.data[4]>>4);
        unsigned int actualCurrentRaw = (rxmsg.data[4]<<4)|(rxmsg.data[5]);
        Serial.print("Pos:");
        Serial.println(uint_to_float(actualPositionRaw, P_MIN, P_MAX, 16));
        Serial.print("Vel:");
        Serial.println(uint_to_float(actualVelocityRaw, V_MIN, V_MAX, 12));
        Serial.print("Curr:");
        Serial.println(uint_to_float(actualCurrentRaw, T_MIN, T_MAX, 12));
        
       }
    }
    delay(50); 
    
}
