#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>

/* Define Joystick connection pins */
#define UP     A1
#define DOWN   A3
#define LEFT   A2
#define RIGHT  A5
#define CLICK  A4

#define P_MIN -95.50
#define P_MAX 95.50
#define V_MIN -45.0
#define V_MAX 45.0
#define KP_MIN 0.0
#define KP_MAX 500.0
#define KD_MIN 0.0
#define KD_MAX 5.0
#define T_MIN -18.0
#define T_MAX 18.0

#define CANSPEED_1000 0

float p_in = 0;
float v_in = 0;
float kp_in = 10;
float kd_in = 0.2;
float t_in = 0;

tCAN message;

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


void setup() {
  //Initialize analog pins as inputs
  pinMode(UP,INPUT);
  pinMode(DOWN,INPUT);
  pinMode(LEFT,INPUT);
  pinMode(RIGHT,INPUT);
  pinMode(CLICK,INPUT);
  
  //Pull analog pins high to enable reading of joystick movements
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(LEFT, HIGH);
  digitalWrite(RIGHT, HIGH);
  digitalWrite(CLICK, HIGH);
 
 //Initialize serial terminal connection
  Serial.begin(115200);

  if(Canbus.init(CANSPEED_1000))  //CAN Speed 1000kbps
    Serial.println("CAN Init ok");
  else
    Serial.println("Can't init CAN");

  message.id = 13; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC
 
  delay(1000);
}



void loop() {

  byte buf[8];
  //Scan analog pins. If pin reads low, print the corresponding joystick movement.
   if (digitalRead(UP) == 0) {
    Serial.println("Enable motor");
    
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
    
    while(digitalRead(UP) == 0);
   }
      
   if (digitalRead(DOWN) == 0) {
    Serial.println("Disable motor");
    
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
    
    while(digitalRead(DOWN) == 0);
   }
      
   if (digitalRead(LEFT) == 0) {
    Serial.println("Left");
    // limit data to be within bounds
    p_in -= 0.1; 
    float p_des = constrain(p_in, P_MIN, P_MAX);
    float v_des = constrain(v_in, V_MIN, V_MAX);
    float kp = constrain(kp_in, KP_MIN, KP_MAX);
    float kd = constrain(kd_in, KD_MIN, KD_MAX);
    float t_ff = constrain(t_in, T_MIN, T_MAX);
    
    // convert floats to unsigned ints
    unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    // pack ints into the can buffer
  
    message.data[0] = p_int >> 8;
    message.data[1] = p_int & 0xFF;
    message.data[2] = v_int >> 4;
    message.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    message.data[4] = kp_int & 0xFF;
    message.data[5] = kd_int >> 4;
    message.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    message.data[7] = t_int & 0xFF;

    delay(50);
    
    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
    mcp2515_send_message(&message);
   }
   
   if (digitalRead(RIGHT) == 0) {
       Serial.println("Right");
    // limit data to be within bounds
    p_in += 0.1; 
    float p_des = constrain(p_in, P_MIN, P_MAX);
    float v_des = constrain(v_in, V_MIN, V_MAX);
    float kp = constrain(kp_in, KP_MIN, KP_MAX);
    float kd = constrain(kd_in, KD_MIN, KD_MAX);
    float t_ff = constrain(t_in, T_MIN, T_MAX);
    
    // convert floats to unsigned ints
    unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);


    // pack ints into the can buffer
  
    message.data[0] = p_int >> 8;
    message.data[1] = p_int & 0xFF;
    message.data[2] = v_int >> 4;
    message.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    message.data[4] = kp_int & 0xFF;
    message.data[5] = kd_int >> 4;
    message.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    message.data[7] = t_int & 0xFF;
    
    delay(50);    

    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
    mcp2515_send_message(&message);
   }

   if (digitalRead(CLICK) == 0) {
    Serial.println("zero");
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
    
    while(digitalRead(CLICK) == 0);       
   }
}
