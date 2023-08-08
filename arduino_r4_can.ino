#include <Arduino_CAN.h>
#include <CanUtil.h>
#include <R7FA4M1_CAN.h>
#include <R7FA6M5_CAN.h>
#include <SyncCanMsgRingbuffer.h>

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

//#define CANSPEED_1000 0 // 이것도 필요 없는거 맞지?

float p_in1 = 0;

float v_in = 0;
float kp_in = 10;
float kd_in = 0.2;
float t_in = 0;

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
// ------------ 위에는 건들지 않아도 될 듯 -------------

static uint32_t const CAN_ID = 7;
static uint32_t msg_cnt = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  if (!CAN.begin(CanBitRate::BR_1000k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }

  delay(1000);
}


void loop() {

  byte buf[8];
  //Scan analog pins. If pin reads low, print the corresponding joystick movement.
   if (digitalRead(UP) == 0) {
    Serial.println("Enable motor");
    uint8_t const msg_data[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};

    memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
    CanMsg msg(CAN_ID, sizeof(msg_data), msg_data);
    
    msg_cnt++;
    while(digitalRead(UP) == 0);
   }


   if (digitalRead(CLICK) == 0) {
    Serial.println("zero");
    uint8_t const msg_data[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

    memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
    CanMsg msg(CAN_ID, sizeof(msg_data), msg_data);

    msg_cnt++;
    while(digitalRead(CLICK) == 0);       
   }


   if (digitalRead(DOWN) == 0) {
    Serial.println("Disable motor");
    uint8_t const msg_data[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};

    memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
    CanMsg msg(CAN_ID, sizeof(msg_data), msg_data);
    
    msg_cnt++;
    while(digitalRead(DOWN) == 0);
   }


// -------------- 일단 아래 건들이지 않기..! --------------    
   if (digitalRead(LEFT) == 0) {
    Serial.println("Left");
    // limit data to be within bounds
    p_in1 -= 0.1; 
    
    float p_des1 = constrain(p_in1, P_MIN, P_MAX);
    float p_des2 = constrain(p_in2, P_MIN, P_MAX);
    float v_des = constrain(v_in, V_MIN, V_MAX);
    float kp = constrain(kp_in, KP_MIN, KP_MAX);
    float kd = constrain(kd_in, KD_MIN, KD_MAX);
    float t_ff = constrain(t_in, T_MIN, T_MAX);
    
    // convert floats to unsigned ints
    unsigned int p_int1 = float_to_uint(p_des1, P_MIN, P_MAX, 16);
    unsigned int p_int2 = float_to_uint(p_des2, P_MIN, P_MAX, 16);
    unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    // pack ints into the can buffer

    uint8_t const msg_data[] = {p_int1 >> 8, p_int1 & 0xFF, v_int >> 4, ((v_int & 0xF) << 4) | (kp_int >> 8), kp_int & 0xFF, kd_int >> 4, ((kd_int & 0xF) << 4) | (t_int >> 8), t_int & 0xFF};

    memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
    CanMsg msg(CAN_ID, sizeof(msg_data), msg_data);
    
    msg_cnt++;

    if (CAN.available())
    {
      CanMsg const msg = CAN.read();  

      unsigned int actualPositionRaw = (msg[1]<<8) | (msg[2]);
      unsigned int actualVelocityRaw = (msg[3]<<4)|(msg[4]>>4);
      unsigned int actualCurrentRaw = (msg[4]<<4)|(msg[5]);
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
   
   if (digitalRead(RIGHT) == 0) {
       Serial.println("Right");
    // limit data to be within bounds
    p_in1 += 0.1; 
    float p_des1 = constrain(p_in1, P_MIN, P_MAX);
    float p_des2 = constrain(p_in2, P_MIN, P_MAX);
    float v_des = constrain(v_in, V_MIN, V_MAX);
    float kp = constrain(kp_in, KP_MIN, KP_MAX);
    float kd = constrain(kd_in, KD_MIN, KD_MAX);
    float t_ff = constrain(t_in, T_MIN, T_MAX);
    
    // convert floats to unsigned ints
    unsigned int p_int1 = float_to_uint(p_des1, P_MIN, P_MAX, 16);
    unsigned int p_int2 = float_to_uint(p_des2, P_MIN, P_MAX, 16);
    unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);


    // pack ints into the can buffer
    uint8_t const msg_data[] = {p_int1 >> 8, p_int1 & 0xFF, v_int >> 4, ((v_int & 0xF) << 4) | (kp_int >> 8), kp_int & 0xFF, kd_int >> 4, ((kd_int & 0xF) << 4) | (t_int >> 8), t_int & 0xFF};

    memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
    CanMsg msg(CAN_ID, sizeof(msg_data), msg_data);
    
    msg_cnt++;

    if (CAN.available())
    {
      CanMsg const msg = CAN.read();  

      unsigned int actualPositionRaw = (msg[1]<<8) | (msg[2]);
      unsigned int actualVelocityRaw = (msg[3]<<4)|(msg[4]>>4);
      unsigned int actualCurrentRaw = (msg[4]<<4)|(msg[5]);
      Serial.print("Pos:");
      Serial.println(uint_to_float(actualPositionRaw, P_MIN, P_MAX, 16));
      Serial.print("Vel:");
      Serial.println(uint_to_float(actualVelocityRaw, V_MIN, V_MAX, 12));
      Serial.print("Curr:");
      Serial.println(uint_to_float(actualCurrentRaw, T_MIN, T_MAX, 12));
        
       }
    
    delay(50); 
    
   }
   


   
}