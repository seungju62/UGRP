#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <Arduino.h>

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

void onMotor(int ID);
void offMotor(int ID);
void zero(int ID);
void runMotor(int ID, float x_des, float v_des, float kp, float kd, float t_ff);
unsigned int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);
