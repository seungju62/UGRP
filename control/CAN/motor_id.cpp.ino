#include "Arduino.h"
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>

tCAN message;

motor_id::motor_id(int id)
{
  _id = id;
  message.id=_id;
  message.header.rtr=0;
  message.header.length=8;
}

void motor_id::On()
{

}