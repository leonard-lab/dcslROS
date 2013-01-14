#ifndef ros_dcsl_messages_belugaInput_h
#define ros_dcsl_messages_belugaInput_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"

namespace dcsl_messages
{

  class belugaInput : public ros::Msg
  {
    public:
      int vertical_motor;
      int thrust_motor;
      int servo;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        int real;
        unsigned int base;
      } u_vertical_motor;
      u_vertical_motor.real = this->vertical_motor;
      *(outbuffer + offset + 0) = (u_vertical_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vertical_motor.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->vertical_motor);
      union {
        int real;
        unsigned int base;
      } u_thrust_motor;
      u_thrust_motor.real = this->thrust_motor;
      *(outbuffer + offset + 0) = (u_thrust_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_thrust_motor.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->thrust_motor);
      union {
        int real;
        unsigned int base;
      } u_servo;
      u_servo.real = this->servo;
      *(outbuffer + offset + 0) = (u_servo.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->servo);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int real;
        unsigned int base;
      } u_vertical_motor;
      u_vertical_motor.base = 0;
      u_vertical_motor.base |= ((typeof(u_vertical_motor.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vertical_motor.base |= ((typeof(u_vertical_motor.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->vertical_motor = u_vertical_motor.real;
      offset += sizeof(this->vertical_motor);
      union {
        int real;
        unsigned int base;
      } u_thrust_motor;
      u_thrust_motor.base = 0;
      u_thrust_motor.base |= ((typeof(u_thrust_motor.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_thrust_motor.base |= ((typeof(u_thrust_motor.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->thrust_motor = u_thrust_motor.real;
      offset += sizeof(this->thrust_motor);
      union {
        int real;
        unsigned int base;
      } u_servo;
      u_servo.base = 0;
      u_servo.base |= ((typeof(u_servo.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo.base |= ((typeof(u_servo.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->servo = u_servo.real;
      offset += sizeof(this->servo);
     return offset;
    }

    const char * getType(){ return "dcsl_messages/belugaInput"; };

  };

}
#endif