#ifndef _ROS_dcsl_messages_belugaInput_h
#define _ROS_dcsl_messages_belugaInput_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace dcsl_messages
{

  class belugaInput : public ros::Msg
  {
    public:
      std_msgs::Header header;
      int16_t vertical_motor;
      int16_t thrust_motor;
      float servo;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_vertical_motor;
      u_vertical_motor.real = this->vertical_motor;
      *(outbuffer + offset + 0) = (u_vertical_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vertical_motor.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->vertical_motor);
      union {
        int16_t real;
        uint16_t base;
      } u_thrust_motor;
      u_thrust_motor.real = this->thrust_motor;
      *(outbuffer + offset + 0) = (u_thrust_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_thrust_motor.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->thrust_motor);
      union {
        float real;
        uint32_t base;
      } u_servo;
      u_servo.real = this->servo;
      *(outbuffer + offset + 0) = (u_servo.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_vertical_motor;
      u_vertical_motor.base = 0;
      u_vertical_motor.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vertical_motor.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->vertical_motor = u_vertical_motor.real;
      offset += sizeof(this->vertical_motor);
      union {
        int16_t real;
        uint16_t base;
      } u_thrust_motor;
      u_thrust_motor.base = 0;
      u_thrust_motor.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_thrust_motor.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->thrust_motor = u_thrust_motor.real;
      offset += sizeof(this->thrust_motor);
      union {
        float real;
        uint32_t base;
      } u_servo;
      u_servo.base = 0;
      u_servo.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo = u_servo.real;
      offset += sizeof(this->servo);
     return offset;
    }

    const char * getType(){ return "dcsl_messages/belugaInput"; };
    const char * getMD5(){ return "98ee24dec3cd4bf4c1d9f2fb5e13d622"; };

  };

}
#endif