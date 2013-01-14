#ifndef ros_dcsl_messages_TwistArray_h
#define ros_dcsl_messages_TwistArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"

namespace dcsl_messages
{

  class TwistArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      unsigned char twists_length;
      geometry_msgs::Twist st_twists;
      geometry_msgs::Twist * twists;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = twists_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < twists_length; i++){
      offset += this->twists[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      unsigned char twists_lengthT = *(inbuffer + offset++);
      if(twists_lengthT > twists_length)
        this->twists = (geometry_msgs::Twist*)realloc(this->twists, twists_lengthT * sizeof(geometry_msgs::Twist));
      offset += 3;
      twists_length = twists_lengthT;
      for( unsigned char i = 0; i < twists_length; i++){
      offset += this->st_twists.deserialize(inbuffer + offset);
        memcpy( &(this->twists[i]), &(this->st_twists), sizeof(geometry_msgs::Twist));
      }
     return offset;
    }

    const char * getType(){ return "dcsl_messages/TwistArray"; };

  };

}
#endif