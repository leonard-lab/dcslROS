#ifndef _ROS_dcsl_messages_BelugaArray_h
#define _ROS_dcsl_messages_BelugaArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dcsl_messages/belugaInput.h"

namespace dcsl_messages
{

  class BelugaArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t belugas_length;
      dcsl_messages::belugaInput st_belugas;
      dcsl_messages::belugaInput * belugas;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = belugas_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < belugas_length; i++){
      offset += this->belugas[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t belugas_lengthT = *(inbuffer + offset++);
      if(belugas_lengthT > belugas_length)
        this->belugas = (dcsl_messages::belugaInput*)realloc(this->belugas, belugas_lengthT * sizeof(dcsl_messages::belugaInput));
      offset += 3;
      belugas_length = belugas_lengthT;
      for( uint8_t i = 0; i < belugas_length; i++){
      offset += this->st_belugas.deserialize(inbuffer + offset);
        memcpy( &(this->belugas[i]), &(this->st_belugas), sizeof(dcsl_messages::belugaInput));
      }
     return offset;
    }

    const char * getType(){ return "dcsl_messages/BelugaArray"; };
    const char * getMD5(){ return "c68faab0b13cd9bb85140b9171105426"; };

  };

}
#endif