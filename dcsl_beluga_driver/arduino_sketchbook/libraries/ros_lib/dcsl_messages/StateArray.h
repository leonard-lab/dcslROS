#ifndef _ROS_dcsl_messages_StateArray_h
#define _ROS_dcsl_messages_StateArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dcsl_messages/State.h"

namespace dcsl_messages
{

  class StateArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t states_length;
      dcsl_messages::State st_states;
      dcsl_messages::State * states;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = states_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < states_length; i++){
      offset += this->states[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t states_lengthT = *(inbuffer + offset++);
      if(states_lengthT > states_length)
        this->states = (dcsl_messages::State*)realloc(this->states, states_lengthT * sizeof(dcsl_messages::State));
      offset += 3;
      states_length = states_lengthT;
      for( uint8_t i = 0; i < states_length; i++){
      offset += this->st_states.deserialize(inbuffer + offset);
        memcpy( &(this->states[i]), &(this->st_states), sizeof(dcsl_messages::State));
      }
     return offset;
    }

    const char * getType(){ return "dcsl_messages/StateArray"; };
    const char * getMD5(){ return "cb425e1b258fdb02a10aa02314e1e1ea"; };

  };

}
#endif