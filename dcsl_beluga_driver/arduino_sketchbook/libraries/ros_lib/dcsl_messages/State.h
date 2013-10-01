#ifndef _ROS_dcsl_messages_State_h
#define _ROS_dcsl_messages_State_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

namespace dcsl_messages
{

  class State : public ros::Msg
  {
    public:
      geometry_msgs::Pose pose;
      geometry_msgs::Twist twist;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "dcsl_messages/State"; };
    const char * getMD5(){ return "c79f0d88a7597db980a56d7ac144c654"; };

  };

}
#endif