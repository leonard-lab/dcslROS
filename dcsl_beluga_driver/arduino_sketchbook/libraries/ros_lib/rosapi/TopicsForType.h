#ifndef _ROS_SERVICE_TopicsForType_h
#define _ROS_SERVICE_TopicsForType_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosapi
{

static const char TOPICSFORTYPE[] = "rosapi/TopicsForType";

  class TopicsForTypeRequest : public ros::Msg
  {
    public:
      char * type;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t * length_type = (uint32_t *)(outbuffer + offset);
      *length_type = strlen( (const char*) this->type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, *length_type);
      offset += *length_type;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_type = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
     return offset;
    }

    const char * getType(){ return TOPICSFORTYPE; };
    const char * getMD5(){ return "dc67331de85cf97091b7d45e5c64ab75"; };

  };

  class TopicsForTypeResponse : public ros::Msg
  {
    public:
      uint8_t topics_length;
      char* st_topics;
      char* * topics;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = topics_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < topics_length; i++){
      uint32_t * length_topicsi = (uint32_t *)(outbuffer + offset);
      *length_topicsi = strlen( (const char*) this->topics[i]);
      offset += 4;
      memcpy(outbuffer + offset, this->topics[i], *length_topicsi);
      offset += *length_topicsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t topics_lengthT = *(inbuffer + offset++);
      if(topics_lengthT > topics_length)
        this->topics = (char**)realloc(this->topics, topics_lengthT * sizeof(char*));
      offset += 3;
      topics_length = topics_lengthT;
      for( uint8_t i = 0; i < topics_length; i++){
      uint32_t length_st_topics = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_topics; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_topics-1]=0;
      this->st_topics = (char *)(inbuffer + offset-1);
      offset += length_st_topics;
        memcpy( &(this->topics[i]), &(this->st_topics), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return TOPICSFORTYPE; };
    const char * getMD5(){ return "b0eef9a05d4e829092fc2f2c3c2aad3d"; };

  };

  class TopicsForType {
    public:
    typedef TopicsForTypeRequest Request;
    typedef TopicsForTypeResponse Response;
  };

}
#endif
