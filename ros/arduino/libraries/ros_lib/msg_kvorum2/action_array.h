#ifndef _ROS_msg_kvorum2_action_array_h
#define _ROS_msg_kvorum2_action_array_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "msg_kvorum2/action.h"

namespace msg_kvorum2
{

  class action_array : public ros::Msg
  {
    public:
      uint32_t data_length;
      typedef msg_kvorum2::action _data_type;
      _data_type st_data;
      _data_type * data;

    action_array():
      data_length(0), st_data(), data(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->data[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (msg_kvorum2::action*)realloc(this->data, data_lengthT * sizeof(msg_kvorum2::action));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->st_data.deserialize(inbuffer + offset);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(msg_kvorum2::action));
      }
     return offset;
    }

    virtual const char * getType() override { return "msg_kvorum2/action_array"; };
    virtual const char * getMD5() override { return "652de7b95d74265824a9b14fd433cef8"; };

  };

}
#endif
