#ifndef _ROS_msg_speech_Speech_h
#define _ROS_msg_speech_Speech_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msg_speech
{

  class Speech : public ros::Msg
  {
    public:
      uint32_t text_length;
      typedef char* _text_type;
      _text_type st_text;
      _text_type * text;
      uint32_t confidence_length;
      typedef float _confidence_type;
      _confidence_type st_confidence;
      _confidence_type * confidence;

    Speech():
      text_length(0), st_text(), text(nullptr),
      confidence_length(0), st_confidence(), confidence(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->text_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->text_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->text_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->text_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->text_length);
      for( uint32_t i = 0; i < text_length; i++){
      uint32_t length_texti = strlen(this->text[i]);
      varToArr(outbuffer + offset, length_texti);
      offset += 4;
      memcpy(outbuffer + offset, this->text[i], length_texti);
      offset += length_texti;
      }
      *(outbuffer + offset + 0) = (this->confidence_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->confidence_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->confidence_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->confidence_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence_length);
      for( uint32_t i = 0; i < confidence_length; i++){
      union {
        float real;
        uint32_t base;
      } u_confidencei;
      u_confidencei.real = this->confidence[i];
      *(outbuffer + offset + 0) = (u_confidencei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidencei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidencei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidencei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t text_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      text_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      text_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      text_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->text_length);
      if(text_lengthT > text_length)
        this->text = (char**)realloc(this->text, text_lengthT * sizeof(char*));
      text_length = text_lengthT;
      for( uint32_t i = 0; i < text_length; i++){
      uint32_t length_st_text;
      arrToVar(length_st_text, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_text-1]=0;
      this->st_text = (char *)(inbuffer + offset-1);
      offset += length_st_text;
        memcpy( &(this->text[i]), &(this->st_text), sizeof(char*));
      }
      uint32_t confidence_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      confidence_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      confidence_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      confidence_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->confidence_length);
      if(confidence_lengthT > confidence_length)
        this->confidence = (float*)realloc(this->confidence, confidence_lengthT * sizeof(float));
      confidence_length = confidence_lengthT;
      for( uint32_t i = 0; i < confidence_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_confidence;
      u_st_confidence.base = 0;
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_confidence = u_st_confidence.real;
      offset += sizeof(this->st_confidence);
        memcpy( &(this->confidence[i]), &(this->st_confidence), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "msg_speech/Speech"; };
    virtual const char * getMD5() override { return "fe4ac4189eeb21d7e82ff92e0334a9a7"; };

  };

}
#endif
