#ifndef _ROS_agent_msg_AgentData_h
#define _ROS_agent_msg_AgentData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace agent_msg
{

  class AgentData : public ros::Msg
  {
    public:
      typedef int16_t _id_type;
      _id_type id;
      typedef int8_t _type_type;
      _type_type type;
      typedef float _scale_type;
      _scale_type scale;
      uint32_t color_length;
      typedef float _color_type;
      _color_type st_color;
      _color_type * color;
      uint32_t coordinates_length;
      typedef float _coordinates_type;
      _coordinates_type st_coordinates;
      _coordinates_type * coordinates;
      uint32_t rotation_length;
      typedef float _rotation_type;
      _rotation_type st_rotation;
      _rotation_type * rotation;
      enum { FISH = 0 };
      enum { OBSTACLE = 1 };
      enum { BUOY = 2 };

    AgentData():
      id(0),
      type(0),
      scale(0),
      color_length(0), st_color(), color(nullptr),
      coordinates_length(0), st_coordinates(), coordinates(nullptr),
      rotation_length(0), st_rotation(), rotation(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      offset += serializeAvrFloat64(outbuffer + offset, this->scale);
      *(outbuffer + offset + 0) = (this->color_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->color_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->color_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->color_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->color_length);
      for( uint32_t i = 0; i < color_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->color[i]);
      }
      *(outbuffer + offset + 0) = (this->coordinates_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->coordinates_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->coordinates_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->coordinates_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->coordinates_length);
      for( uint32_t i = 0; i < coordinates_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->coordinates[i]);
      }
      *(outbuffer + offset + 0) = (this->rotation_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rotation_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rotation_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rotation_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotation_length);
      for( uint32_t i = 0; i < rotation_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->rotation[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->type = u_type.real;
      offset += sizeof(this->type);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->scale));
      uint32_t color_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      color_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      color_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      color_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->color_length);
      if(color_lengthT > color_length)
        this->color = (float*)realloc(this->color, color_lengthT * sizeof(float));
      color_length = color_lengthT;
      for( uint32_t i = 0; i < color_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_color));
        memcpy( &(this->color[i]), &(this->st_color), sizeof(float));
      }
      uint32_t coordinates_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      coordinates_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      coordinates_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      coordinates_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->coordinates_length);
      if(coordinates_lengthT > coordinates_length)
        this->coordinates = (float*)realloc(this->coordinates, coordinates_lengthT * sizeof(float));
      coordinates_length = coordinates_lengthT;
      for( uint32_t i = 0; i < coordinates_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_coordinates));
        memcpy( &(this->coordinates[i]), &(this->st_coordinates), sizeof(float));
      }
      uint32_t rotation_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rotation_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rotation_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rotation_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rotation_length);
      if(rotation_lengthT > rotation_length)
        this->rotation = (float*)realloc(this->rotation, rotation_lengthT * sizeof(float));
      rotation_length = rotation_lengthT;
      for( uint32_t i = 0; i < rotation_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_rotation));
        memcpy( &(this->rotation[i]), &(this->st_rotation), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "agent_msg/AgentData"; };
    virtual const char * getMD5() override { return "6c880462a368e1566115c4d84986c2d3"; };

  };

}
#endif
