#ifndef _ROS_msg_kvorum2_senneed_h
#define _ROS_msg_kvorum2_senneed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace msg_kvorum2
{

  class senneed : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _need_food_type;
      _need_food_type need_food;
      typedef int32_t _need_comfort_type;
      _need_comfort_type need_comfort;
      typedef int32_t _need_save_type;
      _need_save_type need_save;
      typedef int32_t _sen_food_type;
      _sen_food_type sen_food;
      typedef int32_t _sen_obstacle_type;
      _sen_obstacle_type sen_obstacle;
      typedef int32_t _sen_danger_type;
      _sen_danger_type sen_danger;
      typedef int32_t _sen_hungry_type;
      _sen_hungry_type sen_hungry;
      typedef int32_t _sen_excit_type;
      _sen_excit_type sen_excit;
      typedef int32_t _sen_inhibit_type;
      _sen_inhibit_type sen_inhibit;
      typedef int32_t _sen_reflex_type;
      _sen_reflex_type sen_reflex;

    senneed():
      header(),
      need_food(0),
      need_comfort(0),
      need_save(0),
      sen_food(0),
      sen_obstacle(0),
      sen_danger(0),
      sen_hungry(0),
      sen_excit(0),
      sen_inhibit(0),
      sen_reflex(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_need_food;
      u_need_food.real = this->need_food;
      *(outbuffer + offset + 0) = (u_need_food.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_need_food.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_need_food.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_need_food.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->need_food);
      union {
        int32_t real;
        uint32_t base;
      } u_need_comfort;
      u_need_comfort.real = this->need_comfort;
      *(outbuffer + offset + 0) = (u_need_comfort.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_need_comfort.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_need_comfort.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_need_comfort.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->need_comfort);
      union {
        int32_t real;
        uint32_t base;
      } u_need_save;
      u_need_save.real = this->need_save;
      *(outbuffer + offset + 0) = (u_need_save.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_need_save.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_need_save.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_need_save.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->need_save);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_food;
      u_sen_food.real = this->sen_food;
      *(outbuffer + offset + 0) = (u_sen_food.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sen_food.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sen_food.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sen_food.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sen_food);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_obstacle;
      u_sen_obstacle.real = this->sen_obstacle;
      *(outbuffer + offset + 0) = (u_sen_obstacle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sen_obstacle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sen_obstacle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sen_obstacle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sen_obstacle);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_danger;
      u_sen_danger.real = this->sen_danger;
      *(outbuffer + offset + 0) = (u_sen_danger.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sen_danger.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sen_danger.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sen_danger.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sen_danger);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_hungry;
      u_sen_hungry.real = this->sen_hungry;
      *(outbuffer + offset + 0) = (u_sen_hungry.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sen_hungry.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sen_hungry.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sen_hungry.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sen_hungry);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_excit;
      u_sen_excit.real = this->sen_excit;
      *(outbuffer + offset + 0) = (u_sen_excit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sen_excit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sen_excit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sen_excit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sen_excit);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_inhibit;
      u_sen_inhibit.real = this->sen_inhibit;
      *(outbuffer + offset + 0) = (u_sen_inhibit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sen_inhibit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sen_inhibit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sen_inhibit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sen_inhibit);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_reflex;
      u_sen_reflex.real = this->sen_reflex;
      *(outbuffer + offset + 0) = (u_sen_reflex.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sen_reflex.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sen_reflex.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sen_reflex.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sen_reflex);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_need_food;
      u_need_food.base = 0;
      u_need_food.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_need_food.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_need_food.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_need_food.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->need_food = u_need_food.real;
      offset += sizeof(this->need_food);
      union {
        int32_t real;
        uint32_t base;
      } u_need_comfort;
      u_need_comfort.base = 0;
      u_need_comfort.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_need_comfort.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_need_comfort.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_need_comfort.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->need_comfort = u_need_comfort.real;
      offset += sizeof(this->need_comfort);
      union {
        int32_t real;
        uint32_t base;
      } u_need_save;
      u_need_save.base = 0;
      u_need_save.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_need_save.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_need_save.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_need_save.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->need_save = u_need_save.real;
      offset += sizeof(this->need_save);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_food;
      u_sen_food.base = 0;
      u_sen_food.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sen_food.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sen_food.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sen_food.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sen_food = u_sen_food.real;
      offset += sizeof(this->sen_food);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_obstacle;
      u_sen_obstacle.base = 0;
      u_sen_obstacle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sen_obstacle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sen_obstacle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sen_obstacle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sen_obstacle = u_sen_obstacle.real;
      offset += sizeof(this->sen_obstacle);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_danger;
      u_sen_danger.base = 0;
      u_sen_danger.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sen_danger.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sen_danger.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sen_danger.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sen_danger = u_sen_danger.real;
      offset += sizeof(this->sen_danger);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_hungry;
      u_sen_hungry.base = 0;
      u_sen_hungry.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sen_hungry.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sen_hungry.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sen_hungry.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sen_hungry = u_sen_hungry.real;
      offset += sizeof(this->sen_hungry);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_excit;
      u_sen_excit.base = 0;
      u_sen_excit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sen_excit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sen_excit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sen_excit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sen_excit = u_sen_excit.real;
      offset += sizeof(this->sen_excit);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_inhibit;
      u_sen_inhibit.base = 0;
      u_sen_inhibit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sen_inhibit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sen_inhibit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sen_inhibit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sen_inhibit = u_sen_inhibit.real;
      offset += sizeof(this->sen_inhibit);
      union {
        int32_t real;
        uint32_t base;
      } u_sen_reflex;
      u_sen_reflex.base = 0;
      u_sen_reflex.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sen_reflex.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sen_reflex.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sen_reflex.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sen_reflex = u_sen_reflex.real;
      offset += sizeof(this->sen_reflex);
     return offset;
    }

    virtual const char * getType() override { return "msg_kvorum2/senneed"; };
    virtual const char * getMD5() override { return "94b8eea9f449cf62466ad6a086d30736"; };

  };

}
#endif
