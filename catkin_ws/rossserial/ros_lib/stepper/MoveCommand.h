#ifndef _ROS_stepper_MoveCommand_h
#define _ROS_stepper_MoveCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace stepper
{

  class MoveCommand : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _movetype_type;
      _movetype_type movetype;
      typedef float _t_type;
      _t_type t;
      float x[6];
      enum { ABSOLUTE = 1 };
      enum { RELATIVE = 2 };
      enum { JOG = 3 };

    MoveCommand():
      header(),
      movetype(0),
      t(0),
      x()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_movetype;
      u_movetype.real = this->movetype;
      *(outbuffer + offset + 0) = (u_movetype.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->movetype);
      union {
        float real;
        uint32_t base;
      } u_t;
      u_t.real = this->t;
      *(outbuffer + offset + 0) = (u_t.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_t.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_t.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_t.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->t);
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_xi;
      u_xi.real = this->x[i];
      *(outbuffer + offset + 0) = (u_xi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_movetype;
      u_movetype.base = 0;
      u_movetype.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->movetype = u_movetype.real;
      offset += sizeof(this->movetype);
      union {
        float real;
        uint32_t base;
      } u_t;
      u_t.base = 0;
      u_t.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_t.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_t.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_t.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->t = u_t.real;
      offset += sizeof(this->t);
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_xi;
      u_xi.base = 0;
      u_xi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x[i] = u_xi.real;
      offset += sizeof(this->x[i]);
      }
     return offset;
    }

    const char * getType(){ return "stepper/MoveCommand"; };
    const char * getMD5(){ return "e208d09b76606d9280d351b16d327a64"; };

  };

}
#endif