#ifndef _ROS_stepper_PositionReport_h
#define _ROS_stepper_PositionReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace stepper
{

  class PositionReport : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _code_type;
      _code_type code;
      typedef int32_t _queue_length_type;
      _queue_length_type queue_length;
      typedef float _queue_time_type;
      _queue_time_type queue_time;
      int32_t positions[6];
      int32_t planner_positions[6];
      enum { ACK = 1 };
      enum { DONE = 2 };
      enum { EMPTY = 3 };

    PositionReport():
      header(),
      code(0),
      queue_length(0),
      queue_time(0),
      positions(),
      planner_positions()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_code;
      u_code.real = this->code;
      *(outbuffer + offset + 0) = (u_code.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->code);
      union {
        int32_t real;
        uint32_t base;
      } u_queue_length;
      u_queue_length.real = this->queue_length;
      *(outbuffer + offset + 0) = (u_queue_length.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_queue_length.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_queue_length.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_queue_length.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->queue_length);
      union {
        float real;
        uint32_t base;
      } u_queue_time;
      u_queue_time.real = this->queue_time;
      *(outbuffer + offset + 0) = (u_queue_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_queue_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_queue_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_queue_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->queue_time);
      for( uint32_t i = 0; i < 6; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_positionsi;
      u_positionsi.real = this->positions[i];
      *(outbuffer + offset + 0) = (u_positionsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positionsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positionsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positionsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->positions[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_planner_positionsi;
      u_planner_positionsi.real = this->planner_positions[i];
      *(outbuffer + offset + 0) = (u_planner_positionsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_planner_positionsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_planner_positionsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_planner_positionsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->planner_positions[i]);
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
      } u_code;
      u_code.base = 0;
      u_code.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->code = u_code.real;
      offset += sizeof(this->code);
      union {
        int32_t real;
        uint32_t base;
      } u_queue_length;
      u_queue_length.base = 0;
      u_queue_length.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_queue_length.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_queue_length.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_queue_length.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->queue_length = u_queue_length.real;
      offset += sizeof(this->queue_length);
      union {
        float real;
        uint32_t base;
      } u_queue_time;
      u_queue_time.base = 0;
      u_queue_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_queue_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_queue_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_queue_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->queue_time = u_queue_time.real;
      offset += sizeof(this->queue_time);
      for( uint32_t i = 0; i < 6; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_positionsi;
      u_positionsi.base = 0;
      u_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->positions[i] = u_positionsi.real;
      offset += sizeof(this->positions[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_planner_positionsi;
      u_planner_positionsi.base = 0;
      u_planner_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_planner_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_planner_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_planner_positionsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->planner_positions[i] = u_planner_positionsi.real;
      offset += sizeof(this->planner_positions[i]);
      }
     return offset;
    }

    const char * getType(){ return "stepper/PositionReport"; };
    const char * getMD5(){ return "87f073e74cda85c22e60cc674c5b76bd"; };

  };

}
#endif