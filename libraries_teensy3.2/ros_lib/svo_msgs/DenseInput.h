#ifndef _ROS_svo_msgs_DenseInput_h
#define _ROS_svo_msgs_DenseInput_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Image.h"

namespace svo_msgs
{

  class DenseInput : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t frame_id;
      geometry_msgs::Pose pose;
      sensor_msgs::Image image;
      float min_depth;
      float max_depth;

    DenseInput():
      header(),
      frame_id(0),
      pose(),
      image(),
      min_depth(0),
      max_depth(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->frame_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->frame_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->frame_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->frame_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frame_id);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->image.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_min_depth;
      u_min_depth.real = this->min_depth;
      *(outbuffer + offset + 0) = (u_min_depth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_depth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_depth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_depth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_depth);
      union {
        float real;
        uint32_t base;
      } u_max_depth;
      u_max_depth.real = this->max_depth;
      *(outbuffer + offset + 0) = (u_max_depth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_depth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_depth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_depth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_depth);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->frame_id =  ((uint32_t) (*(inbuffer + offset)));
      this->frame_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->frame_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->frame_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->frame_id);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->image.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_min_depth;
      u_min_depth.base = 0;
      u_min_depth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_depth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_depth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_depth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_depth = u_min_depth.real;
      offset += sizeof(this->min_depth);
      union {
        float real;
        uint32_t base;
      } u_max_depth;
      u_max_depth.base = 0;
      u_max_depth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_depth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_depth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_depth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_depth = u_max_depth.real;
      offset += sizeof(this->max_depth);
     return offset;
    }

    const char * getType(){ return "svo_msgs/DenseInput"; };
    const char * getMD5(){ return "cea677f47dcf08581cc9f5efece2f7e7"; };

  };

}
#endif