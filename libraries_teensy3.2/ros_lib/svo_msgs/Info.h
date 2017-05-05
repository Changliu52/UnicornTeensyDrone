#ifndef _ROS_svo_msgs_Info_h
#define _ROS_svo_msgs_Info_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace svo_msgs
{

  class Info : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float processing_time;
      uint8_t keyframes_length;
      int32_t st_keyframes;
      int32_t * keyframes;
      int32_t num_matches;
      int32_t tracking_quality;
      int32_t stage;

    Info():
      header(),
      processing_time(0),
      keyframes_length(0), keyframes(NULL),
      num_matches(0),
      tracking_quality(0),
      stage(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_processing_time;
      u_processing_time.real = this->processing_time;
      *(outbuffer + offset + 0) = (u_processing_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_processing_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_processing_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_processing_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->processing_time);
      *(outbuffer + offset++) = keyframes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < keyframes_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_keyframesi;
      u_keyframesi.real = this->keyframes[i];
      *(outbuffer + offset + 0) = (u_keyframesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_keyframesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_keyframesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_keyframesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->keyframes[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_num_matches;
      u_num_matches.real = this->num_matches;
      *(outbuffer + offset + 0) = (u_num_matches.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_matches.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num_matches.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num_matches.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_matches);
      union {
        int32_t real;
        uint32_t base;
      } u_tracking_quality;
      u_tracking_quality.real = this->tracking_quality;
      *(outbuffer + offset + 0) = (u_tracking_quality.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tracking_quality.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tracking_quality.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tracking_quality.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tracking_quality);
      union {
        int32_t real;
        uint32_t base;
      } u_stage;
      u_stage.real = this->stage;
      *(outbuffer + offset + 0) = (u_stage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stage);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_processing_time;
      u_processing_time.base = 0;
      u_processing_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_processing_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_processing_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_processing_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->processing_time = u_processing_time.real;
      offset += sizeof(this->processing_time);
      uint8_t keyframes_lengthT = *(inbuffer + offset++);
      if(keyframes_lengthT > keyframes_length)
        this->keyframes = (int32_t*)realloc(this->keyframes, keyframes_lengthT * sizeof(int32_t));
      offset += 3;
      keyframes_length = keyframes_lengthT;
      for( uint8_t i = 0; i < keyframes_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_keyframes;
      u_st_keyframes.base = 0;
      u_st_keyframes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_keyframes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_keyframes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_keyframes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_keyframes = u_st_keyframes.real;
      offset += sizeof(this->st_keyframes);
        memcpy( &(this->keyframes[i]), &(this->st_keyframes), sizeof(int32_t));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_num_matches;
      u_num_matches.base = 0;
      u_num_matches.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_matches.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num_matches.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num_matches.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->num_matches = u_num_matches.real;
      offset += sizeof(this->num_matches);
      union {
        int32_t real;
        uint32_t base;
      } u_tracking_quality;
      u_tracking_quality.base = 0;
      u_tracking_quality.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tracking_quality.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tracking_quality.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tracking_quality.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tracking_quality = u_tracking_quality.real;
      offset += sizeof(this->tracking_quality);
      union {
        int32_t real;
        uint32_t base;
      } u_stage;
      u_stage.base = 0;
      u_stage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stage = u_stage.real;
      offset += sizeof(this->stage);
     return offset;
    }

    const char * getType(){ return "svo_msgs/Info"; };
    const char * getMD5(){ return "175acf2e539a9219addbcbeafca8552f"; };

  };

}
#endif