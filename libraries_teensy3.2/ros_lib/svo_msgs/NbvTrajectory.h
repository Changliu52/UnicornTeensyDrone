#ifndef _ROS_svo_msgs_NbvTrajectory_h
#define _ROS_svo_msgs_NbvTrajectory_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace svo_msgs
{

  class NbvTrajectory : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t trajectory_length;
      geometry_msgs::Pose st_trajectory;
      geometry_msgs::Pose * trajectory;

    NbvTrajectory():
      header(),
      trajectory_length(0), trajectory(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = trajectory_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < trajectory_length; i++){
      offset += this->trajectory[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t trajectory_lengthT = *(inbuffer + offset++);
      if(trajectory_lengthT > trajectory_length)
        this->trajectory = (geometry_msgs::Pose*)realloc(this->trajectory, trajectory_lengthT * sizeof(geometry_msgs::Pose));
      offset += 3;
      trajectory_length = trajectory_lengthT;
      for( uint8_t i = 0; i < trajectory_length; i++){
      offset += this->st_trajectory.deserialize(inbuffer + offset);
        memcpy( &(this->trajectory[i]), &(this->st_trajectory), sizeof(geometry_msgs::Pose));
      }
     return offset;
    }

    const char * getType(){ return "svo_msgs/NbvTrajectory"; };
    const char * getMD5(){ return "eb6b516341d535312e1b3849a61711c1"; };

  };

}
#endif