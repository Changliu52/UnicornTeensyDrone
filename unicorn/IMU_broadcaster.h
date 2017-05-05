//©2016 Chang Liu
#ifndef ROS_IMU_BROADCASTER_H_
#define ROS_IMU_BROADCASTER_H_

#include <geometry_msgs/TransformStamped.h>

namespace tf
{

  class IMUBroadcaster
  {
    public:
      IMUBroadcaster() : publisher_("teensy/imu", &internal_msg) {}

      void init(ros::NodeHandle &nh)
      {
        nh.advertise(publisher_);
        internal_msg.header.frame_id = "Teensy";
        internal_msg.child_frame_id = "gravity";
      }

      void sendTransform_teensy(ros::NodeHandle &nh, float* Q, float* accel)
      {
        internal_msg.header.frame_id = "Teensy";
        
        internal_msg.transform.translation.x = accel[1];
        internal_msg.transform.translation.y = accel[2];
        internal_msg.transform.translation.z = accel[3];
        
        internal_msg.transform.rotation.w = Q[0]; 
        internal_msg.transform.rotation.x = Q[1];
        internal_msg.transform.rotation.y = Q[2]; 
        internal_msg.transform.rotation.z = Q[3]; 
          
        internal_msg.header.stamp = nh.now();
        
        publisher_.publish(&internal_msg);
      }

      
      void sendTransform_teensy_freeExposure(ros::NodeHandle &nh, float* Q, float* accel)
      {
        internal_msg.header.frame_id = "Exposure";
        
        internal_msg.transform.translation.x = accel[1];
        internal_msg.transform.translation.y = accel[2];
        internal_msg.transform.translation.z = accel[3];
        
        internal_msg.transform.rotation.w = Q[0]; 
        internal_msg.transform.rotation.x = Q[1];
        internal_msg.transform.rotation.y = Q[2]; 
        internal_msg.transform.rotation.z = Q[3]; 
          
        internal_msg.header.stamp = nh.now();
        
        publisher_.publish(&internal_msg);
      }
      
      void sendTransform_reset(ros::NodeHandle &nh, float* Q, float* initPosition)
      {
        internal_msg.header.frame_id = "reset";
        
        internal_msg.transform.translation.x = initPosition[0];
        internal_msg.transform.translation.y = initPosition[1];
        internal_msg.transform.translation.z = initPosition[2];
        
        internal_msg.transform.rotation.w = Q[0]; 
        internal_msg.transform.rotation.x = Q[1];
        internal_msg.transform.rotation.y = Q[2]; 
        internal_msg.transform.rotation.z = Q[3]; 
          
        internal_msg.header.stamp = nh.now();
        
        publisher_.publish(&internal_msg);
      }

    private:
      geometry_msgs::TransformStamped internal_msg;
      ros::Publisher publisher_;
  };

}

#endif
