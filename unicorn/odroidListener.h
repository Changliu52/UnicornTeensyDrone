//©2016 Chang Liu
//
//  transform_listener.h
//  
//
//  Created by chang liu on 28/01/2015.
//
//

#ifndef ROS_ODROIDLISTENER_H_
#define ROS_ODROIDLISTENER_H_

#include <vi_ekf/teensyPilot.h>
#include "PositionManageSVO.h"

namespace vi_ekf
{
    
    class odroidListener
    {
    public:
        // Constructor
        odroidListener() : Subscriber_("ekf/output", &messageCb)
        {}
        
        // Initialisation function
        void init(ros::NodeHandle &nh)
        {
            nh.subscribe(Subscriber_);
        }
        
        
    private:
        // core variables
        vi_ekf::teensyPilot internal_msg;
        ros::Subscriber<vi_ekf::teensyPilot> Subscriber_;
        
        // Callback function when message arrives
        static void messageCb(const vi_ekf::teensyPilot& internal_msg)  // we get this message with 10ms latency!
        {
            qVision[0]        = internal_msg.qw; 
            qVision[1]        = internal_msg.qx; 
            qVision[2]        = internal_msg.qy; 
            qVision[3]        = internal_msg.qz; 
            pVision[0]        = internal_msg.px; 
            pVision[1]        = internal_msg.py; 
            pVision[2]        = internal_msg.pz; 
            vVision[0]        = internal_msg.vx; 
            vVision[1]        = internal_msg.vy; 
            vVision[2]        = internal_msg.vz; 
            VIbias_accel[0]   = internal_msg.bx; 
            VIbias_accel[1]   = internal_msg.by; 
            VIbias_accel[2]   = internal_msg.bz; 
            lambdaVision      = internal_msg.lambda;//(float)(nh.now().toSec()-946688000.0)- internal_msg.lambda; // 
            vision_available_ = internal_msg.status;
            vision_new_measurment_ = 1;

            // for debugging
            vVision_raw[0] = vVision[0];
            vVision_raw[1] = vVision[1];
            vVision_raw[2] = vVision[2];
            
            // compensate velocity delay
            compensate_visionVelocity(vVision);
        }
    };
    
}

#endif
