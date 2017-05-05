//©2016 Chang Liu
#ifndef POSITION_CONTROLLER_H_
#define POSITION_CONTROLLER_H_

#include "AccelController.h"
#include "AttitudeController.h"
#include "spektrumSatil.h"
#include "ESC_PWM.h"
#include <Arduino.h>

//#######################
// ACCELERATION controller (this controller update RATE is driven by the vision update RATE)
//#######################
void attach_acceleration_controller()
{
  for (int i=0; i<3; i++) {
      
      // PID-a controller
      accelD[i] = accel_user[i] - (kp_i[2]*position_integ[i]) - (kp_p[1]*acceSmt[i]);//pida controller
        
      // Limit acceleration command output
      if (accelD[i] > 4.00) accelD[i] =4.00;
      else if (accelD[i] < -4.00) accelD[i] =-4.00;
    }
}


//#######################
// POSITION controller (this controller update RATE is driven by the vision update RATE)
//#######################
void attach_position_controller()
{
  // check odroid disconnection (running at mainloop speed) make sure to copy this into detach_positon_controller() (with some changes!)
  if (!vision_new_measurment_) {
    if (odroid_lost_count_ < LOOP_HZ) odroid_lost_count_++;
    else vision_available_ = 4;
    return;
  } else odroid_lost_count_ = 0;
  
  // Position Control: Compute desired acceleration vector
  float timenow = micros();

  // update controller
  if (timenow > position_timer && position_timer > 0.0001) {
    float dt = timenow - position_timer;
    for (int i=0; i<3; i++) {
      // velocity error
      float veloError = 0.00;
      if(i<2) veloError = vVision[i]-veloD[i]; else veloError = vVision[i];
      // limit differential
      if (veloError > 3.00) veloError = 3.00;
      else if (veloError <-3.00) veloError =-3.00;

      // position integral error
      float posiError = pVision[i]-posiD[i];
      if (veloError > -kp_i[1] && veloError < kp_i[1]){ // only accumulate in slow motion
        if (posiError > 0.5) posiError = 0.5; // set maximum integral steps
        if (posiError < -0.5) posiError = -0.5;
        position_integ[i] += posiError * dt/1000000.0;
      }
      if(position_integ[i] > 2.0) position_integ[i] = 2.0; //limit max integral part
      if(position_integ[i] <-2.0) position_integ[i] =-2.0;
      
      // PID-a controller
      accelD[i] = - (kp_p[0] *posiError) - (kp_d[0] *veloError) - (kp_i[0]*position_integ[i]) - (kp_a*acceSmt[i]);//pida controller
      
      // Limit acceleration command output
      if (accelD[i] > 4.00) accelD[i] =4.00;
      else if (accelD[i] < -4.00) accelD[i] =-4.00;
    }
  } 
  // update timer
  position_timer = timenow;
  // reset new vision update flag
  vision_new_measurment_ = 0;
}


// detach position controller
//___________________________
void detach_position_controller()
{
  // check odroid disconnection (running at mainloop speed)
  if (!vision_new_measurment_) {
    if (odroid_lost_count_ < LOOP_HZ) odroid_lost_count_++;
      else vision_available_ = 4;
  } else {
    vision_new_measurment_ = 0;
    odroid_lost_count_ = 0;
  }
  
  // reset position timer
  position_timer   = micros();
  // reset position integral
  if(!armed||!onAir){
    for (int i=0; i<3; i++) {position_integ[i] = 0.00;}
  }
}

//########################
// filter all the acceleration command from outer controller to inner controller
//########################
void filter_accel_d()
{
  for (int i=0; i<3; i++){
    filter_value(accelD+i, accelD_smt+i, kp_d[1]);
  }
}

//######################
// Automatic control process
//######################
//——————————————————————
// RC
void auto_control_process_RC(float timeChange)
{
  // When vision is available and finished smooth transation, then only do manual control conversion
  if(vision_available_ ==1 && newMap ==0){ // newMap for smooth transation
    convert_RC_velo();
    compute_velo2posi(timeChange/1000000.0);
  // When vision is NOT available, or in the smooth transation
  } else {
    convert_RC_accel();
    filter_accel_d();
    attach_acceleration_controller();
    detach_position_controller(); // reset vision controller
    // Convert accel to attitude and thrust
    accel_2_attitudeThrust();
  }
}

//______________________
// OUTTER
void auto_control_process_OUTTER()
{
  // When vision is available and finished smooth transation
  if(vision_available_ ==1 && newMap ==0){ // newMap for smooth transation
    attach_position_controller();
    // Filter outter controller output
    filter_accel_d();
    // Convert accel to attitude and thrust
    accel_2_attitudeThrust();
  }
}

//______________________
// INNER
void auto_control_process_INNER(float timeChange)
{
  attach_attitude_controller(timeChange);
  apply_body_inertia();
  CommandQuad250();
}
#endif
