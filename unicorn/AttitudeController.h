//©2016 Chang Liu
#ifndef __Attitude_Controller_H__
#define __Attitude_Controller_H__

#include <Arduino.h>
#include "ESC_PWM.h"

// Enable attitude controller
//_____________________________________
void attach_attitude_controller(float dt)
{
  // --------->  
  // Compute RdT and RT
  float RdT[3][3];           transpose3(Rd, RdT);
  float RT[3][3];            transpose3(R,  RT);

  // Compute RdT_R and RT_Rd
  float RdT_R[3][3];         multiply3(RdT, R, RdT_R);
  float RT_Rd[3][3];         multiply3(RT, Rd, RT_Rd);

  // ErrorR = 0.5 * (Rd' * R - R' * Rd)V;
  float RdT_R_RT_Td[3][3];   subtract3(RdT_R, RT_Rd, RdT_R_RT_Td);
  float ErrorR[3];           veeSkew3(RdT_R_RT_Td, ErrorR);
                             scale3(ErrorR, 0.50);
    

  // --------->  
  // Compute OmegaD = (RdT * DotRd)V
  float RdT_DotRd[3][3];     multiply3(RdT, DotRd, RdT_DotRd);
  float OmegaD[3];           veeSkew3(RdT_DotRd, OmegaD);
  
  // Compute RT_Rd_OmegaD
  float RT_Rd_OmegaD[3];     multiply3(RT_Rd, OmegaD, RT_Rd_OmegaD);
  
  // ####
  // scale RT_Rd_OmegaD for better hover stability
  /*float commandLevel = accelD_smt[0]+position_integ[0]; // used to find the maximum acceleration axis (without integral)
  commandLevel *= commandLevel;
  for (int i=1; i<3; i++) {
    float commandProbe = accelD_smt[i]+position_integ[i];
    commandProbe *= commandProbe;
    if (commandProbe > commandLevel) commandLevel = commandProbe;
  }

  // computer omega supression factor
  float OmegaSupressor = 0.9; // -0.5 -0.3 0 0.3 0.5 -> 1 0.5 0.5 1
  commandLevel = sqrt(commandLevel);
  if (commandLevel <= 0.7)    OmegaSupressor = 0.7;
  else if (commandLevel < 0.9) OmegaSupressor = commandLevel;
  */
  // apply suppression
  //scale3(RT_Rd_OmegaD, 0.8);
  // ####
  
  // ErrorOmega = Omega - RT_Rd_OmegaD
  float ErrorOmega[3];       subtract3(Omega, RT_Rd_OmegaD, ErrorOmega);
  for (int i=0; i<3; i++){
    if (ErrorOmega[i]> 50.0) ErrorOmega[i] = 30.0;//50.0
    else if (ErrorOmega[i]<-50.0) ErrorOmega[i] =-30.0;
  }
  
  // --------->  PID HERE
  // Desired Angular accleration should applied as output from controller
  for (int i=0; i<3; i++){
    IErrorR[i] += ErrorR[i]*dt/1000000.0;
    if (IErrorR[i] > I_cut) IErrorR[i] = I_cut;
    if (IErrorR[1] <-I_cut) IErrorR[i] =-I_cut;
  }
  AngularAccelD[0] = - kp[0]*ErrorR[0] - ki[0]*IErrorR[0] - kd[0]*ErrorOmega[0];
  AngularAccelD[1] = - kp[1]*ErrorR[1] - ki[1]*IErrorR[1] - kd[1]*ErrorOmega[1];
  AngularAccelD[2] = - kp[2]*ErrorR[2] - ki[2]*IErrorR[2] - kd[2]*ErrorOmega[2];
}




// Disable attitude controller
//_____________________________________
void detach_attitude_controller()
{
  // Reset Yaw
  desireAttitude[2] = atan2(R[1][0], R[0][0]);
  desireAttitude_raw[2] = desireAttitude[2];
  // Reset attitude integral
  for(int i=0; i<3; i++){
    IErrorR[i] = 0.0;
  }
}


// apply quadrotor inertia
//________________________________________
void apply_body_inertia()
{
  float J[3][3] = {{1000*Jx, 0, 0}, {0, 1000*Jy, 0}, { 0, 0, 1000*Jz}};
  multiply3(J, AngularAccelD, Moment);
}


// Use RC throttle directly
//__________________________________________________
void directRC_thrust()
{
  averageThrust = throttleLevel;
}


//____________________
// For smooth take off
//____________________
void smooth_takeoff_throttle()
{
  if ((throttleDamp <=1.00 && armed) && onAir){
    throttleDamp = throttleDamp+0.02; // takes 2 seconds to reach 1.00, at 50Hz
  }
  else if (!armed || !onAir) {
    throttleDamp = 0.00;
  }
}


//__________________________________________
// Map the controller outputs to motor commands (NED convention)
//-><-
//<-->
void CommandQuad250()
{
  // Body dimension matrix convertion B^(-1),
  float quadMat[3][3] = {{sqrt(2)/ARM_LENGTH, 0, 0}, {0, sqrt(2)/ARM_LENGTH, 0}, {0, 0, 1}}; // assume CQ=1
  multiply3(quadMat, Moment, DeltaT);

  // Thrust Ramp Limit
  //averageThrust = averageThrust*throttleDamp;
  
  // Compute thrust for each axis in grams (for the sake of implicity, I did not divid it by 4 as proper inverse.)
                                                                 //  Z            y           x
  thrust[0] = min(max_thurst, min(2.00*averageThrust, max(20.0, ( DeltaT[2] - DeltaT[1] + DeltaT[0] + averageThrust))));
  thrust[1] = min(max_thurst, min(2.00*averageThrust, max(20.0, (-DeltaT[2] - DeltaT[1] - DeltaT[0] + averageThrust))));
  thrust[2] = min(max_thurst, min(2.00*averageThrust, max(20.0, ( DeltaT[2] + DeltaT[1] - DeltaT[0] + averageThrust))));
  thrust[3] = min(max_thurst, min(2.00*averageThrust, max(20.0, (-DeltaT[2] + DeltaT[1] + DeltaT[0] + averageThrust))));

  // Convert it to arduino servo throttle
  thrust2throttle250_PWM_CLOSELoop(thrust, throttle); //note here 'throttle' is in pwm%
  // Output PWM to ESCs
  moveServo(throttle);//note here 'throttle' is in pwm%
}
#endif
