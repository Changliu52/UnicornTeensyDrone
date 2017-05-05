//©2016 Chang Liu
#ifndef __SATIL_H__
#define __SATIL_H__

#include <Arduino.h>
#include "MyMatMath.h"

#define Start_byte 0x01
#define Stop_byte 0x03


#define chnNum 7
int chns[chnNum] = {0,0,0,0,0,0,0};
int chnsChange;
int chnsOld[chnNum] = {0,0,0,0,0,0,0};

// This function is coupled with spectrum DSM2 satellite receiver
template <class USerial>
boolean update_receiver(USerial *s)
{
  if (s->available() >= 16) 
  {
    // Check Start Bytes
    if (s->read() != Start_byte)  return 0;
    // Read channels
    for (int i = 0; i < chnNum; i ++){
      // update buffer
      chns[i] = (s->read() << 8) + s->read();
    }
    // Check Stop Bytes
    if (s->read() != Stop_byte) return 0;
    return 1;
  }
  return 0;
}

//_______________________
// Read Manual
template <class USerial>
boolean read_receiver(USerial *s)
{
    // Get Control From Receiver
  if (update_receiver(s)) {
    Serial.println(chns[4]);
    // Set Flags (Finite State Machine)
    if (chns[4] == 6315) {armed = 1;} else {armed = 0;}
    if (chns[5] >= 200 && chns[5] <= 854) {onAir = 1;} else {onAir = 0;}
    if (chns[2] == 4266) {enableControl = 1;} else {enableControl = 0;}
    
    // Throttle
    throttleLevel_raw = min(400.00, max(0, ((float)chns[5] - 175.00) * 400.00 / (848.00 - 175.00))); // throttle level rescaled to bottom:0 ~ 400:top
    
    // Yaw
    float yawChange = 0.5 * ((3237.00 - (float)chns[6]) / (3925.00 - 3237.00) + 0.49);
    if (yawChange > 0.04 || yawChange < -0.04) desireAttitude_raw[2] = desireAttitude_raw[2] - (0.3*yawChange);
    // Roll & Pitch (value follows front-left-up bodyframe)
    desireAttitude_raw[0] =  min(pi/4.00, max(-pi/4.00, ((float)chns[0]-1195.0)*(pi/2.00)/(1195.0-1876.0) + (pi/4.00))); // angle around x-axis left:-45 ~ 45:right
    desireAttitude_raw[1] = -min(pi/4.00, max(-pi/4.00, ((float)chns[3]-2221.0)*(pi/2.00)/(2221.0-2895.0) + (pi/4.00))); // angle around y-axis down:-45 ~ 45:up
    
    return true;
  }
  return false;
}



//____________________
// Function used to compute Rd as desired rotation matrix
void computeRd_RC()
{
  // Compute Bd_z  (z-axis of the desired body frame)
  float Bd_z_w[3];
  Bd_z_w[0] =  sin(desireAttitude[1]);
  Bd_z_w[1] = -sin(desireAttitude[0]) * cos(desireAttitude[1]);
  Bd_z_w[2] =  cos(desireAttitude[0]) * cos(desireAttitude[1]);
  
  float YawRot[3][3];
  YawRot[0][0] = cos(desireAttitude[2]);   YawRot[0][1] = -sin(desireAttitude[2]);   YawRot[0][2] = 0;
  YawRot[1][0] = sin(desireAttitude[2]);   YawRot[1][1] =  cos(desireAttitude[2]);   YawRot[1][2] = 0;
  YawRot[2][0] = 0;                        YawRot[2][1] =  0;                        YawRot[2][2] = 1;
  
  float Bd_z[3];            multiply3(YawRot, Bd_z_w, Bd_z);
  
  // Compute inital Bd_x (x-axis of the desired body frame in world xy plane)
  float W_x[3] = {1,0,0};
  float Bd_x[3];            multiply3(YawRot,  W_x,  Bd_x);
  
  // Compute Bd_y = (Bd_z X Bd_x)/||Bd_z X Bd_x|| (y-axis of the desired body frame)
  float Bd_y[3];
  cross3(Bd_z, Bd_x, Bd_y);
  float Bd_yNorm = norm3(Bd_y);
  if(Bd_yNorm > 0.001){                  // Make sure Bd_y is not parallel with Bd_z!!
    scale3(Bd_y, 1/norm3(Bd_y));
  
    // Compute real Bd_x = Bd_y X Bd_z (project it to the plane normal to Bd_z)
    cross3(Bd_y, Bd_z, Bd_x);
  
    //----> Compute Rd
    Rd[0][0] = Bd_x[0];  Rd[0][1] = Bd_y[0];  Rd[0][2] = Bd_z[0];
    Rd[1][0] = Bd_x[1];  Rd[1][1] = Bd_y[1];  Rd[1][2] = Bd_z[1];
    Rd[2][0] = Bd_x[2];  Rd[2][1] = Bd_y[2];  Rd[2][2] = Bd_z[2];
    
    //----> Compute DotRd = (Rd - RdOld)/dt
    float timenow = micros();
    float dt = timenow - Rd_timer_last;
    Rd_timer_last = timenow;
    if (dt > 0.0) {
      if (dt > 40000.0) dt = 20000.0; // smooth transation (assmue 50 hz)
      subtract3(Rd, RdOld, DotRd);
      scale3(DotRd, 1000000.00f/dt);       // sin(Rad)/s ~= Rad/s
    }
    copy3(Rd, RdOld);
  }
}

//________________
// Acceleration
void convert_RC_accel()
{
  // desired acceleration in world frame (front-left-up)
  accel_user[0] =  5.00*desireAttitude[1]*4.00/pi;     //-5 ~ 5 m/s/s front x
  accel_user[1] = -5.00*desireAttitude[0]*4.00/pi;     //-5 ~ 5 m/s/s left y
  accel_user[2] =  4.00*(throttleLevel-200.00)/200.00; //-4 ~ 4 m/s/s up z
  
  // Yaw rotate to body frame
  float YawRot[3][3];
  YawRot[0][0] = cos(desireAttitude[2]);   YawRot[0][1] = -sin(desireAttitude[2]);   YawRot[0][2] = 0;
  YawRot[1][0] = sin(desireAttitude[2]);   YawRot[1][1] =  cos(desireAttitude[2]);   YawRot[1][2] = 0;
  YawRot[2][0] = 0;                        YawRot[2][1] =  0;                        YawRot[2][2] = 1;
  float temp[3];            multiply3(YawRot, accel_user, temp);
  copy3(temp, accel_user);
}


//________________
// Acceleration Altitude
void convert_RC_accel_alt()
{
  // desired acceleration in world frame (front-left-up)
  accelD[0] =  5.00*desireAttitude[1]*4.00/pi;     //-5 ~ 5 m/s/s front x
  accelD[1] = -5.00*desireAttitude[0]*4.00/pi;     //-5 ~ 5 m/s/s left y
  veloD[2]  = -1.00+3.00*(throttleLevel)/400.00;        //-1 ~ 2 m up altitude
  
  // Yaw rotate to body frame
  float YawRot[3][3];
  YawRot[0][0] = cos(desireAttitude[2]);   YawRot[0][1] = -sin(desireAttitude[2]);   YawRot[0][2] = 0;
  YawRot[1][0] = sin(desireAttitude[2]);   YawRot[1][1] =  cos(desireAttitude[2]);   YawRot[1][2] = 0;
  YawRot[2][0] = 0;                        YawRot[2][1] =  0;                        YawRot[2][2] = 1;
  float temp[3];            multiply3(YawRot, accelD, temp);
  copy3(temp, accelD);
}



//_____________
// Velocity
void convert_RC_velo()
{
  // desired velocity in body frame (front-left-up)
  veloD[0] =  1.00*desireAttitude[1]*4.00/pi;     //-1.0 ~ 1.0 m/s front x
  veloD[1] = -1.00*desireAttitude[0]*4.00/pi;     //-1.0 ~ 1.0 m/s left y
  veloD[2] = -1.00+3.00*(throttleLevel)/400.00;  //-1 ~ 2 m up altitude

  // avoid remote control drift
  if (veloD[0] > -0.05 && veloD[0] < 0.05) veloD[0] = 0.0;
  if (veloD[1] > -0.05 && veloD[1] < 0.05) veloD[1] = 0.0;
  
  // Yaw rotate to world frame
  float YawRot[3][3];
  YawRot[0][0] = cos(desireAttitude[2]);   YawRot[0][1] = -sin(desireAttitude[2]);   YawRot[0][2] = 0;
  YawRot[1][0] = sin(desireAttitude[2]);   YawRot[1][1] =  cos(desireAttitude[2]);   YawRot[1][2] = 0;
  YawRot[2][0] = 0;                        YawRot[2][1] =  0;                        YawRot[2][2] = 1;
  float temp[3];            multiply3(YawRot, veloD, temp);
  copy3(temp, veloD);
}



//______________
// Position
void compute_velo2posi(float dt) // dt in sec
{
  // velocity to position
  if (veloD[0]*veloD[0] > 0.0001) posiD[0] += veloD[0]*dt; 
  if (veloD[1]*veloD[1] > 0.0001) posiD[1] += veloD[1]*dt;
  posiD[2]  = veloD[2] + posAltiOffset; // velo[2] is altitude already
}


// Shift mean filter for RC remote in world frame
//_____________________________
void filter_value(float* raw, float* outputBuffer, float bufferSize)
{
  outputBuffer[0] = ((bufferSize-1)/bufferSize)*outputBuffer[0] + raw[0]/bufferSize;
}

//____________________
// Function to filter RC controller 
void filterRC()
{
  filter_value(&throttleLevel_raw, &throttleLevel, 10.0);
  for (int i=0; i<2; i++){
    filter_value(desireAttitude_raw+i, desireAttitude+i, 10.0);
  }
  filter_value(desireAttitude_raw+2, desireAttitude+2, 30.0);
}
#endif
