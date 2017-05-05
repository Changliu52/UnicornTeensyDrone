//©2016 Chang Liu

#ifndef __MYFREEIMU_H__
#define __MYFREEIMU_H__

#include <Arduino.h>
#include "COM.h"

// Shift mean filter for accel in world frame
//_____________________________
void filterAcce(float* acce, float* acceNew)
{
  float bufferSize = 20.0; //40
  for (int i=0; i<3; i++){
    acceNew[i] = ((bufferSize-1)/bufferSize)*acceNew[i] + acce[i]/bufferSize;
  }
}

// Mean FreeIMU reading function
//_________________________________
void read_IMU(float timeChange)
{
  // for COM Debug£££££££££
  /*if(((qVision[0] != 0.0f) || (qVision[1] != 0.0f) || (qVision[2] != 0.0f) || (qVision[3] != 0.0f)) && vision_available_ == 1) {
    STATUS.v.qD[0] = qVision[0];//pVision[0];//SVOposition_w[1];//
    STATUS.v.qD[1] = qVision[1];//pVision[1];//SVOposition_w[2];//
    STATUS.v.qD[2] = qVision[2];//pVision[2];//SVOposition_w[3];//
    STATUS.v.qD[3] = qVision[3];//1.0;//accelD_smt[2];
  }*/
  
  // Get NED angles
  my3IMU.getEuler_EarthAccel_EstAlt_gyro_vis(q, acce, AltiOutput, Omega, qVision, XBeeSerial, vision_available_, VIbias_accel);

  // reset qVision
  qVision[0] = 0.0; qVision[1] = 0.0; qVision[2] = 0.0; qVision[3] = 0.0; // reset vision orientation buffer
  
  // Compute R
  float _2qxqx = 2*q[1]*q[1];  float _2qyqy = 2*q[2]*q[2];  float _2qzqz = 2*q[3]*q[3];
  float _2qxqy = 2*q[1]*q[2];  float _2qxqz = 2*q[1]*q[3];  float _2qxqw = 2*q[1]*q[0];
  float _2qyqz = 2*q[2]*q[3];  float _2qyqw = 2*q[2]*q[0];
  float _2qzqw = 2*q[3]*q[0];

  R[0][0] = 1 - _2qyqy - _2qzqz;   R[0][1] = _2qxqy - _2qzqw;        R[0][2] = _2qxqz + _2qyqw;
  R[1][0] = _2qxqy + _2qzqw;       R[1][1] = 1 - _2qxqx - _2qzqz;    R[1][2] = _2qyqz - _2qxqw;
  R[2][0] = _2qxqz - _2qyqw;       R[2][1] = _2qyqz + _2qxqw;        R[2][2] = 1 - _2qxqx - _2qyqy;
  
  // smooth acceleration
  filterAcce(acce+1, acceSmt);
}
#endif

