//©2016 Chang Liu
#ifndef POSITION_MANAGE_SVO_H_
#define POSITION_MANAGE_SVO_H_

#include "spektrumSatil.h"

//________________________
// For smooth transation
//-------------------------
void vision_smoothTrans()
{
  // This takes care of the smooth transation to vision position control
  if (((newMap >0 && vision_available_==1) && enableControl) && selfie_state == selfieACTIVE) {
    convert_RC_velo(); // set current veloD for initial run problem
    // refresh position command
    posiD[0] = pVision[0];
    posiD[1] = pVision[1];
    posAltiOffset = pVision[2]-veloD[2];
    // refresh complementary filter buffer
    for (int i=0; i<3; i++){
      vVision_cf[i] = vVision[i];
    }
    // Update counter
    newMap-- ;
  } 
  
  // Resets the newMap counter whenever vision becomes unavailable
  else if((vision_available_ !=1 || !enableControl) || selfie_state != selfieACTIVE) {
    // set new map for smooth transation
    newMap = 200; // skip 200 measurements to make sure reliable reading (remember this is runing at 400Hz!!)
  }
}


// Complementary filter to combine velocity with acceleration
//______________________________________
void complementaryFilter3D(float* veloOut, float* Velo, float* Accel, float tau, float loopTime) // loopTime in s
{
  float a=tau/(tau+loopTime);
  for (int i=0; i<3; i++) {
    veloOut[i]= a* (veloOut[i] + loopTime*(9.8*Accel[i])) + (1-a)*Velo[i]; // note here accel is scaled 0.1 times (accel is in 10m/s)
  }
}


// Compensate velocity delay
//_________________________________________
#define delay_bl 12 // this value is computed by total delay from vision computer: [0.02s(vision)+0.01s(serial)]*360(imuLoopHz) = 10.8
uint8_t delay_bf_index = 0;
float deltaV_x[delay_bl]; // velocity compensation of each cycle in WORLD frame
float deltaV_y[delay_bl];
float deltaV_z[delay_bl];
float deltaV_comp[3] = {0.0, 0.0, 0.0}; // {x, y, z} in world frame for directly compensate velocity

void init_vision_velo_delayCompensator()
{
  for (int i=0; i<delay_bl; i++) {
    deltaV_x[i] = 0.0; deltaV_y[i] = 0.0; deltaV_z[i] = 0.0;
  }
}

void update_vision_velo_delayCompensator(float* acelIn, float dt)
{ 
  // add new accel measurement
  deltaV_x[delay_bf_index] = 9.8* acelIn[0] * dt; deltaV_comp[0] += deltaV_x[delay_bf_index];
  deltaV_y[delay_bf_index] = 9.8* acelIn[1] * dt; deltaV_comp[1] += deltaV_y[delay_bf_index];
  deltaV_z[delay_bf_index] = 9.8* acelIn[2] * dt; deltaV_comp[2] += deltaV_z[delay_bf_index];

  // shift indexer
  delay_bf_index++;
  if (delay_bf_index== delay_bl) delay_bf_index = 0;

  // delete oldest data
  deltaV_comp[0] -= deltaV_x[delay_bf_index];
  deltaV_comp[1] -= deltaV_y[delay_bf_index];
  deltaV_comp[2] -= deltaV_z[delay_bf_index];
}

void compensate_visionVelocity(float* vVelo)
{
  for (int i=0; i<3; i++) {
    vVelo[i] += deltaV_comp[i];
  }
}

#endif
