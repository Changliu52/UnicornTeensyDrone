//©2016 Chang Liu

#ifndef __Accel_Controller_H__
#define __Accel_Controller_H__
#include <Arduino.h>

// function to Convert desired acceleration vector to desired thrust and desired attitude.
// Note: dt is in micro-seconds
void accel_2_attitudeThrust()
{ 
  // Make sure not close to shut down motors or reverse flying (need better codes!!)
  for (int i=0; i<3; i++) {
    if (accelD_smt[i] <-2.50) accelD_smt[i] =-2.50;
    else if (accelD_smt[i] > 2.50) accelD_smt[i] = 2.50;
  }
  
  // compute total thrust vector in world frame
  // thrust_v = force_v - mg_v
  float force_v[3];        scale3(accelD_smt, M_QUAD, force_v);
  float mg_v[3]            = {0, 0, -M_QUAD*g};
  float thrust_v[3];       subtract3(force_v, mg_v, thrust_v);
  
  // totolThust = ||thrust_v||
  float totalThrust        = norm3(thrust_v);
  
  // acceleration limit: beta * force_v = mg_v + thrust_v; && ||thrust_v|| = max_total_thrust
  float max_total_thrust = 4.00*max_thurst*0.9*g/1000.00;  // 90% max throttle in N
  if (totalThrust > max_total_thrust) {
    float a = norm3(force_v); a = a*a;
    float b = -2*force_v[2]*mg_v[2];
    float c = (mg_v[2]*mg_v[2]) - (max_total_thrust*max_total_thrust);
    float beta = (-b + sqrt(b*b-(4*a*c)))/2/a;
    scale3(force_v, beta);
    subtract3(force_v, mg_v, thrust_v);
    totalThrust = max_total_thrust;
    //Serial.println(beta);
  }
 
  // average thrust for each rotor is averageThust = ||thrust_v||/4
  averageThrust = totalThrust/4.00; // in N
  averageThrust = averageThrust*1000.00/g;  // convert to gram (Convert averageThrust to gram for PololuServo)
  
  // rescale Bd_z to unit vector (z-axis of the desired body frame)  
  float Bd_z[3];           scale3(thrust_v, 1.00/totalThrust, Bd_z);
  
  
  //______
  // Compute inital Bd_x (x-axis of the desired body frame in world xy plane)
  float Bd_x[3];
  Bd_x[0] = cos(desireAttitude[2]);
  Bd_x[1] = sin(desireAttitude[2]);
  Bd_x[2] = 0;
  
  // Compute Bd_y = (Bd_z X Bd_x)/||Bd_z X Bd_x|| (y-axis of the desired body frame)
  float Bd_y[3];            
  cross3(Bd_z, Bd_x, Bd_y);
  float Bd_yNorm = norm3(Bd_y);
  if(Bd_yNorm > 0.001){                  // Make sure Bd_y is not parallel with Bd_z!!
    scale3(Bd_y, 1/norm3(Bd_y));
  
    // Compute real Bd_x = Bd_y X Bd_z (project it to the plane normal to Bd_z)
    cross3(Bd_y, Bd_z, Bd_x);
  
    //----> Compute Rd
    if (Bd_x[0] + Bd_y[1] + Bd_z[2]> -0.999 || Bd_x[0] + Bd_y[1] + Bd_z[2]< -1.001){ // check reverse rotation
      Rd[0][0] = Bd_x[0];  Rd[0][1] = Bd_y[0];  Rd[0][2] = Bd_z[0];
      Rd[1][0] = Bd_x[1];  Rd[1][1] = Bd_y[1];  Rd[1][2] = Bd_z[1];
      Rd[2][0] = Bd_x[2];  Rd[2][1] = Bd_y[2];  Rd[2][2] = Bd_z[2];
      
      //----> Compute DotRd = (Rd - RdOld)/dt
      float timenow = micros();
      float dt = timenow - Rd_timer_last;
      Rd_timer_last = timenow;
      if (dt > 0.0) {// && dt < 40000.0) { // make sure we have reasonable timing (assumes runing at 50 Hz but allow minimium 25 Hz)
        //if (dt > 40000.0) dt = 20000.0; // smooth transation (assmue 50 hz)
        subtract3(Rd, RdOld, DotRd);
        scale3(DotRd, 1000000.00f/dt);       // sin(Rad)/s ~= Rad/s       
      }
      copy3(Rd, RdOld); // Log old Rd
    }
  }
}


#endif
