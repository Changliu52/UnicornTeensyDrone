//©2016 Chang Liu

#include <Wire.h>
#include <SPI.h>

// FreeIMU Headers
#include <ADXL345.h>
#include <bma180.h>
#include <HMC58X3.h>
#include <LSM303.h>
#include <ITG3200.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>
#include <L3G.h>
#include <LPS331.h> 
#include <RunningAverage.h>
#include <iCompass.h>
#include <AP_Math_freeimu.h>
#include <Filter.h>    // Filter library
#include <Butter.h>    // Butterworth filter
#include <FilteringScheme.h>
//#define DEBUG
#include "DebugUtils.h"
#include "FreeIMU.h"

// My headers
#include <ros.h>
#include <ros/time.h>
#include "IMU_broadcaster.h"
#include "globleVairbles.h"
#include "PositionManageSVO.h"
#include "MyMatMath.h"
#include "ESC_PWM.h"
#include "spektrumSatil.h"
#include "AccelController.h"
#include "PositionController.h"
#include "AttitudeController.h"
#include "myFreeIMU.h"
#include "COM.h"
#include "selfieManager.h"

//____________________________
// SETUP FUNCTION
void setup() { 
  XBeeSerial      ->begin(57600);
  SpectrumSerial  ->begin(115200);
  FlowSerial      ->begin(19200);
  Serial.          begin(57600); // this was ignored by the teensy board (it always runs at 119200)
  
  // LED indicator.
  pinMode(13, OUTPUT); 
 
  // Initialise IMU
  Wire.begin();
  delay(5);
  my3IMU.init(1); // the parameter enable or disable fast mode
  delay(5);

  // Initialise the motors
  motorInit();
  
  // Initialise ROS interface
  nh.initNode();
  ros_listener.init(nh);
  imu_broadcaster.init(nh);

  pinMode(slamLED, OUTPUT);
  analogWrite(slamLED, 0);

  // Initialise User onboard input
  pinMode(selfie_pin, INPUT); // sets the digital pin as output

  // initialise delay compensator for vision velocity measurment
  init_vision_velo_delayCompensator();
}


//_____________________________
// MAIN LOOP
uint32_t main_timer = micros();
uint32_t time_logger = 0;
uint8_t main_counter = 0;
uint8_t LED = 1;

String inString = "0";  // tobe deleted

void loop() {
   // Timer
   float timeNow = micros();
   float timeChange = timeNow - main_timer;
   if (timeChange < 0) return;

   // Main Control loop
   if (timeChange >= SAMPLING_RATE) {
     // Timer update
     main_timer = timeNow;
     time_logger += timeChange;
     main_counter++;

     
     //################################
     // Manual Control UPDATE
     //################################
     
     //________________________________
     // Smooth vision transation
     vision_smoothTrans(); // update the newMap counter (find me in PositionController.h)
     
     //______________________________________
     // Get Manual Control command (50 Hz, now has been filtered to 400 Hz)
     boolean RCNewData = read_receiver(SpectrumSerial);
     filterRC();// this boost the update rate to main loop 400 Hz

     // ######### Automatic control #########
     if(enableControl){
         selfie_process_RC(timeChange);
         
     // ########### Manual control ###########
     }else{
         convert_RC_accel();
         filter_accel_d();
         attach_acceleration_controller();
         // Convert accel to attitude and thrust
         accel_2_attitudeThrust();
     }

     
     //################################
     // Position and velocity UPDATE
     //################################
     //______________________________________
     // Read Vision measurement frome ROS
     nh.spinOnce();
     //complementaryFilter3D(vVision_cf, vVision, acce+1, kp_p[1], timeChange/1000000.0); // Fast velocity measurement fusing with accel. (loopTime in s)

     //################################
     // Outer Loop Control
     //################################
     //____________________
     // Attach Outer loop
     if(enableControl){
       selfie_process_OUTTER();
     }  
     
     
     //################################
     // IMU UPDATE
     //################################
     //______________________________________
     // Read IMU fusion data
     read_IMU(timeChange);
     
     
     //################################
     // AUTOPILOT UPDATE
     //################################
     // only update controller when loop is in reasonable speed 
     if (timeChange < 3*SAMPLING_RATE) { 
      
       //______________________________________
       // Not ARMED: motor stop
       if (!armed) {
         motorsStop();
         detach_attitude_controller();
         detach_position_controller();
         reset_selfie_command();
       }
     
       // Armed but not on the air: motors minimum spin, while controller disabled
       else if (armed && !onAir) {
         motorsNeutr();
         detach_attitude_controller();
         detach_position_controller();
         reset_selfie_command();
       }
     
       // Armed and on the air: controller and motors fully functioning with RC
       else if (armed && onAir && !enableControl) {
         attach_attitude_controller(timeChange);
         apply_body_inertia();
         CommandQuad250();
         detach_position_controller();
       }
     
       // automatic controller takes over
       else if (armed && onAir && enableControl) {
         selfie_process_INNER(timeChange);
       }
       
     // log the skipped loop
     } else {
       rosStatus = timeChange;
     }

     //_________________
     // update vision delay compensation buffer
     //_________________
     update_vision_velo_delayCompensator(acce+1, timeChange/1000000.0); // see PositionManageSVO.h

     
     // ########################
     // 100 Hz 
     // ########################
     if (main_counter % (LOOP_HZ/200) == 0 && initialised){
       // Broadcast IMU data to ROS
       if (armed){
         imu_broadcaster.sendTransform_teensy(nh, q, acce); // sent unfiltered acceleration
       } else {
         imu_broadcaster.sendTransform_teensy_freeExposure(nh, q, acce); // sent unfiltered acceleration
       }
       //XBeeSerial->print(pVision[0]); XBeeSerial->print(' '); XBeeSerial->print(pVision[1]); XBeeSerial->print(' '); XBeeSerial->println(pVision[2]);
     }
     
     
     //################################
     // COM UPDATE
     //################################
     //________________________________________
     // Read Gains and Broadcast
     if (Read_gains(XBeeSerial)) {//
       Boardcast_Gains(XBeeSerial);
     }
     if (main_counter % (LOOP_HZ/20) == 0){//######### 20 Hz ############
       Boardcast_status(XBeeSerial, timeChange);
       
       // update user selfie button status
       update_selfie_command();
       
       // Update SLAM indicator LED
       if(vision_available_==1) {
         analogWrite(slamLED, 8191);
       } else if (vision_available_ <= 3) {
         analogWrite(slamLED, 500);
       } else if (vision_available_ == 4) {
         analogWrite(slamLED, 0);
       }
       //if(rosStatus==-2) analogWrite(slamLED, 0);
     }
     
     
     //################################
     // LED and IMU initialisation ################# 2 Hz #############
     //################################
     if (main_counter == (LOOP_HZ/2)){
       LED = -LED;
       if(LED == 1){
         digitalWrite(13, LOW);
         if (!initialised){
           if((R[2][2]-lastR22)>-0.001 && (R[2][2]-lastR22)<0.001) initCount++;
           else initCount == 0;
           // if converged, then set initflag, and start to estimate gyro drift!!!
           if (initCount> 3) {
            initialised = 1;
            my3IMU.setExlong();
            //my3IMU.initialised = 0; // reset altitude estimator
           }
           digitalWrite(13, HIGH);
           lastR22 = R[2][2];
         }
       }else{
         digitalWrite(13, HIGH);
       }
       
       // loop Hz check, average over 0.5 second
       STATUS.v.LoopHz = ((float)(LOOP_HZ/2)*1000000.0 / time_logger);
       //XBeeSerial->println(STATUS.v.LoopHz);
       time_logger  = 0;
       main_counter = 0;
     }
   }
}
