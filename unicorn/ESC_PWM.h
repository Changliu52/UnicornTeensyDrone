//©2016 Chang Liu
#ifndef __ESCPWM_H__
#define __ESCPWM_H__

#include <Arduino.h>
/*  ESC / Servo signal generation done in hardware by FLEX timer, without ISR (yay!)

    We are using flex timer0 which supports 8 channels.

    Currently generating 400 Hz PWM signal that is fed into electronic speed controllers
    corresponding to each rotor.

    This code will probably be expanded to also generate servo signal for
    gimbal stabilization (hopefully in near future).

    Big thanks to kha from #aeroquad for helping me get this up and running.
   
    == PIN Configuration ==
   
    Channel - PIN name - Teensy 3.0 PIN numbering

    FTM0_CH0 - PTC1 - 22 - m1
    FTM0_CH1 - PTC2 - 23 - m2
    FTM0_CH2 - PTC3 - 9  - Not used
    FTM0_CH3 - PTC4 - 10 - Not used
    FTM0_CH4 - PTD4 - 6  - m5
    FTM0_CH5 - PTD5 - 20 - m3
    FTM0_CH6 - PTD6 - 21 - m4
    FTM0_CH7 - PTD7 - 5  - m6
*/
#define M0 22
#define M1 23
#define M2 20
#define M3 21

#define maxPWM 8191.0


//#######################################
void motorsStop()
{
    analogWrite(M0, 10);
    analogWrite(M1, 10);
    analogWrite(M2, 10);
    analogWrite(M3, 10);
}

void motorsNeutr()
{
    analogWrite(M0, 400);
    analogWrite(M1, 400);
    analogWrite(M2, 400);
    analogWrite(M3, 400);
}


void motorInit()
{
  analogWriteFrequency(M0, 1000);//5859Hz , this set all the pin on this timer
  analogWriteResolution(13); // 0-8191!!! -> 0-100%PWM
  motorsStop();
}

void motors_update(uint16_t* MotorOut) //MotorOut in us
{
    analogWrite(M0, MotorOut[0]);
    analogWrite(M1, MotorOut[1]);
    analogWrite(M2, MotorOut[2]);
    analogWrite(M3, MotorOut[3]);
}


//__________________________________________________--
// Convert duty cycle% to pwm value, according to the Motor pwm resolution
void moveServo(float* dutycycles)
{
    #define minDuty 0 
    #define maxDuty 100
    
    uint16_t motorPWM[4];
    for (int i=0; i<4; i++){
      // bracket
      if (dutycycles[i]<minDuty) {dutycycles[i]=minDuty;}
      else if (dutycycles[i]>maxDuty) {dutycycles[i]=maxDuty;}
      // convert to duty cycle
      motorPWM[i] = (uint16_t)((dutycycles[i]-minDuty)*maxPWM/(maxDuty-minDuty)); //Map the duty cycle to the corresponding PWM pulse.
    }
    //Serial.println(motorPWM[0]);
    motors_update(motorPWM);
}



//######################################
//________________________________
// convert motor value from command to thrust value (lineariser)
// Remember to change maxthrust in globleVairbles.h 
void thrust2throttle250_PWM_CLOSELoop (float* thrust, float* PWMout) {
    for (int i=0; i<4; i++) {
        //PWMout[i] = 2.71*sqrt(thrust[i])-0.009837; // elite
        PWMout[i] = 2.262*sqrt(thrust[i])+2.575; // AirGear first 4:13 measurements
        if(PWMout[i]>50.0) PWMout[i]=50.0;
    }
}
#endif
