//©2016 Chang Liu
#ifndef __SELFIE_H__
#define __SELFIE_H__

#include <Arduino.h>

//##############
// function to update selfie button command
//##############
void update_selfie_command()
{
  int timeNow = millis();
  
  // button pressed
  if(digitalRead(selfie_pin) == HIGH){
    // button hold for more than 1 seconds
    if(timeNow-selfie_timer > 1000){
      selfie_timer = timeNow; // reset timer
      // change state
      if (vision_available_ !=1 && selfie_state ==selfieDISARM) return;
      selfie_state++;
      selfie_state = selfie_state%3;
    }
    
  // button NOT pressed
  } else {
    selfie_timer = timeNow; // reset timer
  }
}

//______________
// Reset selfie Command
void reset_selfie_command()
{
  selfie_timer = millis(); // reset timer
  selfie_state = selfieDISARM; // disARM quad
}


//######################
// functions to process selfie command
//######################
void selfie_process_RC(float timeChange)
{
  if (selfie_state == selfieACTIVE){
    auto_control_process_RC(timeChange);
  }
}

void selfie_process_OUTTER()
{
  if (selfie_state == selfieACTIVE){
    auto_control_process_OUTTER();
  }
}

void selfie_process_INNER(float timeChange)
{
  // control activated
  if (selfie_state == selfieACTIVE) {
    auto_control_process_INNER(timeChange);
    
  // disARMed 
  } else if (selfie_state == selfieDISARM) {
    motorsStop();
    detach_attitude_controller();
    detach_position_controller();
    
  // motors minmum spin
  } else if (selfie_state == selfieMINSPIN) {
    motorsNeutr();
    detach_attitude_controller();
    detach_position_controller();
  }
}
#endif
