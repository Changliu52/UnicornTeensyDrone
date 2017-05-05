//©2016 Chang Liu
#ifndef __COM_H__
#define __COM_H__

#include <Arduino.h>

// constants
const uint8_t START_BYTE = 0xF0;
const uint8_t STOP_BYTE  = 0xA5;

// Mode variable
int MODE = 0; //0 - No com, 1 - gain tuning, 2 - status debugging

// Gains var
union{
  struct{
    float PGain[3];
    float IGain[3];
    float DGain[3];
  }
  v;
  uint8_t GAINmessage[36];
}
Gains;


// Status var
union{
  struct{
    float q[4];
    float qD[4];
    
    float currentHeight;
    float desireHeight;
    
    float currentVelox;
    float desireVelox;
    
    float currentVeloy;
    float desireVeloy;
    
    uint32_t LoopHz;
  }
  v;
  uint8_t message[60];
}
STATUS;



//_______________________________________________
// Functions
template <class USerial>
// Helper Function to read control command from PC
boolean read_data(USerial *s, uint8_t message[], uint8_t messID, uint8_t stringLength)
{
  // NB. String length is basically the message + the iD, so mess length plus 1
  if (s->available() >= stringLength+4) {
    
    if (s->read() != START_BYTE) return 0;
    if (s->read() != stringLength) return 0;
    if (s->read() != messID) return 0;
    
    uint8_t checksum = START_BYTE + stringLength + messID;
    
    for (int i = 0; i< stringLength-1; i++) {
      checksum += message[i] = s->read();
    }
    
    if(s->read() != checksum || s->read() != STOP_BYTE) return 0;
    s->flush(); // Avoid Serial Jem
    return 1;
  } else {
    return 0;
  }
}

// Helper Function to read synchronised data from UART
template <class USerial>
boolean read_sync_data(uint8_t message[], uint8_t messID, uint8_t stringLength, USerial *s)
{
  // NB. String length is basically the message + the iD, so mess length plus 1
  if (s->available() >= stringLength+4) {
    
    if (s->read() != START_BYTE) return 0;
    if (s->read() != stringLength) return 0;
    if (s->read() != messID) return 0;
    
    uint8_t checksum = START_BYTE + stringLength + messID;
    
    for (int i = 0; i< stringLength-1; i++) {
      checksum += message[i] = s->read();
    }
    
    if(s->read() != checksum || s->read() != STOP_BYTE) return 0;
    return 1;
  } else {
    return 0;
  }
}


template <class USerial>
// This Helper function is coupled with Serialio function in Matlab
void write_data(uint8_t writeBytes[], uint8_t messID, uint8_t stringLength, USerial *s)
{  
  // NB. String length is basically the message + the iD, so mess length plus 1

  // Write Header
  s->write(START_BYTE);      // 1.These 2 bytes are 'start message'
  s->write(stringLength);    // 2.String length is basically the message + the iD, so mess length plus 1
  s->write(messID);          // 3.From above table

  // Write Message
  uint8_t checksum = START_BYTE+messID+stringLength;
  for (int i=0;i<(stringLength-1);++i) {
    s->write(writeBytes[i]);
    checksum += writeBytes[i];  // Compute checksum 
  }

  // Write Checksum
  s->write(checksum); //
  s->write(STOP_BYTE); //
}





//_______________________________________
// Global Functions
template <class USerial>
// This function is coupled with Serialio function in Matlab
boolean Read_gains(USerial *s)
{
  if(read_data(s, Gains.GAINmessage, 0x01, 0x25)){
    for (int i = 0; i < 3; i++) {
      kp_p[i] = Gains.v.PGain[i];//kp[i]  = Gains.v.PGain[i];//kvp[i]  = Gains.v.PGain[i];//
      kp_i[i] = Gains.v.IGain[i];//ki[i]  = Gains.v.IGain[i];//kvi[i]  = Gains.v.IGain[i];//
      kp_d[i] = Gains.v.DGain[i];//kd[i]  = Gains.v.DGain[i];//kvd[i]  = Gains.v.DGain[i];//
    }
    return 1;
  }else{
    return 0;
  }
}


template <class USerial>
// This function is coupled with Serialio function in Matlab
void Boardcast_Gains(USerial *s)
{
  for (int i = 0; i < 3; i++) {
    Gains.v.PGain[i]    = kp_p[i];//Gains.v.PGain[i]    = kp[i];//kvp[i];//
    Gains.v.IGain[i]    = kp_i[i];//Gains.v.IGain[i]    = ki[i];//kvi[i];//
    Gains.v.DGain[i]    = kp_d[i];//Gains.v.DGain[i]    = kd[i];//kvd[i];//
  }
  write_data(Gains.GAINmessage, 0x01, 0x25, s);
}


template <class USerial>
// This function is coupled with Serialio function in Matlab
void Boardcast_status(USerial *s, float dt)
{
  STATUS.v.q[0] = q[0];
  STATUS.v.q[1] = q[1];
  STATUS.v.q[2] = q[2];
  STATUS.v.q[3] = q[3];
  
  STATUS.v.qD[0] = sqrt(1.0 + Rd[0][0] + Rd[1][1] + Rd[2][2]) / 2.0;
  float w4 = (4.0 * STATUS.v.qD[0]);
  STATUS.v.qD[1] = (Rd[2][1] - Rd[1][2]) / w4;
  STATUS.v.qD[2] = (Rd[0][2] - Rd[2][0]) / w4;
  STATUS.v.qD[3] = (Rd[1][0] - Rd[0][1]) / w4;
  
  /*STATUS.v.qD[0] = qVision[0];//pVision[0];//SVOposition_w[1];//
  STATUS.v.qD[1] = qVision[1];//pVision[1];//SVOposition_w[2];//
  STATUS.v.qD[2] = qVision[2];//pVision[2];//SVOposition_w[3];//
  STATUS.v.qD[3] = qVision[3];//1.0;//accelD_smt[2];*/
  
  STATUS.v.currentHeight = pVision[0];//vVision[1];//lambdaVision;//lambdaVision;//vVision_cf[1];//lambdaVision;//-omegaDelay_w[0];//SVOvelosity[1];//ultrasonic_alti;//flowAltitude;//R[2][2];//flowAltitude;//0;//AltiOutput[1];//flowVeloWorldSmt[0];//DotVeloD[1];//flowVeloWorld[0];//accelD[0];//flowVeloWorld[0];
  STATUS.v.desireHeight  = pVision[1];//vVision_raw[1];//lambdaVision;//vVision_raw[1];//lambdaVision;//lambdaVision;//vVision[1];//accelD_smt[0];//lambdaVision; //ultrasonic_valu;//veloD[2];//veloD[0];//flowVeloWorldSmtOld[0];//flowAltitude;//flowVeloWorldSmt[1];//1000.00/altiPeriod;//DotVeloD[2];//flowVeloWorld[1];//accelD[0];//veloD[2];//flowVeloWorld[1];
  
  STATUS.v.currentVelox   = pVision[2];//pVision[2];//flowVeloWorldSmt[0];
  STATUS.v.desireVelox    = posiD[0];//pVision[0];//acceSmt[0];//rosStatus;//posiD[1];//pVision_cf[2];//veloD[0];
  
  STATUS.v.currentVeloy   = posiD[1];//pVision[1];//(float)vision_available_;//posiD[2];
  if (armed && onAir && enableControl){
    STATUS.v.desireVeloy    = posiD[2];//pVision[2];//1.0;
  }else {
    STATUS.v.desireVeloy    = 0.0;
  }
  
  write_data(STATUS.message, 0x02, 0x3D, s);
}

#endif
