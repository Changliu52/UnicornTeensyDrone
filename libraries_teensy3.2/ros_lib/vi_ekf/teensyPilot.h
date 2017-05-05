#ifndef _ROS_vi_ekf_teensyPilot_h
#define _ROS_vi_ekf_teensyPilot_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace vi_ekf
{

  class teensyPilot : public ros::Msg
  {
    public:
      float qw;
      float qx;
      float qy;
      float qz;
      float px;
      float py;
      float pz;
      float vx;
      float vy;
      float vz;
      float bx;
      float by;
      float bz;
      float lambda;
      int8_t status;

    teensyPilot():
      qw(0),
      qx(0),
      qy(0),
      qz(0),
      px(0),
      py(0),
      pz(0),
      vx(0),
      vy(0),
      vz(0),
      bx(0),
      by(0),
      bz(0),
      lambda(0),
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_qw;
      u_qw.real = this->qw;
      *(outbuffer + offset + 0) = (u_qw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qw);
      union {
        float real;
        uint32_t base;
      } u_qx;
      u_qx.real = this->qx;
      *(outbuffer + offset + 0) = (u_qx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qx);
      union {
        float real;
        uint32_t base;
      } u_qy;
      u_qy.real = this->qy;
      *(outbuffer + offset + 0) = (u_qy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qy);
      union {
        float real;
        uint32_t base;
      } u_qz;
      u_qz.real = this->qz;
      *(outbuffer + offset + 0) = (u_qz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qz);
      union {
        float real;
        uint32_t base;
      } u_px;
      u_px.real = this->px;
      *(outbuffer + offset + 0) = (u_px.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_px.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_px.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_px.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->px);
      union {
        float real;
        uint32_t base;
      } u_py;
      u_py.real = this->py;
      *(outbuffer + offset + 0) = (u_py.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_py.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_py.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_py.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->py);
      union {
        float real;
        uint32_t base;
      } u_pz;
      u_pz.real = this->pz;
      *(outbuffer + offset + 0) = (u_pz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pz);
      union {
        float real;
        uint32_t base;
      } u_vx;
      u_vx.real = this->vx;
      *(outbuffer + offset + 0) = (u_vx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vx);
      union {
        float real;
        uint32_t base;
      } u_vy;
      u_vy.real = this->vy;
      *(outbuffer + offset + 0) = (u_vy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vy);
      union {
        float real;
        uint32_t base;
      } u_vz;
      u_vz.real = this->vz;
      *(outbuffer + offset + 0) = (u_vz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vz);
      union {
        float real;
        uint32_t base;
      } u_bx;
      u_bx.real = this->bx;
      *(outbuffer + offset + 0) = (u_bx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bx);
      union {
        float real;
        uint32_t base;
      } u_by;
      u_by.real = this->by;
      *(outbuffer + offset + 0) = (u_by.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_by.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_by.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_by.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->by);
      union {
        float real;
        uint32_t base;
      } u_bz;
      u_bz.real = this->bz;
      *(outbuffer + offset + 0) = (u_bz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bz);
      union {
        float real;
        uint32_t base;
      } u_lambda;
      u_lambda.real = this->lambda;
      *(outbuffer + offset + 0) = (u_lambda.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lambda.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lambda.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lambda.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lambda);
      union {
        int8_t real;
        uint8_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_qw;
      u_qw.base = 0;
      u_qw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->qw = u_qw.real;
      offset += sizeof(this->qw);
      union {
        float real;
        uint32_t base;
      } u_qx;
      u_qx.base = 0;
      u_qx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->qx = u_qx.real;
      offset += sizeof(this->qx);
      union {
        float real;
        uint32_t base;
      } u_qy;
      u_qy.base = 0;
      u_qy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->qy = u_qy.real;
      offset += sizeof(this->qy);
      union {
        float real;
        uint32_t base;
      } u_qz;
      u_qz.base = 0;
      u_qz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->qz = u_qz.real;
      offset += sizeof(this->qz);
      union {
        float real;
        uint32_t base;
      } u_px;
      u_px.base = 0;
      u_px.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_px.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_px.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_px.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->px = u_px.real;
      offset += sizeof(this->px);
      union {
        float real;
        uint32_t base;
      } u_py;
      u_py.base = 0;
      u_py.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_py.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_py.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_py.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->py = u_py.real;
      offset += sizeof(this->py);
      union {
        float real;
        uint32_t base;
      } u_pz;
      u_pz.base = 0;
      u_pz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pz = u_pz.real;
      offset += sizeof(this->pz);
      union {
        float real;
        uint32_t base;
      } u_vx;
      u_vx.base = 0;
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vx = u_vx.real;
      offset += sizeof(this->vx);
      union {
        float real;
        uint32_t base;
      } u_vy;
      u_vy.base = 0;
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vy = u_vy.real;
      offset += sizeof(this->vy);
      union {
        float real;
        uint32_t base;
      } u_vz;
      u_vz.base = 0;
      u_vz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vz = u_vz.real;
      offset += sizeof(this->vz);
      union {
        float real;
        uint32_t base;
      } u_bx;
      u_bx.base = 0;
      u_bx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bx = u_bx.real;
      offset += sizeof(this->bx);
      union {
        float real;
        uint32_t base;
      } u_by;
      u_by.base = 0;
      u_by.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_by.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_by.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_by.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->by = u_by.real;
      offset += sizeof(this->by);
      union {
        float real;
        uint32_t base;
      } u_bz;
      u_bz.base = 0;
      u_bz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bz = u_bz.real;
      offset += sizeof(this->bz);
      union {
        float real;
        uint32_t base;
      } u_lambda;
      u_lambda.base = 0;
      u_lambda.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lambda.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lambda.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lambda.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lambda = u_lambda.real;
      offset += sizeof(this->lambda);
      union {
        int8_t real;
        uint8_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return "vi_ekf/teensyPilot"; };
    const char * getMD5(){ return "f7c0a5df511c8bd0f4fea596f54972b0"; };

  };

}
#endif