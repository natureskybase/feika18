/*
 * @Author: Ptisak
 * @Date: 2022-04-07 14:03:50
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2023-03-19 23:46:57
 */
#ifndef __pid_H__
#define __pid_H__

//#include "stdint.h"
#include "common.h"

enum
{
  LLAST = 0,
  LAST,
  NOW,
  POSITION_PID,
  DELTA_PID,
};
typedef struct
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3];

  float pout;
  float iout;
  float dout;
  float out;

  float input_max_err;   // input max err;
  float output_deadband; // output deadband;

  uint32 pid_mode;
  uint32 max_out;
  uint32 integral_limit;

  void (*f_param_init)(struct pid_t_ *pid,
                       uint32 pid_mode,
                       uint32 max_output,
                       uint32 inte_limit,
                       float p,
                       float i,
                       float d)                    reentrant;
  void (*f_pid_reset)(struct pid_t_ *pid, float p, float i, float d)                 reentrant;

} pid_t_;

#if 0
#define PID_PARAM_DEFAULT \
  {                       \
    0,                    \
        0,                \
        0,                \
        0,                \
        0,                \
        {0, 0, 0},        \
        0,                \
        0,                \
        0,                \
        0,                \
        0,                \
        0,                \
  }\

typedef struct
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3]; //error

  float pout; 
  float iout; 
  float dout; 
  float out;

  float input_max_err;    //input max err;
  float output_deadband;  //output deadband; 

  float p_far;
  float p_near;
  float grade_range;
  
  uint32 pid_mode;
  uint32 max_out;
  uint32 integral_limit;

  void (*f_param_init)(struct pid_t *pid, 
                       uint32      pid_mode,
                       uint32      max_output,
                       uint32      inte_limit,
                       float         p,
                       float         i,
                       float         d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} grade_pid_t;
#endif

void PID_struct_init(
    pid_t_ *pid,
    uint32 mode,
    uint32 maxout,
    uint32 intergral_limit,

    float kp,
    float ki,
    float kd);

float pid_calc(pid_t_ *pid, float get, float set);
float position_pid_calc(pid_t_ *pid, float fdb, float ref);
void ControlLoop(void);
void ControtLoopTaskInit(void);

/* clear integral */
void pid_clear_integral(pid_t_ *pid);


extern pid_t_ pid_left_;
extern pid_t_ pid_right_;
#endif
