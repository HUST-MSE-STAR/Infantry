#ifndef ALGORITHM_LIB_H
#define ALGORITHM_LIB_H

#include "main.h"
#include "string.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
/* ----------------------- ramp_algorithm----------------------------- */
typedef __packed struct
{
    fp32 input;        
    fp32 out;          
    fp32 min_value;    
    fp32 max_value;    
    fp32 frame_period; 
} ramp_function_source_t;
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

/* ----------------------- first_order_filter_algorithm----------------------------- */
typedef __packed struct
{
    fp32 input;        
    fp32 out;          
    fp32 num[1];       
    fp32 frame_period; 
} first_order_filter_type_t;
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, fp32 num[1]);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);

/* ----------------------- PID_algorithm----------------------------- */
typedef struct
{
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  
    fp32 max_iout; 

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  
    fp32 error[3]; 

} pid_type_def;
void pid_init(pid_type_def *pid, fp32 KP, fp32 KI, fp32 KD, fp32 max_iout, fp32 max_out);
extern fp32 pid_calc(pid_type_def *pid, fp32 ref, fp32 set);
/* ----------------------- other_algorithm----------------------------- */
fp32 LIMIT_MIN_MAX(fp32 Value, fp32 minValue, fp32 maxValue);
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
fp32 float_abs(fp32 Value);
#endif
