#include "control_algorithm.h"

/**
  * @brief          斜波函数初始化
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，S
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         none
  */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}
/**
  * @brief          斜波函数计算
  * @param[in]      斜波函数结构体
  * @param[in]      斜率
  * @retval         无
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
  * @brief          一阶低通滤波
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，S
  * @param[in]      滤波参数
  * @retval         无
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}
/**
  * @brief          一阶低通滤波计算
  * @param[in]      一阶低通滤波结构体
  * @param[in]      输入值
  * @retval         无
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out 
+ first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}
/**
  * @brief          PID初始化
  * @param[out]     PID结构数据指针
  * @param[in]      KP
  * @param[in]      KI
  * @param[in]      KD
  * @param[in]      PID最大输出
  * @param[in]      PID最大积分输出
  * @retval         none
  */
void pid_init(pid_type_def *pid, fp32 KP, fp32 KI, fp32 KD, fp32 max_iout, fp32 max_out)
{
    pid->Kp = KP;
    pid->Ki = KI;
    pid->Kd = KD;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}
/**
  * @brief          PID计算
  * @param[out]     PID结构数据指针
  * @param[in]      反馈数据
  * @param[in]      设定值
  * @retval         PID输出
  */
fp32 pid_calc(pid_type_def *pid, fp32 set, fp32 get)
{
	if (pid == NULL)
	{
		return 0.0f;
	}
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->set = set;
	pid->fdb = get;
	pid->error[0] = set - get; 
		
	pid->Pout  = pid->Kp * pid->error[0];  
	pid->Iout += pid->Ki * pid->error[0];  
	pid->Dout  = pid->Kd * (pid->error[0] - pid->error[1]);   
	LIMIT_MIN_MAX(pid->Iout, -pid->max_iout, pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;   
	LIMIT_MIN_MAX(pid->out, -pid->max_out, pid->max_out);
		
  return pid->out;
}
/**
  * @brief          限幅函数
  * @param          输入值
  * @param          下限
  * @param          上限
  * @retval         none
  */
fp32 LIMIT_MIN_MAX(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}
/**
  * @brief          循环限幅函数
  * @param          输入值
  * @param          下限
  * @param          上限
  * @retval         none
  */
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

fp32 float_abs(fp32 Value)
{
	if(Value>0)
		return Value;
	else
		return -Value;
	
}








