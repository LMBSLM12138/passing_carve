#include "PID.h"

void PID_Init(PID* pid,float Kp,float Ki,float Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Last_error = 0;
	pid->integration = 0;
}

float PID_Calculation(PID* pid, float ExSpeed, float RealSpeed)
{
	float error = ExSpeed - RealSpeed;
	pid->integration += error;
	//积分限幅
	if (pid->integration >  1000) pid->integration =  1000;
  if (pid->integration < -1000) pid->integration = -1000;
	
	float derivative = error - pid->Last_error;
	pid->Last_error = error;
	float out = pid->Kp * error + pid->Ki * pid->integration + pid->Kd * derivative;
	return out;
}

float PID_Calculation_Angle(PID* pid, float ExSpeed, float RealSpeed)
{
		float error = ExSpeed - RealSpeed;
		if (error > 180.0f) {
				error -= 360.0f;
		}
		if (error < -180.0f) {
				error += 360.0f;
		}
		pid->integration += error;
		//积分限幅
		if (pid->integration >  1000) pid->integration =  1000;
		if (pid->integration < -1000) pid->integration = -1000;

		float derivative = error - pid->Last_error;
		pid->Last_error = error;
		float out = pid->Kp * error + pid->Ki * pid->integration + pid->Kd * derivative;
		return out;
}



float clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}
