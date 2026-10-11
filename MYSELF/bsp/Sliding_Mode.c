#include "Sliding_Mode.h"
#include "main.h"



Sliding_Yaw_Mode sliding_mode;

void Sliding_Yaw_Mode_Init(Sliding_Yaw_Mode* mode, float Kj, float Kc, float Kk, float Km)
{
	mode->Kj = Kj;
	mode->Kc = Kc;
	mode->Kk = Kk;
	mode->Km = Km;
}

void Sliding_Pitch_Mode_Init(Sliding_Pitch_Mode* mode,float Kj,float Kc1,float Kc2,float Kk,float Km)
{
    mode->Kj = Kj;
    mode->Kc1 = Kc1;
    mode->Kc2 = Kc2;
    mode->Kk = Kk;
    mode->Km = Km;
    mode->integral_e = 0.0f;
}

float Sliding_Yaw_Mode_Calculation(Sliding_Yaw_Mode* mode, float ExAngle, float RealAngle, float ExAngle_double, float RealSpeed)
{
	float e = RealAngle - ExAngle;
	if(e > 180) e -= 360;
	else if(e < -180) e += 360;
	
	float sign_s = 0;
	if ((((mode->Kc) * e + RealSpeed) <1)&&(((mode->Kc) * e + RealSpeed) > -1))
	{
	 sign_s = (mode->Kc) * e + RealSpeed;
	}
	else if (((mode->Kc) * e + RealSpeed) > 1)
	{
	 sign_s = 1;
	}
	else if (((mode->Kc) * e + RealSpeed) < -1)
	{
	 sign_s = -1;
	}

	float out = (mode->Kj * (ExAngle_double - (mode->Kc + mode->Kk) * RealSpeed - (mode->Kc) * (mode->Kk)*(e) - (mode->Km) * sign_s));

	return out;
}

float Sliding_Pitch_Mode_Calculation(Sliding_Pitch_Mode* mode, float ExAngle, float RealAngle, float ExAngle_double, float RealSpeed, float dt)
{
	float e = RealAngle - ExAngle;
	float s;
	float sign_s = 0;

	mode->integral_e += e * dt;
	// 计算s
    s = mode->Kc1 * e
      + RealSpeed
      + mode->Kc2 * mode->integral_e;

	if ((((mode->Kc1) * e + RealSpeed) <1)&&(((mode->Kc1) * e + RealSpeed) > -1))
	{
	 sign_s = (mode->Kc1) * e + RealSpeed;
	}
	else if (((mode->Kc1) * e + RealSpeed) > 1)
	{
	 sign_s = 1;
	}
	else if (((mode->Kc1) * e + RealSpeed) < -1)
	{
	 sign_s = -1;
	}


	float out = mode->Kj * (0.0f- mode->Kc1 * RealSpeed- mode->Kc2 * e- mode->Kk * s- mode->Km * sign_s);

	return out;
}
