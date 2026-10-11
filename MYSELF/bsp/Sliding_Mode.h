#ifndef __SLIDING_MODE_H
#define __SLIDING_MODE_H




typedef struct {
	float Kj;//转动惯量
	float Kc;//ce的常量c
	float Kk;//ks的常量k
	float Km;//sign(s)的常数
}Sliding_Yaw_Mode;

typedef struct {
	float Kj;         // 转动惯量
	float Kc1;        // c1
	float Kc2;        // c2
	float Kk;         // k
	float Km;         // epsilon
	float integral_e; // integral(e dt)
} Sliding_Pitch_Mode;


extern Sliding_Yaw_Mode sliding_mode;

void Sliding_Yaw_Mode_Init(Sliding_Yaw_Mode* mode, float Kj, float Kc, float Kk, float Km);
float Sliding_Yaw_Mode_Calculation(Sliding_Yaw_Mode* mode, float ExAngle, float RealAngle, float ExAngle_double, float RealSpeed);

extern Sliding_Pitch_Mode sliding_pitch_mode;

void Sliding_Pitch_Mode_Init(Sliding_Pitch_Mode* mode, float Kj, float Kc1, float Kc2, float Kk, float Km);
float Sliding_Pitch_Mode_Calculation(Sliding_Pitch_Mode* mode, float ExAngle, float RealAngle, float ExAngle_double, float RealSpeed, float dt);


#endif
