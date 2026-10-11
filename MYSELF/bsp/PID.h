#ifndef __PID_H
#define __PID_H    


#include "main.h"

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float Last_error;
	float integration;
}PID;


#endif
