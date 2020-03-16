#include "pid_regulator.h"

void PID_Reset(PID_Regulator_t *pid){
	pid->componentKp = 0;
	pid->componentKi = 0;
	pid->componentKd = 0;
	pid->err[0] = 0;
	pid->err[1] = 0;
	pid->output = 0;
}

void PID_Calc(PID_Regulator_t *pid){
	pid->err[1] = pid->err[0];
	pid->err[0] = pid->ref-pid->fdb;
	pid->componentKi += pid->err[0];
	
	if(pid->componentKi < -pid->componentKiMax)
		pid->componentKi = -pid->componentKiMax;
	else if(pid->componentKi > pid->componentKiMax)
		pid->componentKi = pid->componentKiMax;
	
	pid->output = pid->kp*pid->err[0] + pid->ki*pid->componentKi + pid->kd*(pid->err[0]-pid->err[1]);
	
	if ( pid->output > pid->outputMax ){
		pid->output = pid->outputMax;
	}
	else if ( pid->output < -pid->outputMax ){
		pid->output = -pid->outputMax;
	}
}
