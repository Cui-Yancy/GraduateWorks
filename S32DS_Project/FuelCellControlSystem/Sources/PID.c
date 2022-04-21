#include "PID.h"
#include <math.h>
#include "rtos.h"

//PID参数初始化
void PID_Init(PID* PID_data, float Kp, float Ki, float Kd)
{
	//PID_data->Target = 0;
	PID_data->Err = 0;
	PID_data->Err_last = 0;
	PID_data->Err_blast = 0;
    PID_data->ErrSum = 0;
	PID_data->Kp = Kp;
	PID_data->Ki = Ki;
	PID_data->Kd = Kd;
}

//PID增量计算
double PID_Calc_Inc(PID* PID_data, double AimTem, double Tem_Now) {
    double incrementTem;
    //PID_data->Target = AimTem;
    PID_data->Err = Tem_Now - AimTem;
    incrementTem =      PID_data->Kp * (PID_data->Err - PID_data->Err_last)
                    +   PID_data->Ki * PID_data->Err
                    +   PID_data->Kd * (PID_data->Err - 2 * PID_data->Err_last + PID_data->Err_blast);
    PID_data->Err_blast = PID_data->Err_last;
    PID_data->Err_last = PID_data->Err;
    return incrementTem;
}


double PID_Calc(PID* PID_data, double AimTem, double Tem_Now) {
	double PWM_out;
	//static double ErrSum = 0;
	//PID_data->Target = AimTem;

    PID_data->Err = Tem_Now - AimTem;
	PID_data->ErrSum += PID_data->Err;
	PWM_out = PID_data->Kp * PID_data->Err
		+ PID_data->Ki * PID_data->ErrSum
		+ PID_data->Kd * (PID_data->Err - PID_data->Err_last);
	PID_data->Err_last = PID_data->Err;
	return PWM_out;
}

double AimTemCompute(double I) {
	return (-0.0007515*pow(I,3)+0.03901*pow(I,2)+0.18814*I+31.91429);
}

