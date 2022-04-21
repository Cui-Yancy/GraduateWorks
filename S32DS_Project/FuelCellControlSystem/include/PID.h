#ifndef PID_H_
#define PID_H_

typedef struct PID{
	//float Target;
	float Err;
	float Err_last;
	float Err_blast;
	float Kp, Ki, Kd;
    double ErrSum;
}PID;

void PID_Init(PID* PID_data, float Kp, float Ki, float Kd);
double PID_Calc_Inc(PID* PID_data, double AimTem, double Tem_Now);
double PID_Calc(PID* PID_data, double AimTem, double Tem_Now);
double AimTemCompute(double I);

#endif /* PID_H_ */
