
#include <uWS/uWS.h>
#include "PID.h"
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	/*Set Kp, Ki and Kd to initial values passed by controller. These are passed from main*/
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	/*Set up inital p, i and d error to zero.*/
	p_error = 0;
	i_error = 0;
	d_error = 0;

}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	return ((-Kp * p_error) - (Ki * i_error) - (Kd * d_error));
}
