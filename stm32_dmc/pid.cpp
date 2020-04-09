#include "pid.h"

PID_Controller::PID_Controller(long *reference, long *response)
{
	ref = reference;
	res = response;
	enabled = true;
	_Kp = 0.1211;
	_Ti = 1.0;
	_Td = 0.0046;
	sumInt = 0.0;
  pre_error = 0;
	theta[0] = 0;
	theta[1] = 0;
	omega[0] = 0.0;
	omega[1] = 0.0;
}

PID_Controller::~PID_Controller(void)
{
}

void PID_Controller::setEnable(const bool value)
{
	enabled = value;
}

void PID_Controller::compute(void)
{
	/* Compute velocity */
	theta[1] = theta[0];
	theta[0] = *res;
	omega[1] = omega[0];
	omega[0] = q0 * (theta[0] - theta[1]) - q1 * omega[1];
	
	if (!enabled)
	{
		sumInt = 0.0;
    _output = 0.0;
		return;
	}
	_error = *ref - *res;
	double up = _Kp * _error;
	double ui = _Kp * (Ts/_Ti) * _error;
	sumInt = sumInt + ui;
	if (sumInt > IMAX)
		sumInt = IMAX;
	if (sumInt < IMIN)
		sumInt = IMIN;
	double ud = -_Kp * _Td * omega[0];
  //double ud = _Kp * (_Td/Ts) * (theta[1] - theta[0]);
  //double ud = _Kp * (_Td/Ts) * (_error - pre_error);
  pre_error = _error;
	_output = up + sumInt + ud;
	if (_output > UMAX)
		_output = UMAX;
	if (_output < UMIN)
		_output = UMIN;
}

void PID_Controller::setKp(const double kp)
{
	_Kp = kp;
}

void PID_Controller::setTi(const double ti)
{
	_Ti = ti;
}

void PID_Controller::setTd(const double td)
{
	_Td = td;
}

double PID_Controller::Kp(void) const
{
	return _Kp;
}

double PID_Controller::Ti(void) const
{
	return _Ti;
}

double PID_Controller::Td(void) const
{
	return _Td;
}

double PID_Controller::error(void) const
{
	return _error;
}

double PID_Controller::output(void) const
{
	return _output;
}

double PID_Controller::velocity(void) const
{
	return omega[0];
}

void PID_Controller::reset(void)
{
  sumInt = 0.0;
  pre_error = 0;
  _output = 0.0;
  *ref = 0;
  *res = 0;
  
  _error = 0;
  theta[0] = 0.0;
  theta[1] = 0.0;
  omega[0] = 0.0;
  omega[1] = 0.0;
}
