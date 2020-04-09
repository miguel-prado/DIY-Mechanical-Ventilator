#include "profile.h"

Profile::Profile(void)
{
	k = 0;
	theta[1] = 0.0;
}

Profile::~Profile(void)
{
}

void Profile::abort(void)
{
  state = STOPPED;
  _despos = 0;
  _desvel = 0;
  acc = 0.0;
  T = 0.0;
  alpha = 0.0;
  theta[0] = 0.0;
  theta[1] = 0.0;
  k = period = 0;
  c3 = c2 = c1 = c0 = 0.0;
}

void Profile::setStep(long step)
{
	theta[1] = step;
}

void Profile::execute(void)
{
  float t = k * TS;
	if (state == STOPPED)
		return;
   
  if (k < (period / 3))
    theta[1] = c2 * t * t + theta[0];
  else if (k < (2 * period / 3))
    theta[1] = (c1/3.0)*t - c0 + theta[0];
  else
    theta[1] = -c2*t*t + c1*t - 5*c0 + theta[0];
  
	k = k + 1;
	if (k > period)
		state = STOPPED;
}

void Profile::startMotion(long curpos)
{
	theta[0] = (double)curpos;
  T = 1.5 * (abs(_despos) / (double)_desvel);
  period = int(T / TS);
  float sgn = _despos >= 0.0? 1.0 : -1.0;
  acc  = 3000.0 * sgn * _desvel /(double)period;
  c2 = acc / 2.0;
  c1 = acc * T;
  c0 = acc * T * T / 18.0;
  k = 0;
  theta[0] = theta[1];
	state = RUNNING;
}

void Profile::setPosition(long pos)
{
	_despos = pos;
}

void Profile::setVelocity(long vel)
{
	_desvel = vel;
}

float Profile::position(void) const
{
	return theta[1];
}

bool Profile::isMoving(void) const
{
  if (state == STOPPED)
    return false;
  else
    return true;
}
