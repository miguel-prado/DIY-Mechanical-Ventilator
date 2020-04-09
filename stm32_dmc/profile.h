#ifndef _VELOCITY_PROFILE_
#define _VELOCITY_PROFILE_

#define TS	0.001

#include <Arduino.h>

enum ProfileState{STOPPED, RUNNING};

class Profile
{
private:
	long _despos;
	long _desvel;
	double acc;
	double T;
	double alpha;
	double theta[2];
	int k, period;
  double c3, c2, c1, c0;
	ProfileState state;
public:
	Profile(void);
	~Profile(void);
  void abort(void);
	void setStep(long);
	void execute(void);
	void startMotion(long);
	void setPosition(long);
	void setVelocity(long);
	float position(void) const;
  bool isMoving(void) const;
};

#endif//_VELOCITY_PROFILE_
