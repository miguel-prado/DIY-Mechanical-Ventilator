#ifndef PID_CONTROL_MAP_UAQ
#define PID_CONTROL_MAP_UAQ

#define Ts		0.001
#define q0		95.2381
#define q1		-0.9048
#define UMAX	10.0
#define UMIN	-10.0
#define IMAX	(1.5 * UMAX)
#define IMIN	(1.5 * UMIN)

#include <Arduino.h>

class PID_Controller
{
private:
	bool enabled;
	long *ref;
	long *res;
	long _error;
  long pre_error;
	double _Kp;
	double _Ti;
	double _Td;
	double _output;
  double sumInt;
	long theta[2];
	double omega[2];
public:
	PID_Controller(long *, long *);
	virtual ~PID_Controller(void);
	void compute(void);
	void setEnable(const bool);
	void setKp(const double);
	void setTi(const double);
	void setTd(const double);
	double Kp(void) const;
	double Ti(void) const;
	double Td(void) const;
	double error(void) const;
	double output(void) const;
	double velocity(void) const;
  void reset(void);
};

#endif//PID_CONTROL_MAP_UAQ
