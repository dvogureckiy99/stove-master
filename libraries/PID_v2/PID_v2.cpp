/**********************************************************************************************
* Arduino PID Library - Version 1.2.1
* by  Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License
**********************************************************************************************/
	
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <PID_v2.h>

/*Constructor (...)*********************************************************
*    The parameters specified here are those for for which we can't set up
*    reliable defaults, so we need to have the user set them.
***************************************************************************/
PID::PID(unsigned int* Input, int* Output, unsigned int* Setpoint,
	double Kp, double Ki, byte POn, byte ControllerDirection)
{
	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	inAuto = false;

	PID::SetOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

	

	PID::SetControllerDirection(ControllerDirection);
	PID::SetTunings(Kp, Ki, POn);

	
}

/*Constructor (...)*********************************************************
*    To allow backwards compatability for v1.1, or for people that just want
*    to use Proportional on Error without explicitly saying so
***************************************************************************/

PID::PID(unsigned int* Input,  int* Output, unsigned int* Setpoint,
	double Kp, double Ki, byte ControllerDirection)
	:PID::PID(Input, Output, Setpoint, Kp, Ki, P_ON_E, ControllerDirection)
{

}


/* Compute() **********************************************************************
*     This, as they say, is where the magic happens.  this function should be called
*   every time "void loop()" executes.  the function will decide for itself whether a new
*   pid Output needs to be computed.  returns true when the output is computed,
*   false when nothing has been done.
**********************************************************************************/
bool PID::Compute()
{
	if (!inAuto) return false;
		/*Compute all the working error variables*/
		unsigned int input = *myInput;
		int error = *mySetpoint - input; //расчёт ошибки
		if(lastInput != 0)
		dInput = (input - lastInput); //расчёт производной, не делится ни на что, т.к. шаг учитывается в коэффициенте
		outputSum += (ki * error); //интегральная составляющая выхода (выход интегратора)
		
		/*Add Proportional on Measurement, if P_ON_M is specified*/
		if (!pOnE) outputSum -= kp * dInput;

		if (outputSum > outMax) outputSum = (int)outMax;
		else if (outputSum < outMin) outputSum = (int) outMin;

		/*Add Proportional on Error, if P_ON_E is specified*/
		int output;
		if (pOnE) output = kp * error;
		else output = 0;
        
		/*Compute Rest of PID Output*/
		output += outputSum ;

		if (output > outMax) output = (int)outMax;
		else if (output < outMin) output = (int)outMin;
		if(controllerDirection)
		{
			output = 1023 - output ; //так как у нас при большей скважность энергии поступает меньше
		}
		*myOutput = output;
		
		/*Remember some variables for next time*/
		lastInput = input;
		return true;	
}

/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted.
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/
void PID::SetTunings(double Kp, double Ki, byte POn)
{
	if (Kp<0 || Ki<0 ) return;

	pOn = POn;
	pOnE = POn == P_ON_E;

	dispKp = Kp; dispKi = Ki; 

	
	kp = Kp;
	ki = Ki * SampleTime;
	

	
}

/* SetTunings(...)*************************************************************
* Set Tunings using the last-rembered POn setting
******************************************************************************/
void PID::SetTunings(double Kp, double Ki) {
	SetTunings(Kp, Ki, pOn);
}


/* SetSampleTime(...) *********************************************************
* sets the period, in Milliseconds, at which the calculation is performed
******************************************************************************/
void PID::SetSampleTime(byte NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		double ratio = (double)NewSampleTime
			/ (double)SampleTime;
		ki *= ratio;
		SampleTime = (byte)NewSampleTime;
	}
}

/* SetOutputLimits(...)****************************************************
*     This function will be used far more often than SetInputLimits.  while
*  the input to the controller will generally be in the 0-1023 range (which is
*  the default already,)  the output will be a little different.  maybe they'll
*  be doing a time window and will need 0-8000 or something.  or maybe they'll
*  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
*  here.
**************************************************************************/
void PID::SetOutputLimits(int Min, int Max)
{
	if (Min >= Max) return;
	outMin = Min;
	outMax = Max;

	if (inAuto)
	{
		if (*myOutput > outMax) *myOutput = outMax;
		else if (*myOutput < outMin) *myOutput = outMin;

		if (outputSum > outMax) outputSum = outMax;
		else if (outputSum < outMin) outputSum = outMin;
	}
}

/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/
void PID::SetMode(byte Mode)
{
	bool newAuto = (Mode == AUTOMATIC); 
	if (newAuto && !inAuto)
	{  /*we just went from manual to auto*/
		PID::Initialize();
	}
	inAuto = newAuto;
}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/
void PID::Initialize()
{
	outputSum = *myOutput;
	lastInput = *myInput;
	if (outputSum > outMax) outputSum = outMax;
	else if (outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
* The PID will either be connected to a DIRECT acting process (+Output leads
* to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
* know which one, because otherwise we may increase the output when we should
* be decreasing.  This is called from the constructor.
******************************************************************************/
void PID::SetControllerDirection(byte Direction)
{
	controllerDirection = Direction;
}

/* Status Funcions*************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened.  these
* functions query the internal state of the PID.  they're here for display
* purposes.  this are the functions the PID Front-end uses for example
******************************************************************************/
double PID::GetKp() { return  dispKp; }
double PID::GetKi() { return  dispKi; }
int PID::GetMode() { return  inAuto ? AUTOMATIC : MANUAL; }
int PID::GetDirection() { return controllerDirection; }
int PID::GetdInput() { return dInput; }

