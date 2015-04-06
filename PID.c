/*
 * PID.c
 *
 * Created: 2015-02-18 4:26:26 PM
 *  Author: Jamie
 */ 

#include <avr/io.h>
#include "PID.h"

//calculates the output of the control loop
void calc_output(PID * pPID)
{
	calc_error(pPID);
	calc_integral(pPID);
	calc_derivative(pPID);
	pPID->output = (pPID->pTerm + pPID->iTerm + pPID->dTerm);
}

//calculate the error of the control loop
void calc_error(PID * pPID)
{
	pPID->prev_err = pPID->err;				//store the previous error
	pPID->err = (*pPID->sp) - (*pPID->pv);		//calculate new error
	pPID->pTerm = (pPID->err) * (pPID->Kp);		//calculate proportion term
}

//calculate the integral term of the control loop
void calc_integral(PID * pPID)
{
	pPID->iTerm = (pPID->Ki)*(pPID->iTerm + pPID->err*pPID->dt); //calculate the integral term
}

//calculate the derivative term of the control loop
void calc_derivative(PID * pPID)
{
	pPID->dTerm = (pPID->Kd)*((pPID->err - pPID->prev_err)/pPID->dt);
}

/**********************************************************************
* This function sets the pointers to the setpoint and process variable
* Pointers are used because these values will most likely be set inside
* an ISR 
**********************************************************************/
void init_PID(PID * pPID,volatile uint8_t * setpoint,volatile uint8_t * measured_value)
{
	pPID->Kp = 0;
	pPID->Ki = 0;
	pPID->Kd = 0;
	pPID->err = 0;
	pPID->prev_err = 0;
	pPID->pTerm = 0;
	pPID->iTerm = 0;
	pPID->dTerm = 0;
	pPID->dt = 0;
	pPID->sp = setpoint;
	pPID->pv = measured_value;
	pPID->output = 0;
	pPID->output = 0;
}

void set_PID_tuning(PID * pPID, float pGain, float iGain, float dGain, float dTime)
{
	pPID->Kp = pGain;
	pPID->Ki = iGain; 
	pPID->Kd = dGain; 
	pPID->dt = dTime; 
}