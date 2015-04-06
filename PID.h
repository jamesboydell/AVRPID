/*
 * PID.h
 *
 * Created: 2015-02-18 5:14:58 PM
 *  Author: Jamie
 */ 


#ifndef PID_H_
#define PID_H_

/********************************************************
* Defines and structures
********************************************************/

/*************************************************************
PID Structure
This structure hold all the information used for the PID controller
Set point and process value are pointers. This is because these values
are likely set elsewhere, like within an ISR.
*************************************************************/
typedef struct
{
	uint8_t * sp; //pointer to unsigned integer set point value
	uint8_t * pv; //pointer to unsigned integer process value
	float output;
	float Kp;
	float Ki;
	float Kd;
	float err;
	float prev_err;
	float iTerm;
	float dTerm;
	float pTerm;
	float dt;
}PID;

/********************************************************
* Function Prototypes
********************************************************/
void calc_output(PID  * pPID);
void init_PID(PID * pPID, volatile uint8_t * setpoint,volatile uint8_t * measured_value);
void calc_error(PID * pPID);
void calc_integral(PID * pPID);
void calc_derivative(PID * pPID);
void set_PID_tuning(PID * pPID, float pGain, float iGain, float dGain, float dTime);

#endif /* PID_H_ */