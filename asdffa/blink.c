#include <msp430.h>
#include <msp430g2553.h>

/*
 * PWM SIGNAL OUTPUT PINS
 * P2.0 - STEERING
 * P2.1 - MOTOR
 * P2.4 - CAMERA
 */

// Function headers
void resetRegisters();
void steer(int direction, int duration);
void accelerate(int direction, int duration);
void pan(int direction, int duration);
void updateSteering();
void updateAcceleration();
void updateCamera();

#define CS(x) _DINT(); x; _EINT();

// Global constants
const unsigned int PWM_FREQ_COUNT = 32670; // TACCRx Count
const int PWM_FREQ = 45; // Freq in Hz.

const unsigned int STEERING_MAX_COUNT = 2647;	// Max Pon count for steering
const unsigned int STEERING_NEUT_COUNT = 2042;	// Neutral Pon count for steering
const unsigned int STEERING_MIN_COUNT = 1437;	// Min Pon count for steering
const unsigned int STEERING_INIT_DUTYRATIO = 1437;

const unsigned int MOTOR_MAX_COUNT = 2100;	// Max Pon count for motor
const unsigned int MOTOR_NEUT_COUNT = 1600;	// Neutral Pon count for motor
const unsigned int MOTOR_MIN_COUNT = 1000;	// Min Pon count for motor
const unsigned int MOTOR_MIN_FORWARD = 1655; // slowest forward speed
const unsigned int MOTOR_INIT_DUTYRATIO = 2100;

const unsigned int CAMERA_MAX_COUNT = 1750;	// Max Pon count for CAMERA
const unsigned int CAMERA_NEUT_COUNT = 1350;	// Neutral Pon count for CAMERA
const unsigned int CAMERA_MIN_COUNT = 950;	// Min Pon count for CAMERA
const unsigned int CAMERA_INIT_DUTYRATIO = 1750;

const int DUTY_MODE = 0;
const int FREQ_MODE = 1;

// Global variables
int steeringState = 0;
unsigned int steeringWantedDirection = 1437;
unsigned int steeringCurrentDirection = 1437;
unsigned int steeringUpdatedDirection = 0;
int steeringStep = 0;

int motorState = 0;
unsigned int motorWantedAcceleration = 2100;
unsigned int motorCurrentAcceleration = 2100;
unsigned int motorUpdatedDirection = 0;
int motorStep = 0;

int cameraState = 0;
unsigned int cameraWantedDirection = 1750;
unsigned int cameraCurrentDirection = 1750;
unsigned int cameraUpdatedDirection = 0;
int cameraStep = 0;

int buttonCount = 1;

int main(void) {

	resetRegisters();			// Give a clean workspace

//    DCOCTL = 0;                                 // Run DCO at 8 MHz
//    BCSCTL1 = CALBC1_8MHZ;                      //
//    DCOCTL  = CALDCO_8MHZ;                      //

	// Register Setup
	P2DIR |= BIT0 | BIT1 | BIT4;				// Set P2.0, P2.1 and P2.4 as output mode
	P2SEL |= BIT0 | BIT1 | BIT4;				// Connect P2.0, P2.1 and P2.4 to TimerA1
    P2SEL2 &= ~(BIT0 | BIT2 | BIT4);            // Ensure correct mode in PSEL2 for P2.0, P2.1, P2.4

	P1REN |= BIT3;				// Enable internal resistor to P1.3
	P1OUT |= BIT3;				// Set P1.3 resistor as pulled-up

	// Timer_A0 Setup
	TA1CTL = TASSEL_2 + MC_2 + ID_1;	// Use SMCLK/1 in continuous-mode
	TA1CCTL0 = TA1CCTL1 = TA1CCTL2 = OUTMOD_4 + CCIE;		// output mode = toggle, interrupt enabled

	// Initial PWM  interrupts
	unsigned int currCount = TA1R;
	TA1CCR0 = currCount + STEERING_INIT_DUTYRATIO; 	// Steering PWM
	TA1CCR1 = currCount + MOTOR_INIT_DUTYRATIO;	// Motor PWM
	TA1CCR2 = currCount + CAMERA_INIT_DUTYRATIO;	// Camera PWM

	// Interrupt Setup
	P1IE |= BIT3; 				// Enable interrupts for P1.3
	P1IES |= BIT3;				// Set P1.3 to trigger on falling edge
	P1IFG &= 0x0;

	// Enable interrupts
	_EINT();

	// Enter LPM0 with interrupts
	_BIS_SR(LPM0_bits | GIE);

	// Don't Return!! Relying on interrupts
	for(;;){
		__delay_cycles(3000000);
		//CS(steer(STEERING_MAX_COUNT, 2))
		//CS(pan(CAMERA_MIN_COUNT, 2))
		__delay_cycles(2000000);

		//CS(steer(STEERING_MIN_COUNT, 2))
		//CS(pan(STEERING_MAX_COUNT, 2))
		__delay_cycles(2000000);
	}
}

// ISR FOR STEERING PWM
#pragma vector=TIMER1_A0_VECTOR
__interrupt void steeringFreqInterrupt(void){
	if(steeringState == FREQ_MODE){ // Frequency interrupt
		CS(
			// Update the steering duty ratio
			updateSteering();

			// Set next interrupt to occur after duty ratio
			TA1CCR0 += steeringCurrentDirection;
			steeringState = DUTY_MODE;
		)
	}else if(steeringState == DUTY_MODE){ // Duty interrupt
		CS(
			// Set next interrupt to occur at frequency
			TA1CCR0 += PWM_FREQ_COUNT - steeringCurrentDirection;
			steeringState = FREQ_MODE;
		)
	}
}

// ISR FOR MOTOR AND CAMERA PWMs
#pragma vector = TIMER1_A1_VECTOR
__interrupt void motorCameraInterrupt(void){
	if(motorState == FREQ_MODE){ // Frequency interrupt
		CS(
			// Update the motor duty ratio
			updateAcceleration();

			// Set next interrupt to occur after duty ratio
			TA1CCR1 += motorCurrentAcceleration;
			motorState = DUTY_MODE;
		)
	}else if(motorState == DUTY_MODE){ // Duty interrupt
		CS(
			// Set next interrupt to occur at frequency
			TA1CCR1 += PWM_FREQ_COUNT - motorCurrentAcceleration;
			motorState = FREQ_MODE;
		)
	}

	if(cameraState == FREQ_MODE){ // Frequency interrupt
		CS(
			// Update the camera duty ratio
			updateCamera();

			// Set next interrupt to occur after duty ratio
			TA1CCR2 += cameraCurrentDirection;
			cameraState = DUTY_MODE;
		)
	}else if(cameraState == DUTY_MODE){ // Duty interrupt
		CS(
			// Set next interrupt to occur at frequency
			TA1CCR2 += PWM_FREQ_COUNT - cameraCurrentDirection;
			cameraState = FREQ_MODE;
		)
	}
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void){
	CS(	// Begin critical section

	if(P1IFG & BIT3){
		switch (buttonCount){ // buttonCount currently initialised to 1
			case 0: {								//To max Pon for motor
				motorWantedAcceleration = MOTOR_MAX_COUNT;
				motorCurrentAcceleration = MOTOR_MAX_COUNT;
				buttonCount++;
				break;
			}
			case 1: {								//To min Pon for motor
				motorWantedAcceleration = MOTOR_MIN_COUNT;
				motorCurrentAcceleration = MOTOR_MIN_COUNT;
				buttonCount++;
				break;
			}
			case 2: {								//To neutral Pon for motor
				motorWantedAcceleration = MOTOR_NEUT_COUNT;
				motorCurrentAcceleration = MOTOR_NEUT_COUNT;
				buttonCount++;
				break;
			}
			case 3: {								//To lowest forward Pon for motor
				accelerate(MOTOR_MIN_FORWARD, 1);
				buttonCount++;
				break;
			}
			case 4: {								//To min Pon for motor
				accelerate(MOTOR_MIN_COUNT, 1);
				buttonCount++;
				break;
			}
			case 5: {								//To neutral Pon for motor
				accelerate(MOTOR_NEUT_COUNT, 1);
				buttonCount = 3;
				break;
			}
			default: break;
		}
		__delay_cycles(1000000);
		P1IFG &= ~BIT3;         // Clear P1.3 Interrupt Flag
	}

	) // End critical section
}

/*
 * Changes the step size of the steering PWM over duration seconds
 */
void steer(int direction, int duration){
	steeringWantedDirection = direction;
	steeringStep = (steeringWantedDirection - steeringCurrentDirection) / (duration * PWM_FREQ);
}

/*
 * Changes the step size of the motor PWM over duration seconds
 */
void accelerate(int acceleration, int duration){
	motorWantedAcceleration = acceleration;
	motorStep = (motorWantedAcceleration - motorCurrentAcceleration) / (duration * PWM_FREQ);
}

/*
 * Changes the step size of the camera PWM over duration seconds
 */
void pan(int direction, int duration){
	cameraWantedDirection = direction;
	cameraStep = (cameraWantedDirection - cameraCurrentDirection) / (duration * PWM_FREQ);
}

/*
 * Updates the duty ratio for steering PWM
 */
void updateSteering(){
	unsigned int steeringUpdatedDirection = steeringCurrentDirection + steeringStep;
	if(steeringStep > 0 && steeringUpdatedDirection >= steeringWantedDirection){	// Positive step boundary
		steeringCurrentDirection = steeringWantedDirection;
	} else if(steeringStep < 0 && steeringUpdatedDirection <= steeringWantedDirection){		// Negative step boundary
		steeringCurrentDirection = steeringWantedDirection;
	} else {	// Within interval
		steeringCurrentDirection = steeringUpdatedDirection;
	}
}

/*
 * Updates the duty ratio for motor PWM
 */
void updateAcceleration(){
	unsigned int motorUpdatedAcceleration = motorCurrentAcceleration + motorStep;
		if(motorStep > 0 && motorUpdatedAcceleration >= motorWantedAcceleration){	// Positive step boundary
			motorCurrentAcceleration = motorWantedAcceleration;
		} else if(motorStep < 0 && motorUpdatedAcceleration <= motorWantedAcceleration){		// Negative step boundary
			motorCurrentAcceleration = motorWantedAcceleration;
		} else {	// Within interval
			motorCurrentAcceleration = motorUpdatedAcceleration;
		}
}

/*
 * Updates the duty ratio for camera PWM
 */
void updateCamera(){
	 = cameraCurrentDirection + cameraStep;
	if(cameraStep > 0 && cameraUpdatedDirection >= cameraWantedDirection){	// Positive step boundary
		cameraCurrentDirection = cameraWantedDirection;
	} else if(cameraStep < 0 && cameraUpdatedDirection <= cameraWantedDirection){		// Negative step boundary
		cameraCurrentDirection = cameraWantedDirection;
	} else {	// Within interval
		cameraCurrentDirection = cameraUpdatedDirection;
	}
}

void resetRegisters(){
	WDTCTL = WDTPW + WDTHOLD;	// Turn off watchdog timer
	P1OUT &= 0x0;
	P1REN &= 0x0;
	P1DIR &= 0x0;
	P1SEL &= 0x0;
	P1IE &= 0x0;
	P1IES &= 0x0;
	P1IFG &= 0x0;
	P2OUT &= 0x0;
	P2REN &= 0x0;
	P2DIR &= 0x0;
	P2SEL &= 0x0;
	P2IE &= 0x0;
	P2IES &= 0x0;
	P2IFG &= 0x0;
}
