#include <msp430.h>
#include <msp430g2553.h>

// Function headers
void resetRegisters();
void steer(int direction, int duration);
void accelerate(int direction, int duration);

#define CS(x) _DINT(); x; _EINT();

// Global constants
const int STEERING_FREQ = 45; // Freq. in Hz
const int STEERING_FREQ_COUNT = 21600; // TACCRx Count
const int STEERING_MAX_COUNT = 1750;	// Max Pon count for steering
const int STEERING_NEUT_COUNT = 1350;	// Neutral Pon count for steering
const int STEERING_MIN_COUNT = 950;	// Min Pon count for steering
const int STEERING_INIT_DUTYRATIO = 1000;

const int MOTOR_FREQ = 45; // Freq in hz.
const int MOTOR_FREQ_COUNT = 21600; // TACCRx Count
const int MOTOR_MAX_COUNT = 2100;	// Max Pon count for motor
const int MOTOR_NEUT_COUNT = 1600;	// Neutral Pon count for motor
const int MOTOR_MIN_COUNT = 1000;	// Min Pon count for motor
const int MOTOR_INIT_DUTYRATIO = 2100;

// Global variables
int steeringWantedDirection = 1000;
int steeringCurrentDirection = 1000;
int steeringStep = 0;

int motorWantedAcceleration = 2100;
int motorCurrentAcceleration = 2100;
int motorStep = 0;

int buttonCount = 1;

int main(void) {

	resetRegisters();			// Give a clean workspace

	// Register Setup
	P2DIR |= BIT1 | BIT4;				// Set P2.1 and P2.4 as output mode
	P2SEL |= BIT1 | BIT4;				// Connect P2.1 and P2.4 to TimerA1
	P1REN |= BIT3;				// Enable internal resistor to P1.3
	P1OUT |= BIT3;				// Set P1.3 resistor as pulled-up

	// Timer_A0 Setup
	TA1CCR0 = STEERING_FREQ_COUNT; 	// Set the max count for timerA1
	TA1CCR1 = STEERING_INIT_DUTYRATIO;	// Set initial steering duty ratio
	TA1CCR2 = MOTOR_INIT_DUTYRATIO;	// Set initial motor duty ratio
	TA1CCTL0 = OUTMOD_4 + CCIE;		// output mode = toggle, interrupt enabled
	TA1CCTL1 = OUTMOD_7;  // output mode = reset/set, interrupt enabled
	TA1CCTL2 = OUTMOD_7;  // output mode = reset/set, interrupt enabled
	TA1CTL = TASSEL_2 + MC_1 + ID_0;	// Use SMCLK in Up-mode

	// Interrupt Setup
	P1IE |= BIT3; 				// Enable interrupts for P1.3
	P1IES |= BIT3;				// Set P1.3 to trigger on falling edge
	P1IFG &= 0x0;

	// Enable interrupts
	_EINT();

	// Enter LPM0 with interrupts
//	_BIS_SR(LPM0_bits | GIE);

	// Don't Return!! Relying on interrupts
	for(;;){
		__delay_cycles(3000000);
		CS(steer(STEERING_MAX_COUNT, 2))
		__delay_cycles(2000000);

		CS(steer(STEERING_MIN_COUNT, 2))
		__delay_cycles(2000000);
	}
}
// TimerA1 CCR0 ISR
#pragma vector=TIMER1_A0_VECTOR
__interrupt void steeringFreqInterrupt(void){
	CS( // Begin critical section

	// Update steering direction
	int steeringUpdatedDirection = steeringCurrentDirection + steeringStep;
	if(steeringStep > 0 && steeringUpdatedDirection >= steeringWantedDirection){	// Positive step boundary
		TACCR1 = steeringWantedDirection;
		steeringCurrentDirection = steeringWantedDirection;
	} else if(steeringStep < 0 && steeringUpdatedDirection <= steeringWantedDirection){		// Negative step boundary
		TACCR1 = steeringWantedDirection;
		steeringCurrentDirection = steeringWantedDirection;
	} else {	// Within interval
		TA1CCR1 = steeringUpdatedDirection;
		steeringCurrentDirection = steeringUpdatedDirection;
	}

	// Update motor acceleration
	int motorUpdatedAcceleration = motorCurrentAcceleration + motorStep;
	if(motorStep > 0 && motorUpdatedAcceleration >= motorWantedAcceleration){	// Positive step boundary
		TACCR2 = motorWantedAcceleration;
		motorCurrentAcceleration = motorWantedAcceleration;
	} else if(motorStep < 0 && motorUpdatedAcceleration <= motorWantedAcceleration){		// Negative step boundary
		TACCR2 = motorWantedAcceleration;
		motorCurrentAcceleration = motorWantedAcceleration;
	} else {	// Within interval
		TA1CCR2 = motorUpdatedAcceleration;
		motorCurrentAcceleration = motorUpdatedAcceleration;
	}

	// TACCR0 interrupt flag is automatically reset

	) // End critical section
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void){
	CS(	// Begin critical section

	if(P1IFG & BIT3){
		switch (buttonCount){ // buttonCount currently initialised to 1
			case 0: {								//To max Pon for motor
				motorWantedAcceleration = MOTOR_MAX_COUNT;
				motorCurrentAcceleration = MOTOR_MAX_COUNT;
				CCR2 = MOTOR_MAX_COUNT;
				buttonCount++;
				break;
			}
			case 1: {								//To min Pon for motor
				motorWantedAcceleration = MOTOR_MIN_COUNT;
				motorCurrentAcceleration = MOTOR_MIN_COUNT;
				CCR2 = MOTOR_MIN_COUNT;
				buttonCount++;
				break;
			}
			case 2: {								//To neutral Pon for motor
				motorWantedAcceleration = MOTOR_NEUT_COUNT;
				motorCurrentAcceleration = MOTOR_NEUT_COUNT;
				CCR2 = MOTOR_NEUT_COUNT;
				buttonCount++;
				break;
			}
			case 3: {								//To lowest forward Pon for motor
				accelerate(1655, 1);
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
 * Changes the steering PWM duty ratio to direction over duration seconds
 */
void steer(int direction, int duration){
	steeringWantedDirection = direction;
	steeringStep = (steeringWantedDirection - steeringCurrentDirection) / (duration * STEERING_FREQ);
}

/*
 * Changes the motor PWM duty ration to acceleration over duration seconds
 */
void accelerate(int acceleration, int duration){
	motorWantedAcceleration = acceleration;
	motorStep = (motorWantedAcceleration - motorCurrentAcceleration) / (duration * MOTOR_FREQ);
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
