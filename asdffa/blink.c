#include <msp430.h>
#include <msp430g2553.h>
//#include <stdio.h>

// Function headers
void resetRegisters();
void steer(int direction, int duration);
void accelerate(int direction, int duration);

#define CS(x) _DINT(); x; _EINT();

// Global constants
const int STEERING_FREQ = 45; // Freq. in Hz
const int STEERING_FREQ_COUNT = 2700; // TACCRx Count
const int STEERING_INIT_DUTYRATIO = 1500;

const int MOTOR_FREQ = 45; // Freq in hz.
const int MOTOR_FREQ_COUNT = 2700; // TACCRx Count
const int MOTOR_INIT_DUTYRATIO = 0;

// Global variables
int steeringWantedDirection = 1500;
int steeringCurrentDirection = 1500;
int steeringStep = 0;

int motorWantedAcceleration = 0;
int motorCurrentAcceleration = 0;
int motorStep = 0;

int buttonCount = 0;

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
	TA1CTL = TASSEL_2 + MC_1 + ID_3;	// Use SMCLK in Up-mode

	// Interrupt Setup
	P1IE |= BIT3; 				// Enable interrupts for P1.3
	P1IES |= BIT3;				// Set P1.3 to trigger on falling edge
	P1IFG &= 0x0;

	_EINT();
	// Power Mode
	// Enter LPM0 with interrupts
//	_BIS_SR(LPM0_bits | GIE);

	// Don't Return!! Relying on interrupts
	for(;;){
		CS(steer(STEERING_FREQ_COUNT * 0.5, 1))
//		accelerate(MOTOR_FREQ_COUNT * 0.05, 2);
		__delay_cycles(9000000);

		CS(steer(STEERING_FREQ_COUNT * 0.05, 1))
//		accelerate(MOTOR_FREQ_COUNT * 0.8, 2);
		__delay_cycles(9000000);
	}
}
// TimerA1 CCR0 ISR
#pragma vector=TIMER1_A0_VECTOR
__interrupt void steeringFreqInterrupt(void){
	_DINT();

	// Update Steering
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

	// Update Motor
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

	_EINT();
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void){
	_DINT();
	if(P1IFG & BIT3){
		int x = (P1IFG & BIT3);
		switch (buttonCount){
			case 0: {
				accelerate(MOTOR_FREQ_COUNT, 2);
				buttonCount++;
				break;
			}
			case 1: {
				accelerate(MOTOR_FREQ_COUNT*.1, 2);
				buttonCount++;
				break;
			}
			case 2: {
				accelerate(MOTOR_FREQ_COUNT*.5, 2);
				buttonCount++;
				break;
			}
			case 3: {
				accelerate(0, 2);
				buttonCount = 0;
				break;
			}
			default: break;
		}
		P1IFG &= ~BIT3;         // Clear P1.3 Interrupt Flag
	}
	_EINT();
}

//void toggleTimerA0DutyRatio(){
//        if(A0DutySelect)
//                TA0CCR1 = TIMER_A0_DUTY_1;
//        else
//                TA0CCR1 = TIMER_A0_DUTY_2;
//
//        A0DutySelect = ~A0DutySelect;
//}
//
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
