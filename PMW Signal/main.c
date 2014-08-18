
#include <msp430.h>
#include <msp430g2553.h>

const int TIMER_A0_FREQ = 512-1;
const int TIMER_A0_DUTY_1 = 255;
const int TIMER_A0_DUTY_2 = 127;


int A0DutySelect = 0;

void toggleTimerA0DutyRatio();
void resetRegisters();

int main(void) {

	resetRegisters();				// Give a clean workspace

	// Register Setup
	P1DIR |= BIT2;				// Set P1.2 as output mode
	P1SEL |= BIT2;				// Connect Port2 to TimerA0
	P1REN |= BIT3;				// Enable internal resistor to P1.3
	P1OUT |= BIT3;				// Set P1.3 resistor as pulled-up

	// Clock Setup
	TA0CCR0 = TIMER_A0_FREQ; 	// Set the max count of timera0
	TA0CCR1 = TIMER_A0_DUTY_1;	// Set 'on' count of timera0
	TA0CCTL1 = OUTMOD_7;		// Use set/reset mode
	TA0CTL = TASSEL_2 + MC_1;	// something something

	// Interrupt Setup
	P1IE |= BIT3; 				// Enable interrupts for P1.3
	P1IES |= BIT3;				// Set P1.3 to trigger on falling edge

	// Power Mode
	_BIS_SR(CPUOFF | GIE);		// Enter LPM0 with interrupts

	// Don't Return!! Relying on interrupts
	for(;;){}
}


// Port 1 ISR
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void){
	if(P1IFG & BIT3){
		toggleTimerA0DutyRatio();
		__delay_cycles(100000);
		P1IFG &= ~BIT3;		// Clear P1.3 Interrupt Flag

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
}

void toggleTimerA0DutyRatio(){
	if(A0DutySelect)
		TA0CCR1 = TIMER_A0_DUTY_1;
	else
		TA0CCR1 = TIMER_A0_DUTY_2;

	A0DutySelect = ~A0DutySelect;
}
