#include <msp430.h>
#include "stdio.h"

void update(unsigned int pwm);
void change(int direction, int duration, unsigned int pwm);
void asdf();

#define CS(x) _DINT(); x; _EINT();

static const unsigned int STEERING = 1;
static const unsigned int MOTOR = 2;
static const unsigned int PAN = 0;
static volatile unsigned pwm_on[3];                      // PWM on duratio
static unsigned pwm_period[3];                  // Total PWM period - on duration + off duration
static unsigned wanted[3];
static volatile int step[3];
static unsigned state[3] = {0};
static volatile unsigned prevDuty[3];
static const unsigned MAX[3] = {4900, 3050, 3833};
static const unsigned MIN[3] = {1095, 1827, 1916};
static const unsigned NEUT[3] = {2998, 2539, 2874};
static unsigned buttonCount = 1;
volatile char tx_char = 'a';
volatile char rx_char;
int i,j = 0;
//
#pragma vector = TIMER1_A0_VECTOR               // - ISR for CCR0
__interrupt void isr_ccr0(void)                 //
{                                               //
    CS(                                            //
    if(state[PAN]++ & 1){
    	TA1CCR0 += (pwm_period[PAN] - prevDuty[PAN]);
    } else{
    	TA1CCR0 += (prevDuty[PAN] = pwm_on[PAN]);
    	update(PAN);
    }
    )
}                                               //
                                                //
#pragma vector = TIMER1_A1_VECTOR               // - ISR for CCR1, CCR2
__interrupt void isr_ccr12(void)                //
{                                               //
    CS(                                            //
    switch(TA1IV) {                             //
        case 0x02:                              // - CCR1
            if(state[STEERING]++ & 1){
            	TA1CCR1 += (pwm_period[STEERING] - prevDuty[STEERING]);
            } else{
            	TA1CCR1 += (prevDuty[STEERING] = pwm_on[STEERING]);
            	update(STEERING);
            }
            break;                              //
        case 0x04:                              // - CCR2
            if(state[MOTOR]++ & 1){
            	TA1CCR2 += (pwm_period[MOTOR] - prevDuty[MOTOR]);
            } else{
            	TA1CCR2 += (prevDuty[MOTOR] = pwm_on[MOTOR]);
            	update(MOTOR);
            }
            break;                              //
    }
    )											//
}                                               //
                                                //
static void pwm_init(void)                      //
{                                               //
    P2DIR |= (BIT3 | BIT1 | BIT4);              // PWM outputs on P2.3, P2.2, P2.4
    P2SEL |= (BIT3 | BIT1 | BIT4);              // Enable timer compare outputs
    P2SEL2 &= ~(BIT3 | BIT1 | BIT4);            //

    P1REN |= BIT3;								// Enable internal resistor to P1.3
    P1OUT |= BIT3;								// Set P1.3 resistor as pulled-up

    P1SEL = BIT1 + BIT2 ;               	    // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2 ;						// P1.1 = RXD, P1.2=TXD

    UCA0CTL1 |= UCSSEL_2;						// SMCLK
    UCA0BR0 = 65;								// 8MHz 9600
    UCA0BR1 = 3;								// 8MHz 9600


    UCA0MCTL = 0x09;							// Modulation UCBRSx = 4

    UCA0CTL1 &= ~UCSWRST;						// **Initialize USCI state machine**

    IE2 |= UCA0RXIE;							// Enable USCI_A0 RX interrupt

	P1IE |= BIT3; 								// Enable interrupts for P1.3
	P1IES |= BIT3;								// Set P1.3 to trigger on falling edge
	P1IFG &= 0x0;


                                                //
    TA1CTL = TASSEL_2 | ID_2 | MC_2;            // Setup timer 1 for SMCLK / 8, continuous mode
                                                //
    TA1CCTL0 = TA1CCTL1 = TA1CCTL2 = OUTMOD_4 | CCIE; // Set timer output to toggle mode, enable interrupt
    TA1CCR0 = TA1CCR1 = TA1CCR2 = TA1R + 1000;  // Set initial interrupt time
}                                               //
                                                //
int main(void)                                  //
{
    static const unsigned int PWM_FREQ_COUNT = 39430;
    WDTCTL = WDTPW | WDTHOLD;                   //
                                                //
    DCOCTL = 0;                                 // Run DCO at 8 MHz
    BCSCTL1 = CALBC1_8MHZ;                      //
    DCOCTL  = CALDCO_8MHZ;                      //
                                                //
    pwm_init();                                 // Initialize PWM

                                           // Setup PWM period
    pwm_period[STEERING] = pwm_period[MOTOR] = pwm_period[PAN] = PWM_FREQ_COUNT;
    pwm_on[STEERING] = MIN[STEERING];                    // Setup servo times

    pwm_on[MOTOR] = MAX[MOTOR];                       //
    pwm_on[PAN] = NEUT[PAN];                         //
                                                //
    _enable_interrupts();                       // Enable all interrupts
                                                //
    for(;;) {                                   //
    	__delay_cycles(10000000);
    			CS(change(MAX[PAN], 2, PAN))
    			__delay_cycles(40000000);
    			CS(change(MIN[PAN], 2, PAN))
    			__delay_cycles(40000000);
    			IE2 |= UCA0TXIE;
    			IFG2 |= UCA0TXIFG;
    }                                           //
}                                               //

/*
 * Changes the steering PWM duty ratio to direction over duration seconds
 */
void change(int direction, int duration, unsigned int pwm){
	static const unsigned int PWM_FREQ = 45;
	static int diff = 0, div = 0;
	wanted[pwm] = direction;
	diff = (wanted[pwm] - pwm_on[pwm]);
	div = (duration * PWM_FREQ);
	step[pwm] = diff / div;
}

void update(unsigned int pwm){
	unsigned int newVal = pwm_on[pwm] + step[pwm];
	if(step[pwm] > 0 && newVal >= wanted[pwm]){	// Positive step boundary
		pwm_on[pwm] = wanted[pwm];
	} else if(step[pwm] < 0 && newVal <=  wanted[pwm]){		// Negative step boundary
		pwm_on[pwm] = wanted[pwm];
	} else {	// Within interval
		pwm_on[pwm] = newVal;
	}
}
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void){
	CS(	// Begin critical section

	if(P1IFG & BIT3){
		switch (buttonCount){ // buttonCount currently initialised to 1
			case 0: {								//To max Pon for motor
				wanted[MOTOR] = MAX[MOTOR];
				pwm_on[MOTOR] = MAX[MOTOR];
				TA1CCR2 = MAX[MOTOR];
				buttonCount++;
				break;
			}
			case 1: {								//To min Pon for motor
				wanted[MOTOR] = MIN[MOTOR];
				pwm_on[MOTOR] = MIN[MOTOR];
				TA1CCR2 = MIN[MOTOR];
				buttonCount++;
				break;
			}
			case 2: {								//To neutral Pon for motor
				wanted[MOTOR] = NEUT[MOTOR];
				pwm_on[MOTOR] = NEUT[MOTOR];
				TA1CCR2 = NEUT[MOTOR];
				buttonCount++;
				tx_char = 'b';//1k
				break;
			}
			case 3: {								//To lowest forward Pon for motor
				change(2900, 1, MOTOR);
				buttonCount++;
				tx_char = 'c';//10k
				break;
			}
			case 4: {								//To min Pon for motor
				change(MIN[MOTOR], 1, MOTOR);
				buttonCount++;
				tx_char = 'a';//5k
				break;
			}
			case 5: {								//To neutral Pon for motor
				change(NEUT[MOTOR], 1, MOTOR);
				buttonCount = 3;
				tx_char = 'b';//1k
				break;
			}
			default: break;
		}
//		__delay_cycles(500000);
		P1IFG &= ~BIT3;         // Clear P1.3 Interrupt Flag
	}
	) // End critical section
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{

//		while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
//		UCA0TXBUF = UCA0RXBUF;                    // TX -> RXed character
	rx_char = UCA0RXBUF;							//Copy from RX buffer, in doing so we ACK the interrupt as well
	if (rx_char == 'b')
	{
		change(1000, 1, 0);

	}else if(rx_char == 'c'){
		change(10000, 1, 0);
	}else if(rx_char == 'a'){
		change(5000, 1, 0);
	}

//	__delay_cycles(10000000);
//	asdf();
//									//Set the rx_flag to 1


}

#pragma vector = USCIAB0TX_VECTOR		//UART TX USCI Interrupt
__interrupt void USCI0TX_ISR(void)
{
	CS(
//		j++;
//		if(j%2){
//			tx_char = 'b';
//		}else {
//			tx_char = 'c';
//		}
		UCA0TXBUF = tx_char;				//Copy char to the TX Buffer
		IE2 &= ~UCA0TXIE; 					//Turn off the interrupt to save CPU
		IFG2 &= ~UCA0TXIFG
	)
}

void asdf(){
	while(1);
}



