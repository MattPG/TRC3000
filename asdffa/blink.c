#include <msp430.h>
#include "stdio.h"
#include "stdint.h"

void update(unsigned int pwm);
void change(int direction, int duration, unsigned int pwm);
void asdf();

#define CS(x) _DINT(); x; _EINT();

static const unsigned int STEERING = 1;
static const unsigned int MOTOR = 2;
static const unsigned int PAN = 0;
static volatile unsigned int pwm_on[3];                      // PWM on duratio
static unsigned pwm_period[3];                  // Total PWM period - on duration + off duration
static unsigned wanted[3];
static volatile int step[3];
static unsigned state[3] = {0};
static volatile unsigned prevDuty[3];
static const unsigned MAX[3] = {4900, 3050, 3833};
static const unsigned MIN[3] = {1095, 1827, 1916};
static const unsigned NEUT[3] = {3000, 2539, 2874};
static unsigned buttonCount = 1;
volatile unsigned int tx_num;
volatile unsigned int rx_num;
volatile unsigned char tx_char;
volatile unsigned char rx_char;
volatile unsigned int tx_sendi = 0;
volatile unsigned int tx_i;
volatile unsigned int rx_i;
volatile unsigned char tx_index;
volatile unsigned int tx_data = 0;
volatile unsigned int rx_data = 0, rx_data1 = 0;
volatile unsigned int xl = 0;
//int check = 0;
volatile unsigned int i,j = 0;
volatile unsigned int leftbit;
volatile unsigned int rightbit;
volatile unsigned int y = 0, x = 0, rx_num = 0;
int test;
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

    P2DIR |= (BIT3 | BIT1 | BIT4);              // PWM outputs on P2.3, P2.1, P2.4
    P2SEL |= (BIT3 | BIT1 | BIT4);              // Enable timer compare outputs
    P2SEL2 &= ~(BIT3 | BIT1 | BIT4);            //

    P1REN |= BIT3;								// Enable internal resistor to P1.3
    P1OUT |= BIT3;								// Set P1.3 resistor as pulled-up

    P1SEL = BIT1 + BIT2 ;               	    // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2 ;						// P1.1 = RXD, P1.2=TXD

    UCA0CTL1 |= UCSSEL_3;						// SMCLK
    UCA0BR0 = 69;								// 8MHz 9600
    UCA0BR1 = 0;								// 8MHz 9600


    UCA0MCTL = UCBRF_0 | UCBRS_4 ;		// Modulation UCBRSx = 0, UCBRFx = 1, UCOS16 enabled

    UCA0CTL1 &= ~UCSWRST;						// **Initialize USCI state machine**

    IE2 |= UCA0RXIE;							// Enable USCI_A0 RX interrupt
//    IE2 |= UCA0TXIE;


	P1IE |= BIT3; 								// Enable interrupts for P1.3
	P1IES |= BIT3;								// Set P1.3 to trigger on falling edge
	P1IFG &= 0x0;

	UCA0TXBUF &= 0x0;
	UCA0RXBUF &= 0x0;
                                                //
    TA1CTL = TASSEL_2 | ID_2 | MC_2;            // Setup timer 1 for SMCLK / 4, continuous mode
                                                //
    TA1CCTL0 = TA1CCTL1 = TA1CCTL2 = OUTMOD_4 | CCIE; // Set timer output to toggle mode, enable interrupt
    TA1CCR0 = TA1CCR1 = TA1CCR2 = TA1R + 1000;  // Set initial interrupt time
}                                               //
//
//#pragma vector = USCIAB0TX_VECTOR		//UART TX USCI Interrupt
//__interrupt void USCI0TX_ISR(void)
//{
//	CS(
//			tx_char = 'z';
//			UCA0TXBUF = tx_char;
//			IFG2 &= ~UCA0TXIFG					//Interrupt flag cleared
//	)
//}
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
//    asdf();
                                           // Setup PWM period
    pwm_period[STEERING] = pwm_period[MOTOR] = pwm_period[PAN] = PWM_FREQ_COUNT;
    pwm_on[STEERING] = MIN[STEERING];                    // Setup servo times

    pwm_on[MOTOR] = MAX[MOTOR];                       //
    pwm_on[PAN] = NEUT[PAN];                         //
                                                //
    _enable_interrupts();                       // Enable all interrupts
                                                //

    for(;;) {                                   //

    	__delay_cycles(8000000);
//		CS(change(MIN[PAN],15,PAN))
//    	__delay_cycles(24000000);
//////
//		CS(change(MAX[PAN],5,PAN))
//    	__delay_cycles(8000000);

//    			CS(change(MAX[STEERING], 2, STEERING))
//    			__delay_cycles(40000000);
//    			CS(change(MIN[STEERING], 2, STEERING))
//    			__delay_cycles(40000000);
//    			if (tx_data == NULL){
//    				tx_data = 'a';
//    			}else if (tx_data == 'a'){
//    				tx_data = 'b';
//    			}
//    			IFG2 |= UCA0TXIFG;

    }                                           //
}                                               //

/*
 * Changes the steering PWM duty ratio to direction over duration seconds
 */
void change(int direction, int duration, unsigned int pwm){
	static const unsigned int PWM_FREQ = 45;
	static long int diff = 0, stepdiv = 0;
	static long unsigned int div = 0;
	wanted[pwm] = direction;
	diff = (wanted[pwm] - pwm_on[pwm]);
	div = (duration * PWM_FREQ);
	stepdiv = diff / div ;
	step[pwm] = stepdiv*10;

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
//				tx_char = 'b';//1k
				break;
			}
			case 3: {								//To lowest forward Pon for motor
				change(2900, 100, MOTOR);
				buttonCount++;
//				tx_char = 'c';//10k
				break;
			}
			case 4: {								//To min Pon for motor
				change(MIN[MOTOR], 100, MOTOR);
				buttonCount++;
//				tx_char = 'a';//5k
				break;
			}
			case 5: {								//To neutral Pon for motor
				change(NEUT[MOTOR], 100, MOTOR);
				buttonCount = 3;
//				tx_char = 'b';//1k
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
	rx_char = UCA0RXBUF;
	rx_data = (int) (rx_char - 'a');
	int rx_num1 = rx_data*40;
	if (rx_num1 <= 1024 && rx_num1 >= 0){
		rx_num = rx_num1;
//		if (rx_num < 512){
//			y = 512 - rx_num;
			x = (pwm_on[PAN] + 512 - rx_num);
			if (x > MAX[PAN]){
				x = MAX[PAN];
			}
//			change(x,3,PAN);
//		}else {
////			y = rx_num - 512;
//			x = (pwm_on[PAN] - rx_num - 512);
			else if (x < MIN[PAN]){
				x = MIN[PAN];
			}
			change(x,5,PAN);
//		}

	}

//	while (!(IFG2&UCA0TXIFG)){                // USCI_A0 TX buffer ready?
//		UCA0TXBUF = UCA0RXBUF;                    // TX -> RXed character
//	}



}
//
//void asdf(){
//	while(1){
//		rx_data = 'c';
//		rx_data -= 'a';
//		rx_char = rx_data;
//	}
//}
//if (UCA0TXBUF == 0x0){
//				UCA0TXBUF = 'a';
//			}else if (UCA0TXBUF == 'a'){
//				UCA0TXBUF = tx_char[1];
//			}else if (UCA0TXBUF == tx_char[1]){
//				UCA0TXBUF = tx_char[2];
//			}else{
//				UCA0TXBUF = 0x0;
//			}

