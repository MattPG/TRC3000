#include <msp430.h>
#include "stdio.h"
#include <math.h>
#include "stdint.h"
#include "msp430g2553.h"

double coordy;
double coordx;
signed int coord2[3];
int loopcount = 0;
int evenodd = 0;
void update(unsigned int pwm);
void change(int direction, int duration, unsigned int pwm);
void asdf();
void receiveSetup();
void transmitSetup();
void turnSouth();
void turnNorth();
void track();
void follow();
void kill();

#define CS(x) _DINT(); x; _EINT();
volatile char h[6];
unsigned char *PRxData;                     // Pointer to RX data
unsigned int RXByteCtr;
volatile unsigned char RxBuffer[60];       // Allocate 128 byte of RAM
volatile signed int coord[30];
unsigned char *PTxData;                     // Pointer to TX data
unsigned int TXByteCtr;
const unsigned char TxData[] =              // Table of data to transmit
{
  0x02,
  0x00,
  0x03
};

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
volatile int rx_data = 0, rx_data1 = 0;
volatile unsigned int xl = 0;
//int check = 0;
volatile unsigned int i,j = 0;
volatile unsigned int leftbit;
volatile unsigned int rightbit;
volatile unsigned int x = 0, rx_num = 0;
int test;
int rx_num1;
int data_x,data_y,data_z;
int tx = 1;
double theta;
int killed = 0;
int go_init;
int track_init;


//
#pragma vector = TIMER1_A0_VECTOR               // - ISR for CCR0
__interrupt void isr_ccr0(void)                 //
{                                               //
                                            //
    if(state[PAN]++ & 1){
    	TA1CCR0 += (pwm_period[PAN] - prevDuty[PAN]);
    } else{
    	TA1CCR0 += (prevDuty[PAN] = pwm_on[PAN]);
    	update(PAN);
    }

}                                               //
                                                //
#pragma vector = TIMER1_A1_VECTOR               // - ISR for CCR1, CCR2
__interrupt void isr_ccr12(void)                //
{                                               //
                                             //
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
    									//
}                                               //
                                                //
static void pwm_init(void)                      //
{                                               //

    P2DIR |= (BIT3 | BIT1 | BIT4);              // PWM outputs on P2.3, P2.1, P2.4
    P2SEL |= (BIT3 | BIT1 | BIT4);              // Enable timer compare outputs
    P2SEL2 &= ~(BIT3 | BIT1 | BIT4);            //

    P1REN |= BIT3;								// Enable internal resistor to P1.3
    P1OUT |= BIT3;								// Set P1.3 resistor as pulled-up

    P1SEL |= BIT1 + BIT2 + BIT6 + BIT7;          // P1.1 = RXD, P1.2 = TXD, P1.6 = SCL, P1.7 = SDA
    P1SEL2 |= BIT1 + BIT2 + BIT6 + BIT7 ;		// P1.1 = RXD, P1.2 = TXD, P1.6 = SCL, P1.7 = SDA
    P1DIR |= BIT6 + BIT7;

    UCA0CTL1 |= UCSSEL_3;						// SMCLK
    UCA0BR0 = 69;								// 8MHz 9600
    UCA0BR1 = 0;								// 8MHz 9600
    IE2 &= ~UCB0RXIE;
    UCB0CTL1 |= UCSWRST; 						// Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;		// I2C Master,sync mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST; 				// Use SMCLK, keep SW rst
    UCB0BR0 = 80; 								// fSCL=SMCLK/80=100kHz
    UCB0BR1 = 0;
    UCB0I2CSA = 0x1E; 							// Slave Address is 01Eh
    UCB0CTL1 &= ~UCSWRST; 						// Clear SW rst, resume
    IE2 |= UCB0TXIE;

    UCA0MCTL = UCBRF_0 | UCBRS_4 ;				// Modulation UCBRSx = 0, UCBRFx = 1, UCOS16 enabled

    UCA0CTL1 &= ~UCSWRST;						// **Initialize USCI state machine**

    IE2 |= UCA0RXIE;					// Enable USCI_A0 RX, USCI_BO RX & TX interrupt


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
    go_init = 0;
    track_init = 0;
}                                               //

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
//    _enable_interrupts();                       // Enable all interrupts
                                                //

    PRxData = (unsigned char *)RxBuffer;    // Start of RX buffer
    RXByteCtr = sizeof h-1;                          // Load RX byte counter

    PTxData = (unsigned char *)TxData; // TX array start address
	TXByteCtr = sizeof TxData; // Load TX byte counter

	while (TXByteCtr != 0){
		//TRANSMIT
		//    	_DINT();
		transmitSetup();
		UCB0CTL1 |= UCTR + UCTXSTT; // I2C TX, start condition
		__bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/interrupts
    	while (UCB0CTL1 & UCTXSTP); // Ensure stop condition got sent

		// Remain in LPM0 until all data are TX'd
	}
    for(;;) {                                   //


    	//RECEIVE
    	while (UCB0CTL1 & UCTXSTP);
    	if (TXByteCtr == 0){
    	    RXByteCtr = 6;                          // Load RX byte counter
    	    while (UCB0CTL1 & UCTXSTP); // Ensure stop condition got sent

    	    tx = 0;
    		receiveSetup();
    		UCB0CTL1 |= UCTXSTT; // I2C RX, start condition
    		__bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/interrupts
    		tx = 1;
			TXByteCtr = 1;
			transmitSetup();
			PTxData = (unsigned char *)&(TxData[(sizeof TxData)-1]);
			UCB0CTL1 |= UCTR + UCTXSTT; // I2C TX, start condition
			__bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/interrupts
			while (UCB0CTL1 & UCTXSTP); // Ensure stop condition got sent
    	}
    	if(loopcount >= 90){
    		loopcount = 0;
    	}
    	// Remain in LPM0 until
    	// all data are RX'd
    	track();
    	follow();
    }                                           //
}                                               //

/*
 * Changes the steering PWM duty ratio to direction over duration seconds
 */
void change(int direction, int duration, unsigned int pwm){
	static const unsigned int PWM_FREQ = 45;
	int diff = 0;
	static int stepdiv = 0;
	static unsigned int div = 0;
	wanted[pwm] = direction;
	int isReverse = wanted[pwm]<pwm_on[pwm];

	diff = isReverse ? pwm_on[pwm]-wanted[pwm] : (wanted[pwm] - pwm_on[pwm]);
	div = (duration * PWM_FREQ);
	stepdiv = diff / div ;
	stepdiv = isReverse ? -stepdiv : stepdiv;
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
		if (rx_char == 'n'){
			turnNorth();
		}else if(rx_char == 's'){
			turnSouth();
		}else if(rx_char == 'k'){
			kill();
		}else if(rx_char >= 32 && rx_char <= 40){
			rx_data = (int)rx_char;
			rx_num1 = (4 - (rx_data-32)) << 7;
			x = pwm_on[PAN] + rx_num1;
			if (x > MAX[PAN]){
				x = MAX[PAN];
			}else if (x < MIN[PAN]){
				x = MIN[PAN];
			}
			change(x,3,PAN);
		}




}

#pragma vector = USCIAB0TX_VECTOR		//UART TX USCI Interrupt
__interrupt void USCI0TX_ISR(void)
{
	if (tx == 1){
		if (TXByteCtr != 0){
			UCB0TXBUF = *(PTxData++);
			TXByteCtr--;
			//			__bic_SR_register_on_exit(CPUOFF);
		}else{
			UCB0CTL1 |= UCTXSTP;
			IFG2 &= ~UCB0TXIFG;
			__bic_SR_register_on_exit(CPUOFF);
		}
	}else{
		RXByteCtr--;                              // Decrement RX byte counter
		if (RXByteCtr)
		{
			*PRxData++ = UCB0RXBUF;
			h[5-RXByteCtr] = UCB0RXBUF;
//
		}
		else
		{
			UCB0CTL1 |= UCTXSTP;                  // Generate I2C stop condition


			loopcount++;

			*PRxData++ = UCB0RXBUF;                   // Move final RX data to PRxData
			h[5-RXByteCtr] = UCB0RXBUF;
//
			unsigned int ciii;
			for (ciii = 0; ciii<3; ciii++){
				coord2[ciii] = h[2*ciii]<<8;
				coord2[ciii] += h[2*ciii+1];
			}
			coordy = (double) coord2[2];
			coordx = (double) coord2[0];
			theta = atan2(coordy,coordx);
			theta = theta * 180 / M_PI;
			if(PRxData > (unsigned char *)0x023C){
				unsigned int ci;
				PRxData = (unsigned char *)0x0200;
				for (ci = 0; ci < 30; ci++){\
					coord[ci] = RxBuffer[2*ci]<<8;
					coord[ci] += RxBuffer[2*ci+1];
				}

			}

			__bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
		}


	}
}



void transmitSetup(){
	tx = 1;
	IE2 &= ~UCB0RXIE;
	    UCB0CTL1 |= UCSWRST; 						// Enable SW reset
	    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;		// I2C Master,sync mode
	    UCB0CTL1 = UCSSEL_2 + UCSWRST; 				// Use SMCLK, keep SW rst
	    UCB0BR0 = 80; 								// fSCL=SMCLK/80=100kHz
	    UCB0BR1 = 0;
	    UCB0I2CSA = 0x1E; 							// Slave Address is 01Eh
	    UCB0CTL1 &= ~UCSWRST; 						// Clear SW rst, resume
	    IE2 |= UCB0TXIE;
}

void receiveSetup(){
	tx = 0;
	IE2 &= ~UCB0TXIE;
	    UCB0CTL1 |= UCSWRST; 						// Enable SW reset
	    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;		// I2C Master,sync mode
	    UCB0CTL1 = UCSSEL_2 + UCSWRST; 				// Use SMCLK, keep SW rst
	    UCB0BR0 = 80; 								// fSCL=SMCLK/80=100kHz
	    UCB0BR1 = 0;
	    UCB0I2CSA = 0x1E; 							// Slave Address is 01Eh
	    UCB0CTL1 &= ~UCSWRST; 						// Clear SW rst, resume
	    IE2 |= UCB0RXIE;
}

void turnNorth(){
	if (theta>-23){
		while(theta>-23){
			change(MAX[STEERING],3,STEERING);
			change(2900,3,MOTOR);
		}
		change(NEUT[STEERING],3,STEERING);
		change(NEUT[MOTOR],3,MOTOR);
	}else{
		while(theta<-23){
			change(MIN[STEERING],3,STEERING);
			change(2900,3,MOTOR);
		}
		change(NEUT[STEERING],3,STEERING);
		change(NEUT[MOTOR],3,MOTOR);
	}
	while((pwm_on[PAN] == NEUT[PAN])&&(pwm_on[STEERING] == NEUT[STEERING])){
		change(2900,3,MOTOR);
	}
	go_init = 1;
}
void turnSouth(){
	if (theta < -158 || theta > 0){
		while((theta < -158 || theta > 0)&&(pwm_on[PAN]!=NEUT[PAN])){
			change(MAX[STEERING],3,STEERING);
			change(2900,3,MOTOR);
		}
		change(NEUT[STEERING],3,STEERING);
		change(NEUT[MOTOR],3,MOTOR);
	}else{
		while((theta > -158 && theta < 0)&&(pwm_on[PAN]!=NEUT[PAN])){
			change(MIN[STEERING],3,STEERING);
			change(2900,3,MOTOR);
		}
		change(NEUT[STEERING],3,STEERING);
		change(NEUT[MOTOR],3,MOTOR);
	}
	while((pwm_on[PAN] == NEUT[PAN])&&(pwm_on[STEERING] == NEUT[STEERING])){
		change(2900,3,MOTOR);
	}
	go_init = 1;

}

void track(){
	if (go_init){
		while(pwm_on[PAN] != NEUT[PAN]){
			if (pwm_on[PAN] < NEUT[PAN]){
				change(MIN[STEERING],3,STEERING);
			} else if(pwm_on[PAN] > NEUT[PAN]){
				change(MAX[STEERING],3,STEERING);
			}
			change(2900,3,MOTOR);
		}
		change(NEUT[STEERING],3,STEERING);
		change(NEUT[MOTOR],3,MOTOR);
		track_init = 1;
	}
}

void follow(){
	if(track_init){
		while(pwm_on[PAN] == NEUT[PAN]){
			change(2900,3,MOTOR);
		}
	}
}


void kill(){
	killed = 1;
	pwm_init();

}





