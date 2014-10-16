/*magnetometer
		|				________>y
		|	*SCL		|
		|	*SDA		|
		|	*VCC		|
		|	*GND		\/
		|				x


*/
#include <msp430.h>
#include "stdio.h"
#include <math.h>
#include "stdint.h"
#include "msp430g2553.h"

double coordy;
double coordx;
signed int coord2[3];
//signed int loopcount = 0;
void update(signed int pwm);
void change(int direction, int duration, signed int pwm);
void asdf();
void receiveSetup();
void transmitSetup();
void heading();
void track();
void turn();
void kill();
void search();
void start();
void getTheta();
int us = 0;

const unsigned int sampleTheta = 8;
#define CS(x) _DINT(); x; _EINT();
const char KILLED_CHAR = '~';

unsigned int startSignal = 0;
volatile char h[6];
//unsigned char *PRxData;                     // Pointer to RX data
signed int RXByteCtr;
//volatile unsigned char RxBuffer[60];       // Allocate 128 byte of RAM
//volatile signed int coord[30];
unsigned char *PTxData;                     // Pointer to TX data
signed int TXByteCtr;
const unsigned char TxData[] =              // Table of data to transmit
{
		0x00,
		0x74,
		0x02,
		0x00,
		0x03
};
static const int PWMDIFF = 500;
int test2,test1;
double thetaBuff[sampleTheta];
static const signed int STEERING = 1;
static const signed int MOTOR = 2;
static const signed int PAN = 0;
static volatile signed int pwm_on[3];                      // PWM on duratio
static long int pwm_period[3];                  // Total PWM period - on duration + off duration
static int wanted[3];
static volatile int step[3];
static unsigned state[3] = {0};
static volatile int prevDuty[3];
static const int MAX[3] = {4900, 3950, 3833};
static const int MIN[3] = {1095, 1727, 1916};
static const int NEUT[3] = {3000, 2839, 2874};
static unsigned buttonCount = 0;
volatile unsigned char rx_char = 0;
volatile int rx_data = 0, rx_data1 = 0;
volatile signed int x = 0, rx_num = 0;
int test;
int rx_num1;
int tx = 1;
double theta;
double thetaBuf;
int killed = 0;
int go_init = 0;
int stateM;

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
            test1 = TA1CCR1;
            break;                              //
        case 0x04:                              // - CCR2
            if(state[MOTOR]++ & 1){
            	TA1CCR2 += (pwm_period[MOTOR] - prevDuty[MOTOR]);
            } else{
            	TA1CCR2 += (prevDuty[MOTOR] = pwm_on[MOTOR]);
            	update(MOTOR);
            }
            test2 = TA1CCR2;
            break;                              //
    }
    									//
}                                               //
                                                //
static void pwm_init(void)                      //
{                                               //

    P2DIR |= (BIT3 | BIT1 | BIT4);              // PWM outputs on P2.3-ccr0 pan, P2.1-ccr1 steering, P2.4-ccr2 motor
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
    IE2 |= UCA0TXIE;

    UCA0MCTL = UCBRF_0 | UCBRS_4 ;				// Modulation UCBRSx = 0, UCBRFx = 1, UCOS16 enabled

    UCA0CTL1 &= ~UCSWRST;						// **Initialize USCI state machine**

    IE2 |= UCA0RXIE;					// Enable USCI_A0 RX, USCI_BO RX & TX interrupt


	P1IE |= BIT3; 								// Enable interrupts for P1.3
	P1IES |= BIT3;								// Set P1.3 to trigger on falling edge
	P1IFG &= 0x0;

	UCA0TXBUF &= 0x0;
	UCA0RXBUF &= 0x0;
//	IFG2 &= ~UCA0RXIFG;
                                                //
    TA1CTL = TASSEL_2 | ID_2 | MC_2;            // Setup timer 1 for SMCLK / 4, continuous mode
                                                //
    TA1CCTL0 = TA1CCTL1 = TA1CCTL2 = OUTMOD_4 | CCIE; // Set timer output to toggle mode, enable interrupt
    TA1CCR0 = TA1CCR1 = TA1CCR2 = TA1R + 1000;  // Set initial interrupt time
//    go_init = 1;
//    track_init = 0;
}                                               //

int main(void)                                  //
{
    static const long int PWM_FREQ_COUNT = 39430;

    WDTCTL = 0x5A80;// WDTPW | WDTHOLD;                   //
                                                //
    DCOCTL = 0;                                 // Run DCO at 8 MHz
    BCSCTL1 = CALBC1_8MHZ;                      //
    DCOCTL  = CALDCO_8MHZ;                      //
                                                //
    pwm_init();                                 // Initialize PWM
//    asdf();
                                           // Setup PWM period
    pwm_period[STEERING] = pwm_period[MOTOR] = pwm_period[PAN] = PWM_FREQ_COUNT;
    pwm_on[STEERING] = NEUT[STEERING];                    // Setup servo times

    pwm_on[MOTOR] = NEUT[MOTOR];                       //

    pwm_on[PAN] = NEUT[PAN];                         //
                                                //
//    _enable_interrupts();                       // Enable all interrupts
                                                //

//    PRxData = (unsigned char *)RxBuffer;    // Start of RX buffer
    RXByteCtr = sizeof h-1;                          // Load RX byte counter

    PTxData = (unsigned char *)TxData; // TX array start address
	TXByteCtr = sizeof TxData; // Load TX byte counter
	_EINT();
//	while (TXByteCtr != 0){
//		//TRANSMIT
//		//    	_DINT();
//		transmitSetup();
//		UCB0CTL1 |= UCTR + UCTXSTT; // I2C TX, start condition
//		__bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/interrupts
//    	while (UCB0CTL1 & UCTXSTP); // Ensure stop condition got sent
//
//
////		 Remain in LPM0 until all data are TX'd
//	}
//	if (TXByteCtr == 0){
//		RXByteCtr = 6;                          // Load RX byte counter
//		while (UCB0CTL1 & UCTXSTP); // Ensure stop condition got sent
//		tx = 0;
//		receiveSetup();
//		UCB0CTL1 |= UCTXSTT; // I2C RX, start condition
//		__bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/interrupts
//	}
	stateM = 2;
	_EINT();
    for(;;) {
    	switch (stateM){
			case(1):{
//				getTheta();
				break;
			}
			case(2):{
	//    		go_init = 1;
				track();
				// State transition in UART case:'A' to 3
				break;
			}
			case(3):{
				turn();
//				stateM = 1; TODO: PUT BACK
				break;
			}
    	}

//    	logSetup();
    	// Remain in LPM0 until
    	// all data are RX'd

    }                                           //
}                                               //

/*
 * Changes the steering PWM duty ratio to direction over duration seconds
 */
void change(int direction, int duration, signed int pwm){
	static const signed int PWM_FREQ = 45;
	int diff = 0;
	static signed int div = 0;
	wanted[pwm] = direction;

	diff = wanted[pwm] - pwm_on[pwm];//isReverse ? pwm_on[pwm]-wanted[pwm] : (wanted[pwm] - pwm_on[pwm]);
	diff *= 10;
	div = (duration * PWM_FREQ);
	step[pwm] = diff / div ;

}

void update(signed int pwm){
	signed int newVal = pwm_on[pwm] + step[pwm];
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
//				change(3500, 100, MOTOR);
				wanted[MOTOR] = 3100;
				pwm_on[MOTOR] = 3100;
				TA1CCR2 = 3100;
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
				buttonCount = 0;
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
{//		CS(
		rx_char = UCA0RXBUF;
		switch(rx_char){
		case(KILLED_CHAR):{
			kill();
			break;
		}
		case('A'):{
			stateM = 3;
//			go_init = 0;
//			turn();
			break;
		}
		case('r'):{
			go_init = 1;
		}
		default:{//		case(' '||'!'||'"'||'#'||'$'||'%'||'&'):{
//			if(go_init){
			CS(if (rx_char >= 32 && rx_char <= 38){
			rx_data = (int)rx_char;
			rx_num1 = (35-rx_data) << 8;
			x = pwm_on[PAN] + rx_num1;

			if (x > MAX[PAN]){
				x = MAX[PAN];
			}else if (x < MIN[PAN]){
				x = MIN[PAN];
			}
			change(x,6,PAN))
			break;
			}
//			}
		}
//		default:{break;}
		}
//		)




}

#pragma vector = USCIAB0TX_VECTOR		//UART TX USCI Interrupt
__interrupt void USCI0TX_ISR(void)
{
	static int buffi = 0;
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
		RXByteCtr--;                              	// Decrement RX byte counter
		if (RXByteCtr)
		{
//			*PRxData++ = UCB0RXBUF;					// Put all received data into buffer
			h[5-RXByteCtr] = UCB0RXBUF;				// Simultaneously put received data for current read through i.e. X[MSB] ... Y[LSB] into a smaller array called h
//
		}
		else
		{
			UCB0CTL1 |= UCTXSTP;                  // Generate I2C stop condition



//			*PRxData++ = UCB0RXBUF;                   // Move final RX data to PRxData and h
			h[5-RXByteCtr] = UCB0RXBUF;
//
			signed int ciii;							// counter for converting raw data into headings (from 2 unsigned chars into 1 signed int)
			for (ciii = 0; ciii<3; ciii++){
				coord2[ciii] = h[2*ciii]<<8;			// headings stored into coord2
				coord2[ciii] += h[2*ciii+1];			// shift h[0],h[2],h[4] 8 bits and add h[1],h[3],h[5] to get x,z,y in coord[0],[1],[2] respectively
			}
			coordy = (double) coord2[2];				// store x and y headings into doubles
			coordx = (double) coord2[0];
			theta = atan2(coordy,coordx);				// atan2 to get angle in radians
			thetaBuff[buffi] = theta * 180 / M_PI;					// convert angle to degrees
			buffi = (buffi == sampleTheta-1) ? 0 : buffi+1;

			__bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
		}


	}

}



void transmitSetup(){
	us = 1;
	tx = 1;
	IE2 &= ~UCB0RXIE;
	IE2 &= ~UCA0TXIE;
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
	us = 1;
	tx = 0;
	IE2 &= ~UCB0TXIE;
	IE2 &= ~UCA0TXIE;
	UCB0CTL1 |= UCSWRST; 						// Enable SW reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;		// I2C Master,sync mode
	UCB0CTL1 = UCSSEL_2 + UCSWRST; 				// Use SMCLK, keep SW rst
	UCB0BR0 = 80; 								// fSCL=SMCLK/80=100kHz
	UCB0BR1 = 0;
	UCB0I2CSA = 0x1E; 							// Slave Address is 01Eh
	UCB0CTL1 &= ~UCSWRST; 						// Clear SW rst, resume
	IE2 |= UCB0RXIE;
}

void getTheta(){
//    	for(asdf = 1; asdf<10;asdf++){
//    	heading();
//    	}
//    	while(!go_init);
    	//RECEIVE
//    	while(rx_char);
//			CS(change(NEUT[MOTOR], 3, MOTOR))
    		double sum = 0;
    		int currTheta;
    		for(currTheta = 0; currTheta<sampleTheta-1;currTheta++){
    			heading();
    			sum += thetaBuff[currTheta];
    		}
    		theta = sum/sampleTheta;
//    		heading(); // Refresh magnetometer values
    		if(thetaBuf < -90 || thetaBuf > 90){
    			if (theta > -60 && theta < -30){
    				CS(change(NEUT[MOTOR],3,MOTOR))
    				CS(change(NEUT[STEERING],3,STEERING))
    				CS(change(NEUT[PAN],3,PAN))
    				__delay_cycles(4000000);
    				stateM = 2;
//    				go_init = 1;
    			}else{
    				stateM = 3;
    			}
    		}else {
    			if (theta > -180 && theta < -145){
    				CS(change(NEUT[MOTOR],3,MOTOR))
    				CS(change(NEUT[STEERING],3,STEERING))
    				CS(change(NEUT[PAN],3,PAN))
    				__delay_cycles(4000000);
    				stateM = 2;
//    				go_init = 1;
    			}else{
    				stateM = 3;
    			}
    		}
}

void track(){// if camera not pointing straight, turn car until straight i.e. pwm_on[STEERING] != NEUT[STEERING]

	if (go_init){										// until pwm_on[PAN] = NEUT[PAN] where pan controlled by bbb
		if(wanted[MOTOR] != 2975){
			CS(change(2975,3,MOTOR))
		}
		while(stateM == 2){
			signed int onPan = pwm_on[PAN];
			signed int neutPan = NEUT[PAN];
			signed int ts = (onPan - neutPan);
			ts = ts>>1;
			if (ts > 300 || ts < 300){
				ts = (NEUT[STEERING] - ts)-150;
			}

			if (ts<(MIN[STEERING]+700)){
				ts = MIN[STEERING] + 700;
			}else if(ts>(MAX[STEERING]-700)){
				ts = MAX[STEERING] - 700;
			}

//			CS(change(2975,3,MOTOR))
			CS(change(ts,3,STEERING))
			__delay_cycles(4000000);
		}
//		thetaBuf = theta;
	}

}

void turn(){
//	_EINT();
//	CS(change(2975,3,MOTOR))

	signed int onPan = pwm_on[PAN];
	signed int neutPan = NEUT[PAN];
	signed int ts = (onPan - neutPan);
//	signed int diffpan = ts;
	ts = ts*3;
	ts = ts/5;
//	ts = ts >>1;
	if (pwm_on[PAN] > NEUT[PAN]){
		ts -= PWMDIFF;
	}else{
		ts += PWMDIFF;
	}
	ts = NEUT[STEERING] - ts;
	change(ts,1,STEERING);
	__delay_cycles(4000000);
}




void kill(){

WDTCTL &= 0xFFFF;
}

void start(){
	if(killed){
		killed = 0;
		__bis_SR_register(~CPUOFF);
		main();
	}
}
void search(){
	change(MIN[PAN],3,PAN);
	__delay_cycles(3000000);
	change(MAX[PAN],3,PAN);
	__delay_cycles(3000000);
}

void heading(){
	while (UCB0CTL1 & UCTXSTP);
	if (TXByteCtr == 0){
		tx = 1;
		TXByteCtr = 1;
		transmitSetup();
		PTxData = (unsigned char *)&(TxData[(sizeof TxData)-1]);
		UCB0CTL1 |= UCTR + UCTXSTT; // I2C TX, start condition
		__bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/interrupts
		while (UCB0CTL1 & UCTXSTP); // Ensure stop condition got sent
		RXByteCtr = 6;                          // Load RX byte counter
		tx = 0;
		receiveSetup();
		UCB0CTL1 |= UCTXSTT; // I2C RX, start condition
		__bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/interrupts
	}
}


