
#include <xc.h>
#include <stdint.h>

/*
Radio transmitter/ receiver 2014-07-31 revised 2016-07-29

DEVICE      12F675 or 12F629
OSCILLATOR  4MHz internal
COMPILER	XC8
*/


/*revision 2016-07-08, want to transmit a mode change to the receiver.
Receive logic will respond thus to a mode change:  it will decode it, set eeprom and clear tenths timeout thus opening the relay, but it won't energise the relay
so I cannot just transmit mode instead of lampON.

Transmit logic will only transmit contents of msgText[] hence added #define MODECHANGE to force it to transmit a mode message when it sees  GP1 held low.
Use GP1 with WPU, active low will set mode03 when transmitting. This sets mode.  Then user can revert to GP1 high and transmitter sending lampON

PICKIT2 settings DIP 8,14,18,20
J1=2, J2-7=right

using PIC12F629 oscal 3458 bandgap 1000.  The tool recalibrated it to 3450 and lost the bandgap setting
*/

eeprom char ver[]="rev 2016-07-29";

#define _XTAL_FREQ    4000000


/*
set config bits do not use brackets.  The options are described in the following file
C:/Program Files/Microchip/xc8/v1.11/docs/chips/12f675.html
Set to use internal 4MHz oscillator, all GP pins as IO
*/
#pragma config BOREN = OFF, CPD = OFF, FOSC = INTRCIO, MCLRE = OFF, WDTE = ON, CP = OFF, PWRTE = OFF


/*CONDITIONAL COMPILATION, see XC8 manual section 2.5.15
generally you leave both RS232 TX&RX in
DECODE defines receiver mode, and if this is enabled you can optionally enable LOOPBACK
which pauses 20sec to allow for RX test char echo, but after which it will send a test pattern
comment out DECODE to create the production burst transmitter build*/
#define RS232TX
#define RS232RX
//#define DECODE 
//#define LOOPBACK

#define MODECHANGE

/*define default boot mode, this can be reprogrammed via rs232*/
eeprom uint8_t EEmode=4;  //only read at boot

/*HARDWARE NOTES
Pin assignments are
[1] Gnd
[2] GP5
[3] GP4
[4] GP3 input only, rs232 RX
[5] GP2 relay output, active hi
[6] GP1 transmit mode03, active low
[7] GP0 rs232 TX
[8] Vcc
*/


//DECLARE FUNCTIONS... none to declare


/*
Create shadow register for GPIO.  Needed because of read-modify-write operations.
usage is sGPIO.port to refer to entire port or sGPIO.RA0 etc for individual bits

Note in the receiver, unless it is driving the relay, the current draw is low therefore its not easy to reboot the MCU by cycling the power because it might take 30 sec for
the capacitor to discharge.  Its better to depower the unit whilst the relay is energised as this will quickly disccharge the cap.
*/
volatile union {
uint8_t		port;
struct {
	unsigned	GP0	:1;   //rs232 transmit
	unsigned	GP1	:1;   //relay monitor IN
	unsigned	GP2	:1;   //relay OUT
	unsigned	GP3	:1;   //rs232 receive, IOC
	unsigned	GP4	:1;
	unsigned	GP5	:1;
	unsigned	GP6	:1;   //not implemented
	unsigned	GP7	:1;   //not implemented
	};
} sGPIO;


/* NOTE on IOC
IOC will be reset if you read or write to the GPIO.  To avoid clearing an IOC condition before its processed in the ISR, we need to avoid reading/writing
the GPIO constantly in the main loop.  It all needs to be done in the ISR including any port polling 
*/


uint8_t	rs232Buffer;  //rs232 data
uint8_t	rs232bitCount;  //bit counter
volatile bit		rs232TXbit; //bit to transmit
volatile bit		rs232bitFlag; //set when bit is pending either in or out
volatile bit		rs232RXbit;  //sampled bit
volatile uint8_t	rs232state;
//rs232 idle is line hi, bits transmit lsb first


volatile bit		relayINbit;  //relay state from PIR device not used, idea was to OR this in
volatile bit		relayLatchBit; //latch status from radio logic

volatile uint8_t    ticks;	//increments every 1200th of a second
volatile unsigned int	tenths;  //increments every 10th of a second
volatile unsigned int	tenthsCountdown;  //decrements every 10th of a second, stops at zero.  Can support 109 mins



//define the relay latch timeout in tenths of a sec, max 0xFFFF for debug purposes set at 300
//assume relay is active high, but can change this, but beware we need to modify the OR to an AND in the merge logic


unsigned int latchTimeout;  //in tenths of a second, can supprt 109 mins



//Declare rs232 state engine
//this is a half-duplex system, with RX taking priority
enum state232
{
TX_START, // start transmit
TX_BIT,  //bit pending
RX_IDLE,  //idle
RX_START,  //start bit seen
RX_BIT, //bit pending
RX_BUFFER, //data has been received user must deal with it
RS232_SHUTDOWN  //hold txBit low and stop transmitting and receiving
};


const char msgText[]="lampON\n\r";
const char mOne[]="mode01\n";  //latch 3 sec for testing
const char mTwo[]="mode02\n";  //latch 5 min
const char mThree[]="mode03\n";  //latch 10 min
const char mFour[]="mode04\n";  //latch 2 min

uint8_t decodeCount;   //decode character counter
uint8_t	msgCount;

uint8_t	m1count; 
uint8_t	m2count;
uint8_t	m3count;
uint8_t	m4count;  



enum stateMode
{
MODE00,
MODE01,
MODE02,
MODE03,
MODE04
};



//MAIN PROGRAM LOOP

void main(void)
{

//boot sequence
//make GP0 an output for rs232TX;
//make GP2 an output for the relay
//GP3 is always an input
//rest are inputs

sGPIO.port = 0xFF; //make all PORTA outputs hi 
GPIO = sGPIO.port;

//GP0 and 2 are outputs
TRISIO=0b11111010;


CMCON = 0x07;  //disable comparator and put in lowest power mode

#ifdef _12F675
ANSEL=0; //set all inputs to digital mode 16F675 device only
#endif
//you also need to set the IOC register to enable intterupt on change to be recognised
//this is done in the RS232 routines

//set up timer 1 with 8 bit prescalar and enable it
//bits <4,5> control prescalar 0=1, 3=8
	
//so we are driving it with 1Mhz or 1 uS periods.  for 1200 baud we count
T1CON=0x01;
//4800 baud is 208 uS, or 200 after 8 cycle adjustment to the point TMR1L is reloaded 0xFF37
//Timer1 triggers an int when it rolls over to zero.  it keeps running during ints

//enable peripheral ints, and int on Timer 1 overflow
INTCONbits.PEIE=1;
PIE1bits.TMR1IE=1;

msgCount=0;
decodeCount=0;
rs232state=RX_IDLE;

tenths=0;
tenthsCountdown=0;
ticks=0;

//enable weakpull up on relayINbit GP1
OPTION_REGbits.nGPPU=0;
WPUbits.WPU1=1;

//boot mode from EEprom, bear in mind this could be corrupt or not initialised
switch (EEmode)
{
case MODE01:
	latchTimeout=30;  //3 sec for debug mode
	break;
case MODE02:
	latchTimeout=3000;  //5 min
case MODE03:
	latchTimeout=6000;  //10 min
	break;
case MODE04:
	latchTimeout=1200;  //2 min
	break;
default:
	//if eeprom is corrupt
	latchTimeout=1200;  //5 min
	EEmode=MODE04;
}

relayLatchBit =1;  //always power up with relay on
tenthsCountdown=latchTimeout;

ei();


//program loop
while(1){


/*Note that we don't use Timer0, so by default PSA=1 and the WDT will use prescalar=111 for approx 2 sec timeout*/
CLRWDT();

//latch timeout handler
if (tenthsCountdown==0){relayLatchBit=0;}  

//assert relay value to port via the ISR. This is done to avoid clearing IOC by accident
sGPIO.GP2=relayLatchBit;

/*LOOPBACK TEST BLOCK*/
#ifdef LOOPBACK
//our debug RS232 code, with message transmit after a 20sec timeout
//2014-07-12 if tenths>200 then means we have no incoming 232 so start transmitting characters
if (tenths>=200){
tenths=200;

//test block, if rs232 is idle, transmit another char from msgText
if (rs232state==RX_IDLE){
rs232Buffer = msgText[msgCount];
msgCount++;

rs232state=TX_START;
//6 letters plus cr lf 
if (msgCount>=8) {msgCount=0;}
}

}//end 20 sec test

if (rs232state==RX_BUFFER){
//simply echo the buffer, but not if DECODE enabled
rs232state=TX_START;
tenths=1;
}
/*END OF LOOPBACK TEST BLOCK*/
#endif

/*
;RS232 notes
;IOC is enabled only if RS232_TX_STATE is 0 (idle)
;RS232_TX_STATE cannot begin execution from 0B unless RS232_RX_STATE=0
;this is what creates half-duplex
;Timer 0 is 208 uS except for RS232_RX_STATE 11 where it is half this at 104 uS to sample mid-bit
;you now trigger TX by setting bit RS232_TX_STATE<7> high, will self zero when done
;RX is valid when RS232_RX_STATE<7> high - user must clear in software
;system looks for N-8-1 but works more reliably when receiving N-8-2
*/


/*
2014-07-13 RX works now that digital inputs are enabled, however if you send JJJ for example, only the first character will transmit properly
this is possibly due to the half duplex nature, as the system is not looking for any input characters whilst it is transmitting.
Its possible to make it full duplex perhaps with an offset register to link the RX edge to the baud clock without changing the baud clock itself which would
disrupt any ongoing TX
*/


#ifdef DECODE
/*DECODE AND RELAY LATCH BLOCK.  Latch works by detecting a valid sequence and setting a decodeTimeout
conditional compiler def DECODE needs to be enabled*/

#ifndef LOOPBACK
if (rs232state==RX_BUFFER){
//character was received over the radio channel

if (msgText[decodeCount] == rs232Buffer) 
	{	//match success, advance counter, not that counter is now 1 ahead of posn of last match
		//e.g. lampON will leave decodeCount = 6 when matched
	decodeCount++;	
	if (decodeCount==6)
		{//matched whole string, set latch
		decodeCount=0;
		tenthsCountdown=latchTimeout;
		relayLatchBit=1;
		}
	}
	else
	{decodeCount=0;} //reset


/*Additional mode switches.  Note the device mode is stored in EEPROM, writes to this complete asynchronously so we cannot read that value
immediately.  For this reason we only read the EEPROM value when we boot, and use a RAM variable to hold curent state.
These mode switches set the latchTimeout value, and update EEmode for next boot.
Note that switching mode will set tenthsCountdown=0 and this will immediately cause the relay to open, thus confirming to the user that their mode
request was indeed set
*/

if (mOne[m1count]==rs232Buffer) 
	{
	m1count++;
	if (m1count==6)
		{m1count=0;
		tenthsCountdown=0;
		latchTimeout=100;  //change to 10 sec for debug (i.e. different to the boot 3 sec) 
		EEmode=MODE01;
		}
	}
	else {m1count=0;}


if (mTwo[m2count]==rs232Buffer) 
	{
	m2count++;
	if (m2count==6)
		{m2count=0;
		tenthsCountdown=0;
		latchTimeout=3000;  //50 mins
		EEmode=MODE02;
		}
	}
	else {m2count=0;}

if (mThree[m3count]==rs232Buffer) {
	m3count++;
	if (m3count==6)
		{m3count=0;
		tenthsCountdown=0;
		latchTimeout=6000;  //10 mins
		EEmode=MODE03;
		}
	}
	else {m3count=0;}

if (mFour[m4count]==rs232Buffer) {
	m4count++;
	if (m4count==6)
		{m4count=0;
		tenthsCountdown=0;
		latchTimeout=1200; //2 min
		EEmode=MODE04;
		}
	}	
	else {m4count=0;}

rs232state=RX_IDLE;  //clear flag allowing us to receive next byte

}  //end rs232 char in buffer	
#endif  //ndef loopback


/*DECODE NOT PRESENT so instead BURST TRANSMIT*/
#else
/*BURST TRANSMIT.  If we are not in loopback test, then we are in production burst transmit
we run the message encoding continuously, the burst is achieved through modulating the tristate of
the outputs

Note: the transmit mode consumes near zero current if DATA is held low.  it draws 15mA at 5v when DATA is high.
this makes sense looking at the module schematic

WE NEED TO DISABLE THE RECEIVE LOGIC because we make use of RX_IDLE and the system will see dummy data from noise on GP3
interestingly if we tie GP3 hi or low the thing stops working
*/

if (tenths<10){
	/*transmit continuously for 1 second.  1200 baud is approx 100 char per sec, so we transmit the lamp on message about 20 times
	note that if GP3 the RX input is tied high or low, it will result in the RX_BUFFER state being reached occasionally so we need
	to release from this also*/
	if (rs232state==RX_IDLE || rs232state==RX_BUFFER){
#ifdef MODECHANGE
		if (GPIObits.GP1==0){
			//if GP1 active low, transmit a mode change message
			rs232Buffer = mThree[msgCount];
		}
		else
		{rs232Buffer = msgText[msgCount];}
#else
		rs232Buffer = msgText[msgCount];
#endif	
		msgCount++;
		rs232state=TX_START; 
		//6 letters + cr lf
		if (msgCount>=8) {msgCount=0;}
	}
}
else
{
	/*disable transmitter by holding GP0 low
	note that we will enter RX_IDLE and this will keep setting GPIE=1 which in turn means the GPIO port will only be updated 10 times per sec */
	rs232state=RS232_SHUTDOWN;
	if (tenths>30){
		tenths=0;
		rs232state=RX_IDLE;  //re-enable transmissions
	}
}

/*END BURST TRANSMIT block*/

#endif //DECODE




/*HALF DUPLEX RS232 HANDLER.  
This handles the state engine and queues bits out and in
To send data, wait for RX_IDLE, load rs232Buffer with data and set state to RX_TXSTART
To receive data, wait for RX_IDLE then watch for RX_BUFFER which indicates a new character has been received into the rs232Buffer

The ISR handles the bit rx/tx and uses Timer 1.  RX makes use of IOC

//http://en.wikipedia.org/wiki/Bitwise_operations_in_C
//http://en.wikipedia.org/wiki/Operators_in_C_and_C%2B%2B
*/

if (rs232bitFlag==0){

switch (rs232state) {

case RX_IDLE:
//receive idle also implies that transmit is idle.  TX idle is logic 1 (line low)
rs232TXbit=1;
IOC=0b00001000;  
/*Any outstanding mismatch will immediately cause an Int, however this will be cleared by the ISR and its preferable to continually clearing
the IOC here because we sit in IDLE a lot and we might miss an edge*/

INTCONbits.GPIE=1;  //enable IOC to look for edge of RX start.  This is the ONLY PLACE where this int is enabled
//note we stay in idle indefinately until external code moves state to TX_start
//or we see the RX pin pulled low in the int routine
break;


case RX_START:
//check the bit state is still 0
if(rs232RXbit==1)
{
//fail, reset and keep looking for an edge
rs232state=RX_IDLE;
}
else
{ //bit was zero, successful start bit seen
rs232bitCount=0;
rs232Buffer=0;
rs232state=RX_BIT;
}
break;

case RX_BIT:
if (rs232bitCount<=7){
if (rs232RXbit==1){rs232Buffer |= (0x01 << rs232bitCount);}  //lsb received first
}
//do nothing, sit around waiting for 1 stop bit, during which time line should go logic 1 again ready for next start
//we don't listen for start until the bit has elapsed
//IMPORTANT:  Since i don't use IOC to detect the exact point the edge falls, there may be some delay if this occurs during
//the main prog loop.  if this is >250 instructions we might lose sync

//NOTE 2014-07-27 once we start receiving chars and see two stop bits, we don't actually resync until we go RX_IDLE after processing the char
//and this might result in loss of sync
else
{//we are at bit 8 or higher expect a SINGLE stop bit
	if (rs232RXbit==1)
		{rs232state=RX_BUFFER;}
	else
		//stop bit was wrong, go idle, dump data and look for sync
		{rs232state=RX_IDLE;}
}
rs232bitCount++;
break;


case TX_START:
//start bit is logic 0 (line hi)
//if external code sets start before flag clears, it simply is ignored, then soon as flag clears, we set start bit
//and transmit for one flag period
rs232TXbit=0;
rs232bitCount=0;
rs232state=TX_BIT;
break;

case TX_BIT:
//bits are transmitted lsb first and the line level is the inverse of the bit itself
// though here i am not inverting the bit...
if (rs232bitCount<=7){
rs232TXbit= (rs232Buffer>>rs232bitCount) & 0x01;  //this has to evaluate to a single bit
}
else
{
//have transmitted all 8 bits, time to send 2 stop bits
//Stop is logic 1 (line low) same as idle
rs232TXbit=1;
}
if (rs232bitCount>=9){rs232state=RX_IDLE;}

rs232bitCount++;
break;

case RS232_SHUTDOWN:
/*disable RX interrupts and take TX low which will put the radio module in a low power state*/
INTCONbits.GPIF=0;
rs232TXbit=0;

} //end switch

//signal to the int routine that the TX needs to go out, or that the next RX bit needs to be handled
rs232bitFlag=1;
} //end bitFlag


}  //end infinite loop
}  //end boot


/*INTERRUPT ROUTINE
Timer 1 is dedicated to RS232 transmit
we use Int on change IOC to detect a start bit on GP3 for rs232RX
and because we are using IOC, we need to do all polling in the ISR to avoid clearing the IOC by accident
and to avoid clearing IOC 1200 times per sec and missing the start of a rs232 transmission, this is reduced to
120 times per sec when in IOC search mode
*/

interrupt ISR(void)
{

if (PIR1bits.TMR1IF)
{
//Timer1 int handler, reset counter with 208 ticks up to 0, i.e. FF30h
// NOTE: look at the compiler .lst file to check the number of operations we execute before we get to the line where the timer
// is reloaded.  this will allow us to fine tune the delay, seems to be about 13 ops

//FCCC is 1200 baud as I doubt radios can handle more reliably.  This is 833 instructions less 13 adjustment
TMR1L=0xCC;
TMR1H=0xFC;

/*we read/write GPIO during this T1 int which means we might miss an RX start edge, but the IOC int itself won't be cleared
meaning the IOC routine can still see the RX start condition when it runs*/

sGPIO.GP0 = rs232TXbit;
GPIO = sGPIO.port;
//sample the RXbit
rs232RXbit=GPIObits.GP3;

//now flag the bit was transmitted or recieved and that the next bit can be sent
rs232bitFlag=0;
PIR1bits.TMR1IF = 0; //clear Timer 1 int flag

//relayINbit=GPIObits.GP1;
//increment ticks, this occurs 1200 times a second with the baud clock
ticks++;
if (ticks>=120){
	ticks=0;
	tenths++;
	tenthsCountdown-=(tenthsCountdown==0)?0:1;  //tenths countdown, stop at zero
}

}


// look for IOC falling edge of RX
#ifdef RS232RX
if (INTCONbits.GPIF)
{
//int on change bit set, would indicate start of RS232 receive
//expect to see expect to see line high = bit 0, and see this again half a bit later at mid-sample
if (GPIObits.GP3==0 && rs232state==RX_IDLE){
	//saw bit 0, set to resample mid bit.  Note that reading GPIO will clear the IOC condition
	rs232bitFlag=1;  //don't process RX_START until we have another Timer1 event (this statement should not be requried)
	rs232state=RX_START;
	//adjust for half a bit at 1200 baud which is 833 halved = 416 then less 13 adjustment = 403
	TMR1L=0x6D;
	TMR1H=0xFE;
	INTCONbits.GPIE=0;  //disable IOC
	//note if IOC resulted in bit 1 then just ignore and look for next IOC
	}
GPIO=sGPIO.port;//write to the port for good measure to clear the IOC
INTCONbits.GPIF=0; //clear the IOC flag

//;start condition, expect to see line high = bit 0 at this mid-bit sample
}
#endif//

}  //end ISR




