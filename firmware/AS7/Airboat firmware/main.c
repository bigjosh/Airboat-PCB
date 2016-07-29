/*
 * main.c
 *
 * Created: 6/13/2016 12:25:41 PM
 *  Author: josh.com
 *  Based on Vibe V2 firmware
 */ 


/*

 Note to gentle reader: 
 The target product is encased in a silicone overmold, so there is no practical way to do a "battery pull" reset 
 on the circuit in case of a latch-up. There was also no good way to add a hardware reset button to the product.
 
 Therefore, this is a defensive style designed to keep running and self correct.
 The program is designed to completely reset the MPU every time the motor turns off or the power charger
 is unplugged or the button is held down for too long.
  
 -josh
 
*/



/* 
	FUSE SETTINGS:
	
	All default except for 
	
	CKDIV8 = No set (do not divide clock)
	SUT_CKSEL = WDOSC_128KHZ_6CK_14CK_0MS (Internal 128Khz osc)
	
	We run on the super slow clock to avoid triggering FCC Part 15 compliance.
	
	Note that once these fuses are set, you need to use a 32Khz programming clock or slow to communicate with the chip. 
*/



// CPU speed in Hz. Needed for timing functions.
// This is the default fuse setting and works fine for PWM frequencies up to about 10Khz (1% lowest duty), or 100KHz (10% lowest duty). 
// This suits the current speed settings, but a high clock might be needed for higher PWM frequencies

#define F_CPU 128000						// Name used by delay.h 

#define CYCLES_PER_S F_CPU					// Better name

#define CYCLES_PER_MS (F_CPU/1000UL)		// More convenient unit

#define US_PER_S	1000000					


// ** Outputs

#define WHITE_LED_PORT PORTB
#define WHITE_LED_DDR DDRB
#define WHITE_LED_BIT 0

#define RED_LED_PORT PORTB
#define RED_LED_DDR DDRB
#define RED_LED_BIT 1


// ** Inputs

// * Button

#define BUTTON_PORT PORTB
#define BUTTON_PIN	PINB
#define BUTTON_DDR  DDRB
#define BUTTON_BIT	3
#define BUTTON_INT	PCINT3

// Is button currently pressed? Pin has a pull-up connected to ground though button, so a down reads as 0 on the pin. 
// (Compiles down to a single SBIC instruction)

#define BUTTON_STATE_DOWN()	((BUTTON_PIN & _BV(BUTTON_BIT))==0)

// * CIP

// CIP is the charge-in-progress signal. It s active LOW when battery is charging.
// High when battery is full or when no charger is attached. 
// It is connected to the STAT line from the battery controller
// The STAT signal is open collector, so we must be pull up this line

#define CIP_PORT PORTB
#define CIP_PIN  PINB
#define CIP_DDR	 DDRB
#define CIP_BIT 2
#define CIP_INT PCINT2

// CIP is pulled LOW by battery charger to indicate Charge In Progress
// (Compiles down to a single SBIC instruction)

#define CIP_STATE_ACTIVE()		((CIP_PIN & _BV(CIP_BIT))==0)


// CIP as receive only serial port
// Can can read data send at quickly switched charger voltage by looking at the CIP pin 
// Speed is limited by out timing precision and also how fast the signal can rise and fall
// Pulling the charger jack low for 0 bits makes the signal fall faster

#define SERIAL_BPS			(1200)					// Run nice and slow at 1200 baud
#define SERIAL_BITTIME_US	(1000000UL/SERIAL_BPS)


// * Battery Voltage

// We use a trick to read the battery voltage from the Vcc pin...
// https://wp.josh.com/2014/11/06/battery-fuel-guage-with-zero-parts-and-zero-pins-on-avr/

// Different cutoffs because the drain of the motor on the battery lowers the voltage, which will recover when the 
// motor is turned off

#define LOW_BATTERY_VOLTS_WARMx10	(31-03)		// Low battery cutoff while running, 3.1 volts for battery less the 0.3V diode drop
#define LOW_BATTERY_VOLTS_COLDx10	(33-03)		// Low battery cutoff to turn on   , 3.3 volts for battery less the 0.3V diode drop

#define CHARGER_VOLTAGE_THRESHOLD   (43)		// If we see this voltage or more on Vcc, then we are connected to a charger. One shottkey diode drop between charger and MCU. 


// ** UI

#define BUTTON_DEBOUNCE_TIME_MS 25			// How long to wait for a button press debounce

#define JACK_DEBOUNCE_TIME_MS 100			// How long to wait for a battery charger state change debounce

#define BUTTON_LONG_PRESS_MS 500			// How long to hold down button to be considered a long press rather than a push
											// Long press immediately turns off motor without needing to cycle though remaining speeds

#define BUTTON_TRANSIT_TIMEOUT_S (10)		// How long does the button need to be held down for to enter transit lockout mode?


#define LOW_BATTERY_LED_ONTIME_MS (1000)	// We show low battery by flashing red LED this long before shutting down.
#define LOW_BATTERY_LED_BRIGHTNESS_PCT (100)// We show low battery by flashing red LED this long before shutting down (percent,100=full brightness).

#define BUTTON_FEEDBACK_BRIGHTNESS_PCT (100)// We blink the LED when the button is pressed. Use this brightness (percent,100=full brightness).

#define REBOOT() 	{wdt_enable( WDTO_30MS);while(1);}		// Use watchdog to reset MPU. Note that this is sometimes used to also debounce button so probably not quicker than 32ms
	
#define PCT_TO_255(x) ((x*100)/255)			// Convert a human friendly percent to 0-255 scale

#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>


static void initButton() {
	BUTTON_PORT|=_BV(BUTTON_BIT); // Enable button pull-up
	
}		

static void disableButton() {
	BUTTON_PORT &= ~_BV(BUTTON_BIT); // Disable button pull-up		
}

static void initCIP() {
	
	// Battery Charger status pin setup
	
	CIP_PORT |= _BV( CIP_BIT);				
	// Activate pull-up, prevent floating input
	// This input is connected to an open-drain on the battery charging chip. It pulls low during charging. 
	// If the CIP is actually low, that means we are getting power from the charger so we don't need to worry about
	// draining the battery.	
}

// MOTOR FUNCTIONS
// ===============
// Note that register values are hard coded rather than #defined because they 
// can not just be moved around. They depend on the Timer hardware of the specific chip/pin. 

// Motor is controller by an N-MOSFET which is connected to pin OC1B. 1 output turns motor on, 0 is off. 
									
// Turn the motor completely off- disconnects from PWM generator

static void motorOff(void) {

		
	GTCCR = 0;		// "Timer/Counter Comparator B disconnected from output pin OC1B"	
					// Will revert back to PORT value, which we always keep at zero.
		
}

// Initialize the motor pin. Sets to output mode, which will drive pin LOW.
// Call this as soon as possible after reset to keep the mosfet from floating and turning on the motor.

static void initMotor() {
	
						// On ATTINYx5, OC1B happens to be an alternate function on pin PB4. 
	DDRB |= _BV(4);		// Set pin to output mode. It will already be low because IO ports default to 0 on reset.
	
}

// Set motor PWM

// match sets the duty cycle and should be between 0 and top. 0=completely off, top=full on. 
// top sets the frequency where PWM frequency = F_CPU/top. The minimum resolution allowed is 2-bit (top set to 0x0003).

// Note: also uses OCR1C for TOP function.
// Note: resets all used registers each time from scratch for safety from glitches. This is probably  unnecessary...
// http://electronics.stackexchange.com/questions/139575/how-often-do-avrs-actually-glitch-and-need-a-watch-dog-reset-in-the-real-world/144318


static void setMotorPWM( uint8_t match , uint8_t top , uint8_t prescale ) {
			
	if (match==0) {			// Special case this because the PWM generator still generates a pulse at 0 duty cycle
							// "If the OCR1x is set equal to BOTTOM (0x0000) the output will be a narrow spike for each TOP+1 timer clock cycle."
		
		motorOff();
		
	} else {
		
		//        01234567
		//        ========
		//        1         CTC1            1="When the CTC1 control bit is set (one), Timer/Counter1 is reset to $00 in the CPU clock cycle after a compare match with OCR1C register value."
		//            pppp	CS1[3:0]		prescaler
		
		TCCR1 = _BV(CTC1) | (prescale & (CS10|CS11|CS12|CS13) );
		
		//         1		 PWM1B			1 = Enable PWM B
		//          11       COM1B			11 = Set the OC1B output line on compare match
		GTCCR = 0b011100000;		
		
		OCR1B = match;		
		
		OCR1C = top;	// Counter top value - resets to zero when we get here
		
		// Note that there is a silicon bug that might make this not work on older parts:
		// http://electronics.stackexchange.com/questions/97596/attiny85-pwm-why-does-com1a0-need-to-be-set-before-pwm-b-will-work
												
	}
	
}


// Returns the current Vcc voltage as a fixed point number with 1 implied decimal places, i.e.
// 50 = 5 volts, 25 = 2.5 volts,  19 = 1.9 volts
//
// More info on this technique:
// https://wp.josh.com/2014/11/06/battery-fuel-guage-with-zero-parts-and-zero-pins-on-avr/
//
// On each reading we: enable the ADC, take the measurement, and then disable the ADC for power savings.
// This takes >1ms becuase the internal reference voltage must stabilize each time the ADC is enabled.
// For faster readings, you could initialize once, and then take multiple fast readings, just make sure to
// disable the ADC before going to sleep so you don't waste power. 

static uint8_t readVccVoltage(void) {
	
	
	/*
	By default, the successive approximation circuitry requires an input clock frequency between 50
	kHz and 200 kHz to get maximum resolution.
	*/	
				
	// Enable ADC, set pre-scaller to /8 which will give a ADC clock of 1mHz/8 = 125kHz. 
	
	ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);


	// Select ADC inputs
	// bit    76543210
	// REFS = 00 0     = Vcc used as Vref
	// ADLAR=   0	   = Reading is right adjusted (only 8 most significant bits)
	// MUX  =     1100 = Single ended, 1.1V (Internal Ref) as Vin (Called just Vbg in the datasheets- presumably for Band Gap)
	
	ADMUX = 0b00001100;

	
	/*
		After switching to internal voltage reference the ADC requires a settling time of 1ms before
		measurements are stable. Conversions starting before this may not be reliable. The ADC must
		be enabled during the settling time.
	*/
		
	_delay_ms(1);
				
	
		
	ADCSRA |= _BV(ADSC);				// Start a conversion


	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...
										//..and ignore the result
						

	/*
		17.6.2 The first conversion after switching voltage source may be inaccurate, and the user is advised to discard this result.
	*/


	ADCSRA |= _BV(ADSC);				// Start a conversion


	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 2nd conversion to be ready...
	
		
	/*
		After the conversion is complete (ADIF is high), the conversion result can be found in the ADC
		Result Registers (ADCL, ADCH).		
		
		When an ADC conversion is complete, the result is found in these two registers.
		When ADCL is read, the ADC Data Register is not updated until ADCH is read.		
	*/
	
	// Note we could have used ADLAR left adjust mode and then only needed to read a single byte here
		
	uint8_t low  = ADCL;
	uint8_t high = ADCH; 
	
	uint16_t adc = (high << 8) | low;		// 0<= result <=1023
			
	// Compute a fixed point with 1 decimal place (i.e. 5v= 50)
	//
	// ADC = (Vin*1024)/Vref
	// ADC = (1.1v * 1024) / Vcc
	
	// Vcc   =  (1.1v * 1024) / ADC
	// Vcc10 = ((1.1v * 1024) / ADC ) * 10			->convert to 1 decimal fixed point
	// Vcc10 = ((11   * 1024) / ADC )				->simplify to all 16-bit integer math
				
	uint8_t vccx10 = (uint8_t) ( (11 * 1024) / adc); 
	
	/*	
		Note that the ADC will not automatically be turned off when entering other sleep modes than Idle
		mode and ADC Noise Reduction mode. The user is advised to write zero to ADEN before entering such
		sleep modes to avoid excessive power consumption.
	*/
	
	ADCSRA &= ~_BV( ADEN );			// Disable ADC to save power
	
	return( vccx10 );
	
}


// Set the motor to run at the specified duty cycle and frequency
// The duty cycle is specified at 4.2 volts as a value 0-255. It is adjusted to scale to the actual voltage. 
// Of course if you specify 100% at 4.2v and only 3.8v is available, then it will just give 100% at the current voltage

void updateMotor( uint8_t top, uint8_t prescale, uint8_t normalizedDuty, uint8_t vccx10 ) {
	
	unsigned voltageAdjustedDuty = (((normalizedDuty * 42U ) / vccx10) );		// All dutys are normalized to 4.2 volts, so adjust to the current volatge level. Note that is could overflow an uint16 if the voltage is lower than the normal value. 
	
	unsigned voltageAdjusedMatch = (voltageAdjustedDuty  * top ) / 255UL;			// Covert the duty that is scaled 0-255 to a match that is scaled 0-top.
																					// Match = (duty/255) * top, but we need to stay integer so switch the order
																					// Keep as a long because it could be bigger than an int due to scaling because of a low voltage
	uint8_t match;
	
	if (voltageAdjusedMatch > top ) {		// Battery to low for requested duty, so give it all we've got
		
		match = top; 
		
	} else {
		
		match = voltageAdjusedMatch;		// We know that adjusted duty will fit into uint_16 here because it is less than top which is a uint16
		
	}
			
	setMotorPWM( match , top , prescale );

}


// Predefined motor speed steps
// These were determined empirically using the calibration controller and lots of trial and error
// because there are complicated non-linearities and resonances. You really need to find these on actual
// hardware in the actual product. 

// TODO: Do we need a faster PLL clock to avoid audible PWM?

typedef struct {
	uint8_t normailzedDuty;			// Duty cycle normalized to 4.2 volts Vcc. 0=off, 0xff=full on at 4.2 volts power
	uint8_t top;					// Top value, which determines the PWM frequency where 	f = (F_CPU/prescale)/top
	uint8_t prescale;				
} speedStepStruct;


#define SPEED_STEP_COUNT 4

const speedStepStruct speedSteps[SPEED_STEP_COUNT] PROGMEM = {
	
	{          0,    0 , 1 },			// step 0 = off
	{		  30,  255 , 1 },
	{	      50,  255 , 1 },
	{	      80,  255 , 1 },
	
};


// We use Timer0 for timing functions and also PWMing the LEDs

#define TIMER0_PRESCALER	1

#define TIMER0_STEPS_PER_S	(CYCLES_PER_S/TIMER0PRESCALER)

#define TIMER0_STEPS_PER_CYCLE 256		// 8-bit timer overflow

#define TIMER0_CYCLES_PER_S (TIMER_0_STEPS_PER_CYCLE/TIMER0_STEPS_PER_S)

// With a 1Mhz clock, the cycle rate comes out to 488.3 hertz, which is more than fast enough for no flicker on the LEDs

// Note that this just turns on the timer. For the LEDs to come on, we need to set the control bits to let the compare bits show up on the pins
// Also note that we are running in inverted mode, which means there will be a tiny glitch each cycle at full power (I should have put the LEDs in backwards!)

static void initLEDs() {
	
	// These registers do not need to be explicitly set since these are default values	
	//OCR0A = 0;		// Start with LEDs at 0% duty
	//OCR0B = 0;
		
	//TCNT0 = 0;		// Start timer counter at 0;
	
	TCCR0A =
		_BV(WGM01)  | _BV(WGM00)			// Mode 3 Fast PWM TOP=0xff, update OCRx at BOTTOM
	
	;


	// Set outputs to 1 (LEDs off)
	WHITE_LED_PORT |= _BV(WHITE_LED_BIT);
	RED_LED_PORT |= _BV(RED_LED_BIT);
	
	// Set pins to output mode
	WHITE_LED_DDR |= _BV( WHITE_LED_BIT);
	RED_LED_DDR |= _BV(RED_LED_BIT);
			
	
}


static void enableLEDs() {
		
	TCCR0B = _BV(CS00);			// Enable timer. Clock/1
				
}


static void disableLEDs() {

	TCCR0B = 0;				// No clock, timer stopped. 
		
}

// TODO: Makes these normalized for Vcc voltage? 

// Set brightness of LEDs. 0=off, 255=full on

#define LED_DIM_FACTOR 2		// Reduce LED brightness by this to compensate for missing current limiting resistor. 

static void setGreenLED( uint8_t b ) {
	
	if (b==0)	{	// Off
		
		// Note that we actually disconnect when brightness is 0 because otherwise the timer sends out a narrow spike on wraparound. 
		
		TCCR0A &= ~ ( _BV( COM0A1  ) | _BV( COM0A0 ) );		// Normal port operation, OC0A disconnected (happens to hold true for all modes)
															// This will leave the pin at 1, which was set in initLEDs()
	
	} else {
			
		OCR0A = b/LED_DIM_FACTOR;							// Set the compare register	- double buffered so will update at next top	
		TCCR0A |= ( _BV( COM0A1  ) | _BV( COM0A0 ) );		// Set OC0A on Compare Match, Clear OC0A at BOTTOM (inverting mode)
						
	}		
		
}


// Set brightness of LEDs. 0=off, 255=full on

static void setRedLED( uint8_t b ) {
	
	if (b==0)	{	// Off

		// Note that we actually disconnect when brightness is 0 because otherwise the timer sends out a narrow spike on wraparound.
		
		TCCR0A &= ~ ( _BV( COM0B1  ) | _BV( COM0B0 ) );		// Normal port operation, OC0A disconnected (happens to hold true for all modes)
		
		// This will leave the pin at 1, which was set in initLEDs()
		
	} else {
				
		OCR0B = b/LED_DIM_FACTOR;							// Set the compare register	- double buffered so will update at next top
		TCCR0A |= ( _BV( COM0B1  ) | _BV( COM0B0 ) );		// Set OC0A on Compare Match, Clear OC0A at BOTTOM (inverting mode)
		
	}
	
}

// Set brightness of both red and green LEDs at the same time. 0=off, 255=full on

static void setOrangeLED( uint8_t b ) {
	setGreenLED(b);
	setRedLED(b/2);		// A bit of color correctio since the red seems brighter
}


void setLEDsOff() {
	setGreenLED(0);
	setRedLED(0);
}

// Display a 16 bit value on the red/white LED pins. 
// White=clock, Red=data
// Each bit is 50us wide, MSB first.
// Helpful for debugging.

void servicePort( uint16_t x ) {
	
	RED_LED_DDR |= _BV(RED_LED_BIT);
	WHITE_LED_DDR |= _BV(WHITE_LED_BIT);
	
	uint16_t bitmask=1<<15;
	
	while (bitmask)	{
		
		if (x & bitmask ) {
			RED_LED_PORT |= _BV( RED_LED_BIT);
		}
		
		WHITE_LED_PORT|=_BV(WHITE_LED_BIT);
		
		_delay_us(25);

		WHITE_LED_PORT &= ~_BV(WHITE_LED_BIT);
		RED_LED_PORT &= ~_BV( RED_LED_BIT);
		
		_delay_us(25);
		
		bitmask >>=1;
				
	}	
	
}

// This function is copied right from the data sheet
// Note we can not use the library function because it has a bug that cuases intermitant resets

void WDT_off(void)
{
	wdt_reset();
	/* Clear WDRF in MCUSR */
	MCUSR = 0x00;
	/* Write logical one to WDCE and WDE */
	WDTCR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */
	WDTCR = 0x00;
	}


// Dummy ISRs for the pin change interrupts.
// These will catch and wake on..
// *Change in battery charger status lines
// *Incoming bit on the power port

EMPTY_INTERRUPT( PCINT0_vect );

	// This is a dummy routine. This is here just so the processor has something to do when it wakes up.
	// This will just return back to the main program. 
	// TODO: Figure out how to just put an IRET in the vector table to save time and code space.

	
void motorTest() {
	
	// Set outputs to 1 (LEDs off)
	WHITE_LED_PORT |= _BV(WHITE_LED_BIT);
	RED_LED_PORT |= _BV(RED_LED_BIT);
	
	// Set pins to output mode
	WHITE_LED_DDR |= _BV( WHITE_LED_BIT);
	RED_LED_DDR |= _BV(RED_LED_BIT);

		TCCR1 = 0b10000001;
		
		//         1		 PWM1B			1 = Enable PWM B
		//          11       COM1B			11 = Set the OC1B output line on compare match
		GTCCR = 0b011100000;
		
		OCR1B = 100;
		
		OCR1C = 255;	// Counter top value - resets to zero when we get here
		
		// Silicon bug:
		// http://electronics.stackexchange.com/questions/97596/attiny85-pwm-why-does-com1a0-need-to-be-set-before-pwm-b-will-work
	//	TCCR1 |= (1 << COM1A0);	
	
}

// Blink a byte out the LEDs for diagnostics 
// Starts with orange, then 8 bits High Bit first (white=1, red=0).
// Each blink 500ms followed by 500ms off


void blinkByte( uint8_t b ) {
	
	setRedLED(255);
	setGreenLED(255);
	_delay_ms(200);
	setGreenLED(0);
	setRedLED(0);
	_delay_ms(400);
	
	uint8_t bitmask = 1<<7;
	
	while (bitmask) {
		
		wdt_reset();

		
		if (b & bitmask) {
			setGreenLED(255);					
		} else {
			setRedLED(255);
		}

		_delay_ms(100);
		setGreenLED(0);
		setRedLED(0);
		_delay_ms(400);
		
		bitmask >>=1;
		
	}
	
}


// Read a single byte from the CIP pin. 
// This should be called immediately after the rising edge on CIP.
// Returns 0 on bad data. 

// The timing values are hard coded because they are tightly constrained by hardware limits. More info here...
//

#define BIT_TIMEOUT_US 20000

uint8_t readByte() {
		
	uint8_t b=0;
	uint8_t mask=1<<7;
		
		
	while (mask) {
		
		if (!CIP_STATE_ACTIVE()) return(0);		// We must start each bit with CIP active so we can sync to the falling edge
												
		unsigned int countdown = 	( (F_CPU / US_PER_S ) * BIT_TIMEOUT_US  );	// Just get some basis in real time. We know that each loop pass *must* take at least 1 cycle so this establishes a bottom number of passes to have at least the TIMEOUT
			
		while (CIP_STATE_ACTIVE()) {			// NOw we wait for the falling edge to sync to
			countdown--;
			if (countdown==0) {
				return(0);							// We waited way too long for the falling edge, no valid data here. 
			}
				
		}
				
		_delay_us(2000);					// Put us in the middle of the level sync part of the bit
		
		if (CIP_STATE_ACTIVE()) {			// If we are high here, then it is a 1 bit
			
			b|=mask;
			
			// We are already high, so We can immediately start looking for the next falling sync edge
						
		} else {
			
			// We are low, so we need to wait for the end of this bit so we can start the loop over back at CIP active
			
			_delay_us(1000);			// Put us 500us into the next sync active area. If it is not active here, then the next pass of the loop will abort.
			
		}
		
		mask >>=1;
		
	}	
	
	
	blinkByte(b);
		
	return( b );
	
}

// Read a serial command from the CIP pin. Protocol described here...
// https://github.com/bigjosh/Airboat-PCB/tree/master/Calibration-Controller
// This should be called immediately after the rising edge on CIP...

uint8_t readCommand() {
	
	return(readByte());
	
	
}
	
int main(void)
{
	
	initMotor();				// Initialize the motor port to drive the MOSFET low
								// Do this 1st thing on reset so the MOSFET won't float and accidentally turn on the motor.
		
	// Next, turn on the watchdog before anything else so we have the max protection. 
									
	uint8_t watchDogResetFlag = MCUSR & _BV(WDRF);		/// Save the watchdog flag
	
	MCUSR &= ~ _BV( WDRF );		// Clear the watchdog flag
								// "In safety level 1, WDE is overridden by WDRF in MCUSR...."
								// "This means that WDE is always set when WDRF is set."
								// IF we left this Set, then we could get watchdogged while sleeping
								
	wdt_enable( WDTO_8S );		// Give ourselves 8 seconds before forced reboot			
	


	// TODO: Sort these out so only the proper ones in proper order
	initButton();
	initLEDs();
	enableLEDs();		// DOnt' forget to disable this timer before sleep order else it will use power
	initCIP();			// Enable the CIP pull-up. We will need it from here on out weather we sleep or run.
		
	_delay_us(1);							// Give the pull-ups a second to work	
	
	
				
	if ( !watchDogResetFlag )		{		// Are we coming out of anything except for a WatchDog reset?
		
		// Cold boot, run test mode
				
		// Blink back and forth to show LEDs work and solicit a button press	
		
		//enableLEDs();			// Turn on PWM timer
					
		for(uint8_t i=0;i<100 && !BUTTON_STATE_DOWN(); i++ ) {
			
			setRedLED(255);
			
			for(uint8_t j=0; j<100 && !BUTTON_STATE_DOWN();j++ ) { 
				_delay_ms(1);				
			}
			
			setRedLED(0);
			setGreenLED(255);
			
			for(uint8_t j=0; j<100 && !BUTTON_STATE_DOWN();j++ ) {
				_delay_ms(1);
			}
			
			setGreenLED(0);
					
			wdt_reset();

		}
				
		_delay_ms(BUTTON_DEBOUNCE_TIME_MS);
		
								
		// TODO: Put more code here for some testing and feedback on initial battery connection at the factory.
		
		// TODO: I think we want 
		
		// TODO: RESET here so we come back up with a clean watchdog? I think that is better so next part is always same state
				
	}

											
	// Ready to begin normal operation!		
	
	if (BUTTON_STATE_DOWN())	{		// Possible stuck button?
		
		// If we get here, either we just finished test mode and the button is still down, in which case
		// we are testing to make sure it goes back up 
		
		// Otherwise we just reset and the button was down when we woke, so likely it is stuck down 
		// and that is what caused the reset. In this case, show the user and then eventually disable the button.
		
		// Each pass of this loop takes ~1 sec.
		
		for( uint16_t t=0; (t < BUTTON_TRANSIT_TIMEOUT_S) && BUTTON_STATE_DOWN(); t++ ) {
			
			
			// To indicate that we are in a stuck-button sequence, we will blink the white LED
			// at 50% brightness, 1Hz, 10% duty cycle. This looks different than other states 
			// and also minimizes battery usage (the LED pulls 10+mA) since we might be doing this
			// for many minutes
			
			// Start with LED off because it looks better when we are coming in from a watchdog
			// reset because the button was held down for more than 8 secs. Otherwise user
			// sees an odd blink pattern. 
			
			
			// Leave the white LED off for 900 ms or until the button goes up
			
			for( uint8_t l=0; l<90 && BUTTON_STATE_DOWN() ; l++) {
				_delay_ms(10);
			}
			
			if (BUTTON_STATE_DOWN()) {												// Suppress breif flash when button released
				setOrangeLED( PCT_TO_255(BUTTON_FEEDBACK_BRIGHTNESS_PCT) );
			}
			
			// Leave the white LED on for 100 ms or until the button goes up
			// Could do this as a single _delay_ms(100) but that might feel un-responsive
			
			
			for( uint8_t l=0; l<10 && BUTTON_STATE_DOWN() ; l++) {
				_delay_ms(10);
			}
			
			setLEDsOff();
					
			wdt_reset();		
			
		}
							
		// Debounce the possible button up transition 				
		
		_delay_ms(BUTTON_DEBOUNCE_TIME_MS);		
		
	}
	
	if (BUTTON_STATE_DOWN())	{			// Do we still have a stuck button?


		// Indicate we are entering transit mode with a 1 second fade out of the white LED
		
		_delay_ms(900);			// Don't break the nice visual pattern established durring the lockout sequence
		
		
		uint8_t brightness=255;
		
		while(brightness--) {						
			setOrangeLED(brightness);
			_delay_ms( 1000/255);			// Whole sequence will take about 1 sec
		}
		
		
		// We end with brightness=0 so LED is off.	
		
		disableButton();		// Disable pull up to avoid running the battery down. 
	
		// Do not enable interrupt on button pin change - we will require a charger state change to wake up
		// Since the interrupt is not enabled, the pin will be disconnected during sleep so any floating
		// on it will not waste power.
	
	} else {
	
		// Leave pull-up enabled
	
		PCMSK = _BV(BUTTON_INT);				// Enable interrupt on button pin so we wake on a press
	
	}
			
	
	PCMSK |= _BV(CIP_INT);	// Enable interrupt on change in state-of-charge pin no matter what
		
	GIMSK |= _BV(PCIE);		// Enable pin change interrupt vector. Activates INTs on CIP always and on button only if we are not in stuck button mode.
			
	// Clear pending interrupt flags. This way we will only get an interrupt if something changes
	// after we read it. There is a race condition where something could change between the flag clear and the
	// reads below, so code should be able to deal with possible redundant interrupt and worst case
	// is that we get woken up an extra time and go back to sleep.	
	
	GIFR = _BV(PCIF);						// Clear interrupt flag so we will interrupt on any change after now...
																		
	if ( !CIP_STATE_ACTIVE() && !(readVccVoltage()>=CHARGER_VOLTAGE_THRESHOLD) ) {			// Check if conditions are ALREADY true since we only wake on change....

			
		// Ok, it is bedtime!
		
		//disableLEDs();					// Turn off timer since we don't need LEDs while sleeping and timer will use power
											// TODO: I think shutodnw mode kills the timers. Check and make sure!
												
		set_sleep_mode( SLEEP_MODE_PWR_DOWN );  // Go into deep sleep where only a pin change can wake us.. uses only ~0.1uA!
					
		// GOOD NIGHT!		
		
		// This code disables the Watchdog. Note that we can not use the library wdt_disable() becuase it has a bug
		// that causes intermittent unwanted resets.
		
		// Note interrupts are already clear when we get here, otherwise we would need to worry about getting interrupted between the two following lines

		wdt_disable();
		WDT_off();
		
		sleep_enable();							// "To enter any of the three sleep modes, the SE bit in MCUCR must be written to logic one and a SLEEP instruction must be executed."				
		sei();                                  // Enable global interrupts. "When using the SEI instruction to enable interrupts, the instruction following SEI will be executed before any pending interrupts."		
		sleep_cpu();							// This must come right after the sei() to avoid race condition

		// GOOD MORNING!
		// If we get here, then a button push or change in charger status woke s up....
			
		sleep_disable();						// "To avoid the MCU entering the sleep mode unless it is the programmer's purpose, it is recommended to write the Sleep Enable (SE) bit to one just before the execution of the SLEEP instruction and to clear it immediately after waking up."
		
		cli();									// We are awake now, and do don't care about interrupts anymore (out interrupt routines don't do anything anyway)
				
		wdt_enable( WDTO_8S );			// Re-enable watchdog on wake Give ourselves 8 seconds before reboot
	}
			
	// Ok, now we are running!!!
		
	// Motor speed
	
	uint8_t currentSpeedStep = 0;				// What motor speed setting are we currently on?
			
	while (1)	{		
		
		
		// This main loop runs for as long as the motor is on. 
		// It can be terminated by battery charger change of state, low battery detection, button press back to 0 speed, or long button press
		// All these changes terminate the loop in a reboot. 
		
		// TODO: Detect difference between fully charged and charger unplugged by looking at Vcc voltage
		
		// Here we assume that CIP will always go active whenever the charger is connected, even if the battery is full.
		// This is empirically found to be true. This is handy because there is no good way for us to interrupt on Vcc changes.

				
		if (CIP_STATE_ACTIVE())		{		// Charging?
						
			// The CIP just went active, so lets see if there is data on the port...
			// It only takes a couple ms to check, so user will not notice any delay before the charging pulse starts
			
			if (!readCommand()) {			
												
				// If not, then we are just charging....
			
				motorOff();						//Turn motor off in case were running before plug went in

				_delay_ms( JACK_DEBOUNCE_TIME_MS );		// Probably not need because the filed readCommand would have taken long enough...
																
				uint8_t brightness=0;
				int8_t direction=1;
																
				while (CIP_STATE_ACTIVE())	{	// White LED pulse for as long as we are charging....
				
					setGreenLED(brightness);
				
					if (brightness==255) {
					
						direction=-1;
					
					} else if (brightness==0) {
						
						_delay_ms(100);				// Pause a second at off, also give the charge controller to sense the minimum current without the LED on
				
						direction=1;
					
					}
				
					brightness+=direction;
				
					_delay_ms(1);		// Slows the speed of the ramping LED
								
					wdt_reset();
				
				}
											
				setGreenLED(0);					// Turn it off now, for instant feedback if unplugged 
			
				REBOOT();						// Reboot for good measure
			}
				
		}
				
							
		uint8_t vccx10 = readVccVoltage();				// Capture the current power supply voltage. This takes ~1ms and will be needed multiple times below
		
		if ( vccx10 >= CHARGER_VOLTAGE_THRESHOLD )		{		// End of charge? If we are seeing a Vcc higher than the battery can produce, then we are attached to a charger. IF CIP is not active, then the battery is at end-of-charge.
			
			motorOff();						// Turn motor off in case were running before plug went in
			
			setGreenLED(255);				// White LED full on
			
			_delay_ms( JACK_DEBOUNCE_TIME_MS );
			
			while ( readVccVoltage() >= CHARGER_VOLTAGE_THRESHOLD  ); 	// White LED on for as long as we are charging....
			
			// Note that this will watchdog timeout after 8 seconds and reboot us,
			// After which we will immediately fall right back to here and continue to show the white LED
			// This will cause a brief blink in the LED every 8 seconds, which I think is good.
			
			// Charger unplugged, reboot for good measure
			
			setGreenLED(0);					// Turn it off now, for instant feedback if unplugged						
				
			// All done charing, reboot for good measure
															
			REBOOT();
		}
	
											
		if (vccx10 <= LOW_BATTERY_VOLTS_COLDx10) {
									
			if ( (currentSpeedStep==0) || ( vccx10 <= LOW_BATTERY_VOLTS_WARMx10) ) {	// Motor off, or running and really low?
			
				motorOff();
											
				setGreenLED(0);									// Needed because both LEDs might be on if we are in the middle of a button press
			
				setRedLED(255);
			
				_delay_ms(LOW_BATTERY_LED_ONTIME_MS);			// Show red LED to user to show low battery
				
				while (BUTTON_STATE_DOWN());					// Wait for button to be released if pressed
																// Will watchdog timeout in 8 seconds if stuck, and then stuck button defense will take over
				setRedLED(0);
						
				REBOOT();
				
			}
		}

								
		uint8_t buttonPressedFlag=0;
		
		if (BUTTON_STATE_DOWN())	{		// Button pushed?
			
			setOrangeLED( PCT_TO_255( BUTTON_FEEDBACK_BRIGHTNESS_PCT ));		// A bit of instant user feedback (orange)
			
			_delay_ms(BUTTON_DEBOUNCE_TIME_MS);			// debounce going down...
			
			if ( currentSpeedStep ==0 ) {				// Special case first press turning on instantly
								
				updateMotor( pgm_read_word(&speedSteps[1].top) , pgm_read_word(&speedSteps[1].prescale), pgm_read_word(&speedSteps[1].normailzedDuty), vccx10);		// Set new motor speed

			}
			
			uint16_t buttonDownCount=0;
			
			while (BUTTON_STATE_DOWN()) {			// Wait for button to go back up or longpress timeout
				
				if (buttonDownCount++ >= BUTTON_LONG_PRESS_MS ) {		// Long press? Shut motor off
					
					// The reboot would do both of these anyway, but we do them redundantly here so UI feels responsive-
					// The full reboot cycle takes 100+ ms.
					
					motorOff();
										
					setLEDsOff(0);
					
					REBOOT();
					
					// If the button is still down once we reboot, we will land in the stuck button detection sequence
					// which will blink the LED for 1/10th second every second until either the button goes up
					// or the transit mode button timeout expires
					
				
				}
				
				_delay_ms(1);		// One loop=~1ms
				
			}
			
			// Pressed less than a long press
			
			buttonPressedFlag=1;		// Debounce after setting new motor speed so UI feels responsive
												
			currentSpeedStep++;
			
			if (currentSpeedStep >= SPEED_STEP_COUNT )	{ // Cycled all the Way around?
								
				currentSpeedStep=0;
				
			}
						
		}
			
		updateMotor( pgm_read_word(&speedSteps[currentSpeedStep].top) , pgm_read_word(&speedSteps[1].prescale), pgm_read_word(&speedSteps[currentSpeedStep].normailzedDuty), vccx10);		// Set new motor speed
				
		if (buttonPressedFlag) {
			
			// Button released, LEDs off again
			
			setLEDsOff();
									
			_delay_ms(BUTTON_DEBOUNCE_TIME_MS);		// debounce the button returning back up
			
			
		}
		
		if (currentSpeedStep==0) {		// Either we stepped though the settings back to off, or we got a spurious wake up
			REBOOT();	
		}
							

		// If we get to here, then we check for a low battery and had the chance to reboot if we found one,
		// so ok to postpone reset...		
		
		wdt_reset();
		
	}
	
}
