// STM32-GPIO
// Usage and other documentation can be found here:
//		https://github.com/wooni005/stm32_gpio
// 2020-01-10 Arjan Wooning

#include <EEPROM.h>
#include "Adafruit_BMP085.h"

static char cmd;
static word value;
static byte stack[10], top, sendLen, dest;

#define RELAY_OFF 0
#define RELAY_ON 1
#define FALSE 0
#define TRUE 1

#define SERIAL_BAUD 57600

#define LED_PIN       PC13
#define I2C_SCL_PIN   PB6
#define I2C_SCL_INDEX 15
#define I2C_SDA_PIN   PB7
#define I2C_SDA_INDEX 14

#define PHASE_1_PIN   PA0
#define PHASE_1_INDEX 0
#define PHASE_2_PIN   PA1
#define PHASE_2_INDEX 1
#define PHASE_3_PIN   PA2
#define PHASE_3_INDEX 2

#define NR_OF_PINS sizeof(pinLayout)

char *pinName[] = {
	"PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PA8", "PA9", "PA10", "PA11", "PA12", "PA13", "PA14", "PA15",
	"PB0", "PB1", "PB2", "PB3", "PB4", "PB5", "PB6", "PB7", "PB8", "PB9", "PB10", "PB11", "PB12", "PB13", "PB14", "PB15",
	"PC13", "PC14", "PC15"};

const byte pinLayout[] = {
	// PC13, //On-board green led
	// PC14, //OSC32 IN
	// PC15, //OSC32 OUT
	PA0,  //pinIndex: 00: Also used for PHASE_1_PIN
	PA1,  //pinIndex: 01: Also used for PHASE_2_PIN
	PA2,  //pinIndex: 02: Also used for PHASE_3_PIN
	PA3,  //pinIndex: 03
	PA4,  //pinIndex: 04
	PA5,  //pinIndex: 05
	PA6,  //pinIndex: 06
	PA7,  //pinIndex: 07
	PB0,  //pinIndex: 08
	PB1,  //pinIndex: 09
	PB10, //pinIndex: 10
	PB11, //pinIndex: 11

	PB9, //pinIndex: 12
	PB8, //pinIndex: 13: 
	PB7, //pinIndex: 14: Also used i2c-SDA
	PB6, //pinIndex: 15: Also used i2c-SCL
	PB5, //pinIndex: 16
	// PB4,	//
	// PB3, //Doesn't work: https://community.st.com/s/question/0D50X00009XkZMmSAN/stm32f103-pb3-just-doesnt-work-as-gpio
	// PA15, //
	//	PA12, //Used for USB Serial
	//	PA11, //Used for USB Serial
	PA10, //pinIndex: 17
	PA9,  //pinIndex: 18
	PA8,  //pinIndex: 19
	PB15, //pinIndex: 20
	PB14, //pinIndex: 21
	PB13, //pinIndex: 22
	PB12  //pinIndex: 23
};

#define NR_OF_PHASES 3

static bool gpioActive = false;
byte oldPinStatus[NR_OF_PINS];
byte antiPinBounce[NR_OF_PINS];
byte outputPin[NR_OF_PINS];

// STM32-GPIO configuration
typedef struct
{
	byte nodeId; // Which GPIO node id
} GPIOconfig;

static GPIOconfig config;
static char pin;
static byte checkBounce = false;

static Adafruit_BMP085 bmp;
static bool bmp085Active = false;
static float bmp085Temp;
static int32_t bmp085Pressure;

static bool dimmingActive = false;
static byte phasePinNr[NR_OF_PHASES];
static bool phaseDimmingActive [NR_OF_PHASES];
static bool phaseSSROn [NR_OF_PHASES];
static uint32_t phaseTimerOn [NR_OF_PHASES];
static uint32_t phaseTimerOff [NR_OF_PHASES];
static uint32_t oldPhaseTimer [NR_OF_PHASES];
static uint32_t oldIOTimer = millis();

static void activityLed(byte on)
{
#ifdef LED_PIN
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, !on);
#endif
}

static void setOutput(char pinIndex, byte pinValue)
{
	byte stm32pinNr = pinLayout[pinIndex];
	if (outputPin[pinIndex] == FALSE)
	{
		Serial.print("Init Pin ");
		Serial.print(pinIndex, DEC);
		Serial.println(" is now output");

		outputPin[pinIndex] = TRUE;
		pinMode(stm32pinNr, OUTPUT);
	}
	digitalWrite(stm32pinNr, pinValue);
}

static void readBmp085()
{
	if (!bmp085Active)
	{
		if (!bmp.begin())
		{
			Serial.println("Could not find a BMP085 sensor!");
			bmp085Active = false;
		}
		else
		{
			Serial.println("Found and initialized BMP085 sensor!");
			bmp085Active = true;
		}
	}
	if (bmp085Active)
	{
		bmp085Temp = bmp.readTemperature();
		bmp085Pressure = bmp.readPressure();
		// Serial.print("Temperature = ");
		// Serial.print(bmp085Temp);
		// Serial.println(" *C");
		// Serial.print("Pressure = ");
		// Serial.print(bmp085Pressure);
		// Serial.println(" Pa");

		Serial.print("OK P "); //BMP085 OK
		Serial.print(bmp085Pressure);
		Serial.print(" ");
		Serial.println(bmp085Temp);
	}
	else
	{
		Serial.println("ERROR"); //BMP085 not found
	}
}

static void printOneChar(char c)
{
	Serial.print(c);
}

static void showString(PGM_P s)
{
	for (;;)
	{
		char c = pgm_read_byte(s++);
		if (c == 0)
			break;
		if (c == '\n')
			printOneChar('\r');
		printOneChar(c);
	}
}

static void displayVersion()
{
	Serial.print("[STM32-GPIO.");
	Serial.print(config.nodeId, DEC);
	Serial.println(']');
}

static void saveConfig()
{
	EEPROM.write(0, (int)config.nodeId);
	Serial.print("Saved nodeId: ");
	Serial.println(config.nodeId, DEC);
}

static void loadConfig()
{
	config.nodeId = EEPROM.read(0);

	if (config.nodeId > 7)
	{
		Serial.println("Init EEPROM for first time");
		config.nodeId = 1;
		saveConfig();
	}
}

const char helpText1[] PROGMEM =
	"\n"
	"Available commands:\n"
	" a                - get all I/O values\n"
	" <nn>i            - get input <nn>\n"
	" <on>,<nn>o       - set output <nn> <on> (nn=pinNr 1..16, <on>=0-off/1-on)\n"
	" <nn>n            - set node ID (0..7)\n"
	" <n>l             - set activity led on/off (0: off, 1: on)\n"
	" p                - get temperature and barometer\n"
	" <n>d             - switch on pulldown inputs (0: INPUT, 1: INPUT_PULLDOWN)\n"
    " <n>u             - switch on pullup inputs (0: INPUT, 1: INPUT_PULLUP)\n"
    " <on>,<off>,<n>r  - set SSR switch on/off time [1=10ms] (on=0..100, off=0..100, <n>=phase nr 1..3)\n"
	" t                - for hardware testing check if pin is bouncing\n"
	" v                - display board name and board id\n"
	" h                - this help\n";

static void showHelp()
{
	showString(helpText1);
	+Serial.println();
	Serial.print("nodeId=");
	Serial.println(config.nodeId, DEC);
}

static bool pinInUse(byte pinIndex)
{
	if ((bmp085Active && ((pinIndex == I2C_SDA_INDEX) || (pinIndex == I2C_SCL_INDEX)))
	||  (dimmingActive && ((pinIndex >= PHASE_1_INDEX) && (pinIndex <= PHASE_3_INDEX)))) {
		return true;
	} else {
		return false;
	}
}

static void resetStatusAllInputs()
{
	//Reset all input pins oldPinStatus
	for (int pinIndex = 0; pinIndex < NR_OF_PINS; pinIndex++)
	{
		oldPinStatus[pinIndex] = !digitalRead(pinLayout[pinIndex]);
		antiPinBounce[pinIndex] = oldPinStatus[pinIndex];
	}
}
//Cmnd example: 10,3a
static void handleSerialInput(char c)
{
	byte stm32pinNr = 0;

	if ('0' <= c && c <= '9')
	{
		value = 10 * value + c - '0';
		return;
	}

	if (c == ',')
	{
		if (top < sizeof stack)
			stack[top++] = value; // truncated to 8 bits
		value = 0;
		return;
	}

	if ('a' <= c && c <= 'z')
	{
		showString("> ");
		for (byte i = 0; i < top; ++i)
		{
			Serial.print((word)stack[i]);
			printOneChar(',');
		}
		Serial.print(value);
		Serial.println(c);
	}

	if (c > ' ')
	{
		switch (c)
		{

		case 'n': // set node id
			config.nodeId = value & 0x03;
			saveConfig();
			break;

		case 'a': // get inputs
			// When resetting all input statusses, the statusses will be automatically sended
			resetStatusAllInputs();
			gpioActive = true;
			break;

		case 'i': // get input <nn>
			cmd = c;
			sendLen = top;
			dest = value;
			gpioActive = true;
			break;

		case 'o': // set output <nn>
			cmd = c;
			sendLen = top;
			dest = value;
			// Serial.print("Output nr: "); //Own Pin nr
			// Serial.print(value, DEC); //Own Pin nr
			// Serial.print(":"); //Own Pin nr
			// Serial.println(stack[0], DEC); //Own Pin nr
			setOutput(value, stack[0]);
			gpioActive = true;
			break;

		case 'r': // set dimming switch on/off time phase <n>
			cmd = c;
			sendLen = top;
			// Serial.print("Phase: ");
			// Serial.print(value, DEC); //Phase nr
			// Serial.print("pulseOn: ");
			// Serial.print(stack[0], DEC);
			// Serial.print(", pulseOff: ");
			// Serial.println(stack[1], DEC);
			if ((value < 1) || (value > 3)) {
				Serial.println("ERROR: Phase nr only in range 1..3");
				break;
			}
			if ((stack[0] < 0) || (stack[0] > 100)) {
				Serial.println("ERROR: PulseOn time only in range 0..100");
				break;
			}
			if ((stack[1] < 0) || (stack[1] > 100)) {
				Serial.println("ERROR: PulseOff time only in range 0..100");
				break;
			}
			setDimming(value-1, stack[0], stack[1]);
			break;

		case 'p': // Read temperature and barometer
			readBmp085();
			break;

		case 'd': //Switch on pulldown inputs on/off
			for (int pinIndex = 0; pinIndex < NR_OF_PINS; pinIndex++)
			{ //setting digital inputs with pullup
				stm32pinNr = pinLayout[pinIndex];
				if (!pinInUse(pinIndex)) 
				{
					if (value != 0)
						pinMode(pinLayout[pinIndex], INPUT_PULLDOWN);
					else
						pinMode(pinLayout[pinIndex], INPUT);
					outputPin[pinIndex] = false;
				}
			}
			gpioActive = true;
			break;

		case 'u': //Switch on pullup inputs on/off
			for (int pinIndex = 0; pinIndex < NR_OF_PINS; pinIndex++)
			{ //setting digital inputs with pullup
				stm32pinNr = pinLayout[pinIndex];
				if (!pinInUse(pinIndex)) 
				{
					if (value != 0)
						pinMode(pinLayout[pinIndex], INPUT_PULLUP);
					else
						pinMode(pinLayout[pinIndex], INPUT);
					outputPin[pinIndex] = false;
				}
			}
			gpioActive = true;
			break;

		case 'v': //display the interpreter version and configuration
			displayVersion();
			break;

		case 'l': // turn activity LED on or off
			activityLed(value);
			break;

		case 't': // For hardware testing check if pin is bouncing
			checkBounce = value;
			break;

		case 'h':
			showHelp();
			break;

			// default:
			//		 Serial.print("Unknown input: ");
			//		 Serial.print(c);
			//		 Serial.print(" (0x");
			//		 Serial.print(c, HEX);
			//		 Serial.println(")");
			// showHelp();
		}
	}
	value = top = 0;
}

static void setDimming(char phase, char pulseOn, char pulseOff)
{
	if (!dimmingActive) 
	{
		// Activate dimming
		dimmingActive = true;
		phasePinNr[0] = PHASE_1_PIN;
		phasePinNr[1] = PHASE_2_PIN;
		phasePinNr[2] = PHASE_3_PIN;
		phaseDimmingActive[0] = false;
		phaseDimmingActive[1] = false;
		phaseDimmingActive[2] = false;
		pinMode(PHASE_1_PIN, OUTPUT);
		pinMode(PHASE_2_PIN, OUTPUT);
		pinMode(PHASE_3_PIN, OUTPUT);
		digitalWrite(PHASE_1_PIN, false);
		digitalWrite(PHASE_2_PIN, false);
		digitalWrite(PHASE_3_PIN, false);
	}

	if (pulseOn != 0) 
	{ // Switch on SSR
		if (!phaseDimmingActive[phase]) 
		{ //Dimming on this phase not activated yet, activate it directly
			phaseDimmingActive[phase] = true; //Activate this phase
			oldPhaseTimer[phase] = millis();
			//Switch immediate the output on
			digitalWrite(phasePinNr[phase], true);
			phaseSSROn[phase] = true;
		}
		// Load the pulse on and off timers and better to decrease the on timer with 1 ms to release the SSR
		// We're not synced by the net frequency and with 9 ms in a 10ms halve cycle, the SSR won't switch a few cycles
		phaseTimerOff[phase] = pulseOff * 10; //Table contains values with [10ms]
		phaseTimerOn[phase]  = pulseOn  * 10; //Table contains values with [10ms]

	} 
	else 
	{ // Switch of SSR
		phaseDimmingActive[phase] = false; //Switch off this phase
		//Switch immediately the output off
		digitalWrite(phasePinNr[phase], false);
		phaseSSROn[phase] = false;
	}
	Serial.print("OK R "); //Status from relais
	Serial.print(phase + 1, DEC); //Phase nr 1...3
	Serial.print(" ");
	Serial.print(phaseTimerOn[phase], DEC);
	Serial.print(" ");
	Serial.println(phaseTimerOff[phase], DEC);
}


void setup()
{
	Serial.begin(SERIAL_BAUD);
	Serial.println();
	loadConfig();
	displayVersion();

	//Set all pins default to INPUT when starting up
	for (int pinIndex = 0; pinIndex < NR_OF_PINS; pinIndex++)
	{ //setting digital inputs with pullup
		pinMode(pinLayout[pinIndex], INPUT_PULLUP);
		outputPin[pinIndex] = false;
	}
	resetStatusAllInputs();
}

void loop()
{
	byte pinStat = 0;
	byte stm32pinNr = 0;

	if (Serial.available())
	{
		handleSerialInput(Serial.read());
	}
	else
	{
		if (dimmingActive) 
		{ // Dimming with SSR is activated
			for (char phase = 0; phase < NR_OF_PHASES; phase++)
			{
				if (phaseDimmingActive[phase]) 
				{ // Dimming on this phase is active
					if (phaseSSROn[phase])
					{ // SSR is on
						if ((millis() - oldPhaseTimer[phase]) >= phaseTimerOn[phase]) {
							//Switch the output off and reload timer
							digitalWrite(phasePinNr[phase], false);
							phaseSSROn[phase] = false;
							oldPhaseTimer[phase] = millis();
						}
					} else {
						// SSR is off
						if ((millis() - oldPhaseTimer[phase]) >= phaseTimerOff[phase]) {
							//Switch the output on and reload timer
							digitalWrite(phasePinNr[phase], true);
							phaseSSROn[phase] = true;
							oldPhaseTimer[phase] = millis();
						}
					}
				}
			}
		}

		if (gpioActive) 
		{ // GPIO is activated, scan for input changes
			if ((millis() - oldIOTimer) >= 10) {
				oldIOTimer = millis();
				for (int pinIndex = 0; pinIndex < NR_OF_PINS; pinIndex++)
				{
					stm32pinNr = pinLayout[pinIndex];
					if (!pinInUse(pinIndex)) 
						pinStat = digitalRead(stm32pinNr);
					else
						pinStat = 0;

					if (antiPinBounce[pinIndex] != pinStat)
					{
						if (checkBounce)
						{
							Serial.print(pinIndex, DEC); //Own Pin nr
							Serial.print(".");
						}
						antiPinBounce[pinIndex] = pinStat;
					}
					else
					{
						// Pin reading is stable voor 100-200ms (not bouncing)
						if (oldPinStatus[pinIndex] != pinStat)
						{
							oldPinStatus[pinIndex] = pinStat;
							Serial.print("OK "); //Status from input
							// Serial.print(config.nodeId, DEC); //NodeId nr
							if (outputPin[pinIndex] == TRUE)
							{
								Serial.print("O ");
							}
							else
							{
								Serial.print("I ");
							}
							Serial.print(pinIndex, DEC); //Own Pin nr
							Serial.print(" ");
							Serial.print(pinName[pinLayout[pinIndex]]); //STM32 Pin name
							Serial.print(" ");
							Serial.println(pinStat);
						}
					}
				}
			}
		}
	}
}
