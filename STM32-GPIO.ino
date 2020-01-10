// ESP-GPIO
#include <EEPROM.h>
#include "Adafruit_BMP085.h"

static char cmd;
static word value;
static byte stack[10], top, sendLen, dest;

#define RELAY_OFF   0
#define RELAY_ON    1
#define FALSE       0
#define TRUE        1

#define SERIAL_BAUD   57600
#define LED_PIN       PC13
#define I2C_SDA_PIN   PB7
#define I2C_SCL_PIN   PB6

#define NR_OF_PINS    sizeof(pinLayout)

char *pinName[] = {
    "PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PA8", "PA9", "PA10", "PA11", "PA12", "PA13", "PA14", "PA15",
    "PB0", "PB1", "PB2", "PB3", "PB4", "PB5", "PB6", "PB7", "PB8", "PB9", "PB10", "PB11", "PB12", "PB13", "PB14", "PB15",
    "PC13", "PC14", "PC15"
};

const byte pinLayout[] = {
  // PC13, //On-board green led
  // PC14, //OSC32 IN
  // PC15, //OSC32 OUT
  PA0,  //00
  PA1,  //01
  PA2,  //02
  PA3,  //03
  PA4,  //04
  PA5,  //05
  PA6,  //06
  PA7,  //07
  PB0,  //08
  PB1,  //09
  PB10, //10
  PB11, //11

  PB9,  //12
  PB8,  //13
  PB7,  //14: Also used for i2c-SDA
  PB6,  //15: Also used for i2c-SCL
  PB5,  //16
  // PB4,  //17
  // PB3, //Doesn't work: https://community.st.com/s/question/0D50X00009XkZMmSAN/stm32f103-pb3-just-doesnt-work-as-gpio
  // PA15, //18
//  PA12, //Used for USB Serial
//  PA11, //Used for USB Serial
  PA10, //17
  PA9,  //18
  PA8,  //19
  PB15, //20
  PB14, //21
  PB13, //22
  PB12  //23
};

byte oldPinStatus[NR_OF_PINS];
byte antiPinBounce[NR_OF_PINS];
byte outputPin[NR_OF_PINS];

// ESP-GPIO configuration
typedef struct {
    byte nodeId;            // Which GPIO node id
} GPIOconfig;

static GPIOconfig config;
static char pin;
static byte checkBounce = false;

static Adafruit_BMP085 bmp;
static byte bmp085Active = false;
static float bmp085Temp;
static int32_t bmp085Pressure;

static void activityLed (byte on) {
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
#endif
}

static void setOutput(char pinIndex, byte pinValue) {
    byte stm32pinNr = pinLayout[pinIndex];
    if (outputPin[pinIndex] == FALSE) {
        Serial.println("Init Pin %d is now output" % pinIndex);
        outputPin[pinIndex] = TRUE;
        pinMode(stm32pinNr, OUTPUT);
    }
    digitalWrite(stm32pinNr, pinValue);
}

static void readBmp085() {
    if (!bmp085Active) {
        if (!bmp.begin()) {
            Serial.println("Could not find a BMP085 sensor!");
            bmp085Active = false;
        } else {
            Serial.println("Found and initialized BMP085 sensor!");
            bmp085Active = true;
        }
    }
    if (bmp085Active) {
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
    } else {
        Serial.println("ERROR"); //BMP085 not found
    }
}

static void printOneChar (char c) {
    Serial.print(c);
}

static void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            printOneChar('\r');
        printOneChar(c);
    }
}

static void displayVersion () {
    Serial.print("[ESP-GPIO.");
    Serial.print(config.nodeId, DEC);
    Serial.println(']');
}

static void saveConfig () {
    EEPROM.write(0, (int)config.nodeId);
    Serial.print("Saved nodeId: ");
    Serial.println(config.nodeId, DEC);
}

static void loadConfig () {
    config.nodeId = EEPROM.read(0);

    if ( config.nodeId > 7) {
        Serial.println("Init EEPROM for first time");
        config.nodeId = 1;
        saveConfig();
    }
}

const char helpText1[] PROGMEM =
    "\n"
    "Available commands:\n"
    "  a           - get all I/O values\n"
    "  <nn> i      - get input <nn>\n"
    "  <on>,<nn> o - set output <nn> <on> (nn=pinNr 1..16, <on>=0-off/1-on)\n"
    "  <nn> n      - set node ID (0..7)\n"
    "  <n> l       - set activity led on/off (0: off, 1: on)\n"
    // "  p           - get temperature and barometer\n"
    "  d           - switch on pulldown inputs (0: INPUT, 1: INPUT_PULLDOWN)\n"
    "  u           - switch on pullup inputs (0: INPUT, 1: INPUT_PULLUP)\n"
    "  t           - for hardware testing check if pin is bouncing\n"
    "  v           - display board name and board id\n"
;

static void showHelp () {
    showString(helpText1);
+    Serial.println();
    Serial.print("nodeId=");
    Serial.println(config.nodeId, DEC);
}

static void resetStatusAllInputs() {
    //Reset all input pins oldPinStatus
    for (int pinIndex = 0; pinIndex < NR_OF_PINS; pinIndex++) {
        oldPinStatus[pinIndex] = !digitalRead(pinLayout[pinIndex]);
        antiPinBounce[pinIndex] = oldPinStatus[pinIndex];
    }
}
//Cmnd example: 10,3a
static void handleSerialInput (char c) {
    byte stm32pinNr = 0;

    if ('0' <= c && c <= '9') {
        value = 10 * value + c - '0';
        return;
    }

    if (c == ',') {
        if (top < sizeof stack)
            stack[top++] = value; // truncated to 8 bits
        value = 0;
        return;
    }

    if ('a' <= c && c <= 'z') {
        showString("> ");
        for (byte i = 0; i < top; ++i) {
            Serial.print((word) stack[i]);
            printOneChar(',');
        }
        Serial.print(value);
        Serial.println(c);
    }

    if (c > ' ') {
        switch (c) {

        case 'n': // set node id
            config.nodeId = value & 0x03;
            saveConfig();
            break;

        case 'a': // get inputs
            // When resetting all input statusses, the statusses will be automatically sended
            resetStatusAllInputs();
            break;

        case 'i': // get input <nn>
            cmd = c;
            sendLen = top;
            dest = value;
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
            break;

        case 'p': // Read temperature and barometer
            readBmp085();
            break;

        case 'd': //Switch on pulldown inputs on/off
            for (int pinIndex = 0; pinIndex < NR_OF_PINS; pinIndex++) { //setting digital inputs with pullup
                stm32pinNr = pinLayout[pinIndex];
                if (!bmp085Active || ((stm32pinNr != I2C_SDA_PIN) && (stm32pinNr != I2C_SCL_PIN))) {
                    if (value != 0) pinMode(pinLayout[pinIndex], INPUT_PULLDOWN);
                    else            pinMode(pinLayout[pinIndex], INPUT);
                    outputPin[pinIndex] = false;
                }
            }
            break;

        case 'u': //Switch on pullup inputs on/off
            for (int pinIndex = 0; pinIndex < NR_OF_PINS; pinIndex++) { //setting digital inputs with pullup
                stm32pinNr = pinLayout[pinIndex];
                if (!bmp085Active || ((stm32pinNr != I2C_SDA_PIN) && (stm32pinNr != I2C_SCL_PIN))) {
                    if (value != 0) pinMode(pinLayout[pinIndex], INPUT_PULLUP);
                    else            pinMode(pinLayout[pinIndex], INPUT);
                    outputPin[pinIndex] = false;
                }
            }
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
        //     Serial.print("Unknown input: ");
        //     Serial.print(c);
        //     Serial.print(" (0x");
        //     Serial.print(c, HEX);
        //     Serial.println(")");
            // showHelp();
        }
    }
    value = top = 0;
}


void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println();
    loadConfig();
    displayVersion();

    //Set all pins default to INPUT when starting up
    for (int pinIndex = 0; pinIndex < NR_OF_PINS; pinIndex++) { //setting digital inputs with pullup
        pinMode(pinLayout[pinIndex], INPUT_PULLUP);
        outputPin[pinIndex] = false;
    }
    resetStatusAllInputs();
}


void loop() {
    byte pinStat = 0;
    byte stm32pinNr = 0;

    if (Serial.available()) {
        handleSerialInput(Serial.read());
    } else {
        for (int pinIndex = 0; pinIndex < NR_OF_PINS; pinIndex++) {
            stm32pinNr = pinLayout[pinIndex];
            if (bmp085Active) {
                if ((stm32pinNr != I2C_SDA_PIN) && (stm32pinNr != I2C_SCL_PIN)) {
                    pinStat = digitalRead(stm32pinNr);
                } else {
                    pinStat = 0;
                }
            } else {
                pinStat = digitalRead(stm32pinNr);
            }

            if (antiPinBounce[pinIndex] != pinStat) {
                if (checkBounce) {
                    Serial.print(pinIndex, DEC); //Own Pin nr
                    Serial.print(".");
                }
                antiPinBounce[pinIndex] = pinStat;
            } else {
                // Pin reading is stable voor 100-200ms (not bouncing)
                if (oldPinStatus[pinIndex] != pinStat) {
                    oldPinStatus[pinIndex] = pinStat;
                    Serial.print("OK "); //Status from input
                    // Serial.print(config.nodeId, DEC); //NodeId nr
                    if (outputPin[pinIndex] == TRUE) {
                        Serial.print("O ");
                    } else {
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
        delay(10); //10ms
    }
}
