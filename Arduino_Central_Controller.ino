#include <EnableInterrupt.h>
//! Message Length Definition:
#define MESSAGE_LENGTH 4
#define SERIAL_PORT_SPEED 115200  // Serial Speed DO NOT CHANGE!

//! Pin Setup: modify by need:
#define RC_CH1_INPUT A8
#define RC_CH2_INPUT A9
#define RC_CH3_INPUT A10
#define SWITCH_LEFT 46
#define SWITHC_RIGHT 44
#define LED_RED 25
#define LED_GREEN 27
#define LED_BLUE 23
#define redPin LED_RED
#define greenPin LED_GREEN
#define bluePin LED_BLUE
#define ESTOP_Relay 53

//! RC CONFIGURATION
#define RC_TOTAL_CHANNELS 3  // TOTAL CHANNELS of RECIEVER, USE 3 for 3pk
#define RC_CH1 0             // DO NOT CHANGE!
#define RC_CH2 1             // DO NOT CHANGE!
#define RC_CH3 2             // DO NOT CHANGE!
#define STEERING RC_CH1      // DO NOT CHANGE!
#define THROTTLE RC_CH2      // DO NOT CHANGE!
#define RE_STOP RC_CH3       // DO NOT CHANGE!

//! Global Variables:
uint16_t rc_values[RC_TOTAL_CHANNELS];
uint32_t rc_start[RC_TOTAL_CHANNELS];
volatile uint16_t rc_shared[RC_TOTAL_CHANNELS];
uint8_t messageOut[MESSAGE_LENGTH];  // [state, steering, throttle]
int ledState = LOW;
unsigned long previousMillis = 0;
unsigned long interval = 100;

//! State Machine:
int current_state = 0;
#define EMERGENCY_STOP 0
#define IDLE 1
#define RC_MODE 2
#define AUTONOMOUS_MODE_EN 3

//! ESTOP Seperator:
int ESTOP_INITATOR = -1;  // 1 for physical estop, 2 for wireless estop

//! rc_read_values copies value in memory directly.
void rc_read_values() {
    noInterrupts();  //* Start of the interrupt
    //* memcpy is used for copying stuff from memory directly to achieve speed.
    // BE CAREFUL! REF: void* memcpy( void* dest, const void* src, std::size_t
    // count );
    memcpy((void *)rc_values, (const void *)rc_shared, sizeof(rc_shared));
    interrupts();  //* End of the interrupt
}

//! calc_input is the generic pulse length counter.
void calc_input(uint8_t channel, uint8_t input_pin) {
    if (digitalRead(input_pin) == HIGH) {
        rc_start[channel] = micros();
    } else {
        uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
        rc_shared[channel] = rc_compare;
    }
}

//! The following are channel specific:
void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }

//! used for converting the RC obtained pulse to servo's pulse
int pulse2percentage(int inputVal) {
    //* map is used for easier converting 1ms~2ms for remote to 700us~2300us for
    // servo map(value, fromLow, fromHigh, toLow, toHigh)
    int temp = rc_values[inputVal];
    if (temp <= 950) {
        return 0;
    } else if (temp >= 2050) {
        return 0;
    } else {
        int val = map(temp, 1000, 2000, -100, 100);
        return val;
    }
}

//! Used for controlling the state!
void state_check() {
    if (1 < 0) {
        // RESERVE for Physical Emergency STOP Signal!
        current_state = EMERGENCY_STOP;
        digitalWrite(ESTOP_Relay,LOW);
        ESTOP_INITATOR = 1;
    } else {
        if (rc_values[RE_STOP] > 1500 || rc_values[RE_STOP] < 900) {
            current_state = EMERGENCY_STOP;
            digitalWrite(ESTOP_Relay,LOW);
            ESTOP_INITATOR = 2;
        } else {
            // Serial.print("SWITCH_LEFT: ");
            // Serial.print(digitalRead(SWITCH_LEFT));
            // Serial.print("\t");
            // Serial.print("SWITHC_RIGHT: ");
            // Serial.print(digitalRead(SWITHC_RIGHT));
            // Serial.println("\t");
            if (digitalRead(SWITCH_LEFT) == HIGH &&
                digitalRead(SWITHC_RIGHT) == HIGH) {
                current_state = IDLE;
                digitalWrite(ESTOP_Relay,LOW);
            } else if (digitalRead(SWITCH_LEFT) == LOW &&
                       digitalRead(SWITHC_RIGHT) == HIGH) {
                current_state = RC_MODE;
                digitalWrite(ESTOP_Relay,HIGH);
            } else if (digitalRead(SWITCH_LEFT) == HIGH &&
                       digitalRead(SWITHC_RIGHT) == LOW) {
                current_state = AUTONOMOUS_MODE_EN;
                digitalWrite(ESTOP_Relay,LOW);
                //?Preserved for future multiple autonomous mode enable.
            }
        }
    }
}

//! Used for Printing out the Serial Readout:
void print_recieved() {
    Serial.print("STEERING:");
    Serial.print(rc_values[STEERING]);
    Serial.print("\t");
    Serial.print("THROTTLE:");
    Serial.print(rc_values[THROTTLE]);
    Serial.print("\t");
    Serial.print("RE_STOP:");
    Serial.print(rc_values[RE_STOP]);
    Serial.print("\t");
    Serial.print("STEERING output:");
    Serial.print(pulse2percentage(STEERING));
    Serial.print("\t");
    Serial.print("THROTTLE output: ");
    Serial.print("\t");
    Serial.println(pulse2percentage(THROTTLE));
}

//! Serial Message Composer:
void compose_message() {
    messageOut[0] = current_state;
    messageOut[3] = (uint8_t)0;
    if (current_state == EMERGENCY_STOP || current_state == IDLE) {
        messageOut[1] = (uint8_t)0;
        messageOut[2] = (uint8_t)0;
    } else if (current_state == RC_MODE) {
        int temp_steering = pulse2percentage(STEERING);
        int temp_throttle = pulse2percentage(THROTTLE);
        if (temp_steering < 0) {
            temp_steering = -1 * temp_steering + 100;
        }
        if (temp_throttle < 0) {
            temp_throttle = -1 * temp_throttle + 100;
        }

        messageOut[1] = (uint8_t)temp_steering;
        messageOut[2] = (uint8_t)temp_throttle;
    } else if (current_state == AUTONOMOUS_MODE_EN) {
        messageOut[1] = (uint8_t)0;
        messageOut[2] = (uint8_t)0;
    } else {
        Serial.println("Something Fucked up for message composition....");
        messageOut[1] = (uint8_t)0;
        messageOut[2] = (uint8_t)0;
    }
}

void send_message() {
    Serial.println("Sending Message....");
    Serial1.flush();
    Serial2.flush();
    Serial1.write(messageOut, MESSAGE_LENGTH);
    delay(1);
    Serial2.write(messageOut, MESSAGE_LENGTH);
    delay(1);
}

void set_led(int current_state){
    switch (current_state)
    {
        case EMERGENCY_STOP:
            digitalWrite(redPin,HIGH);
            digitalWrite(greenPin,LOW);
            digitalWrite(bluePin,LOW);
            break;

        case IDLE:
            digitalWrite(redPin,HIGH);
            digitalWrite(greenPin,HIGH);
            digitalWrite(bluePin,HIGH);
            break;

        case RC_MODE:
            digitalWrite(redPin,LOW);
            digitalWrite(greenPin,HIGH);
            digitalWrite(bluePin,LOW);
            break;

        case AUTONOMOUS_MODE_EN:
            digitalWrite(redPin,LOW);
            digitalWrite(greenPin,LOW);
            digitalWrite(bluePin,HIGH);            
            break;
    
        default:
            break;
    }
}

//! MAIN SETUP
void setup() {
    Serial.begin(SERIAL_PORT_SPEED);
    Serial2.begin(SERIAL_PORT_SPEED);
    Serial1.begin(SERIAL_PORT_SPEED);
    //* RC Reciever Configuration
    pinMode(RC_CH1_INPUT, INPUT_PULLUP);
    pinMode(RC_CH2_INPUT, INPUT_PULLUP);
    pinMode(RC_CH3_INPUT, INPUT_PULLUP);
    enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
    enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
    enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);

    //* Switch Configuration
    pinMode(SWITCH_LEFT, INPUT_PULLUP);
    pinMode(SWITHC_RIGHT, INPUT_PULLUP);

    //* LED CONFIGURATION
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);  
    pinMode(ESTOP_Relay, OUTPUT);
}

//! MAIN LOOP
void loop() {
    unsigned long currentMillis = millis();
    rc_read_values();
    state_check();
    switch (current_state) {
        case EMERGENCY_STOP:
            compose_message();
            send_message();
            set_led(current_state);
            Serial.print("EMERGENCY STOP! EMERGENCY STOP! EMERGENCY STOP! \t");
            if (ESTOP_INITATOR == 1) {
                Serial.println("Physical Button!");
            } else if (ESTOP_INITATOR == 2) {
                Serial.println("Remote!");
            } else {
                Serial.println("System Fucked UP!!!");  // Should never get
                                                        // here, but whatever!
            }
            break;

        case IDLE:
            compose_message();
            send_message();
            set_led(current_state);
            Serial.println("NO MODE SET!!! ROBOT IDLE!!!");
            break;

        case RC_MODE:
            compose_message();
            send_message();
            set_led(current_state);
            Serial.println("Remote Control Mode!!!");
            print_recieved();
            break;

        case AUTONOMOUS_MODE_EN:
            compose_message();
            send_message();
            set_led(current_state);
            Serial.println("AUTONOMOUS Mode Enabled!!!");
            break;
        default:
            break;
    }
}
