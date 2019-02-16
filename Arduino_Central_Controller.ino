#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 115200  // Serial Speed DO NOT CHANGE!
//! Pin Setup: modify by need:
#define RC_CH1_INPUT 52
#define RC_CH2_INPUT 50
#define RC_CH3_INPUT 48
#define SWITCH_LEFT 46
#define SWITHC_RIGHT 44

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

//! State Machine:
int current_state = 0;
#define EMERGENCY_STOP 0;
#define IDLE 1;
#define RC_MODE 2;
#define AUTONOMOUS_MODE_EN 3;

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
    int val = map(rc_values[inputVal], 1000, 2000, -100, 100);
    return val;
}

void state_check() {
    if (true) {
        // RESERVE for Physical Emergency STOP Signal!
        current_state = EMERGENCY_STOP;
        ESTOP_INITATOR = 1;
    } else {
        if (rc_values[STEERING] == 0 || rc_values[THROTTLE] == 0) {
            current_state = EMERGENCY_STOP;
            ESTOP_INITATOR = 2;
        } else {
            if (digitalRead(SWITCH_LEFT) == LOW &&
                digitalRead(SWITHC_RIGHT) == LOW) {
                current_state = IDLE;
            } else if (digitalRead(SWITCH_LEFT) == LOW &&
                       digitalRead(SWITHC_RIGHT) == HIGH) {
                current_state = RC_MODE;
            } else if (digitalRead(SWITCH_LEFT) == HIGH &&
                       digitalRead(SWITHC_RIGHT) == LOW) {
                current_state = AUTONOMOUS_MODE_EN;
                //?Preserved for future multiple autonomous mode enable.
            }
        }
    }
}

//! MAIN SETUP
void setup() {
    Serial.begin(SERIAL_PORT_SPEED);

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
}

//! MAIN LOOP
void loop() {
    rc_read_values();

    switch (current_state) {
        case EMERGENCY_STOP:
            Serial.println(
                "EMERGENCY STOP! EMERGENCY STOP! EMERGENCY STOP!") break;

        case IDLE:
            Serial.println("NO MODE SET!!! ROBOT IDLE!!!");

            break;

        case RC_MODE:
            SERIAL.println("Remote Control Mode!!!");
            rc_read_values();
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
            Serial.println(pulse2percentage(STEERING));
            Serial.println("THROTTLE output: ");
            Serial.print("\t");
            Serial.println(pulse2percentage(THROTTLE));

            break;

        default:
            break;
    }
}
