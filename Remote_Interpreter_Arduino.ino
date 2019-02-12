#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 115200 //Serial Speed DO NOT CHANGE!
#define RC_TOTAL_CHANNELS 3      // TOTAL CHANNELS of RECIEVER, USE 3 for 3pk

#define RC_CH1 0 // DO NOT CHANGE!
#define RC_CH2 1 // DO NOT CHANGE!
#define RC_CH3 3 // DO NOT CHANGE!

#define STEERING RC_CH1 // DO NOT CHANGE!
#define THROTTLE RC_CH2 // DO NOT CHANGE!
#define RE_STOP RC_CH3  // DO NOT CHANGE!

//! Pin Setup: modify by need:
#define RC_CH1_INPUT 52
#define RC_CH2_INPUT 50
#define RC_CH3_INPUT 48

//! Global Variables:
uint16_t rc_values[RC_TOTAL_CHANNELS];
uint32_t rc_start[RC_TOTAL_CHANNELS];
volatile uint16_t rc_shared[RC_TOTAL_CHANNELS];

//! rc_read_values copies value in memory directly.
void rc_read_values()
{
  noInterrupts(); //* Start of the interrupt
  //* memcpy is used for copying stuff from memory directly to achieve speed. BE CAREFUL!
  //REF: void* memcpy( void* dest, const void* src, std::size_t count );
  memcpy((void *)rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts(); //* End of the interrupt
}

//! calc_input is the generic pulse length counter.
void calc_input(uint8_t channel, uint8_t input_pin)
{
  if (digitalRead(input_pin) == HIGH)
  {
    rc_start[channel] = micros();
  }
  else
  {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

//! The following are channel specific:
void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }

//! used for converting the RC obtained pulse to servo's pulse
int pulse2degree()
{
  //* map is used for easier converting 1ms~2ms for remote to 700us~2300us for servo
  //map(value, fromLow, fromHigh, toLow, toHigh)
  int val = map(rc_values[STEERING], 1000, 2000, 700, 2300);
  return val;
}

//! MAIN SETUP
void setup()
{
  Serial.begin(SERIAL_PORT_SPEED);

  pinMode(RC_CH1_INPUT, INPUT_PULLUP);
  pinMode(RC_CH2_INPUT, INPUT_PULLUP);
  pinMode(RC_CH3_INPUT, INPUT_PULLUP);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
}

//! MAIN LOOP
void loop()
{
  rc_read_values();
  Serial.print("CH1:");
  Serial.print(rc_values[RC_CH1]);
  Serial.print("\t");
  Serial.print("CH2:");
  Serial.print(rc_values[RC_CH2]);
  Serial.print("\t");
  Serial.print("CH3:");
  Serial.print(rc_values[RC_CH3]);
  Serial.print("\t");
  Serial.print("STEERING output:");
  Serial.println(pulse2degree());
}
