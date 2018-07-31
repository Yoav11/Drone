#include <EnableInterrupt.h>

Servo myservo;

int rcThrottle = 2;
int rcYaw = 6;
int rcPitch = 5;
int rcRoll = 4;


uint16_t rc_values[4];
uint32_t rc_start[4];
volatile uint16_t rc_shared[4];

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() {
  calc_input(0, rcThrottle);
}
void calc_ch2() {
  calc_input(1, rcYaw);
}
void calc_ch3() {
  calc_input(2, rcPitch);
}
void calc_ch4() {
  calc_input(3, rcRoll);
}


void setupRemote() {

  Serial.begin(57600);

  pinMode(rcThrottle, INPUT);
  pinMode(rcPitch, INPUT);
  pinMode(rcRoll, INPUT);
  pinMode(rcYaw, INPUT);

  enableInterrupt(rcThrottle, calc_ch1, CHANGE);
  enableInterrupt(rcYaw, calc_ch2, CHANGE);
  enableInterrupt(rcPitch, calc_ch3, CHANGE);
  enableInterrupt(rcRoll, calc_ch4, CHANGE);


}

void loopRemote() {

  rc_read_values();

  Serial.print("CH1:"); Serial.print(rc_values[0]); Serial.print("\t");
  Serial.print("CH2:"); Serial.print(rc_values[1]); Serial.print("\t");
  Serial.print("CH3:"); Serial.print(rc_values[2]); Serial.print("\t");
  Serial.print("CH4:"); Serial.println(rc_values[3]);

  delay(200);

}

