#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#include "motors.h"

unsigned long last_ts;
unsigned long time_now;
unsigned long time_elapsed;
int pwr = 40;

Motors_c power;

void setup() {

  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  last_ts = millis();

}

void loop() {

  time_now = millis();

  time_elapsed = time_now - last_ts;


  if (time_elapsed < 3270) {
    power.motors(pwr , pwr);
  }
  else if (time_elapsed < 4770) {
    power.motors(0, pwr);
  }
  else if (time_elapsed < 8040) {
    power.motors(pwr , pwr);
  }
  else if (time_elapsed < 9540) {
    power.motors(0, pwr);
  }
  else if (time_elapsed < 12810) {
    power.motors(pwr , pwr);
  }
  else if (time_elapsed < 14310) {
    power.motors(0, pwr);
  }
  else if (time_elapsed < 17580) {
    power.motors(pwr , pwr);
  }
  else if (time_elapsed < 19080) {
    power.motors(0, pwr);
  }
  else if (time_elapsed > 19080) {
    power.motors(0, 0);
  }

  Serial.print("Time Elapsed: ");
  Serial.print(time_elapsed);
  Serial.print("\n");

}
