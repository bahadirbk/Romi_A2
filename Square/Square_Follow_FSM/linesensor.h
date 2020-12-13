#ifndef _CAL_H
#define _CAL_H

#include "motors.h"
Motors_c power;

#define LEFT_SENSOR  A2
#define MID_SENSOR   A3
#define RIGHT_SENSOR A4

float l_value = 0; // left sensor
float m_value = 0; // mid sensor
float r_value = 0; // right sensor

int thresholding = 200;

float readLeftSensor() {
  return (analogRead(LEFT_SENSOR) - l_value);
}
float readMidSensor() {
  return (analogRead(MID_SENSOR) - m_value);
}

float readRightSensor() {
  return (analogRead(RIGHT_SENSOR) - r_value);
}

boolean isLeftOnLine() {
  return (readLeftSensor() > thresholding) ;
}
boolean isMidOnLine() {
  return (readMidSensor() > thresholding) ;
}
boolean isRightOnLine() {
  return (readRightSensor() > thresholding) ;
}

void Calibration (int rate) {
  for (int i = 0; i < rate; i++) {
    l_value += analogRead(LEFT_SENSOR);
    m_value += analogRead(MID_SENSOR);
    r_value += analogRead(RIGHT_SENSOR);
  }

  l_value /= (float)rate;
  m_value /= (float)rate;
  r_value /= (float)rate;

}



void lineFollow() {

  float ITotal = readLeftSensor() + readMidSensor() + readRightSensor();
  float PLeft  = readLeftSensor() / ITotal;
  float PRight = readRightSensor() / ITotal;

  float MPower = (PLeft - PRight) * 125.0f /*Max motor power*/;

  Serial.print("left: "); Serial.print(readLeftSensor());
  Serial.print(" Mid: "); Serial.print(readMidSensor());
  Serial.print(" Right: "); Serial.println(readRightSensor());

  if (isLeftOnLine() && isRightOnLine()) {
    power.motors(30,30); // left and right motor respectively
  } else {
     power.motors(-MPower, MPower);
  }

  delay(20);
}

#endif
