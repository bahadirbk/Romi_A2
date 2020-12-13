#include "linesensor.h"
#include "encoders.h"
#include "pid.h"
#include "kinematics.h"
#include "motors.h"

#include <USBCore.h>
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

#define BAUD_RATE 9600

// Flags used for a basic finite state machine
#define INITIAL_STATE     0
#define FOLLOW_SQUARE     1
#define RESETTING         2

#define SQUARE_EDGE_MM  400

int state;


unsigned long   spd_update_ts; // to sequence our speed update
unsigned long   pid_update_ts; // pid update
unsigned long   pose_print_ts;
unsigned long   initial_state_ts;

// to capture our last encoder count
long            last_e0_count;
long            last_e1_count;

//speed estimates
float e0_spd;
float e1_spd;

//speed demands
float demand_0;
float demand_1;

//powers
float pwr_0;
float pwr_1;

// Gains for motor PID
float Kp_0 = 50; //Proportional gain
float Kd_0 = 0; //Derivative gain
float Ki_0 = 0.165; //Integral gain
float Kp_1 = 50; //Proportional gain
float Kd_1 = 0; //Derivative gain
float Ki_1 = 0.165; //Integral gain

// Class instances
PID_c         left_PID(Kp_0, Ki_0, Kd_0);
PID_c         right_PID(Kp_1, Ki_1, Kd_1);
Kinematics_c  RomiPose;



void beep() {
  analogWrite(6, 80);
  delay(250);
  analogWrite(6, 0);
  delay(250);
}


void setup() {

  setupEncoder0();
  setupEncoder1();

  power.pinModes(); // motors pin allignments

  power.motors(0, 0); //initial motor speeds

  Serial.begin( BAUD_RATE ); //serial communication
  delay(1500);

  // Flag up reset to Serial monitor
  if (SERIAL_ACTIVE)Serial.println("*** RESET ***");

  Calibration(1000);  //calibrating line sensors

  state = INITIAL_STATE; //first value of state


  //timestamps
  pid_update_ts     = millis();
  spd_update_ts     = millis();
  pose_print_ts     = millis();
  initial_state_ts  = millis();


  demand_0 = 0.8;
  demand_1 = 0.8;

  pwr_0     = 0;
  pwr_1     = 0;

  e0_spd = 0;
  e1_spd = 0;

  last_e0_count = 0;
  last_e1_count = 0;

  // We set the robot to start kinematics
  // X = 0, Y = 0, Theta = 0.05
  RomiPose.setPose( 0, 0, 0.05 );

}


// put your main code here, to run repeatedly:
void loop() {
  // Always update kinematics
  RomiPose.update( e0_count, e1_count );
  printPose();  //print pose parameters

  switch (state) {
    case INITIAL_STATE: {
        intialisingBeeps();
        break;
      }
    case FOLLOW_SQUARE: {
        if (RomiPose.theta < 6.2) {
          speed_update(); //get update on speed
          pidUpdate();
          square_run(SQUARE_EDGE_MM, SQUARE_EDGE_MM); //provide lenght of square in mm
        }
        else {
          state = RESETTING;
        }
        break;
      }
    case RESETTING: {
        power.motors(0, 0);
        Serial.println("Resetting...........");
        delay(500);
        break;
      }
    default: {
      
        break;
      }
  }

  //print_count();  //prints the counts for both wheels
  delay(1);

}

void intialisingBeeps() {
  // fuction for state 0
  unsigned long initial_state_dt = millis() - initial_state_ts;
  if ( initial_state_dt < 2000 ) {

    beep();

  } else {

    left_PID.reset();
    right_PID.reset();

    state =   1;
  }
}

void pidUpdate() {
  // output_signal <----PID-- demand, measurement
  unsigned long pid_update_dt = millis() - pid_update_ts;

  if ( pid_update_dt > 10) {
    pid_update_ts = millis();

    pwr_0 = left_PID.update(demand_0, e0_spd);
    pwr_1 = right_PID.update(demand_1, e1_spd);

    power.motors(pwr_0, pwr_1);

  }
}

void square_run(int X, int Y) {
  //X,Y and theta are coordinates from global frame
  //The first "if" is checking if we reached 500 mm in X,
  //the rest two conditions are for interlocking that if once we,
  //reach 500 mm in X then this "if" does not run again
  if ( RomiPose.x < X  && RomiPose.y < 50 && RomiPose.theta < 1  ) {
    //pwr_l,pwr_r are left and right motor PID output
    demand_0 = 0.8;
  }
  //Rotating 90 deg or 1.57 rad, at (X , Y)= (500 , 0) mm.
  //The theta value of 1.3 is to control overshoot
  //as actually romi will rotate more, to around 1.6 radian

  else if (RomiPose.theta < 1.3 ) {
    demand_0 = -0.8; //reversing demand for left wheel to trun on spot
  }
  // Checking if we reach Y=500mm. The other check in theta
  //is for interlocking so that this if does not run once we reach Y=500 mm

  else if (RomiPose.y < Y & RomiPose.theta < 3.2  ) {
    demand_0 = 0.8;
  }
  //Rotating additional 90 deg or 1.57 rad, at (X , Y)= (500 , 500) mm.
  //The theta value of 3 rad is to control overshoot in rotation
  else if (RomiPose.theta < 3 ) {
    demand_0 = -0.8; //reversing demand for left wheel to trun on spot
  }
  //Now we are going back in X direction. We are stopping at x>30
  //to compansate for overshoot. Rest of two conditions are for interlocking
  //so that this "else if" does not run once we reach x=30
  else if (RomiPose.x > 20 && RomiPose.theta > 3 && RomiPose.y > 300 ) {
    demand_0 = 0.8;
  }
  //Rotating additional 90 deg or 1.57 rad, at (X , Y)= (0 , 500) mm.
  //The theta value of 4.5 rad is to control overshoot in rotation
  else if (RomiPose.theta < 4.5 ) {
    demand_0 = -0.8; //reversing demand for left wheel to trun on spot
  }
  //Now we are going back in Y direction. We are stopping at y>30
  //to compansate for undershoot. Rest of  condition is for interlocking
  //so that else if does not run once we reach y = -10
  else if (RomiPose.y > -10 && RomiPose.theta > 4 ) {
    demand_0 = 0.8;
  }
  //Rotating additional 90 deg or 1.57 rad, at (X , Y)= (0 , 0) mm.
  //We get full 2*pi or 6.2 radian rotation here!
  else if ( RomiPose.theta < 6.2 ) {
    demand_0 = -0.8; //reversing demand for left wheel to trun on spot

  }
  // Stopping
  else {
    demand_0 = 0;
    demand_1 = 0;
  }
}

void printPose() {
  unsigned long pose_print_dt = millis() - pose_print_ts;

  if (pose_print_dt > 100) {
    pose_print_ts = millis();
    Serial.print(RomiPose.x);
    Serial.print(",");
    Serial.print(RomiPose.y);
    Serial.print( "," );
    Serial.println( RomiPose.theta );
  }
}


void speed_update() {
  unsigned long spd_update_dt = millis() - spd_update_ts;

  if (spd_update_dt > 20) {
    spd_update_ts = millis();

    long e0_diff = e0_count - last_e0_count;  // calculate differance
    last_e0_count = e0_count; // save for the next loop

    e0_spd = (float)e0_diff / (float)spd_update_dt;

    long e1_diff = e1_count - last_e1_count;  // calculate differance
    last_e1_count = e1_count; // save for the next loop

    e1_spd = (float)e1_diff / (float)spd_update_dt;

    //Serial.print(e0_spd);
    //Serial.print(",");
    //Serial.println(e1_spd);
  }
}

void print_count() {
  Serial.print( e0_count);
  Serial.print( ", ");
  Serial.println( e1_count );

  delay( 2 );
}
