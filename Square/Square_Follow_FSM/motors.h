#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15


class Motors_c {

  public:
    void pinModes() {
      pinMode( L_PWM_PIN, OUTPUT );
      pinMode( L_DIR_PIN, OUTPUT );
      pinMode( R_PWM_PIN, OUTPUT );
      pinMode( R_DIR_PIN, OUTPUT );

    }
    void motors(float l_power, float r_power) {
      if ((l_power <= 0) && (r_power <= 0)) {
        digitalWrite( L_DIR_PIN, HIGH  );
        l_power = abs(l_power);
        digitalWrite( R_DIR_PIN, HIGH  );
        r_power = abs(r_power);
      }
      else if ((l_power >= 0) && (r_power >= 0) ) {
        digitalWrite( L_DIR_PIN, LOW  );
        digitalWrite( R_DIR_PIN, LOW  );
      }
      else if ((l_power > 0) && (r_power < 0)) {
        digitalWrite( L_DIR_PIN, LOW  );
        digitalWrite( R_DIR_PIN, HIGH  );
        r_power = abs(r_power);
      }
      else if ((l_power < 0) && (r_power > 0)) {
        digitalWrite( L_DIR_PIN, HIGH  );
        l_power = abs(l_power);
        digitalWrite( R_DIR_PIN, LOW  );
      }
      analogWrite( L_PWM_PIN, l_power );
      analogWrite( R_PWM_PIN, r_power );

    }
};

#endif
