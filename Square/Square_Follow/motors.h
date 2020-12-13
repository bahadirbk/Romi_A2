#ifndef _MOTORS_H
#define _MOTORS_H

class Motors_c {

  public:
    void motors(float l_power, float r_power) {
      if ((l_power < 0) && (r_power < 0)) {
        digitalWrite( L_DIR_PIN, HIGH  );
        l_power = abs(l_power);
        digitalWrite( R_DIR_PIN, HIGH  );
        r_power = abs(r_power);
      }
      else if ((l_power > 0) && (r_power > 0) ) {
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
