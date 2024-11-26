/* ********** PIN DEFINITION ********** */
// #define LED_BUILTIN PC13

#define DIRECTION_MOTOR_RIGHT 10
#define PWM_MOTOR_RIGHT       9
#define BRAKE_MOTOR_RIGHT     8

#define DIRECTION_MOTOR_LEFT  7
#define PWM_MOTOR_LEFT        6
#define BRAKE_MOTOR_LEFT      5

#define START_STOP_PIN 30
#define MUSIC_BOX_PIN 46


/* ********** GLOBAL VARIABLES DEFINITION ********** */
bool is_right_wheel_cmd = false, is_left_wheel_cmd = false;
bool is_right_wheel_forward = true, is_left_wheel_forward = true;

char value[] = ".....";
uint8_t value_idx = 0;
bool is_cmd_complete = true;


unsigned int right_encoder_counter = 0, left_encoder_counter = 0;
String right_wheel_sign = "p", left_wheel_sign = "p";  // 'p' = positive, 'n' = negative

double right_wheel_meas_vel = 0, left_wheel_meas_vel = 0;
double right_wheel_cmd_vel = 0, left_wheel_cmd_vel = 0;

unsigned long last_millis = 0;
const unsigned long interval = 100;


/* ********** SETUP FUNCTION ********** */
void setup() {
  /* ***** GENERAL CONFIG ***** */
  // analogWriteFrequency(489);

  /* ***** PINS CONFIG ***** */
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(START_STOP_PIN, OUTPUT);
  pinMode(DIRECTION_MOTOR_RIGHT, OUTPUT);
  pinMode(PWM_MOTOR_RIGHT, OUTPUT);
  pinMode(BRAKE_MOTOR_RIGHT, OUTPUT);
  pinMode(DIRECTION_MOTOR_LEFT, OUTPUT);
  pinMode(PWM_MOTOR_LEFT, OUTPUT);
  pinMode(BRAKE_MOTOR_LEFT, OUTPUT);
  pinMode(MUSIC_BOX_PIN, OUTPUT);
  
  /* ***** SERIAL CONFIG ***** */
  Serial.begin(115200);
  Serial.setTimeout(1);

  /* ***** MOTOR CONFIG ***** */
  digitalWrite(START_STOP_PIN, LOW);
  digitalWrite(BRAKE_MOTOR_LEFT, HIGH);
  digitalWrite(BRAKE_MOTOR_RIGHT, HIGH);
  digitalWrite(MUSIC_BOX_PIN, HIGH);

  /* ***** ENCODER CONFIG ***** */


  /* ***** PID CONFIG ***** */

}


/* ********** LOOP FUNCTION ********** */
void loop() {
  /* ***** SERIAL READ (MOTOR) ***** */
  if(Serial.available())
  {
    char chr = Serial.read();
    // Serial.println(chr);

    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        is_left_wheel_forward = true;
      }
    }
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        is_left_wheel_forward = false;
      }
    }
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }

      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '.';
      value[2] = '.';
      value[3] = '.';
      value[4] = '.';
      value[5] = '.';
    }
    else if((chr >= '0' && chr <= '9') || chr == '.')
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
    else if(chr == '\n')
    {
      // Serial.println("RESETTING ...");
    }
    else
    {
      // Serial.println("ERROR MESSAGE, IGNORING ...");
    }
  }


  /* ***** SERIAL WRITE (ENCODER) ***** */
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    String encoder_read = "r" + right_wheel_sign + String(right_encoder_counter) + ",l" + left_wheel_sign + String(left_encoder_counter) + ",";
    right_wheel_meas_vel = (right_encoder_counter * (0.471/30.0)) * 10;
    left_wheel_meas_vel = (left_encoder_counter * (0.471/30.0)) * 10;
    // Serial.println(encoder_read);

    if(is_right_wheel_forward == true) digitalWrite(DIRECTION_MOTOR_RIGHT, LOW);
    else digitalWrite(DIRECTION_MOTOR_RIGHT, HIGH);

    if(is_left_wheel_forward == true) digitalWrite(DIRECTION_MOTOR_LEFT, HIGH);
    else digitalWrite(DIRECTION_MOTOR_LEFT, LOW);

    if(right_wheel_cmd_vel != 0)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(BRAKE_MOTOR_RIGHT, LOW);
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(BRAKE_MOTOR_RIGHT, HIGH);
    }

    if(left_wheel_cmd_vel != 0)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(BRAKE_MOTOR_LEFT, LOW);
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(BRAKE_MOTOR_LEFT, HIGH);
    }

    if(right_wheel_cmd_vel > 255) right_wheel_cmd_vel = 255;
    if(left_wheel_cmd_vel > 255) left_wheel_cmd_vel = 255;

    // Serial.print("rp");
    // Serial.print(int(right_wheel_cmd_vel));
    // Serial.print("lp");
    // Serial.println(int(left_wheel_cmd_vel));

    analogWrite(PWM_MOTOR_RIGHT, int(right_wheel_cmd_vel));
    analogWrite(PWM_MOTOR_LEFT, int(left_wheel_cmd_vel));

    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
  }
}
