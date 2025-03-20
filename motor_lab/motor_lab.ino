#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Stepper.h>
#include <Servo.h>
#include <Encoder.h>

#define MAX_MOTOR_SPEED 800
/** @brief min motor speed */
#define MIN_MOTOR_SPEED 200

#define TICKS_PER_REV 3600

// Pin Declarations
const int flex_pin = A10;
const int trigger_out = 41;
const int echo_in = 39;
const int servo_pwm = 9;
const int button_pin = 27;
const int stepper_1 = 43;
const int stepper_2 = 45;
const int stepper_3 = 47;
const int stepper_4 = 49;
const int button_in = 19;
const int pot_pin = A14;

// Ultrasonic
float time_since_trigger, distance;

// Stepper Motor
const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, stepper_1, stepper_2, stepper_3, stepper_4);
int stepper_steps = 0;
int stepper_steps_prev = 0;

// Servo Motor
Servo pwmServo;

// Flex Sensor
int flex_value = 0;
int flex_value_prev = 0;

int pot_value = 0;

// Variable declarations
int servo_angle;
int stepper_angle;
int dc_position; 
int dc_velocity; 

// Debouncing constants
unsigned long debounce_time = 250;
unsigned long curr_time = 0;
unsigned long prev_time = 0;

volatile int sensor_ctr = 0;

// DC motor
const int encoderA = 2; // yellow
const int encoderB = 3; // white
const int motor_pin_en = 35;
const int motor_pin_A = 31;
const int motor_pin_B = 33;

// motor stuff
int duty_cycle = 0;
float time_since_last_pid_calculation = 0;
volatile int curr_position = 0;
int target_position = 100;
float last_error, error_sum, error_diff, error = 0;
float p = 0.1;
float i = 0.001;
float d = 0.01;
typedef enum {FREE = 0x0, FORWARD = 0x1, BACKWARD = 0x3, STOP = 0x2} encoder_state;
encoder_state state = FREE;

int angle =0;

Encoder motor_encoder(18, 19);

void ultrasonic () {
  digitalWrite(trigger_out, LOW);
  delayMicroseconds(10);
  digitalWrite(trigger_out, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_out, LOW);

  time_since_trigger = pulseIn(echo_in, HIGH);
  // speed of sound --> 343 m/s --> 0.0343 cm/us
  const float speed_of_sound_cm_s = 0.0343;
  // distance sound travels is (duration * 0.0343/us) / 2
  distance = (time_since_trigger*speed_of_sound_cm_s)/2;
}

void servoControl() {
  int angle = map(distance, 5, 25, 0, 180);
  angle = constrain(angle, 0, 180);
  pwmServo.write(angle);
}

void flex() {
  flex_value_prev = flex_value;
  flex_value = analogRead(flex_pin);
  flex_value = map(flex_value, 700, 1000, -100, 100);
  if (abs(flex_value - flex_value_prev) <= 10){ // Filters out any noise, sets it to 0
    stepper_steps = 0;
  }
  else{
    stepper_steps = flex_value * 10;
  }
}

void push_button(){
  curr_time = millis();
  if ((curr_time - prev_time) > debounce_time){
    sensor_ctr ++;
    if (sensor_ctr > 2)
      sensor_ctr = 0;
  }
  prev_time = curr_time;
}

int angle_to_ticks (int angle){
  return map(angle, 0, 360, 0, 3600);
}

void encoder(void) {
  uint8_t a = digitalRead(encoderA);
  uint8_t b = digitalRead(encoderB);
  switch (state) {
    case FREE:
        if (a == 1 && b == 0) {
            curr_position++;
            state = FORWARD;
        } else if (a == 0 && b == 1) {
            curr_position--;
            state = BACKWARD;
        }
        break;
    case FORWARD:
        if (a == 1 && b == 1) {
            curr_position++;
            state = STOP;
        } else if (a == 0 && b == 0) {
            curr_position--;
            state = FREE;
        }
        break;
    case BACKWARD:
        if (a == 1 && b == 1) {
            curr_position--;
            state = STOP;
        } else if (a == 0 && b == 0) {
            curr_position++;
            state = FREE;
        }
        break;
    case STOP:
        if (a == 1 && b == 0) {
            curr_position++;
            state = FORWARD;
        } else if (a == 0 && b == 1) {
            curr_position--;
            state = BACKWARD;
        }
        break;
    default:
        break;
  }

  if (curr_position < 0) {
    curr_position = TICKS_PER_REV;
  }
  if (curr_position > TICKS_PER_REV) {
    curr_position = 0;
  }
}

/** @brief prev error global variable */
volatile int32_t error_prev = 0;
/** @brief cumulative error */
volatile int32_t integral = 0;

void pid_calculations(void) {
  int32_t error = target_position - curr_position;  // Current error
  error_prev = -999999;
  while (!(-50 <= error_prev && error_prev <= 50)){
    if (error_prev == -999999) {
      error_prev = 0;
    }
    if (error > TICKS_PER_REV/2) {
        error -= TICKS_PER_REV;
    } else if (error < -(TICKS_PER_REV/2)) {
        error += TICKS_PER_REV;
    }

    // Calculate PID control
    int32_t derivative = error - error_prev;  // Derivative of error
    integral += error;  // Integral of error
    float output = p * error + i * integral + d * derivative;
    encoder_state dir;

    if (output > 0)
      dir = FORWARD;
    else if (output < 0)
      dir = BACKWARD;
    // Limit motor speed
    if (abs(output) > MAX_MOTOR_SPEED)
        output = MAX_MOTOR_SPEED;
    else if (abs(output) < MIN_MOTOR_SPEED) 
        output = MIN_MOTOR_SPEED;
    error_prev = error;
    if (-50 <= error && error <= 50)
      dir = STOP;
    set_motor(output, dir);
    error = target_position - curr_position;  // Current error
    delay(10);
  }
}

void set_motor(uint32_t duty_cycle, uint32_t direction) {
  // get output of pid as a percentage
  int duty = map(duty_cycle, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED, 0, 255);
  analogWrite(motor_pin_en, duty);
  switch (direction) {
      case FREE:
          digitalWrite(motor_pin_A, LOW);
          digitalWrite(motor_pin_B, LOW);
          break;
      case BACKWARD:
          digitalWrite(motor_pin_A, HIGH);
          digitalWrite(motor_pin_B, LOW);
          break;
      case FORWARD:
          digitalWrite(motor_pin_A, LOW);
          digitalWrite(motor_pin_B, HIGH);
          break;
      case STOP:
          digitalWrite(motor_pin_A, HIGH);
          digitalWrite(motor_pin_B, HIGH);
          break;
      default:
          break;
  }
}

void set_motor_gui(uint32_t duty, uint32_t direction) {
  // get output of pid as a percentage
  analogWrite(motor_pin_en, duty);
  switch (direction) {
      case FREE:
          digitalWrite(motor_pin_A, LOW);
          digitalWrite(motor_pin_B, LOW);
          break;
      case BACKWARD:
          digitalWrite(motor_pin_A, HIGH);
          digitalWrite(motor_pin_B, LOW);
          break;
      case FORWARD:
          digitalWrite(motor_pin_A, LOW);
          digitalWrite(motor_pin_B, HIGH);
          break;
      case STOP:
          digitalWrite(motor_pin_A, HIGH);
          digitalWrite(motor_pin_B, HIGH);
          break;
      default:
          break;
  }
}

void potentiometer() {
  pot_value = analogRead(pot_pin);
  pot_value = map(pot_value, 0, 511, 0, 360);
}

void setup(void) {
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(flex_pin, INPUT);
  pinMode(pot_pin, INPUT);
  pinMode(trigger_out, OUTPUT);
  pinMode(echo_in, INPUT);
  pinMode(button_in, INPUT);
  myStepper.setSpeed(10);
  pwmServo.attach(servo_pwm);
  Serial.println("Welcome!");
  attachInterrupt(digitalPinToInterrupt(button_in), push_button, RISING);

  pinMode(encoderA, INPUT); 
  pinMode(encoderB, INPUT); 

  attachInterrupt(digitalPinToInterrupt(encoderA), encoder, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(encoderB), encoder, CHANGE);  

  pinMode(motor_pin_en, OUTPUT);
  pinMode(motor_pin_A, OUTPUT);
  pinMode(motor_pin_B, OUTPUT);
}

void loop() {

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int separator1 = input.indexOf(',');
    int separator2 = input.indexOf(',', separator1 + 1);
    int separator3 = input.indexOf(',', separator2 + 1);
    
    if (separator1 != -1 && separator2 != -1 && separator3 != -1) {
      String servo_value_str = input.substring(0, separator1);
      String stepper_angle_str = input.substring(separator1 + 1, separator2);
      String dc_position_str = input.substring(separator2 + 1, separator3);
      String dc_velocity_str = input.substring(separator3 + 1);


      servo_angle = servo_value_str.toInt();
      pwmServo.write(servo_angle);

      stepper_angle = stepper_angle_str.toInt();
      stepper_steps = map(stepper_angle, -360, 360, -stepsPerRevolution, stepsPerRevolution);

      dc_position = dc_position_str.toInt();
      dc_velocity = dc_velocity_str.toInt();
      if (dc_position != 0)
        angle = dc_position;
      else{
        if (dc_velocity > 0)
          set_motor_gui(dc_velocity, FORWARD);
        else if (dc_velocity < 0)
          set_motor_gui(abs(dc_velocity), BACKWARD);
        delay(500);
        set_motor_gui(0, STOP);
      }
      delay(1000);
    }
  } 

  else {
    flex();
    potentiometer();
    ultrasonic();
    angle = pot_value;
  }
  
  switch (sensor_ctr){
    case 0:
      Serial.println("stepper");
      myStepper.step(stepper_steps);
      break;
    case 1:
      Serial.println("servo");
      servoControl();
      break;
    case 2:
      Serial.println("encoder");
      target_position = angle_to_ticks(angle);
      pid_calculations();
      break;
  }
  delay(800);
}
