#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int flex_pin = A10;
const int trigger_out = 41;
const int echo_in = 39;
const int encoderA = 2; // yellow
const int encoderB = 3; // white
const int motor_pin_en = 35;
const int motor_pin_A = 31;
const int motor_pin_B = 33;

float time_since_trigger, distance;
int value;

int ui_var = 0;

// motor stuff
int duty_cycle = 0;
float time_since_last_pid_calculation = 0;
volatile int curr_position = 0;
int target_position = 0;
float last_error, error_sum, error_diff, error = 0;
float p = 0.1;
float i = 0.1;
float d = 0.1;
uint8_t prev_state = 0b00;

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

void flex() {
  value = analogRead(flex_pin);
  value = map(value, 681, 929, 0, 100);
}

void ui(sensors_event_t a, sensors_event_t g, sensors_event_t temp){
  switch (ui_var) {
    case 1:
      Serial.print("Acceleration X: ");
      Serial.print(a.acceleration.x);
      Serial.print(", Y: ");
      Serial.print(a.acceleration.y);
      Serial.print(", Z: ");
      Serial.print(a.acceleration.z);
      Serial.println(" m/s^2");

      Serial.print("Rotation X: ");
      Serial.print(g.gyro.x);
      Serial.print(", Y: ");
      Serial.print(g.gyro.y);
      Serial.print(", Z: ");
      Serial.print(g.gyro.z);
      Serial.println(" rad/s");

      Serial.print("Temperature: ");
      Serial.print(temp.temperature);
      Serial.println(" degC");
      Serial.println("");
      break;
    case 2:
      Serial.print("Flex: ");
      Serial.println(value);
      Serial.println("");
      break;
    case 3: 
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      Serial.println("");
      break;
    default:
//      Serial.println("Please type in 1 to read values from IMU, 2 to read values from flex sensor, and 3 to read values from the ultrasonic sensor");
      break;
    Serial.println("");
  }
}

void encoder(void) {
  // based on the waveform i saw on google
  // if (digitalRead(encoderB) == HIGH) {
  //   curr_position++;
  // }
  // else {
  //   curr_position--;
  // }
  uint8_t stateA = digitalRead(encoderA);
  uint8_t stateB = digitalRead(encoderB);
  uint8_t state = (stateB << 1) | stateA;
  if ((prev_state == 0b00 && state == 0b01) ||
      (prev_state == 0b01 && state == 0b11) ||
      (prev_state == 0b11 && state == 0b10) ||
      (prev_state == 0b10 && state == 0b00)) {
      curr_position++;  // Clockwise
  } else if ((prev_state == 0b00 && state == 0b10) ||
              (prev_state == 0b10 && state == 0b11) ||
              (prev_state == 0b11 && state == 0b01) ||
              (prev_state == 0b01 && state == 0b00)) {
      curr_position++;  // Counterclockwise
  }

  curr_position = curr_position % 700;

  prev_state = state;  // Update last state
//  Serial.println(curr_position);
}

void pid_calculations(void) {
  // position pid controller
  noInterrupts();  // Disable interrupts
  int position = curr_position;  // Copy the position to a local variable
  interrupts();  // Re-enable interrupts
  Serial.println(position);

  if (abs(target_position - position) <= 0.1*target_position) {
    Serial.println("here");
    analogWrite(motor_pin_en, 0);
    digitalWrite(motor_pin_A, LOW);
    digitalWrite(motor_pin_B, LOW);
  }
  else {
    float current_time = millis();
    float time = (current_time - time_since_last_pid_calculation) / 1000;
    float current_error = target_position - position;
    error_sum += current_error * time;
    error_diff = (last_error - current_error) / time;

    error = (p * current_error) + (i * error_sum) + (d * error_diff);

    // update
    time_since_last_pid_calculation = current_time;
    last_error = current_error;

    set_motor();
  }
  
}

void set_motor(void) {
  // get output of pid as a percentage
  duty_cycle = error / 700 * 255;
  if (duty_cycle > 255) {
    duty_cycle = 255;
  }
  else if (duty_cycle < 175) {
    duty_cycle = 175;
  }
  analogWrite(motor_pin_en, duty_cycle);
  digitalWrite(motor_pin_A, HIGH);
  digitalWrite(motor_pin_B, LOW);
}

void setup(void) {
  Serial.begin(9600);
  pinMode(flex_pin, INPUT);
  pinMode(trigger_out, OUTPUT);
  pinMode(echo_in, INPUT);
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("Welcome!");

  // encoder shenanigans
  pinMode(encoderA, INPUT); 
  pinMode(encoderB, INPUT); 

  prev_state = (digitalRead(encoderA) << 1) | digitalRead(encoderB);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoder, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(encoderB), encoder, CHANGE);  


  // set target
  target_position = 200;

  // motor shenanigans
  pinMode(motor_pin_en, OUTPUT);
  pinMode(motor_pin_A, OUTPUT);
  pinMode(motor_pin_B, OUTPUT);
}

void loop() {
//  if (Serial.available() > 0) {
//    ui_var = Serial.parseInt();
//  }

//  sensors_event_t a, g, temp;
//  mpu.getEvent(&a, &g, &temp);

//  flex();
//  ultrasonic();
//  ui(a, g, temp);
//  delay(800);
  // pid_calculations();
}
