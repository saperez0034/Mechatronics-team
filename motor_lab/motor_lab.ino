#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
Servo pwmServo;

const int flex_pin = A10;
const int trigger_out = 41;
const int echo_in = 39;
const int encoderA = 21; // yellow
const int encoderB = 20; // white
const int motor_pin_en = 35;
const int motor_pin_A = 31;
const int motor_pin_B = 33;
const int servo_pwm = 9;

float time_since_trigger, distance;
int value;

int ui_var = 0;

// motor stuff
float time_since_last_pid_calculation = 0;
volatile int encoderA_last = LOW;
volatile int curr_position = 0;
volatile int target_position = 0;
float last_error, error_sum, error_diff, error = 0;
float p = 0.1;
float i = 0.1;
float d = 0.1;

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
      // Serial.println("Please type in 1 to read values from IMU, 2 to read values from flex sensor, and 3 to read values from the ultrasonic sensor");
      break;
    Serial.println("");
  }
}

void encoderA_ISR() {
  if (digitalRead(encoderA) == digitalRead(encoderB)) {
    curr_position--;
  } else {
    curr_position++;
  }
}

void encoderB_ISR() {
  if (digitalRead(encoderA) == digitalRead(encoderB)) {
    curr_position++;
  } else {
    curr_position--;
  }
}

void pid_calculations(void) {
  // position pid controller
  float current_time = millis();
  float time = (current_time - time_since_last_pid_calculation) / 1000;
  float current_error = target_position - curr_position;
  error_sum += current_error * time;
  error_diff = (last_error - current_error) / time;
  // Serial.println(current_error);
  // Serial.println(error_sum);
  // Serial.println(error_diff);


  error = (p * current_error) + (i * error_sum) + (d * error_diff);

  // update
  time_since_last_pid_calculation = current_time;
  last_error = current_error;
  set_motor();
}

void set_motor(void) {
  // get output of pid as a percentage
  int duty_cycle = error / 1200 * 255;
  if (duty_cycle > 255) {
    duty_cycle = 255;
  }
  else if (duty_cycle < 100) {
    duty_cycle = 100;
  }
  analogWrite(motor_pin_en, duty_cycle);
  digitalWrite(motor_pin_A, HIGH);
  digitalWrite(motor_pin_B, LOW);
}

void servoControl() {
  int angle = map(distance, 5, 25, 0, 180);
  angle = constrain(angle, 0, 180);
  pwmServo.write(angle);
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
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderA_ISR, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderB_ISR, CHANGE);   

  // set target
  target_position = 500;

  // motor shenanigans
  pinMode(motor_pin_en, OUTPUT);
  pinMode(motor_pin_A, OUTPUT);
  pinMode(motor_pin_B, OUTPUT);
  pwmServo.attach(servo_pwm);
}

void loop() {
  if (Serial.available() > 0) {
    ui_var = Serial.parseInt();
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  flex();
  ultrasonic();
  ui(a, g, temp);
  servoControl();

  // pid_calculations();
  delay(800);
}
