#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;

// PID constants
float Kp_pitch = 0.5;
float Ki_pitch = 0.01;
float Kd_pitch = 0.1;
float Kp_altitude = 0.5;
float Ki_altitude = 0.01;
float Kd_altitude = 0.1;
float Kp_yaw = 0.5;
float Ki_yaw = 0.01;
float Kd_yaw = 0.1;

// PID variables
float pitch_error, last_pitch_error, pitch_integral, pitch_derivative, pitch_pid_output;
float altitude_error, last_altitude_error, altitude_integral, altitude_derivative, altitude_pid_output;
float yaw_error, last_yaw_error, yaw_integral, yaw_derivative, yaw_pid_output;

// Motor pins
int motor1 = 3;
int motor2 = 5;
int motor3 = 6;
int motor4 = 9;

// Altitude sensor pin
int altitude_pin = A0;

// Receiver pins
int throttle_pin = 2;
int pitch_pin = 3;
int roll_pin = 4;
int yaw_pin = 5;

// Servo objects
Servo throttle_servo;
Servo pitch_servo;
Servo roll_servo;
Servo yaw_servo;

// Safety variables
bool emergency_shutdown = false;
unsigned long last_maneuver_time = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);

  throttle_servo.attach(throttle_pin);
  pitch_servo.attach(pitch_pin);
  roll_servo.attach(roll_pin);
  yaw_servo.attach(yaw_pin);
}

void loop() {
  // Check for emergency shutdown
  if (emergency_shutdown) {
    analogWrite(motor1, 0);
    analogWrite(motor2, 0);
    analogWrite(motor3, 0);
    analogWrite(motor4, 0);
    return;
  }

  // Read MPU6050 sensor values
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Compute pitch, roll, and yaw angles
  float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  float roll = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / PI;
  float yaw = gy * 250.0 / 32768.0;

  // Read receiver inputs
  int throttle_input = throttle_servo.read();
  int pitch_input = pitch_servo.read();
  int roll_input = roll_servo.read();
  int yaw_input = yaw_servo.read();

  // Map receiver inputs to PID setpoints
  float pitch_setpoint = map(pitch_input, 0, 180, -30, 30);
  float altitude_setpoint = map(throttle_input, 1000, 2000, 0, 1023);
  float yaw_setpoint = map(yaw_input, 0, 180, -180, 180);

// Compute PID errors
  pitch_error = pitch_setpoint - pitch;
  altitude_error = altitude_setpoint - analogRead(altitude_pin);
  yaw_error = yaw_setpoint - yaw;

// Compute PID outputs
  pitch_integral += Ki_pitch * pitch_error;
  altitude_integral += Ki_altitude * altitude_error;
  yaw_integral += Ki_yaw * yaw_error;

  pitch_derivative = Kd_pitch * (pitch_error - last_pitch_error);
  altitude_derivative = Kd_altitude * (altitude_error - last_altitude_error);
  yaw_derivative = Kd_yaw * (yaw_error - last_yaw_error);

  pitch_pid_output = Kp_pitch * pitch_error + pitch_integral + pitch_derivative;
  altitude_pid_output = Kp_altitude * altitude_error + altitude_integral + altitude_derivative;
  yaw_pid_output = Kp_yaw * yaw_error + yaw_integral + yaw_derivative;

// Update last error for next iteration
  last_pitch_error = pitch_error;
  last_altitude_error = altitude_error;
  last_yaw_error = yaw_error;

// Compute motor outputs
  int motor1_output = throttle_input + pitch_pid_output + altitude_pid_output - yaw_pid_output;
  int motor2_output = throttle_input - pitch_pid_output + altitude_pid_output + yaw_pid_output;
  int motor3_output = throttle_input - pitch_pid_output - altitude_pid_output - yaw_pid_output;
  int motor4_output = throttle_input + pitch_pid_output - altitude_pid_output + yaw_pid_output;

// Limit motor outputs to valid range
  motor1_output = constrain(motor1_output, 1000, 2000);
  motor2_output = constrain(motor2_output, 1000, 2000);
  motor3_output = constrain(motor3_output, 1000, 2000);
  motor4_output = constrain(motor4_output, 1000, 2000);

// Write motor outputs
  analogWrite(motor1, motor1_output);
  analogWrite(motor2, motor2_output);
  analogWrite(motor3, motor3_output);
  analogWrite(motor4, motor4_output);
}

// Function to shutdown motors in case of emergency
  void emergencyShutdown() {
  emergency_shutdown = true;
}






  