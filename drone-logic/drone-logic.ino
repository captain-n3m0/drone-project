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
  float yaw
