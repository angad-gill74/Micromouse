#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050_light.h>  // MPU6050_light library

#define XSHUT_FRONT 5
#define XSHUT_LEFT 19
#define XSHUT_RIGHT 18

// Define motor driver pins (DRV8833)
#define MOTOR_A_IN1 25  // Motor A Input 1
#define MOTOR_A_IN2 33  // Motor A Input 2
#define MOTOR_B_IN1 32  // Motor B Input 1
#define MOTOR_B_IN2 35  // Motor B Input 2

// Define encoder pins
#define ENCODER_A1 13  // Motor A Encoder A
#define ENCODER_A2 12  // Motor A Encoder B
#define ENCODER_B1 14  // Motor B Encoder A
#define ENCODER_B2 27  // Motor B Encoder B

// LiDAR sensors
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();  // Front sensor
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();   // Left sensor
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();  // Right sensor

const int threshold = 150;  // Threshold distance in millimeters

// Separate measurement variables for each sensor
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

// Variables for encoder counting
volatile int encoderACount = 0;
volatile int encoderBCount = 0;
unsigned long previousMillis = 0;
const long interval = 1000; // Interval to calculate speed (1 second)

// MPU6050 for gyroscope
MPU6050 mpu(Wire);
float gyroZ = 0.0;
float targetAngle = 0.0;  // Target angle for turning

// Encoder ISR
void IRAM_ATTR encoderA_ISR() {
  encoderACount++;
}

void IRAM_ATTR encoderB_ISR() {
  encoderBCount++;
}

#define LOX1_ADDRESS 0x30  // I2C address for Front sensor
#define LOX2_ADDRESS 0x31  // I2C address for Left sensor
#define LOX3_ADDRESS 0x32  // I2C address for Right sensor

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  Wire.begin();

  // Set motor driver pins as output
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Set encoder pins as input with pull-up resistors
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);
  
  // Attach interrupt service routines for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoderB_ISR, RISING);

  // Initialize XSHUT pins
  pinMode(XSHUT_FRONT, OUTPUT);
  digitalWrite(XSHUT_FRONT, LOW);
  pinMode(XSHUT_LEFT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_RIGHT, LOW);

  // Initialize sensors one by one, setting their unique I2C addresses
  initSensor(lox1, XSHUT_FRONT, LOX1_ADDRESS);  // Initialize Front sensor
  initSensor(lox2, XSHUT_LEFT, LOX2_ADDRESS);  // Initialize Left sensor
  initSensor(lox3, XSHUT_RIGHT, LOX3_ADDRESS);  // Initialize Right sensor

  Serial.println(F("All sensors initialized"));

  // Initialize MPU6500 (using MPU6050_light library)
  if (mpu.begin() != 0) {
    Serial.println("Failed to initialize MPU6500");
    while (1);
  }
  Serial.println("MPU6500 initialized successfully");

  // Calibrate the MPU6500
  mpu.calcOffsets(); // This will calibrate the accelerometer and gyroscope, might take a few seconds
  Serial.println("MPU6500 calibration done");
}

void loop() {
  unsigned long currentMillis = millis();
  mpu.update();  // Update MPU6050 readings

  // Read distance from Front sensor
  lox1.rangingTest(&measure1, false);
  if (measure1.RangeStatus != 4) {
    Serial.print("Front distance: ");
    Serial.print(measure1.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Front sensor out of range");
  }

  // Read distance from Left sensor
  lox2.rangingTest(&measure2, false);
  if (measure2.RangeStatus != 4) {
    Serial.print("Left distance: ");
    Serial.print(measure2.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Left sensor out of range");
  }

  // Read distance from Right sensor
  lox3.rangingTest(&measure3, false);
  if (measure3.RangeStatus != 4) {
    Serial.print("Right distance: ");
    Serial.print(measure3.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Right sensor out of range");
  }

  delay(100);

  // Gyroscope readings (Z-axis for orientation)
  gyroZ = mpu.getAngleZ();  // Get Z-axis angle
  Serial.print("Gyro Z: ");
  Serial.println(gyroZ);

  // Decision-making based on LiDAR sensor readings
  if (measure2.RangeMilliMeter > threshold) {
    turnLeft();
  }
  else{
    if(measure1.RangeMilliMeter > threshold) {
      runMotors();
    }
    else{
      if(measure3.RangeMilliMeter > threshold){
        turnRight();
      }
      else{
        turnRight();
        turnRight();
      }
    }
  }

  delay(100);
}

int readLidarDistance(Adafruit_VL53L0X &lox) {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) {  // Ignore out of range readings
    return measure.RangeMilliMeter;
  } else {
    return -1;  // Invalid distance
  }
}

void runMotors() {
  Serial.println("_______ Running Motors ______");
  // Run Motor A forward using DRV8833
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);

  // Run Motor B forward using DRV8833
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
}

void stopMotors() {
  Serial.println("XXXXXXXX Stopping Motors XXXXXXXXXX");
  // Stop Motor A using DRV8833
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);

  // Stop Motor B using DRV8833
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}

void turnLeft() {
  // Turn left using the gyroscope for precise orientation control
  Serial.println("<---- Turning Left");
  stopMotors();
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  targetAngle = gyroZ + 90;  // Set target to 90 degrees left turn

  // Rotate until the target angle is achieved
  while (mpu.getAngleZ() < targetAngle) {
    // Motor A reverse, Motor B forward
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);

    mpu.update();  // Update MPU6050 readings
  }

  stopMotors();
}

void turnRight() {
  // Turn right using the gyroscope for precise orientation control
  Serial.println("Turning Right  --->");
  stopMotors();
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  targetAngle = gyroZ - 45;  // Set target to 90 degrees right turn

  // Rotate until the target angle is achieved
  while (mpu.getAngleZ() > targetAngle) {
    // Motor A forward, Motor B reverse
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);

    mpu.update();  // Update MPU6050 readings
  }

  stopMotors();
}

// Function to initialize each VL53L0X sensor
void initSensor(Adafruit_VL53L0X &lox, int xshutPin, int newAddress) {
  // Turn on the sensor by setting its XSHUT pin HIGH
  digitalWrite(xshutPin, HIGH);
  delay(10); // Allow time for the sensor to boot up

  // Initialize the sensor
  if (!lox.begin()) {
    Serial.print(F("Failed to boot sensor at XSHUT pin: "));
    Serial.println(xshutPin);
    while (1);
  }

  // Set a unique I2C address for the sensor
  lox.setAddress(newAddress);
  Serial.print("Sensor initialized with new address: 0x");
  Serial.println(newAddress, HEX);
}
