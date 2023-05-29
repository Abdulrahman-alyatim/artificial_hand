#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pin connected to voltage divider output for flex sensor
const int flexPin1 = A1;
const int flexPin2 = A2;
const int flexPin3 = A4;
const int flexPin4 = A3;
const int flexPin5 = A5;

// Change these constants according to your project's design
const float VCC = 5.0;                  // voltage at Ardunio 5V line
const float R_DIV = 10000.0;            // resistor used to create a voltage divider
const float flatResistance = 25000.0;   // resistance when flat
const float bendResistance = 100000.0;  // resistance at 90 deg

// Change these constants according to your calibration
const int NUM_CALIBRATION_READINGS = 10;  // number of readings to take during calibration
float calibrationSum1 = 0.0;              // sum of readings during calibration for flex sensor 1
float calibrationValue1 = 0.0;

float calibrationSum2 = 0.0;
float calibrationValue2 = 0.0;

float calibrationSum3 = 0.0;
float calibrationValue3 = 0.0;

float calibrationSum4 = 0.0;
float calibrationValue4 = 0.0;

float calibrationSum5 = 0.0;
float calibrationValue5 = 0.0;


// Change this constant according to your averaging preference
const int NUM_AVERAGING_READINGS = 5;  // number of readings to average

// Change these constants according to your servo's specs
const int SERVO_MIN = 150;  // pulse length for minimum servo position
const int SERVO_MAX = 600;  // pulse length for maximum servo position
const int SERVO_FREQ = 50;  // frequency of the PWM signal in Hz

void setup() {
  Serial.begin(9600);
  pinMode(flexPin1, INPUT);
  pinMode(flexPin2, INPUT);
  pinMode(flexPin3, INPUT);
  pinMode(flexPin4, INPUT);
  pinMode(flexPin5, INPUT);

  // Initialize the PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // Calibrate the sensors
  Serial.println("Calibrating sensors. Please do not touch.");

  // Calibrate flex sensor 1
  for (int i = 0; i < NUM_CALIBRATION_READINGS; i++) {
    int ADCflex = analogRead(flexPin1);
    float Vflex = ADCflex * VCC / 1023.0;
    float Rflex = R_DIV * (VCC / Vflex - 1.0);
    calibrationSum1 += Rflex;
    delay(10);
  }
  calibrationValue1 = calibrationSum1 / NUM_CALIBRATION_READINGS;
  Serial.println("Calibration complete for flex sensor 1. Calibration value: " + String(calibrationValue1));

  // Calibrate flex sensor 2
  for (int i = 0; i < NUM_CALIBRATION_READINGS; i++) {
    int ADCflex = analogRead(flexPin2);
    float Vflex = ADCflex * VCC / 1023.0;
    float Rflex = R_DIV * (VCC / Vflex - 1.0);
    calibrationSum2 += Rflex;
    delay(10);
  }
  calibrationValue2 = calibrationSum2 / NUM_CALIBRATION_READINGS;
  Serial.println("Calibration complete for flex sensor 2. Calibration value: " + String(calibrationValue2));
  Serial.println();

  // Calibrate flex sensor 3
  for (int i = 0; i < NUM_CALIBRATION_READINGS; i++) {
    int ADCflex = analogRead(flexPin3);
    float Vflex = ADCflex * VCC / 1023.0;
    float Rflex = R_DIV * (VCC / Vflex - 1.0);
    calibrationSum3 += Rflex;
    delay(10);
  }
  calibrationValue3 = calibrationSum3 / NUM_CALIBRATION_READINGS;
  Serial.println("Calibration complete for flex sensor 3. Calibration value: " + String(calibrationValue3));
  Serial.println();

  // Calibrate flex sensor 4
  for (int i = 0; i < NUM_CALIBRATION_READINGS; i++) {
    int ADCflex = analogRead(flexPin4);
    float Vflex = ADCflex * VCC / 1023.0;
    float Rflex = R_DIV * (VCC / Vflex - 1.0);
    calibrationSum4 += Rflex;
    delay(10);
  }
  calibrationValue4 = calibrationSum4 / NUM_CALIBRATION_READINGS;
  Serial.println("Calibration complete for flex sensor 4. Calibration value: " + String(calibrationValue4));
  Serial.println();

  // Calibrate flex sensor 5
  for (int i = 0; i < NUM_CALIBRATION_READINGS; i++) {
    int ADCflex = analogRead(flexPin5);
    float Vflex = ADCflex * VCC / 1023.0;
    float Rflex = R_DIV * (VCC / Vflex - 1.0);
    calibrationSum5 += Rflex;
    delay(10);
  }
  calibrationValue5 = calibrationSum5 / NUM_CALIBRATION_READINGS;
  Serial.println("Calibration complete for flex sensor 5. Calibration value: " + String(calibrationValue5));
  Serial.println();
}

void loop() {
  // Take a number of readings and average them for each flex sensor
  float readingSum1 = 0.0;

  for (int i = 0; i < NUM_AVERAGING_READINGS; i++) {
    int ADCflex1 = analogRead(flexPin1);
    float Vflex1 = ADCflex1 * VCC / 1023.0;
    float Rflex1 = R_DIV * (VCC / Vflex1 - 1.0);
    readingSum1 += Rflex1;
    delay(10);
  }
  float averageReading1 = readingSum1 / NUM_AVERAGING_READINGS;

  float readingSum2 = 0.0;
  for (int i = 0; i < NUM_AVERAGING_READINGS; i++) {
    int ADCflex2 = analogRead(flexPin2);
    float Vflex2 = ADCflex2 * VCC / 1023.0;
    float Rflex2 = R_DIV * (VCC / Vflex2 - 1.0);
    readingSum2 += Rflex2;
    delay(10);
  }
  float averageReading2 = readingSum2 / NUM_AVERAGING_READINGS;

  float readingSum3 = 0.0;
  for (int i = 0; i < NUM_AVERAGING_READINGS; i++) {
    int ADCflex3 = analogRead(flexPin3);
    float Vflex3 = ADCflex3 * VCC / 1023.0;
    float Rflex3 = R_DIV * (VCC / Vflex3 - 1.0);
    readingSum3 += Rflex3;
    delay(10);
  }
  float averageReading3 = readingSum3 / NUM_AVERAGING_READINGS;


  float readingSum4 = 0.0;
  for (int i = 0; i < NUM_AVERAGING_READINGS; i++) {
    int ADCflex4 = analogRead(flexPin4);
    float Vflex4 = ADCflex4 * VCC / 1023.0;
    float Rflex4 = R_DIV * (VCC / Vflex4 - 1.0);
    readingSum4 += Rflex4;
    delay(10);
  }
  float averageReading4 = readingSum4 / NUM_AVERAGING_READINGS;

  float readingSum5 = 0.0;
  for (int i = 0; i < NUM_AVERAGING_READINGS; i++) {
    int ADCflex5 = analogRead(flexPin5);
    float Vflex5 = ADCflex5 * VCC / 1023.0;
    float Rflex5 = R_DIV * (VCC / Vflex5 - 1.0);
    readingSum5 += Rflex5;
    delay(10);
  }
  float averageReading5 = readingSum5 / NUM_AVERAGING_READINGS;


  float angle1 = map(averageReading1, calibrationValue1, bendResistance, 0, 90.0);
  float angle2 = map(averageReading2, calibrationValue2, bendResistance, 0, 90.0);
  float angle3 = map(averageReading3, calibrationValue3, bendResistance, 0, 90.0);
  float angle4 = map(averageReading4, calibrationValue4, bendResistance, 0, 90.0);
  float angle5 = map(averageReading5, calibrationValue5, bendResistance, 0, 90.0);
  if (angle1 < 0 )
    angle1 = 0;
  if (angle1 > 5 )
    angle1 = 5;

  if (angle2 < 0)
    angle2 = 0;
  if (angle2 > 5 )
    angle2 = 5;

  if (angle3 < 0)
    angle3 = 0;
  if (angle3 > 5 )
    angle3 = 5;

  if (angle4 < 0)
    angle4 = 0;
  if (angle4 > 5 )
    angle4 = 5;

  if (angle5 < 0)
    angle5 = 0;
  if (angle5 > 5 )
    angle5 = 5;

  Serial.print("Bend 1: ");
  Serial.print(angle1);
  Serial.print(" degrees, ");

  Serial.print(" Bend 2: ");
  Serial.print(angle2);
  Serial.print(" degrees, ");

  Serial.print(" Bend 3: ");
  Serial.print(angle3);
  Serial.print(" degrees");

  Serial.print(" Bend 4: ");
  Serial.print(angle4);
  Serial.print(" degrees");

  Serial.print(" Bend 5: ");
  Serial.print(angle5);
  Serial.print(" degrees\n");

  // Map the bend angle to a servo position and send the signal to the PCA9685
  int servoPos1 = map(angle1, 0, 5, SERVO_MIN, SERVO_MAX);
  int servoPos2 = map(angle2, 5, 0, SERVO_MIN, SERVO_MAX);
  int servoPos3 = map(angle3, 0, 5, SERVO_MAX, SERVO_MIN);
  int servoPos4 = map(angle4, 0, 5, SERVO_MAX, SERVO_MIN);
  int servoPos5 = map(angle5, 0, 5, SERVO_MAX, SERVO_MIN);
  if (servoPos1 < 0)
    servoPos1 = 0;
  if (servoPos2 < 0)
    servoPos2 = 0;
  if (servoPos3 < 0)
    servoPos3 = 0;
  if (servoPos4 < 0)
    servoPos4 = 0;
  if (angle5 < 0)
    servoPos5 = 0;
  pwm.setPWM(0, 0, servoPos1);
  pwm.setPWM(1, 0, servoPos2);
  pwm.setPWM(2, 0, servoPos3);
  pwm.setPWM(3, 0, servoPos4);
  pwm.setPWM(4, 0, servoPos5);
  delay(500);
}
