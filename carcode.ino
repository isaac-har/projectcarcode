#include <ECE3.h>

const int left_nslp_pin = 31;   // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin = 11;  // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin = 29;
const int right_dir_pin = 30;
const int left_pwm_pin = 40;
const int right_pwm_pin = 39;

uint16_t sensorValues[8];
float calibratedSensorValues[8];
float summed_values[8];

float weightedError = 0;
float sensorSum = 0;

float baseSpeed;
float kp;
float kd;

float correction = 0;
float previousError = 0;

float leftWheelSpeed;
float rightWheelSpeed;

const float MAX_SENSOR_VALUE = 49000.0;
const int weights[8] = { -15, -14, -12, -8, 8, 12, 14, 15 };

int crossPieceCount = 0;

void turnAround() {

  //Stop briefly
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
  delay(15);

  //Reverse left motor spin direction, turn for 400 ms
  digitalWrite(left_dir_pin, HIGH);
  digitalWrite(right_dir_pin, LOW);

  analogWrite(left_pwm_pin, 240);
  analogWrite(right_pwm_pin, 240);

  delay(246);

  //Stop briefly again
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
  //Make left motor forward again
  digitalWrite(left_dir_pin, LOW);

  delay(15);

  //Drive forward again off the cross piece
  analogWrite(left_pwm_pin, 240);
  analogWrite(right_pwm_pin, 240);

  crossPieceCount++;
  delay(243);
  
}

void setup() {
  ECE3_Init();
  Serial.begin(9600);  // set the data rate in bits per second for serial data transmission


  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);

  digitalWrite(left_dir_pin, LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin, LOW);

  delay(2000);
}

void loop() {

  ECE3_read_IR(sensorValues);

  //Calibrating each sensor reading
  calibratedSensorValues[0] = ((sensorValues[0] - 668.0) * 1000L) / 926.0;
  calibratedSensorValues[1] = ((sensorValues[1] - 574.0) * 1000L) / 1091.0;
  calibratedSensorValues[2] = ((sensorValues[2] - 551.0) * 1000L) / 972.0;
  calibratedSensorValues[3] = ((sensorValues[3] - 551.0) * 1000L) / 948.0;
  calibratedSensorValues[4] = ((sensorValues[4] - 574.0) * 1000L) / 925.0;
  calibratedSensorValues[5] = ((sensorValues[5] - 551.0) * 1000L) / 938.0;
  calibratedSensorValues[6] = ((sensorValues[6] - 505.0) * 1000L) / 971.0;
  calibratedSensorValues[7] = ((sensorValues[7] - 611.0) * 1000L) / 1055.0;

  //Weighting the sensor values
  for (int i = 0; i < 8; i++) {
    calibratedSensorValues[i] *= weights[i];
    sensorSum += calibratedSensorValues[i];
  }

  //Normalizing based on the highest possible sensor reading
  weightedError = sensorSum / MAX_SENSOR_VALUE;

  //Slow down if error is large i.e. on a turn
  if (abs(weightedError) < 0.45) {
    baseSpeed = 230;
    kp = 134;
    kd = 1135;
  } else {
    baseSpeed = 145;
    kp = 76;
    kd = 622;
  }

  //Use PD values to determine how much to change wheel speed
  correction = (kp * weightedError) + (kd * (weightedError - previousError));

  leftWheelSpeed = baseSpeed - correction;
  if (leftWheelSpeed > 255) {
    leftWheelSpeed = 255;
  }
  if (leftWheelSpeed < 0) {
    leftWheelSpeed = 0;
  }

  rightWheelSpeed = baseSpeed + correction;
  if (rightWheelSpeed > 255) {
    rightWheelSpeed = 255;
  }
  if (rightWheelSpeed < 0) {
    rightWheelSpeed = 0;
  }

  analogWrite(left_pwm_pin, leftWheelSpeed);
  analogWrite(right_pwm_pin, rightWheelSpeed);

  if (abs(weightedError - 0.0472) < 0.001 && abs(previousError - 0.0472) < 0.001) {
    crossPieceCount++;
  }

  if (crossPieceCount == 1) {
    turnAround();
  }
  if (crossPieceCount >= 3) {
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);

    delay(999999);
  }


  // Serial.print("WE: ");
  // Serial.print(weightedError, 6);
  // Serial.print("\tPE: ");
  // Serial.print(previousError, 6);
  // Serial.print("\tCorr: ");
  // Serial.print(correction);
  // Serial.print("\tL: ");
  // Serial.print(leftWheelSpeed);
  // Serial.print("\tR: ");
  // Serial.println(rightWheelSpeed);
  // delay(100);



  previousError = weightedError;
  sensorSum = 0;
  weightedError = 0;
}
