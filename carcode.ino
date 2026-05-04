#include <ECE3.h>


const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

uint16_t sensorValues[8] = {};
uint16_t summed_values[8] = {};
int16_t averagedSensorValues[8] = {};
int unweightedSensorSum = 0;
int number_samples = 5;
int weightedError = 0;
int sensorSum = 0;

float baseSpeed = 100;
//PD controller
float kp = 0.01;
float kd = kp ;

float correction = 0;
float previousError = 0;

int leftWheelSpeed;
int rightWheelSpeed;

void setup() {
  ECE3_Init();
  Serial.begin(9600);  // set the data rate in bits per second for serial data transmission
  delay(1000);

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);

   analogWrite(left_pwm_pin,baseSpeed);  
  analogWrite(right_pwm_pin,baseSpeed);  
  resetEncoderCount_left();
  resetEncoderCount_right();
}

void loop() {
  digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin,LOW);


  for (int j = 0; j < number_samples; j++) {
    // Read raw sensor values
    ECE3_read_IR(sensorValues);

    // Add the current sensor values using a for loop
    for (unsigned char i = 0; i < 8; i++) {
      summed_values[i] += sensorValues[i];
    }
  }

  // Print average values (average value = summed_values / number_samples
  // Serial.println("Average sensor values: ");
  for (unsigned char i = 0; i < 8; i++) {
    averagedSensorValues[i] = summed_values[i] / number_samples;
    // Serial.print(averagedSensorValues[i]);
    // Serial.print('\t');  // tab to format the raw data into columns in the Serial monitor
    unweightedSensorSum += averagedSensorValues[i];
  }
  // Serial.println();



  averagedSensorValues[0] = averagedSensorValues[0] * -15;
  averagedSensorValues[1] = averagedSensorValues[1] * -14;
  averagedSensorValues[2] = averagedSensorValues[2] * -12;
  averagedSensorValues[3] = averagedSensorValues[3] * -8;
  averagedSensorValues[4] = averagedSensorValues[4] * 8;
  averagedSensorValues[5] = averagedSensorValues[5] * 12;
  averagedSensorValues[6] = averagedSensorValues[6] * 14;
  averagedSensorValues[7] = averagedSensorValues[7] * 15;


  for (int i = 0; i < 8; i++) {
    sensorSum += averagedSensorValues[i];
  }
  weightedError = (sensorSum * 1000L) / (2 * unweightedSensorSum);
  Serial.println(weightedError);

  correction = kp * weightedError + kd * (weightedError - previousError);

  
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

  analogWrite(left_pwm_pin,leftWheelSpeed);  
  analogWrite(right_pwm_pin,rightWheelSpeed);  

  for (unsigned char i = 0; i < 8; i++) {
    summed_values[i] = 0;
    sensorValues[i] = 0;
    averagedSensorValues[i] = 0;
  }

  previousError = weightedError;
  unweightedSensorSum = 0;
  sensorSum = 0;
  weightedError = 0;
  
}


