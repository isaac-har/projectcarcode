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


//PD controller
float baseSpeed = 50;
int maxError = 2100;
float errorMargin = 0.5;

float kp = (baseSpeed * errorMargin) / maxError;
float kd = kp * 5;

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



  averagedSensorValues[0] = averagedSensorValues[0] * -8;
  averagedSensorValues[1] = averagedSensorValues[1] * -4;
  averagedSensorValues[2] = averagedSensorValues[2] * -2;
  averagedSensorValues[3] = averagedSensorValues[3] * -1;
  averagedSensorValues[4] = averagedSensorValues[4] * 1;
  averagedSensorValues[5] = averagedSensorValues[5] * 2;
  averagedSensorValues[6] = averagedSensorValues[6] * 4;
  averagedSensorValues[7] = averagedSensorValues[7] * 8;


  for (int i = 0; i < 8; i++) {
    sensorSum += averagedSensorValues[i];
  }
  weightedError = (sensorSum * 1000L) / unweightedSensorSum;
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

  ChangeWheelSpeeds(left_pwm_pin, leftWheelSpeed, right_pwm_pin, rightWheelSpeed);

  for (unsigned char i = 0; i < 8; i++) {
    summed_values[i] = 0;
    sensorValues[i] = 0;
    averagedSensorValues[i] = 0;
  }

  previousError = weightedError;
  unweightedSensorSum = 0;
  sensorSum = 0;
  weightedError = 0;

 
  delay(1000);
  
}

void  ChangeWheelSpeeds(int initialLeftSpd, int finalLeftSpd, int initialRightSpd, int finalRightSpd) {
/*  
 *   This function changes the car speed gradually (in about 30 ms) from initial
 *   speed to final speed. This non-instantaneous speed change reduces the load 
 *   on the plastic geartrain, and reduces the failure rate of the motors. 
 */
  int diffLeft  = finalLeftSpd-initialLeftSpd;
  int diffRight = finalRightSpd-initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft  = abs(diffLeft)/stepIncrement;
  int numStepsRight = abs(diffRight)/stepIncrement;
  int numSteps = max(numStepsLeft,numStepsRight);
  
  int pwmLeftVal = initialLeftSpd;        // initialize left wheel speed 
  int pwmRightVal = initialRightSpd;      // initialize right wheel speed 
  int deltaLeft = (diffLeft)/numSteps;    // left in(de)crement
  int deltaRight = (diffRight)/numSteps;  // right in(de)crement

  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);    
    analogWrite(right_pwm_pin,pwmRightVal); 
    delay(30);   
  } // end for int k
//  if(finalLeftSpd  == 0) analogWrite(left_pwm_pin,0); ;
//  if(finalRightSpd == 0) analogWrite(right_pwm_pin,0);
  analogWrite(left_pwm_pin,finalLeftSpd);  
  analogWrite(right_pwm_pin,finalRightSpd);  
} // end void  ChangeWheelSpeeds

//void ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd) {
//  /*  
// *   This function changes the car base speed gradually (in about 300 ms) from
// *   initialBaseSpeed to finalBaseSpeed. This non-instantaneous speed change
// *   reduces the load on the plastic geartrain, and reduces the failure rate of 
// *   the motors. 
// */
//  int speedChangeTime = 300; // milliseconds
//  int numSteps = 5;
//  int pwmVal = initialBaseSpd; // initialize left wheel speed 
//  int deltaSpeed = (finalBaseSpd-initialBaseSpd)/numSteps; // in(de)crement
//  for(int k=0;k<numSteps;k++) {
////    pwmVal = pwmLeftVal + deltaSpeed;
////    analogWrite(left_pwm_pin,pwmVal);    
////    analogWrite(right_pwm_pin,pwmVal); 
////    delay(60);   
//  } // end for int k
//} // end void ChangeBaseSpeed





int average()  //average pulse count
{
  int getL=getEncoderCount_left();
  int getR=getEncoderCount_right();
//  Serial.print(getL);Serial.print("\t");Serial.println(getR);
  return ((getEncoderCount_left() + getEncoderCount_right())/2);
}
