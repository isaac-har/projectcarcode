#include <ECE3.h>

uint16_t sensorValues[8] = {};
uint16_t summed_values[8] = {};
int16_t averagedSensorValues[8] = {};
int unweightedSensorSum = 0;
int number_samples = 5;
int weightedError = 0;
int sensorSum = 0;


void setup() {
  ECE3_Init();
  Serial.begin(9600);  // set the data rate in bits per second for serial data transmission
  delay(1000);
}

void loop() {



  for (int j = 0; j < number_samples; j++) {
    // Read raw sensor values
    ECE3_read_IR(sensorValues);

    // Add the current sensor values using a for loop
    for (unsigned char i = 0; i < 8; i++) {
      summed_values[i] += sensorValues[i];
    }
  }

  // Print average values (average value = summed_values / number_samples
  Serial.println("Average sensor values: ");
  for (unsigned char i = 0; i < 8; i++) {
    averagedSensorValues[i] = summed_values[i] / number_samples;
    Serial.print(averagedSensorValues[i]);
    Serial.print('\t');  // tab to format the raw data into columns in the Serial monitor
    unweightedSensorSum += averagedSensorValues[i];
  }
  Serial.println();



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

  for (unsigned char i = 0; i < 8; i++) {
    summed_values[i] = 0;
    sensorValues[i] = 0;
    averagedSensorValues[i] = 0;
  }
  unweightedSensorSum = 0;
  sensorSum = 0;
  weightedError = 0;
  delay(1000);
  
}
