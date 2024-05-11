#include <DabbleESP32.h>

// UART pins
#define RX0 3
#define TX0 1

void setup() {
  Serial.begin(57600, SERIAL_8N1, RX0, TX0);
  Dabble.begin("MyEsp32");   //set bluetooth name of your device
}

void loop() {
  Dabble.processInput();  //this function is used to refresh data obtained from smartphone.
  print_Accelerometer_data();
}

void print_Accelerometer_data()
{
  Serial.print(Sensor.getAccelerometerXaxis());
  Serial.print('\t');
  Serial.print(Sensor.getAccelerometerYaxis());
  Serial.print('\t');
  Serial.println(Sensor.getAccelerometerZaxis());
  delay(1050);
}
