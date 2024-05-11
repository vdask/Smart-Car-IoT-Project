// server libraries
#include <WiFi.h>
#include <PubSubClient.h>

// imu libraries
#include <Wire.h>
#include <MPU6050.h>

// Wi-Fi credentials
const char* ssid = "WIND_26BC11_EXT";
const char* password = "JRNRHRJJ";

// MQTT broker details
const char* mqtt_server = "192.168.1.144";
int mqtt_port = 1883;
const char* vel_topic = "linear velocity";
const char* angular_vel_topic = "angular velocity";

WiFiClient espClient;
PubSubClient client(espClient);

MPU6050 mpu;

const int MPU_ADDR = 0x68;  // MPU6050 I2C address

// speed calculation variables
struct VelocityData {
  float velocity;
  float angularVelocity;
};
float prevAccelY = 0.0;

void setup() {
  // Initialize imu and start I2C communication
  Wire.begin();
  mpu.initialize();

  // Initialize Serial monitor
  Serial.begin(57600);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  
  // Connect to MQTT broker
  client.setServer(mqtt_server, mqtt_port);
  //client.setCallback(callback);

  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void loop() {
   // Call the speedometer function to get the velocity and angular velocity
  VelocityData speedData = speedometer();

  // Access the velocity and angular velocity
  float velCmPerS = speedData.velocity;
  float angularVelocityZ = speedData.angularVelocity;

  // Convert velocity and angular velocity to character arrays
  char velMessage[10];
  char angularVelMessage[10];

  snprintf(velMessage, sizeof(velMessage), "%.2f", velCmPerS); // Adjust the format as needed
  snprintf(angularVelMessage, sizeof(angularVelMessage), "%.2f", angularVelocityZ); // Adjust the format as needed
  
  // Publish the values to MQTT topics
  if (client.publish(vel_topic, velMessage)) {
    Serial.print("Velocity: ");
    Serial.print(velCmPerS);
    Serial.print(" cm/s\t");
  } else {
    Serial.println("Failed to publish velocity value");
  }

  if (client.publish(angular_vel_topic, angularVelMessage)) {
    Serial.print("Angular Velocity: ");
    Serial.print(angularVelocityZ);
    Serial.println(" deg/s");
  } else {
    Serial.println("Failed to publish angular velocity value");
  }
  
  // Wait for 1 second before the next reading
  delay(10);
}

VelocityData speedometer() {
  VelocityData data;

  // Read accelerometer data
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert raw values to m/s²
  float accelY = ay / 16384.0; // Assuming accelerometer range +/- 2g

  static float velocityY = 0.0; // Initial velocity
  float dt = 0.01; // Time step in seconds

  // Calculate velocity using integration (trapezoidal rule numerical approximation)
  float newVelocityY = velocityY + 0.5 * (accelY + prevAccelY) * dt;

  // Only update velocity if there's motion
  if (abs(accelY) > 0.07) {  // You can adjust the threshold as needed
    velocityY = newVelocityY;
  } else {
    velocityY = 0.0;  // No motion, so velocity is reset
  }

  // Store the current acceleration for the next iteration
  prevAccelY = accelY;

  // Convert speed from m/s to cm/s
  data.velocity = newVelocityY * 100;

  // Get angular velocity around Z-axis directly from gyroscope
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Convert raw gyroscope value to degrees per second
  float gzFloat = gz / 131.0;
  
  // Apply offset to the angular velocity
  data.angularVelocity = gzFloat + 1.5;

  return data;
}
