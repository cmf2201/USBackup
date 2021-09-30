//include nessacary libraries
#include "ping1d.h"
#include "HardwareSerial.h"
static void smartdelay(unsigned long ms);

HardwareSerial SonarSerial(1);
// This serial port is used to communicate with the Ping device
// Here, we use pin 27 as esp32 rx (Ping tx, white), 28 as esp32 tx (Ping rx, green)
static const uint8_t espRxPin = 27;
static const uint8_t espTxPin = 26;
static Ping1D ping { SonarSerial };

void setup()
{
  //start serial communication
  SonarSerial.begin(115200, SERIAL_8N1, 27, 26);
  Serial.begin(115200);
  Serial.println("Blue Robotics ping1d-simple.ino");
  while (!ping.initialize()) {
    Serial.println("\nPing device failed to initialize!");
    Serial.println("Are the Ping rx/tx wired correctly?");
    Serial.print("Ping rx is the green wire, and should be connected to esp32 pin ");
    Serial.print(espTxPin);
    Serial.println(" (esp32 tx)");
    Serial.print("Ping tx is the white wire, and should be connected to esp32 pin ");
    Serial.print(espRxPin);
    Serial.println(" (esp32 rx)");
    delay(2000);
  }
}

void loop()
{
  //if receiving data from ping sonar, update to serial monitor
    if (ping.update()) {
      Serial.print("Distance: ");
      Serial.print(ping.distance());
      Serial.print("\tConfidence: ");
      Serial.println(ping.confidence());
    } else {
      Serial.println("No update received!");
    }
}
