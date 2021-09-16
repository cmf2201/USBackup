/*
  WriteSingleField
  
  Description: Writes a value to a channel on ThingSpeak every 20 seconds.
  
  Hardware: ESP32 based boards
  
  !!! IMPORTANT - Modify the secrets.h file for this project with your network connection and ThingSpeak channel details. !!!
  
  Note:
  - Requires installation of EPS32 core. See https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md for details. 
  - Select the target hardware from the Tools->Board menu
  - This example is written for a network using WPA encryption. For WEP or WPA, change the WiFi.begin() call accordingly.
  
  ThingSpeak ( https://www.thingspeak.com ) is an analytic IoT platform service that allows you to aggregate, visualize, and 
  analyze live data streams in the cloud. Visit https://www.thingspeak.com to sign up for a free account and create a channel.  
  
  Documentation for the ThingSpeak Communication Library for Arduino is in the README.md folder where the library was installed.
  See https://www.mathworks.com/help/thingspeak/index.html for the full ThingSpeak documentation.
  
  For licensing information, see the accompanying license file.
  
  Copyright 2020, The MathWorks, Inc.
*/

#include <WiFi.h>
#include <Wire.h>                     // enable I2C.

//------Thingspeak communication------
#include "secrets.h"
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros

char ssid[] = SECRET_SSID;   // your network SSID (name) 
char pass[] = SECRET_PASS;   // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

int number = 0;

//------Sensor Communication-----

char sensordata[30];                  // A 30 byte character array to hold incoming data from the sensors
byte sensor_bytes_received = 0;       // We need to know how many characters bytes have been received

byte code = 0;                        // used to hold the I2C response code.
byte in_char = 0;                     // used as a 1 byte buffer to store in bound bytes from the I2C Circuit.

#define TOTAL_CIRCUITS 1              // <-- CHANGE THIS | set how many I2C circuits are attached to the Tentacle shield(s): 1-8

int channel_ids[] = {102};// <-- CHANGE THIS.
// A list of I2C ids that you set your circuits to.
// This array should have 1-8 elements (1-8 circuits connected)

char *channel_names[] = {"TP"}; // <-- CHANGE THIS.
// A list of channel names (must be the same order as in channel_ids[]) 
// it's used to give a name to each sensor ID. This array should have 1-8 elements (1-8 circuits connected).
// {"PH Tank 1", "PH Tank 2", "EC Tank 1", "EC Tank2"}, or {"PH"}


void setup() {
  //----------thingspeak-----------
  Serial.begin(115200);  //Initialize serial
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo native USB port only
  }
  
  WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client);  // Initialize ThingSpeak
  
  //---------Sensor i2c communciation begin-------
  Wire.begin();                // enable I2C port.
}

void loop() {

  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }

  // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
  // pieces of information in a channel.  Here, we write to field 1.
  int x = ThingSpeak.writeField(myChannelNumber, 1, sensordata, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
  
  for (int channel = 0; channel < TOTAL_CIRCUITS; channel++) {       // loop through all the sensors
  
    Wire.beginTransmission(channel_ids[channel]);     // call the circuit by its ID number.
    Wire.write('r');                              // request a reading by sending 'r'
    Wire.endTransmission();                            // end the I2C data transmission.
    
    delay(1000);  // AS circuits need a 1 second before the reading is ready

    sensor_bytes_received = 0;                        // reset data counter
    memset(sensordata, 0, sizeof(sensordata));        // clear sensordata array;

    Wire.requestFrom(channel_ids[channel], 48, 1);    // call the circuit and request 48 bytes (this is more then we need).
    code = Wire.read();

    while (Wire.available()) {          // are there bytes to receive?
      in_char = Wire.read();            // receive a byte.

      if (in_char == 0) {               // null character indicates end of command
        Wire.endTransmission();         // end the I2C data transmission.
        break;                          // exit the while loop, we're done here
      }
      else {
        sensordata[sensor_bytes_received] = in_char;      // append this byte to the sensor data array.
        sensor_bytes_received++;
      }
    }
    
    Serial.print(channel_names[channel]);   // print channel name
    Serial.print(':');

    switch (code) {                          // switch case based on what the response code is.
      case 1:                               // decimal 1  means the command was successful.
        Serial.println(sensordata);       // print the actual reading
        break;                                // exits the switch case.

      case 2:                                // decimal 2 means the command has failed.
        Serial.println("command failed");   // print the error
        break;                                 // exits the switch case.

      case 254:                              // decimal 254  means the command has not yet been finished calculating.
        Serial.println("circuit not ready"); // print the error
        break;                                 // exits the switch case.

      case 255:                              // decimal 255 means there is no further data to send.
        Serial.println("no data");          // print the error
        break;                                 // exits the switch case.
    }
  }
  
  delay(20000); // Wait 20 seconds to update the channel again
}
