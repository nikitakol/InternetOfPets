#include "thingProperties.h"

// LowPower GPS - Runs on battery and recovers from power loss
#include <ArduinoLowPower.h>
#include <TinyGPS.h>
#include <WiFiNINA.h>
#include <SPI.h>

#if defined(ESP32)
static int const LED_BUILTIN = 2;
#endif

#define WAITING_TIME 1 //waiting time between messages
#define GPS_INFO_BUFFER_SIZE 128

bool debug = true; //DEBUG switch/////''

const int backlights = 4;
const int pulsador = 5;
const int audio = 3;

TinyGPS gps;//GPS Object

//GPS data variables
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long chars;
unsigned short sentences, failed_checksum;
char GPS_info_char;
char GPS_info_buffer[GPS_INFO_BUFFER_SIZE];
unsigned int received_char;
bool message_started = false;
int i = 0;

// GPS coordinate structure, 12 bytes size on 32 bits platforms
struct gpscoord {
  float a_latitude; // 4 bytes
  float a_longitude; // 4 bytes
};


//////////////// Waiting function //////////////////
void Wait(int m, bool s) {
  //m minutes to wait
  //s slow led pulses
  if (debug)
    digitalWrite(LED_BUILTIN, LOW);
  if (s) {
    int seg = m * 100;
    for (int i = 0; i < seg; i++) {
      // nada
      delay(1000);
    }
  } else {
    int seg = m * 50;
    for (int i = 0; i < seg; i++) {
      delay(1000);
    }
  }
}

////////////////// Convert GPS function //////////////////
/* Converts GPS float data to Char data */
String ConvertGPSdata(const void* data, uint8_t len) {
  uint8_t* bytes = (uint8_t*)data;
  String chain ;
   if (debug) {
    Serial.print("Length: "); Serial.println(len);
  }
  for (uint8_t i = len - 1; i < len; --i) {
    if (bytes[i] < 12) {
      chain.concat(byte(0)); // Not tested
    }
    chain.concat(char(bytes[i]));
    if (debug) Serial.print(bytes[i], HEX);
  }
   if (debug) {
    Serial.println("");
 //   Serial.print("String to send: "); Serial.println(cadena);
  }
  return chain;
}

////////////////////////// Get GPS position function/////////////////////
String GetGPSposition() {
  
  int messages_count = 0;
  String pos;
  
  if (debug) 
    Wait(1, false);
    while (messages_count < 5000) {
    while (Serial1.available()) {
      
      int GPS_info_char = Serial1.read();
      
      if (GPS_info_char == '$') messages_count ++; // start of message. Counting messages.
      
      if (debug) {
        if (GPS_info_char == '$') { // start of message
          message_started = true;
          received_char = 0;
        } else if (GPS_info_char == '*') { //end of message
          message_started = false; // ready for the new message
        } else if (message_started == true) { // the message is already started and I got a new character
          if (received_char <= GPS_INFO_BUFFER_SIZE) { // to avoid buffer overflow
            GPS_info_buffer[received_char] = GPS_info_char;
            received_char++;
          } else { // resets everything (overflow happened)
            message_started = false;
            received_char = 0;
          }
        }
      }
      if (gps.encode(GPS_info_char)) {
        gps.f_get_position(&latitude, &longitude);
        
        // Store coordinates into dedicated structure
        gpscoord coords = {longitude, latitude};
        
        gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, 
         &hundredths);
        
        if (debug) {
          Serial.println();
          Serial.println();
          Serial.print("Latitude/Longitude: ");
          Serial.print(latitude, 5);
          Serial.print(", ");
          Serial.println(longitude, 5);
          Serial.println();
          Serial.print("Satelites: "); Serial.println(gps.satellites());
          Serial.println();
          digitalWrite(backlights, HIGH);
          delay(500);
          digitalWrite(backlights, LOW);
          delay(500);  
        }
        
        gps.stats(&chars, &sentences, &failed_checksum);
        if (debug)
          pos = ConvertGPSdata(&coords, sizeof(gpscoord)); //Send data
        return pos;
      }
    }
  }
  pos = "No Signal";
}


////////SETUP/////////
void setup() {
  /* Initialize serial and wait up to 5 seconds for port to open */
  Serial.begin(9600);

  /* This function takes care of connecting your sketch variables to the ArduinoIoTCloud object */
  initProperties();

  /* Initialize Arduino IoT Cloud library */
  ArduinoCloud.begin(ArduinoIoTPreferredConnection, false);

  setDebugMessageLevel(4);
  ArduinoCloud.printDebugInfo();

  //Serial1 pins 13-14 for 3.3V connection to GPS.
  Serial1.begin(9600);
  while (!Serial1) {}
  if (debug) {
    Serial.println("GPS Connected");
  }

}

void loop() {
  ArduinoCloud.update();
  String position_data(latitude, longitude);
  position_data = GetGPSposition();
  location = {latitude, longitude};
  ArduinoCloud.update();
  Serial.print(position_data);
  Serial.print("lat: ");
  Serial.print(latitude, 7);
  Serial.print("  Long: ");
  Serial.println(longitude, 7);
  Wait(1, false);
}

/*
  Since Latitude is READ_WRITE variable, onLatitudeChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onLatitudeChange()  {
  // Add your code here to act upon Latitude change
  ArduinoCloud.update();
}
/*
  Since Longitude is READ_WRITE variable, onLongitudeChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onLongitudeChange()  {
  // Add your code here to act upon Longitude change
  ArduinoCloud.update();
}