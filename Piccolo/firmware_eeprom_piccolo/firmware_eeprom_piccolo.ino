//********************************************************************************//
//  Piccolo T-Stick Experimental - LOLIN D32 PRO - USB -WiFi                      //
//  Same as Sopranino but using built in Touch functions of the ESP32             //
//********************************************************************************//


// FIRMWARE VERSION: 99999

//  OBS:
//  1-) Use esp32 1.0.1 or newer (https://github.com/espressif/arduino-esp32/releases)
//  2-) Also install ESP8266 board library even if you'll use the ESP32 (https://github.com/esp8266/Arduino)
//  3-) Some used library doesn't allow the creation of functions with "setup" in the name
//  4-) Board currently in use: LOLIN D32 PRO
//  5-) Dont forget to change T-Stick Specific Definitions

#include <FS.h>  // this needs to be first, or it all crashes and burns...

#define ESP32; // define ESP8266 or nothing (to use ESP32) 

#if !defined(ESP8266)
#include "SPIFFS.h"
#endif

#include <WiFi32Manager.h> // https://github.com/edumeneses/WiFi32Manager
// already includes:
// Wifi.h (https://github.com/esp8266/Arduino) or ESP8266WiFi.h (https://github.com/esp8266/Arduino)
// AND
// WebServer.h or ESP8266WebServer.h
// AND
// DNSServer.h

#include <ArduinoJson.h>  // https://github.com/bblanchon/ArduinoJson
// Wifi.h requires ArduinoJson 5.13.5

#include <Wire.h>
#include <WiFiUdp.h>
#include <OSCMessage.h> // https://github.com/CNMAT/OSC

#include <SPI.h>
#include <Adafruit_LSM9DS1.h> // https://github.com/adafruit/Adafruit_LSM9DS1
#include <Adafruit_Sensor.h> // https://github.com/adafruit/Adafruit_Sensor


//////////////////////////////////
// T-Stick Specific Definitions //
//////////////////////////////////

char device[20] = "PiccoloWiFi-888";
char APpasswd[15] = "mappings";
char APpasswdValidate[15] = "mappings";
char APpasswdTemp[15] = "mappings";
int directSendOSC = 0;
char directSendOSCCHAR[3] = "0";
unsigned int infoTstick[2] = {888, 99999};    // serial number and firmware revision
char infoTstickCHAR0[6] = "888"; // same serial # to be saved in json
char infoTstickCHAR1[6] = "99999"; // same firmware # to be saved in json
int calibrate = 0;
char calibrateCHAR[5] = "0"; // same calibrate # to be saved in json

// Debug & calibration definitions
#define DEBUG false
#define CALIB false

//////////////////////////
// Capsense Definitions //
//////////////////////////

#define SENSOR_EN 0x00
#define FSS_EN 0x02
#define SENSITIVITY0 0x08
#define SENSITIVITY1 0x09
#define SENSITIVITY2 0x0A
#define SENSITIVITY3 0x0B
#define DEVICE_ID 0x90    // Should return 0xA05 (returns 2 bytes)
#define FAMILY_ID 0x8F
#define SYSTEM_STATUS 0x8A
#define I2C_ADDR 0x37     // Should return 0x37
#define REFRESH_CTRL 0x52
#define SENSOR_EN 0x00    // We should set it to 0xFF for 16 sensors
#define BUTTON_STAT 0xAA  // Here we red the status of the sensors (2 bytes)
#define CTRL_CMD 0x86     // To configure the Capsense
#define CTRL_CMD_STATUS 0x88
#define CTRL_CMD_ERROR 0x89
#define BUTTON_LBR 0x1F
#define SPO_CFG 0x4C      //CS15 configuration address
#define GPO_CFG 0x40
#define CALC_CRC 0x94
#define CONFIG_CRC 0x7E

//////////////////////
// WiFi Definitions //
//////////////////////

//define your default values here, if there are different values in config.json, they are overwritten.
IPAddress oscEndpointIP(192, 168, 5, 1); // remote IP of your computer
unsigned int oscEndpointPORT = 57120; // remote port to receive OSC
char oscIP[17] = "192.168.10.1";
char oscPORT[7] = "8000";
int timeout1 = 5000; bool timeout1check = false;
WiFiUDP oscEndpoint;            // A UDP instance to let us send and receive packets over UDP
const unsigned int portLocal = 8888;       // local port to listen for OSC packets (actually not used for sending)
bool udpConnected = false;
bool sendOSC = true;
static int bufferFromHost[4] = {0, 0, 0, 0};
int interTouch[2];
char zero[3] = "0";
char one[3] = "1";


//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////

// Adafruit library
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

float deltat = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0;    // used to calculate integration interval
uint32_t Now = 0;           // used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float pitch, yaw, roll, heading;
float outAccel[3] = {0, 0, 0};
float outGyro[3] = {0, 0, 0};
float outMag[3] = {0, 0, 0};

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value


////////////////////////////
// Sketch Output Settings //
////////////////////////////

#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 20 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -14.34 // Declination (degrees) in Montreal, QC.


//////////////
// defaults //
//////////////

int piezoPin = 32;
int pressurePin = 33;
int ledPin = 12; //changed for The thing dev during testing
int ledStatus = 0;
int ledTimer = 1000;
byte touch[2] = {0, 0};
int touchMask[2] = {255, 255};
char touchMaskCHAR0[7] = "255"; // same cal # to be saved in json
char touchMaskCHAR1[7] = "255"; // same cal # to be saved in json
unsigned int calibrationData[2] = {0, 1024};
char calibrationDataCHAR0[6] = "0"; // same cal # to be saved in json
char calibrationDataCHAR1[6] = "1024"; // same cal # to be saved in json
unsigned int pressure = 0;
//uint32_t dataTransferRate = 20; // sending data at 50Hz
uint32_t dataTransferRate = 1000; // sending data at 1Hz
uint32_t deltaTransferRate = 0;

int buttonPin = 35; //m5stick button

////////////////////////
//control definitions //
////////////////////////

unsigned long then = 0;
unsigned long now = 0;
unsigned long started = 0;
unsigned long lastRead = 0;
byte interval = 10;
byte touchInterval = 15;
unsigned long lastReadSensors = 0;


void setup() {
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(115200);
  if (DEBUG == true) {
    Serial.println("\n Starting");
  }

  // Starting WiFiManager in Trigger Mode
  Wifimanager_init(DEBUG);

#if defined(ESP8266)
  WiFi.hostname(device);
#else
  WiFi.setHostname(device);
#endif

  //initIMU();

  //initCapsense();

  pinMode(ledPin, OUTPUT);

  delay(100);

  //  if(DEBUG==true){
  //    if (WiFi.status() == WL_CONNECTED) {
  //      Serial.println("Connected to the saved network");
  //      Serial.print("Connected to: "); Serial.println(WiFi.SSID());
  //      Serial.print("IP address: "); Serial.println(WiFi.localIP()); // Print Ip on serial monitor or any serial debugger
  //      WiFi.printDiag(Serial); // Remove this line if you do not want to see WiFi password printed
  //      delay(100);
  //      // Starting UDP
  //      //oscEndpoint.begin(portLocal);
  //      udpConnected = connectUDP();
  //    }
  //    else {
  //      Serial.println("Failed to connect, finishing setup anyway");
  //      Serial.print("Saved SSID: "); Serial.println(WiFi.SSID());
  //      WiFi.printDiag(Serial); // Remove this line if you do not want to see WiFi password printed
  //    }
  //  }
  //
  //  delay(100);

  if (DEBUG == true) {
    Serial.println("Setup complete.");
  }
}

void loop() {

  // Calling WiFiManager configuration portal
  if ( outAccel[0] > 1 && interTouch[0] == 9 && interTouch[1] == 144 ) {
    Wifimanager_portal(device, APpasswd, true, DEBUG);
  }

  if (digitalRead(buttonPin) == 0) {
    ESP_LOGD("main", "Button pressed; opening portal");
    Wifimanager_portal(device, APpasswd, true, DEBUG);
  }

  byte dataRec = OSCMsgReceive();

  if (dataRec) { //Check for OSC messages from host computers
    if (DEBUG) {
      Serial.println();
      for (int i = 0; i < 4; i++) {
        Serial.printf("From computer %d: ", i); Serial.println(bufferFromHost[i]);
      }
      Serial.println();
    }
    char message = bufferFromHost[0];

    OSCMessage msg0("/information");

    switch (message) {
      case 's': // start message,
        msg0.add(infoTstick[0]);
        msg0.add(infoTstick[1]);
        oscEndpoint.beginPacket(oscEndpointIP, oscEndpointPORT);
        msg0.send(oscEndpoint);
        oscEndpoint.endPacket();
        msg0.empty();

        started = millis();
        break;
      case 'x': // stop message,
        started = 0;
        break;
      case 'c': // calibrate message
        switch (bufferFromHost[1]) {
          case 1: // FSR calibration
            calibrationData[0] = bufferFromHost[2];
            calibrationData[1] = bufferFromHost[3];
            calibrate = 1;
            bufferFromHost[1] = 0;
            bufferFromHost[2] = 0;
            bufferFromHost[3] = 0;
            break;
          default:
            calibrate = 0;
            break;
        }
        break;
      case 'w':     //write settings
        switch ((char)bufferFromHost[1]) {
          case 'i': //write info
            infoTstick[0] = bufferFromHost[2];
            infoTstick[1] = bufferFromHost[3];
            bufferFromHost[1] = 0;
            bufferFromHost[2] = 0;
            bufferFromHost[3] = 0;
            break;
          case 'T': //write touch mask
            touchMask[0] = bufferFromHost[2];
            touchMask[1] = bufferFromHost[3];
            bufferFromHost[1] = 0;
            bufferFromHost[2] = 0;
            bufferFromHost[3] = 0;
            break;
          case 'w': // write settings to memory (json)
            save_to_json(DEBUG);
            bufferFromHost[1] = 0;
            break;
          case 'r': // sending the config info trough OSC
            msg0.add(infoTstick[0]);
            msg0.add(infoTstick[1]);
            msg0.add(calibrate);
            msg0.add(calibrationData[0]);
            msg0.add(calibrationData[1]);
            msg0.add(touchMask[0]);
            msg0.add(touchMask[1]);
            oscEndpoint.beginPacket(oscEndpointIP, oscEndpointPORT);
            msg0.send(oscEndpoint);
            oscEndpoint.endPacket();
            msg0.empty();
            bufferFromHost[1] = 0;
            break;
          default:
            break;
        }
      default:
        break;
    }
  }

  now = millis();

  if (directSendOSC == 0) {
    if (now < started + 2000) { // needs a ping/keep-alive every 2 seconds or less or will time-out
      TStickRoutine();
    }
    else { // runs when there's no ping/keep-alive
      now = millis();

      // Calling configuration portal if the T-Stick is disconnected
      if (millis() - lastRead > interval) {
        lastRead = millis();
        pressure = analogRead(pressurePin);
      }
      if ( pressure > 1000 ) {
        Wifimanager_portal(device, APpasswd, true, DEBUG);
      }
      else {
        if ((now - then) > ledTimer) {
          ledBlink();
          then = now;
        }
        else if ( then > now) {
          then = 0;
        }
      }
    }
  } else {
    TStickRoutine();
  }

} // END LOOP

void ledBlink() {
  ledStatus = (ledStatus + 1) % 2;
  digitalWrite(ledPin, ledStatus);
}
