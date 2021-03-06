/*
  Base on the paperbot code on CCLE, we modify that the car can able to read all the sensors
  Using same communication method of paperbot, websocket, to communicate with computer
  Code modified by Loc Nguyen
  
  Wireless Servo Control, with ESP as Access Point

  Usage: 
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1. 
    A webpage with four buttons should appear. Click them to move the robot.

  Installation: 
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP

  Requirements:
    Arduino support for ESP8266 board
      In Arduino, add URL to Files > Preferences > Additional Board Managers URL.
      See https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon

    Websockets library
      To install, Sketch > Include Library > Manage Libraries... > Websockets > Install
      https://github.com/Links2004/arduinoWebSockets
    
    ESP8266FS tool
      To install, create "tools" folder in Arduino, download, and unzip. See 
      https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system

  Hardware: 
  * NodeMCU Amica DevKit Board (ESP8266 chip)
  * Motorshield for NodeMCU 
  * 2 continuous rotation servos plugged into motorshield pins D1, D2
  * Ultra-thin power bank 
  * Paper chassis

*/

#include <Arduino.h>

#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

#include <Servo.h>

#include "debug.h"
#include "file.h"
#include "server.h"

const int WHELL_RADIUS = 20; //mm
const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;
int tt=2187;
int PWML = 90;
int PWMR = 90;
int curL;
int curR;
int sp = 180;
float heading = 0; //angle from magnetometer
int rs1 = 0; // range sensor 1
int rs2 = 0; // range sensor 2

// Define for sensor
#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PORT 14
#define SCL_PORT 12
VL53L0X sensor;
VL53L0X sensor2;

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();

    
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}




// WiFi AP parameters
char ap_ssid[13];
char* ap_password = "";

// WiFi STA parameters
char* sta_ssid = 
  "...";
char* sta_password = 
  "...";

char* mDNS_name = "paperbot";

String html;
String css;

void setup() {
    setupPins();
    // Setup for sensor
    SensorSetup();
    sprintf(ap_ssid, "TIGER_%08X", ESP.getChipId());

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        LED_ON;
        delay(500);
        LED_OFF;
        delay(500);
    }
    LED_ON;
    //setupSTA(sta_ssid, sta_password);
    setupAP(ap_ssid, ap_password);
    LED_OFF;

    setupFile();
    html = loadFile("/controls.html");
    css = loadFile("/style.css");
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);

    setupHTTP();
    setupWS(webSocketEvent);
    //setupMDNS(mDNS_name);

    stop();
}

void SensorSetup()
{
    // Arduino initializations
  Wire.begin(SDA_PORT,SCL_PORT);
  
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);  
  
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  digitalWrite(D3, HIGH);
  delay(150);
  
  sensor.init(true);
  delay(100);
  sensor.setAddress((uint8_t)22);

  digitalWrite(D4, HIGH);
  delay(150);
  sensor2.init(true);
  delay(100);
  sensor2.setAddress((uint8_t)25);
  
  byte count = 0;

  for (byte i = 1; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  
  delay(1000);
}

void loop() {
    wsLoop();
    httpLoop();
}

void GetSensorData()
{
    // :::  Magnetometer ::: 

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t mx=(Mag[3]<<8 | Mag[2]);
  int16_t my=(Mag[1]<<8 | Mag[0]);
  int16_t mz=(Mag[5]<<8 | Mag[4]);
  
  // Shift mx and my to origin
  mx = mx - 73;
  my = my  + 109;
  
  heading = atan2(my, mx);

// Correct for when signs are reversed.
  if(heading < 0)
     heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Read range sensors
  rs1 = sensor.readRangeSingleMillimeters();
  rs2 = sensor2.readRangeSingleMillimeters();
}


//
// Movement Functions //
//

void drive(int left, int right) {
  curL = left;
  curR = right;
  servo_left.write(left);
  servo_right.write(180-right);
}

void f(int vl, int vr, int t) //t in milisecond
{
   drive(V2PWM_L(vl),V2PWM_R(vr));
   delay(t);
}

void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}

void forward() {
  DEBUG("forward");
  drive(sp, sp);
}

void backward() {
  DEBUG("backward");
  drive(180-sp, 180-sp);
}

void left() {
  DEBUG("left");
  drive(180-sp, sp);
}

void right() {
  DEBUG("right");
  drive(sp, 180-sp);
}

int V2PWM_L(int v)  //v (mm/s)
{
  v = (int)(60*v/(2*3.14*WHELL_RADIUS));
  if (v < -75) return 0;
  if (v > 75) return 180;
  return (int)(((float)v+190.9)/2.3786);
}

int V2PWM_R(int v)  //v (mm/s)
{
  v = (int)(60*v/(2*3.14*WHELL_RADIUS));
  if (v < -75) return 180;
  if (v > 75) return 0;
  return (int)(((float)v-221.38)/-2.38);
}

//
// Setup //
//

void setupPins() {
    // setup Serial, LEDs and Motors
    Serial.begin(115200);
    DEBUG("Started serial.");

    pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
    LED_OFF;                     //Turn off LED
    DEBUG("Setup LED pin.");

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    DEBUG("Setup motor pins");
}

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            break;
        case WStype_CONNECTED: 
        {
            // IPAddress ip = webSocket.remoteIP(id);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
            DEBUG("Web socket connected, id = ", id);

            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid);
            break;
        }
        case WStype_BIN:
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);

            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);

        case WStype_TEXT:
            DEBUG("On connection #", id)
            DEBUG("  got text: ", (char *)payload);

            switch (payload[0]){
              case '#':
                  if(payload[1] == 'C') {
                  LED_ON;
                  wsSend(id, "Hello world!");
                  //data path
                  
                  stop();

                                    
                  }
                  else if(payload[1] == 'F') 
                    forward();
                  else if(payload[1] == 'B') 
                    backward();
                  else if(payload[1] == 'L') 
                    left();
                  else if(payload[1] == 'R') 
                    right();
                  else if(payload[1] == 'U') {
                    if(payload[2] == 'L') 
                      PWML -= 1;
                    else if(payload[2] == 'R') 
                      tt += 1;
                      char tx[20] = "PWM @ (xxx, xxx)";
                      sprintf(tx, "PWM @ (%3d, %3d)", tt, tt);
                      wsSend(id, tx);
                    }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') 
                    PWML += 1;
                  else if(payload[2] == 'R') 
                    tt -= 1;
                  char tx[20] = "PWM @ (xxx, xxx)";
                  sprintf(tx, "PWM @ (%3d, %3d)", tt, tt);
                  wsSend(id, tx);
                }
                else 
                  stop();
            
            break;
          case '@':
                char tx[30];
                GetSensorData();
                sprintf(tx, "%d, %d, %d, %d, %f", curL, curR, rs1,rs2,heading);
                wsSend(id, tx);
                break;
          }     
          break;
    }
}
