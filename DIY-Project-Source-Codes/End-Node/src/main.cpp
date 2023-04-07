// ---------------------------------- make2explore.com -------------------------------------------------------//
// Project           - Application of LoRa WSN in Landslide Monitoring/Detection/Prevention Systems
// Created By        - info@make2explore.com
// Last Modified     - 14/02/2023 01:26:00 @admin
// Software          - C/C++, PlatformIO IDE, Visual Studio Code Editor, Libraries
// Hardware          - Arduino Mega 2560, NodeMCU ESP8266, Wio Terminal, Wio-E5 Dev Boards, Displays, Sensors         
// Sensors Used      - DHT22, Soil Moisture - Resistive, Capacitive, Vibration, Rain, ADXL345 Accelerometer
// Source Repo       - github.com/make2explore
// -----------------------------------------------------------------------------------------------------------//
// This code is of Wireless (LoRa) **End Node**. Which will be *Displaying* data coming from LoRa WSN 
// deployed at Landslide prone site. Here concerned Authorities can monitor/Analyse 
// the data to take Prevention/Rescue Measures.

// Data from Sensor Node (Rain + Soil Moisture + Humidity + Temperature + Vibrations + Descending Velocity)
// Data from GW Node (Rain+Humidity+Temp) data for comparing actual situations. 

// All data coming from SN and GW Node will be displayed at End Node. Here we can implement more complex
// System using ML and AI for predition of Landslide

// The main Objective of this project is to test the use of LoRa WSN in such "Landslide Monitoring" systems
// in order to achieve good data reception and reliablity in hilly Areas, "Shadow Regions" or 
// Remote Areas with very low coverage of other Wired/Wireless Networks (e.g GSM/WiFi/Ethernet etc.)
// -----------------------------------------------------------------------------------------------------------//

// Include Libraries
#include <Arduino.h>
#include <SoftwareSerial.h>   // Software Serial Library for communicating with Wio E5 LoRa Module
#include "TFT_eSPI.h"         // Include TFT LCD library 
#include "Free_Fonts.h"       // Include free fonts library 
#include "Seeed_FS.h"         // Including SD card library
#include "RawImage.h"         // Including image processing library

// Invoke Display and Create display Instance 
TFT_eSPI tft; //initialize TFT LCD

// Lets define Sofware serial Pins for Wio-E5 Module
const byte rxPin = D0;
const byte txPin = D1;

// Set up a new SoftwareSerial object
SoftwareSerial e5 (rxPin, txPin);

// LoRa Data receive buffer
static char recv_buf[512];
static bool is_exist = false;

// Variables for collecting Sensor data and parameters
// prfix SN is for data received from (WSN) Sensor Node
float SN_m1, SN_m2, SN_humi, SN_temp, SN_disp, SN_rain_per, GW_temperature, GW_humidity, GW_rain_per;;
bool SN_vib, SN_stat;

// Readings Update Interval Settings
const unsigned long updateInterval = 5000;
unsigned long previousTime = 0;
unsigned long previousUpdateTime = 0;

// Function for parsing the incomming data String
String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;
 
    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Function for Hex to ASCII conversion
byte aNibble(char in) {
  if (in >= '0' && in <= '9') {
    return in - '0';
  } else if (in >= 'a' && in <= 'f') {
    return in - 'a' + 10;
  } else if (in >= 'A' && in <= 'F') {
    return in - 'A' + 10;
  }
  return 0;
}

// Function for Hex to ASCII conversion
char * unHex(const char* input, char* target, size_t len) {
  if (target != nullptr && len) {
    size_t inLen = strlen(input);
    if (inLen & 1) {
      Serial.println(F("unhex: malformed input"));
    }
    size_t chars = inLen / 2;
    if (chars >= len) {
      Serial.println(F("unhex: target buffer too small"));
      chars = len - 1;
    }
    for (size_t i = 0; i < chars; i++) {
      target[i] = aNibble(*input++);
      target[i] <<= 4;
      target[i] |= aNibble(*input++);
    }
    target[chars] = 0;
  } else {
    Serial.println(F("unhex: no target buffer"));
  }
  return target;
}

// Function to check response for AT commands
static int at_send_check_response(char *p_ack, int timeout_ms, char *p_cmd, ...)
{
    int ch = 0;
    int index = 0;
    int startMillis = 0;
    va_list args;
    memset(recv_buf, 0, sizeof(recv_buf));
    va_start(args, p_cmd);
    e5.printf(p_cmd, args);
    Serial.printf(p_cmd, args);
    va_end(args);
    delay(200);
    startMillis = millis();
 
    if (p_ack == NULL)
    {
        return 0;
    }
    do
    {
        while (e5.available() > 0)
        {
            ch = e5.read();
            recv_buf[index++] = ch;
            Serial.print((char)ch);
            delay(2);
        }
        if (strstr(recv_buf, p_ack) != NULL)
        {
            return 1;
        }
 
    } while (millis() - startMillis < timeout_ms);
    return 0;
}

// Function for parsing the incoming LoRa data
static int recv_parse(void)
{
    char ch;
    int index = 0;
    memset(recv_buf, 0, sizeof(recv_buf));
    while (e5.available() > 0)
    {
        ch = e5.read();
        recv_buf[index++] = ch;
        Serial.print((char)ch);
        delay(2);
    }
 
    if (index)
    {
        char *p_start = NULL;
        char data[128] = {
            0,
        };

        p_start = strstr(recv_buf, "+TEST: RX \"454E2C");
        if (p_start)
        {
            p_start = strstr(recv_buf, "454E2C");
            if (p_start && (1 == sscanf(p_start, "454E2C%s", data)))
            {
                data[60] = 0;
                //Serial.println(data);
                //Serial.println("Hello");
                char output[128];
                char* text = unHex(data, output, sizeof(output));
                SN_m1 = (getValue(text, ',', 0)).toFloat();
                SN_m2 = (getValue(text, ',', 1)).toFloat();
                SN_rain_per = (getValue(text, ',', 2)).toFloat();
                SN_humi = (getValue(text, ',', 3)).toFloat();
                SN_temp = (getValue(text, ',', 4)).toFloat();
                SN_disp = (getValue(text, ',', 5)).toFloat();
                SN_vib = (getValue(text, ',', 6)).toInt();
                SN_stat = (getValue(text, ',', 7)).toInt();

                GW_rain_per = (getValue(text, ',', 8)).toFloat();  
                GW_humidity = (getValue(text, ',', 9)).toFloat();  
                GW_temperature = (getValue(text, ',', 10)).toFloat();
                //Serial.println(GW_temperature);              
                Serial.print("\r\n");
            }
            return 1;
        }
    }
    return 0;
}


// Function for Receiving incomming LoRa Packets
static int node_recv(uint32_t timeout_ms)
{
    at_send_check_response("+TEST: RXLRPKT", 1500, "AT+TEST=RXLRPKT\r\n");
    int startMillis = millis();
    do
    {
        if (recv_parse())
        {
            return 1;
        }
    } while (millis() - startMillis < timeout_ms);
    return 0;
}


// Function to configure Wio-E5 Module in Test Mode - Check AT commands Specification Guide
// for more details about these command sequences  
void configLoRaModule(){
  //Configure Wio E5 Module in Test Mode
  Serial.println("Configuring Wio E5 Module ...");
  delay(250);
  if (at_send_check_response("+AT: OK", 100, "AT\r\n"))
  {
    is_exist = true;
    at_send_check_response("+MODE: TEST", 1500, "AT+MODE=TEST\r\n");
    at_send_check_response("+TEST: RFCFG", 1500, "AT+TEST=RFCFG,866,SF12,125,12,15,14,ON,OFF,OFF\r\n");
    delay(500);
  }
  else
  {
    is_exist = false;
    Serial.print("No E5 module found.\r\n");
  }
  delay(500);
}


// Function to Display the Sensor Readings (WSN)
void DisplayReadings1(){

    drawImage<uint8_t>("m2e-SN.bmp", 0, 0); //Display this 8-bit image in sd card from (0, 0)
    tft.setTextColor(TFT_WHITE);
    tft.setFreeFont(&FreeSerifBold9pt7b); //set font type 

    tft.drawFloat(SN_m1,2,93,62); //draw text string

    tft.drawFloat(SN_m2,2,249,60); //draw text string

    tft.drawFloat(SN_rain_per,2,93,110); //draw text string

    tft.drawFloat(SN_disp,2,244,111); //draw text string

    tft.drawFloat(SN_temp,2,240,160); //draw text string

    tft.drawFloat(SN_humi,2,86,209); //draw text string

    if(SN_vib == 1){
        tft.setTextColor(TFT_RED);
        tft.drawString("Detected",87,162);
    } else {
        tft.setTextColor(TFT_WHITE);
        tft.drawString("Not Det",87,162);
    }
    
    if(SN_stat == 1){
        tft.setTextColor(TFT_RED);
        tft.drawString("Alert !",250,209);
    } else {
        tft.setTextColor(TFT_GREEN);
        tft.drawString("OK",250,209);
    }

}

// Function to Display the Sensor Readings (GW Node)
void DisplayReadings2(){

  drawImage<uint8_t>("m2e-GW.bmp", 0, 0); //Display this 8-bit image in sd card from (0, 0)
  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(&FreeSerifBold9pt7b); //set font type 
  tft.drawFloat(GW_rain_per,2,197,70); //draw text string

  tft.drawFloat(GW_temperature,2,82,126); //draw text string

  tft.drawFloat(GW_humidity,2,237,126); //draw text string
  
  tft.setFreeFont(&FreeSerifBold12pt7b); //set font type 
  if(SN_stat == 1){
    tft.setTextColor(TFT_RED);
    tft.drawString("Alert !",200,195);
  } else {
    tft.setTextColor(TFT_GREEN);
    tft.drawString("OK",200,195);
  }
  
}

// Function to Setup Display for Initial Screen - make2explore logo
void HomeScreen(){
  drawImage<uint16_t>("WioTerminal-screen-m2e.bmp", 0, 0); //Display this 8-bit image in sd card from (0, 0)
}

// Function to Setup Display for First (WSN Data) Screen
void FirstScreen(){
  drawImage<uint8_t>("m2e-SN.bmp", 0, 0); //Display this 8-bit image in sd card from (0, 0)
}

// Function to Setup Display for First (GW Node Data) Screen
void secondScreen(){
  drawImage<uint8_t>("m2e-GW.bmp", 0, 0); //Display this 8-bit image in sd card from (0, 0)
}

// Function to Setup the Initializations and Configurations
void setup() {
    Serial.begin (9600);
    e5.begin(9600);
    //Initialise SD card
    if (!SD.begin(SDCARD_SS_PIN, SDCARD_SPI)) {
        while (1);
    }

    pinMode(WIO_5S_PRESS, INPUT_PULLUP);

    tft.begin(); //start TFT LCD 
    tft.setRotation(1); //set screen rotation 

    configLoRaModule();

    HomeScreen();
    delay(3000);
    FirstScreen();
    
}

// Function main Loop
void loop() {
    if (is_exist)
    {
        node_recv(6000);
        DisplayReadings1();
    }

    if (digitalRead(WIO_5S_PRESS) == LOW) {
        DisplayReadings2();
        delay(2000);
    }
}
// ---------------------------------- make2explore.com ----------------------------------------------------//