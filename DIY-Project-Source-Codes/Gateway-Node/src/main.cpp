// ---------------------------------- make2explore.com -------------------------------------------------------//
// Project           - Application of LoRa WSN in Landslide Monitoring/Detection/Prevention Systems
// Created By        - info@make2explore.com
// Last Modified     - 14/02/2023 01:26:00 @admin
// Software          - C/C++, PlatformIO IDE, Visual Studio Code Editor, Libraries
// Hardware          - Arduino Mega 2560, NodeMCU ESP8266, Wio Terminal, Wio E5 Dev Boards, Displays, Sensors         
// Sensors Used      - DHT22, Soil Moisture - Resistive, Capacitive, Vibration, Rain, ADXL345 Accelerometer
// Source Repo       - github.com/make2explore
// -----------------------------------------------------------------------------------------------------------//
// This code is of Wireless (LoRa) **Gateway Node**. Which will be *Relaying* data coming from LoRa WSN 
// deployed at Landslide prone site, To the End node, Where concerned Authorities can monitor/Analyse 
// the data to take Prevention/Rescue Measures.
// physical data (Rain + Soil Moisture + Humidity + Temperature + Vibrations + Descending Velocity)
// GW Node also send its own location's parameters (Rain+Humidity+Temp) data for comparing actual situations. 
// It helps in calculating perimeter of Raining area. Since Sensor node and GW node are at about 5-7Km 
// away/distant from each other.

// The main Objective of this project is to test the use of LoRa WSN in such "Landslide Monitoring" systems
// in order to achieve good data reception and reliablity in hilly Areas, "Shadow Regions" or 
// Remote Areas with very low coverage of other Wired/Wireless Networks (e.g GSM/WiFi/Ethernet etc.)
// -----------------------------------------------------------------------------------------------------------//

// Include Libraries
#include <Arduino.h>
#include <SPI.h>              // SPI Library needed for display
#include <TFT_eSPI.h>         // Graphics library
#include "m2e-logo.h"         // make2explore Logo Bitmap header file
#include <SoftwareSerial.h>   // Software Serial Library for communicating with Wio E5 Mini Board
#include "DHT.h"              // Include DHT Sensors library

// Invoke Display and Create display Instance 
TFT_eSPI tft = TFT_eSPI();

// Lets define Sofware serial Pins for WIo E5 Mini Dev Board
const byte rxPin = D2;
const byte txPin = D3;

// Set up a new SoftwareSerial object
SoftwareSerial e5 (rxPin, txPin);

// LoRa Data receive buffer
static char recv_buf[512];
static bool is_exist = false;

// Rain Sensor (10K Pot as Tipping bucket Rain Gauge) attached to Analog Pin A0
const int rainSensor = A0;  // ESP8266 Analog Pin ADC0 = A0 for tipping bucketRain Sensor
// Rain Sensors Calibration values
const uint16_t noRain = 21;
const uint16_t FullRain = 1024;

// DHT Sensor Definitions
#define DHTPIN D6     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

// Sensor variable declarations
uint8_t GW_temperature, GW_humidity, GW_rain_per; 

// Variables for collecting Sensor data and parameters
// prfix SN is for data received from (WSN) Sensor Node
uint8_t SN_m1, SN_m2, SN_humi, SN_temp, SN_rain_per;
float SN_disp;
bool SN_vib, SN_stat;
int RSSI, SNR;

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
        int rss = 0;
        int snr = 0;

        p_start = strstr(recv_buf, "+TEST: RX \"47572C");
        if (p_start)
        {
            p_start = strstr(recv_buf, "47572C");
            if (p_start && (1 == sscanf(p_start, "47572C%s", data)))
            {
              data[70] = 0;
              //Serial.println(data);
              char output[128];
              char* text = unHex(data, output, sizeof(output));
              //Serial.println(text);
              SN_m1 = (getValue(text, ',', 0)).toInt();
              SN_m2 = (getValue(text, ',', 1)).toInt();
              SN_rain_per = (getValue(text, ',', 2)).toInt();
              SN_humi = (getValue(text, ',', 3)).toInt();
              SN_temp = (getValue(text, ',', 4)).toInt();
              SN_disp = (getValue(text, ',', 5)).toFloat();
              SN_vib = (getValue(text, ',', 6)).toInt();
              SN_stat = (getValue(text, ',', 7)).toInt();
              Serial.println("\r\n");

            }
            p_start = strstr(recv_buf, "RSSI:");
            if (p_start && (1 == sscanf(p_start, "RSSI:%d,", &rss))){
              RSSI = rss;
            }

            p_start = strstr(recv_buf, "SNR:");
            if (p_start && (1 == sscanf(p_start, "SNR:%d", &snr))){
              SNR = snr;
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

// Function for LoRa packet preparation and sending
static int LoRa_send()
{
  String sensorData = "";
  char cmd[256] = "";
  int ret = 0;
  char data[128] = "";

  sensorData = sensorData + String(SN_m1) + "," + String(SN_m2) + "," + String(SN_rain_per) + "," + String(SN_humi) 
                + "," + String(SN_temp) + "," + String(SN_disp) + "," + String(SN_vib) + "," + String(SN_stat) + "," 
                + String(GW_rain_per) + "," + String(GW_humidity) + "," + String(GW_temperature);

  strncpy(data,sensorData.c_str(),sizeof(data));
  data[sizeof(data) -1] = 0;
  
  sprintf(cmd, "AT+TEST=TXLRSTR,\"EN,%s\"\r\n", data);
  //Serial.print("Printing cmd String : ");
  //Serial.println(cmd);

  ret = at_send_check_response("TX DONE", 6000, cmd);
    if (ret == 1)
    {
      Serial.println("");
      Serial.print("Sent successfully!\r\n");
    }
    else
    {
      Serial.println("");
      Serial.print("Send failed!\r\n");
    }
  return ret;
}

// Function for First receive data from WSN then Relay(Send) to End Node via LoRa
static void node_recv_then_send(uint32_t timeout)
{
    int ret = 0;
    ret = node_recv(timeout);
    delay(100);
    if (!ret)
    {
        Serial.print("\r\n");
        return;
    }
    LoRa_send();
    Serial.print("\r\n");
}

// Function to configure Wio E5 Mini in Test Mode - Check AT commands Specification Guide
// for more details about these command sequences  
void configLoRaModule(){
  //Configure Wio E5 Mini Board in Test Mode
  Serial.println("Configuring Wio E5 Mini Board ...");
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

// Function to Setup Display for Initial Screen
void HomeScreen(){
  tft.fillScreen(TFT_BLACK);                    // Black background
  tft.drawRect(0,0,239,239,TFT_WHITE);       // A 100x100 black rectangle starting from (110, 70)
  tft.fillRect(0,0,240,40,TFT_BROWN);
  tft.setTextColor(TFT_WHITE);          //sets the text colour to black
  tft.setTextSize(2);                   //sets the size of text
  tft.drawString("LoRa Gateway Node", 14,12);
  tft.setTextColor(TFT_YELLOW);          //sets the text colour to black
  tft.drawString("GW node local data", 12,55);

  tft.setTextColor(TFT_WHITE); 
  tft.drawString("Rain = ", 10, 85);
  tft.drawString("%", 215, 85);

  tft.drawString("Humi = ", 10, 110);
  tft.drawString("%", 215, 110);

  tft.drawString("Temp = ", 10, 135);
  tft.drawString("C", 215,135);

  tft.drawString("RSSI = ", 10, 160);
  tft.drawString("dBm", 195, 160);

  tft.drawString("SNR  = ", 10, 185);
  tft.drawString("dB", 205, 185);

  tft.drawString("Stat = ", 10, 210);
}

// Function to Display the sensor Readings
void displayReadings(){

  tft.setTextColor(TFT_CYAN, TFT_BLACK); 

  tft.drawFloat(GW_rain_per, 2, 100, 85);
  
  tft.drawFloat(GW_humidity, 2, 100, 110);

  tft.drawFloat(GW_temperature, 2, 100, 135);

  tft.drawNumber(RSSI, 100, 160);

  tft.drawNumber(SNR, 100, 185);

  if(SN_stat == 1){
    tft.drawString("Alert!", 100, 210);
  } else {
    tft.drawString("OK", 100, 210);
  }
  
}

// Function to Initialise DHT Sensor and Check Readings
void getDHTReadings(){
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  GW_temperature = t;
  GW_humidity = h;
  //Serial.print("Humidity: ");
  //Serial.print(h);
  //Serial.println("%");
  //Serial.print("Temperature: ");
  //Serial.print(t);
  //Serial.println("°C ");      
}

// Check Rain Sensor (10K Pot as Tipping bucket Rain Gauge) attached to Analog Pin A0
void getRainReading(){
  GW_rain_per = map(analogRead(rainSensor), noRain, FullRain, 0, 100);
  //Serial.print("Rain Percentage : ");
  //Serial.print(rain_per);
  //Serial.println("%");
}

// Function to Setup the Initializations and Configurations
void setup(void) {
  
  Serial.begin (9600);
  e5.begin(9600);
  tft.begin ();                                 // initialize a ST7789 chip
  tft.setSwapBytes (true);                      // swap the byte order for pushImage() - corrects endianness

  tft.setRotation(0);
  tft.fillScreen (TFT_BLACK);
  tft.pushImage (0,0,240,240,m2elogo);
  delay(3000);
  
  HomeScreen();         // Display Home Screen

  configLoRaModule();   // Configure Wio E5 Mini Dev Board

  dht.begin();          // Init DHT Sensor
}

// Function main Loop
void loop() {
  if (is_exist)
  {
    getDHTReadings();
    getRainReading();
    displayReadings();
    node_recv_then_send(5000);
  }
}

// ---------------------------------- make2explore.com----------------------------------------------------//