// ---------------------------------- make2explore.com -------------------------------------------------------//
// Project           - Application of LoRa WSN in Landslide Monitoring/Detection/Prevention Systems
// Created By        - info@make2explore.com
// Last Modified     - 14/02/2023 12:30:00 @admin
// Software          - C/C++, PlatformIO IDE, Visual Studio Code Editor, Libraries
// Hardware          - Arduino Mega 2560, NodeMCU ESP8266, Wio Terminal, Wio-E5 Dev Boards, Displays, Sensors         
// Sensors Used      - DHT22, Soil Moisture - Resistive, Capacitive, Vibration, Rain, ADXL345 Accelerometer
// Source Repo       - github.com/make2explore
// -----------------------------------------------------------------------------------------------------------//
// This code is of **Wireless (LoRa) Sensor Node**. Which will be collecting data from Landslide prone site
// physical data (Rain + Soil Moisture + Humidity + Temperature + Vibrations + Descending Velocity)
// and sending it to Gateway Node in order relay it over End node

// The main Objective of this project is to test the use of LoRa WSN in such "Landslide Monitoring" systems
// in order to achieve good data reception and reliablity in hilly Areas, "Shadow Regions" or 
// Remote Areas with very low coverage of other Wired/Wireless Networks (e.g GSM/WiFi/Ethernet etc.)
// -----------------------------------------------------------------------------------------------------------//

// Include Libraries
#include <Arduino.h>
#include <Adafruit_GFX.h>       // Include core graphics library
#include <Adafruit_ST7735.h>    // Include Adafruit_ST7735 library to drive the display
#include "DHT.h"                // Include DHT Sensors library
#include <Adafruit_Sensor.h>    // Include Generic Sensor Library
#include <Adafruit_ADXL345_U.h> // Include MEMS ADXL345 Sensor Library

// Declare pins for the display:
#define TFT_CS     53
#define TFT_RST    49  // You can also connect this to the Arduino reset in which case, set this #define pin to -1!
#define TFT_DC     48
// The rest of the pins are pre-selected as the default hardware SPI for Arduino Mega (SCK = 52 and SDA = 51)

// Vibration sensor connected to pin 8
#define vibSensor_pin 8

// Invoke Display and Create display Instance 
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// LoRa Data receive buffer
static char recv_buf[512];
static bool is_exist = false;

// DHT Sensor Definitions
#define DHTPIN 9     // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);

// Sensor variable declarations
// m1  = Capacitive Soil moisture
// m2  = Resistive Soil moisture
uint8_t m1, m2, humi, temp, rain_per;
float disp;
bool vib, stat;
String status = "";

// Soil Moisture Sensors Calibration values
const int AirValueM1 = 877;   //replace the value with value when placed in air using calibration code 
const int WaterValueM1 = 480; //replace the value with value when placed in water using calibration code 

const int AirValueM2 = 1023;   //replace the value with value when placed in air using calibration code 
const int WaterValueM2 = 336; //replace the value with value when placed in water using calibration code 

// Rain Sensor (10K Pot as Tipping bucket Rain Gauge) attached to Analog Pin A2
const int rainSensor = A2;  // Mega2560 Analog Pin A2 = A2 for tipping bucket Rain Sensor

// Soil Moisture Sensors Connections
const int CapSoil = A0;
const int ResSoil = A1;

// Rain Sensors Calibration values
const uint16_t noRain = 21;
const uint16_t FullRain = 1024;

// Readings Update Interval Settings
const unsigned long sendInterval = 20000;
const unsigned long updateInterval = 5000;

unsigned long previousTime = 0;
unsigned long previousUpdateTime = 0;

// ADXL345 Accelerometer 
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();


// Function to check response for AT commands
static int at_send_check_response(char *p_ack, int timeout_ms, char *p_cmd, ...)
{
    int ch;
    int index = 0;
    int startMillis = 0;
    va_list args;
    memset(recv_buf, 0, sizeof(recv_buf));
    va_start(args, p_cmd);
    Serial1.print(p_cmd);
    Serial.print(p_cmd);
    va_end(args);
    delay(200);
    startMillis = millis();

    if (p_ack == NULL)
        return 0;

    do
    {
        while (Serial1.available() > 0)
        {
            ch = Serial1.read();
            recv_buf[index++] = ch;
            Serial.print((char)ch);
            delay(2);
        }

        if (strstr(recv_buf, p_ack) != NULL)
            return 1;

    } while (millis() - startMillis < timeout_ms);
    Serial.println();
    return 0;
}

// Function to configure Wio-E5 LoRa Dev Board in Test Mode - Check AT commands Specification Guide
// for more details about these command sequences  
void configLoRaModule(){
  //Configure LoRa E5 Dev Kit in Test Mode
  Serial.println("Configuring Wio E5 LoRa Dev Board ...");
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
void setupDisplay(){
  // Display setup:
  Serial.println("");
  Serial.println("Setting up Display ...");
  delay(250);
  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);  // Initialize a ST7735S chip, black tab

  tft.fillScreen(ST7735_BLACK);  // Fill screen with black

  tft.setRotation(2);  // Set orientation of the display. Values are from 0 to 3. If not declared, orientation would be 0,
                         // which is portrait mode.

  tft.setTextWrap(false);  // By default, long lines of text are set to automatically “wrap” back to the leftmost column.
                           // To override this behavior (so text will run off the right side of the display - useful for
                           // scrolling marquee effects), use setTextWrap(false). The normal wrapping behavior is restored
                           // with setTextWrap(true).

  tft.fillRect(0, 0, 127, 20, ST7735_MAGENTA); // Draw filled rectangle (x,y,width,height,color)
  tft.setCursor(15, 7);  // Set position (x,y)
  tft.setTextColor(ST7735_WHITE);  // Set color of text. First is the color of text and after is color of background
  tft.setTextSize(1);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
  tft.println("LoRa Sensor Node");  // Print a text or value

  // Stop using a custom font:
  tft.setFont();  // Reset to standard font, to stop using any custom font previously set
  tft.setTextSize(1);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
  tft.setCursor(5, 28);  // Set position (x,y)
  //tft.setTextColor(ST7735_WHITE);  // Set color of text. First is the color of text and after is color of background
  tft.println("Soil M1 : ");  // Print a text or value
  tft.setCursor(110, 28);  // Set position (x,y)
  tft.print('%');

  tft.setCursor(5, 45);  // Set position (x,y)
  tft.println("Soil M2 : ");  // Print a text or value
  tft.setCursor(110, 45);  // Set position (x,y)
  tft.print('%');

  tft.setCursor(5, 60);  // Set position (x,y)
  tft.println("Rain Pr : ");  // Print a text or value
  tft.setCursor(110, 60);  // Set position (x,y)
  tft.print('%');

  tft.setCursor(5, 75);  // Set position (x,y)
  tft.println("Humi Pr : ");  // Print a text or value
  tft.setCursor(110, 75);  // Set position (x,y)
  tft.print('%');

  tft.setCursor(5, 90);  // Set position (x,y)
  tft.println("Temp Rd : ");  // Print a text or value
  tft.setCursor(110, 90);  // Set position (x,y)
  tft.print('C');

  tft.setCursor(5, 105);  // Set position (x,y)
  tft.println("Vibr Dt : ");  // Print a text or value
  tft.setCursor(110, 105);  // Set position (x,y)
  tft.println("Ct");

  tft.setCursor(5, 120);  // Set position (x,y)
  tft.println("Disp Ac : ");  // Print a text or value
  tft.setCursor(110, 120);  // Set position (x,y)
  tft.println("m/s");

  tft.fillRect(1,142,127,18, ST7735_BLUE);
  tft.setCursor(15, 148);  // Set position (x,y)
  tft.setTextColor(ST7735_WHITE);
  tft.println("Status : ");  // Print a text or value
}

// Function to Initialise DHT Sensor and Check Readings
void checkDHT(){
  // Wait a few seconds between measurements.
  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  temp = t;
  humi = h;

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  //Serial.print(F("Humidity: "));
  //Serial.print(h);
  //Serial.print(F("%  Temperature: "));
  //Serial.print(t);
  //Serial.print(F("°C "));

}

// Function to Test ADXL345 Accelerometer
void init_accel(){
  if(!accel.begin())
  {
    Serial.println("No ADXL345 sensor detected.");
    while(1);
  }
}

// Function to Get Readings from ADXL345 Accelerometer
void getAccel(){
  sensors_event_t event; 
  accel.getEvent(&event);
  // Wait a few seconds between measurements.
  disp = event.acceleration.x;
  //Serial.print("Accelearation : ");
  //Serial.println(disp);
  //elay(2000);  
}

// Function to Get Readings from Soil Moisture Sensors
void getSoilM(){
  //Serial.print("Capacitive Soil Moisture : ");
  //Serial.println(analogRead(CapSoil));
  //Serial.print("Resistive Soil Moisture : ");
  //Serial.println(analogRead(ResSoil));  
  m1 = map(analogRead(CapSoil), AirValueM1, WaterValueM1, 0, 100);
  m2 = map(analogRead(ResSoil), AirValueM2, WaterValueM2, 0, 100);
}

// Check Rain Sensor (10K Pot as Tipping bucket Rain Gauge) attached to Analog Pin A2
void checkRain(){
  rain_per = map(analogRead(rainSensor), noRain, FullRain, 0, 100);
}

// Function to Get All Sensor Readings at a time
void getReadings(){
  checkDHT();
  init_accel();
  getSoilM();
  checkRain();
  getAccel();
  vib = digitalRead(vibSensor_pin);
}

// Function for checking critical landslide conditions
void checkStatus(){
  if((m1 < 60) && (m2 < 60) && (rain_per < 50) && (humi < 60) && (temp > 25) && (disp < 1) && (vib == 0)){
    status = "Normal";
  }
  else if((m1 > 60) && (m2 > 60) && (rain_per > 50) && (humi > 60) && (temp < 25)){
    if ((disp > 1) && (vib == 1)){
      status = "Alert";
      stat = 1;
    }
  else
    status = "Normal";
    stat = 0;
  }
}

// Function to Display Readings from Sensors Locally
void displayReadings(){

  tft.setTextSize(1);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
  tft.setCursor(70, 28);  // Set position (x,y)
  tft.setTextColor(ST7735_CYAN, ST7735_BLACK);  // Set color of text. First is the color of text and after is color of background
  tft.println(m1);  // Print a text or value

  tft.setCursor(70, 45);  // Set position (x,y)
  tft.println(m2);  // Print a text or value

  tft.setCursor(70, 60);  // Set position (x,y)
  tft.println(rain_per);  // Print a text or value

  tft.setCursor(70, 75);  // Set position (x,y)
  tft.println(humi);  // Print a text or value


  tft.setCursor(70, 90);  // Set position (x,y)
  tft.println(temp);  // Print a text or value


  tft.setCursor(70, 105);  // Set position (x,y)
  tft.println(vib);  // Print a text or value


  tft.setCursor(70, 120);  // Set position (x,y)
  tft.println(disp);  // Print a text or value

  if(stat == 1){
  tft.fillRect(1,142,127,18, ST7735_RED);
  tft.setCursor(15, 148);  // Set position (x,y)
  tft.setTextColor(ST7735_WHITE);
  tft.println("Status : ");  // Print a text or value    
  tft.setTextColor(ST7735_WHITE, ST7735_RED); 
  }else {
    tft.setTextColor(ST7735_WHITE, ST7735_BLUE); 
  }
  
  tft.setCursor(70, 148);  // Set position (x,y)
  tft.println(status);  // Print a text or value
}

// Function for LoRa packet preparation and sending
static int LoRa_send()
{
  String sensorData = "";
  char cmd[256] = "";
  char data[128] = "";
  int ret = 0;
  sensorData = sensorData + String(m1) + "," + String(m2) + "," + String(rain_per) + "," + String(humi) 
                + "," + String(temp) + "," + String(disp) + "," + String(vib) + "," + String(stat);
  //Serial.print("Printing Sensor Data String : ");
  //Serial.println(sensorData);

  strncpy(data,sensorData.c_str(),sizeof(data));
  data[sizeof(data) -1] = 0;

  sprintf(cmd, "AT+TEST=TXLRSTR,\"GW,%s\"\r\n", data);
  //Serial.print("Printing cmd String : ");
  //Serial.print(cmd);

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

// Function to Setup the Initializations and Configurations
void setup() {
  // put your setup code here, to run once:
  //initialize the library
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("LandSlide Monitoring - Starting!!");
  delay(250);
  pinMode(vibSensor_pin, INPUT);
  configLoRaModule();
  setupDisplay();
  dht.begin();
  checkDHT();
  delay(1000);
  Serial.println("Setup Completed !!");
  delay(250);
}

// Function main Loop
void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentUpdateTime = millis();
  if (currentUpdateTime - previousUpdateTime >= updateInterval) {
    getReadings();
    checkStatus();
    displayReadings();
    previousUpdateTime = currentUpdateTime;
  }

  unsigned long currentTime = millis();
  if (currentTime - previousTime >= sendInterval) {
    LoRa_send();
    previousTime = currentTime;
  }
}
// ---------------------------------- make2explore.com----------------------------------------------------//