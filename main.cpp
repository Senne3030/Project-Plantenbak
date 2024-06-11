// Eindporject Plantenbak -- Made in Visual Studio Code -- Senne Van Dingenen 1IoT //

// On startup disconnect the GND on the relay module so it doesn't pump all the water out for no reason.

#include <Arduino.h> // Include the needed libraries
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <LiquidCrystal_I2C.h>
#include <FastLED.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BME280.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi and MQTT Server Configuration
const char *ssid = "telenet-A5C9B";          // Change to your WiFi SSID //embed
const char *password = "nuN695U3fJQ3";       // Change to your WiFi password //weareincontrol
const char *mqtt_server = "senne3030.local"; // Change to your MQTT broker address
const char *mqtt_username = "senne";         // Change to your MQTT username
const char *mqtt_password = "senne123";      // Change to your MQTT password
const int mqtt_port = 1883;

WiFiClient espClient;           // WiFi client
PubSubClient client(espClient); // MQTT client

#define BME_SDA 21
#define BME_SCL 22
Adafruit_BME280 bme;

#define OneWireBus 4
OneWire oneWire(OneWireBus);
DallasTemperature sensors(&oneWire);

#define LED_PIN 13
#define NUM_LEDS 100
CRGB leds[NUM_LEDS];

#define SS_PIN 5   // Use any digital pin for SS (Slave Select)
#define SCK_PIN 18 // Use the hardware SPI pins for SCK, MISO, MOSI
#define MOSI_PIN 23
#define MISO_PIN 19
Adafruit_PN532 nfc(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);

#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 20
#define LCD_ROWS 4
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

unsigned long previousMillis = 0;
const long WaitTime = 30000; // 30 seconds

const int LDR = 34;
const int MoistureSensor = 32;
const int WaterSensor = 33;
// const int RedButton = 35;
const int Buzzer = 2;
const int VentilatorK1 = 14;
const int WaterPumpK2 = 27;

float SoilMoisture = 0;
float SoilTemperatureC = 0;
float GlobalHumidity = 0;
float GlobalTemperature = 0;
int LCDSetting = 0;
int LASTLCDNUMBER = 5;
String WaterStatus = "";
String LedStripStatus = "";
String SunLightStatus = "";
String Access = "";
uint8_t CARD[4];
uint8_t UnknownCARD[4];
String cardString = "";
String Plant = "";
int VentilatorStatus = 0;
int WaterPumpStatus = 0;

int LedSetting = 0;
int VentilatorSetting = 0;
int WaterSetting = 0;

// RFID tags that are known by the system:
uint8_t authorizedUID1[] = {0x3, 0x45, 0xC5, 0x18};
uint8_t authorizedUID2[] = {0x3, 0xD4, 0xA, 0x19};
uint8_t authorizedUID3[] = {0xD3, 0xA8, 0xA5, 0x18};

// Function prototypes to make sure it knows those voids exist
void RFID_Reader();
void LCD_Print();
void LEDSTRIP();
void SoilMoistureSensor();
void SoilTemperature();
void WaterLevel();
void BME280Sensor();
void LCD_Setting();
void rfidReadTask(void *parameter);
void BuzzerSound1();
void BuzzerSound2();
void CheckTask(void *parameter);
void setup_wifi();
void reconnect();
void Ventilator();
void WaterPump();
void SendData();
void PlantSelect();

void setup_wifi() // Function to set up wifi
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  lcd.setCursor(0, 0);
  lcd.print("Connecting to       ");
  lcd.setCursor(0, 1);
  lcd.print(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  lcd.setCursor(0, 2);
  lcd.print("WiFi connected      ");
}

void reconnect() // Function to set up the MQTT connection
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password))
    {
      Serial.println("connected");
    }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print("Proces paused.      ");
      lcd.setCursor(0, 1);
      lcd.print("Attempting MQTT     ");
      lcd.setCursor(0, 2);
      lcd.print("connection...       ");
      lcd.setCursor(0, 3);
      lcd.print("Failed, State: " + String(client.state()) + "   ");

      VentilatorSetting = 0;
      WaterSetting = 0;

      WaterPump();
      Ventilator();

      Serial.print("failed, state: ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() // Function to set everything up
{
  Serial.begin(9600);
  Serial.println("Starting..");

  ///////////// LCD /////////////
  lcd.init();
  lcd.backlight();
  ///////////////////////////////

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  // pinMode(RedButton, INPUT_PULLUP);
  pinMode(Buzzer, OUTPUT);
  pinMode(VentilatorK1, OUTPUT);
  pinMode(WaterPumpK2, OUTPUT);

  //////// BME280 Sensor ////////
  Wire.begin(BME_SDA, BME_SCL);

  if (!bme.begin(0x76))
  {
    Serial.println("Could not find BME280 sensor!");
    lcd.setCursor(0, 0);
    lcd.print("Couldn't find BME280");
    while (1)
      ;
  }

  bme.setSampling(Adafruit_BME280::MODE_NORMAL,     // Operating mode
                  Adafruit_BME280::SAMPLING_X2,     // Temperature oversampling
                  Adafruit_BME280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BME280::SAMPLING_X16,    // Humidity oversampling
                  Adafruit_BME280::FILTER_X16,      // Filtering
                  Adafruit_BME280::STANDBY_MS_500); // Standby time
  ///////////////////////////////

  /////// Soil Temperature ///////
  sensors.begin();
  ///////////////////////////////

  ///////////// LEDS /////////////
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  ///////////////////////////////

  ///////////// RFID /////////////
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata)
  {
    Serial.println("Didn't find PN53x board");
    lcd.setCursor(0, 0);
    lcd.print("Couldn't find RFID  ");
    while (1)
      ;
  }

  Serial.print("Found chip PN5");
  Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware ver. ");
  Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.');
  Serial.println((versiondata >> 8) & 0xFF, DEC);

  nfc.SAMConfig();
  Serial.println("RFID card reader ready");
  ////////////////////////////////////////////////

  ///////// rfidReadTask /////////
  xTaskCreate(
      rfidReadTask,     // Task function
      "RFID Read Task", // Task name
      4096,             // Stack size (bytes)
      NULL,             // Task parameter
      2,                // Priority (0 is lowest)
      NULL              // Task handle
  );
  ///////////////////////////////

  ////////// CheckTask //////////
  xTaskCreate(
      CheckTask,    // Task function
      "Check Task", // Task name
      4096,         // Stack size (bytes)
      NULL,         // Task parameter
      1,            // Priority (0 is lowest)
      NULL          // Task handle
  );
  ///////////////////////////////

  Serial.println("Project Plantenbak");
}

void loop() // Main loop
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  Serial.println("------------------------");

  PlantSelect();

  LEDSTRIP();
  SoilMoistureSensor();
  SoilTemperature();
  WaterLevel();
  BME280Sensor();

  Ventilator();
  WaterPump();

  LCD_Print();
  SendData();

  delay(1000);
}

void RFID_Reader() // Function to read which RFID card was scanned
{
  uint8_t success;
  uint8_t uid[7] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
  uint8_t uidLength;                      // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  // Wait for an RFID card to be present
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success)
  {
    Serial.println("-----------------");
    Serial.println("Found an RFID card!");

    Serial.print("UID Length: ");
    Serial.print(uidLength, DEC);
    Serial.println(" bytes");
    Serial.print("UID Value: ");
    for (uint8_t i = 0; i < uidLength; i++)
    {
      Serial.print(" 0x");
      Serial.print(uid[i], HEX);
    }
    Serial.println("");

    if (memcmp(uid, authorizedUID1, uidLength) == 0)
    {
      BuzzerSound1();
      Access = "Access authorized";
      Plant = "Strawberries";
      memcpy(CARD, authorizedUID1, sizeof(authorizedUID1));
      Serial.println(Plant);
      Serial.println("Access authorized");
      Serial.println("-----------------");
    }
    else if (memcmp(uid, authorizedUID2, uidLength) == 0)
    {
      BuzzerSound1();
      Access = "Access authorized";
      Plant = "Lettuce";
      memcpy(CARD, authorizedUID2, sizeof(authorizedUID2));
      Serial.println(Plant);
      Serial.println("Access authorized");
      Serial.println("-----------------");
    }
    else if (memcmp(uid, authorizedUID3, uidLength) == 0)
    {
      BuzzerSound1();
      Access = "Access authorized";
      Plant = "Carrots";
      memcpy(CARD, authorizedUID3, sizeof(authorizedUID3));
      Serial.println(Plant);
      Serial.println("Access authorized");
      Serial.println("-----------------");
    }
    else
    {
      BuzzerSound2();
      memcpy(UnknownCARD, uid, sizeof(uid));
      memcpy(CARD, UnknownCARD, sizeof(UnknownCARD)); // I do this 2 times because when I do this one time Access gets corrupted for some reason

      Access = "Access denied";
      Plant = "Unknown in system";
      Serial.println(Plant);
      Serial.println("Access denied");
      Serial.println("-----------------");
    }

    cardString = "";

    for (size_t i = 0; i < sizeof(CARD); i++)
    {
      if (CARD[i] < 0x10)
      {
        cardString += "0";
      }
      cardString += String(CARD[i], HEX);
      if (i < sizeof(CARD) - 1)
      {
        cardString += " ";
      }
    }

    delay(1000);
    return;
  }
}

void LCD_Print() // Function to Print all values on the LCD
{
  Serial.println("Printing on LCD");
  lcd.clear();

  if (LCDSetting != 0)
  {
    lcd.setCursor(19, 0);
    lcd.print(LCDSetting);
  }

  if (LCDSetting == 1)
  {
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.setCursor(6, 0);
    lcd.print(GlobalTemperature);
    lcd.setCursor(12, 0);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("Humid: ");
    lcd.setCursor(7, 1);
    lcd.print(GlobalHumidity);
    lcd.setCursor(13, 1);
    lcd.print("%");

    lcd.setCursor(0, 2);
    lcd.print("Soil Temp: ");
    lcd.setCursor(11, 2);
    lcd.print(SoilTemperatureC);
    lcd.setCursor(17, 2);
    lcd.print("C");

    lcd.setCursor(0, 3);
    lcd.print("Soil Moist: ");
    lcd.setCursor(12, 3);
    lcd.print(SoilMoisture);
    lcd.setCursor(18, 3);
    lcd.print("%");
  }
  else if (LCDSetting == 2)
  {
    lcd.setCursor(0, 0);
    lcd.print("Water: ");
    lcd.setCursor(7, 0);
    lcd.print(WaterStatus);

    lcd.setCursor(0, 1);
    lcd.print("Pump: ");
    lcd.setCursor(6, 1);
    if (WaterPumpStatus == 1)
    {
      lcd.print("On");
    }
    else
    {
      lcd.print("Off");
    }

    lcd.setCursor(0, 2);
    lcd.print("Natural Light: ");
    lcd.setCursor(15, 2);
    lcd.print(SunLightStatus);

    lcd.setCursor(0, 3);
    lcd.print("Led strip: ");
    lcd.setCursor(11, 3);
    lcd.print(LedStripStatus);
  }
  else if (LCDSetting == 3)
  {
    lcd.setCursor(0, 0);
    lcd.print("Ventilator: ");
    lcd.setCursor(12, 0);
    if (VentilatorStatus == 1)
    {
      lcd.print("On");
    }
    else
    {
      lcd.print("Off");
    }

    lcd.setCursor(1, 3);
    lcd.print("Project Plantenbak");
  }
  else if (LCDSetting == 4)
  {
    lcd.setCursor(0, 0);
    lcd.print("Last RFID card: ");

    lcd.setCursor(0, 1);
    lcd.print(cardString);

    lcd.setCursor(0, 2);
    lcd.print(Plant);

    lcd.setCursor(0, 3);
    lcd.print(Access);
  }
  else if (LCDSetting == 0)
  {
    lcd.setCursor(0, 0);
    // lcd.print("Hold the red button");
    lcd.print("All systems working");
    lcd.setCursor(0, 1);
    lcd.print("Starting..");
    // lcd.print("to change the menu");
    lcd.setCursor(0, 3);
    lcd.print("Scan your RFID card");
  }
}

void LCD_Setting() // every 4 seconds change "page" (show other data)
{
  LCDSetting++;
  if (LCDSetting >= LASTLCDNUMBER)
  {
    LCDSetting = 1; // 0 of you wanna see the text in the begining and 1 if you want data
  }
  Serial.print("LCDSetting: ");
  Serial.println(LCDSetting);
}

void LEDSTRIP() // Function to turn ledstrip on and off
{
  int ValueNumberLDR = 0;

  int LDRvalue = analogRead(LDR);
  Serial.print("LDR value: ");
  Serial.println(LDRvalue);

  if (LedSetting == 0)
  {
    ValueNumberLDR = 0;
  }
  else if (LedSetting == 1)
  {
    ValueNumberLDR = 2500;
  }
  else if (LedSetting == 2)
  {
    ValueNumberLDR = 1000;
  }
  else if (LedSetting == 3)
  {
    ValueNumberLDR = 2000;
  }

  if (LDRvalue <= ValueNumberLDR) // leds go on whenever
  {
    fill_solid(leds, NUM_LEDS, CRGB(50, 0, 100)); // Purple
    FastLED.show();
    LedStripStatus = "On";

    Serial.print("LED strip: ");
    Serial.println(LedStripStatus);
  }
  else
  {
    fill_solid(leds, NUM_LEDS, CRGB(0, 0, 0)); // off
    FastLED.show();
    LedStripStatus = "Off";

    Serial.print("LED strip: ");
    Serial.println(LedStripStatus);
  }

  if (LDRvalue <= 2500)
  {
    SunLightStatus = "Some";

    if (LDRvalue <= 1000)
    {
      SunLightStatus = "None";
    }
  }
  else
  {
    SunLightStatus = "A lot";
  }
  Serial.print("SunLightStatus: ");
  Serial.println(SunLightStatus);
}

void SoilMoistureSensor() // Function to get soil moisture data
{

  float sensorValue = analogRead(MoistureSensor);
  float moisturePercentage = map(sensorValue, 0, 4095, 0, 100);

  Serial.print("Soil Moisture: ");
  Serial.print(moisturePercentage);
  Serial.println("%");
  SoilMoisture = moisturePercentage;
}

void SoilTemperature() // Function to get soil temperature data
{
  sensors.requestTemperatures();

  float temperatureCelsius = sensors.getTempCByIndex(0);

  if (temperatureCelsius != DEVICE_DISCONNECTED_C)
  {
    Serial.print("Soil Temperature: ");
    Serial.print(temperatureCelsius);
    Serial.println(" °C");
    SoilTemperatureC = temperatureCelsius;
  }
  else
  {
    SoilTemperatureC = -999;
    Serial.print("Soil Temperature: ");
    Serial.println("Error: ");
    Serial.println(SoilTemperatureC);
  }
}

void WaterLevel() // Function to get water container data
{
  int sensor = analogRead(WaterSensor);
  Serial.print("WaterSensor: ");
  Serial.println(sensor);

  if (sensor == 0 || sensor < 3500)
  {
    Serial.println("Water reservoir: EMPTY ");
    WaterStatus = "EMPTY";
  }
  else if (sensor >= 3500)
  {
    Serial.println("Water reservoir: Functional ");
    WaterStatus = "Good";
  }
}

void BME280Sensor() // Function to get temperature and humidity data
{
  float temperature = bme.readTemperature();
  // float pressure = bme.readPressure() / 100.0; // Convert pressure to hPa
  float humidity = bme.readHumidity();

  GlobalTemperature = temperature;
  GlobalHumidity = humidity;

  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");

  // Serial.print("Pressure = ");
  // Serial.print(pressure);
  // Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");
}

void BuzzerSound1() // Brief one time beeping sound
{
  tone(Buzzer, 1000);
  delay(250);
  noTone(Buzzer);
}

void BuzzerSound2() // 2 beeps, longer beeping sound
{
  tone(Buzzer, 600);
  delay(300);
  tone(Buzzer, 580);
  delay(350);
  noTone(Buzzer);
}

void rfidReadTask(void *parameter) // Function to use an apart loop to run RFID_reader
{
  while (1)
  {
    RFID_Reader();

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void CheckTask(void *parameter) // Function to use an apart loop to run LCD_Setting
{
  while (1)
  {
    if (!client.connected())
    {
    }
    else
    {
      delay(4000);

      LCD_Setting();

      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

void Ventilator() // Function to cool the plantbox whenever it's too hot
{
  int WantedTemperature;

  if (VentilatorSetting == 0)
  {
    WantedTemperature = 999;
  }
  else if (VentilatorSetting == 1)
  {
    WantedTemperature = 21;
  }
  else if (VentilatorSetting == 2)
  {
    WantedTemperature = 18;
  }
  else if (VentilatorSetting == 3)
  {
    WantedTemperature = 19;
  }

  if (GlobalTemperature < WantedTemperature)
  {
    digitalWrite(VentilatorK1, HIGH);
    VentilatorStatus = 0;
    Serial.println("K1 HIGH (NOT OPPERATIONAL)");
  }
  else
  {
    digitalWrite(VentilatorK1, LOW);
    VentilatorStatus = 1;
    Serial.println("K1 LOW (OPPERATIONAL)");
  }
}

void WaterPump() // Function to pump water into the plantbox whenever it needs water
{
  float Moisturenumber = 50.00;
  float MoisturenumberWithVentilator = 69.00;
  int TimeWatering;
  int activated;

  unsigned long currentMillis = millis();

  if (WaterSetting == 0)
  {
    TimeWatering = 0;
    activated = 0;
  }
  else if (WaterSetting == 1)
  {
    TimeWatering = 2000;
    activated = 1;
  }
  else if (WaterSetting == 2)
  {
    TimeWatering = 3000;
    activated = 1;
  }
  else if (WaterSetting == 3)
  {
    TimeWatering = 4000;
    activated = 1;
  }

  if (currentMillis - previousMillis >= WaitTime)
  {
    if (activated == 1)
    {
      if (VentilatorStatus == 1)
      {
        if (SoilMoisture <= MoisturenumberWithVentilator)
        {
          digitalWrite(WaterPumpK2, LOW);
          Serial.println("K2 LOW (OPPERATIONAL)");
          WaterPumpStatus = 1;
          SendData();
          delay(TimeWatering);
          SendData();
          digitalWrite(WaterPumpK2, HIGH);
          Serial.println("K2 HIGH (Timer concluded) " + String(TimeWatering) + " milliseconds");
          WaterPumpStatus = 0;
          previousMillis = currentMillis;
        }
        else
        {
          digitalWrite(WaterPumpK2, HIGH);
          WaterPumpStatus = 0;
          Serial.println("K2 HIGH (NOT OPPERATIONAL)");
        }
      }
      else
      {

        if (SoilMoisture <= Moisturenumber)
        {
          digitalWrite(WaterPumpK2, LOW);
          Serial.println("K2 LOW (OPPERATIONAL)");
          WaterPumpStatus = 1;
          SendData();
          delay(TimeWatering);
          SendData();
          digitalWrite(WaterPumpK2, HIGH);
          Serial.println("K2 HIGH (Timer concluded) " + String(TimeWatering) + " milliseconds");
          WaterPumpStatus = 0;
          previousMillis = currentMillis;
        }
        else
        {
          digitalWrite(WaterPumpK2, HIGH);
          WaterPumpStatus = 0;
          Serial.println("K2 HIGH (NOT OPPERATIONAL)");
        }
      }
    }
    else
    {
      digitalWrite(WaterPumpK2, HIGH);
      WaterPumpStatus = 0;
      Serial.println("K2 HIGH (NOT ACTIVATED)");
    }
  }
  else
  {
    digitalWrite(WaterPumpK2, HIGH);
    WaterPumpStatus = 0;
    Serial.println("K2 HIGH (" + String(WaitTime) + " millisecond timer running)");
  }
}

void SendData() // Function to send all data to raspberry pi via MQTT
{
  // change all to a string so it can be sent
  String Ledstrip;
  String watercontainer;
  String fan = String(VentilatorStatus);
  String waterPUMP = String(WaterPumpStatus);
  String GlobalTemperatureString = String(GlobalTemperature);
  String GlobalHumidityString = String(GlobalHumidity);
  String localSoilMoisture = String(SoilMoisture);

  if (VentilatorK1 == HIGH)
  {
    fan = "1";
  }
  else if (VentilatorK1 == LOW)
  {
    fan = "0";
  }

  if (LedStripStatus == "On")
  {
    Ledstrip = "1";
  }
  else if (LedStripStatus == "Off")
  {
    Ledstrip = "0";
  }

  if (WaterStatus == "Good")
  {
    watercontainer = "1";
  }
  else if (WaterStatus == "EMPTY")
  {
    watercontainer = "0";
  }

  if (SoilTemperatureC == -999)
  {
  }
  else
  {
    String SoilTemperature = String(SoilTemperatureC);
    client.publish("home/plantenbak/Soiltemperature", SoilTemperature.c_str());
  }

  // Publish sensor readings to MQTT topics
  client.publish("home/plantenbak/temperature", GlobalTemperatureString.c_str());
  client.publish("home/plantenbak/humidity", GlobalHumidityString.c_str());
  client.publish("home/plantenbak/soilmoisture", localSoilMoisture.c_str());
  client.publish("home/plantenbak/led", Ledstrip.c_str());
  client.publish("home/plantenbak/waterPump", waterPUMP.c_str());
  client.publish("home/plantenbak/watercontainer", watercontainer.c_str());
  client.publish("home/plantenbak/fan", fan.c_str());
}

void PlantSelect() // selects what plant you want to grow (with RFID)
{
  if (Plant == "Unknown in system" || Plant == "")
  {
    LedSetting = 0;
    VentilatorSetting = 0;
    WaterSetting = 0;
  }
  else if (Plant == "Strawberries")
  {
    LedSetting = 1;
    VentilatorSetting = 1;
    WaterSetting = 1;
  }
  else if (Plant == "Lettuce")
  {
    LedSetting = 2;
    VentilatorSetting = 2;
    WaterSetting = 2;
  }
  else if (Plant == "Carrots")
  {
    LedSetting = 3;
    VentilatorSetting = 3;
    WaterSetting = 3;
  }
  Serial.print("Selected: ");
  Serial.println(Plant);
}
