/*
  Example sketch for VEML6070-Breakout (Digital UV Light Sensor).
  Rset=270k on breakout, UVA sensitivity: 5.625 uW/cm²/step
  Board           I2C/TWI Pins
                  SDA, SCL
  ----------------------------
  Uno, Ethernet    A4, A5
  Mega             20, 21
  Leonardo          2,  3
  Due              20, 21
  Integration Times and UVA Sensitivity:
    Rset=240k -> 1T=100.0ms ->   5.000 uW/cm²/step
    Rset=270k -> 1T=112.5ms ->   5.625 uW/cm²/step
    Rset=300k -> 1T=125.0ms ->   6.250 uW/cm²/step
    Rset=600k -> 1T=250.0ms ->  12.500 uW/cm²/step
*/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Wire.h>

/* Place your settings in the settings.h file for ssid, password, and dweet post string
FILE: settings.h
// WiFi settings
const char* ssid = "MYSSID";
const char* password = "MyPassword";
const char* dweet = "/dweet/for/myservice?uv="
*/
#include "settings.h"

// Host
const char* host = "dweet.io";

#define I2C_ADDR 0x38

//Integration Time
#define IT_1_2 0x0 //1/2T
#define IT_1   0x1 //1T
#define IT_2   0x2 //2T
#define IT_4   0x3 //4T

#define VEML6070_ADDR_ARA (0x18 >> 1)
#define VEML6070_ADDR_CMD (0x70 >> 1)
#define VEML6070_ADDR_DATA_LSB (0x71 >> 1)
#define VEML6070_ADDR_DATA_MSB (0x73 >> 1)
// VEML6070 command register bits
#define VEML6070_CMD_SD 0x01
#define VEML6070_CMD_WDM 0x02
#define VEML6070_CMD_IT_0_5T 0x00
#define VEML6070_CMD_IT_1T 0x04
#define VEML6070_CMD_IT_2T 0x08
#define VEML6070_CMD_IT_4T 0x0C
#define VEML6070_CMD_DEFAULT (VEML6070_CMD_WDM | VEML6070_CMD_IT_4T)

byte cmd = VEML6070_CMD_DEFAULT;
enum RISK_LEVEL {low, moderate, high, very_high, extreme};

// forwards
void connectToSerial(void);
void connectToWifi(void);
bool initialize_VEML(void);
uint16_t read_uvs_step(void);
RISK_LEVEL convert_to_risk_level(uint16_t uvs_step);
const char* risk_level_str(RISK_LEVEL level);
void sendToDweet(const char* uvlevel);



void setup()
{
  connectToSerial();
  connectToWifi();

   if(initialize_VEML() != true) {
     Serial.println("Unable to initialize");

   }

   // get led ready
   pinMode(2, OUTPUT);
}

void connectToSerial() {
  Serial.begin(9600);
  while(!Serial); //wait for serial port to connect (needed for Leonardo only)
}

void connectToWifi() {
  // Connect to WiFi
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
   }
   Serial.println("");
   Serial.println("WiFi connected");

   // Print the IP address
   Serial.println(WiFi.localIP());
}


bool initialize_VEML() {
    byte err = 0;

    // connect to wire as master
    Wire.setClock(100000);
    Wire.begin(4, 5);

    // read ARA to clear interrupt
    Wire.requestFrom(VEML6070_ADDR_ARA, 1);
    if(Wire.available()) {
      byte data = Wire.read();

      // Initialize command register
      Wire.beginTransmission(VEML6070_ADDR_CMD);
      Wire.write(cmd);
      err = Wire.endTransmission();
      delay(200);
      if(err == 0) {
        Serial.println("Initialized successfully.");
        return true;
      }

    }
    return false;
}

void loop()
{
  uint16_t uvs_step;
  RISK_LEVEL risk_level;
  char level[256];

  uvs_step = read_uvs_step();
  risk_level = convert_to_risk_level(uvs_step);

  Serial.printf("UV Level at: %s, raw value: %d\n", risk_level_str(risk_level), uvs_step);
  snprintf(level, sizeof(level)/sizeof(char), "%d:%s", uvs_step, risk_level_str(risk_level));
  sendToDweet(level);
  delay(1000);
}

uint16_t read_uvs_step(void)
{
  byte lsb, msb;
  uint16_t data = 0;

  Wire.requestFrom(VEML6070_ADDR_DATA_MSB, 1);
  if(Wire.available()) {
    msb = Wire.read();

    Wire.requestFrom(VEML6070_ADDR_DATA_LSB, 1);
    if(Wire.available()) {
      lsb = Wire.read();
      data = ((uint16_t)msb << 8) | (uint16_t)lsb;
      Serial.println("Got good value");
      // flash led
      digitalWrite(2, LOW);
      delay(250);
      digitalWrite(2, HIGH);
      return data;
    }
  }
}

RISK_LEVEL convert_to_risk_level(uint16_t uvs_step) {
  uint16_t risk_level_mapping_table[4] = {2241, 4482, 5976, 8217};
  uint16_t i;
  for (i = 0; i < 4; i++) {
    if (uvs_step <= risk_level_mapping_table[i]) {
      break;
    }
  }
  return (RISK_LEVEL)i;
}


void sendToDweet(const char* uvlevel) {
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    delay(1000);
    return;
  }

  // This will send the request to the server
  client.print(String("POST ") + dweet + uvlevel + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
  delay(10);

  // Read all the lines of the reply from server and print them to Serial
  while(client.available()){
    String line = client.readStringUntil('\r');
  }
}

const char* risk_level_str(RISK_LEVEL level) {
  switch(level) {
      case low :
        return "low";
      case moderate :
        return "moderate";
      case high :
        return "high";
      case very_high:
        return "very high";
      case extreme:
        return "extreme";
      default:
        return "UNKNOWN";
  }
}
