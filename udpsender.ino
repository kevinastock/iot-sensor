#include <Wire.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_SGP30.h>

char ssid[] = "vanilla";        // your network SSID (name)
char pass[] = "columbus";    // your network password (use for WPA, or use as key for WEP)

IPAddress server(192,168,0,200); // vesper
unsigned int serverPort = 9942;
unsigned int localPort = 2072;

const int PACKET_SIZE = 44;
byte packetBuffer[PACKET_SIZE];

WiFiUDP Udp;

Adafruit_BMP280 bme; // I2C
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // Arg is unused... this library is awful
Adafruit_SGP30 sgp;


void blink_forever() {
  while(true) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);                       // wait for a second
  }
}

void setup() {
  WiFi.setPins(8,7,4,2);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  //Initialize serial and wait for port to open:
  // Serial.begin(9600);
  
  if (!bme.begin()) {  
    //Serial.println("Couldn't find bmp280 sensor!");
    blink_forever();
  }

  if (!htu.begin()) {
    //Serial.println("Couldn't find htu21df sensor!");
    blink_forever();
  }

  if (!tsl.begin()) {
    //Serial.println("Couldn't find tsl2591 sensor!");
    blink_forever();
  }
  tsl.setGain(TSL2591_GAIN_LOW);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  if (!sgp.begin()){
    //Serial.println("Couldn't find sgp30 sensor!");
    blink_forever();
  }
  
  sgp.setIAQBaseline(0x87D8, 0x8C94);
  
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    //Serial.println("WiFi shield not present");
    // don't continue:
    blink_forever();
  }

  connectWifi();
  //printWiFiStatus();

  Udp.begin(localPort);
}

uint16_t tsl_read16(uint8_t reg)
{
  uint16_t x;
  uint16_t t;

  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(TSL2591_ADDR, 2);
  t = Wire.read();
  x = Wire.read();

  x <<= 8;
  x |= t;
  return x;
}

uint16_t tsl_full() { // WARN: Use this before tsl_ir
  return tsl_read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW);
}

uint16_t tsl_ir() {
  return tsl_read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW);
}

float gains[] = {1, 25, 428, 9876};

float tsl_max_lux(uint16_t full, int gain, int timing) {
  float cp1 = (gains[gain] * (timing+1.0F) * 100.0F) / TSL2591_LUX_DF;
  return ((float)full) / cp1;
}

void tsl_calibrate(uint16_t last_full) {
  float last_max_lux = tsl_max_lux(last_full, tsl.getGain()/16, tsl.getTiming());
  int last_gain = 0;
  int last_timing = 0;
  for (int gain = 0; gain < 4; gain++) {
    float ag = gains[gain];
    for (int timing = 0; timing < 6; timing++) {
      if (tsl_max_lux(65535, gain, timing) / 2.0F <= last_max_lux) {
        if (last_gain*16 != tsl.getGain() || last_timing != tsl.getTiming()) {
          tsl.setGain((tsl2591Gain_t)(last_gain*16));
          tsl.setTiming((tsl2591IntegrationTime_t)last_timing);
        }
        return;
      }
      last_gain = gain;
      last_timing = timing;
    }
  }

  tsl.setGain(TSL2591_GAIN_MAX);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);
}

/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

void updateSample(int pin, uint16_t* minimum, uint16_t* maximum) {
  unsigned int sample = analogRead(pin);
  *minimum = min(sample, *minimum);
  *maximum = max(sample, *maximum);
}

void connectWifi() {
  while (WiFi.status() != WL_CONNECTED) {
    //Serial.print("Attempting to connect to SSID: ");
    //Serial.println(ssid);
    WiFi.begin(ssid, pass);

    unsigned long starttime = millis();
    do {
      delay(2000);
    } while (WiFi.status() != WL_CONNECTED && millis() - starttime < 11000);
  }
}

unsigned long last_loop = 0;

void loop() {
  unsigned long loop_start = millis();

  tsl.enable();

  unsigned long lux_start = millis();
  
  uint16_t micMax = 0;
  uint16_t micMin = 1024;
  
  uint16_t xMax = 0;
  uint16_t xMin = 1024;
  uint16_t yMax = 0;
  uint16_t yMin = 1024;
  uint16_t zMax = 0;
  uint16_t zMin = 1024;

  float pressure = bme.readPressure();
  float temp = htu.readTemperature();
  float humidity = htu.readHumidity();

  sgp.setHumidity(getAbsoluteHumidity(temp, humidity));

  sgp.IAQmeasure();
  uint16_t tvoc = sgp.TVOC;
  uint16_t eco2 = sgp.eCO2;

  // Wait for lux sensor to finish integration
  while (millis() - lux_start < 900) {
    updateSample(A0, &micMin, &micMax);
    updateSample(A3, &xMin, &xMax);
    updateSample(A4, &yMin, &yMax);
    updateSample(A5, &zMin, &zMax);
  }

  uint16_t full = tsl_full();
  uint16_t ir = tsl_ir();
  tsl.disable();
  uint16_t gain = tsl.getGain() / 16;
  uint16_t timing = tsl.getTiming();
  float lux = tsl.calculateLux(full, ir);

  tsl_calibrate(full);

  memset(packetBuffer, 0, PACKET_SIZE);
  *(float*)(packetBuffer+0) = lux;
  *(float*)(packetBuffer+4) = pressure;
  *(float*)(packetBuffer+8) = temp;
  *(float*)(packetBuffer+12) = humidity;
  *(uint16_t*)(packetBuffer+16) = tvoc;
  *(uint16_t*)(packetBuffer+18) = eco2;
  *(uint16_t*)(packetBuffer+20) = full;
  *(uint16_t*)(packetBuffer+22) = ir;
  *(uint16_t*)(packetBuffer+24) = gain;
  *(uint16_t*)(packetBuffer+26) = timing;
  *(uint16_t*)(packetBuffer+28) = micMin;
  *(uint16_t*)(packetBuffer+30) = micMax;
  *(uint16_t*)(packetBuffer+32) = xMin;
  *(uint16_t*)(packetBuffer+34) = xMax;
  *(uint16_t*)(packetBuffer+36) = yMin;
  *(uint16_t*)(packetBuffer+38) = yMax;
  *(uint16_t*)(packetBuffer+40) = zMin;
  *(uint16_t*)(packetBuffer+42) = zMax;

  connectWifi();
  sendPacket();

  // Only send once per second
  unsigned long remaining = 1000 - (millis() - loop_start);
  if (remaining > 0) {
    //Serial.println(remaining);
    delay(remaining);
  }
  
  // bme.readTemperature(); // float
  // bme.readPressure(); // float
  // htu.readTemperature(); // float
  // htu.readHumidity(); // float

  // If you have a temperature / humidity sensor, you can set the absolute humidity to enable the humditiy compensation for the air quality signals
  //float temperature = 22.1; // [°C]
  //float humidity = 45.2; // [%RH]
  //sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));
  
  // sgp.IAQmeasure() // null
  // sgp.TVOC // uint16_t ppb
  // sgp.eCO2 // uint16_t ppm

/*  

  delay(700);
  uint16_t full = tsl_full();
  uint16_t ir = tsl_ir();
  tsl.disable();
  float lux = tsl.calculateLux(full, ir);

  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("FullI: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("IrI: ")); Serial.print(ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.print(lux, 6); Serial.print(F("  "));
  Serial.print(F("Gain: ")); Serial.print(tsl.getGain()); Serial.print(F("  "));
  Serial.print(F("Timing: ")); Serial.println(tsl.getTiming());

  tsl_calibrate(full);
  */


   
}


unsigned long sendPacket()
{
  Udp.beginPacket(server, serverPort);
  Udp.write(packetBuffer, PACKET_SIZE);
  Udp.endPacket();
}


/*
void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
*/
