#include <BearSSLHelpers.h>
#include <CertStoreBearSSL.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiGratuitous.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiClientSecureAxTLS.h>
#include <WiFiClientSecureBearSSL.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiServerSecureAxTLS.h>
#include <WiFiServerSecureBearSSL.h>
#include <WiFiUdp.h>

#include <EEPROM.h>
#include <PubSubClient.h>

#include <Arduino.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial or Arduino package without SoftwareSerial support

#define RX_PIN D2                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN D1                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

MHZ19 myMHZ19;                                             // Constructor for library

SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial
//HardwareSerial mySerial(1);                              // (ESP32 Example) create device to MH-Z19 serial

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long getDataTimer = 0;

struct Settings
{
  char ssid[32];
  char password[32];
  char mqtt_server[16];

  char topic[64];
  char arg[16];

  char wifi_devname[16];
};

Settings settings = {"<Wifi Access point>", "<password>", "<MQTT server>", "wifi_sensor/CO2", "value", "CO2Sensor"};

void callback(char* topic, byte* payload, unsigned int length) 
{
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(settings.wifi_devname)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
    Serial.begin(9600);                                     // Device to serial monitor feedback

    EEPROM.begin(4096);
    Serial.print("NV size:");Serial.println((int)SPI_FLASH_SEC_SIZE);

    mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
    //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); // (ESP32 Example) device to MH-Z19 serial start   
    myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 

    myMHZ19.autoCalibration(false);                         // Turn auto calibration ON (OFF autoCalibration(false))

    WiFi.mode(WIFI_STA);
    WiFi.begin(settings.ssid, settings.password);
  
    client.setServer(settings.mqtt_server, 1883);
    client.setCallback(callback);

}

void loop()
{
    if (!client.connected()) {
      reconnect();
    }
    else if (millis() - getDataTimer >= 2000)
    {
        int CO2; 
        char outBuf[256];
        /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
        if below background CO2 levels or above range (useful to validate sensor). You can use the 
        usual documented command with getCO2(false) */

        CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
        
        Serial.print("CO2 (ppm): ");                      
        Serial.println(CO2);                                

        int8_t Temp;
        Temp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
        Serial.print("Temperature (C): ");                  
        Serial.println(Temp);                               

        sprintf(outBuf,"{\"CO2\":%d,\"tempreture\":%d}",CO2,Temp);
  
        Serial.println(outBuf);
        client.publish(settings.topic, outBuf);

        getDataTimer = millis();
    }
}
