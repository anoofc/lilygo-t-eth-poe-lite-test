#define DEBUG 1

#include <Arduino.h>
#include <ETH.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include "eth_properties.h"

// ETHERNET CONFIGURATION
IPAddress ip(10, 255, 250, 98);                         //the Arduino's IP
IPAddress subnet(255, 255, 254, 0);                     //subnet mask
IPAddress gateway(10, 255, 250, 1);                     //gateway IP
IPAddress outIp(10, 255, 250, 129);                     //destination IP
const unsigned int inPort = 7001;                       //Arduino's Port
const unsigned int outPort = 7000;                      //destination Port
byte mac[] = {0x90, 0xA2, 0xDA, 0x0A, 0x2B, 0X1E};      //Arduino's MAC

WiFiUDP Udp;

void oscSend(const char* address, const char* type, uint8_t column) {
  char fullAddress[50];
  snprintf(fullAddress, sizeof(fullAddress), "%s%d/connect", address, column);
  OSCMessage msg(fullAddress);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void readSerial(){
  if (Serial.available()>0){
    char incoming = Serial.read();
    uint8_t column = int(incoming) - 48; // Convert char to int (ASCII to int)
    // if (column < 0 || column > 9) return; // Ignore invalid input
    oscSend("/composition/columns/", "i", column);
    if (DEBUG) Serial.println("Connect: " + String(column));
    Serial.print("ETH IP: ");
    Serial.println(ETH.localIP());
  }
}
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("esp32-ethernet");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      // Set static IP after connection
      ETH.config(ip, gateway, subnet);
      if (!ETH.config(ip, gateway, subnet)) {
        Serial.println("Failed to configure static IP");
      } else {
        Serial.println("Static IP configured");
      }
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH IP: ");
      Serial.println(ETH.localIP());
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      break;
    default:
      break;
  }
}
void setup(){
  Serial.begin(115200);
  ETH.begin( ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE_0 );
  WiFi.onEvent(WiFiEvent);
  Udp.begin(inPort);
}

void loop(){
  readSerial();
}