#define DEBUG 1

#define OUT_IP "10.255.250.129" // Destination IP

#include <Arduino.h>
#include <ETH.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include "eth_properties.h"

// ETHERNET CONFIGURATION
IPAddress ip(10, 255, 250, 150);                         //the Arduino's IP
IPAddress subnet(255, 255, 254, 0);                     //subnet mask
IPAddress gateway(10, 255, 250, 1);                     //gateway IP
IPAddress outIp(10, 255, 250, 129);                     //destination IP
const unsigned int inPort = 7001;                       //Arduino's Port
const unsigned int outPort = 7000;                      //destination Port

WiFiUDP Udp;

void sendUDP(const char* message, const char* remoteIP, uint16_t remotePort) {
  IPAddress ip;
  if (!ip.fromString(remoteIP)) { Serial.println("❌ Invalid IP address"); return; }
  if (Udp.beginPacket(ip, remotePort) == 0) { Serial.println("❌ Failed to start UDP packet"); return;}
  Udp.write(reinterpret_cast<const uint8_t*>(message), strlen(message));
  if (Udp.endPacket() == 0) { Serial.println("❌ UDP send failed!"); } 
  else                      { Serial.printf("📤 Sent UDP to %s:%d -> %s\n", remoteIP, remotePort, message);}
}

void sendUDP_(String message, const char* remoteIP, uint16_t remotePort) {
  sendUDP(message.c_str(), remoteIP, remotePort);
}


void oscSend(const char* address, const char* type, uint8_t column) {
  char fullAddress[50];
  snprintf(fullAddress, sizeof(fullAddress), "%s%d/connect", address, column);
  OSCMessage msg(fullAddress);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void receiveUdp() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];  // temp buffer
    int len = Udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) { incomingPacket[len] = 0;}
    String receivedData = String(incomingPacket);
    Serial.printf("📥 Received UDP: %s\n", receivedData.c_str());
  }
}

void readSerial(){
  if (Serial.available()>0){
    char incoming = Serial.read();
    if (incoming == 'i') {Serial.print("ETH IP: "); Serial.println(ETH.localIP());}
    if (incoming == 'm') {Serial.print("ETH MAC: "); Serial.println(ETH.macAddress());}
    if (incoming == 'r') { sendUDP_("Hello" , OUT_IP, outPort); }
    uint8_t column = int(incoming) - 48; // Convert char to int (ASCII to int)
    if (column < 0 || column > 9) {return;} // Ignore invalid columns
    oscSend("/composition/columns/", "i", column);
    if (DEBUG) Serial.printf("✅ Connect Column: %d\n", column);
    if (DEBUG) Serial.printf("📡 Sending OSC: /composition/columns/%d/connect\n", column);
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
  ETH.begin( ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE_0);
  ETH.config(ip, gateway, subnet);
  WiFi.onEvent(WiFiEvent);
  Udp.begin(inPort);
}

void loop(){
  readSerial();
  receiveUdp();
}