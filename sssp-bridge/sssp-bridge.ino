
// Select deployment target
// #define ESP32
#define ESP8266

#ifdef ESP8266
#include <ESP8266WiFi.h>
// needed to set MAC address
extern "C" {
  #include <user_interface.h>
}
#else
#include <WiFi.h>
#endif

// #include <time.h>
#include <WiFiClient.h>
// #include <MQTT.h>
#include <PubSubClient.h>

WiFiClient net;
// MQTTClient client;
PubSubClient client(net);

#include "credentials.h"

#define CHAR_CR         "\x0D"
#define BAUD_SMARTFET   38400
#define PAYLOAD_SIZE    7

#define GAMMA_FACTOR    2

// #define OUTPUT_DBG
// #define OUTPUT_SER1 


void MQTT_connect();
void verifyFingerprint();
void update_flux(int pwr_level, int channel);

void print_mac(uint8_t* mac) {
  for(int i = 0; i < 5; i++) {
    Serial.print(mac[i], HEX);
    Serial.print(":");
  }
  Serial.println(mac[5], HEX);
}


void setup() {
  #ifdef LED_INDICATOR
  pinMode(LED_INDICATOR, OUTPUT);
  digitalWrite(LED_INDICATOR, HIGH);
  #endif
  
  #ifdef OUTPUT_SER1
  Serial.begin(115200);
  Serial1.begin(BAUD_SMARTFET);
  #else
  Serial.begin(BAUD_SMARTFET);
  #endif
  
  #ifdef ESP32
  esp_base_mac_addr_set(WLAN_MACADDR);
  #else 
  bool mac_changed = wifi_set_macaddr(STATION_IF, WLAN_MACADDR);
  #endif

  WiFi.mode(WIFI_STA);

  if (!mac_changed) {
    Serial.println("Failed to set MAC!! Rebooting");
    ESP.restart();
  }

  #ifdef OUTPUT_DBG
  Serial.print("Setting MAC address to:  ");
  print_mac(WLAN_MACADDR);
  Serial.print("\n\nConnecting to ");
  Serial.println(WLAN_SSID);
  #endif 

  // use PSK to connect, if supplied
  #ifdef WLAN_PASS
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  #else
  WiFi.begin(WLAN_SSID);
  #endif

  // wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef OUTPUT_DBG
    Serial.print(".");
    #endif
  }

  #ifdef OUTPUT_DBG
  Serial.println("\nWiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  #endif

  // verify_cert();

  client.setServer(AIO_SERVER, AIO_SERVERPORT);
  client.setCallback(parse_payload);
  MQTT_connect();

}


void parse_payload(const char* topic, byte* payload, unsigned int length) {
  if(length != PAYLOAD_SIZE) {
    Serial.println("unrecognized data");
    return;
  }
  char rgb_str[PAYLOAD_SIZE + 1];
  memcpy(rgb_str, payload, PAYLOAD_SIZE);
  rgb_str[PAYLOAD_SIZE] = '\0';

  uint32_t rgb_dec[3];
  for(int i = 0; i < 3; i++) {
    char channel[3];
    //1char offset for the leading '#'
    strncpy(channel, &rgb_str[1 + 2*i], 2);
    channel[2] = '\0';
    //parse str as base16 (hex)
    uint32_t chval = strtoul(channel, NULL, 16);
    float chval_norm = chval / 255.0;
    float chval_gammacorr = pow(chval_norm, GAMMA_FACTOR);

    uint32_t scaled_power = floor(chval_gammacorr * 100);
    rgb_dec[i] = scaled_power;
  }

  #ifdef OUTPUT_DBG
  // receipt string
  Serial.printf("R: %d   G: %d   B: %d\n", rgb_dec[0], rgb_dec[1], rgb_dec[2]);
  #endif

  // write color intensities to power channels, RGB order
  update_flux(rgb_dec[0], 1);
  update_flux(rgb_dec[1], 2);
  update_flux(rgb_dec[2], 3);
  

}

void loop() {

  client.loop();

  if (!client.connected()) {
    MQTT_connect();
  }

}

void update_flux(int pwr_level, int channel) {

  if(pwr_level > 100 || pwr_level < 0) {
    #ifdef OUTPUT_DBG
    Serial.println("pwr level out of range");
    #endif
    return;
  }

  #ifdef OUTPUT_SER1
  Serial1.print(CHAR_CR);
  Serial1.printf("sail%03d%drsch", pwr_level, channel);
  // Serial1.print(CHAR_CR);
  #else
  Serial.print(CHAR_CR);
  Serial.printf("sail%03d%drsch", pwr_level, channel);
  #endif

}

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect() {
  
  // Stop if already connected.
  if (client.connected()) {
    return;
  }

  #ifdef OUTPUT_DBG
  Serial.println("Connecting to MQTT");
  #endif

  #ifdef LED_INDICATOR
  digitalWrite(LED_INDICATOR, HIGH);
  #endif

  // Loop until we're reconnected
  while (!client.connected()) {
    #ifdef OUTPUT_DBG
    Serial.print("Attempting MQTT connection...");
    #endif

    // Attempt to connect
    if (client.connect(CLIENT_ID, AIO_USERNAME, AIO_KEY) ) {

      client.subscribe(TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 3 seconds");

      delay(3000);
    }
  }


  #ifdef OUTPUT_DBG
  Serial.println("MQTT Connected!");
  #endif

  #ifdef LED_INDICATOR
  digitalWrite(LED_INDICATOR, LOW);
  #endif
}
