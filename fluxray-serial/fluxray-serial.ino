#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

extern "C" {
  #include <user_interface.h>
}


/************************* WiFi Access Point *********************************/
// #define WLAN_SSID
// #define WLAN_PASS 
// const uint8_t WLAN_MACADDR[6];

/************************* Adafruit.io Setup *********************************/
// #define AIO_SERVER    
// #define AIO_SERVERPORT
// #define AIO_USERNAME  
// #define AIO_KEY    

// private details suppressed - define in credentials.h
#include "credentials.h"

#define CHAR_CR         "\x0D"
#define BAUD_SMARTFET   38400

WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
const char* fingerprint = "26 96 1C 2A 51 07 FD 15 80 96 93 AE F7 32 CE B9 0D 01 55 C4";
Adafruit_MQTT_Subscribe rgb_feed = 
  Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/srad-nm-fluxray108");




void MQTT_connect();
void verifyFingerprint();

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
  #endif


}

void setup() {
  pinMode(LED_INDICATOR, OUTPUT);
  digitalWrite(LED_INDICATOR, HIGH);
  
  #ifdef OUTPUT_SER1
  Serial.begin(115200);
  Serial1.begin(BAUD_SMARTFET);
  #else
  Serial.begin(115200);
  #endif
  
  //(baud, protocol, rx, tx)
  // serout.begin(38400, SERIAL_8N1, 13, 11);
  

  Serial.print("Setting MAC address to:  ");
  wifi_set_macaddr(STATION_IF, WLAN_MACADDR);

  Serial.print("\n\nConnecting to ");
  Serial.println(WLAN_SSID);
  WiFi.mode(WIFI_STA);

  #ifdef WLAN_PASS
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  #else
  WiFi.begin(WLAN_SSID);
  #endif

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  verifyFingerprint();
  mqtt.subscribe(&rgb_feed);

}

void loop() {
  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &rgb_feed) {
      Serial.print(F("Got: "));
      char* rgb_str = (char *)rgb_feed.lastread;
      Serial.println(rgb_str);
      uint32_t rgb_dec[3];
      
      for(int i = 0; i < 3; i++) {
        char channel[3];
        //1char offset for the leading '#'
        strncpy(channel, &rgb_str[1 + 2*i], 2);
        channel[2] = '\0';
        //parse str as base16 (hex)
        uint32_t val = strtoul(channel, NULL, 16);
        uint32_t scaled_power = (val * 100) / 256;
        rgb_dec[i] = scaled_power;
      }
  
      Serial.printf("WW: %d   NW: %d   CW: %d\n", rgb_dec[0], rgb_dec[1], rgb_dec[2]);

      // write HSF control to CH3
      uint32_t total_power = rgb_dec[0] + rgb_dec[2];
      bool needs_active_cooling = (total_power > 30);
      update_flux((needs_active_cooling ? 100 : 0), 3);

      // write color intensities to power channels
      update_flux(rgb_dec[0], 1);
      update_flux(rgb_dec[2], 2);

    }
  }
}


void verifyFingerprint() {

  const char* host = AIO_SERVER;

  Serial.print("Connecting to ");
  Serial.println(host);

  if (! client.connect(host, AIO_SERVERPORT)) {
    Serial.println("Connection failed. Halting execution.");
    while (1);
  }

  // if (client.verify(fingerprint, host)) {
  //   Serial.println("Connection secure.");
  // } else {
  //   Serial.println("Connection insecure! Halting execution.");
  //   while (1);
  // }

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");
  digitalWrite(LED_INDICATOR, HIGH);

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 1 seconds...");
    mqtt.disconnect();
    delay(1000);
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }

  Serial.println("MQTT Connected!");
  digitalWrite(LED_INDICATOR, LOW);
}
