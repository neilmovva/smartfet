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

#include <WiFiClientSecure.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"




/************************* WiFi Access Point *********************************/
// #define WLAN_SSID
// #define WLAN_PASS 
// const uint8_t WLAN_MACADDR[6];

/************************* Adafruit.io Setup *********************************/
// #define AIO_SERVER    
// #define AIO_SERVERPORT
// #define AIO_USERNAME  
// #define AIO_KEY    
// #define AIO_SSL_FINGERPRINT

// private details suppressed - define the above in credentials.h
#include "credentials.h"

#define CHAR_CR         "\x0D"
#define BAUD_SMARTFET   38400

#define GAMMA_FACTOR    1.5

WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe rgb_feed = 
  Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/srad-nm-fluxray108");


#define OUTPUT_DBG
// #define OUTPUT_SER1 


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
  #else
  Serial.print(CHAR_CR);
  Serial.printf("sail%03d%drsch", pwr_level, channel);
  #endif


}

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
  wifi_set_macaddr(STATION_IF, WLAN_MACADDR);
  #endif

  WiFi.mode(WIFI_STA);


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
    delay(250);
    #ifdef OUTPUT_DBG
    Serial.print(".");
    #endif
  }

  #ifdef OUTPUT_DBG
  Serial.println("\nWiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  #endif

  verifyFingerprint();

  #ifdef OUTPUT_DBG
  Serial.println("Subscribing to feed...");
  #endif

  mqtt.subscribe(&rgb_feed);

}

void loop() {
  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &rgb_feed) {

      char* rgb_str = (char *)rgb_feed.lastread;

      #ifdef OUTPUT_DBG
      Serial.print(F("Got: "));
      Serial.println(rgb_str);
      #endif

      // parse command string into numeric power levels
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
      Serial.printf("WW: %d   NW: %d   CW: %d\n", rgb_dec[0], rgb_dec[1], rgb_dec[2]);
      #endif

      // write HSF control to CH3
      uint32_t total_power = rgb_dec[0] + rgb_dec[2];
      bool needs_active_cooling = (total_power > 30);
      // update_flux((needs_active_cooling ? 100 : 0), 3);

      // write color intensities to power channels
      update_flux(rgb_dec[0], 1);
      update_flux(rgb_dec[2], 2);

    }
  }
}


void verifyFingerprint() {

  const char* host = AIO_SERVER;

  #ifdef OUTPUT_DBG
  Serial.print("Connecting to ");
  Serial.println(host);
  #endif

  if (! client.connect(host, AIO_SERVERPORT)) {
    Serial.println("Connection failed. Restarting.");
    ESP.restart();
  }

  #ifdef OUTPUT_DBG
  Serial.println("Connected!");
  #endif

  if (client.verify(AIO_SSL_FINGERPRINT, host)) {
    #ifdef OUTPUT_DBG
    Serial.println("Connection secure.");
    #endif
  } else {
    Serial.println("Connection insecure! Attempting restart.");
    ESP.restart();
  }

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  #ifdef OUTPUT_DBG
  Serial.print("Connecting to MQTT... ");
  #endif

  #ifdef LED_INDICATOR
  digitalWrite(LED_INDICATOR, HIGH);
  #endif

  uint8_t retries = 3;
  int8_t retval;
  // retval 0 signfies success
  while ((retval = mqtt.connect()) != 0) { 

    #ifdef OUTPUT_DBG
    Serial.println(mqtt.connectErrorString(retval));
    Serial.println("Retrying MQTT...");
    #endif

    mqtt.disconnect();
    delay(1000);
    retries--;
    if (retries == 0) {
      // restart ESP (via SDK, "clean" reset)
      ESP.restart();
    }
  }

  #ifdef OUTPUT_DBG
  Serial.println("MQTT Connected!");
  #endif

  #ifdef LED_INDICATOR
  digitalWrite(LED_INDICATOR, LOW);
  #endif
}
