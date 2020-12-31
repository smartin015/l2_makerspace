#include "ros_hal.h"
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include "config_generated.h"

#define CFG_PATH "/secret.bin"
#define FORMAT_SPIFFS_IF_FAILED false

#define BUFSZ 1024
char cfg_buf[BUFSZ];
const Config *cfg;

using namespace websockets;
void onMessageCallback(WebsocketsMessage message) {
    Serial.print("Got Message: ");
    Serial.println(message.data());
}
WebsocketsClient client;

bool confOK = false;
char out[BUFSZ];
int wifiStatus = -1;


void onEventsCallback(WebsocketsEvent event, String data) {
  if(event == WebsocketsEvent::ConnectionOpened) {
    sprintf(out, "ws open");
  } else if(event == WebsocketsEvent::ConnectionClosed) {
    sprintf(out, "ws close");
  }
}

void ros_hal::init() {
  // TODO load wifi config from SPIFFS
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    sprintf(out, "ERR SPIFFS");
    return;
  }
  if (!SPIFFS.exists(CFG_PATH)) {
    sprintf(out, "ERR NOFILE");
    return;
  }
  File file = SPIFFS.open(CFG_PATH);
  if (!file) {
    sprintf(out, "ERR FOPEN");
    return;
  }
  int nb = file.readBytes(cfg_buf, BUFSZ);
  file.close();
  if (nb <= 0) {
    sprintf(out, "ERR CFGREAD");
    return;
  }

  cfg = GetConfig(cfg_buf);
  if (!cfg) {
    sprintf(out, "ERR CFGPARSE");
    return;
  }
  if (!cfg->ssid()) {
    sprintf(out, "ERR NOSSID (%d)", nb);
    return;
  }
  if (!cfg->pass()) {
    sprintf(out, "ERR NOPASS");
    return;
  }
  confOK = true;
  WiFi.begin(cfg->ssid()->c_str(), cfg->pass()->c_str());
  sprintf(out, "Config OK, wifi %d", WiFi.status());
  client.onMessage(onMessageCallback);
  client.onEvent(onEventsCallback);
}

const char* ros_hal::get_status() {
  return out;
}

const State* ros_hal::get_state() {
  if (WiFi.status() != WL_CONNECTED) {
    return nullptr;
  }
  return nullptr; // TODO
}

void ros_hal::spin() { 
  if (!confOK) {
    return;
  }
  if (WiFi.status() != wifiStatus) {
    wifiStatus = WiFi.status();
    sprintf(out, "WiFi code %d", wifiStatus);

    if (wifiStatus == WL_CONNECTED) {
		  client.connect(cfg->ws_ip()->c_str(), cfg->ws_port(), "/");
      sprintf(out, "Wifi OK, connecting WS");
    }
  }
  client.poll();
}
