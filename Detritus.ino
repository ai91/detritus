/*
 * 2-channel switch firmware for WiFi Curtain/Blinds module based on ESP8285:  
 * https://s.click.aliexpress.com/e/_dXr335D
 * 
 * For more information on wiring, flashing, etc. see https://ibn.by/2020/02/23/detritus-firmware/
 * 
 * Works via MQTT over WiFi, as well as with directly attached to wall switch.
 * 
 * Working modes:
 *   * LED blinks slow (once a second) - module in setup mode. At this moment open WiFi hotspot is enabled.
 *                                       Once enabled, one can connect 'Detritus-xxxxx' WiFi network and open
 *                                       configuration page on 192.168.4.1 address.
 *   * LED blinks fast (ten times a second) - module in operating offline mode. Happens if module was misconfigured
 *                                       (or not configured at all), and setup mode closed after timeout (3 minutes).
 *                                       In this mode only manual wall switch works. Once an hour module reboots and
 *                                       waits another 3 minutes for setup.
 *   * LED is on                       - module in fully operating mode. 
 *
 * Operation controls:
 *   * Enter setup mode                - quickly press/release some switch 5 times
 *
 * Configuration (setup mode):
 *   Following parameters can be found under 'Configure WiFi' menu item:
 *    * SSID/password - connection to WiFi
 *    * mqtt server      - MQTT server address (or host name)
 *    * mqtt port        - MQTT port
 *    * mqtt client name - just quess :-)
 *    * mqtt user
 *    * mqtt password
 *    * mqtt output topic 1        - topic for output of relay 1 current state. Value is 0/1 for off/on states. 
 *                                   Additionally contains a manual switch state by providing a dot in the end.
 *                                   Examples:
 *                                    '0.' - relay is off, appropriate switch is pressed
 *                                    '1' - relay is on, appropriate switch released
 *                                    '1.' - relay is on, appropriate switch pressed.
 *    * mqtt commands 1 topic       - topic for relay 1 commands input.
 *    * mqtt output topic 2         - topic for output of relay 2 current state. Value is 0/1 for off/on states. 
 *    * mqtt commands 2 topic       - topic for relay 2 commands input.
 *    * swap relays                 - when checked, swaps manual switches for relays (s1 controls l2, s2 controls l1).
 *    * respect switch states       - when disabled - each manual switch state change triggers relay state change. 
 *                                    when enabled - pressing manual switch, turns on relay. If relay already was on, 
 *                                    then does nothing. 
 *    * invert switch keys          - inverts manual switch logic (also makes effect on mqtt output of '.' symbol). 
 * 
 * Supports following commands over MQTT /cmd topics:
 *  * 1    - turn on appropriate relay (closes)
 *  * 0    - turn off appropriate relay (opens)
 *  * set  - enter setup mode. Analogue to pressing 'Pairing Button' on module. Can be sent to any command-topic.
 *              Example:
 *                'set' - open WiFi hotspot 'Detritus-xxxxx' will be enabled for 3 minutes. After 3 minutes will exit setup mode.
 *  * ota  - enter OTA mode.
 * 
 * Author: Anar Ibragimoff (anar@ibn.by)
 * 
 */
#include <LittleFS.h>

#include <ESP8266WiFi.h>        // https://github.com/esp8266/Arduino
#include <DNSServer.h>          // Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>   // Local WebServer used to serve the configuration portal
#include <WiFiManager.h>        // https://github.com/tzapu/WiFiManager/tree/development
#include <ArduinoOTA.h>

#include <Ticker.h>

#include <WiFiUdp.h>
#include <mDNSResolver.h>       // https://github.com/madpilot/mDNSResolver

#include <ArduinoJson.h>        // https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>       // https://github.com/knolleary/pubsubclient

// GPIOs
#define LED_BUILTIN 3
#define SWITCH_S1 4
#define SWITCH_S2 5
#define RELAY_L1 12
#define RELAY_L2 14
#define BUTTON 13

#define CONFIG_TIMEOUT_MS 300000
#define OTA_TIMEOUT_MS 300000

#define LOOP_DELAY_MS 10
#define MANUAL_SETUP_MAX_DELAY_MS 500
#define MANUAL_SETUP_COUNTER 5

// commands
#define CMD_ON "1"
#define CMD_OFF "0"
#define CMD_SETUP "set"
#define CMD_OTA "ota"
#define CMD_RESET "rst"


// --------------------------------------------------
#define CONFIG_FILE "/config.json"

#define CHECKBOX "true' type='checkbox' style='width: auto;"
#define CHECKBOX_CHECKED "true' type='checkbox' checked style='width: auto;"

// MQTT config
char mqttServer[40]   = "TiffanyAching.local";
char mqttPort[6]      = "1883";
char mqttClientName[40];
char mqttUser[40]     = "moist";
char mqttPassword[40] = "password";
char mqttOutTopic1[40] = "ibnhouse/light/kitchen/sta1";
char mqttInTopic1[40]  = "ibnhouse/light/kitchen/cmd1";
char mqttOutTopic2[40] = "ibnhouse/light/kitchen/sta2";
char mqttInTopic2[40]  = "ibnhouse/light/kitchen/cmd2";

boolean offline = true;

WiFiClient espClient;
WiFiManager wifiManager;
WiFiManagerParameter customMqttServer("mqtt_server", "mqtt server", "mqtt server", 40);
WiFiManagerParameter customMqttPort("mqtt_port", "mqtt port", "port", 6);
WiFiManagerParameter customMqttClientName("mqtt_client_name", "mqtt client name", "mqtt client name", 16);
WiFiManagerParameter customMqttUser("mqtt_user", "mqtt user", "mqtt user", 16);
WiFiManagerParameter customMqttPassword("mqtt_password", "mqtt password", "mqtt password", 16);
WiFiManagerParameter customMqttOutTopic1("mqtt_out_topic1", "mqtt output 1 topic", "mqtt out 1 topic", 40);
WiFiManagerParameter customMqttInTopic1("mqtt_in_topic1", "mqtt commands 1 topic", "mqtt in 1 topic", 40);
WiFiManagerParameter customMqttOutTopic2("mqtt_out_topic2", "mqtt output 2 topic", "mqtt out 2 topic", 40);
WiFiManagerParameter customMqttInTopic2("mqtt_in_topic2", "mqtt commands 2 topic", "mqtt in 2 topic", 40);
WiFiManagerParameter customSwapRelays("swap_relays", "", CHECKBOX, 70);
WiFiManagerParameter customSwapRelaysLabel("swap relays");
WiFiManagerParameter customRespectSwitchState("respect_switch", "", CHECKBOX, 70);
WiFiManagerParameter customRespectSwitchStateLabel("respect switch states");
WiFiManagerParameter customInvertSwitch("invert_switch", "", CHECKBOX, 70);
WiFiManagerParameter customInvertSwitchLabel("invert switch keys");
bool wifiManagerSetupRunning = false;
unsigned long wifiManagerSetupStart;
bool otaRunning = false;
unsigned long otaStart;
bool restart = false;

PubSubClient mqttClient(espClient);
unsigned long mqttConnectAttempt = 0;
unsigned long mqttConnectDelay = 0;
WiFiUDP udp;
mDNSResolver::Resolver mDnsResolver(udp);
IPAddress mqttServerIp = INADDR_NONE;

boolean swapRelays = false;
boolean invertSwitch = false;
boolean respectSwitchState = true;
boolean s1State;
boolean s2State;
boolean l1State;
boolean l2State;
unsigned long manualSetupActivatorTime;
int manualSetupModeCounter;

Ticker ledTicker;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SWITCH_S1, INPUT);
  pinMode(SWITCH_S2, INPUT);
  pinMode(RELAY_L1, OUTPUT);
  pinMode(RELAY_L2, OUTPUT);
  pinMode(BUTTON, INPUT);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP 

  // enable Auto Light Sleep (to reduce consumption during delay() periods in main loop)
  // enabling this + delay() in the main loop reduces consumption by module 0.8w -> 0.5w
  wifi_set_sleep_type(LIGHT_SLEEP_T); 

  digitalWrite(LED_BUILTIN, LOW); // turn it on

  s1State = ( digitalRead(SWITCH_S1) == LOW ); 
  s2State = ( digitalRead(SWITCH_S2) == LOW );

  manualSetupModeCounter = 0;

  startWifiManager(false);
  
  // MQTT connection
  mqttClient.setCallback(mqttCallback);

  // OTA progress
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (digitalRead(LED_BUILTIN) == HIGH) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
  });

}

void loop() {

  if (otaRunning) {
    
    ArduinoOTA.handle();

    if ((millis() - otaStart) > OTA_TIMEOUT_MS) {
      restart = true;
    }
    
  } else {
    
    gpioLoop();
  
    wifimanagerLoop();
  
    if (!offline) {
      mqttLoop();
    }
  
    // try to save power by delaying, which forces a light sleep mode.
    delay(LOOP_DELAY_MS);
    
  }

  if (restart) {
    ESP.restart();
  }
}

void gpioLoop() {
  
  // is WifiManager configuration portal requested?
  if ( digitalRead(BUTTON) == LOW ) {
    startWifiManager(true);
  }

  // process manual switches
  boolean s1Changed = false;
  boolean s2Changed = false;
  if ( digitalRead(SWITCH_S1) == LOW ) {
    if (s1State == false) {
      s1Changed = true;
    }
    s1State = true;
  } else {
    if (s1State == true) {
      s1Changed = true;
    }
    s1State = false;
  }
  if ( digitalRead(SWITCH_S2) == LOW ) {
    if (s2State == false) {
      s2Changed = true;
    }
    s2State = true;
  } else {
    if (s2State == true) {
      s2Changed = true;
    }
    s2State = false;
  }

  if (s1Changed) {
    if (respectSwitchState) {
      if (swapRelays) {
        if (invertSwitch) {
          if (s1State) {
            disableL2();
          } else {
            enableL2();
          }
        } else {
          if (s1State) {
            enableL2();
          } else {
            disableL2();
          }
        }
      } else {
        if (invertSwitch) {
          if (s1State) {
            disableL1();
          } else {
            enableL1();
          }
        } else {
          if (s1State) {
            enableL1();
          } else {
            disableL1();
          }
        }
      }
    } else {
      if (swapRelays) {
        if (l2State) {
          disableL2();
        } else {
          enableL2();
        }
      } else {
        if (l1State) {
          disableL1();
        } else {
          enableL1();
        }
      }
    }
  }

  if (s2Changed) {
    if (respectSwitchState) {
      if (swapRelays) {
        if (invertSwitch) {
          if (s2State) {
            disableL1();
          } else {
            enableL1();
          }
        } else {
          if (s2State) {
            enableL1();
          } else {
            disableL1();
          }
        }
      } else {
        if (invertSwitch) {
          if (s2State) {
            disableL2();
          } else {
            enableL2();
          }
        } else {
          if (s2State) {
            enableL2();
          } else {
            disableL2();
          }
        }
      }
    } else {
      if (swapRelays) {
        if (l1State) {
          disableL1();
        } else {
          enableL1();
        }
      } else {
        if (l2State) {
          disableL2();
        } else {
          enableL2();
        }
      }
    }
  }

  if (s1Changed || s2Changed) {
    unsigned long manualSetupActivatorDelay = millis() - manualSetupActivatorTime;
    if (manualSetupActivatorDelay < MANUAL_SETUP_MAX_DELAY_MS){
      if (++manualSetupModeCounter == MANUAL_SETUP_COUNTER) {
        startWifiManager(true);
      }
    } else {
      manualSetupModeCounter = 0;
    }
    manualSetupActivatorTime = millis();
  }

}

void wifimanagerLoop() {

  wifiManager.process();
  if (wifiManagerSetupRunning) {
    if ((millis() - wifiManagerSetupStart) > CONFIG_TIMEOUT_MS) {
      wifiManager.stopConfigPortal();
      wifiManagerSetupStopped();
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (offline) {
      mDnsResolver.setLocalIP(WiFi.localIP());
      mqttServerIp = mDnsResolver.search(mqttServer);
      // MQTT connection
      if(mqttServerIp != INADDR_NONE) {
        mqttClient.setServer(mqttServerIp, atoi(mqttPort));
      } else {
        mqttClient.setServer(mqttServer, atoi(mqttPort));
      }
    }
    offline = false;
  } else {
    offline = true;
  }

  if (!offline) {
    mDnsResolver.loop();
  }
  
}

void ledTick()
{
  int ledState = digitalRead(LED_BUILTIN);
  digitalWrite(LED_BUILTIN, !ledState);
}

//callback notifying us of the need to save config
void saveParamsCallback () {
  
  //save the custom parameters to FS
  //read updated parameters
  strcpy(mqttServer, customMqttServer.getValue());
  strcpy(mqttPort, customMqttPort.getValue());
  strcpy(mqttClientName, customMqttClientName.getValue());
  strcpy(mqttUser, customMqttUser.getValue());
  strcpy(mqttPassword, customMqttPassword.getValue());
  strcpy(mqttOutTopic1, customMqttOutTopic1.getValue());
  strcpy(mqttInTopic1, customMqttInTopic1.getValue());
  strcpy(mqttOutTopic2, customMqttOutTopic2.getValue());
  strcpy(mqttInTopic2, customMqttInTopic2.getValue());
  swapRelays = strcmp("true", customSwapRelays.getValue()) == 0 ? true : false;
  respectSwitchState = strcmp("true", customRespectSwitchState.getValue()) == 0 ? true : false;
  invertSwitch = strcmp("true", customInvertSwitch.getValue()) == 0 ? true : false;

  DynamicJsonDocument json(1024);
  json["mqtt_server"] = mqttServer;
  json["mqtt_port"] = mqttPort;
  json["mqtt_client_name"] = mqttClientName;
  json["mqtt_user"] = mqttUser;
  json["mqtt_password"] = mqttPassword;
  json["mqtt_out_topic1"] = mqttOutTopic1;
  json["mqtt_in_topic1"] = mqttInTopic1;
  json["mqtt_out_topic2"] = mqttOutTopic2;
  json["mqtt_in_topic2"] = mqttInTopic2;
  json["swap_relays"] = swapRelays ? "true" : "false";
  json["respect_switch_state"] = respectSwitchState ? "true" : "false";
  json["invert_switch"] = invertSwitch ? "true" : "false";

  File configFile = LittleFS.open(CONFIG_FILE, "w");
  serializeJson(json, configFile);
  configFile.close();
  //end save
  
  wifiManagerSetupStopped();
}

void wifiManagerSetupStarted(WiFiManager *myWiFiManager) {
  ledTicker.attach_ms(1000, ledTick); // start slow blinking
  wifiManagerSetupRunning = true;
  wifiManagerSetupStart = millis();
}

void wifiManagerSetupStopped() {
  //ESP.restart();
  restart = true; // don't restart immediately. let WifiManager finish handleWifiSave() execution

  //ledTicker.detach();
  //digitalWrite(LED_BUILTIN, LOW); // turn it on
  //wifiManagerSetupRunning = false;

  //WiFi.hostname(mqttClientName);
}

void startWifiManager(boolean onDemand) {

  if (wifiManagerSetupRunning) {
    return;
  }

  if (!onDemand) {
    String apName = "Detritus-" + String(ESP.getChipId(), HEX);
    strcpy(mqttClientName, apName.c_str());
    
    if (LittleFS.begin()) {
      if (LittleFS.exists("/config.json")) {
        //file exists, reading and loading
        File configFile = LittleFS.open(CONFIG_FILE, "r");
        if (configFile) {
          size_t size = configFile.size();
          // Allocate a buffer to store contents of the file.
          std::unique_ptr<char[]> buf(new char[size]);
  
          configFile.readBytes(buf.get(), size);
          DynamicJsonDocument json(1024);
          DeserializationError jsonError = deserializeJson(json, buf.get());
          if (!jsonError) {
            if (json.containsKey("mqtt_server") && strlen(json["mqtt_server"]) > 0) strcpy(mqttServer, json["mqtt_server"]);
            if (json.containsKey("mqtt_port") && strlen(json["mqtt_port"]) > 0) strcpy(mqttPort, json["mqtt_port"]);
            if (json.containsKey("mqtt_client_name") && strlen(json["mqtt_client_name"]) > 0) strcpy(mqttClientName, json["mqtt_client_name"]);
            if (json.containsKey("mqtt_user") && strlen(json["mqtt_user"]) > 0) strcpy(mqttUser, json["mqtt_user"]);
            if (json.containsKey("mqtt_password") && strlen(json["mqtt_password"]) > 0) strcpy(mqttPassword, json["mqtt_password"]);
            if (json.containsKey("mqtt_out_topic1") && strlen(json["mqtt_out_topic1"]) > 0) strcpy(mqttOutTopic1, json["mqtt_out_topic1"]);
            if (json.containsKey("mqtt_in_topic1") && strlen(json["mqtt_in_topic1"]) > 0) strcpy(mqttInTopic1, json["mqtt_in_topic1"]);
            if (json.containsKey("mqtt_out_topic2") && strlen(json["mqtt_out_topic2"]) > 0) strcpy(mqttOutTopic2, json["mqtt_out_topic2"]);
            if (json.containsKey("mqtt_in_topic2") && strlen(json["mqtt_in_topic2"]) > 0) strcpy(mqttInTopic2, json["mqtt_in_topic2"]);
            if (json.containsKey("swap_relays") && strlen(json["swap_relays"]) > 0) swapRelays = strcmp("true", json["swap_relays"]) == 0 ? true : false;
            if (json.containsKey("respect_switch_state") && strlen(json["respect_switch_state"]) > 0) respectSwitchState = strcmp("true", json["respect_switch_state"]) == 0 ? true : false;
            if (json.containsKey("invert_switch") && strlen(json["invert_switch"]) > 0) invertSwitch = strcmp("true", json["invert_switch"]) == 0 ? true : false;
          }
        }
      }
    }
    //end read
  
    WiFi.hostname(mqttClientName);
    ArduinoOTA.setHostname(mqttClientName);
  
    customMqttServer.setValue(mqttServer, 40);
    customMqttPort.setValue(mqttPort, 6);
    customMqttClientName.setValue(mqttClientName, 40);
    customMqttUser.setValue(mqttUser, 40);
    customMqttPassword.setValue(mqttPassword, 40);
    customMqttOutTopic1.setValue(mqttOutTopic1, 40);
    customMqttInTopic1.setValue(mqttInTopic1, 40);
    customMqttOutTopic2.setValue(mqttOutTopic2, 40);
    customMqttInTopic2.setValue(mqttInTopic2, 40);
  
    wifiManager.setSaveParamsCallback(saveParamsCallback);
    wifiManager.setAPCallback(wifiManagerSetupStarted);
  
    wifiManager.setConfigPortalTimeout(CONFIG_TIMEOUT_MS / 1000);
    wifiManager.setConfigPortalBlocking(false);
  
    //add all your parameters here
    wifiManager.addParameter(&customMqttServer);
    wifiManager.addParameter(&customMqttPort);
    wifiManager.addParameter(&customMqttClientName);
    wifiManager.addParameter(&customMqttUser);
    wifiManager.addParameter(&customMqttPassword);
    wifiManager.addParameter(&customMqttOutTopic1);
    wifiManager.addParameter(&customMqttInTopic1);
    wifiManager.addParameter(&customMqttOutTopic2);
    wifiManager.addParameter(&customMqttInTopic2);
    wifiManager.addParameter(&customSwapRelays);
    wifiManager.addParameter(&customSwapRelaysLabel);
    wifiManager.addParameter(&customRespectSwitchState);
    wifiManager.addParameter(&customRespectSwitchStateLabel);
    wifiManager.addParameter(&customInvertSwitch);
    wifiManager.addParameter(&customInvertSwitchLabel);

  }
  
  // refresh dirty hacked boolean values
  customSwapRelays.setValue(swapRelays ? CHECKBOX_CHECKED : CHECKBOX, 70);
  customRespectSwitchState.setValue(respectSwitchState ? CHECKBOX_CHECKED : CHECKBOX, 70);
  customInvertSwitch.setValue(invertSwitch ? CHECKBOX_CHECKED : CHECKBOX, 70);
  
  if (onDemand) {
    wifiManager.startConfigPortal(mqttClientName);
  } else {
    wifiManager.autoConnect(mqttClientName);
  }

}

void enableL1() {
  digitalWrite(RELAY_L1, HIGH); // turn on
  l1State = true;
  mqttCommunicate(true);
}

void enableL2(){
  digitalWrite(RELAY_L2, HIGH); // turn on
  l2State = true;
  mqttCommunicate(false);
}

void disableL1(){
  digitalWrite(RELAY_L1, LOW); // turn off
  l1State = false;
  mqttCommunicate(true);
}

void disableL2(){
  digitalWrite(RELAY_L2, LOW); // turn off
  l2State = false;
  mqttCommunicate(false);
}


void mqttLoop() {
  if (!mqttClient.connected()) {
    if(!mqttReconnect()){
      return;
    }
  }
  mqttClient.loop();
}

void mqttCommunicate(boolean l1Report) {
  if (!offline && mqttClient.connected()) {
    char mqttMsg[3];
    mqttMsg[2] = 0;
    if(l1Report){
      mqttMsg[0] = l1State ? '1' : '0';
      if (swapRelays) {
        if (invertSwitch) {
          mqttMsg[1] = s2State ? 0 : '.';
        } else {
          mqttMsg[1] = s2State ? '.' : 0;
        }
      } else {
        if (invertSwitch) {
          mqttMsg[1] = s1State ? 0 : '.';
        } else {
          mqttMsg[1] = s1State ? '.' : 0;
        }
      }
      mqttClient.publish(mqttOutTopic1, mqttMsg, true);
    } else {
      mqttMsg[0] = l2State ? '1' : '0';
      if (swapRelays) {
        if (invertSwitch) {
          mqttMsg[1] = s1State ? 0 : '.';
        } else {
          mqttMsg[1] = s1State ? '.' : 0;
        }
      } else {
        if (invertSwitch) {
          mqttMsg[1] = s2State ? 0 : '.';
        } else {
          mqttMsg[1] = s2State ? '.' : 0;
        }
      }
      mqttClient.publish(mqttOutTopic2, mqttMsg, true);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {

  char payloadCopy[10];
  int _length = std::min<unsigned int>(length, 9);
  strncpy(payloadCopy, (char*)payload, _length);
  payloadCopy[_length] = 0x0;

  String cmd = String(payloadCopy);
  if (cmd.startsWith(CMD_SETUP)){
    startWifiManager(true);
  }

  if (cmd.startsWith(CMD_RESET)){
    restart = true;
  }

  if (cmd.startsWith(CMD_OTA)){
    otaStart = millis();
    otaRunning = true;
    ledTicker.attach_ms(300, ledTick); // start fast blinking
    ArduinoOTA.begin();
  }

  if(strcmp(topic, mqttInTopic1) == 0) {
    if (cmd.startsWith(CMD_ON)){
      enableL1();
    }
    if (cmd.startsWith(CMD_OFF)){
      disableL1();
    }
  } else if (strcmp(topic, mqttInTopic2) == 0) {
    if (cmd.startsWith(CMD_ON)){
      enableL2();
    }
    if (cmd.startsWith(CMD_OFF)){
      disableL2();
    }
  }

}

boolean mqttReconnect() {
  if (!mqttClient.connected()) {
    if (millis() - mqttConnectAttempt > mqttConnectDelay ) { // don't attempt more often than a delay
      mqttConnectDelay += 1000; // increase reconnect attempt delay by 1 second
      if (mqttConnectDelay > 60000) { // don't attempt more frequently than once a minute
        mqttConnectDelay = 60000;
      }
      // Attempt to connect
      mqttConnectAttempt = millis();
      if (mqttClient.connect(mqttClientName, mqttUser, mqttPassword)) {
        // Once connected resubscribe
        mqttClient.subscribe(mqttInTopic1);
        mqttClient.subscribe(mqttInTopic2);
        mqttConnectDelay = 0;
        return true;
      }
    }
  }
  return false;
}
