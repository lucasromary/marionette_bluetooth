#include <Arduino.h>
#include <DynamixelShield.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(8, 9); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif
#define BAUDRATE_DXL 2000000
#include <AccelStepper.h>
#include <BluetoothSerial.h>
#include <SPI.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

/*
 * WIFI & BLUETOTOH & MQTT VARIABLES
 */

#define WIFI_SSID "crunchlab"
#define WIFI_PASSWORD "crunchlab"

#define MQTT_HOST IPAddress(172, 23, 193, 33)
#define MQTT_PORT 1883
#define MQTT_TOPIC_STATE "Marionette/State/State"
#define MQTT_TOPIC_SEQUENCE "Marionette/State/SequenceStep"
#define MQTT_TOPIC_MOVEMENT "Marionette/State/movement"
#define MQTT_TOPIC_ERRORS "Marionette/Errors"
#define MQTT_TOPIC_MAINTENANCE_REBOOT_MOTOR "Marionette/Maintenance/reboot_motor"
#define MQTT_TOPIC_MAINTENANCE_RESTART "Marionette/Maintenance/restart"
#define MQTT_TOPIC_MAINTENANCE_GET_POS "Marionette/Maintenance/get_pos"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
int status = WL_IDLE_STATUS;

BluetoothSerial SerialBT;

/*
 * STEPPER VARIABLES
 */

#define dirPin 23
#define stepPin 22
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

/*
 * DYNAMIXEL VARIABLES
 */

/*
** ID 1  Main gauche (avance/recule)
** ID 2  Main droite (avance/recule)
** ID 3  Genou gauche
** ID 4  Aile droite et gauche
** ID 5  Tête arrière
** ID 6  Main droite intérieur
** ID 7  Main droite extérieur
** ID 8  Genou droit
** ID 9  Tête droite
** ID 10 Tête gauche
** ID 11 Main gauche intérieur
** ID 12 Main gauche extérieur
** ID 13 Aile gauche (avance/recule)
** ID 14 Aile droite (avance/recule)
** ID 15 Tête rotation
*/
const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 15;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data
{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data
{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

DynamixelShield dxl;

// This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

long timer = 0;
long previous_timer = 0;

// single motor
int goal_position = 0;
int present_position = 0;

// Multi_motor
int goal_position_sequence1[DXL_ID_CNT] = {50, 50, 14000, 14000, 14000, 14000, 14000, 10000, 14000, 14000, 14000, 14000, 50, 50, 50};
int goal_position_sequence2[DXL_ID_CNT] = {50, 50, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 50, 50, 50};
int goal_position_sequence3[DXL_ID_CNT] = {50, 50, 8000, 8000, 8000, 8000, 8000, 4000, 8000, 8000, 8000, 8000, 50, 50, 50};
int goal_position_sequence_repos[DXL_ID_CNT] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
int goal_position_sequence_test[DXL_ID_CNT] = {50, 50, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 50, 50, 50};

int goal_position_sequence_coucou1[DXL_ID_CNT] = {50, 50, 50, -1024, 1024, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
int goal_position_sequence_coucou2[DXL_ID_CNT] = {50, 50, 50, -2048, 1024, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
int goal_position_sequence_coucou3[DXL_ID_CNT] = {50, 50, 50, -1024, 2028, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};

int goal_position_sequence_tete_baisse[DXL_ID_CNT] = {50, 4096, 1024, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
int goal_position_sequence_tete_releve[DXL_ID_CNT] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};

int goal_position_debout[DXL_ID_CNT] = {50, 50, -5000, -7700, -7000, -6000, 5000, 6000, 9200, 9200, 5500, 6500, 50, 50, 50};
int goal_position_tete_tourne_gauche[DXL_ID_CNT] = {50, 50, -5000, -7700, -6500, -6000, 5000, 6000, 7500, 10400, 5500, 6500, 50, 50, 512};
int goal_position_tete_tourne_droite[DXL_ID_CNT] = {50, 50, -5000, -7700, -6500, -6000, 5000, 6000, 10400, 7500, 5500, 6500, 50, 50, -512};

int goal_position_tab[DXL_ID_CNT];
int present_position_tab[DXL_ID_CNT];
int movement = 0;
int sequence = 0;

int idx[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int goal_pos_dynamixel[DXL_ID_CNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int parse_tab[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int compteur_sync_read = 0;

/*
** Maintenance
*/
void check_hardware_issue()
{
  bool error = 0;
  for (int i = 0; i < DXL_ID_CNT; i++)
  {
    uint8_t res = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DXL_ID_LIST[i]);
    switch (res)
    {
    case 1:
      Serial.print("Voltage Input error (Overvoltage) for dynamixel : ");
      Serial.println(i + 1);
      error = 1;
      mqttClient.publish((MQTT_TOPIC_ERRORS), 1, false, "1");
      break;
    case 4:
      Serial.print("Overheating error for dynamixel : ");
      Serial.println(i + 1);
      error = 1;
      mqttClient.publish((MQTT_TOPIC_ERRORS), 1, false, "4");
      break;
    case 8:
      Serial.print("motor encodeur error for dynamixel : ");
      Serial.println(i + 1);
      error = 1;
      mqttClient.publish((MQTT_TOPIC_ERRORS), 1, false, "8");
      break;
    case 16:
      Serial.print("Electrical shock error for dynamixel : ");
      Serial.println(i + 1);
      error = 1;
      mqttClient.publish((MQTT_TOPIC_ERRORS), 1, false, "16");
      break;
    case 32:
      Serial.print("Overload for dynamixel : ");
      Serial.println(i + 1);
      error = 1;
      mqttClient.publish((MQTT_TOPIC_ERRORS), 1, false, "32");
      break;
    default:
      break;
    }
  }
  if (!error)
  {
    Serial.print("No hardware errors.");
  }
}

bool is_goals_reached()
{
  bool goal_reached = 1;
  for (int i = 0; i < DXL_ID_CNT; i++)
  {
    if (abs(goal_position_tab[i] - present_position_tab[i]) > 50)
    {
      Serial.print("not reached for dynamixel ");
      Serial.println(i + 1);
      goal_reached = 0;
    }
  }
  return goal_reached;
}

void reboot_dynamixel(int id)
{
  if (dxl.reboot(id))
  {
    dxl.torqueOff(id);
    delay(100);
    dxl.torqueOn(id);
    delay(100);
  }
}

/*
** WIFI & BLUETOTOH & MQTT FONCTIONS
*/
void connectToWifi()
{
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  mqttClient.subscribe(MQTT_TOPIC_MAINTENANCE_REBOOT_MOTOR, 2);
  mqttClient.subscribe(MQTT_TOPIC_MAINTENANCE_RESTART, 2);
  mqttClient.subscribe(MQTT_TOPIC_MAINTENANCE_GET_POS, 2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  String messageTemp;
  for (int i = 0; i < len; i++)
  {
    // Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  Serial.print("  message received on topic: ");
  Serial.print(topic);
  Serial.print("  : ");
  Serial.println(messageTemp);

  if (String(topic) == MQTT_TOPIC_MAINTENANCE_REBOOT_MOTOR)
  {
    Serial.println("Reboot motor : ");
    Serial.println(messageTemp.toInt());
    reboot_dynamixel(messageTemp.toInt());
  }
  if (String(topic) == MQTT_TOPIC_MAINTENANCE_RESTART && messageTemp == "1")
  {
    Serial.println("Restart");
    ESP.restart();
  }
  if (String(topic) == MQTT_TOPIC_MAINTENANCE_GET_POS && messageTemp == "1")
  {
    Serial.println("Get motors position");
    for(int i = 0; i < DXL_ID_CNT; i++){
      String temp = "Marionette/Maintenance/get_pos/" + String(i);
      mqttClient.publish((temp).c_str(), 1, false, String(present_position_tab[i]).c_str());
    }
  }
}

void onMqttPublish(uint16_t packetId)
{

  // Serial.println("Publish acknowledged.");
}

void init_mqtt()
{
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();
}

/*
** UTILITIES FONCTIONS
*/
void initOTA()
{
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("ESP32_marionette");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
      .onStart([]()
               {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

  ArduinoOTA.begin();
  Serial.println(WiFi.localIP());
}

void parsing(String myString)
{

  /*
    int ind1 = myString.indexOf(',');                  // finds location of first ,
    angle = myString.substring(0, ind1);           // captures first data String
    ind2 = myString.indexOf(',', ind1 + 1);        // finds location of second ,
    fuel = myString.substring(ind1 + 1, ind2 + 1); // captures second data String
    ind3 = myString.indexOf(',', ind2 + 1);
    speed1 = myString.substring(ind2 + 1, ind3 + 1);
    ind4 = myString.indexOf(',', ind3 + 1);
    altidude = myString.substring(ind3 + 1); // captures remain part of data after last ,
  */
  idx[0] = myString.indexOf(',');
  parse_tab[0] = (myString.substring(0, idx[0])).toInt();
  // Serial.println(parse_tab[0]);
  for (int i = 1; i < 16; i++)
  {
    idx[i] = myString.indexOf(',', idx[i - 1] + 1);
    parse_tab[i] = (myString.substring(idx[i - 1] + 1, idx[i] + 1)).toInt();
    // Serial.println(parse_tab[i]);
  }
}

/*
** DYNAMIXEL FONCTIONS
*/
void go_to_positions_multiple_motors(int *tab)
{
  movement++;
  uint8_t i, recv_cnt;
  mqttClient.publish((MQTT_TOPIC_SEQUENCE), 1, false, String(sequence).c_str());
  mqttClient.publish((MQTT_TOPIC_MOVEMENT), 1, false, String(movement).c_str());
  // Insert a new Goal Position to the SyncWrite Packet
  for (int i = 0; i < DXL_ID_CNT; i++)
  {
    sw_data[i].goal_position = tab[i];
  }
  sw_infos.is_info_changed = true;

  /**
   * Sync Write, changement de la valeur de goal_position en fonction du tableau en entrée
   */
  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  if (dxl.syncWrite(&sw_infos) == true)
  {
    for (i = 0; i < sw_infos.xel_count; i++)
    {
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.print(sw_infos.p_xels[i].id);
      DEBUG_SERIAL.print(" Goal Position: ");
      DEBUG_SERIAL.println(sw_data[i].goal_position);
      goal_position_tab[i] = sw_data[i].goal_position;
    }
  }
  else
  {
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }

  DEBUG_SERIAL.println("________________________________________________");
  // delay(250);
  previous_timer = millis();
  /**
   * Sync read, On releve l'information de la position. Lorsque l'objectif est atteint (goal-present < 10) pour chacun des moteurs on peut passer à un prochain mouvement
   */
  compteur_sync_read = 0;

  while (dxl.readControlTableItem(MOVING, DXL_ID_LIST[0]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[1]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[2]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[3]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[4]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[5]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[6]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[7]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[8]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[9]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[10]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[11]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[12]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[13]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[14]) || millis() - previous_timer < 200)
  {
    recv_cnt = dxl.syncRead(&sr_infos);
    compteur_sync_read++;
    /*
    if (recv_cnt > 0)
    {
      for (i = 0; i < 2; i++)
      {
        present_position_tab[i] = sr_data[i].present_position;
        DEBUG_SERIAL.print("  ID: ");
        DEBUG_SERIAL.print(sr_infos.p_xels[i].id);
        DEBUG_SERIAL.print(" Present Position: ");
        DEBUG_SERIAL.print(sr_data[i].present_position);
        DEBUG_SERIAL.print(" Goal Position: ");
        DEBUG_SERIAL.println(goal_position_tab[i]);
      }
    }
    */
  }

  DEBUG_SERIAL.print(" Goal achieved timer: ");
  DEBUG_SERIAL.println(millis() - previous_timer);
  Serial.println(compteur_sync_read);

  for (i = 0; i < sw_infos.xel_count; i++)
  {
    DEBUG_SERIAL.print("  ID: ");
    DEBUG_SERIAL.print(sw_infos.p_xels[i].id);
    DEBUG_SERIAL.print(" final Position: ");
    DEBUG_SERIAL.print(sr_data[i].present_position);
    present_position_tab[i] = sr_data[i].present_position;
    DEBUG_SERIAL.print(" error Position: ");
    DEBUG_SERIAL.println(abs(sr_data[i].present_position - sw_data[i].goal_position));
  }
  if (!is_goals_reached())
  {
    delay(5000);
    check_hardware_issue();
  }
  DEBUG_SERIAL.println("=======================================================");
}

void go_to_position(int position, int ID_moteur)
{
  uint8_t recv_cnt;
  // ID_moteur--;
  // Insert a new Goal Position to the SyncWrite Packet
  sw_data[ID_moteur].goal_position = position;
  sw_infos.is_info_changed = true;

  /**
   * Sync Write, changement de la valeur de goal_position en fonction du tableau en entrée
   */
  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  if (dxl.syncWrite(&sw_infos) == true)
  { /*
     DEBUG_SERIAL.print("  ID: ");
     DEBUG_SERIAL.print(sw_infos.p_xels[ID_moteur].id);
     DEBUG_SERIAL.print(" Goal Position: ");
     DEBUG_SERIAL.println(sw_data[ID_moteur].goal_position);
     */
    goal_position = sw_data[ID_moteur].goal_position;
  }
  else
  {
    // DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    // DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  // DEBUG_SERIAL.println();

  previous_timer = millis();
  /**
   * Sync read, On releve l'information de la position. Lorsque l'objectif est atteint (goal-present < 10) pour chacun des moteurs on peut passer à un prochain mouvement
   */
  while (dxl.readControlTableItem(MOVING, DXL_ID_LIST[ID_moteur]) || millis() - previous_timer < 50)
  {
    recv_cnt = dxl.syncRead(&sr_infos);
    if (recv_cnt > 0)
    {
      DEBUG_SERIAL.print(dxl.readControlTableItem(MOVING_STATUS, DXL_ID_LIST[ID_moteur]));
      DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.print(dxl.readControlTableItem(MOVING, DXL_ID_LIST[ID_moteur]));
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.print(sr_infos.p_xels[ID_moteur].id);
      present_position = sr_data[ID_moteur].present_position;
      DEBUG_SERIAL.print(" Present Position: ");
      DEBUG_SERIAL.print(present_position);
      DEBUG_SERIAL.print(" Goal Position: ");
      DEBUG_SERIAL.println(goal_position);
    }
  }

  DEBUG_SERIAL.print("Goal achieved timer: ");
  DEBUG_SERIAL.print(millis() - previous_timer);
  delay(1000);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println(dxl.readControlTableItem(MOVING, DXL_ID_LIST[ID_moteur]));
  DEBUG_SERIAL.println("=======================================================");
}

void set_velocity_accel(int vel, int accel, int ID_moteur)
{
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_LIST[ID_moteur], vel);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_LIST[ID_moteur], accel);
}

void init_dynamixel()
{

  dxl.begin(BAUDRATE_DXL);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  for (int i = 0; i < DXL_ID_CNT; i++)
  {
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_EXTENDED_POSITION);
  }
  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;

  // Prepare the SyncRead structure
  for (int i = 0; i < DXL_ID_CNT; i++)
  {
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t *)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT; i++)
  {
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t *)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }

  sw_infos.is_info_changed = true;

  for (int i = 0; i < DXL_ID_CNT; i++)
  {
    set_velocity_accel(50, 100, i);
  }
}

/*
** Scénarii
*/
void coucou(int vel, int accel)
{
  for (int i = 0; i < 12; i++)
  {
    set_velocity_accel(vel, accel, i);
  }
  DEBUG_SERIAL.println(dxl.readControlTableItem(PROFILE_VELOCITY, DXL_ID_LIST[0]));     // DEFAULT 10
  DEBUG_SERIAL.println(dxl.readControlTableItem(PROFILE_ACCELERATION, DXL_ID_LIST[0])); // 0
  // delay(5000);
  go_to_positions_multiple_motors(goal_position_sequence_coucou1);
  go_to_positions_multiple_motors(goal_position_sequence_coucou2);
  go_to_positions_multiple_motors(goal_position_sequence_coucou3);
  go_to_positions_multiple_motors(goal_position_sequence_coucou2);
  go_to_positions_multiple_motors(goal_position_sequence_coucou3);
  go_to_positions_multiple_motors(goal_position_sequence_coucou2);
  go_to_positions_multiple_motors(goal_position_sequence_coucou3);
  // go_to_positions_multiple_motors(goal_position_sequence_coucou1);
  // delay(2000);
}

void tourner_tete()
{
  go_to_positions_multiple_motors(goal_position_debout);
  go_to_positions_multiple_motors(goal_position_tete_tourne_gauche);
  go_to_positions_multiple_motors(goal_position_tete_tourne_droite);
  go_to_positions_multiple_motors(goal_position_sequence_repos);
}

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);

  /*
  ** INIT ALL
  */
  init_mqtt();

  initOTA();

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);
  SerialBT.begin("ESP32marionette"); // Bluetooth device name

  init_dynamixel();

  /*
  ** Test multiple motors
  */
  sequence = 1;
  go_to_positions_multiple_motors(goal_position_sequence_repos); // debout
  go_to_positions_multiple_motors(goal_position_sequence_test); // debout
  go_to_positions_multiple_motors(goal_position_sequence_repos); // debout

  /* Test single motor
  go_to_position(50,3);
  delay(1000);
  go_to_position(500,3);
  delay(1000);
  go_to_position(50,3);
  delay(1000);
  */

  /*
  ** TEST PROFILE VEL & ACC

   set_velocity_accel(0, 300, 4);

   //DEBUG_SERIAL.println(dxl.readControlTableItem(PROFILE_VELOCITY, DXL_ID_LIST[0])); //DEFAULT 10
   //DEBUG_SERIAL.println(dxl.readControlTableItem(PROFILE_ACCELERATION, DXL_ID_LIST[0])); // 0

   go_to_position(4096,4);
   delay(1000);
   go_to_position(0,4);
   delay(1000);
   */
}

void loop()
{
  ArduinoOTA.handle();
  if (SerialBT.available())
  {
    String consigne2 = SerialBT.readString();
    Serial.println(consigne2);
    parsing(consigne2);
    for (int i = 0; i <= 14; i++)
    {
      goal_pos_dynamixel[i] = parse_tab[i];
    }
    go_to_positions_multiple_motors(goal_pos_dynamixel);

    stepper.moveTo(parse_tab[15]);
    stepper.runToPosition();
  }
  // check_hardware_issue();
  /*
    go_to_position_test(1024, 0);
    delay(1000);
    go_to_position_test(0, 0);
    delay(1000);
  */
}
