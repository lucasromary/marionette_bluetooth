

#include <Arduino.h>
#include <DynamixelShield.h>

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
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <SPI.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char *ssid = "crunchlab";
const char *password = "crunchlab";

BluetoothSerial SerialBT;

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 23
#define stepPin 22
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

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
const uint8_t DXL_ID_LIST[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
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
bool goal_achieved = 0;

// Multi_motor
int goal_position_sequence1[15] = {50, 50, 14000, 14000, 14000, 14000, 14000, 10000, 14000, 14000, 14000, 14000, 50, 50, 50};
int goal_position_sequence2[15] = {50, 50, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 50, 50, 50};
int goal_position_sequence3[15] = {50, 50, 8000, 8000, 8000, 8000, 8000, 4000, 8000, 8000, 8000, 8000, 50, 50, 50};
int goal_position_sequence_repos[15] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
int goal_position_sequence_test[15] = {50, 50, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 50, 50, 50};

int goal_position_sequence_coucou1[15] = {50, 50, 50, -1024, 1024, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
int goal_position_sequence_coucou2[15] = {50, 50, 50, -2048, 1024, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
int goal_position_sequence_coucou3[15] = {50, 50, 50, -1024, 2028, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};

int goal_position_sequence_tete_baisse[15] = {50, 4096, 1024, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
int goal_position_sequence_tete_releve[15] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};

int goal_position_debout[15] = {50, 50, -5000, -7700, -7000, -6000, 5000, 6000, 9200, 9200, 5500, 6500, 50, 50, 50};
int goal_position_tete_tourne_gauche[15] = {50, 50, -5000, -7700, -6500, -6000, 5000, 6000, 7500, 10600, 5500, 6500, 50, 50, 650};
int goal_position_tete_tourne_droite[15] = {50, 50, -5000, -7700, -6500, -6000, 5000, 6000, 10600, 7500, 5500, 6500, 50, 50, -650};

// genou gauche levé {50, 50, -5000, 50, 50, 50, 50, 50, 50, 50, 1750, 50, 50, 50, 50};
/*
Séquence marionette
*/

// Séquence 1

int goal_position_tab[15];
int present_position_tab[15];
bool goal_achieved_tab[15];
int goal_achieved_result = 0;
int incomingByte = 0;
int motor_choice = -1;
int position_choice = -1;

//
int idx[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int goal_pos_dynamixel[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int parse_tab[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int goal_pos_pap = 0;

void go_to_positions_multiple_motors_test(int *tab)
{
  uint8_t i, recv_cnt;

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
  //delay(250);
  previous_timer = millis();
  /**
   * Sync read, On releve l'information de la position. Lorsque l'objectif est atteint (goal-present < 10) pour chacun des moteurs on peut passer à un prochain mouvement
   */
  int compteur = 0;

 while (dxl.readControlTableItem(MOVING, DXL_ID_LIST[0]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[1]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[2]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[3]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[4]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[5]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[6]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[7]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[8]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[9]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[10]) || dxl.readControlTableItem(MOVING, DXL_ID_LIST[11]) || millis() - previous_timer < 100)
  {
  //while ((dxl.readControlTableItem(MOVING, DXL_ID_LIST[0]) && dxl.readControlTableItem(MOVING, DXL_ID_LIST[1]) && dxl.readControlTableItem(MOVING, DXL_ID_LIST[2]) && dxl.readControlTableItem(MOVING, DXL_ID_LIST[3]) && dxl.readControlTableItem(MOVING, DXL_ID_LIST[4]) && dxl.readControlTableItem(MOVING, DXL_ID_LIST[5]) && dxl.readControlTableItem(MOVING, DXL_ID_LIST[6]) && dxl.readControlTableItem(MOVING, DXL_ID_LIST[7]) && dxl.readControlTableItem(MOVING, DXL_ID_LIST[8]) && dxl.readControlTableItem(MOVING, DXL_ID_LIST[9]) && dxl.readControlTableItem(MOVING, DXL_ID_LIST[10]) && dxl.readControlTableItem(MOVING, DXL_ID_LIST[11])) || millis() - previous_timer < 100)
  //{
    recv_cnt = dxl.syncRead(&sr_infos);
    compteur++;
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
  Serial.println(compteur);

  for (i = 0; i < sw_infos.xel_count; i++)
  {
    DEBUG_SERIAL.print("  ID: ");
    DEBUG_SERIAL.print(sw_infos.p_xels[i].id);
    DEBUG_SERIAL.print(" final Position: ");
    DEBUG_SERIAL.print(sr_data[i].present_position);
    DEBUG_SERIAL.print(" error Position: ");
    DEBUG_SERIAL.println(abs(sr_data[i].present_position - sw_data[i].goal_position));
  }

  DEBUG_SERIAL.println("=======================================================");
}

void go_to_position_test(int position, int ID_moteur)
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

void coucou(int vel, int accel)
{
  for (int i = 0; i < 12; i++)
  {
    set_velocity_accel(vel, accel, i);
  }
  DEBUG_SERIAL.println(dxl.readControlTableItem(PROFILE_VELOCITY, DXL_ID_LIST[0]));     // DEFAULT 10
  DEBUG_SERIAL.println(dxl.readControlTableItem(PROFILE_ACCELERATION, DXL_ID_LIST[0])); // 0
  // delay(5000);
  go_to_positions_multiple_motors_test(goal_position_sequence_coucou1);
  go_to_positions_multiple_motors_test(goal_position_sequence_coucou2);
  go_to_positions_multiple_motors_test(goal_position_sequence_coucou3);
  go_to_positions_multiple_motors_test(goal_position_sequence_coucou2);
  go_to_positions_multiple_motors_test(goal_position_sequence_coucou3);
  go_to_positions_multiple_motors_test(goal_position_sequence_coucou2);
  go_to_positions_multiple_motors_test(goal_position_sequence_coucou3);
  // go_to_positions_multiple_motors(goal_position_sequence_coucou1);
  // delay(2000);
}

void tourner_tete(){
  go_to_positions_multiple_motors_test(goal_position_debout);
  go_to_positions_multiple_motors_test(goal_position_tete_tourne_gauche);
  go_to_positions_multiple_motors_test(goal_position_tete_tourne_droite);
  go_to_positions_multiple_motors_test(goal_position_sequence_repos);
}

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

void setup()
{
  // put your setup code here, to run once:
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);

  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);
  Serial.println(F("A4988 Initialized"));

  dxl.begin(BAUDRATE_DXL);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  initOTA();

  SerialBT.begin("ESP32marionette"); // Bluetooth device name

  for (i = 0; i < DXL_ID_CNT; i++)
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
  for (i = 0; i < DXL_ID_CNT; i++)
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

  for (i = 0; i < DXL_ID_CNT; i++)
  {
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t *)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }

  sw_infos.is_info_changed = true;
  
  for (int i = 0; i < 15; i++)
  {
    set_velocity_accel(50, 100, i);
  }

  go_to_positions_multiple_motors_test(goal_position_sequence_repos); // debout
  go_to_positions_multiple_motors_test(goal_position_sequence_test); // debout
  go_to_positions_multiple_motors_test(goal_position_sequence_repos); // debout
  /*
  go_to_position_test(50,3);
  delay(1000);
  go_to_position_test(500,3);
  delay(1000);
  go_to_position_test(50,3);
  delay(1000);
  */
  /*
    DEBUG_SERIAL.print("start");
    go_to_positions_multiple_motors_test(goal_position_sequence1); //debout
    delay(5000);
    go_to_positions_multiple_motors_test(goal_position_sequence2); //
    delay(5000);
    stepper.moveTo(1500);
    stepper.runToPosition();
    go_to_positions_multiple_motors_test(goal_position_sequence_repos);
    stepper.moveTo(0);
    stepper.runToPosition();
    DEBUG_SERIAL.print("finish");
  */

  /* TEST PROFILE VEL & ACC

   set_velocity_accel(0, 300, 0);

   //DEBUG_SERIAL.println(dxl.readControlTableItem(PROFILE_VELOCITY, DXL_ID_LIST[0])); //DEFAULT 10
   //DEBUG_SERIAL.println(dxl.readControlTableItem(PROFILE_ACCELERATION, DXL_ID_LIST[0])); // 0

   go_to_position_test(4096,0);
   delay(1000);
   go_to_position_test(0,0);
   delay(1000);

   set_velocity_accel(40, 300, 0);
   go_to_position_test(4096,0);
   delay(1000);
   go_to_position_test(0,0);
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
      go_to_positions_multiple_motors_test(goal_pos_dynamixel);
      stepper.moveTo(parse_tab[15]);
      stepper.runToPosition();
    }
  /*
    go_to_position_test(1024, 0);
    delay(1000);
    go_to_position_test(0, 0);
    delay(1000);
  */
}
