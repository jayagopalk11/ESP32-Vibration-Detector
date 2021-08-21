#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include<EEPROM.h>
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <driver/adc.h>
// MPU registers
#define SIGNAL_PATH_RESET  0x68
#define I2C_SLV0_ADDR      0x37
#define ACCEL_CONFIG       0x1C
#define MOT_THR            0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR            0x20  // This seems wrong // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MOT_DETECT_CTRL    0x69
#define INT_ENABLE         0x38
#define PWR_MGMT           0x6B //SLEEPY TIME
#define INT_STATUS 0x3A
#define MPU6050_ADDRESS 0x68 //AD0 is 0
#define MAX_WIFI_INTENTS      60

WiFiClient client;

#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    ""
#define AIO_KEY         ""


WebServer server(80);
boolean motionDetected = false;
String content;
int statusCode;
const int PushButton = 2;
int Push_button_state = HIGH;
String wifi_ssid = "";
String wifi_password = "";
String node_server = "";
int _intents = 0;


//Adafruit_MQTT_Client mqtt(&client, node_server.c_str(), AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Client *mqtt;// = Adafruit_MQTT_Client(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish *sensor;// = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "iot/test1");

//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot() {
  Serial.println("creatingsite");
  IPAddress ip = WiFi.softAPIP();
  String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
  content = "<!DOCTYPE HTML>\r\n<html>Intruder Detector from ESP32 at ";
  content += ipStr;
  content += "<p>";
  content += "</p><form method='get' action='setting'><label>SSID: </label><input name='ssid' length=32><br><label>Password: </label><input name='pass' length=64><br><label>Node Server IP: </label><input name='server' length=64><br><input type='submit'></form>";
  content += "<br><button><a href='/read'>Read EEPROM</a></button>";
  content += "</html>";
  server.send(200, "text/html", content);
}
void setting() {
  String qsid = server.arg("ssid");
  String qpass = server.arg("pass");
  String qserver = server.arg("server");
  if (qsid.length() > 0 && qpass.length() > 0) {
  Serial.println("clearing eeprom");
  for (int i = 0; i < 96; ++i) {
    EEPROM.write(i, 0);
  }
  Serial.println(qsid);
  Serial.println("");
  Serial.println(qpass);
  Serial.println("");
  Serial.println(qserver);
  Serial.println("");
  
  Serial.println("writing eeprom ssid:");
  for (int i = 0; i < qsid.length(); ++i)
  {
    EEPROM.write(i, qsid[i]);
  }
  Serial.println("writing eeprom pass:");
  for (int i = 0; i < qpass.length(); ++i)
  {
    EEPROM.write(32 + i, qpass[i]);
  }
  Serial.println("writing eeprom pass:");
  for (int i = 0; i < qserver.length(); ++i)
  {
    EEPROM.write(64 + i, qserver[i]);
  }
  EEPROM.commit();
  content = "{\"Success\":\"saved to eeprom... reset to boot into new wifi\"}";
  statusCode = 200;
  } else {
  content = "{\"Error\":\"404 not found\"}";
  statusCode = 404;
  Serial.println("Sending 404");
  }
  server.send(statusCode, "application/json", content);
}
void readEEPROM() {
  String rString = "SSID: ";
  rString += EEPROM.readString(0);
  Serial.print("Str:");
  Serial.println(rString);
  rString += "; Password: ";
  rString += EEPROM.readString(32);
  Serial.print("Str:");
  Serial.println(rString);
  rString += "; Node Server IP: ";
  rString += EEPROM.readString(64);
  Serial.print("Str:");
  Serial.println(rString);
  server.send(200, "text/html", rString);
}

//===============================================================
// Setup
//===============================================================

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.begin();
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}
//example showing using readbytev   ----    readByte(MPU6050_ADDRESS, GYRO_CONFIG);
uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                            // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

// sens argument configures wake up sensitivity
void configureMPU(int sens){
  writeByte( MPU6050_ADDRESS, 0x6B, 0x00);
  writeByte( MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0x07);//Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
  // writeByte( MPU6050_ADDRESS, I2C_SLV0_ADDR, 0x20);//write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
  writeByte( MPU6050_ADDRESS, ACCEL_CONFIG, 0x01);//Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
  writeByte( MPU6050_ADDRESS, MOT_THR, sens);  //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).
  writeByte( MPU6050_ADDRESS, MOT_DUR, 1 );  //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
  writeByte( MPU6050_ADDRESS, MOT_DETECT_CTRL, 0x15); //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )
  writeByte( MPU6050_ADDRESS, 0x37, 140 ); // now INT pin is active low
  writeByte( MPU6050_ADDRESS, INT_ENABLE, 0x40 ); //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.
  writeByte( MPU6050_ADDRESS, PWR_MGMT, 8 ); // 101000 - Cycle & disable TEMP SENSOR
  writeByte( MPU6050_ADDRESS, 0x6C, 7); // Disable Gyros
  Serial.println("MPU6050 configured");
}

void setupESPMPU(void){
  configureMPU(1);
  //attachInterrupt(intpin, doInt, RISING);
  adc_power_off();  // adc power off disables wifi entirely, upstream bug
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15,0); delay(1500);//1 = High, 0 = Low
  adc_power_off();
  esp_deep_sleep_start();  
}

void getCreds(){
  EEPROM.begin(512);
  delay(10);
  // read eeprom for ssid and pass
  Serial.println("Reading EEPROM ssid");
  String esid;
  for (int i = 0; i < 32; ++i)
  {
    esid += char(EEPROM.read(i));
  }
  Serial.print("SSID: ");
  //Serial.println(esid);
  Serial.println("Reading EEPROM pass");
  String epass = "";
  for (int i = 32; i < 64; ++i)
  {
    epass += char(EEPROM.read(i));
  }
  Serial.print("PASS: ");
  //Serial.println(epass);
  Serial.println("Reading EEPROM Node Server IP");
  String eserver = "";
  for (int i = 64; i < 96; ++i)
  {
    eserver += char(EEPROM.read(i));
  }
  Serial.print("SERVER: ");
  Serial.println(eserver);
  String ssidEeprom = esid.c_str();
  if ( ssidEeprom.length() > 1 ) {
    /*
    Serial.print(">SSID: ");
    Serial.print(esid.c_str());
    Serial.print(">PASS: ");
    Serial.print(epass.c_str());
    */
    wifi_ssid=esid.c_str();
    wifi_password=epass.c_str();
    node_server=eserver.c_str();
  }
}


void initWiFi(void) {
  Serial.println(F("Start WiFi"));
  WiFi.persistent(true);
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
    while (WiFi.status() != WL_CONNECTED && _intents <= MAX_WIFI_INTENTS) {
      delay(500);
      Serial.print(F("."));
      _intents++;
    }
  }
  Serial.println();
  if (_intents > MAX_WIFI_INTENTS) {
    Serial.print(F("Failed to connect to "));
    Serial.println(wifi_ssid);
  } else {
    Serial.print(F("Connected to "));
    Serial.println(wifi_ssid);
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("Mac addresss: "));
    Serial.println(WiFi.macAddress());
  }  
}

void invokeURL(void){
  Serial.println( "Alert Triggered");
  const int API_TIMEOUT = 1000;
    /*
   * MQTT pub goes here
   */
  String payload = "{\"message\":\"Warning! alarm triggered - in Door\",\"alert\":\"100\",\"location\":\"Door\",\"value\":100}";
  mqtt = new Adafruit_MQTT_Client(&client, node_server.c_str(), AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
  sensor = new Adafruit_MQTT_Publish(mqtt, AIO_USERNAME "iot/test1");
  MQTT_connect();
  Serial.print(F("\nSending sensor val "));
  Serial.print(payload);
  Serial.print("...");
  if (!sensor->publish((char*) payload.c_str())) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  delay(500);
  
}


void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt->connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt->connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt->connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt->disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

void setup(void){
  pinMode(PushButton, INPUT);
  EEPROM.begin(512);
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting Sketch...");
  Push_button_state = digitalRead(PushButton);
  Serial.println(Push_button_state);
  if ( Push_button_state == HIGH )
  { 
    Serial.println("Config button pressed , starting webserver...");
    delay(500);
  //ESP32 As access point IP: 192.168.4.1
    WiFi.mode(WIFI_AP); //Access Point mode
    WiFi.softAP("ESPWebServer", "12345678");    //Password length minimum 8 char
    delay(200);
    IPAddress Ip(192, 168, 5, 1);
    IPAddress NMmask(255, 255, 255, 0);
    WiFi.softAPConfig(Ip, Ip, NMmask);
    server.on("/", handleRoot);      //This is display page
    server.on("/setting", setting);
    server.on("/read", readEEPROM);
    server.begin();                  //Start server
    Serial.println("HTTP server started");
    Serial.println(WiFi.softAPIP());
  }else{
    Serial.println("Arming MPU & ESP and putting them to deep sleep ");
    getCreds();
  }
}

//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void loop(void){
  if ( Push_button_state == HIGH ){
    server.handleClient();
    delay(1);
  }else{
    Serial.println("alert");
    initWiFi();
    invokeURL();
    setupESPMPU();
  }
}
