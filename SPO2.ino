/*

PulseOxy 
--------
by jvoiges

- 
- added MQTT Support. Send Heartrate, SPO2, Temperature
- use with ESP32
- Arduino IDE 1.8

- Enhanced App based upon the Heartrate examples from Sparkfun
- https://github.com/sparkfun/MAX30105_Breakout
- uses MAX30105 Library


  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected

*/

// SPO2 Sensor
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

// OTA 
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include "ServerPages.h" 

// Storing connection Data
#include <EEPROM.h>
#define EEPROM_SIZE 512

// MQTT
#include <PubSubClient.h>


//Create semphore und Task
SemaphoreHandle_t sem = NULL;
TaskHandle_t Sensor_reading;
TaskHandle_t Communication;
void fkt_Communication( void * pvParameters );
void fkt_Sensor_reading( void * pvParameters );

int glob_BPM = 0;
int glob_beatAvg = 0;
int glob_spo2 = 0;
float glob_temp = 0.0;
uint32_t glob_ir = 0;
uint32_t glob_red = 0;
int glob_changedHeartBeat = 0;
int glob_changedIR = 0;
 

//////////////////////////////////////////////////////////
// Add your MQTT Broker IP address -- and Wifi connection
char mqtt_server[128] = "";  // ="IP_ADR";
char *apSID = "PulseOxy";
char ssid[128] = ""; // "YourSID";
char password[128] = ""; //  "YouPW";
const byte energySaving = 0;
//////////////////////////////////////////////////////////

static bool hasIoTHub = false;
const char* host = "esp32";

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];
int value = 0;

 int iCount = 0;
 
// OTA
WebServer server(80);
int isWifiConnected = 0;

MAX30105 particleSensor;

// HeartBeat nach Zeitabstand
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
int  beatAvg = 0;
long lastbeatAvg = 0;
int  lastSpo2 = 0;

float beatsPerMinute;

#define _VAL_BUF_SIZE 4
int _bufSPO2[_VAL_BUF_SIZE];
int _bufHR[_VAL_BUF_SIZE];
int _bufTemp[_VAL_BUF_SIZE];

//////////

// SPO2 nach Durchschnittsberechnung / O2 nicht nehmn
#define MAX_BRIGHTNESS 255

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data


int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = LED_BUILTIN; // 13; //Blinks with each data read

void setupSPO2 () {

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  // while (Serial.available() == 0) ; //wait until user presses a key
  sleep (1);
  // Serial.read();

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  
}

void setupHeartbeat () {
  // Nur HeartRate
  // Initialize sensor

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  // while (Serial.available() == 0) ; //wait until user presses a key
  delay (10);
  // Serial.read();

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}


void handleSaveData () {
    String strSID;
    String strPW;
    String emptyStr = "                                      ";
    String strMqtt;

    strSID = server.arg("SID");
    strPW = server.arg("wifipwd");
    strMqtt = server.arg("MQTTSERVER");
    
    EEPROM.writeString(0,emptyStr);
    EEPROM.writeString(128,emptyStr);
    EEPROM.writeString(256,emptyStr);

    Serial.print("MQTT: ");
    Serial.println(strMqtt);    
    Serial.print("SID: ");
    Serial.println(strSID);
    Serial.print("PW: ");
    Serial.println(strPW);   
    EEPROM.writeString(0, strSID);
    EEPROM.writeString(128, strPW);
    EEPROM.writeString(256, strMqtt);
    EEPROM.commit();
    server.send(200, "text/plain", "Saved");  
    delay (200);
    ESP.restart();  
}

void handleNotFound() {
  digitalWrite(readLED, 1);
  Serial.println(F("webserver Page  not found"));
  
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);

  Serial.println(message);
  digitalWrite(readLED, 0);
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      //digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      // digitalWrite(ledPin, LOW);
    }
  }
}

void setup()
{
  String strSID;
  String strPW;
  String strMqtt;
  
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  delay(10);
  Serial.println("Start ------ ------ ------ ");
  sem =  xSemaphoreCreateMutex();
 
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  strSID = EEPROM.readString(0);
  strPW =  EEPROM.readString(128);
  strMqtt =  EEPROM.readString(256);
  strSID.toCharArray(ssid, strSID.length()+1);
  strPW.toCharArray(password, strPW.length()+1);
  strMqtt.toCharArray(mqtt_server, strMqtt.length()+1);

  Serial.println("Stored:");
  Serial.print("SID:"); Serial.println(strSID);
  Serial.print("PW:"); Serial.println(strPW);
  Serial.print("MQTT:"); Serial.println(strMqtt);
  
  // Connect to WiFi network
  WiFi.begin(ssid, password);
  Serial.println("try wifi connect");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED && iCount < 20) {
    delay(500);
    Serial.print(".");
    iCount++;
  }
  Serial.println("end try"); 
  
  if (WiFi.status() != WL_CONNECTED) {
    // create own access point
    WiFi.softAP(apSID);
  
    IPAddress IP = WiFi.softAPIP();
    Serial.print("acting as AP --- IP address: ");
    Serial.println(IP);  
    // WiFi.printDiag(Serial);
    isWifiConnected = 0;  
  } else {
    isWifiConnected = 1;
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // wifi ok check mqtt
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
  
    if (client.connect("ESP8266Client")) {
          Serial.println("connected");
          client.subscribe("esp32/output");
    }      
  }

  //use mdns for host name resolution
  if (!MDNS.begin(host)) {
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  
  // return index page which is stored in serverIndex 
  server.onNotFound(handleNotFound);
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  server.on("/savewifi", handleSaveData );
      
  // handling uploading firmware file 
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      // flashing firmware to ESP
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
  
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  //setupSPO2();
  setupHeartbeat();

  
  Serial.println(F("Part ID:"));
  Serial.println(particleSensor.readPartID(), DEC);
  Serial.println(F("Rev ID:"));
  Serial.println(particleSensor.getRevisionID());
  Serial.println(F("Temp:"));
  Serial.println(particleSensor.readTemperature());

  //////////////
  //Task Handling
  xTaskCreatePinnedToCore(
                    fkt_Sensor_reading,   /* Task function. */
                    "Sensor_reading",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Sensor_reading,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  xTaskCreatePinnedToCore(
                    fkt_Communication,   /* Task function. */
                    "Communication",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Communication,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
  }

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      // delay(5000);
    }
  }
}

void mqtt_sendDiff (long red, long ir) {
  long a = 0;
  char sDiff[128];
    
  if (ir < 50000) {
    a = 0; 
  } else {
    a = red - ir;
    a = abs(a);
  }
  snprintf (sDiff, sizeof(sDiff), "%8d", a);
  if (!energySaving) { 
    client.publish("Health/diff",sDiff);
    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
  }
}

void mqtt_sendIR (long ir) {
  long a = 0;
  char sIR[128];
    
  if (ir < 50000) {
    a = 0; 
  } else {
    a = ir;
    a = abs(a);
  }
  snprintf (sIR, sizeof(sIR), "%8d", a);
  if (!energySaving) { 
    client.publish("Health/ir",sIR);
    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read    
  }
}

void mqtt_sendSPO2 (int spo2) {
  char sSPO2[128];

  if (spo2 != lastSpo2) {  
    snprintf (sSPO2, sizeof(sSPO2), "%d", spo2);
    client.publish("Health/SPO2",sSPO2);
    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
  }
  lastSpo2 = spo2;
}

void mqtt_sendTemperature (float fTemp) {
  char sTemp[128];
  
  if ( 0 < fTemp < 50) {
    snprintf (sTemp, sizeof(sTemp), "%2.2f",fTemp);
  } else
    snprintf (sTemp, sizeof(sTemp), "0");
        
  client.publish("Health/Temp",sTemp);
  digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

  // Serial.print(sTemp);Serial.print(" ");
}

void mqtt_sendHeartbeat (int b) {
  char sHR[128];
    
  if ( 10 < b < 250) {
    snprintf (sHR, sizeof(sHR), "%d", b);
  } else {
    snprintf (sHR, sizeof(sHR),"%d", "0"); 
  }
  
  //if (lastbeatAvg != b) { 
    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
    client.publish("Health/HR",sHR);
  //}
  lastbeatAvg = b;  
}

void getSamples (int iStart, int iEnd) {
  
  long delta = 0;
  byte i = 0;
    
  //read the first 100 samples, and determine the signal range
  for (i = iStart ; i < iEnd ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();

   if (irBuffer[i] > 50000) {

      if (checkForBeat(irBuffer[i]) == true)
      {
        //We sensed a beat!
        delta = millis() - lastBeat;
        lastBeat = millis();
    
        beatsPerMinute = 60 / (delta / 1000.0);
        
        if (beatsPerMinute < 255 && beatsPerMinute >= 0)
        {
          rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
          rateSpot %= RATE_SIZE; //Wrap variable
    
          //Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }           
      }
    } else {
       beatsPerMinute = beatAvg = spo2 = heartRate = 0;
//       xSemaphoreTake(sem, portMAX_DELAY);  
       glob_BPM = glob_beatAvg = glob_spo2 = 0; glob_changedHeartBeat = 1;
//       xSemaphoreGive(sem);  
    } 

//    xSemaphoreTake(sem, portMAX_DELAY);
    glob_ir = irBuffer[i];
    glob_red = redBuffer[i];
    glob_changedIR = 1;
//    xSemaphoreGive(sem); 
           
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }
  digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
  
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, iEnd, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  if ((10 < spo2 <= 100) && validSPO2 == 1) {        
  } else {
    spo2 = 0;
  }
  float t =  particleSensor.readTemperature();  
  // xSemaphoreTake(sem, portMAX_DELAY);
  glob_BPM = beatsPerMinute;
  glob_beatAvg = beatAvg;
  glob_spo2 = spo2;
  glob_temp = t;
  glob_changedHeartBeat = 1;
  // xSemaphoreGive(sem); 

  // Serial.print(heartRate);Serial.print(" ");Serial.println(spo2);
}

void getSPO2_HeartBeat()
{

  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  
  //getSamples (0,bufferLength);
  
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  iCount = 0;
  while (iCount++ < 12) // 16 Sekunden SPO2
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top

    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }
    getSamples (75,100);
  }
}

void fkt_Communication( void * pvParameters ){
  int iBPM, iBeatAvg, iSpo2;
  float fT;
  int iIr, iRed, i, ii;
  
  Serial.print("fkt_Communication running on core ");
  Serial.println(xPortGetCoreID());

  while (1) {       
    // Serial.print("fkt_Communication running on core ");
    // Serial.println(xPortGetCoreID());
 
      server.handleClient();
//      if (!client.connected()) {
//        reconnect();
//      }   
//      client.loop();        
      
//    xSemaphoreTake(sem, portMAX_DELAY);  
      i = glob_changedIR; ii = glob_changedHeartBeat;
      iRed = glob_red, iIr = glob_ir, iSpo2 = glob_spo2, iBeatAvg = glob_beatAvg, fT = glob_temp;
//    xSemaphoreGive(sem);
        
    if (i) {
      // mqtt_sendDiff(glob_red, glob_ir);  
      mqtt_sendIR ( iIr);
      // Serial.print("IR="); Serial.print(iIr);
      // Serial.print(", RED="); Serial.print(iRed);
      Serial.println(iRed); 
    } else {
      // Serial.print("IR_"); Serial.print(iIr);
      // Serial.print(", RED_"); Serial.print(iRed);               
    }
           
    if (ii) {
      mqtt_sendSPO2 (iSpo2);
      mqtt_sendHeartbeat(iBeatAvg);
      mqtt_sendTemperature (fT);
      
      // Serial.print(", BPM="); Serial.print(beatsPerMinute);
      // Serial.print(", Avg BPM="); Serial.print(iBeatAvg);
      // Serial.print(",-- spo2="); Serial.print(iSpo2);
      // Serial.print(", validSPO2="); Serial.print(validSPO2);
      // Serial.print(", heartRate="); Serial.print(heartRate);
      // Serial.print(", validHeartRate="); Serial.println(validHeartRate);
    } else {
      // Serial.print(", BPM_"); Serial.print(beatsPerMinute);
      // Serial.print(", Avg BPM_"); Serial.print(iBeatAvg);
      // Serial.print(",-- spo2_"); Serial.print(iSpo2);
      // Serial.print(", validSPO2_"); Serial.print(validSPO2);
      // Serial.print(", heartRate_"); Serial.print(heartRate);
      // Serial.print(", validHeartRate_"); Serial.println(validHeartRate);
      
    }
    glob_changedIR = glob_changedHeartBeat = 0;
    
    vTaskDelay(10);
  }
}

void fkt_Sensor_reading( void * pvParameters ){
  Serial.print("fkt_Sensor_reading running on core ");
  Serial.println(xPortGetCoreID());  
  
  while (1) {
    getSPO2_HeartBeat ();  
    // Serial.print("fkt_Sensor_reading running on core ");
    // Serial.println(xPortGetCoreID());  
    vTaskDelay(10);
  }
}
  
void loop()
{
  Serial.print("MAIN running on core ");
  Serial.println(xPortGetCoreID());
  // MAIN not exe
  vTaskSuspend(NULL);  
}
  
