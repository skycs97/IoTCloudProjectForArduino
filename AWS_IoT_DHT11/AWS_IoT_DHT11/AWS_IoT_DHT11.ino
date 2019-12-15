/*
  AWS IoT WiFi

  This sketch securely connects to an AWS IoT using MQTT over WiFi.
  It uses a private key stored in the ATECC508A and a public
  certificate for SSL/TLS authetication.

  It publishes a message every 5 seconds to arduino/outgoing
  topic and subscribes to messages on the arduino/incoming
  topic.

  The circuit:
  - Arduino MKR WiFi 1010 or MKR1000

  The following tutorial on Arduino Project Hub can be used
  to setup your AWS account and the MKR board:

  https://create.arduino.cc/projecthub/132016/securely-connecting-an-arduino-mkr-wifi-1010-to-aws-iot-core-a9f365

  This example code is in the public domain.
*/
#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>

#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>

#include "arduino_secrets.h"

#include "DHT.h"
#define DHTPIN 2        // 2번 핀을 온습도센서에 연결
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);

Servo myservo;
#define motor1Pin1 3
#define motor1Pin2 4     //3,4번에 워터모터연결
#define soilPin A1
#define sunlightPin A3
#define servoPin 5

#include <ArduinoJson.h>

/////// Enter your sensitive data in arduino_secrets.h
const char ssid[]        = SECRET_SSID;
const char pass[]        = SECRET_PASS;
const char broker[]      = SECRET_BROKER;
const char* certificate  = SECRET_CERTIFICATE;

WiFiClient    wifiClient;            // Used for the TCP socket connection
BearSSLClient sslClient(wifiClient); // Used for SSL/TLS connection, integrates with ECC508
MqttClient    mqttClient(sslClient);

unsigned long lastMillis = 0;
char water_val[10] = "OFF"; // 워터모터 ON,OFF 넣는 변수
char servo_val[10] = "CLOSE"; // 서보모터 OPEN,HALFOPEN,CLOSE 넣는 변수
float water_stand = 30.0f;  // 워터모터 기준값 변수

bool watermotor = false;
int sunvisor = 0;


void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!ECCX08.begin()) {
    Serial.println("No ECCX08 present!");
    while (1);
  }

  // Set a callback to get the current time
  // used to validate the servers certificate
  ArduinoBearSSL.onGetTime(getTime);

  // Set the ECCX08 slot to use for the private key
  // and the accompanying public certificate for it
  sslClient.setEccSlot(0, certificate);

  // Optional, set the client id used for MQTT,
  // each device that is connected to the broker
  // must have a unique client id. The MQTTClient will generate
  // a client id for you based on the millis() value if not set
  //
  // mqttClient.setId("clientId");

  // Set the message callback, this function is
  // called when the MQTTClient receives a message
  mqttClient.onMessage(onMessageReceived);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  myservo.attach(servoPin);
  dht.begin();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!mqttClient.connected()) {
    // MQTT client is disconnected, connect
    connectMQTT();
  }
  
  // poll for new MQTT messages and send keep alives
  mqttClient.poll();
  
  float soilmoisture = analogRead(soilPin);  // 습도센서 1번 연결
  soilmoisture = map(soilmoisture,0,1023,100,0);
  soilmoisture = constrain(soilmoisture,0,100); // 값을 0부터 100사이의 값으로
  
   //워터 모터가 ON 이면서 수분값이 30 이상이면 작동을 멈춤
  if (soilmoisture >= water_stand ) {
      watermotor = false;
  }
  //워터 모터가 ON 이면서 수분값이 30 미만이면 작동을 계속함
  else if (soilmoisture < water_stand ) {
      watermotor = true;
  }
  pump(watermotor);

  // publish a message roughly every 10 seconds.
  if (millis() - lastMillis > 10000) {
    lastMillis = millis();
    char payload[512];
    getDeviceStatus(payload);
    sendMessage(payload);
  }
}

unsigned long getTime() {
  // get the current time from the WiFi module  
  return WiFi.getTime();
}

void connectWiFi() {
  Serial.print("Attempting to connect to SSID: ");
  Serial.print(ssid);
  Serial.print(" ");

  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println();

  Serial.println("You're connected to the network");
  Serial.println();
}

void connectMQTT() {
  Serial.print("Attempting to MQTT broker: ");
  Serial.print(broker);
  Serial.println(" ");

  while (!mqttClient.connect(broker, 8883)) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println();

  Serial.println("You're connected to the MQTT broker");
  Serial.println();

  // delta 주제 구독
  mqttClient.subscribe("$aws/things/MyMKRWiFi1010/shadow/update/delta");
}

void getDeviceStatus(char* payload) {  // 이 함수도 10초마다 갱신
  // Read temperature as Celsius (the default)
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  float soilmoisture = analogRead(soilPin);  // 습도센서 1번 연결
  soilmoisture = map(soilmoisture,0,1023,100,0);
  soilmoisture = constrain(soilmoisture,0,100); // 값을 0부터 100사이의 값으로
  
  float sunlight = analogRead(sunlightPin);    // 조도센서 3번 연결
  sunlight = map(sunlight,0,1023,100,0);
  sunlight = constrain(sunlight,0,100);  // 값을 0부터 100사이의 값으로

 
  // make payload for the device update topic ($aws/things/MyMKRWiFi1010/shadow/update)
  sprintf(payload,"{\"state\":{\"reported\":{\"temperature\":\"%f\",\"humidity\":\"%f\",\"soilMoisture\":\"%f\",\"sunlight\":\"%f\",\"watermotor\":\"%s\",\"sunvisor\":\"%s\"}}}",temperature,humidity,soilmoisture,sunlight,water_val,servo_val);
}

void sendMessage(char* payload) {
  char TOPIC_NAME[]= "$aws/things/MyMKRWiFi1010/shadow/update";
  
  Serial.print("Publishing send message:");
  Serial.println(payload);
  mqttClient.beginMessage(TOPIC_NAME);
  mqttClient.print(payload);
  mqttClient.endMessage();
}


void onMessageReceived(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // store the message received to the buffer
  char buffer[512] ;
  int count=0;
  while (mqttClient.available()) {
     buffer[count++] = (char)mqttClient.read();
  }
  buffer[count]='\0'; // 버퍼의 마지막에 null 캐릭터 삽입
  Serial.println(buffer);
  Serial.println();
  
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, buffer);
  JsonObject root = doc.as<JsonObject>();
  JsonObject state = root["state"]; 
  const char* servo_delta = state["sunvisor"];

  char payload[512];
    if (strcmp(servo_delta,"OPEN")==0) {
      strcpy(servo_val, "OPEN");
      sprintf(payload,"{\"state\":{\"reported\":{\"sunvisor\":\"%s\"}}}",servo_val);      
      sendMessage(payload);    
      servo(2);
    }
    else if (strcmp(servo_delta,"HALFOPEN")==0) {
      strcpy(servo_val, "HALFOPEN");
      sprintf(payload,"{\"state\":{\"reported\":{\"sunvisor\":\"%s\"}}}",servo_val);
      sendMessage(payload);
      servo(1);
    }
    else if (strcmp(servo_delta,"CLOSE")==0) {
      strcpy(servo_val, "CLOSE");
      sprintf(payload,"{\"state\":{\"reported\":{\"sunvisor\":\"%s\"}}}",servo_val);
      sendMessage(payload);
      servo(0);
  }
}

void pump(bool flag)  // 펌프 돌아가는 함수
{
  if(flag){
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    strcpy(water_val, "ON");
  }
  else{
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    strcpy(water_val, "OFF");
  }
}

void servo(int state){
  if(state == 0){
    myservo.write(0);
  }
  else if(state == 1){
    myservo.write(90);
  }
  else if(state == 2){
    myservo.write(180);
  }
}
