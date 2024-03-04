#include <esp_now.h>
#include <WiFi.h>

const int trigPin = 4;
const int echoPin = 5;
long duration;
int distance;

int tmp = 0;

uint8_t broadcastAddress1[] = {0x08, 0xD1, 0xF9, 0x29, 0xE1, 0xC4};
uint8_t broadcastAddress2[] = {0x08, 0xD1, 0xF9, 0x26, 0x50, 0x18};

#define sensorPin1 34 
#define sensorPin2 15
#define sensorPin3 33
#define sensorPin4 32
#define sensorPin5 35

#define pwmA 2
#define leftMotorPin1 13
#define leftMotorPin2 12

#define pwmB 4
#define rightMotorPin1 14
#define rightMotorPin2 27

const int threshold = 4000;

typedef struct {
  int b;
} struct_message;

struct_message datarecv;
struct_message datasend;

esp_now_peer_info_t peerInfo1;
esp_now_peer_info_t peerInfo2;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&datarecv, incomingData, sizeof(datarecv));
  tmp = datarecv.b;
}

void setup() {
  Serial.begin(115200);

  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);
  pinMode(sensorPin3, INPUT);
  pinMode(sensorPin4, INPUT);
  pinMode(sensorPin5, INPUT);

  pinMode(pwmA, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);

  WiFi.mode(WIFI_STA);

  // ESP-NOW initialization for ESP8266 (Master)
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_register_recv_cb(OnDataRecv);

  // Peer setup for ESP32 (Slave)
  memcpy(peerInfo1.peer_addr, broadcastAddress1, 6);
  peerInfo1.channel = 0;  
  peerInfo1.encrypt = false;

  memcpy(peerInfo2.peer_addr, broadcastAddress2, 6);
  peerInfo2.channel = 0;  
  peerInfo2.encrypt = false;

  if (esp_now_add_peer(&peerInfo1) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  if (esp_now_add_peer(&peerInfo2) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  datasend.b = 96;
  Serial.println(tmp);

  int sensorValues[5];
  sensorValues[0] = analogRead(sensorPin1);
  sensorValues[1] = analogRead(sensorPin2);
  sensorValues[2] = analogRead(sensorPin3);
  sensorValues[3] = analogRead(sensorPin4);
  sensorValues[4] = analogRead(sensorPin5);

  if (tmp == 69) {

    uv();

    if (sensorValues[2] > threshold){
      digitalWrite(pwmA, HIGH);
      digitalWrite(pwmB, HIGH);
      moveForward();
      Serial.println("Front");
    }

    else if (sensorValues[0] > threshold){
      digitalWrite(pwmA, HIGH);
      digitalWrite(pwmB, HIGH);
      turnLeft();
      Serial.println("Left");
    // delay(100);
    }
    else if (sensorValues[4] > threshold){
      digitalWrite(pwmA, HIGH);
      digitalWrite(pwmB, HIGH);
      turnRight();
      Serial.println("Right");
    // delay(100);
    }

    else if(sensorValues[0] < threshold && sensorValues[2] < threshold && sensorValues[4] < threshold || distance <= 10){
      digitalWrite(pwmA, LOW);
      digitalWrite(pwmB, LOW);
      stopMotors();
      Serial.println("Stop");
      sendDataToESP32();
      while(1){}
    }
  }

  else {
    digitalWrite(pwmA, LOW);
    digitalWrite(pwmB, LOW);
    stopMotors();
  }

  uv();

  if (distance <= 9) {
    sendDataToESP32();
    while(1){}
  }
}

void sendDataToESP32() {
  esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *) &datasend, sizeof(datasend));
    
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

    result = esp_now_send(broadcastAddress2, (uint8_t *) &datasend, sizeof(datasend));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
}

void uv() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
}

void moveForward() {
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}

void stopMotors() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
}

void turnLeft() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}

void turnRight() {
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
}