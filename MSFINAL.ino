#include <esp_now.h>
#include <WiFi.h>

int tmp = 0;

uint8_t broadcastAddress1[] = {0x08, 0xD1, 0xF9, 0x29, 0xE1, 0xC4};
uint8_t broadcastAddress2[] = {0x08, 0xD1, 0xF9, 0x26, 0x50, 0x18};

#define led 18


#define sensorPin1 34 
#define sensorPin2 23
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
  pinMode(led, OUTPUT);


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

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_register_recv_cb(OnDataRecv);

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
  else{
    digitalWrite(led, HIGH);
  }

  if (esp_now_add_peer(&peerInfo2) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  else{
    digitalWrite(led, HIGH);
  }
}

void loop() {
  datasend.b = 96;
  linefollow();

}

void linefollow() {
  Serial.println(tmp);

  int sensorValues[5];
  sensorValues[0] = analogRead(sensorPin1);
  sensorValues[1] = analogRead(sensorPin2);
  sensorValues[2] = analogRead(sensorPin3);
  sensorValues[3] = analogRead(sensorPin4);
  sensorValues[4] = analogRead(sensorPin5);

  if (tmp == 69) {

    if (sensorValues[2] > threshold){
      digitalWrite(pwmA, HIGH);
      digitalWrite(pwmB, HIGH);
      moveForward();
      Serial.println("Front");
    }

    if (sensorValues[0] > threshold){
      digitalWrite(pwmA, HIGH);
      digitalWrite(pwmB, HIGH);
      turnLeft();
      Serial.println("Left");
    }

    if (sensorValues[4] > threshold){
      digitalWrite(pwmA, HIGH);
      digitalWrite(pwmB, HIGH);
      turnRight();
      Serial.println("Right");
    }

    if(sensorValues[0] < threshold && sensorValues[2] < threshold && sensorValues[4] < threshold){
      digitalWrite(pwmA, LOW);
      digitalWrite(pwmB, LOW);
      stopMotors();
      Serial.println("Stop");
    }
  }

  else {
    digitalWrite(pwmA, LOW);
    digitalWrite(pwmB, LOW);
    stopMotorsinit();
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
  sendDataToESP32();
  while(1){}
}

void turnLeft() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  delay(50);
}

void turnRight() {
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  delay(50);
}

void stopMotorsinit() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
}