#include "esp_now.h"
#include "WiFi.h"

int tmp;

#define pwmA 2
#define leftMotorPin1 13
#define leftMotorPin2 12  

typedef struct{
  int b;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len){
  memcpy(&myData, incomingData, sizeof(myData));
  tmp = myData.b;
}


void setup() {

  pinMode(pwmA, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK){
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {

  Serial.println(tmp);

  if (tmp == 96) {

    digitalWrite(pwmA, HIGH);
    moveForward();
    delay(13000);
    stopMotors();
    while(1){}
  }

  else {
    digitalWrite(pwmA, LOW);
    stopMotors();
  }

}

void moveForward() {
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  Serial.println("CHALU");
}

void stopMotors() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  Serial.println("BAND");
}
