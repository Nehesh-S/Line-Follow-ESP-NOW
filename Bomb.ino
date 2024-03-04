#include <ESP8266WiFi.h>
#include <espnow.h>
#define BUTTON_PIN D1
int bvalue;
uint8_t broadcastAddress1[] = {0x08, 0xD1, 0xF9, 0x37, 0x7D, 0x30};

typedef struct struct_message {
  int b;
} struct_message;
struct_message myData;
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}
 
void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}
 
void loop() {
  bvalue = digitalRead(BUTTON_PIN);
  Serial.print(bvalue);
  if (bvalue == 0)
  {
    Serial.println("");
    myData.b = 69;
    esp_now_send(broadcastAddress1, (uint8_t *) &myData, sizeof(myData));
    delay(500);
  }
}