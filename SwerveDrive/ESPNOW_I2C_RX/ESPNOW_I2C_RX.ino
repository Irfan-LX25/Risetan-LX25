#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>

TaskHandle_t mainTask;
TaskHandle_t I2CTask;
void mainApplicationTask(void *parameter); 
void I2CCommunicationSendTask(void *parameter);

int SLAVE_ADDR_MODULE[4] = {10, 11, 12, 13};
int Drive_SP[4];
int Steer_SP[4];
short int lx;
short int ly;
short int rx;

#define pinBuzzer 25
#define EEPROM_SIZE 512
#define addsDegree 0

int8_t zeroDegree;
int8_t eepromVal;

typedef struct struct_message {
    bool stat[15];
    int joyData[4];
} struct_message;

struct_message recvData;
bool espnow_connected = false;
unsigned long lastDataTime = 0;
const unsigned long TIMEOUT_MS = 1000;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void setupESP_NOW();

// HAPUS variabel global ini karena menyebabkan konflik
// int speedVal[4];
// int degreeVal[4]; 
String dataModule[4];

void readRecvData(){
  lx = map(recvData.joyData[0], -4095, 4095, -128, 127);
  ly = map(recvData.joyData[1], -4095, 4095, -128, 127);
  rx = map(recvData.joyData[2], -4095, 4095, -128, 127);
  
  // Tambahkan dead zone lebih besar
  if(abs(lx) < 15) lx = 0;
  if(abs(ly) < 15) ly = 0;
  if(abs(rx) < 15) rx = 0;
}

void addValue(int degreeVal[4], int speedVal[4])
{
  for (int i = 0; i < 4; i++){
    Steer_SP[i] = degreeVal[i];
    Drive_SP[i] = speedVal[i];
  }
}

void onConnect()
{
  for (int i = 0; i < 3; i++) { 
    digitalWrite(pinBuzzer, HIGH);
    delay(100);
    digitalWrite(pinBuzzer, LOW);
    delay(100);
  }
  Serial.println("ESP-NOW Connected.");
}

void onDisconnect()
{
  for (int i = 0; i < 5; i++){
    digitalWrite(pinBuzzer, HIGH);
    delay(50);
    digitalWrite(pinBuzzer, LOW);
    delay(50);
  }
  Serial.println("ESP-NOW Disconnected.");
  for (int i = 0; i < 4; i++){
    Drive_SP[i] = 0;
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&recvData, incomingData, sizeof(recvData));
  lastDataTime = millis();
  
  if (!espnow_connected) {
    espnow_connected = true;
    onConnect();
  }
}

void setupESP_NOW() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW Ready");
}

void checkConnection() {
  if (espnow_connected && (millis() - lastDataTime > TIMEOUT_MS)) {
    espnow_connected = false;
    onDisconnect();
  }
}

void setup() {
  Serial.begin(115200);  
  Serial.println("Starting ESP-NOW Receiver...");
  Serial.println("\nI2C Scanner");

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  
  pinMode(pinBuzzer, OUTPUT);
  setupESP_NOW();
  
  zeroDegree = EEPROM.read(addsDegree);
  for (int i = 0; i < 4; i++){
    Steer_SP[i] = zeroDegree;
    Drive_SP[i] = 0;  
  }
  
  xTaskCreatePinnedToCore(
    I2CCommunicationSendTask,
    "I2CCommunicationSendTask",  
    10000,
    NULL,
    1,
    &I2CTask,
    0
  );
  
  xTaskCreatePinnedToCore(
    mainApplicationTask,
    "mainApplicationTask",
    10000,
    NULL,
    1,
    &mainTask,
    1
  );
  
  Serial.println("Setup completed");
}

void loop() {
  checkConnection();
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

void mainApplicationTask(void *parameter){
  pinMode(2, OUTPUT);
  
  unsigned long lastRawDebug = 0;
  
  for(;;) {
    if (espnow_connected) {
      readRecvData();
      bool btnUP = recvData.stat[4];   
      bool btnDOWN = recvData.stat[6];  
      bool btnLEFT = recvData.stat[5];  
      bool btnRIGHT = recvData.stat[7]; 

      if (millis() - lastRawDebug > 2000) {
        lastRawDebug = millis();
        Serial.println("=== RAW DATA DEBUG ===");
        Serial.printf("Raw Joy: [0]:%d, [1]:%d, [2]:%d, [3]:%d\n", 
                     recvData.joyData[0], recvData.joyData[1], 
                     recvData.joyData[2], recvData.joyData[3]);
        Serial.printf("Mapped: LX:%d, LY:%d, RX:%d\n", lx, ly, rx);
        Serial.printf("Buttons: ");
        for (int i = 0; i < 15; i++) {
          Serial.printf("%d", recvData.stat[i]);
          if (i < 14) Serial.print(",");
        }
        Serial.println();
        Serial.printf("Important BTNs: UP[4]:%d, LEFT[5]:%d, DOWN[6]:%d, RIGHT[7]:%d\n",
                     recvData.stat[4], recvData.stat[5], recvData.stat[6], recvData.stat[7]);
        Serial.println("=====================");
      }

      bool btnUP_active = !recvData.stat[4];   
      bool btnDOWN_active = !recvData.stat[6];  
      bool btnLEFT_active = !recvData.stat[5]; 
      bool btnRIGHT_active = !recvData.stat[7]; 

      // Reset values
      for (int i = 0; i < 4; i++) {
        Drive_SP[i] = 0;
        Steer_SP[i] = zeroDegree;
      }

      bool commandActive = false;
      // Forward - sesuai program asli: ly <= -30 && (ly >= -40 && lx <= 40)
      if ((ly <= -30 && (ly >= -40 && lx <= 40)) || btnUP_active) {
        if (!commandActive) {
          Serial.println(">>> FORWARD - Speed 100, Degree 0");
          int degreeVal[4] = {0, 0, 0, 0};
          int speedVal[4] = {100, 100, 100, 100};
          addValue(degreeVal, speedVal);
          commandActive = true;
        }
      }
      // Backward - sesuai program asli: ly >= 30 && (lx >= -50 && lx <=50)
      else if ((ly >= 30 && (lx >= -50 && lx <=50)) || btnDOWN_active) {
        if (!commandActive) {
          Serial.println(">>> BACKWARD - Speed 100, Degree 180");
          int degreeVal[4] = {180, 180, 180, 180};
          int speedVal[4] = {100, 100, 100, 100};
          addValue(degreeVal, speedVal);
          commandActive = true;
        }
      }
      // Right - sesuai program asli: ly >= 30 && (lx >= -50 && lx <=50)  
      else if ((ly >= 30 && (lx >= -50 && lx <=50)) || btnRIGHT_active) {
        if (!commandActive) {
          Serial.println(">>> RIGHT - Speed 100, Degree 90");
          int degreeVal[4] = {90, 90, 90, 90};
          int speedVal[4] = {100, 100, 100, 100};
          addValue(degreeVal, speedVal);
          commandActive = true;
        }
      }
      // Left - sesuai program asli: ly <= -30 && (lx >= -50 && lx <=50)
      else if ((ly <= -30 && (lx >= -50 && lx <=50)) || btnLEFT_active) {
        if (!commandActive) {
          Serial.println(">>> LEFT - Speed 100, Degree -90");
          int degreeVal[4] = {-90, -90, -90, -90};
          int speedVal[4] = {100, 100, 100, 100};
          addValue(degreeVal, speedVal);
          commandActive = true;
        }
      }
      // Forward Right - sesuai program asli: ly <= -30 && (lx >= 31 && lx <=127)
      else if (ly <= -30 && (lx >= 31 && lx <=127)) {
        if (!commandActive) {
          Serial.println(">>> FORWARD RIGHT - Speed 100, Degree 45");
          int degreeVal[4] = {45, 45, 45, 45};
          int speedVal[4] = {100, 100, 100, 100};
          addValue(degreeVal, speedVal);
          commandActive = true;
        }
      }
      // Backward Right - sesuai program asli: ly >= 30 && (lx >= 31 && lx <=127)
      else if (ly >= 30 && (lx >= 31 && lx <=127)) {
        if (!commandActive) {
          Serial.println(">>> BACKWARD RIGHT - Speed 100, Degree 135");
          int degreeVal[4] = {135, 135, 135, 135};
          int speedVal[4] = {100, 100, 100, 100};
          addValue(degreeVal, speedVal);
          commandActive = true;
        }
      }
      // Forward Left - sesuai program asli: ly <= -50 && (lx <= -36 && lx >=-128)
      else if (ly <= -50 && (lx <= -36 && lx >=-128)) {
        if (!commandActive) {
          Serial.println(">>> FORWARD LEFT - Speed 100, Degree -45");
          int degreeVal[4] = {-45, -45, -45, -45};
          int speedVal[4] = {100, 100, 100, 100};
          addValue(degreeVal, speedVal);
          commandActive = true;
        }
      }
      // Backward Left - sesuai program asli: ly >= 35 && (lx <= -36 && lx <=-128)
      else if (ly >= 35 && (lx <= -36 && lx <=-128)) {
        if (!commandActive) {
          Serial.println(">>> BACKWARD LEFT - Speed 100, Degree -135");
          int degreeVal[4] = {-135, -135, -135, -135};
          int speedVal[4] = {100, 100, 100, 100};
          addValue(degreeVal, speedVal);
          commandActive = true;
        }
      }
      // Rotate Left - sesuai program asli: rx <= -40
      else if (rx <= -40) {
        if (!commandActive) {
          Serial.println(">>> ROTATE LEFT - Speed 200");
          int degreeVal[4] = {-45, -135, 45, 135};
          int speedVal[4] = {200, 200, 200, 200};
          addValue(degreeVal, speedVal);
          commandActive = true;
        }
      }
      // Rotate Right - sesuai program asli: rx >= 40
      else if (rx >= 40) {
        if (!commandActive) {
          Serial.println(">>> ROTATE RIGHT - Speed 200");
          int degreeVal[4] = {135, 45, -135, -45};
          int speedVal[4] = {200, 200, 200, 200};
          addValue(degreeVal, speedVal);
          commandActive = true;
        }
      }
      
      if (!commandActive) {
        static bool wasStopped = false;
        if (!wasStopped) {
          Serial.println("--- IDLE: No command active ---");
          wasStopped = true;
        }
        for (int i = 0; i < 4; i++) {
          Drive_SP[i] = 0;
        }
      } else {
        wcstold;
      }

      if (Steer_SP[0] > 0) {
        eepromVal = -1;
      }
      EEPROM.write(addsDegree, eepromVal);
      EEPROM.commit();
      
    } else {
    
      for (int i = 0; i < 4; i++) {
        Drive_SP[i] = 0;
      }
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Kurangi frekuensi update
  }
}


void I2CCommunicationSendTask(void *parameter){
  Wire.begin();
  Wire.setClock(100000);
  for (;;)
  {
    for (int i = 0; i < 4; i++)
    {
      String dataModule = String(Drive_SP[i]) + "#" + String(Steer_SP[i]);
      //int dataSize = dataModule.length();
      //char dataToSend[dataSize + 1];
     // dataModule.toCharArray(dataToSend, dataSize + 1); 
      Wire.beginTransmission(SLAVE_ADDR_MODULE[i]);
      Wire.write(dataModule.c_str());
      byte error = Wire.endTransmission();
      //Wire.write((const uint8_t *) dataToSend, dataSize);
      //Wire.endTransmission();
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
