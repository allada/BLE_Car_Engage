#define txCalibratedPower 0xBD
//#include <AltSoftSerial.h>
//AltSoftSerial Serial1;
#include <stdint.h>
#include <Timer.h>
#include <Wire.h>
bool bleReady = false;

//SoftwareSerial mySerial(10, 11); // RX, TX

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
Timer t;
#define mySerial Serial
#define leftLockDir 4
#define rightLockDir 5
#define execLock 6
#define lightsPin 7
#define enginePin 3

uint8_t lightPtr = 0;
int lightAry[] = {
  -1, -1, -1, -1,
  -1, -1, -1, -1,
  -1, -1, -1, -1,
  -1, -1, -1, -1
  -1, -1, -1, -1,
  -1, -1, -1, -1,
  -1, -1, -1, -1,
  -1, -1, -1, -1,
  -1, -1, -1, -1,
  -1, -1, -1, -1,
  -1, -1, -1, -1
};

bool isScanning = false;
bool deviceFound = false;
bool carOn = false;
uint8_t deviceNotFoundCount = 0;
const uint8_t validDevice[] = {0xF4, 0xF8, 0x68, 0x04, 0xA5, 0x78};
uint32_t loopsWithoutMsg = 0;
bool deviceFoundInTime = false;

int BH1750_address = 0x23; // i2c Addresse
byte buff[2];

typedef struct DeviceInfo {
  uint8_t eventType;
  uint8_t addrType;
  uint8_t addr[6];
  uint8_t rssi;
  uint8_t dataLen;
  uint8_t dataField[255];
};

void parseDeviceInfo(uint8_t *buff, struct DeviceInfo *info){
  info->eventType = buff[0];
  info->addrType = buff[1];
  memcpy(&info->addr, &buff[2], 6);
  info->rssi = buff[8];
  info->dataLen = buff[9];
  memcpy(&info->dataField, &buff[10], info->dataLen);
  return;
}
float getRange(uint8_t rssi) {
    int16_t ratio_db = txCalibratedPower - rssi;
    float ratio_linear = pow(10, ratio_db / 10);

    return sqrt(ratio_linear);
}
void p(char *fmt, ... ){
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, 128, fmt, args);
  va_end (args);
}
void sendCmd(uint8_t *data, uint8_t len){
  //Serial.write("<-");
  //for(uint8_t i=0;i<len;i++){
    //p("%02X", data[i]);
  //}
  //p("\r\n");
  mySerial.write(data, len);
}
void getPackage(uint8_t *out){
  while(mySerial.available() < 3){
    delay(2);
  }
  uint8_t type = mySerial.read();
  uint8_t event = mySerial.read();
  uint8_t data_len = mySerial.read();
  //p("->%02X%02X%02X", type, event, data_len);
  for(uint8_t i=0;i<data_len;i++){
    if(mySerial.available() == 0){
      while(mySerial.available() == 0){
        delay(2);
      }
    }
    out[i] = mySerial.read();
    //p("%02X", out[i]);
  }
  //p("\r\n");
}
uint8_t waitResponse(){
  loopsWithoutMsg = 0;
  uint8_t buff[255];
  getPackage(&buff[0]);
  uint8_t success = buff[2];
  free(buff);
  return success;
}

void checkForEvents(){
  if(mySerial.available() >= 3){
    loopsWithoutMsg = 0;
    uint8_t type = mySerial.read();
    uint8_t event_code = mySerial.read();
    uint8_t data_len = mySerial.read();
    //p("->%02X%02X%02X", type, event_code, data_len);
    uint8_t buff[255];
    for(uint8_t i=0;i<data_len;i++){
      if(mySerial.available() == 0){
        while(mySerial.available() == 0){
          delay(2);
        }
      }
      buff[i] = mySerial.read();
      //p("%02X", buff[i]);
    }
    //p("\r\n");
    uint16_t event, success;
    struct DeviceInfo info;
    switch(type){
      case 0x04:
        success = buff[2];
        event = (buff[1] << 8) | buff[0];
        switch(event){
          case 0x060D:
           // GAP_DeviceInformation (device found)
           parseDeviceInfo(&buff[3], &info);
           if(memcmp(validDevice, info.addr, sizeof(validDevice)) == 0){
             p("Found correct device!\r\n");
             deviceFoundInTime = true;
             deviceFound = true;
           }
           break;
           case 0x0600: // GAP Device Init Done
             p("Device is ready\r\n");
             bleReady = true;
           break;
           case 0x0601: // GAP_DeviceDiscovery done
             p("Device discovery done\r\n");
             isScanning = false;
           break;
        }
        break;
    }
  }
}
/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
 */

// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(enginePin, OUTPUT);
  pinMode(leftLockDir, OUTPUT);
  pinMode(rightLockDir, OUTPUT);
  pinMode(execLock, OUTPUT);

  pinMode(lightsPin, OUTPUT);
  
  digitalWrite(enginePin, LOW);
  digitalWrite(leftLockDir, LOW);
  digitalWrite(rightLockDir, LOW);
  digitalWrite(execLock, LOW);
  digitalWrite(lightsPin, LOW);

mySerial.begin(57600);
  //Serial.begin(57600);
  //Serial.begin(115200);
  //while(!Serial);

  // Init ble device
  static uint8_t buf[42] = {
    0x01, // Type
    0x00, 0xFE, // Opcode 0xFE00 (GAP_DEVICEINT)
    0x26, // Data Len
    0x08, // Profile role (role task id 8 = centeral)
    0x05, // Max Scan Rsps
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // IRK
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // SRK
    0x00, 0x00, 0x00, 0x00, // Sign Counter
  };
  static uint8_t  gapCentralRoleIRK[16] = {0};
  static uint8_t  gapCentralRoleSRK[16] = {0};
  static uint32_t gapCentralRoleSignCounter = 1;
  memcpy(&buf[38], &gapCentralRoleSignCounter, 4);
  sendCmd(buf, 42);
  waitResponse();

  Wire.begin();
  Wire.beginTransmission(BH1750_address);
  Wire.write(0x10); // 1 [lux] aufloesung
  Wire.endTransmission();
  delay(200);
  t.every(100, takeReading);
  
  t.oscillate(13, 500, HIGH);
}
void loop() {
  checkForEvents();
  if(!bleReady){
    return;
  }
  if(!isScanning){
    if(!deviceFoundInTime){
      deviceNotFoundCount++;
    }else{
      deviceNotFoundCount = 0;
    }
    // Begin scanning
    uint8_t buf[7] = {
      0x01, // Type : 0x01 = Command
      0x04, 0xFE, // Opcode 0xFE04 (GAP_DeviceDiscoveryRequest)
      0x03, // Data Length
      0x03, // Mode : 0x02 = Limited Discoverble Devices
      0x01, // Active Scan (true)
      0x00, // White list when creating link (false)
    };
    sendCmd(buf, 7);
    //Serial.println("Trying Scan");
    if(waitResponse() == 0x00){
      isScanning = true;
    }
    deviceFoundInTime = false;
    delay(35);
  }
  if(deviceNotFoundCount >= 2 && !deviceFoundInTime){
    deviceFound = false;
    deviceNotFoundCount = 0;
    digitalWrite(enginePin, LOW);
    if(carOn){
      turnOff();
    }
    carOn = false;
  }else if(deviceFoundInTime){
    if(!carOn){
      turnOn();
    }
    uint8_t retCount;
    int avg = getLightAvg(retCount);
    if(avg < 175 && retCount > 3){
      turnLightsOn();
    }
    carOn = true;
  }
  t.update();
  //Serial.print("-\n");
}
void turnLightsOn(){
  digitalWrite(lightsPin, HIGH);
}
void turnOff(){
  digitalWrite(execLock, HIGH);
  digitalWrite(lightsPin, LOW);
  digitalWrite(enginePin, LOW);
  //Serial.println("Turning Car Off");
  t.after(750, finishTurnOff);
}
void finishTurnOff(){
  digitalWrite(execLock, LOW);
  //Serial.println("Finished Turning Car Off");
}
void turnOn(){
  digitalWrite(enginePin, HIGH);
  digitalWrite(leftLockDir, HIGH);
  digitalWrite(rightLockDir, HIGH);
  digitalWrite(execLock, HIGH);
  //Serial.println("Turning Car On");
  t.after(750, finishTurnOn);
}
void finishTurnOn(){
  digitalWrite(rightLockDir, LOW);
  digitalWrite(leftLockDir, LOW);
  digitalWrite(execLock, LOW);
  //Serial.println("Finished Turning Car On");
}
int getLightAvg(uint8_t &retCount){
  uint32_t tmpVar = 0;
  retCount = 0;
  for(int i = 0; i < (sizeof(lightAry) / sizeof(int)); i++){
    if(lightAry[i] != -1){
      tmpVar += lightAry[i];
      retCount++;
    }
  }
  if(retCount == 0){
    return 0;
  }
  return (int) (tmpVar / retCount);
}
void takeReading(){
  if(carOn){
    uint16_t reading = readLight();
    if(reading != -1){
      if(lightPtr > ((sizeof(lightAry) / sizeof(int)) - 1) ){
        lightPtr = 0;
      }
      lightAry[lightPtr] = reading;
      lightPtr++;
    }
  }
}
int16_t readLight(){

  byte i=0;
  Wire.beginTransmission(BH1750_address);
  Wire.requestFrom(BH1750_address, 2);
  while(Wire.available()){
    buff[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();  
  if(i == 2){
    float valf = ((buff[0] << 8) | buff[1]) / 1.2;
    if(valf < 0 || valf > 0x7FFF){
      return 0x7FFF;
    }else{
      return (int) valf;
    }
  }else{
    return -1;
  }
}
