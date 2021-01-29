/*  To change the upload speed, add the following to the ESP_gateway entry into C:\Users\benkokes\AppData\Local\arduino15\packages\esp32\hardware\esp32\1.0.4-rc1\boards.txt
    esp32-gateway.menu.UploadSpeed.921600=921600
    esp32-gateway.menu.UploadSpeed.921600.upload.speed=921600

 * Sketches used as examples to assemble this code
   BLECentral Scanner:https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLETests/Arduino/BLE_scan/BLE_scan.ino
   Ethernet example: https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/ETH_LAN8720/ETH_LAN8720.ino
   Current time service: https://github.com/kerikun11/ESP32_BLE_Current_Time_Service/blob/master/gatt-cts-client/src/main.cpp
   NTP: https://github.com/arduino-libraries/NTPClient
   rtc:https://github.com/fbiego/ESP32Time
   Ring Buffer: https://github.com/Locoduino/RingBuffer
*/
#include <ESP32Time.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <ETH.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include "time.h"
#include <chrono>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>
#include <RingBuf.h>

struct adv_info{
  std::string adv_name;
  std::string adv_addr;
  unsigned long int adv_update_time;
  unsigned long int time_since_refresh;
}adv_datas;

RingBuf<adv_info, 10> history_buf;

static const BLEUUID GATT_CTS_UUID((uint16_t)0x1805);
static const BLEUUID GATT_CTS_CTC_UUID((uint16_t)0x2A2B);
std::string ble_name ("Lifeonometer");
std::string found_ble_name;
bool device_found = false;

ESP32Time rtc;
WiFiUDP ntpUDP;
NTPClient NTPtimeClient(ntpUDP, "pool.ntp.org", -28800,1);
unsigned long int epock_time_latest = 0;
unsigned long int epoch_time_old = 0;
tm time_struct_old;
tm time_struct_temp;
unsigned long int time_update_delta = 0;
BLEAdvertisedDevice foundDevice;

int scanTime = 5; //In seconds
BLEScan *pBLEScan;
BLEServer *pServer;

static bool eth_connected = false;
unsigned long int loop_count=0;


void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

  /* BLE Server */
  class MyBLEServerCallbacks : public BLEServerCallbacks {
  public:
    MyBLEServerCallbacks() {}
    virtual void onConnect(BLEServer *pServer) override {
      BLEDevice::getAdvertising()->stop();
    }
    virtual void onDisconnect(BLEServer *pServer) override {
      BLEDevice::getAdvertising()->start();
    }
  };

//Callback to capture desird advertiser.
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
       if (advertisedDevice.haveName()){        //Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
        found_ble_name = advertisedDevice.toString().c_str();
        if(found_ble_name.find(ble_name) != std::string::npos){
        advertisedDevice.getScan()->stop();
        device_found = true;
        foundDevice = advertisedDevice;
        Serial.print("Device name: ");
        Serial.print(advertisedDevice.getName().c_str());
        Serial.print(", Address: ");
        Serial.println(foundDevice.getAddress().toString().c_str());
        //save to structure
        adv_datas.adv_name = advertisedDevice.getName().c_str();
        adv_datas.adv_addr  = foundDevice.getAddress().toString().c_str();
        time_struct_temp = rtc.getTimeStruct();
        adv_datas.adv_update_time = mktime(&time_struct_temp); //save existing time before updating
        adv_datas.time_since_refresh = (adv_datas.adv_update_time - epock_time_latest); //time since last update
        history_buf.push(adv_datas);      
        }
       }
    }
};

  class MyBLECharacteristicCallbacks : public BLECharacteristicCallbacks {
  public:
    MyBLECharacteristicCallbacks() {}
    virtual void onRead(BLECharacteristic *pCharacteristic) override {
      uint8_t data[10] = {0};
     
      data[0] = (rtc.getYear()) & 0xff; 
      data[1] = ((rtc.getYear()) >> 8) & 0xff;
      data[2] = rtc.getMonth()+1;
      data[3] = rtc.getDay();
      data[4] = rtc.getHour();
      data[5] = rtc.getMinute();
      data[6] = rtc.getSecond();
      data[7] = rtc.getDayofWeek();
      data[8] = 0; // 256 fractions of a second
      data[9] = 0;
      
      pCharacteristic->setValue(data, sizeof(data));
    }
    virtual void onWrite(BLECharacteristic *pCharacteristic) override {

    }
  };

void setup() {
  Serial.begin(115200);
  
  rtc.setTime(30, 30, 3, 30, 11, 2013); 

  WiFi.onEvent(WiFiEvent);
  ETH.begin();
  delay(1000);
  if (eth_connected) { 
      NTPtimeClient.begin();
      delay(1000);
      if(NTPtimeClient.update()){
        Serial.println("NTP_RetrieveSuccess:" + NTPtimeClient.getEpochTime()); 
      }else{
        Serial.println("Initial NTP time retrieval failure");
      }
  }

  Serial.println("Scanning...");

  BLEDevice::init("ESP32 CTS Server");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyBLEServerCallbacks());
      // Current Time Service 
  BLEService *pService = pServer->createService(GATT_CTS_UUID);
  
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
  GATT_CTS_CTC_UUID, BLECharacteristic::PROPERTY_READ |
                         BLECharacteristic::PROPERTY_WRITE |
                         BLECharacteristic::PROPERTY_NOTIFY);
  uint8_t data[10] = {0xDD,0x07,11,30,10,10,10,7,0,0};
  pCharacteristic->setValue(data, 10);

  pCharacteristic->setCallbacks(new MyBLECharacteristicCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value  
}

void loop() {
 
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  
  if(device_found){
    device_found =false;
    pServer->connect(foundDevice.getAddress());    
  }

  if(loop_count%14400 == 0)  // check for time once a day.( 1 scan is 5 sec, + 1 sec delay = 6 seconds. (86400s/day) / 6 = 14400
  {
   if (eth_connected) { 
      if(!NTPtimeClient.update()){
        delay(1000);
        Serial.println("Failed to obtain time");
      }else{
        time_struct_old = rtc.getTimeStruct();
        epoch_time_old = mktime(&time_struct_old); //save existing time before updating
        
        epock_time_latest =NTPtimeClient.getEpochTime(); // update system time
        rtc.setTime(epock_time_latest);
        
        Serial.print("New RXd NPT Time:" + rtc.getTime("%A, %B %d %Y %H:%M:%S")+ " Delta from old system time(sec):"+(epock_time_latest - epoch_time_old));
        Serial.print("  ");
        Serial.print(" RingBuff Fill:");
        Serial.println(history_buf.size());
      }
    }
    Serial.println("ESP32 Time:" + rtc.getTime("%A, %B %d %Y %H:%M:%S") );
  }

  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory

  loop_count++;
  delay(1000);
}
