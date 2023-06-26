#include "SoftwareSerial.h"
#include <espduino.h>
#include <mqtt.h>
#include "Arduino.h"
#include <TroykaCurrent.h>

#define PUB_DELAY (5 * 1000) /* 5 секунд */
float lastValue; // Устанавливаем глобальную переменную с значением тока
ACS712 sensorCurrent(A0); //  Устанавливаем датчик тока

SoftwareSerial debugPort(9, 10); // RX, TX
ESP esp(&Serial, &debugPort, 4);
MQTT mqtt(&esp);
boolean wifiConnected = false;

void wifiCb(void* response) //Подключаемся к Wifi
{
  uint32_t status;
  RESPONSE res(response);

  if(res.getArgc() == 1) {
    res.popArgs((uint8_t*)&status, 4);
    if(status == STATION_GOT_IP) {
      debugPort.println("WIFI CONNECTED");
      mqtt.connect("dev.rightech.io", 1883, false); // Параметры mqtt
      wifiConnected = true; //Проверка подключения
    } else {
      wifiConnected = false;
      mqtt.disconnect();
    }

  }
}
void setup() {
  Serial.begin(9600);
  debugPort.begin(9600);
  esp.enable();
  delay(500);
  esp.reset();
  delay(500);
  while(!esp.ready());

  debugPort.println("ARDUINO: setup mqtt client");
  if(!mqtt.begin("mqtt-dariazheltov-7bxs7u", "", "", 120, 1)) {
    while(1);
  }
  mqtt.lwt("/lwt", "offline", 0, 0); // Устанавливаем mqtt lwt
  /*Подключаемся к Wi-Fi*/
  esp.wifiCb.attach(&wifiCb);
  esp.wifiConnect("TP-Link_B99B","28792680");
}

void checkChange(float value) {
  if(value > 0){
    onItem(value);
  }
  else{
    offItem(value);
  }
}

void onItem(float value) {
  if ((0.054<value) and (value<0.15))
  {
    Serial.println("on Lamp");
  }
  else if((0.15<value) and (value<0.3)){
    Serial.println("on monitor ");
  }
  else if((0.4<value) and (value<0.9)){
    Serial.println("on notebook");
  }
  else if((5<value) and (value<8)){
    Serial.println("on iron");
  }
}

void offItem(float value) {
  if ((-0.054>value) and (value>-0.15))
  {
    Serial.println("off Lamp");
  }
  else if((-0.15>value) and (value>-0.3)){
    Serial.println("off monitor ");
  }
  else if((-0.4>value) and (value>-0.9)){
    Serial.println("off notebook");
  }
  else if((-5>value) and (value>-8)){
    Serial.println("off iron");
  }
}

void loop() {
  esp.process();
  if(wifiConnected) {
    float changeValue;
    float currentValue;
    currentValue = sensorCurrent.readCurrentAC();
    mqtt.publish("base/state/status", "OK");
    mqtt.publish("base/state/ampere", currentValue);
    checkChange(changeValue);
    changeValue = currentValue - lastValue;
    lastValue = currentValue;
    if (currentValue > 7) // Если потребление тока выше 7А
    {
      Serial.println("ALERT VERY HIGH CURRENT - OFF ALL SYSTEM!!!");
      mqtt.publish("base/state/status", "ALARM");
      digitalWrite(in1, HIGH); // Выключаем реле
    }
    Serial.print("Current is ");
    Serial.print(currentValue);
    Serial.println(" A");
    delay(1500);
  }
}
