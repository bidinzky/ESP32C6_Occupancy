// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief This example demonstrates Zigbee occupancy sensor.
 *
 * The example demonstrates how to use Zigbee library to create a end device occupancy sensor.
 * The occupancy sensor is a Zigbee end device, which is reporting data to the Zigbee network.
 * Tested with PIR sensor HC-SR501 connected to GPIO4.
 *
 * Proper Zigbee mode must be selected in Tools->Zigbee mode
 * and also the correct partition scheme must be selected in Tools->Partition Scheme.
 *
 * Please check the README.md for instructions and more detailed description.
 *
 * Created by Jan Procházka (https://github.com/P-R-O-C-H-Y/)
 */

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include "LD2410S.h"

/* Zigbee occupancy sensor configuration */
#define OCCUPANCY_SENSOR_ENDPOINT_NUMBER 10
uint8_t button = BOOT_PIN;

ZigbeeOccupancySensor zbOccupancySensor = ZigbeeOccupancySensor(OCCUPANCY_SENSOR_ENDPOINT_NUMBER);

// Pin definitions
constexpr int RX_PIN = 17;
constexpr int TX_PIN = 16;

// Sensor object
LD2410S sensor;
class Listener: public LD2410SListener
{
  public:
    bool presence;
    int distance;
    std::string *fw;
  public:
    void on_presence(bool presence) {
      this->presence = presence;
      zbOccupancySensor.setOccupancy(presence);
      Serial.println(presence);
    };
    void on_distance(int distance) {
      this->distance = distance;
      Serial.println(distance);
    };
    void on_threshold_update(bool running) {
      Serial.print("running threshold udpdate");
      Serial.println(running);
    };
    void on_threshold_progress(int progress) {
      Serial.print("progress: ");
      Serial.println(progress);
    };
    void on_fw_version(std::string& fw) {
      this->fw = &fw;
      Serial.println(fw.c_str());
    };
};
Listener listener;

void setup() {
  Serial.begin(115200);

  // Init button + PIR sensor
  pinMode(button, INPUT_PULLUP);

  Serial.println("init sensor");
   Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  while (!Serial1) ;
  sensor.register_listener(&listener);

  // Optional: set Zigbee device name and model
  zbOccupancySensor.setManufacturerAndModel("Espressif", "ZigbeeOccupancyPIRSensor");

  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbOccupancySensor);

  Serial.println("Starting Zigbee...");
  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();
  } else {
    Serial.println("Zigbee started successfully!");
  }
  Serial.println("Connecting to network");
  
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  sensor.setup(Serial1);
  Serial.println("Sensor init");
  if(listener.fw != 0) {
    Serial.println(listener.fw->c_str());
  }
  sensor.
}

int startTime = 0;
int prevButtonState = HIGH;
void loop() {
    sensor.loop();
  int currentButtonState = digitalRead(button);
  // Checking button for factory reset
  if (currentButtonState == LOW && prevButtonState == HIGH) {
    //RISING EDGE
    startTime = millis();
  }
  if(prevButtonState == LOW && currentButtonState == HIGH) {
    //FALLING EDGE
    if((millis() - startTime) > 3000) {
      // If key pressed for more than 3secs, factory reset Zigbee and reboot
      Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
      //delay(1000);
      Zigbee.factoryReset();
    }else if((millis() - startTime) > 500) {
      // trigger calibration
      Serial.println("start auto threshold update");
      sensor.start_auto_threshold_update();
    }
  }
  prevButtonState = currentButtonState;
}
