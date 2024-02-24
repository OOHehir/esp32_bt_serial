#include "BluetoothSerial.h"
#include "driver/gpio.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdio.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define LED_BT_BLUE                                                            \
  2 // BT: Internal LED (or LED on the pin D2) for the connection indication
    // (connected LED ON / disconnected LED OFF)
#define LED_BT_RED                                                             \
  15 // BT: LED (LED on the pin D4) for the connection indication (connected LED
     // OFF / disconnected LED ON)
#define LED_PIN GPIO_NUM_8             // Heartbeat
unsigned long previousMillisReconnect; // BT: Variable used for comparing millis
                                       // counter for the reconnection timer
bool ledBtState = false; // BT: Variable used to change the indication LED state
bool SlaveConnected; // BT: Variable used to store the current connection state
                     // (true=connected/false=disconnected)
int recatt = 0;      // BT: Variable used to count the reconnection attempts

String myName =
    "ESP32-BT-Master"; // BT: Variable used to store the SERVER(Master)
                       // bluetooth device name; just for prinitng
String slaveName =
    "ELOC_NONAME"; // BT: Variable used to store the CLIENT(Slave) bluetooth
                      // device name; just for prinitng; just for printing in
                      // this case
String MACadd =
    "03:B4:16:72:36:9C"; // BT: Variable used to store the CLIENT(Slave)
                         // bluetooth device Mac address; just for prinitng;
                         // just for printing in this case
uint8_t address[6] = {0xC8, 0xF0, 0x9E, 0xFB, 0xBE, 0x4E}; // BT: Variable used to store the CLIENT(Slave) MAC address
                 // used for the connection; Use your own andress in the same
                 // format

BluetoothSerial SerialBT;

void flashLED(void *) {
  while (1) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// BT: Bt_Status callback function
void Bt_Status(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event ==
      ESP_SPP_OPEN_EVT) { // BT: Checks if the SPP connection is open, the event
                          // comes// event == Client connected
    Serial.println("Client Connected"); // BT: Write to the serial monitor
    digitalWrite(
        LED_BT_BLUE,
        HIGH); // BT: Turn ON the BLUE bluetooth indication LED (solid light)
    digitalWrite(LED_BT_RED,
                 LOW); // BT: Turn OFF the RED bluetooth indication LED
    SlaveConnected =
        true;   // BT: Set the variable true = CLIENT is connected to the SERVER
    recatt = 0; // BT: Reset the reconnect attempts counter
  } else if (event == ESP_SPP_CLOSE_EVT) { // BT: event == Client disconnected
    Serial.println("Client Disconnected"); // BT: Write to the serial monitor
    digitalWrite(
        LED_BT_RED,
        HIGH); // BT: Turn ON the RED bluetooth indication LED (solid light)
    digitalWrite(LED_BT_BLUE,
                 LOW); // BT: Turn OFF the BLUE bluetooth indication LED
    SlaveConnected =
        false; // BT: Set the variable false = CLIENT connection lost
  }
}

void SlaveConnect() { // BT: This function connects/reconnects to the
                      // CLIENT(Slave)
  Serial.println(
      "Function BT connection executed"); // BT: Write to the serial monitor
  Serial.printf("Connecting to slave BT device named \"%s\" and MAC address "
                "\"%s\" is started.\n",
                slaveName.c_str(),
                MACadd.c_str()); // BT: Write to the serial monitor
  SerialBT.connect(
      address); // BT: Establishing the connection with the CLIENT(Slave) with
                // the Mac address stored in the address variable
}

extern "C" {
void app_main(void);
}

void app_main(void) {
  Serial.begin(115200);
  delay(3000);
  printf("\n");
  printf("Running\n");
  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  printf("This is a %s chip with %d CPU core(s), WiFi%s%s%s, ",
         CONFIG_IDF_TARGET, chip_info.cores,
         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "\n",
         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "\n");

  printf("silicon revision %d, \n", chip_info.revision);

  printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                       : "external");

  printf("Minimum free heap size: %d bytes\n",
         esp_get_minimum_free_heap_size());

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  SerialBT.register_callback(Bt_Status);
  SerialBT.begin(myName, true);  // Bluetooth device name, start as master
  SlaveConnect();

  while (1) {
    if (Serial.available()) {
      SerialBT.write(Serial.read());
    }
    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }

    if (!SlaveConnected) { // BT: Condition to evalute if the connection is
                           // established/lost
      if (millis() - previousMillisReconnect >=
          10000) { // BT: Check that 10000ms is passed
        previousMillisReconnect =
            millis(); // BT: Set previousMillisReconnect to current millis
        recatt++;     // BT: Increase the the reconnection attempts counter +1
        Serial.print("Trying to reconnect. Attempt No.: "); // BT: Write to the
                                                            // serial monitor
        Serial.println(
            recatt); // BT: Write the attempts count to the serial monitor
        Serial.println(
            "Stopping Bluetooth..."); // BT: Write to the serial monitor
        SerialBT.end();               // BT: Close the bluetooth device
        Serial.println(
            "Bluetooth stopped !"); // BT: Write to the serial monitor
        Serial.println(
            "Starting Bluetooth..."); // BT: Write to the serial monitor
        SerialBT.begin(myName,
                       true); // BT: Starts the bluetooth device with the name
                              // stored in the myName variable as SERVER(Master)
        Serial.printf("The device \"%s\" started in master mode, make sure "
                      "slave BT device is on!\n",
                      myName.c_str());
        SlaveConnect(); // BT: Calls the bluetooth connection function to cnnect
                        // to the CLIENT(Slave)
      }
    }
    // BT: Data send/receive via bluetooth
    if (Serial.available()) { // BT: Checks if there are data from the serial
                              // monitor available
      SerialBT.write(Serial.read()); // BT: Sends the data via bluetooth
    }
    if (SerialBT.available()) {      // BT: Checks if there are data from the
                                     // bluetooth available
      Serial.write(SerialBT.read()); // BT: Write the data to the serial monitor
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
