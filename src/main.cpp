#include "BluetoothSerial.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdio.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled. Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
  #error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

#define BT_CONNECTION_LED GPIO_NUM_13  // Flashing-> not connected, steady -> connected
#define HEARTBEAT_LED GPIO_NUM_14      // Flashing-> normal

int reconnection_count = 0;
bool bt_connected = false;

TaskHandle_t flashLED_TaskHandler = nullptr;

String relay_bt_name = "ESP32-BT-Master";
uint8_t slave_mac_address[6] = {0xC8, 0xF0, 0x9E, 0xFB, 0xBE, 0x4E};  // MAC address of the slave device
BluetoothSerial SerialBT;

extern "C" {
void app_main(void);
}

void flashLED(void *) {
  while (1) {
    gpio_set_level(HEARTBEAT_LED, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(HEARTBEAT_LED, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    if (bt_connected) {
      gpio_set_level(BT_CONNECTION_LED, HIGH);
    } else {
      gpio_set_level(BT_CONNECTION_LED, ~gpio_get_level(BT_CONNECTION_LED));
    }
  }
}

void bt_status_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_DATA_IND_EVT) {
      while (SerialBT.available() > 0) {
        char c = SerialBT.read();
        Serial.write(c);
      }
  } else if (event == ESP_SPP_OPEN_EVT) {
    Serial.println("Client Connected");
    bt_connected = true;
    reconnection_count = 0;
  } else if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("Client Disconnected");
    bt_connected = false;
  }
}

void slave_connect() {
  static u_int32_t previousMillisReconnect = 0;
  // Wait 1 sec between attempts
  if (millis() - previousMillisReconnect >= 10000) {
    Serial.printf("This device BT name: %s\n", relay_bt_name.c_str());
    Serial.printf("Attempting to connect to device address: ");
    for (auto i = 0; i < sizeof(slave_mac_address); i++) {
      Serial.printf("%#2X, ", slave_mac_address[i]);
    }
    Serial.println();
    previousMillisReconnect = millis();
    Serial.print("Attempt No.: ");
    Serial.println(++reconnection_count);
    SerialBT.end();
    SerialBT.begin(relay_bt_name, true);
    SerialBT.connect(slave_mac_address);
  }
}

void app_main(void) {
  if (xTaskCreate(flashLED, "flashLED", 1024 * 2, NULL, 10, &flashLED_TaskHandler) != pdPASS) {
    printf("Failed to create task");
  }

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

  SerialBT.register_callback(bt_status_callback);
  SerialBT.begin(relay_bt_name, true);

  // Remove extraneous serial output from this device
  esp_log_level_set("*", ESP_LOG_ERROR);

  while (1) {
    if (!bt_connected) {
        slave_connect();
    }

    if (Serial.available()) {
      uint8_t buf[48] = {};
      auto len = Serial.read(buf, sizeof(buf));

      // Some devices require a newline character to process the input
      buf[len++] = '\n';
      // printf("Received %d bytes\n", len);
      // printf("Sending: %s\n", buf);
      SerialBT.write(buf, len);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
