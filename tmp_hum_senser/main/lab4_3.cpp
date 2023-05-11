#include "DFRobot_LCD.h"
#include <stdio.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_log.h"


#define SDA_PIN 10
#define SCL_PIN 8


static const char *TAG = "main";

#define I2C_MASTER_SCL_IO 8 /*!< GPIO number for I2C master SCL */
#define I2C_MASTER_SDA_IO 10 /*!< GPIO number for I2C master SDA */
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_FREQ_HZ 100000
#define SHTC3_SENSOR_ADDR 0x70 /*!< SHTC3 I2C address */

#define SHTC3_CMD_READ_ID 0xEFC8
#define SHTC3_CMD_MEASURE 0x7866 /*!< Measurement command in normal mode */

static esp_err_t i2c_master_init()
{
	i2c_config_t conf = {
    	.mode = I2C_MODE_MASTER,
    	.sda_io_num = I2C_MASTER_SDA_IO,
    	.scl_io_num = I2C_MASTER_SCL_IO,
    	.sda_pullup_en = GPIO_PULLUP_ENABLE,
    	.scl_pullup_en = GPIO_PULLUP_ENABLE,
    	.master = {
        	.clk_speed = I2C_MASTER_FREQ_HZ,
    	},
	};


  esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to i2c_param_config %d", err);
    return err;
  }

  err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
  if(err != ESP_OK) {
    ESP_LOGE(TAG,"Failed to i2c_driver_install %d", err);
    return err;
  }

  return err;
}

static esp_err_t shtc3_read(uint16_t command, uint8_t *data, size_t size)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
 
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, command >> 8, true);
  i2c_master_write_byte(cmd, command & 0xFF, true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));

  i2c_cmd_link_delete(cmd);

  vTaskDelay(pdMS_TO_TICKS(20)); // Wait 20ms for sensor to process the data

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));

  i2c_cmd_link_delete(cmd);
  return err;
}

static float calculate_humidity(uint16_t raw_humidity){
  return 100.0 * (float)raw_humidity / 65535.0;
}

static float calculate_temperature(uint16_t raw_temperature){
  return -45.0 + 175.0 * (float)raw_temperature / 65535.0;
}

float celsius_to_fahrenheit(float celsius) {
  return (celsius * 9.0 / 5.0) + 32;
}


// CRC-8 calculation function
uint8_t crc8(const uint8_t *data, int len) {
  const uint8_t generator = 0x31;
  uint8_t crc = 0xFF;

  for (int i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ generator;
      } else {
        crc = (crc << 1);
      }
    }
  }

  return crc;
}

void shtc3_task() {
  char buffer[20]; // Buffer to hold the string to be printed

  while (1) {
  
  	int colorR = 0;
  	int colorG = 255;
  	int colorB = 0;
  	ESP_LOGI(TAG, "Starting...");

  	//i2c_port_t i2c_port = I2C_NUM_0;
  	uint8_t lcd_cols = 16;
  	uint8_t lcd_rows = 2;
  	uint8_t lcd_addr = 0x27;
  	uint8_t rgb_addr = 0x62;

  	DFRobot_LCD lcd(lcd_cols, lcd_rows);
 
  // Configure the I2C peripheral

	ESP_LOGI("test", "0");


  // Initialising the display 
  	lcd.init();
  	ESP_LOGI("test", "1");
  // settting the color
  	lcd.setRGB(colorR, colorG, colorB);
    uint8_t data[6] = {0,};
    uint16_t raw_temperature = 0;
    uint16_t raw_humidity = 0;
    esp_err_t err = shtc3_read(SHTC3_CMD_MEASURE, data, 6);
    if (err == ESP_OK) {
      // Validate received data using CRC
      if (crc8(data, 2) == data[2] && crc8(&data[3], 2) == data[5]) {
        raw_temperature = (data[0] << 8) | data[1];
        raw_humidity = (data[3] << 8) | data[4];

        float temperature_c = calculate_temperature(raw_temperature);
        float humidity = calculate_humidity(raw_humidity);

        // Clear the display
        lcd.clear();

        // Print temperature
        snprintf(buffer, sizeof(buffer), " Temp: %.1fC", temperature_c);
        lcd.setCursor(0, 0); // Set cursor to first line
        lcd.printstr(buffer);

        // Print humidity
        snprintf(buffer, sizeof(buffer), "Hum : %.1f%%", humidity);
        lcd.setCursor(0, 1); // Set cursor to second line
        lcd.printstr(buffer);

        ESP_LOGI(TAG, "Temp %.0fC with a %.0f%% hum", temperature_c, humidity);
      } else {
        ESP_LOGE(TAG, "CRC check failed for received data");
      }
    } else {
      ESP_LOGE(TAG, "Failed to read data from SHTC3 sensor %d", err);
    }

    vTaskDelay(pdMS_TO_TICKS(2000)); // Read data every 2 seconds
  }
}


void app_main(void) {

  esp_err_t err = i2c_master_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize I2C master");
    return;

  }
  shtc3_task();

}

}





