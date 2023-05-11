

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
}
// Include your DFRobot_LCD.h header here
#include "DFRobot_LCD.h"


#define SDA_PIN 10
#define SCL_PIN 8

static const char *TAG = "main";

extern "C" {
void app_main(void)
{

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
  ESP_LOGI("test", "2");
  lcd.setCursor(0,0);
  lcd.printstr("Hello CSE121!");
  lcd.setCursor(0,1);
  lcd.printstr("Betros");
	ESP_LOGI("test", "3");
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
}


 
 
 
 /* extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
}
// Include your DFRobot_LCD.h header here
#include "DFRobot_LCD.h"


#define SDA_PIN 10
#define SCL_PIN 8

static const char *TAG = "main";

extern "C" {
void app_main(void);
}
void app_main(void)
{

  int colorR = 0;
  int colorG = 255;
  int colorB = 0;


    DFRobot_LCD lcd();
 
 
   // Initialising the display 
  lcd.init();
  
  // settting the color
  lcd.setRGB(colorR, colorG, colorB);
  lcd.setCursor(0,0);
  lcd.printstr("Hello CSE121!");
  lcd.setCursor(0,1);
  lcd.printstr("Renau");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/

/*
void app_main(void) {
  int colorR = 0;
  int colorG = 255;
  int colorB = 0;
  
  //making the LCD object
  DFRobot_LCD lcd(16,2);
  
  // Initialising the display 
  lcd.init();
  
  // settting the color
  lcd.setRGB(colorR, colorG, colorB);
  lcd.setCursor(0,0);
  lcd.(0, 255, 0);
  lcd.printstr( "Hello CSE121!");
  lcd.setCursor(0,1);
  lcd.printstr( "Renau" );
}
*/
