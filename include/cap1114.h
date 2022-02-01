#ifndef CAP1114_H
#define CAP1114_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"


//I2C Parameters.
#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          
#define I2C_MASTER_FREQ_HZ          400000                     
#define I2C_MASTER_TX_BUF_DISABLE   0                          
#define I2C_MASTER_RX_BUF_DISABLE   0                          
#define I2C_MASTER_TIMEOUT_MS       1000

#define SWITCH_SLIDER_BTN 2                 /*! BUTTON ID (on CAP1114) THAT SWITCHES BETWEEN SLIDERS*/ 

#define HOLD_TIME 20						/*! Time required to consider a button held*/

//Registers addresses.
#define I2C_SLAVE_ADDRESS 0b0101000
#define CAP_VENDOR_ADDRESS 0xfe
#define CAP_CONFIGURATION_REGISTER 0x20
#define CAP_SENSOR_ENABLE_REGISTER 0x21
#define CAP_GPIO_DIRECTION_REGISTER 0x70
#define CAP_GPIO_OUTPUT_TYPE_REGISTER 0x71
#define CAP_LED_CONTROL_REGISTER_1 0x73
#define CAP_LED_CONTROL_REGISTER_2 0x74
#define CAP_LED_POLARITY_REGISTER_1 0x75
#define CAP_LED_LINK_REGISTER 0x80
#define CAP_LED_BEHAVIOR_REGISTER_3 0x83
#define CAP_MULTITOUCH_REGISTER 0x2a
#define CAP_BUTTON_READ_REGISTER_1 0x03
#define CAP_BUTTON_READ_REGISTER_2 0x04
#define CAP_GROUP_STATUS_REGISTER 0x0f
#define CAP_STATUS_CONTROL_REGISTER 0x00
#define CAP_SLEEP_CHANNEL_REGISTER 0x29
#define CAP_SLIDER_VOLUMETRIC_REGISTER 0x06
#define CAP_VOLUMETRIC_STEP_REGISTER 0x09

//Utility functions
int comp_bit(uint8_t src, uint8_t index);
esp_err_t ui_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t ui_register_write(uint8_t reg_addr, uint8_t data);
esp_err_t ui_init(void (*s_done)(), uint8_t s_count, float ** s_targets, float * s_mins, float * s_maxs, void (**s_callbacks)(float f), uint8_t * s_t_leds, uint8_t * s_leds, uint8_t s_led_count);

void ui_set_led(uint8_t num, uint8_t state);
void ui_test_led();
uint32_t read_buttons(); //Reads raw button value, prefere was_btn_pressed.
void ui_get_vendor(); //Reads the vendor ID, should be 5d

uint8_t was_btn_pressed(int btn_id); 
int is_button_held(int btn_id);
void ui_reset_held();

void set_led_editable_flag(uint8_t flag);


void ui_sleep();
void ui_wake_up();

#endif