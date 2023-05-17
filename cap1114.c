#include "cap1114.h"

uint8_t slider_counts = 0; //Number of different sliders.

float ** slider_targets;
float * slider_mins;
float * slider_maxs; 
void (**slider_callbacks)(float f);

uint8_t * target_leds = NULL; //List of leds that indicates wich value is targeted by the slider.

uint8_t * slider_leds = NULL; //List of leds ON the slider.
uint8_t slider_led_count = 0;

uint8_t slider_target = 0;

uint8_t previous_buttons[4] = {0};
int held_time[7] = {0};
uint8_t btn_pressed[7] = {0};

uint16_t led_mask = 0;

uint8_t can_write_led_flag = 1;
uint8_t has_modified = 0;
uint8_t ui_awake = 1;
uint64_t last_time_slider_changed = 0;
uint64_t last_time_ui_changed = 0;

void (*slider_done)();

static xTaskHandle s_ui_t_hdl = NULL;



int is_button_held(int btn_id){
	return(held_time[btn_id]>HOLD_TIME)?1:0;
}

int is_button_long_held(int btn_id){
	return(held_time[btn_id]>LONG_HOLD_TIME)?1:0;
}

void ui_set_showing_batt(){
	has_modified = 1;
	last_time_slider_changed = esp_timer_get_time();
	for(int i = 0; i < slider_counts; i++){
		ui_set_led(target_leds[i], 0);
	}
}

void ui_reset_held(){
	for(int i = 0; i < 7; i++){
		held_time[i] = 0;
	}
}

void ui_set_awake(){
	ui_awake = 1;
	last_time_ui_changed = esp_timer_get_time();
}

uint8_t was_btn_pressed(int btn_id){
	uint8_t ret = btn_pressed[btn_id];
	btn_pressed[btn_id] = 0;
	return ret;
}

int comp_bit(uint8_t src, uint8_t index){		//wrapper to compare a bit. TODO : make it a define.
	return ((src & (0b1<<index)) == (0b1<<index))?1:0;
}


esp_err_t ui_register_read(uint8_t reg_addr, uint8_t *data, size_t len){
    return i2c_master_write_read_device(I2C_MASTER_NUM, I2C_SLAVE_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

esp_err_t ui_register_write(uint8_t reg_addr, uint8_t data){
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, I2C_SLAVE_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);

    return ret;
}


void set_led_editable_flag(uint8_t flag){ //Sets a flag to prevent the CAP routine from writing LEDs.
	can_write_led_flag = flag;
}

void ui_set_led(uint8_t num, uint8_t state){
	if(can_write_led_flag == 0){
		return;
	}
	uint16_t mask = 0b0000000000000000 | (0b1 << num);
	
	if(state){
		led_mask |= mask;
	}else{
		led_mask &= (~mask);
	}
	uint8_t m1 = led_mask;
	uint8_t m2 = (led_mask >> 8);
	ui_register_write(CAP_LED_CONTROL_REGISTER_1, m1);
	ui_register_write(CAP_LED_CONTROL_REGISTER_2, m2);
}



void write_slider_leds(float value, float min, float max){
	float step = ((max-min)/slider_led_count)*0.99; //*0.99 to make sure to reach the top LED
	for(int i = 0; i < slider_led_count; i++){
		ui_set_led(slider_leds[i], (value >= min + step*i)?1:0);
	}
}

void write_last_slider_leds(float value, float min, float max){
	float step = ((max-min)/slider_led_count)*1.001; //*0.99 to make sure to reach the top LED
	ESP_LOGI("UI","Val : %f , Step : %f", value,step);
	for(int i = 0; i < slider_led_count; i++){
		uint8_t next_state = (value >= min + step*(i+1));
		if((value >= min + step*i) !=next_state){ui_set_led(slider_leds[i], 1);}
		else {ui_set_led(slider_leds[i], 0);}
		
	}
}

void move_slider_up(){
	slider_target++;
	if(slider_target >= slider_counts){
		slider_target = 0;
	}
	for(int i = 0; i < slider_counts; i++){
		ui_set_led(target_leds[i], i == slider_target ? 1:0);
	}
	
	write_slider_leds(* (slider_targets[slider_target]), slider_mins[slider_target], slider_maxs[slider_target]);
}
void slider_target_sleep(){
	
	slider_target = 0;
	write_last_slider_leds(* (slider_targets[slider_target]), slider_mins[slider_target], slider_maxs[slider_target]);
	
	ui_set_led(target_leds[0], 1);				//Turning only the first target on.
	for(int i = 1; i < slider_counts; i++){
		ui_set_led(target_leds[i], 0);
	}
	
}

void status_led(int mode){
	
	switch (mode)
	{
	case 0:
		ui_set_led(10, 0);
		//ui_register_write(CAP_LED_BEHAVIOR_REGISTER_3,  0x0);
		

		break;
	case 1:
		//ui_set_led(11,0);
		ui_register_write(CAP_LED_BEHAVIOR_REGISTER_3,  0x0);
		ui_set_led(10,1);
		break;

	case 2:
		//ui_set_led(11,0);
		ui_register_write(CAP_LED_BEHAVIOR_REGISTER_3, 0b00110000);
		ui_set_led(10,1);
		
	
		break;
	
	default:
		ui_set_led(10, 0);
		ui_register_write(CAP_LED_BEHAVIOR_REGISTER_3,  0x0);
		break;
	}
	return;
}


static void ui_routine(void *arg){
    ESP_LOGI("I2C", "starting the UI sub routine.");
	for (;;) {
		vTaskDelay(100 / portTICK_RATE_MS); 
		
		//No interaction for some time. let's fall back to default slider.
		if(has_modified && (last_time_slider_changed + 4000000 < esp_timer_get_time() )){
			ESP_LOGI("UI", "Stopped modifying.");
			last_time_slider_changed = esp_timer_get_time();
			has_modified = 0;
			slider_target_sleep();
			slider_done();
		}
		if (ui_awake ==1){
			ui_register_write(CAP_BREATHE_DUTY_CYCLE_REGISTER,  0xA0);
			ui_register_write(CAP_DIRECT_DUTY_CYCLE_REGISTER,  0xA0);
		}
		if(ui_awake && (last_time_ui_changed + 20000000 < esp_timer_get_time() )){
			ESP_LOGI("UI", "Dimming");
			last_time_ui_changed = esp_timer_get_time();

			ui_register_write(CAP_BREATHE_DUTY_CYCLE_REGISTER,  0x30);
			ui_register_write(CAP_DIRECT_DUTY_CYCLE_REGISTER,  0x30);

			ui_awake = 0;
		}
		
		uint32_t buttons = read_buttons();
		uint8_t *b = (uint8_t *) &buttons;
		
		//Changing the slider mode.
		if( (comp_bit(b[1], SWITCH_SLIDER_BTN)?1:0) && (comp_bit(previous_buttons[1], SWITCH_SLIDER_BTN)?0:1) ){
			
			move_slider_up();
			ESP_LOGI("Equalizer", "switched slider to mode : %d", slider_target );
	
			last_time_slider_changed = esp_timer_get_time();
			last_time_ui_changed = esp_timer_get_time();
			has_modified = 1;
			ui_awake = 1;
			
		}
		
		//Handling the slider.
		if((comp_bit(b[0], 3)||comp_bit(b[0], 2))&& b[3] != 0){
			last_time_slider_changed = esp_timer_get_time();
			last_time_ui_changed = esp_timer_get_time();
			
			//Mapping to desired value.
			float new_value = slider_mins[slider_target] + ( ((float)b[3] - 2.0)/96.0) * (slider_maxs[slider_target] - slider_mins[slider_target]);
			
			*(slider_targets[slider_target])= new_value;
			slider_callbacks[slider_target](new_value);
			
			write_slider_leds(* slider_targets[slider_target], slider_mins[slider_target], slider_maxs[slider_target]);
			has_modified = 1;
			ui_awake = 1;
		}
		
		//Making sure the power button is held.
		for(int i = 0; i < 7; i++){
			if(comp_bit(b[1], i)){
				btn_pressed[i] = 1;
				held_time[i]++;
			}else{
				held_time[i] = 0;
			}	
		}
		
		//Battery
		

		
		
		previous_buttons[0] = b[0];
		previous_buttons[1] = b[1];
		previous_buttons[2] = b[2];
		previous_buttons[3] = b[3];
		
		
	}    
}





esp_err_t ui_init(void (*s_done)(), uint8_t s_count, float ** s_targets, float * s_mins, float * s_maxs, void (**s_callbacks)(float f), uint8_t * s_t_leds, uint8_t * s_leds, uint8_t s_led_count){
	
	
	int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);
	
	esp_err_t err = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	
	ui_register_write(CAP_CONFIGURATION_REGISTER, 0b00101101);
	
	
	
	
	ui_register_write(CAP_GPIO_DIRECTION_REGISTER, 0b11111111);
	ui_register_write(CAP_LED_LINK_REGISTER, 0b00000000);
	ui_register_write(CAP_GPIO_OUTPUT_TYPE_REGISTER, 0b00000000); 
	ui_register_write(CAP_LED_POLARITY_REGISTER_1, 0b00000000); 
	
	
	ui_register_write(CAP_SENSOR_ENABLE_REGISTER, 0b11111111);
	ui_register_write(CAP_VOLUMETRIC_STEP_REGISTER, 0b00001100);
	ui_register_write(CAP_STATUS_CONTROL_REGISTER, 0b00000000);
	ui_register_write(CAP_LED_BEHAVIOR_REGISTER_3, 0b00110000);
	ui_register_write(CAP_SLEEP_CHANNEL_REGISTER, 0b00001000);
	ui_register_write(CAP_DATA_SENSITIVITY_REGISTER, 0b00111111);
	ui_register_write(CAP_BREATHE_PERIOD, 0x0C);
	ui_register_write(CAP_QUEUE_CONTROL_REGISTER, 0x04);

	
	slider_done = s_done;
	
	slider_counts = s_count; //Number of different sliders.
	slider_targets = s_targets;
	slider_mins = s_mins;
	slider_maxs = s_maxs; 
	slider_callbacks = s_callbacks;

	target_leds = s_t_leds; //List of leds that indicates wich value is targeted by the slider.

	slider_leds = s_leds; //List of leds ON the slider.
	slider_led_count = s_led_count;
	
	slider_target_sleep();
	xTaskCreatePinnedToCore(ui_routine, "uiT", 8192, NULL, 2, &s_ui_t_hdl, 1);
	
	return err;
}



 
void ui_get_vendor(){
	uint8_t data[2];
	ui_register_read(CAP_VENDOR_ADDRESS, data, 1);
	ESP_LOGI("I2C", "CAP_VENDOR_ID = %X", data[0]);
}



void ui_sleep(){
	ESP_LOGI("UI", "Going to sleep :)");
	uint8_t control[2];
	ui_register_read(CAP_STATUS_CONTROL_REGISTER, control, 1);
	ui_register_write(CAP_LED_BEHAVIOR_REGISTER_3, 0x00);
	ui_register_write(CAP_LED_CONTROL_REGISTER_1, 0x00);
	ui_register_write(CAP_LED_CONTROL_REGISTER_2, 0x00);
	ui_register_write(CAP_STATUS_CONTROL_REGISTER, control[0] | 0b00100000);
	
}
void ui_wake_up(){
	ESP_LOGI("UI", "Waking up :)");
	uint8_t control[2];
	ui_register_read(CAP_STATUS_CONTROL_REGISTER, control, 1);
	ui_register_write(CAP_STATUS_CONTROL_REGISTER, control[0] & 0b11011111);
}

uint32_t read_buttons(){
	uint8_t control[2];
	
	ui_register_read(CAP_STATUS_CONTROL_REGISTER, control, 1);
	
	uint8_t data[5];
	
	ui_register_read(CAP_SLIDER_VOLUMETRIC_REGISTER, data+3, 1);
	ui_register_read(CAP_GROUP_STATUS_REGISTER, data, 1);
	ui_register_read(CAP_BUTTON_READ_REGISTER_1, data+1, 1);
	ui_register_read(CAP_BUTTON_READ_REGISTER_2, data+2, 1);
	
	ui_register_write(CAP_STATUS_CONTROL_REGISTER, control[0] & 0b10001110);
	
	
	
	//ESP_LOGI("I2C", "Buttons Pressed : "BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN , BYTE_TO_BINARY(data[2]), BYTE_TO_BINARY(data[1]));
	return (data[3] << 24) |(data[2] << 16) | (data[1] << 8) | data[0];
}




void ui_test_led(){
	for(uint8_t i = 0; i < 12;i++){
		ui_set_led(i, 1);
		vTaskDelay(200 / portTICK_RATE_MS);

	}
	for(uint8_t i = 0; i < 12;i++){
		ui_set_led(i, 0);
		vTaskDelay(200 / portTICK_RATE_MS);
	}
}


