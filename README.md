
# CAP1114-ESP32lib
This library is a component for the ESP-IDF to interface with the CAP1114 capacitive touch IC.

## Features

- Multiple slider support with callbacks
- Full led control
- Sleep mode
- Button support

## Documentation

Make sure to edit the cap1114.h to set the correct pins and diverse parameters.

Functions :
| **ui_init** |  Callback to the slider when no longer touched, Number of sliders, Values affected by slider, Minimum for each slider, Maximum for each slider, Callback for each slider, LEDs ID that indicate which slider is enabled, slider value indicator LEDs ID, number of slider value indicator LEDs | Inits the UI |
|--|--|--|
| **ui_test_led** |  | Blinks every LED |
|--|--|--|
| **ui_get_vendor** |  | Gets the chip vendor ID |
|--|--|--|
| **was_btn_pressed** | Button ID | Checks if a button was pressed |
|--|--|--|
| **is_button_held** | Button ID | Checks if a button was held |
|--|--|--|
| **ui_reset_held** |  | Resets the flags for every button held  |
|--|--|--|
| **ui_sleep** |  | Puts the CAP1114 chip to sleep |
|--|--|--|
| **ui_wake_up** |  | Wakes up the CAP114 chip. |
|--|--|--|
| **set_led_editable_flag** | flag (1 or 0) | Enables or disables changing the LED state |
