#include "hal/led/led.h"
#ifdef LED_CYW43_SUPPORTED
    #include "pico/cyw43_arch.h"
#elif LED_KB2040_SUPPORTED
	#include "peri/ws2812/ws2812.h"
#else
	#include "hardware/gpio.h"
#endif

// ======================================== INTERNAL FUNCTIONS ===============================================
static uint8_t pin_used = 0;	// Default pin of board LED (Pico / Pico 2)
static bool led_state = false;


// ======================================== CALLABLE FUNCTIONS ===============================================
void set_gpio_default_led(uint8_t led_pin){
	#ifdef LED_CYW43_SUPPORTED
		pin_used = CYW43_WL_GPIO_LED_PIN;
	#elif LED_KB2040_SUPPORTED
		pin_used = 17;
	#elif LED_TINY2040_SUPPORTED
		pin_used = 19;
	#else
		pin_used = led_pin;
	#endif
};


uint8_t get_gpio_default_led(void){
	return pin_used;
};


bool init_default_led(void){
	led_state = false;
	set_gpio_default_led(25);
	#ifdef LED_CYW43_SUPPORTED
		cyw43_arch_gpio_put(pin_used, false);
	#elif LED_KB2040_SUPPORTED
		PIO pio = pio0;
		ws2812_init(pio, pin_used);
		put_pixel_rgb(0, 0, 0); 
	#elif LED_TINY2040_SUPPORTED
		// Init of residual LED pins on Tiny2040
		gpio_init(18);
		gpio_set_dir(18, GPIO_OUT);
		gpio_put(18, true);

		gpio_init(19);
		gpio_set_dir(19, GPIO_OUT);
		gpio_put(19, true);
		
		gpio_init(20);
		gpio_set_dir(20, GPIO_OUT);
		gpio_put(20, true);		
	#else
		gpio_init(pin_used);
		gpio_set_dir(pin_used, GPIO_OUT);
		gpio_put(pin_used, false);
	#endif
	return true;
};


bool set_state_default_led(bool state){
	led_state = state;
	#ifdef LED_CYW43_SUPPORTED 
		cyw43_arch_gpio_put(pin_used, led_state);
	#elif LED_KB2040_SUPPORTED
		if(led_state) 
			put_pixel_rgb(0, 32, 0);
		else
			put_pixel_rgb(0, 0, 0);
	#elif LED_TINY2040_SUPPORTED
		if(led_state){
			gpio_put(pin_used, false);
		} else {
			gpio_put(pin_used, true);
		}
	#else
		gpio_put(pin_used, led_state);
	#endif
	return led_state;
};


bool get_state_default_led(void){
	#ifdef LED_CYW43_SUPPORTED
		// GPIO pin for the LED on the CYW43 is write-only
	#elif LED_KB2040_SUPPORTED
		// WS2812 state cannot be read back
	#elif LED_TINY2040_SUPPORTED
		led_state = !gpio_get(pin_used);
	#else
		led_state = gpio_get(pin_used);
	#endif
	return led_state;
};


bool toggle_state_default_led(void){
	if(get_state_default_led()){
		set_state_default_led(false);
	} else {
		set_state_default_led(true);
	}
	return led_state;
};
