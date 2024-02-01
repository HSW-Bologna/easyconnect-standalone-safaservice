#ifndef INTERFACE_H_INCLUDED
#define INTERFACE_H_INCLUDED


#include <stdint.h>


#define NUM_LED_BALLAST 4


typedef enum {
    INTERFACE_LED_1 = 0,
    INTERFACE_LED_2,
    INTERFACE_LED_3,
    INTERFACE_LED_4,
} interface_led_t;


void    interface_init(void);
void    interface_set_led_state_off(interface_led_t led);
void    interface_set_led_state_on(interface_led_t led);
void    interface_set_led_state_blink(interface_led_t led, unsigned long millis);
void    interface_set_warning_alarm_off(void);
void    interface_set_warning(void);
void    interface_set_alarm(void);
uint8_t interface_manage(void);
void    interface_set_safety(uint8_t led);


#endif
