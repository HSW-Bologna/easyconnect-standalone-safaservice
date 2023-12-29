#ifndef HARDWAREPROFILE_H_INCLUDED
#define HARDWAREPROFILE_H_INCLUDED

/*
 * Definizioni dei pin da utilizzare
 */

#include <driver/gpio.h>

#define HAP_SAFETY_INPUT             GPIO_NUM_2
#define HAP_SAFETY_ALARM             GPIO_NUM_3
#define HAP_PULS_RESET_LIFETIME_LAMP GPIO_NUM_0
#define HAP_EXP_LIFE_LAMP            GPIO_NUM_1
#define HAP_BAL1                     GPIO_NUM_10
#define HAP_BAL2                     GPIO_NUM_4
#define HAP_BAL3                     GPIO_NUM_6
#define HAP_BAL4                     GPIO_NUM_5
#define HAP_TXD                      GPIO_NUM_21
#define HAP_RXD                      GPIO_NUM_20
#define HAP_DIR485                   GPIO_NUM_7
#define HAP_LED_R                    GPIO_NUM_8
#define HAP_LEG_G                    GPIO_NUM_9

#endif
