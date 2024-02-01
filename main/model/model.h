#ifndef MODEL_H_INCLUDED
#define MODEL_H_INCLUDED


#include <stdlib.h>


#define MODBUS_MAX_DEVICES 4


typedef enum {
    BALLAST_SEQUENCE_NONE = 0,
    BALLAST_SEQUENCE_1,
    BALLAST_SEQUENCE_2,
    BALLAST_SEQUENCE_3,
    BALLAST_SEQUENCE_4,
    BALLAST_SEQUENCE_DONE,
} ballast_sequence_t;


typedef enum {
    BALLAST_PRESENCE_UNKNOWN = 0,
    BALLAST_PRESENCE_FOUND,
    BALLAST_PRESENCE_MISSING,
} ballast_presence_t;


typedef struct {
    struct {
        uint8_t present;
        uint8_t comm_ok;
        uint16_t class;
        uint16_t alarms;
        uint16_t state;
        uint16_t work_hours;
    } ballast[MODBUS_MAX_DEVICES];

    ballast_sequence_t sequence;
    unsigned long      sequence_ts;
    uint8_t            safety_ok;
} mut_model_t;

typedef const mut_model_t model_t;


void    model_init(mut_model_t *pmodel);
void    model_set_ballast_communication_ok(mut_model_t *pmodel, uint8_t address, uint8_t comm_ok);
void    model_set_ballast_state(mut_model_t *pmodel, uint8_t address, uint16_t state, uint16_t alarms);
void    model_set_ballast_work_hours(mut_model_t *pmodel, uint8_t address, uint16_t work_hours);
uint8_t model_get_working_hours_warning(model_t *pmodel);
uint8_t model_get_working_hours_alarm(model_t *pmodel);
uint8_t model_is_ballast_configured_correctly(model_t *pmodel, size_t ballast);
void    model_set_ballast_class(mut_model_t *pmodel, uint8_t address, uint16_t class);
uint8_t model_are_all_ballast_working(model_t *pmodel);
uint8_t model_is_safety_ok(model_t *pmodel);
uint8_t model_ballast_should_be_on(model_t *pmodel, size_t ballast);
uint8_t model_ballast_present(model_t *pmodel, size_t ballast);


#endif
