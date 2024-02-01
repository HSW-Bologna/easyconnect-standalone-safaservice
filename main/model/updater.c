#include <assert.h>
#include "updater.h"
#include "model.h"
#include "services/system_time.h"


static const unsigned long SEQUENCE_PERIOD_MS = 1000;


void model_updater_manage(mut_model_t *pmodel) {
    assert(pmodel != NULL);

    if (model_is_safety_ok(pmodel) && !model_get_working_hours_alarm(pmodel)) {
        switch (pmodel->sequence) {
            case BALLAST_SEQUENCE_NONE:
                pmodel->sequence    = BALLAST_SEQUENCE_1;
                pmodel->sequence_ts = get_millis();
                break;

            case BALLAST_SEQUENCE_1:
                if (is_expired(pmodel->sequence_ts, get_millis(), SEQUENCE_PERIOD_MS)) {
                    pmodel->sequence    = BALLAST_SEQUENCE_2;
                    pmodel->sequence_ts = get_millis();
                }
                break;
            case BALLAST_SEQUENCE_2:
                if (is_expired(pmodel->sequence_ts, get_millis(), SEQUENCE_PERIOD_MS)) {
                    pmodel->sequence    = BALLAST_SEQUENCE_3;
                    pmodel->sequence_ts = get_millis();
                }
                break;
            case BALLAST_SEQUENCE_3:
                if (is_expired(pmodel->sequence_ts, get_millis(), SEQUENCE_PERIOD_MS)) {
                    pmodel->sequence    = BALLAST_SEQUENCE_4;
                    pmodel->sequence_ts = get_millis();
                }
                break;
            case BALLAST_SEQUENCE_4:
                if (is_expired(pmodel->sequence_ts, get_millis(), SEQUENCE_PERIOD_MS)) {
                    pmodel->sequence    = BALLAST_SEQUENCE_DONE;
                    pmodel->sequence_ts = get_millis();
                }
                break;
            case BALLAST_SEQUENCE_DONE:
                break;
        }
    } else {
        pmodel->sequence = BALLAST_SEQUENCE_NONE;
    }
}
