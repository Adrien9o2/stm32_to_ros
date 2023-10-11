/* Define to prevent from recursive inclusion --------------------------------*/

#ifndef __STEPPERMOTOR_PROPERTIES_H
#define __STEPPERMOTOR_PROPERTIES_H

namespace StepperMotor
{

    typedef enum {
        BWD = 0, /* Backward. */
        FWD = 1  /* Forward. */
    } direction_t;

    /**
     * @brief Step modes.
     */
    typedef enum {
        STEP_MODE_FULL = 0, /* Full-step. */
        STEP_MODE_HALF,     /* Half-step. */
        STEP_MODE_1_4,      /* 1/4 microstep. */
        STEP_MODE_1_8,      /* 1/8 microstep. */
        STEP_MODE_1_16,     /* 1/16 microstep. */
        STEP_MODE_1_32,     /* 1/32 microstep. */
        STEP_MODE_1_64,     /* 1/64 microstep. */
        STEP_MODE_1_128,    /* 1/128 microstep. */
        STEP_MODE_1_256,    /* 1/256 microstep. */
        STEP_MODE_UNKNOWN,  /* Unknown. */
        STEP_MODE_WAVE      /* Full-step one-phase-on. */
    } step_mode_t;

}
#endif /* __STEPPERMOTOR_CLASS_H */
