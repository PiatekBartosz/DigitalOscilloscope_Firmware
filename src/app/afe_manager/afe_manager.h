#ifndef AFE_MANAGER_H
#define AFE_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

/* Macros */

    /* Type Definitions */

    typedef enum afe_manager_coupling_e
    {
        AFE_MANAGER_COUPLING_AC = 0,
        AFE_MANAGER_COUPLING_DC = 1,

    } afe_manager_coupling_t;

    typedef enum afe_manager_attenuation_e
    {
        AFE_MANAGER_ATTEN_1_TO_1 = 0,
        AFE_MANAGER_ATTEN_1_TO_100 = 1,

    } afe_manager_attenuation_t;

    typedef enum afe_manager_channel_e
    {
        AFE_MANAGER_CH1 = 0,
        AFE_MANAGER_CH2 = 1,

    } afe_manager_channel_t;

/* Function Prototypes */

int afe_manager_init(void);

    int afe_manager_setGain(const afe_manager_channel_t channel, const float percent);
    int afe_manager_setOffset(const afe_manager_channel_t channel, const float percent);
    int afe_manager_setAttenuation(const afe_manager_channel_t channel, const afe_manager_attenuation_t attenuation);
    int afe_manager_setCoupling(const afe_manager_channel_t channel, const afe_manager_coupling_t coupling);
    int afe_manager_setTriggerCoupling(const afe_manager_coupling_t coupling);

#ifdef __cplusplus
}
#endif

#endif /* AFE_MANAGER_H */
