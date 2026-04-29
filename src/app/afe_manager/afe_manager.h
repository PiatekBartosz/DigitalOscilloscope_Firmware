#ifndef AFE_MANAGER_H
#define AFE_MANAGER_H

/* Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

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
    AFE_MANAGER_CH1 = 1,
    AFE_MANAGER_CH2 = 2,

} afe_manager_channel_t;

/* Current AFE settings — readable by the TCP server */
typedef struct afe_manager_channel_state_s
{
    float gain_percent;
    float offset_percent;
    afe_manager_attenuation_t attenuation;
    afe_manager_coupling_t coupling;
} afe_manager_channel_state_t;

typedef struct afe_manager_state_s
{
    afe_manager_channel_state_t ch[2]; /* index 0 = CH1, 1 = CH2 */
    afe_manager_coupling_t trigger_coupling;
    bool interleaved;
} afe_manager_state_t;

/* Function Prototypes */

int afe_manager_init(void);

int afe_manager_setGain(const afe_manager_channel_t channel, const float percent);
int afe_manager_setOffset(const afe_manager_channel_t channel, const float percent);
int afe_manager_setAttenuation(const afe_manager_channel_t channel, const afe_manager_attenuation_t attenuation);
int afe_manager_setCoupling(const afe_manager_channel_t channel, const afe_manager_coupling_t coupling);
int afe_manager_setTriggerCoupling(const afe_manager_coupling_t coupling);
int afe_manager_setInterleaved(const bool isInterleaved);

/* Returns a snapshot of the current AFE configuration. */
afe_manager_state_t afe_manager_getState(void);

#endif /* AFE_MANAGER_H */
