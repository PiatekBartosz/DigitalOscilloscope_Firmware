#ifndef AFE_MANAGER_H
#define AFE_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

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

/* Differential full-scale range expected by the LTC2299 input. */
typedef enum afe_manager_adc_range_e
{
    AFE_MANAGER_ADC_RANGE_1_VPP = 1,
    AFE_MANAGER_ADC_RANGE_2_VPP = 2,

} afe_manager_adc_range_t;

/* Current AFE settings — readable by the TCP server */
typedef struct afe_manager_channel_state_s
{
    float gain_percent;
    float offset_percent;
    afe_manager_attenuation_t attenuation;
    afe_manager_coupling_t coupling;
    afe_manager_adc_range_t adc_range_vpp;
} afe_manager_channel_state_t;

typedef struct afe_manager_state_s
{
    afe_manager_channel_state_t ch[2]; /* index 0 = CH1, 1 = CH2 */
    afe_manager_channel_t trigger_source;
    float trigger_level_percent;
    bool interleaved;
} afe_manager_state_t;

int afe_manager_init(void);

int afe_manager_setGain(const afe_manager_channel_t channel, const float percent);
int afe_manager_setOffset(const afe_manager_channel_t channel, const float percent);
int afe_manager_setAttenuation(const afe_manager_channel_t channel, const afe_manager_attenuation_t attenuation);
int afe_manager_setCoupling(const afe_manager_channel_t channel, const afe_manager_coupling_t coupling);
int afe_manager_setAdcRange(const afe_manager_channel_t channel, const afe_manager_adc_range_t range);
int afe_manager_setTriggerSource(const afe_manager_channel_t channel);
int afe_manager_setTriggerLevel(const float percent);
int afe_manager_setInterleaved(const bool isInterleaved);

/* Voltage applied to the analog trigger comparator threshold input. */
float afe_manager_getTriggerLevelVoltage(void);

/* Returns a snapshot of the current AFE configuration. */
afe_manager_state_t afe_manager_getState(void);

#endif /* AFE_MANAGER_H */
