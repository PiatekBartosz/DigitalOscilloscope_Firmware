#include "afe_manager/afe_manager.h"

#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(afe_manager);

#define AFE_MANGER_DAC_LIMIT_LOW_DOUBLE  (0.0L)
#define AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE (100.0L)
#define AFE_MANGER_DAC_HIGH_CODE         0xFFFu

/* The DAC61408 driver configures every output for a -5 V to +5 V range.
 * The trigger comparator observes AN_TRIGGER_CHx_P, the positive leg of a
 * differential ADC input centred at 1.5 V. */
#define AFE_MANAGER_DAC_MIN_VOLTS         (-5.0L)
#define AFE_MANAGER_DAC_MAX_VOLTS         (5.0L)
#define AFE_MANAGER_ADC_COMMON_MODE_VOLTS (1.5L)

/* Power-on configuration. Firmware is the single source of truth; clients
 * must read STATUS rather than applying their own startup defaults. */
#define AFE_MANAGER_DEFAULT_GAIN_PERCENT          (50.0f)
#define AFE_MANAGER_DEFAULT_OFFSET_PERCENT        (50.0f) /* 0 V on a -5 V..+5 V DAC */
#define AFE_MANAGER_DEFAULT_TRIGGER_LEVEL_PERCENT (50.0f)

#define AFE_MANGER_DAC_GAIN_CH1                   0u
#define AFE_MANGER_DAC_GAIN_CH2                   1u
#define AFE_MANGER_DAC_OFFSET_CH1                 2u
#define AFE_MANGER_DAC_OFFSET_CH2                 3u
#define AFE_MANGER_DAC_TRIGGER_LEVEL              4u

typedef struct afe_manager_config_s
{
    const struct device *const dacDev;
    const struct gpio_dt_spec attenuationCh1Gpio;
    const struct gpio_dt_spec attenuationCh2Gpio;
    const struct gpio_dt_spec couplingCh1Gpio;
    const struct gpio_dt_spec couplingCh2Gpio;
    const struct gpio_dt_spec triggerSourceGpio;
    const struct gpio_dt_spec interleavedGpio;

} afe_manager_config_t;

static afe_manager_state_t afe_manager_state = {
    .ch[0] = {.gain_percent = AFE_MANAGER_DEFAULT_GAIN_PERCENT,
              .offset_percent = AFE_MANAGER_DEFAULT_OFFSET_PERCENT,
              .attenuation = AFE_MANAGER_ATTEN_1_TO_1,
              .coupling = AFE_MANAGER_COUPLING_DC,
              .adc_range_vpp = AFE_MANAGER_ADC_RANGE_2_VPP},
    .ch[1] = {.gain_percent = AFE_MANAGER_DEFAULT_GAIN_PERCENT,
              .offset_percent = AFE_MANAGER_DEFAULT_OFFSET_PERCENT,
              .attenuation = AFE_MANAGER_ATTEN_1_TO_1,
              .coupling = AFE_MANAGER_COUPLING_DC,
              .adc_range_vpp = AFE_MANAGER_ADC_RANGE_2_VPP},
    .trigger_source = AFE_MANAGER_CH1,
    .trigger_level_percent = AFE_MANAGER_DEFAULT_TRIGGER_LEVEL_PERCENT,
    .interleaved = false,
};

static afe_manager_config_t afe_manager_config = {
    .dacDev = DEVICE_DT_GET(DT_ALIAS(dac0)),

    .attenuationCh1Gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(attenuation_ch1), gpios),
    .attenuationCh2Gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(attenuation_ch2), gpios),

    .couplingCh1Gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(coupling_ch1), gpios),
    .couplingCh2Gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(coupling_ch2), gpios),

    .triggerSourceGpio = GPIO_DT_SPEC_GET(DT_NODELABEL(trigger_source), gpios),

    .interleavedGpio = GPIO_DT_SPEC_GET(DT_NODELABEL(interleaved), gpios),
};

static int afe_manager_initGpio(void)
{
    int errorCode = 0;

    const struct gpio_dt_spec *gpios[] = {
        &afe_manager_config.attenuationCh1Gpio, &afe_manager_config.attenuationCh2Gpio,
        &afe_manager_config.couplingCh1Gpio,    &afe_manager_config.couplingCh2Gpio,
        &afe_manager_config.triggerSourceGpio,  &afe_manager_config.interleavedGpio};
    do
    {
        for (size_t i = 0; i < ARRAY_SIZE(gpios); i++)
        {
            if (!device_is_ready(gpios[i]->port))
            {
                errorCode = -ENODEV;
                LOG_ERR("Gpio device %d is not ready", errorCode);
                break;
            }
        }
        if (errorCode != 0)
        {
            break;
        }

        for (size_t i = 0; i < ARRAY_SIZE(gpios); i++)
        {
            errorCode = gpio_pin_configure_dt(gpios[i], GPIO_OUTPUT_INACTIVE);
            if (errorCode != 0)
            {
                LOG_ERR("Device %d cannot be configured", errorCode);
                break;
            }
        }

    } while (0);

    return errorCode;
}

static int afe_manager_initDac(void)
{
    int errorCode = 0;

    do
    {
        if (!device_is_ready(afe_manager_config.dacDev))
        {
            LOG_ERR("DAC device %s is not ready", afe_manager_config.dacDev->name);
            errorCode = -ENODEV;
            break;
        }

    } while (0);

    return errorCode;
}

static uint32_t afe_manager_encodeDac(const double value)
{
    double clamped = value;
    if (clamped < AFE_MANGER_DAC_LIMIT_LOW_DOUBLE)
        clamped = AFE_MANGER_DAC_LIMIT_LOW_DOUBLE;
    if (clamped > AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE)
        clamped = AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE;

    double scaled = (clamped / (AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE - AFE_MANGER_DAC_LIMIT_LOW_DOUBLE)) *
                    (double)AFE_MANGER_DAC_HIGH_CODE;

    return (uint32_t)(scaled + 0.5L);
}

static double afe_manager_decodeDac(const uint32_t value) __attribute__((unused));
static double afe_manager_decodeDac(const uint32_t value)
{
    double scaled = ((double)value) / (double)AFE_MANGER_DAC_HIGH_CODE;
    return scaled * (AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE - AFE_MANGER_DAC_LIMIT_LOW_DOUBLE);
}

static uint32_t afe_manager_encodeDacVoltage(const double voltage)
{
    double clamped = voltage;
    if (clamped < AFE_MANAGER_DAC_MIN_VOLTS)
        clamped = AFE_MANAGER_DAC_MIN_VOLTS;
    if (clamped > AFE_MANAGER_DAC_MAX_VOLTS)
        clamped = AFE_MANAGER_DAC_MAX_VOLTS;

    double scaled = ((clamped - AFE_MANAGER_DAC_MIN_VOLTS) / (AFE_MANAGER_DAC_MAX_VOLTS - AFE_MANAGER_DAC_MIN_VOLTS)) *
                    (double)AFE_MANGER_DAC_HIGH_CODE;
    return (uint32_t)(scaled + 0.5L);
}

static double afe_manager_triggerLevelVoltage(const afe_manager_channel_t channel, const float percent)
{
    const double differential_vpp = (double)afe_manager_state.ch[channel - 1].adc_range_vpp;

    /* For Vdiff centred at VCM: V+ = VCM + Vdiff / 2.  Consequently a
     * differential full-scale range of Vpp has V+ limits VCM +/- Vpp / 4. */
    const double minimum = AFE_MANAGER_ADC_COMMON_MODE_VOLTS - differential_vpp / 4.0L;
    const double maximum = AFE_MANAGER_ADC_COMMON_MODE_VOLTS + differential_vpp / 4.0L;
    return minimum + ((double)percent / 100.0L) * (maximum - minimum);
}

static int afe_manager_applyTriggerLevel(const afe_manager_channel_t channel, const float percent)
{
    return dac_write_value(afe_manager_config.dacDev, AFE_MANGER_DAC_TRIGGER_LEVEL,
                           afe_manager_encodeDacVoltage(afe_manager_triggerLevelVoltage(channel, percent)));
}

static bool afe_manager_isChannelValid(const afe_manager_channel_t channel)
{
    bool isValid = true;
    if ((channel != AFE_MANAGER_CH1) && (channel != AFE_MANAGER_CH2))
    {
        LOG_ERR("Channel can be either %u or %u", AFE_MANAGER_CH1, AFE_MANAGER_CH2);
        isValid = false;
    }

    return isValid;
}

static bool afe_manager_isPercentValid(const float percent)
{
    return percent >= 0.0f && percent <= 100.0f;
}

int afe_manager_init(void)
{
    int errorCode = 0;

    do
    {
        errorCode = afe_manager_initGpio();
        if (errorCode != 0)
        {
            LOG_ERR("Failed initalizing AFE GPIO error: %d", errorCode);
            break;
        }

        errorCode = afe_manager_initDac();
        if (errorCode != 0)
        {
            LOG_ERR("Failed initalizing AFE DAC error: %d", errorCode);
            break;
        }

        errorCode = afe_manager_setAttenuation(AFE_MANAGER_CH1, afe_manager_state.ch[0].attenuation);
        if (errorCode == 0)
            errorCode = afe_manager_setAttenuation(AFE_MANAGER_CH2, afe_manager_state.ch[1].attenuation);
        if (errorCode == 0)
            errorCode = afe_manager_setCoupling(AFE_MANAGER_CH1, afe_manager_state.ch[0].coupling);
        if (errorCode == 0)
            errorCode = afe_manager_setCoupling(AFE_MANAGER_CH2, afe_manager_state.ch[1].coupling);
        if (errorCode == 0)
            errorCode = afe_manager_setTriggerSource(afe_manager_state.trigger_source);
        if (errorCode == 0)
            errorCode = afe_manager_setInterleaved(afe_manager_state.interleaved);
        if (errorCode != 0)
        {
            LOG_ERR("Failed applying AFE GPIO power-on state: %d", errorCode);
            break;
        }

        /* Sync hardware DAC to the power-on shadow values. */
        const uint8_t dac_map[4] = {
            AFE_MANGER_DAC_GAIN_CH1,
            AFE_MANGER_DAC_OFFSET_CH1,
            AFE_MANGER_DAC_GAIN_CH2,
            AFE_MANGER_DAC_OFFSET_CH2,
        };
        const float init_pct[4] = {
            afe_manager_state.ch[0].gain_percent,
            afe_manager_state.ch[0].offset_percent,
            afe_manager_state.ch[1].gain_percent,
            afe_manager_state.ch[1].offset_percent,
        };
        for (size_t i = 0; i < ARRAY_SIZE(dac_map); i++)
        {
            errorCode = dac_write_value(afe_manager_config.dacDev, dac_map[i], afe_manager_encodeDac(init_pct[i]));
            if (errorCode != 0)
            {
                LOG_ERR("DAC channel %zu init failed: %d", i, errorCode);
                break;
            }
        }
        if (errorCode != 0)
        {
            break;
        }

        errorCode =
            afe_manager_applyTriggerLevel(afe_manager_state.trigger_source, afe_manager_state.trigger_level_percent);
        if (errorCode != 0)
        {
            LOG_ERR("Trigger-level DAC init failed: %d", errorCode);
            break;
        }

        LOG_INF("Succesfully initalized AFE");

    } while (0);

    return errorCode;
}

int afe_manager_setGain(const afe_manager_channel_t channel, const float percent)
{
    int errorCode = 0;
    uint32_t rawValue = 0;
    uint8_t absoluteChannel = 0;

    do
    {
        if (!afe_manager_isChannelValid(channel))
        {
            errorCode = -EINVAL;
            break;
        }

        if (!afe_manager_isPercentValid(percent))
        {
            LOG_ERR("Gain percent %.2f outside [0, 100]", (double)percent);
            errorCode = -EINVAL;
            break;
        }

        rawValue = afe_manager_encodeDac(percent);

        absoluteChannel = (channel == AFE_MANAGER_CH1) ? AFE_MANGER_DAC_GAIN_CH1 : AFE_MANGER_DAC_GAIN_CH2;

        errorCode = dac_write_value(afe_manager_config.dacDev, absoluteChannel, rawValue);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set gain DAC value, error: %d", errorCode);
            break;
        }

        afe_manager_state.ch[channel - 1].gain_percent = percent;

    } while (0);

    return errorCode;
}

int afe_manager_setOffset(const afe_manager_channel_t channel, const float percent)
{
    int errorCode = 0;
    uint32_t rawValue = 0;
    uint8_t absoluteChannel = 0;

    do
    {
        if (!afe_manager_isChannelValid(channel))
        {
            errorCode = -EINVAL;
            break;
        }

        if (!afe_manager_isPercentValid(percent))
        {
            LOG_ERR("Offset percent %.2f outside [0, 100]", (double)percent);
            errorCode = -EINVAL;
            break;
        }

        rawValue = afe_manager_encodeDac(percent);

        absoluteChannel = (channel == AFE_MANAGER_CH1) ? AFE_MANGER_DAC_OFFSET_CH1 : AFE_MANGER_DAC_OFFSET_CH2;

        errorCode = dac_write_value(afe_manager_config.dacDev, absoluteChannel, rawValue);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set offset DAC value, error: %d", errorCode);
            break;
        }

        afe_manager_state.ch[channel - 1].offset_percent = percent;

    } while (0);

    return errorCode;
}

int afe_manager_setAttenuation(const afe_manager_channel_t channel, const afe_manager_attenuation_t attenuation)
{
    int errorCode = 0;
    const struct gpio_dt_spec *gpio = NULL;

    do
    {
        if (!afe_manager_isChannelValid(channel))
        {
            errorCode = -EINVAL;
            break;
        }

        gpio = (channel == AFE_MANAGER_CH1) ? &afe_manager_config.attenuationCh1Gpio
                                            : &afe_manager_config.attenuationCh2Gpio;

        /* The installed AFE uses an active-low attenuation-select control:
         * low selects the 1:100 path and high bypasses it for 1:1. */
        errorCode = gpio_pin_set_dt(gpio, (attenuation == AFE_MANAGER_ATTEN_1_TO_1) ? 1 : 0);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set attenuation, error: %d", errorCode);
            break;
        }

        afe_manager_state.ch[channel - 1].attenuation = attenuation;

    } while (0);

    return errorCode;
}

int afe_manager_setCoupling(const afe_manager_channel_t channel, const afe_manager_coupling_t coupling)
{
    int errorCode = 0;
    const struct gpio_dt_spec *gpio = NULL;

    do
    {
        if (!afe_manager_isChannelValid(channel))
        {
            errorCode = -EINVAL;
            break;
        }

        gpio = (channel == AFE_MANAGER_CH1) ? &afe_manager_config.couplingCh1Gpio : &afe_manager_config.couplingCh2Gpio;

        errorCode = gpio_pin_set_dt(gpio, (coupling == AFE_MANAGER_COUPLING_DC) ? 1 : 0);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set coupling, error: %d", errorCode);
            break;
        }

        afe_manager_state.ch[channel - 1].coupling = coupling;

    } while (0);

    return errorCode;
}

int afe_manager_setAdcRange(const afe_manager_channel_t channel, const afe_manager_adc_range_t range)
{
    if (!afe_manager_isChannelValid(channel) ||
        (range != AFE_MANAGER_ADC_RANGE_1_VPP && range != AFE_MANAGER_ADC_RANGE_2_VPP))
    {
        return -EINVAL;
    }

    const afe_manager_adc_range_t previous = afe_manager_state.ch[channel - 1].adc_range_vpp;
    afe_manager_state.ch[channel - 1].adc_range_vpp = range;

    if (channel == afe_manager_state.trigger_source)
    {
        const int errorCode = afe_manager_applyTriggerLevel(channel, afe_manager_state.trigger_level_percent);
        if (errorCode != 0)
        {
            afe_manager_state.ch[channel - 1].adc_range_vpp = previous;
            LOG_ERR("Failed to apply trigger level after ADC range change: %d", errorCode);
            return errorCode;
        }
    }

    return 0;
}

int afe_manager_setTriggerSource(const afe_manager_channel_t channel)
{
    int errorCode = 0;

    do
    {
        if (!afe_manager_isChannelValid(channel))
        {
            errorCode = -EINVAL;
            break;
        }

        errorCode = afe_manager_applyTriggerLevel(channel, afe_manager_state.trigger_level_percent);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to apply trigger level for selected source, error: %d", errorCode);
            break;
        }

        errorCode = gpio_pin_set_dt(&afe_manager_config.triggerSourceGpio, (channel == AFE_MANAGER_CH2) ? 1 : 0);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set trigger source, error: %d", errorCode);
            break;
        }

        afe_manager_state.trigger_source = channel;

    } while (0);

    return errorCode;
}

int afe_manager_setTriggerLevel(const float percent)
{
    int errorCode = 0;

    do
    {
        if (!afe_manager_isPercentValid(percent))
        {
            LOG_ERR("Trigger percent %.2f outside [0, 100]", (double)percent);
            errorCode = -EINVAL;
            break;
        }

        errorCode = afe_manager_applyTriggerLevel(afe_manager_state.trigger_source, percent);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set trigger level DAC value, error: %d", errorCode);
            break;
        }

        afe_manager_state.trigger_level_percent = percent;

    } while (0);

    return errorCode;
}

float afe_manager_getTriggerLevelVoltage(void)
{
    return (float)afe_manager_triggerLevelVoltage(afe_manager_state.trigger_source,
                                                  afe_manager_state.trigger_level_percent);
}

int afe_manager_setInterleaved(const bool isInterleaved)
{
    int errorCode = 0;

    do
    {
        errorCode = gpio_pin_set_dt(&afe_manager_config.interleavedGpio, isInterleaved ? 1 : 0);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set interleaved gpio, error: %d", errorCode);
            break;
        }

        afe_manager_state.interleaved = isInterleaved;

    } while (0);

    return errorCode;
}

afe_manager_state_t afe_manager_getState(void)
{
    return afe_manager_state;
}
